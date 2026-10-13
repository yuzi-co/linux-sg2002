// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sophgo SG2002 JPEG encoder
 *
 * The SG2002 has a Chips&Media JPEG unit (JPU) at 0x0b000000, beside the
 * Coda980 H.264 and the WAVE420L H.265 cores. It is an older relative of the
 * CODAJ12: one instance, no firmware, no slice or instance registers and no
 * version register. Everything is programmed through its register file, so
 * this driver does what a firmware would: it loads the Huffman and
 * quantisation tables, writes the JPEG header through the bit writer and
 * starts one picture per mem2mem job. The hardware appends the scan header,
 * the entropy-coded data and the end-of-image marker.
 *
 * Encoder only: NV12, NV21 and packed 4:2:2 (UYVY, YUYV, VYUY, YVYU) input,
 * 4:2:0 baseline JPEG output, quality through
 * V4L2_CID_JPEG_COMPRESSION_QUALITY. A limited-range source (the OUTPUT
 * format's quantization set to V4L2_QUANTIZATION_LIM_RANGE) is expanded to
 * JPEG's full range in the quantiser, so a capture buffer can be encoded
 * as it is, without a scaler pass.
 *
 * Every register access happens with the unit's clocks on and its reset
 * released: on the SG2002 a read of a clock-gated block does not fault, it
 * stalls the bus. The clocks go on when the first file handle opens and off
 * when the last one closes, and the interrupt line is enabled only in between.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../../sophgo/sg2002-video-clk.h"

#define SG_JPEG_NAME			"sg2002-jpeg"

/* Picture control */
#define JPU_PIC_START			0x000
#define  JPU_START_PIC			BIT(0)
#define  JPU_START_INIT			BIT(1)
#define JPU_PIC_STATUS			0x004
#define  JPU_STATUS_DONE		BIT(0)
#define  JPU_STATUS_ERROR		BIT(1)
#define  JPU_STATUS_BBC_FULL		BIT(2)
#define  JPU_STATUS_OVERFLOW		BIT(3)
#define  JPU_STATUS_END			(JPU_STATUS_DONE | JPU_STATUS_ERROR | \
					 JPU_STATUS_BBC_FULL | JPU_STATUS_OVERFLOW)
#define JPU_PIC_ERRMB			0x008
#define JPU_PIC_CTRL			0x010
#define  JPU_PIC_CTRL_ENCODER		BIT(3)
#define  JPU_PIC_CTRL_TC_DIRECTION	BIT(4)
#define JPU_PIC_SIZE			0x014
#define JPU_MCU_INFO			0x018
#define JPU_ROT_INFO			0x01c
#define JPU_SCL_INFO			0x020
#define JPU_CLP_INFO			0x028
#define JPU_OP_INFO			0x02c

/* Source frame */
#define JPU_DPB_CONFIG			0x030
#define  JPU_DPB_CBCR_INTERLEAVE	2
#define  JPU_DPB_CRCB_INTERLEAVE	3
/* Packed 4:2:2: bit 4, and the byte order in bits 3:2. */
#define  JPU_DPB_PACKED_YUYV		(BIT(4) | (0 << 2))
#define  JPU_DPB_PACKED_UYVY		(BIT(4) | (1 << 2))
#define  JPU_DPB_PACKED_YVYU		(BIT(4) | (2 << 2))
#define  JPU_DPB_PACKED_VYUY		(BIT(4) | (3 << 2))
#define  JPU_DPB_FRAME_ENDIAN(x)	((x) << 6)
#define JPU_DPB_BASE_Y			0x034
#define JPU_DPB_BASE_CB			0x038
#define JPU_DPB_BASE_CR			0x03c
#define JPU_DPB_YSTRIDE			0x064
#define JPU_DPB_CSTRIDE			0x068

/* Table memories */
#define JPU_HUFF_CTRL			0x080
#define JPU_HUFF_DATA			0x088
#define  JPU_TABLE_WRITE		0x3
#define JPU_QMAT_CTRL			0x090
#define JPU_QMAT_DATA			0x098
#define JPU_RST_INTVAL			0x0b0
#define JPU_INTR_MASK			0x0c0
#define  JPU_INTR_MASK_ALL		0xff
#define JPU_CYCLE_INFO			0x0c8

/* Bit writer */
#define JPU_GBU_CTRL			0x100
#define  JPU_GBU_CTRL_STUFF_FF		BIT(3)
#define JPU_GBU_BT_PTR			0x110
#define JPU_GBU_WD_PTR			0x114
#define JPU_GBU_PBIT_08			0x120
#define JPU_GBU_PBIT_16			0x124
#define JPU_GBU_PBIT_32			0x12c
#define JPU_GBU_BBSR			0x140
#define JPU_GBU_BBER			0x144
#define JPU_GBU_BBIR			0x148
#define JPU_GBU_BBHR			0x14c

/* Bitstream buffer controller */
#define JPU_BBC_END_ADDR		0x208
#define JPU_BBC_WR_PTR			0x20c
#define JPU_BBC_RD_PTR			0x210
#define JPU_BBC_EXT_ADDR		0x214
#define JPU_BBC_INT_ADDR		0x218
#define JPU_BBC_DATA_CNT		0x21c
#define JPU_BBC_CTRL			0x228
#define  JPU_BBC_CTRL_AUTO_RUN		BIT(0)
#define  JPU_BBC_CTRL_ENDIAN(x)		((x) << 1)
#define JPU_BBC_CUR_POS			0x22c
#define JPU_BBC_BAS_ADDR		0x230
#define JPU_BBC_STRM_CTRL		0x234
#define  JPU_BBC_STRM_END		BIT(31)
#define JPU_BBC_FLUSH_CMD		0x238

/*
 * 4:2:0 with luma and two chroma components: six blocks per MCU, three
 * components, component info 10/5/5 (2x2 luma, 1x1 chroma), and two bus
 * requests per MCU.
 */
#define JPU_MCU_INFO_420		((6 << 16) | (3 << 12) | (10 << 8) | (5 << 4) | 5)
#define JPU_OP_INFO_420			2

/* Quantisation table memory offsets for Y, Cb and Cr. */
static const u32 jpu_qmat_base[3] = { 0x00, 0x40, 0x80 };

#define SG_JPEG_MIN_DIM			16
#define SG_JPEG_MAX_DIM			8192
#define SG_JPEG_HEADER_MAX		1024
#define SG_JPEG_HUFF_ENTRIES		(256 + 256 + 16 + 16)
/* The bytes at the end of a picture searched for EOI, and synced for it. */
#define SG_JPEG_TAIL			32U

static unsigned int timeout_ms = 500;
module_param(timeout_ms, uint, 0644);
MODULE_PARM_DESC(timeout_ms, "Time a picture may take before it is aborted (default: 500)");

static bool use_irq = true;
module_param(use_irq, bool, 0444);
MODULE_PARM_DESC(use_irq, "Finish pictures from the interrupt; 0 polls the status register (default: 1)");

static bool sw_header;
module_param(sw_header, bool, 0644);
MODULE_PARM_DESC(sw_header,
		 "Write the JPEG header with the CPU instead of the bit writer (default: 0)");

static unsigned int stream_endian;
module_param(stream_endian, uint, 0644);
MODULE_PARM_DESC(stream_endian, "Bitstream endianness, 0 to 3 (default: 0)");

static unsigned int frame_endian;
module_param(frame_endian, uint, 0644);
MODULE_PARM_DESC(frame_endian, "Source frame endianness, 0 to 3 (default: 0)");

static unsigned int debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "1 logs every picture (default: 0)");

/*
 * The clocks of the JPU, in the order Sophgo's JPEG driver enables them: the
 * JPEG core first, then the video codec fabric it shares with the H.264 and
 * H.265 cores. The common clock framework counts the shared ones, so turning
 * them off here leaves them on for a running H.264 encode.
 */
static const char * const sg_jpeg_clk_names[] = {
	"jpeg", "apb-jpeg", "vc-src0", "cfg-reg-vc", "axi-video-codec",
};

struct sg_jpeg_fmt {
	u32 fourcc;
	u32 dpb_config;
	bool packed;			/* packed 4:2:2, else 4:2:0 semi-planar */
};

static const struct sg_jpeg_fmt sg_jpeg_out_fmts[] = {
	{ V4L2_PIX_FMT_NV12, JPU_DPB_CBCR_INTERLEAVE, false },
	{ V4L2_PIX_FMT_NV21, JPU_DPB_CRCB_INTERLEAVE, false },
	{ V4L2_PIX_FMT_UYVY, JPU_DPB_PACKED_UYVY, true },
	{ V4L2_PIX_FMT_YUYV, JPU_DPB_PACKED_YUYV, true },
	{ V4L2_PIX_FMT_VYUY, JPU_DPB_PACKED_VYUY, true },
	{ V4L2_PIX_FMT_YVYU, JPU_DPB_PACKED_YVYU, true },
};

struct sg_jpeg_q_data {
	const struct sg_jpeg_fmt *fmt;	/* OUTPUT only */
	u32 width;
	u32 height;
	u32 bytesperline;
	u32 sizeimage;
	u32 sequence;
	u32 quantization;		/* OUTPUT only */
};

struct sg_jpeg_dev {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex mutex;		/* ioctls and queues */

	void __iomem *base;
	struct clk_bulk_data clks[ARRAY_SIZE(sg_jpeg_clk_names)];
	struct reset_control *rstc;
	int irq;

	struct mutex power_mutex;	/* power_users and power changes */
	unsigned int power_users;

	/* hw_lock guards powered, the running job and its start time. */
	spinlock_t hw_lock;
	bool powered;
	struct sg_jpeg_ctx *job_ctx;
	ktime_t job_start;
	struct delayed_work timeout_work;
	struct work_struct poll_work;

	u32 huff[SG_JPEG_HUFF_ENTRIES];
	unsigned long missed_irqs;
};

struct sg_jpeg_ctx {
	struct v4l2_fh fh;
	struct sg_jpeg_dev *jpeg;
	struct v4l2_ctrl_handler hdl;
	struct sg_jpeg_q_data out;
	struct sg_jpeg_q_data cap;
	u8 qtab[2][64];			/* luma, chroma; zig-zag order */
	u8 header[SG_JPEG_HEADER_MAX];
	unsigned int header_len;
};

static inline struct sg_jpeg_ctx *file_to_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct sg_jpeg_ctx, fh);
}

static inline u32 jpu_read(struct sg_jpeg_dev *jpeg, u32 reg)
{
	return readl(jpeg->base + reg);
}

static inline void jpu_write(struct sg_jpeg_dev *jpeg, u32 reg, u32 val)
{
	writel(val, jpeg->base + reg);
}

/*
 * Tables. The quantisation and Huffman tables are those of ITU-T T.81
 * Annex K; the quantisation tables are scaled by quality the way the IJG
 * library does it.
 */

static const u8 zigzag[64] = {
	 0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* Natural (row-major) order. */
static const u8 std_luma_q[64] = {
	16,  11,  10,  16,  24,  40,  51,  61,
	12,  12,  14,  19,  26,  58,  60,  55,
	14,  13,  16,  24,  40,  57,  69,  56,
	14,  17,  22,  29,  51,  87,  80,  62,
	18,  22,  37,  56,  68, 109, 103,  77,
	24,  35,  55,  64,  81, 104, 113,  92,
	49,  64,  78,  87, 103, 121, 120, 101,
	72,  92,  95,  98, 112, 100, 103,  99,
};

static const u8 std_chroma_q[64] = {
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
};

/* Huffman specifications: 16 code length counts, then the symbols. */
static const u8 std_dc_luma[16 + 12] = {
	0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static const u8 std_dc_chroma[16 + 12] = {
	0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static const u8 std_ac_luma[16 + 162] = {
	0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03,
	0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d,
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static const u8 std_ac_chroma[16 + 162] = {
	0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04,
	0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

/*
 * The encoder's Huffman memory holds, per symbol, the code length minus one
 * in bits 19:16 and the code in bits 15:0; unused symbols are 0. It is loaded
 * as the luma AC, chroma AC, luma DC and chroma DC tables, 256 symbols for an
 * AC table and 16 for a DC table. The codes are the canonical ones of T.81
 * Annex C.
 */
static int sg_jpeg_huff_codes(const u8 *spec, unsigned int nsym, u32 *out)
{
	unsigned int len, i, k = 0;
	u32 code = 0;

	for (len = 1; len <= 16; len++) {
		for (i = 0; i < spec[len - 1]; i++, k++) {
			u8 sym = spec[16 + k];

			if (sym >= nsym)
				return -EINVAL;
			out[sym] = ((len - 1) << 16) | code++;
		}
		if (code > (1U << len))
			return -EINVAL;
		code <<= 1;
	}

	return 0;
}

static int sg_jpeg_build_huff(struct sg_jpeg_dev *jpeg)
{
	u32 *h = jpeg->huff;
	int ret;

	memset(jpeg->huff, 0, sizeof(jpeg->huff));
	ret = sg_jpeg_huff_codes(std_ac_luma, 256, h);
	if (!ret)
		ret = sg_jpeg_huff_codes(std_ac_chroma, 256, h + 256);
	if (!ret)
		ret = sg_jpeg_huff_codes(std_dc_luma, 16, h + 512);
	if (!ret)
		ret = sg_jpeg_huff_codes(std_dc_chroma, 16, h + 528);

	return ret;
}

static void sg_jpeg_set_quality(struct sg_jpeg_ctx *ctx, int quality)
{
	unsigned int scale, v, i;

	quality = clamp(quality, 1, 100);
	scale = quality < 50 ? 5000 / quality : 200 - 2 * quality;

	for (i = 0; i < 64; i++) {
		v = (std_luma_q[zigzag[i]] * scale + 50) / 100;
		ctx->qtab[0][i] = clamp(v, 1U, 255U);
		v = (std_chroma_q[zigzag[i]] * scale + 50) / 100;
		ctx->qtab[1][i] = clamp(v, 1U, 255U);
	}
}

/*
 * The header up to and including the frame header: SOI, a JFIF APP0, both
 * quantisation tables, the four Huffman tables and SOF0. The JPU appends SOS,
 * the scan and EOI.
 */
struct sg_jpeg_writer {
	u8 *p;
};

static void put8(struct sg_jpeg_writer *w, u8 v)
{
	*w->p++ = v;
}

static void put16(struct sg_jpeg_writer *w, u16 v)
{
	put_unaligned_be16(v, w->p);
	w->p += 2;
}

static void putn(struct sg_jpeg_writer *w, const u8 *src, size_t n)
{
	memcpy(w->p, src, n);
	w->p += n;
}

static void put_dht(struct sg_jpeg_writer *w, u8 class_id, const u8 *spec, size_t nsym)
{
	put16(w, 0xffc4);
	put16(w, 2 + 1 + 16 + nsym);
	put8(w, class_id);
	putn(w, spec, 16 + nsym);
}

static void sg_jpeg_build_header(struct sg_jpeg_ctx *ctx)
{
	static const u8 jfif[] = {
		0xff, 0xe0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00,
		0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
	};
	struct sg_jpeg_writer w = { ctx->header };
	int i;

	put16(&w, 0xffd8);
	putn(&w, jfif, sizeof(jfif));

	for (i = 0; i < 2; i++) {
		put16(&w, 0xffdb);
		put16(&w, 2 + 1 + 64);
		put8(&w, i);
		putn(&w, ctx->qtab[i], 64);
	}

	put_dht(&w, 0x00, std_dc_luma, 12);
	put_dht(&w, 0x10, std_ac_luma, 162);
	put_dht(&w, 0x01, std_dc_chroma, 12);
	put_dht(&w, 0x11, std_ac_chroma, 162);

	put16(&w, 0xffc0);
	put16(&w, 8 + 3 * 3);
	put8(&w, 8);
	put16(&w, ctx->out.height);
	put16(&w, ctx->out.width);
	put8(&w, 3);
	/* Component id, sampling factors, quantisation table. */
	put8(&w, 1); put8(&w, 0x22); put8(&w, 0);
	put8(&w, 2); put8(&w, 0x11); put8(&w, 1);
	put8(&w, 3); put8(&w, 0x11); put8(&w, 1);

	ctx->header_len = w.p - ctx->header;
}

/* Hardware */

static int sg_jpeg_hw_init(struct sg_jpeg_dev *jpeg)
{
	u32 val;
	int ret;

	jpu_write(jpeg, JPU_PIC_START, JPU_START_INIT);
	ret = readl_poll_timeout_atomic(jpeg->base + JPU_PIC_START, val,
					!(val & JPU_START_INIT), 1, 10000);
	if (ret)
		dev_err(jpeg->dev, "init did not complete, start 0x%x status 0x%x\n",
			val, jpu_read(jpeg, JPU_PIC_STATUS));
	return ret;
}

/* Mask the sources and clear any latched status, so the line goes low. */
static void sg_jpeg_quiesce(struct sg_jpeg_dev *jpeg)
{
	jpu_write(jpeg, JPU_INTR_MASK, JPU_INTR_MASK_ALL);
	jpu_write(jpeg, JPU_PIC_STATUS, jpu_read(jpeg, JPU_PIC_STATUS));
}

static void sg_jpeg_log_state(struct sg_jpeg_dev *jpeg, const char *when)
{
	dev_info(jpeg->dev,
		 "%s: start 0x%x status 0x%x errmb 0x%x cycles %u bbc wr 0x%x rd 0x%x bas 0x%x end 0x%x gbu 0x%x\n",
		 when, jpu_read(jpeg, JPU_PIC_START), jpu_read(jpeg, JPU_PIC_STATUS),
		 jpu_read(jpeg, JPU_PIC_ERRMB), jpu_read(jpeg, JPU_CYCLE_INFO),
		 jpu_read(jpeg, JPU_BBC_WR_PTR), jpu_read(jpeg, JPU_BBC_RD_PTR),
		 jpu_read(jpeg, JPU_BBC_BAS_ADDR), jpu_read(jpeg, JPU_BBC_END_ADDR),
		 jpu_read(jpeg, JPU_GBU_CTRL));
}

static int sg_jpeg_power_on(struct sg_jpeg_dev *jpeg)
{
	int ret;

	/* A video clock profile chosen since the last stream. */
	sg2002_video_clk_apply();
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(jpeg->clks), jpeg->clks);
	if (ret)
		return ret;

	/* The SG2002 reset controller has no pulse: assert, then deassert. */
	ret = reset_control_assert(jpeg->rstc);
	if (!ret) {
		usleep_range(10, 20);
		ret = reset_control_deassert(jpeg->rstc);
	}
	if (ret) {
		clk_bulk_disable_unprepare(ARRAY_SIZE(jpeg->clks), jpeg->clks);
		return ret;
	}
	usleep_range(10, 20);

	/* First register access: clocks on, reset released. */
	sg_jpeg_quiesce(jpeg);
	ret = sg_jpeg_hw_init(jpeg);
	if (ret) {
		reset_control_assert(jpeg->rstc);
		clk_bulk_disable_unprepare(ARRAY_SIZE(jpeg->clks), jpeg->clks);
		return ret;
	}

	spin_lock_irq(&jpeg->hw_lock);
	jpeg->powered = true;
	spin_unlock_irq(&jpeg->hw_lock);
	if (jpeg->irq > 0)
		enable_irq(jpeg->irq);

	return 0;
}

static void sg_jpeg_power_off(struct sg_jpeg_dev *jpeg)
{
	if (jpeg->irq > 0)
		disable_irq(jpeg->irq);

	spin_lock_irq(&jpeg->hw_lock);
	sg_jpeg_quiesce(jpeg);
	jpeg->powered = false;
	spin_unlock_irq(&jpeg->hw_lock);

	reset_control_assert(jpeg->rstc);
	clk_bulk_disable_unprepare(ARRAY_SIZE(jpeg->clks), jpeg->clks);
}

static int sg_jpeg_power_get(struct sg_jpeg_dev *jpeg)
{
	int ret = 0;

	mutex_lock(&jpeg->power_mutex);
	if (!jpeg->power_users)
		ret = sg_jpeg_power_on(jpeg);
	if (!ret)
		jpeg->power_users++;
	mutex_unlock(&jpeg->power_mutex);

	return ret;
}

static void sg_jpeg_power_put(struct sg_jpeg_dev *jpeg)
{
	mutex_lock(&jpeg->power_mutex);
	if (!--jpeg->power_users)
		sg_jpeg_power_off(jpeg);
	mutex_unlock(&jpeg->power_mutex);
}

static void jpu_write_bytes(struct sg_jpeg_dev *jpeg, const u8 *p, size_t n)
{
	for (; n >= 4; p += 4, n -= 4)
		jpu_write(jpeg, JPU_GBU_PBIT_32, get_unaligned_be32(p));
	if (n >= 2) {
		jpu_write(jpeg, JPU_GBU_PBIT_16, get_unaligned_be16(p));
		p += 2;
		n -= 2;
	}
	if (n)
		jpu_write(jpeg, JPU_GBU_PBIT_08, *p);
}

static void sg_jpeg_load_tables(struct sg_jpeg_dev *jpeg, struct sg_jpeg_ctx *ctx)
{
	unsigned int comp, i;

	jpu_write(jpeg, JPU_HUFF_CTRL, JPU_TABLE_WRITE);
	for (i = 0; i < SG_JPEG_HUFF_ENTRIES; i++)
		jpu_write(jpeg, JPU_HUFF_DATA, jpeg->huff[i]);
	jpu_write(jpeg, JPU_HUFF_CTRL, 0);

	/*
	 * Each entry carries the divisor in bits 31:20 and its reciprocal,
	 * 2^19 / q, in bits 19:0.
	 *
	 * A limited-range source is expanded here. The unit level-shifts
	 * every sample by 128 before the DCT, and the decoder adds 128 back,
	 * so scaling every coefficient by k scales each sample about 128:
	 * out = 128 + (in - 128) * k. Dividing by q / k instead of q, while
	 * the header keeps q, does that at no cost. Chroma, centred on 128,
	 * comes out exact with k = 255 / 224. Luma with k = 255 / 219 comes
	 * out 2.4 levels below (in - 16) * k, the one part a scale cannot
	 * give: limited black 16 lands on 0 (clamped from -2.4), limited
	 * white 235 on 253.
	 */
	for (comp = 0; comp < 3; comp++) {
		const u8 *q = ctx->qtab[comp ? 1 : 0];
		u32 num = 1, den = 1;

		if (ctx->out.quantization == V4L2_QUANTIZATION_LIM_RANGE) {
			num = 255;
			den = comp ? 224 : 219;
		}

		jpu_write(jpeg, JPU_QMAT_CTRL, jpu_qmat_base[comp] | JPU_TABLE_WRITE);
		for (i = 0; i < 64; i++) {
			u32 div = max_t(u32, DIV_ROUND_CLOSEST(q[i] * den, num), 1);
			u32 rcp = 0x80000 * num / (q[i] * den);

			jpu_write(jpeg, JPU_QMAT_DATA,
				  (div << 20) | min_t(u32, rcp, 0xfffff));
		}
		jpu_write(jpeg, JPU_QMAT_CTRL, jpu_qmat_base[comp]);
	}
}

static void sg_jpeg_job_done(struct sg_jpeg_dev *jpeg, struct sg_jpeg_ctx *ctx,
			     u32 status, bool timed_out)
{
	struct vb2_v4l2_buffer *src, *dst;
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	dma_addr_t start;
	u32 wr, errmb, cycles, size, cap;
	u8 *vaddr;
	s64 us;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	us = ktime_us_delta(ktime_get(), jpeg->job_start);
	start = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	cap = vb2_plane_size(&dst->vb2_buf, 0);
	wr = jpu_read(jpeg, JPU_BBC_WR_PTR);
	errmb = jpu_read(jpeg, JPU_PIC_ERRMB);
	cycles = jpu_read(jpeg, JPU_CYCLE_INFO);
	jpu_write(jpeg, JPU_BBC_FLUSH_CMD, 0);

	if (timed_out || !(status & JPU_STATUS_DONE) ||
	    (status & (JPU_STATUS_ERROR | JPU_STATUS_BBC_FULL | JPU_STATUS_OVERFLOW))) {
		dev_err(jpeg->dev,
			"picture %u failed: status 0x%x%s, errmb 0x%x, wr 0x%x of 0x%llx+%u, %lld us\n",
			ctx->out.sequence, status, timed_out ? " (timeout)" : "", errmb, wr,
			(u64)start, cap, us);
		state = VB2_BUF_STATE_ERROR;
		size = 0;
	} else {
		size = wr - lower_32_bits(start);
		if (size > cap) {
			dev_err(jpeg->dev, "write pointer 0x%x outside the buffer\n", wr);
			state = VB2_BUF_STATE_ERROR;
			size = 0;
		}
	}

	/*
	 * The unit ends the picture with EOI. Trim anything after it, and
	 * add it should a picture end without one.
	 */
	vaddr = vb2_plane_vaddr(&dst->vb2_buf, 0);
	if (state == VB2_BUF_STATE_DONE && vaddr && size >= 2) {
		bool cached = dst->vb2_buf.vb2_queue->non_coherent_mem;
		u32 i, end = 0, tail = min(size, SG_JPEG_TAIL);

		/*
		 * A cached buffer (V4L2_MEMORY_FLAG_NON_COHERENT) is synced
		 * for the CPU only when the buffer is done, after this: drop
		 * any stale lines of the tail before reading it.
		 */
		if (cached)
			dma_sync_single_for_cpu(jpeg->dev, start + size - tail, tail,
						DMA_FROM_DEVICE);
		for (i = size; i >= 2 && i + 16 > size; i--) {
			if (vaddr[i - 2] == 0xff && vaddr[i - 1] == 0xd9) {
				end = i;
				break;
			}
		}
		if (end) {
			size = end;
		} else if (size + 2 <= cap) {
			vaddr[size++] = 0xff;
			vaddr[size++] = 0xd9;
			/* Write it back, or the sync for the CPU discards it. */
			if (cached)
				dma_sync_single_for_device(jpeg->dev, start + size - 2, 2,
							   DMA_TO_DEVICE);
			dev_warn_once(jpeg->dev, "picture without EOI, appended\n");
		}
	}

	if (debug)
		dev_info(jpeg->dev, "picture %u: status 0x%x, %u bytes, %u cycles, %lld us\n",
			 ctx->out.sequence, status, size, cycles, us);

	vb2_set_plane_payload(&dst->vb2_buf, 0, size);
	dst->sequence = ctx->cap.sequence++;
	src->sequence = ctx->out.sequence++;
	v4l2_m2m_buf_copy_metadata(src, dst);
	dst->flags |= V4L2_BUF_FLAG_KEYFRAME;

	if (state != VB2_BUF_STATE_DONE) {
		/* Leave the unit idle and clean for the next picture. */
		sg_jpeg_quiesce(jpeg);
		sg_jpeg_hw_init(jpeg);
	}

	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_buf_done(dst, state);
	v4l2_m2m_job_finish(jpeg->m2m_dev, ctx->fh.m2m_ctx);
}

/* Take the running job, if there is one. Called with hw_lock held. */
static struct sg_jpeg_ctx *sg_jpeg_claim(struct sg_jpeg_dev *jpeg)
{
	struct sg_jpeg_ctx *ctx = jpeg->job_ctx;

	jpeg->job_ctx = NULL;
	return ctx;
}

static void sg_jpeg_device_run(void *priv)
{
	struct sg_jpeg_ctx *ctx = priv;
	struct sg_jpeg_dev *jpeg = ctx->jpeg;
	struct sg_jpeg_q_data *q = &ctx->out;
	struct vb2_v4l2_buffer *src, *dst;
	dma_addr_t src_addr, dst_addr, bas;
	u32 dst_size, aligned_w, aligned_h;
	unsigned long flags;
	unsigned int hlen = 0;
	bool use_sw;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	src_addr = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst->vb2_buf, 0);

	aligned_w = ALIGN(q->width, 16);
	aligned_h = ALIGN(q->height, 16);

	if (sg_jpeg_hw_init(jpeg)) {
		spin_lock_irqsave(&jpeg->hw_lock, flags);
		jpeg->job_ctx = NULL;
		jpeg->job_start = ktime_get();
		spin_unlock_irqrestore(&jpeg->hw_lock, flags);
		sg_jpeg_job_done(jpeg, ctx, 0, false);
		return;
	}

	bas = dst_addr;
	use_sw = sw_header && vb2_plane_vaddr(&dst->vb2_buf, 0);
	if (use_sw) {
		u8 *vaddr = vb2_plane_vaddr(&dst->vb2_buf, 0);

		/* The bitstream continues at an 8-byte boundary; pad with 0xff fill. */
		hlen = ALIGN(ctx->header_len, 8);
		memcpy(vaddr, ctx->header, ctx->header_len);
		memset(vaddr + ctx->header_len, 0xff, hlen - ctx->header_len);
		if (dst->vb2_buf.vb2_queue->non_coherent_mem)
			dma_sync_single_for_device(jpeg->dev, dst_addr, hlen, DMA_TO_DEVICE);
		bas = dst_addr + hlen;
	}

	jpu_write(jpeg, JPU_CLP_INFO, 0);
	jpu_write(jpeg, JPU_BBC_BAS_ADDR, bas);
	jpu_write(jpeg, JPU_BBC_END_ADDR, dst_addr + dst_size);
	jpu_write(jpeg, JPU_BBC_WR_PTR, bas);
	jpu_write(jpeg, JPU_BBC_RD_PTR, bas);
	jpu_write(jpeg, JPU_BBC_CUR_POS, 0);
	/* 64 words of 32 bits per 256-byte page */
	jpu_write(jpeg, JPU_BBC_DATA_CNT, 64);
	jpu_write(jpeg, JPU_BBC_EXT_ADDR, use_sw ? dst_addr : bas);
	jpu_write(jpeg, JPU_BBC_INT_ADDR, 0);
	if (use_sw)
		jpu_write(jpeg, JPU_BBC_STRM_CTRL,
			  JPU_BBC_STRM_END | ((dst_size - hlen) / 256));

	jpu_write(jpeg, JPU_GBU_BT_PTR, 0);
	jpu_write(jpeg, JPU_GBU_WD_PTR, 0);
	jpu_write(jpeg, JPU_GBU_BBSR, 0);
	jpu_write(jpeg, JPU_GBU_CTRL, 0);
	jpu_write(jpeg, JPU_GBU_BBER, 64 * 2 - 1);
	jpu_write(jpeg, JPU_GBU_BBIR, 64);
	jpu_write(jpeg, JPU_GBU_BBHR, 64);

	jpu_write(jpeg, JPU_PIC_CTRL, JPU_PIC_CTRL_TC_DIRECTION | JPU_PIC_CTRL_ENCODER);
	jpu_write(jpeg, JPU_SCL_INFO, 0);
	jpu_write(jpeg, JPU_DPB_CONFIG,
		  JPU_DPB_FRAME_ENDIAN(frame_endian & 3) | q->fmt->dpb_config);
	jpu_write(jpeg, JPU_RST_INTVAL, 0);
	jpu_write(jpeg, JPU_BBC_CTRL,
		  JPU_BBC_CTRL_ENDIAN(stream_endian & 3) | JPU_BBC_CTRL_AUTO_RUN);
	jpu_write(jpeg, JPU_OP_INFO, JPU_OP_INFO_420);

	sg_jpeg_load_tables(jpeg, ctx);
	if (!use_sw)
		jpu_write_bytes(jpeg, ctx->header, ctx->header_len);

	jpu_write(jpeg, JPU_PIC_SIZE, aligned_w << 16 | aligned_h);
	jpu_write(jpeg, JPU_ROT_INFO, 0);
	jpu_write(jpeg, JPU_MCU_INFO, JPU_MCU_INFO_420);
	jpu_write(jpeg, JPU_GBU_CTRL, 0);

	if (q->fmt->packed) {
		/*
		 * One plane; the stride is in bytes, two per pixel. The unit
		 * halves the chroma vertically itself and codes 4:2:0.
		 */
		jpu_write(jpeg, JPU_DPB_BASE_Y, src_addr);
		jpu_write(jpeg, JPU_DPB_BASE_CB, src_addr);
		jpu_write(jpeg, JPU_DPB_BASE_CR, src_addr);
		jpu_write(jpeg, JPU_DPB_YSTRIDE, q->bytesperline);
		jpu_write(jpeg, JPU_DPB_CSTRIDE, q->bytesperline / 2);
	} else {
		jpu_write(jpeg, JPU_DPB_BASE_Y, src_addr);
		jpu_write(jpeg, JPU_DPB_BASE_CB, src_addr + q->bytesperline * q->height);
		jpu_write(jpeg, JPU_DPB_BASE_CR, src_addr + q->bytesperline * q->height);
		jpu_write(jpeg, JPU_DPB_YSTRIDE, q->bytesperline);
		/* Interleaved chroma: half the width, two bytes per sample pair. */
		jpu_write(jpeg, JPU_DPB_CSTRIDE, q->bytesperline);
	}

	spin_lock_irqsave(&jpeg->hw_lock, flags);
	jpeg->job_ctx = ctx;
	jpeg->job_start = ktime_get();
	jpu_write(jpeg, JPU_PIC_STATUS, jpu_read(jpeg, JPU_PIC_STATUS));
	jpu_write(jpeg, JPU_INTR_MASK, 0);
	jpu_write(jpeg, JPU_PIC_START, JPU_START_PIC);
	spin_unlock_irqrestore(&jpeg->hw_lock, flags);

	if (jpeg->irq > 0)
		mod_delayed_work(system_wq, &jpeg->timeout_work,
				 msecs_to_jiffies(timeout_ms));
	else
		queue_work(system_highpri_wq, &jpeg->poll_work);
}

static irqreturn_t sg_jpeg_irq(int irq, void *data)
{
	struct sg_jpeg_dev *jpeg = data;
	irqreturn_t ret = IRQ_NONE;

	spin_lock(&jpeg->hw_lock);
	if (jpeg->powered && jpu_read(jpeg, JPU_PIC_STATUS)) {
		/* Keep the line low until the thread has read the status. */
		jpu_write(jpeg, JPU_INTR_MASK, JPU_INTR_MASK_ALL);
		ret = IRQ_WAKE_THREAD;
	}
	spin_unlock(&jpeg->hw_lock);

	return ret;
}

static irqreturn_t sg_jpeg_irq_thread(int irq, void *data)
{
	struct sg_jpeg_dev *jpeg = data;
	struct sg_jpeg_ctx *ctx;
	u32 status;

	spin_lock_irq(&jpeg->hw_lock);
	status = jpu_read(jpeg, JPU_PIC_STATUS);
	jpu_write(jpeg, JPU_PIC_STATUS, status);
	ctx = (status & JPU_STATUS_END) ? sg_jpeg_claim(jpeg) : NULL;
	spin_unlock_irq(&jpeg->hw_lock);

	if (!ctx) {
		dev_dbg(jpeg->dev, "interrupt without a job, status 0x%x\n", status);
		return IRQ_HANDLED;
	}

	cancel_delayed_work(&jpeg->timeout_work);
	sg_jpeg_job_done(jpeg, ctx, status, false);

	return IRQ_HANDLED;
}

static void sg_jpeg_timeout_work(struct work_struct *work)
{
	struct sg_jpeg_dev *jpeg = container_of(to_delayed_work(work),
						struct sg_jpeg_dev, timeout_work);
	struct sg_jpeg_ctx *ctx = NULL;
	u32 status = 0;
	s64 left;

	spin_lock_irq(&jpeg->hw_lock);
	if (jpeg->job_ctx) {
		left = timeout_ms * 1000LL - ktime_us_delta(ktime_get(), jpeg->job_start);
		if (left > 0) {
			/* A newer job than the one this work was armed for. */
			mod_delayed_work(system_wq, &jpeg->timeout_work,
					 usecs_to_jiffies(left) + 1);
		} else {
			status = jpu_read(jpeg, JPU_PIC_STATUS);
			jpu_write(jpeg, JPU_PIC_STATUS, status);
			ctx = sg_jpeg_claim(jpeg);
		}
	}
	spin_unlock_irq(&jpeg->hw_lock);

	if (!ctx)
		return;

	if (status & JPU_STATUS_DONE) {
		jpeg->missed_irqs++;
		dev_warn_ratelimited(jpeg->dev,
				     "picture done (status 0x%x) without an interrupt, %lu so far\n",
				     status, jpeg->missed_irqs);
		sg_jpeg_job_done(jpeg, ctx, status, false);
		return;
	}

	sg_jpeg_log_state(jpeg, "timeout");
	sg_jpeg_job_done(jpeg, ctx, status, true);
}

static void sg_jpeg_poll_work(struct work_struct *work)
{
	struct sg_jpeg_dev *jpeg = container_of(work, struct sg_jpeg_dev, poll_work);
	struct sg_jpeg_ctx *ctx;
	u32 status;
	int ret;

	ret = readl_poll_timeout(jpeg->base + JPU_PIC_STATUS, status,
				 status & JPU_STATUS_END, 50, timeout_ms * 1000);

	spin_lock_irq(&jpeg->hw_lock);
	jpu_write(jpeg, JPU_PIC_STATUS, status);
	ctx = sg_jpeg_claim(jpeg);
	spin_unlock_irq(&jpeg->hw_lock);

	if (!ctx)
		return;
	if (ret)
		sg_jpeg_log_state(jpeg, "timeout");
	sg_jpeg_job_done(jpeg, ctx, status, ret);
}

static const struct v4l2_m2m_ops sg_jpeg_m2m_ops = {
	.device_run = sg_jpeg_device_run,
};

/* Formats */

static const struct sg_jpeg_fmt *sg_jpeg_find_fmt(u32 fourcc);

static void sg_jpeg_fill_out(struct v4l2_pix_format *pix)
{
	const struct sg_jpeg_fmt *fmt = sg_jpeg_find_fmt(pix->pixelformat);

	pix->width = clamp_t(u32, ALIGN(pix->width, 16), SG_JPEG_MIN_DIM, SG_JPEG_MAX_DIM);
	pix->height = clamp_t(u32, ALIGN(pix->height, 2), SG_JPEG_MIN_DIM, SG_JPEG_MAX_DIM);
	pix->field = V4L2_FIELD_NONE;
	if (pix->quantization != V4L2_QUANTIZATION_LIM_RANGE)
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	if (fmt && fmt->packed) {
		/*
		 * The unit reads whole 16x16 MCUs, rows up to the next multiple
		 * of 16: eight past a 1080-line frame.
		 */
		pix->bytesperline = clamp_t(u32, ALIGN(pix->bytesperline, 16),
					    pix->width * 2, SG_JPEG_MAX_DIM * 2);
		pix->sizeimage = max_t(u32, pix->sizeimage,
				       pix->bytesperline * ALIGN(pix->height, 16));
		if (!pix->colorspace)
			pix->colorspace = V4L2_COLORSPACE_JPEG;
		return;
	}
	pix->bytesperline = clamp_t(u32, ALIGN(pix->bytesperline, 16), pix->width,
				    SG_JPEG_MAX_DIM);
	/*
	 * The unit reads whole MCUs: luma rows up to the next multiple of 16
	 * and chroma rows up to the next multiple of 8, which for 1080 lines
	 * runs four chroma rows past a tight buffer.
	 */
	pix->sizeimage = max_t(u32, pix->sizeimage,
			       pix->bytesperline * pix->height +
			       pix->bytesperline * ALIGN(pix->height, 16) / 2);
	if (!pix->colorspace)
		pix->colorspace = V4L2_COLORSPACE_JPEG;
}

static u32 sg_jpeg_cap_size(u32 width, u32 height)
{
	/* One byte per pixel is far above any sane quality setting. */
	return PAGE_ALIGN(width * height + SG_JPEG_HEADER_MAX + SZ_64K);
}

static const struct sg_jpeg_fmt *sg_jpeg_find_fmt(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sg_jpeg_out_fmts); i++)
		if (sg_jpeg_out_fmts[i].fourcc == fourcc)
			return &sg_jpeg_out_fmts[i];
	return NULL;
}

static int sg_jpeg_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, SG_JPEG_NAME, sizeof(cap->driver));
	strscpy(cap->card, "SG2002 JPEG encoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" SG_JPEG_NAME, sizeof(cap->bus_info));
	return 0;
}

static int sg_jpeg_enum_fmt_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(sg_jpeg_out_fmts))
		return -EINVAL;
	f->pixelformat = sg_jpeg_out_fmts[f->index].fourcc;
	return 0;
}

static int sg_jpeg_enum_fmt_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_JPEG;
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int sg_jpeg_enum_framesizes(struct file *file, void *priv,
				   struct v4l2_frmsizeenum *fs)
{
	if (fs->index || (fs->pixel_format != V4L2_PIX_FMT_JPEG &&
			  !sg_jpeg_find_fmt(fs->pixel_format)))
		return -EINVAL;
	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise.min_width = SG_JPEG_MIN_DIM;
	fs->stepwise.max_width = SG_JPEG_MAX_DIM;
	fs->stepwise.step_width = 16;
	fs->stepwise.min_height = SG_JPEG_MIN_DIM;
	fs->stepwise.max_height = SG_JPEG_MAX_DIM;
	fs->stepwise.step_height = 2;
	return 0;
}

static int sg_jpeg_g_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sg_jpeg_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	memset(pix, 0, sizeof(*pix));
	pix->pixelformat = ctx->out.fmt->fourcc;
	pix->width = ctx->out.width;
	pix->height = ctx->out.height;
	pix->bytesperline = ctx->out.bytesperline;
	pix->sizeimage = ctx->out.sizeimage;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->quantization = ctx->out.quantization;
	return 0;
}

static int sg_jpeg_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sg_jpeg_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	memset(pix, 0, sizeof(*pix));
	pix->pixelformat = V4L2_PIX_FMT_JPEG;
	pix->width = ctx->cap.width;
	pix->height = ctx->cap.height;
	pix->sizeimage = ctx->cap.sizeimage;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	return 0;
}

static int sg_jpeg_try_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (!sg_jpeg_find_fmt(pix->pixelformat))
		pix->pixelformat = sg_jpeg_out_fmts[0].fourcc;
	sg_jpeg_fill_out(pix);
	return 0;
}

static int sg_jpeg_try_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sg_jpeg_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	/* The picture size follows the OUTPUT queue. */
	pix->pixelformat = V4L2_PIX_FMT_JPEG;
	pix->width = ctx->out.width;
	pix->height = ctx->out.height;
	pix->bytesperline = 0;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->sizeimage = max_t(u32, PAGE_ALIGN(pix->sizeimage),
			       sg_jpeg_cap_size(pix->width, pix->height));
	return 0;
}

static int sg_jpeg_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sg_jpeg_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (vb2_is_busy(vq))
		return -EBUSY;

	sg_jpeg_try_fmt_out(file, priv, f);
	ctx->out.fmt = sg_jpeg_find_fmt(pix->pixelformat);
	ctx->out.width = pix->width;
	ctx->out.height = pix->height;
	ctx->out.bytesperline = pix->bytesperline;
	ctx->out.sizeimage = pix->sizeimage;
	ctx->out.quantization = pix->quantization;

	ctx->cap.width = pix->width;
	ctx->cap.height = pix->height;
	ctx->cap.sizeimage = max(ctx->cap.sizeimage,
				 sg_jpeg_cap_size(pix->width, pix->height));
	sg_jpeg_build_header(ctx);
	return 0;
}

static int sg_jpeg_s_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sg_jpeg_ctx *ctx = file_to_ctx(file);
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	sg_jpeg_try_fmt_cap(file, priv, f);
	ctx->cap.sizeimage = f->fmt.pix.sizeimage;
	return 0;
}

static const struct v4l2_ioctl_ops sg_jpeg_ioctl_ops = {
	.vidioc_querycap		= sg_jpeg_querycap,
	.vidioc_enum_fmt_vid_cap	= sg_jpeg_enum_fmt_cap,
	.vidioc_enum_fmt_vid_out	= sg_jpeg_enum_fmt_out,
	.vidioc_enum_framesizes		= sg_jpeg_enum_framesizes,
	.vidioc_g_fmt_vid_cap		= sg_jpeg_g_fmt_cap,
	.vidioc_g_fmt_vid_out		= sg_jpeg_g_fmt_out,
	.vidioc_try_fmt_vid_cap		= sg_jpeg_try_fmt_cap,
	.vidioc_try_fmt_vid_out		= sg_jpeg_try_fmt_out,
	.vidioc_s_fmt_vid_cap		= sg_jpeg_s_fmt_cap,
	.vidioc_s_fmt_vid_out		= sg_jpeg_s_fmt_out,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* Queues */

static int sg_jpeg_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			       unsigned int *nplanes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct sg_jpeg_ctx *ctx = vb2_get_drv_priv(vq);
	u32 size = V4L2_TYPE_IS_OUTPUT(vq->type) ? ctx->out.sizeimage : ctx->cap.sizeimage;

	if (*nplanes)
		return (*nplanes != 1 || sizes[0] < size) ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int sg_jpeg_buf_prepare(struct vb2_buffer *vb)
{
	struct sg_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	bool out = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type);
	u32 size = out ? ctx->out.sizeimage : ctx->cap.sizeimage;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	if (out) {
		vbuf->field = V4L2_FIELD_NONE;
		if (!vb2_get_plane_payload(vb, 0))
			vb2_set_plane_payload(vb, 0, size);
	} else {
		vb2_set_plane_payload(vb, 0, 0);
	}
	return 0;
}

static void sg_jpeg_buf_queue(struct vb2_buffer *vb)
{
	struct sg_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int sg_jpeg_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct sg_jpeg_ctx *ctx = vb2_get_drv_priv(vq);

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ctx->out.sequence = 0;
	else
		ctx->cap.sequence = 0;
	return 0;
}

static void sg_jpeg_stop_streaming(struct vb2_queue *vq)
{
	struct sg_jpeg_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			return;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops sg_jpeg_qops = {
	.queue_setup		= sg_jpeg_queue_setup,
	.buf_prepare		= sg_jpeg_buf_prepare,
	.buf_queue		= sg_jpeg_buf_queue,
	.start_streaming	= sg_jpeg_start_streaming,
	.stop_streaming		= sg_jpeg_stop_streaming,
};

static int sg_jpeg_queue_init(void *priv, struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	struct sg_jpeg_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &sg_jpeg_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->jpeg->mutex;
	src_vq->dev = ctx->jpeg->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	/*
	 * The JPEG is read by the CPU, which on the SG2002 sees a coherent
	 * buffer uncached: about 50 MB/s, 6 ms for a 1080p picture. Let user
	 * space ask for a cached one (V4L2_MEMORY_FLAG_NON_COHERENT); vb2
	 * syncs it when the buffer is queued and done.
	 */
	dst_vq->allow_cache_hints = 1;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &sg_jpeg_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->jpeg->mutex;
	dst_vq->dev = ctx->jpeg->dev;
	return vb2_queue_init(dst_vq);
}

static int sg_jpeg_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sg_jpeg_ctx *ctx = container_of(ctrl->handler, struct sg_jpeg_ctx, hdl);

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		sg_jpeg_set_quality(ctx, ctrl->val);
		sg_jpeg_build_header(ctx);
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sg_jpeg_ctrl_ops = {
	.s_ctrl = sg_jpeg_s_ctrl,
};

/* File operations */

static int sg_jpeg_open(struct file *file)
{
	struct sg_jpeg_dev *jpeg = video_drvdata(file);
	struct v4l2_pix_format pix = {
		.pixelformat = V4L2_PIX_FMT_NV12,
		.width = 1920,
		.height = 1080,
	};
	struct sg_jpeg_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->jpeg = jpeg;

	ret = sg_jpeg_power_get(jpeg);
	if (ret) {
		dev_err(jpeg->dev, "cannot power the unit: %d\n", ret);
		goto err_free;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));

	sg_jpeg_fill_out(&pix);
	ctx->out.fmt = &sg_jpeg_out_fmts[0];
	ctx->out.width = pix.width;
	ctx->out.height = pix.height;
	ctx->out.bytesperline = pix.bytesperline;
	ctx->out.sizeimage = pix.sizeimage;
	ctx->out.quantization = pix.quantization;
	ctx->cap.width = pix.width;
	ctx->cap.height = pix.height;
	ctx->cap.sizeimage = sg_jpeg_cap_size(pix.width, pix.height);

	v4l2_ctrl_handler_init(&ctx->hdl, 1);
	v4l2_ctrl_new_std(&ctx->hdl, &sg_jpeg_ctrl_ops,
			  V4L2_CID_JPEG_COMPRESSION_QUALITY, 1, 100, 1, 85);
	if (ctx->hdl.error) {
		ret = ctx->hdl.error;
		goto err_ctrl;
	}
	ctx->fh.ctrl_handler = &ctx->hdl;
	v4l2_ctrl_handler_setup(&ctx->hdl);

	if (mutex_lock_interruptible(&jpeg->mutex)) {
		ret = -ERESTARTSYS;
		goto err_ctrl;
	}
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jpeg->m2m_dev, ctx, sg_jpeg_queue_init);
	mutex_unlock(&jpeg->mutex);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctrl;
	}

	v4l2_fh_add(&ctx->fh, file);
	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_exit(&ctx->fh);
	sg_jpeg_power_put(jpeg);
err_free:
	kfree(ctx);
	return ret;
}

static int sg_jpeg_release(struct file *file)
{
	struct sg_jpeg_ctx *ctx = file_to_ctx(file);
	struct sg_jpeg_dev *jpeg = ctx->jpeg;

	mutex_lock(&jpeg->mutex);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&jpeg->mutex);
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	sg_jpeg_power_put(jpeg);
	return 0;
}

static const struct v4l2_file_operations sg_jpeg_fops = {
	.owner		= THIS_MODULE,
	.open		= sg_jpeg_open,
	.release	= sg_jpeg_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* Platform device */

static int sg_jpeg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sg_jpeg_dev *jpeg;
	struct video_device *vfd;
	unsigned int i;
	int ret;

	jpeg = devm_kzalloc(dev, sizeof(*jpeg), GFP_KERNEL);
	if (!jpeg)
		return -ENOMEM;
	jpeg->dev = dev;
	mutex_init(&jpeg->mutex);
	mutex_init(&jpeg->power_mutex);
	spin_lock_init(&jpeg->hw_lock);
	INIT_DELAYED_WORK(&jpeg->timeout_work, sg_jpeg_timeout_work);
	INIT_WORK(&jpeg->poll_work, sg_jpeg_poll_work);

	ret = sg_jpeg_build_huff(jpeg);
	if (ret)
		return ret;

	jpeg->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(jpeg->base))
		return PTR_ERR(jpeg->base);

	for (i = 0; i < ARRAY_SIZE(jpeg->clks); i++)
		jpeg->clks[i].id = sg_jpeg_clk_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(jpeg->clks), jpeg->clks);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get clocks\n");

	jpeg->rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(jpeg->rstc))
		return dev_err_probe(dev, PTR_ERR(jpeg->rstc), "cannot get reset\n");

	/* The unit addresses 32 bits; DRAM is below 4 GiB on the SG2002. */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/*
	 * Power the unit once to see that it answers and to leave its
	 * interrupt sources masked and clear before the line is requested.
	 */
	ret = sg_jpeg_power_on(jpeg);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the unit\n");
	for (i = 0; i < ARRAY_SIZE(jpeg->clks); i++)
		dev_info(dev, "clock %s: %lu Hz\n", jpeg->clks[i].id,
			 clk_get_rate(jpeg->clks[i].clk));
	sg_jpeg_log_state(jpeg, "probe");
	sg_jpeg_power_off(jpeg);

	if (use_irq) {
		jpeg->irq = platform_get_irq(pdev, 0);
		if (jpeg->irq < 0)
			return jpeg->irq;
		/* Enabled only while the unit is powered. */
		ret = devm_request_threaded_irq(dev, jpeg->irq, sg_jpeg_irq,
						sg_jpeg_irq_thread,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						dev_name(dev), jpeg);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request irq %d\n", jpeg->irq);
	}

	ret = v4l2_device_register(dev, &jpeg->v4l2_dev);
	if (ret)
		return ret;

	jpeg->m2m_dev = v4l2_m2m_init(&sg_jpeg_m2m_ops);
	if (IS_ERR(jpeg->m2m_dev)) {
		ret = PTR_ERR(jpeg->m2m_dev);
		goto err_v4l2;
	}

	vfd = &jpeg->vfd;
	strscpy(vfd->name, SG_JPEG_NAME, sizeof(vfd->name));
	vfd->fops = &sg_jpeg_fops;
	vfd->ioctl_ops = &sg_jpeg_ioctl_ops;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	vfd->lock = &jpeg->mutex;
	vfd->v4l2_dev = &jpeg->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;
	video_set_drvdata(vfd, jpeg);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	platform_set_drvdata(pdev, jpeg);
	dev_info(dev, "JPEG encoder at /dev/video%d, %s\n", vfd->num,
		 jpeg->irq > 0 ? "interrupt driven" : "polled");
	return 0;

err_m2m:
	v4l2_m2m_release(jpeg->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&jpeg->v4l2_dev);
	return ret;
}

static void sg_jpeg_remove(struct platform_device *pdev)
{
	struct sg_jpeg_dev *jpeg = platform_get_drvdata(pdev);

	video_unregister_device(&jpeg->vfd);
	cancel_delayed_work_sync(&jpeg->timeout_work);
	cancel_work_sync(&jpeg->poll_work);
	v4l2_m2m_release(jpeg->m2m_dev);
	v4l2_device_unregister(&jpeg->v4l2_dev);
	if (jpeg->missed_irqs)
		dev_info(jpeg->dev, "%lu pictures finished without an interrupt\n",
			 jpeg->missed_irqs);
}

static const struct of_device_id sg_jpeg_of_match[] = {
	{ .compatible = "sophgo,sg2002-jpeg" },
	{ }
};
MODULE_DEVICE_TABLE(of, sg_jpeg_of_match);

static struct platform_driver sg_jpeg_driver = {
	.driver = {
		.name = SG_JPEG_NAME,
		.of_match_table = sg_jpeg_of_match,
	},
	.probe = sg_jpeg_probe,
	.remove = sg_jpeg_remove,
};
module_platform_driver(sg_jpeg_driver);

MODULE_DESCRIPTION("Sophgo SG2002 JPEG encoder");
MODULE_LICENSE("GPL");
