// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Sophgo Technology Inc.
 * Copyright (C) 2024 Yuntao Dai <d1581209858@live.com>
 * Copyright (C) 2025 Junhui Liu <junhui.liu@pigmoral.tech>
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define RECV_CPU		1

#define MAILBOX_MAX_CPU		4
#define MAILBOX_MAX_CHAN	8
#define MAILBOX_MSG_LEN		8

#define MBOX_EN_REG(cpu)	(cpu << 2)
#define MBOX_DONE_REG(cpu)	((cpu << 2) + 2)
#define MBOX_SET_CLR_REG(cpu)	(0x10 + (cpu << 4))
#define MBOX_SET_INT_REG(cpu)	(0x18 + (cpu << 4))
#define MBOX_SET_REG		0x60

#define MAILBOX_CONTEXT_OFFSET	0x0400
#define MAILBOX_CONTEXT_SIZE	0x0040

#define MBOX_CONTEXT_BASE_INDEX(base, index) \
	((u64 __iomem *)(base + MAILBOX_CONTEXT_OFFSET) + index)

/*
 * Cvitek's vendor spinlock block starts at mailbox + 0xc0.  Field 4 is the
 * cross-core mailbox lock, so its 16-bit token register is at +0xd0.
 */
#define MBOX_HWLOCK_BANK_OFFSET	0x00c0
#define MBOX_HWLOCK_FIELD	4
#define MBOX_HWLOCK_REG		(MBOX_HWLOCK_BANK_OFFSET + \
				 sizeof(u32) * MBOX_HWLOCK_FIELD)
#define MBOX_HWLOCK_TASK_ATTEMPTS	1024
#define MBOX_HWLOCK_IRQ_ATTEMPTS	64
#define MBOX_HWLOCK_RETRY_MS	1
#define MBOX_HWLOCK_LINUX_TOKEN_MASK	GENMASK(7, 0)
#define MBOX_HWLOCK_RTOS_TOKEN_MASK	GENMASK(15, 8)

static_assert(MBOX_HWLOCK_REG == 0x00d0);
static_assert(!(MBOX_HWLOCK_LINUX_TOKEN_MASK &
		MBOX_HWLOCK_RTOS_TOKEN_MASK));

/**
 * struct cv1800_mbox_chan_priv - cv1800 mailbox channel private data
 * @idx: index of channel
 * @cpu: send to which processor
 */
struct cv1800_mbox_chan_priv {
	int idx;
	int cpu;
};

struct cv1800_mbox {
	struct mbox_controller mbox;
	struct cv1800_mbox_chan_priv priv[MAILBOX_MAX_CHAN];
	struct mbox_chan chans[MAILBOX_MAX_CHAN];
	u64 content[MAILBOX_MAX_CHAN];
	u64 deferred_data[MAILBOX_MAX_CHAN];
	struct delayed_work retry_work;
	void __iomem *mbox_base;
	void __iomem *hwlock;
	raw_spinlock_t register_lock;
	atomic64_t lock_timeouts;
	atomic64_t unlock_errors;
	atomic64_t busy_retries;
	u8 deferred;
	u8 subscribed;
	u8 next_token;
	u16 owner_token;
	int irq;
};

/* Caller holds register_lock with local interrupts disabled. */
static bool cv1800_hwlock_acquire(struct cv1800_mbox *mbox,
				   unsigned int attempts)
{
	unsigned int attempt;
	u16 token;

	token = ++mbox->next_token;
	if (!token)
		token = ++mbox->next_token;
	mbox->owner_token = token;

	/* The remote critical sections contain only a few ordered MMIO cycles. */
	for (attempt = 0; attempt < attempts; ++attempt) {
		writew(token, mbox->hwlock);
		if (readw(mbox->hwlock) == token)
			return true;
		cpu_relax();
	}

	return false;
}

/* Caller holds register_lock with local interrupts disabled. */
static bool cv1800_hwlock_release(struct cv1800_mbox *mbox)
{
	if (readw(mbox->hwlock) != mbox->owner_token)
		return false;

	/* Flush payload/register writes before making the field available. */
	wmb();
	writew(mbox->owner_token, mbox->hwlock);
	return true;
}

static bool cv1800_registers_lock(struct cv1800_mbox *mbox,
				   unsigned long *flags,
				   unsigned int attempts)
{
	raw_spin_lock_irqsave(&mbox->register_lock, *flags);
	if (cv1800_hwlock_acquire(mbox, attempts))
		return true;
	raw_spin_unlock_irqrestore(&mbox->register_lock, *flags);

	atomic64_inc(&mbox->lock_timeouts);
	dev_err_ratelimited(mbox->mbox.dev,
		"mailbox hardware lock timed out (%lld total)\n",
		atomic64_read(&mbox->lock_timeouts));
	return false;
}

static void cv1800_registers_unlock(struct cv1800_mbox *mbox,
				     unsigned long flags)
{
	bool released = cv1800_hwlock_release(mbox);

	raw_spin_unlock_irqrestore(&mbox->register_lock, flags);
	if (!released) {
		atomic64_inc(&mbox->unlock_errors);
		dev_err_ratelimited(mbox->mbox.dev,
			"mailbox hardware unlock failed (%lld total)\n",
			atomic64_read(&mbox->unlock_errors));
	}
}

/* Caller owns both the local and hardware register locks. */
static bool cv1800_slot_busy_locked(struct cv1800_mbox *mbox, int idx)
{
	unsigned int cpu;

	for (cpu = 0; cpu < MAILBOX_MAX_CPU; ++cpu) {
		if (readb(mbox->mbox_base + MBOX_EN_REG(cpu)) & BIT(idx))
			return true;
	}
	return false;
}

static bool cv1800_transmit_locked(struct cv1800_mbox *mbox, int idx, int cpu,
				    u64 message)
{
	u8 en, valid;

	if (cv1800_slot_busy_locked(mbox, idx))
		return false;

	memcpy_toio(MBOX_CONTEXT_BASE_INDEX(mbox->mbox_base, idx),
		    &message, MAILBOX_MSG_LEN);

	valid = BIT(idx);
	writeb(valid, mbox->mbox_base + MBOX_SET_CLR_REG(cpu));
	en = readb(mbox->mbox_base + MBOX_EN_REG(cpu));
	writeb(en | valid, mbox->mbox_base + MBOX_EN_REG(cpu));
	wmb();
	writeb(valid, mbox->mbox_base + MBOX_SET_REG);
	return true;
}

static void cv1800_retry_deferred(struct work_struct *work)
{
	struct cv1800_mbox *mbox = container_of(to_delayed_work(work),
						 struct cv1800_mbox, retry_work);
	unsigned long flags;
	u8 deferred;
	size_t i;

	if (!cv1800_registers_lock(mbox, &flags,
				   MBOX_HWLOCK_TASK_ATTEMPTS)) {
		raw_spin_lock_irqsave(&mbox->register_lock, flags);
		deferred = mbox->deferred & mbox->subscribed;
		raw_spin_unlock_irqrestore(&mbox->register_lock, flags);
		if (deferred)
			mod_delayed_work(system_highpri_wq, &mbox->retry_work,
					 msecs_to_jiffies(MBOX_HWLOCK_RETRY_MS));
		return;
	}

	deferred = mbox->deferred;
	mbox->deferred &= mbox->subscribed;
	deferred &= mbox->subscribed;
	for (i = 0; i < MAILBOX_MAX_CHAN; ++i) {
		if (!(deferred & BIT(i)))
			continue;
		if (cv1800_transmit_locked(mbox, i, mbox->priv[i].cpu,
					   mbox->deferred_data[i]))
			mbox->deferred &= ~BIT(i);
	}
	deferred = mbox->deferred;
	cv1800_registers_unlock(mbox, flags);
	if (deferred) {
		atomic64_inc(&mbox->busy_retries);
		dev_warn_ratelimited(mbox->mbox.dev,
			"mailbox payload slot remains busy (%lld retries)\n",
			atomic64_read(&mbox->busy_retries));
		mod_delayed_work(system_highpri_wq, &mbox->retry_work,
				 msecs_to_jiffies(MBOX_HWLOCK_RETRY_MS));
	}
}

static int cv1800_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct cv1800_mbox_chan_priv *priv = chan->con_priv;
	struct cv1800_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	unsigned long flags;
	u64 message;

	memcpy(&message, data, sizeof(message));
	if (cv1800_registers_lock(mbox, &flags,
				  MBOX_HWLOCK_IRQ_ATTEMPTS)) {
		bool transmitted = cv1800_transmit_locked(mbox, priv->idx,
							priv->cpu, message);

		cv1800_registers_unlock(mbox, flags);
		if (transmitted)
			return 0;
		atomic64_inc(&mbox->busy_retries);
		dev_warn_ratelimited(mbox->mbox.dev,
			"mailbox payload slot busy, deferring (%lld retries)\n",
			atomic64_read(&mbox->busy_retries));
	}

	/*
	 * The mailbox core cannot recover an initial ->send_data error: its
	 * request remains queued without becoming the polled active request.
	 * Accept one core-serialized message per channel into stable storage.
	 */
	raw_spin_lock_irqsave(&mbox->register_lock, flags);
	if (mbox->subscribed & BIT(priv->idx)) {
		mbox->deferred_data[priv->idx] = message;
		mbox->deferred |= BIT(priv->idx);
	}
	raw_spin_unlock_irqrestore(&mbox->register_lock, flags);
	if (READ_ONCE(mbox->subscribed) & BIT(priv->idx))
		mod_delayed_work(system_highpri_wq, &mbox->retry_work,
				 msecs_to_jiffies(MBOX_HWLOCK_RETRY_MS));

	return 0;
}

static bool cv1800_last_tx_done(struct mbox_chan *chan)
{
	struct cv1800_mbox_chan_priv *priv =
		(struct cv1800_mbox_chan_priv *)chan->con_priv;
	struct cv1800_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	unsigned long flags;
	bool done;

	if (!(READ_ONCE(mbox->subscribed) & BIT(priv->idx)))
		return false;
	if (!cv1800_registers_lock(mbox, &flags,
				   MBOX_HWLOCK_IRQ_ATTEMPTS))
		return false;
	if (mbox->deferred & BIT(priv->idx))
		done = false;
	else
		done = !cv1800_slot_busy_locked(mbox, priv->idx);
	cv1800_registers_unlock(mbox, flags);
	return done;
}

static irqreturn_t cv1800_mbox_isr(int irq, void *dev_id)
{
	struct cv1800_mbox *mbox = dev_id;
	unsigned long flags;
	u8 en, pending = 0;
	u8 set, valid;
	size_t i;

	if (!cv1800_registers_lock(mbox, &flags,
				   MBOX_HWLOCK_TASK_ATTEMPTS))
		return IRQ_HANDLED;

	set = readb(mbox->mbox_base + MBOX_SET_INT_REG(RECV_CPU));
	for (i = 0; i < MAILBOX_MAX_CHAN; ++i) {
		valid = set & BIT(i);
		if (!valid)
			continue;

		/* Copy before ACK/EN-clear releases the sender's payload slot. */
		memcpy_fromio(&mbox->content[i],
			      MBOX_CONTEXT_BASE_INDEX(mbox->mbox_base, i),
			      MAILBOX_MSG_LEN);
		pending |= valid;
		writeb(valid, mbox->mbox_base + MBOX_SET_CLR_REG(RECV_CPU));
		en = readb(mbox->mbox_base + MBOX_EN_REG(RECV_CPU));
		writeb(en & ~valid, mbox->mbox_base + MBOX_EN_REG(RECV_CPU));
	}
	cv1800_registers_unlock(mbox, flags);

	/* A callback can immediately send a response; never call it locked. */
	for (i = 0; i < MAILBOX_MAX_CHAN; ++i) {
		if ((pending & BIT(i)) &&
		    (READ_ONCE(mbox->subscribed) & BIT(i)) &&
		    mbox->chans[i].cl)
			mbox_chan_received_data(&mbox->chans[i], &mbox->content[i]);
	}

	return pending ? IRQ_HANDLED : IRQ_NONE;
}

static irqreturn_t cv1800_mbox_irq(int irq, void *dev_id)
{
	struct cv1800_mbox *mbox = dev_id;
	u8 set;

	/* The threaded handler owns all payload, ACK and enable operations. */
	set = readb(mbox->mbox_base + MBOX_SET_INT_REG(RECV_CPU));
	return set ? IRQ_WAKE_THREAD : IRQ_NONE;
}

static int cv1800_mbox_startup(struct mbox_chan *chan)
{
	struct cv1800_mbox_chan_priv *priv = chan->con_priv;
	struct cv1800_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	unsigned long flags;

	raw_spin_lock_irqsave(&mbox->register_lock, flags);
	mbox->deferred &= ~BIT(priv->idx);
	mbox->subscribed |= BIT(priv->idx);
	raw_spin_unlock_irqrestore(&mbox->register_lock, flags);
	return 0;
}

static void cv1800_mbox_shutdown(struct mbox_chan *chan)
{
	struct cv1800_mbox_chan_priv *priv = chan->con_priv;
	struct cv1800_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	unsigned long flags;
	u8 deferred;

	raw_spin_lock_irqsave(&mbox->register_lock, flags);
	mbox->subscribed &= ~BIT(priv->idx);
	mbox->deferred &= ~BIT(priv->idx);
	raw_spin_unlock_irqrestore(&mbox->register_lock, flags);

	/* Stop deferred TX and callbacks before mailbox core clears chan->cl. */
	cancel_delayed_work_sync(&mbox->retry_work);
	synchronize_irq(mbox->irq);

	/* Close mailbox-core's polling callback race with chan->cl teardown. */
	hrtimer_cancel(&mbox->mbox.poll_hrt);
	raw_spin_lock_irqsave(&mbox->register_lock, flags);
	deferred = mbox->deferred & mbox->subscribed;
	raw_spin_unlock_irqrestore(&mbox->register_lock, flags);
	if (deferred)
		mod_delayed_work(system_highpri_wq, &mbox->retry_work,
				 msecs_to_jiffies(MBOX_HWLOCK_RETRY_MS));
	if (READ_ONCE(mbox->subscribed)) {
		spin_lock_irqsave(&mbox->mbox.poll_hrt_lock, flags);
		hrtimer_start(&mbox->mbox.poll_hrt, 0, HRTIMER_MODE_REL);
		spin_unlock_irqrestore(&mbox->mbox.poll_hrt_lock, flags);
	}
}

static const struct mbox_chan_ops cv1800_mbox_chan_ops = {
	.startup = cv1800_mbox_startup,
	.shutdown = cv1800_mbox_shutdown,
	.send_data = cv1800_mbox_send_data,
	.last_tx_done = cv1800_last_tx_done,
};

static struct mbox_chan *cv1800_mbox_xlate(struct mbox_controller *mbox,
					   const struct of_phandle_args *spec)
{
	struct cv1800_mbox_chan_priv *priv;

	u32 idx = spec->args[0];
	u32 cpu = spec->args[1];

	if (spec->args_count != 2 || idx >= mbox->num_chans ||
	    cpu >= MAILBOX_MAX_CPU)
		return ERR_PTR(-EINVAL);

	priv = mbox->chans[idx].con_priv;
	priv->cpu = cpu;

	return &mbox->chans[idx];
}

static const struct of_device_id cv1800_mbox_of_match[] = {
	{ .compatible = "sophgo,cv1800b-mailbox", },
	{},
};
MODULE_DEVICE_TABLE(of, cv1800_mbox_of_match);

static int cv1800_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cv1800_mbox *mb;
	int irq, idx, err;

	mb = devm_kzalloc(dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;

	mb->mbox_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mb->mbox_base))
		return dev_err_probe(dev, PTR_ERR(mb->mbox_base),
				     "Failed to map resource\n");
	mb->hwlock = mb->mbox_base + MBOX_HWLOCK_REG;
	raw_spin_lock_init(&mb->register_lock);
	INIT_DELAYED_WORK(&mb->retry_work, cv1800_retry_deferred);
	atomic64_set(&mb->lock_timeouts, 0);
	atomic64_set(&mb->unlock_errors, 0);
	atomic64_set(&mb->busy_retries, 0);

	mb->mbox.dev = dev;
	mb->mbox.chans = mb->chans;
	mb->mbox.txdone_poll = true;
	mb->mbox.ops = &cv1800_mbox_chan_ops;
	mb->mbox.num_chans = MAILBOX_MAX_CHAN;
	mb->mbox.of_xlate = cv1800_mbox_xlate;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	mb->irq = irq;

	err = devm_request_threaded_irq(dev, irq, cv1800_mbox_irq,
					cv1800_mbox_isr, IRQF_ONESHOT,
					dev_name(&pdev->dev), mb);
	if (err < 0)
		return err;

	for (idx = 0; idx < MAILBOX_MAX_CHAN; idx++) {
		mb->priv[idx].idx = idx;
		mb->mbox.chans[idx].con_priv = &mb->priv[idx];
	}

	platform_set_drvdata(pdev, mb);

	err = devm_mbox_controller_register(dev, &mb->mbox);
	if (err)
		return dev_err_probe(dev, err, "Failed to register mailbox\n");

	return 0;
}

static void cv1800_mbox_remove(struct platform_device *pdev)
{
	struct cv1800_mbox *mbox = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&mbox->retry_work);
}

static struct platform_driver cv1800_mbox_driver = {
	.driver = {
		.name = "cv1800-mbox",
		.of_match_table = cv1800_mbox_of_match,
	},
	.probe	= cv1800_mbox_probe,
	.remove = cv1800_mbox_remove,
};

module_platform_driver(cv1800_mbox_driver);

MODULE_DESCRIPTION("cv1800 mailbox driver");
MODULE_LICENSE("GPL");
