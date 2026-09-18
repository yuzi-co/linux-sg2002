// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Inochi Amaoto <inochiama@outlook.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/gcd.h>
#include <linux/spinlock.h>

#include "clk-cv18xx-ip.h"
#include "clk-cv18xx-pll.h"

/* GATE */
static inline struct cv1800_clk_gate *hw_to_cv1800_clk_gate(struct clk_hw *hw)
{
	struct cv1800_clk_common *common = hw_to_cv1800_clk_common(hw);

	return container_of(common, struct cv1800_clk_gate, common);
}

static int gate_enable(struct clk_hw *hw)
{
	struct cv1800_clk_gate *gate = hw_to_cv1800_clk_gate(hw);

	return cv1800_clk_setbit(&gate->common, &gate->gate);
}

static void gate_disable(struct clk_hw *hw)
{
	struct cv1800_clk_gate *gate = hw_to_cv1800_clk_gate(hw);

	cv1800_clk_clearbit(&gate->common, &gate->gate);
}

static int gate_is_enabled(struct clk_hw *hw)
{
	struct cv1800_clk_gate *gate = hw_to_cv1800_clk_gate(hw);

	return cv1800_clk_checkbit(&gate->common, &gate->gate);
}

static unsigned long gate_recalc_rate(struct clk_hw *hw,
				      unsigned long parent_rate)
{
	return parent_rate;
}

static int gate_determine_rate(struct clk_hw *hw,
			       struct clk_rate_request *req)
{
	req->rate = req->best_parent_rate;

	return 0;
}

static int gate_set_rate(struct clk_hw *hw, unsigned long rate,
			 unsigned long parent_rate)
{
	return 0;
}

const struct clk_ops cv1800_clk_gate_ops = {
	.disable = gate_disable,
	.enable = gate_enable,
	.is_enabled = gate_is_enabled,

	.recalc_rate = gate_recalc_rate,
	.determine_rate = gate_determine_rate,
	.set_rate = gate_set_rate,
};

/* DIV */
#define _DIV_EN_CLK_DIV_FACTOR_FIELD		BIT(3)

#define DIV_GET_EN_CLK_DIV_FACTOR(_reg) \
	FIELD_GET(_DIV_EN_CLK_DIV_FACTOR_FIELD, _reg)

#define DIV_SET_EN_DIV_FACTOR(_reg) \
	_CV1800_SET_FIELD(_reg, 1, _DIV_EN_CLK_DIV_FACTOR_FIELD)

static inline struct cv1800_clk_div *hw_to_cv1800_clk_div(struct clk_hw *hw)
{
	struct cv1800_clk_common *common = hw_to_cv1800_clk_common(hw);

	return container_of(common, struct cv1800_clk_div, common);
}

static int div_enable(struct clk_hw *hw)
{
	struct cv1800_clk_div *div = hw_to_cv1800_clk_div(hw);

	return cv1800_clk_setbit(&div->common, &div->gate);
}

static void div_disable(struct clk_hw *hw)
{
	struct cv1800_clk_div *div = hw_to_cv1800_clk_div(hw);

	cv1800_clk_clearbit(&div->common, &div->gate);
}

static int div_is_enabled(struct clk_hw *hw)
{
	struct cv1800_clk_div *div = hw_to_cv1800_clk_div(hw);

	return cv1800_clk_checkbit(&div->common, &div->gate);
}

static int div_helper_set_rate(struct cv1800_clk_common *common,
			       struct cv1800_clk_regfield *div,
			       unsigned long val)
{
	unsigned long flags;
	u32 reg;

	if (div->width == 0)
		return 0;

	spin_lock_irqsave(common->lock, flags);

	reg = readl(common->base + div->reg);
	reg = cv1800_clk_regfield_set(reg, val, div);
	if (div->initval > 0)
		reg = DIV_SET_EN_DIV_FACTOR(reg);

	writel(reg, common->base + div->reg);

	spin_unlock_irqrestore(common->lock, flags);

	return 0;
}

static u32 div_helper_get_clockdiv(struct cv1800_clk_common *common,
				   struct cv1800_clk_regfield *div)
{
	u32 clockdiv = 1;
	u32 reg;

	if (!div || div->initval < 0 || (div->width == 0 && div->initval <= 0))
		return 1;

	if (div->width == 0 && div->initval > 0)
		return div->initval;

	reg = readl(common->base + div->reg);

	if (div->initval == 0 || DIV_GET_EN_CLK_DIV_FACTOR(reg))
		clockdiv = cv1800_clk_regfield_get(reg, div);
	else if (div->initval > 0)
		clockdiv = div->initval;

	return clockdiv;
}

static int div_helper_determine_rate(struct cv1800_clk_regfield *div,
				     struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	if (div->width == 0) {
		if (div->initval <= 0)
			req->rate = DIV_ROUND_UP_ULL(req->best_parent_rate, 1);
		else
			req->rate = DIV_ROUND_UP_ULL(req->best_parent_rate, div->initval);

		return 0;
	}

	return divider_determine_rate(hw, req, NULL, div->width, div->flags);
}

static int do_div_determine_rate(struct clk_rate_request *req, int id, void *data)
{
	struct cv1800_clk_div *div = data;

	return div_helper_determine_rate(&div->div, &div->common.hw, req);
}

static bool div_is_better_rate(struct cv1800_clk_common *common,
			       unsigned long target, unsigned long now,
			       unsigned long best)
{
	if (common->features & CLK_DIVIDER_ROUND_CLOSEST)
		return abs_diff(target, now) < abs_diff(target, best);

	return now <= target && now > best;
}

static int mux_helper_determine_rate(struct cv1800_clk_common *common,
				     struct clk_rate_request *req,
				     int (*round)(struct clk_rate_request *,
						  int,
						  void *),
				     void *data)
{
	unsigned long best_parent_rate = 0, best_rate = 0;
	struct clk_hw *best_parent, *hw = &common->hw;
	unsigned int i;
	int ret;

	if (clk_hw_get_flags(hw) & CLK_SET_RATE_NO_REPARENT) {
		struct clk_rate_request tmp_req = *req;

		best_parent = clk_hw_get_parent(hw);
		tmp_req.best_parent_hw = best_parent;
		tmp_req.best_parent_rate = clk_hw_get_rate(best_parent);

		ret = round(&tmp_req, -1, data);
		if (ret)
			return ret;

		best_parent_rate = tmp_req.best_parent_rate;
		best_rate = tmp_req.rate;

		goto find;
	}

	for (i = 0; i < clk_hw_get_num_parents(hw); i++) {
		struct clk_rate_request tmp_req = *req;
		struct clk_hw *parent;

		parent = clk_hw_get_parent_by_index(hw, i);
		if (!parent)
			continue;

		tmp_req.best_parent_hw = parent;
		tmp_req.best_parent_rate = clk_hw_get_rate(parent);

		ret = round(&tmp_req, i, data);
		if (ret)
			continue;

		if (tmp_req.rate == req->rate) {
			best_parent = parent;
			best_parent_rate = tmp_req.best_parent_rate;
			best_rate = tmp_req.rate;
			goto find;
		}

		if (div_is_better_rate(common, req->rate,
				       tmp_req.rate, best_rate)) {
			best_parent = parent;
			best_parent_rate = tmp_req.best_parent_rate;
			best_rate = tmp_req.rate;
		}
	}

	if (best_rate == 0)
		return -EINVAL;

find:
	req->best_parent_hw = best_parent;
	req->best_parent_rate = best_parent_rate;
	req->rate = best_rate;
	return 0;
}

static int div_determine_rate(struct clk_hw *hw,
			      struct clk_rate_request *req)
{
	struct cv1800_clk_div *div = hw_to_cv1800_clk_div(hw);

	return mux_helper_determine_rate(&div->common, req,
					 do_div_determine_rate, div);
}

static unsigned long div_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	struct cv1800_clk_div *div = hw_to_cv1800_clk_div(hw);
	unsigned long val;

	val = div_helper_get_clockdiv(&div->common, &div->div);
	if (val == 0)
		return 0;

	return divider_recalc_rate(hw, parent_rate, val, NULL,
				   div->div.flags, div->div.width);
}

static int div_set_rate(struct clk_hw *hw, unsigned long rate,
			 unsigned long parent_rate)
{
	struct cv1800_clk_div *div = hw_to_cv1800_clk_div(hw);
	unsigned long val;

	val = divider_get_val(rate, parent_rate, NULL,
			      div->div.width, div->div.flags);

	return div_helper_set_rate(&div->common, &div->div, val);
}

const struct clk_ops cv1800_clk_div_ops = {
	.disable = div_disable,
	.enable = div_enable,
	.is_enabled = div_is_enabled,

	.determine_rate = div_determine_rate,
	.recalc_rate	= div_recalc_rate,
	.set_rate = div_set_rate,
};

static inline struct cv1800_clk_bypass_div *
hw_to_cv1800_clk_bypass_div(struct clk_hw *hw)
{
	struct cv1800_clk_div *div = hw_to_cv1800_clk_div(hw);

	return container_of(div, struct cv1800_clk_bypass_div, div);
}

static int do_bypass_div_determine_rate(struct clk_rate_request *req, int id,
					void *data)
{
	struct cv1800_clk_bypass_div *div = data;

	if (id == -1) {
		if (cv1800_clk_checkbit(&div->div.common, &div->bypass)) {
			req->rate = req->best_parent_rate;

			return 0;
		}

		return do_div_determine_rate(req, -1, &div->div);
	}

	if (id == 0) {
		req->rate = req->best_parent_rate;

		return 0;
	}

	return do_div_determine_rate(req, id - 1, &div->div);
}

static int bypass_div_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	struct cv1800_clk_bypass_div *div = hw_to_cv1800_clk_bypass_div(hw);

	return mux_helper_determine_rate(&div->div.common, req,
					 do_bypass_div_determine_rate, div);
}

static unsigned long bypass_div_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct cv1800_clk_bypass_div *div = hw_to_cv1800_clk_bypass_div(hw);

	if (cv1800_clk_checkbit(&div->div.common, &div->bypass))
		return parent_rate;

	return div_recalc_rate(hw, parent_rate);
}

static int bypass_div_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct cv1800_clk_bypass_div *div = hw_to_cv1800_clk_bypass_div(hw);

	if (cv1800_clk_checkbit(&div->div.common, &div->bypass))
		return 0;

	return div_set_rate(hw, rate, parent_rate);
}

static u8 bypass_div_get_parent(struct clk_hw *hw)
{
	struct cv1800_clk_bypass_div *div = hw_to_cv1800_clk_bypass_div(hw);

	if (cv1800_clk_checkbit(&div->div.common, &div->bypass))
		return 0;

	return 1;
}

static int bypass_div_set_parent(struct clk_hw *hw, u8 index)
{
	struct cv1800_clk_bypass_div *div = hw_to_cv1800_clk_bypass_div(hw);

	if (index)
		return cv1800_clk_clearbit(&div->div.common, &div->bypass);

	return cv1800_clk_setbit(&div->div.common, &div->bypass);
}

const struct clk_ops cv1800_clk_bypass_div_ops = {
	.disable = div_disable,
	.enable = div_enable,
	.is_enabled = div_is_enabled,

	.determine_rate = bypass_div_determine_rate,
	.recalc_rate = bypass_div_recalc_rate,
	.set_rate = bypass_div_set_rate,

	.set_parent = bypass_div_set_parent,
	.get_parent = bypass_div_get_parent,
};

/* MUX */
static inline struct cv1800_clk_mux *hw_to_cv1800_clk_mux(struct clk_hw *hw)
{
	struct cv1800_clk_common *common = hw_to_cv1800_clk_common(hw);

	return container_of(common, struct cv1800_clk_mux, common);
}

static int mux_enable(struct clk_hw *hw)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);

	return cv1800_clk_setbit(&mux->common, &mux->gate);
}

static void mux_disable(struct clk_hw *hw)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);

	cv1800_clk_clearbit(&mux->common, &mux->gate);
}

static int mux_is_enabled(struct clk_hw *hw)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);

	return cv1800_clk_checkbit(&mux->common, &mux->gate);
}

static int do_mux_determine_rate(struct clk_rate_request *req, int id, void *data)
{
	struct cv1800_clk_mux *mux = data;

	return div_helper_determine_rate(&mux->div, &mux->common.hw, req);
}

static int mux_determine_rate(struct clk_hw *hw,
			      struct clk_rate_request *req)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);

	return mux_helper_determine_rate(&mux->common, req,
					 do_mux_determine_rate, mux);
}

static unsigned long mux_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);
	unsigned long val;

	val = div_helper_get_clockdiv(&mux->common, &mux->div);
	if (val == 0)
		return 0;

	return divider_recalc_rate(hw, parent_rate, val, NULL,
				   mux->div.flags, mux->div.width);
}

static int mux_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);
	unsigned long val;

	val = divider_get_val(rate, parent_rate, NULL,
			      mux->div.width, mux->div.flags);

	return div_helper_set_rate(&mux->common, &mux->div, val);
}

static u8 mux_get_parent(struct clk_hw *hw)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);
	u32 reg = readl(mux->common.base + mux->mux.reg);

	return cv1800_clk_regfield_get(reg, &mux->mux);
}

static int _mux_set_parent(struct cv1800_clk_mux *mux, u8 index)
{
	u32 reg;

	reg = readl(mux->common.base + mux->mux.reg);
	reg = cv1800_clk_regfield_set(reg, index, &mux->mux);
	writel(reg, mux->common.base + mux->mux.reg);

	return 0;
}

static int mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);
	unsigned long flags;

	spin_lock_irqsave(mux->common.lock, flags);

	_mux_set_parent(mux, index);

	spin_unlock_irqrestore(mux->common.lock, flags);

	return 0;
}

const struct clk_ops cv1800_clk_mux_ops = {
	.disable = mux_disable,
	.enable = mux_enable,
	.is_enabled = mux_is_enabled,

	.determine_rate = mux_determine_rate,
	.recalc_rate = mux_recalc_rate,
	.set_rate = mux_set_rate,

	.set_parent = mux_set_parent,
	.get_parent = mux_get_parent,
};

static inline struct cv1800_clk_bypass_mux *
hw_to_cv1800_clk_bypass_mux(struct clk_hw *hw)
{
	struct cv1800_clk_mux *mux = hw_to_cv1800_clk_mux(hw);

	return container_of(mux, struct cv1800_clk_bypass_mux, mux);
}

static int do_bypass_mux_determine_rate(struct clk_rate_request *req, int id,
					void *data)
{
	struct cv1800_clk_bypass_mux *mux = data;

	if (id == -1) {
		if (cv1800_clk_checkbit(&mux->mux.common, &mux->bypass)) {
			req->rate = req->best_parent_rate;

			return 0;
		}

		return do_mux_determine_rate(req, -1, &mux->mux);
	}

	if (id == 0) {
		req->rate = req->best_parent_rate;

		return 0;
	}

	return do_mux_determine_rate(req, id - 1, &mux->mux);
}

static int bypass_mux_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	struct cv1800_clk_bypass_mux *mux = hw_to_cv1800_clk_bypass_mux(hw);

	return mux_helper_determine_rate(&mux->mux.common, req,
					 do_bypass_mux_determine_rate, mux);
}

static unsigned long bypass_mux_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct cv1800_clk_bypass_mux *mux = hw_to_cv1800_clk_bypass_mux(hw);

	if (cv1800_clk_checkbit(&mux->mux.common, &mux->bypass))
		return parent_rate;

	return mux_recalc_rate(hw, parent_rate);
}

static int bypass_mux_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct cv1800_clk_bypass_mux *mux = hw_to_cv1800_clk_bypass_mux(hw);

	if (cv1800_clk_checkbit(&mux->mux.common, &mux->bypass))
		return 0;

	return mux_set_rate(hw, rate, parent_rate);
}

static u8 bypass_mux_get_parent(struct clk_hw *hw)
{
	struct cv1800_clk_bypass_mux *mux = hw_to_cv1800_clk_bypass_mux(hw);

	if (cv1800_clk_checkbit(&mux->mux.common, &mux->bypass))
		return 0;

	return mux_get_parent(hw) + 1;
}

static int bypass_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct cv1800_clk_bypass_mux *mux = hw_to_cv1800_clk_bypass_mux(hw);
	int ret;

	if (index == 0)
		return cv1800_clk_setbit(&mux->mux.common, &mux->bypass);

	ret = mux_set_parent(hw, index - 1);
	if (ret)
		return ret;

	return cv1800_clk_clearbit(&mux->mux.common, &mux->bypass);
}

const struct clk_ops cv1800_clk_bypass_mux_ops = {
	.disable = mux_disable,
	.enable = mux_enable,
	.is_enabled = mux_is_enabled,

	.determine_rate = bypass_mux_determine_rate,
	.recalc_rate = bypass_mux_recalc_rate,
	.set_rate = bypass_mux_set_rate,

	.set_parent = bypass_mux_set_parent,
	.get_parent = bypass_mux_get_parent,
};

/* MMUX */
static inline struct cv1800_clk_mmux *hw_to_cv1800_clk_mmux(struct clk_hw *hw)
{
	struct cv1800_clk_common *common = hw_to_cv1800_clk_common(hw);

	return container_of(common, struct cv1800_clk_mmux, common);
}

static u8 mmux_get_parent_id(struct cv1800_clk_mmux *mmux)
{
	struct clk_hw *hw = &mmux->common.hw;
	struct clk_hw *parent = clk_hw_get_parent(hw);
	unsigned int i;

	for (i = 0; i < clk_hw_get_num_parents(hw); i++) {
		if (parent == clk_hw_get_parent_by_index(hw, i))
			return i;
	}

	BUG();
}

static int mmux_enable(struct clk_hw *hw)
{
	struct cv1800_clk_mmux *mmux = hw_to_cv1800_clk_mmux(hw);

	return cv1800_clk_setbit(&mmux->common, &mmux->gate);
}

static void mmux_disable(struct clk_hw *hw)
{
	struct cv1800_clk_mmux *mmux = hw_to_cv1800_clk_mmux(hw);

	cv1800_clk_clearbit(&mmux->common, &mmux->gate);
}

static int mmux_is_enabled(struct clk_hw *hw)
{
	struct cv1800_clk_mmux *mmux = hw_to_cv1800_clk_mmux(hw);

	return cv1800_clk_checkbit(&mmux->common, &mmux->gate);
}

static int do_mmux_determine_rate(struct clk_rate_request *req, int id, void *data)
{
	struct cv1800_clk_mmux *mmux = data;
	s8 div_id;

	if (id == -1) {
		if (cv1800_clk_checkbit(&mmux->common, &mmux->bypass)) {
			req->rate = req->best_parent_rate;

			return 0;
		}

		id = mmux_get_parent_id(mmux);
	}

	div_id = mmux->parent2sel[id];

	if (div_id < 0) {
		req->rate = req->best_parent_rate;

		return 0;
	}

	return div_helper_determine_rate(&mmux->div[div_id], &mmux->common.hw,
					 req);
}

static int mmux_determine_rate(struct clk_hw *hw,
			       struct clk_rate_request *req)
{
	struct cv1800_clk_mmux *mmux = hw_to_cv1800_clk_mmux(hw);

	return mux_helper_determine_rate(&mmux->common, req,
					 do_mmux_determine_rate, mmux);
}

static unsigned long mmux_recalc_rate(struct clk_hw *hw,
				      unsigned long parent_rate)
{
	struct cv1800_clk_mmux *mmux = hw_to_cv1800_clk_mmux(hw);
	unsigned long val;
	struct cv1800_clk_regfield *div;

	if (cv1800_clk_checkbit(&mmux->common, &mmux->bypass))
		return parent_rate;

	if (cv1800_clk_checkbit(&mmux->common, &mmux->clk_sel))
		div = &mmux->div[0];
	else
		div = &mmux->div[1];

	val = div_helper_get_clockdiv(&mmux->common, div);
	if (val == 0)
		return 0;

	return divider_recalc_rate(hw, parent_rate, val, NULL,
				   div->flags, div->width);
}

/*
 * Move the CPU onto the inactive lane, running from that lane's own PLL at
 * no more than @limit. The SG200X TRM requires this before the active
 * divider is rewritten, and equally before the active lane's PLL is
 * retuned. Leaves the mux parked; cv1800_clk_mmux_unpark() puts it back.
 */
int cv1800_clk_mmux_park(struct cv1800_clk_mmux *mmux, unsigned long limit)
{
	struct clk_hw *hw = &mmux->common.hw;
	struct cv1800_clk_regfield *park_div, *park_mux;
	struct clk_hw *park_parent;
	unsigned long park_rate, flags;
	unsigned int lane, park_lane;
	u32 reg, park_val;

	if (mmux->retune_state != CV1800_MMUX_RUNNING)
		return -EBUSY;

	if (cv1800_clk_checkbit(&mmux->common, &mmux->bypass))
		return -EINVAL;

	lane = cv1800_clk_checkbit(&mmux->common, &mmux->clk_sel) ? 0 : 1;
	park_lane = !lane;
	park_div = &mmux->div[park_lane];
	park_mux = &mmux->mux[park_lane];

	/*
	 * Selector zero of the parked lane. Both it and the active lane's
	 * PLL are critical clocks; neither is retuned here.
	 */
	park_parent = clk_hw_get_parent_by_index(hw,
					mmux->sel2parent[park_lane][0]);
	if (!park_parent || !clk_hw_is_enabled(park_parent))
		return -EINVAL;
	park_rate = clk_hw_get_rate(park_parent);
	if (!park_rate || !limit)
		return -EINVAL;
	park_val = DIV_ROUND_UP_ULL(park_rate, limit);
	if (park_val > GENMASK(park_div->width - 1, 0))
		return -ERANGE;

	spin_lock_irqsave(mmux->common.lock, flags);
	mmux->retune_saved_lane = readl(mmux->common.base + park_div->reg);
	reg = cv1800_clk_regfield_set(mmux->retune_saved_lane, park_val,
				      park_div);
	reg = cv1800_clk_regfield_set(reg, 0, park_mux);
	writel(DIV_SET_EN_DIV_FACTOR(reg), mmux->common.base + park_div->reg);
	/* Complete the inactive-lane update before selecting it. */
	readl(mmux->common.base + park_div->reg);
	reg = readl(mmux->common.base + mmux->clk_sel.reg);
	writel(reg ^ BIT(mmux->clk_sel.shift),
	       mmux->common.base + mmux->clk_sel.reg);
	readl(mmux->common.base + mmux->clk_sel.reg);
	mmux->retune_state = CV1800_MMUX_PARKED;
	spin_unlock_irqrestore(mmux->common.lock, flags);

	return 0;
}

/*
 * Return the CPU to the lane it came from, optionally programming that
 * lane's divider to @div first. A negative @div keeps the existing value,
 * which is what a pure parent-PLL retune wants.
 */
int cv1800_clk_mmux_unpark(struct cv1800_clk_mmux *mmux, int div)
{
	struct cv1800_clk_regfield *active_div, *park_div;
	unsigned int lane, park_lane;
	unsigned long flags;
	u32 reg;

	if (mmux->retune_state != CV1800_MMUX_PARKED)
		return -EINVAL;

	/* While parked the selector names the parked lane; invert it. */
	park_lane = cv1800_clk_checkbit(&mmux->common, &mmux->clk_sel) ? 0 : 1;
	lane = !park_lane;
	active_div = &mmux->div[lane];
	park_div = &mmux->div[park_lane];

	if (div >= 0 && div > (int)GENMASK(active_div->width - 1, 0))
		return -ERANGE;

	spin_lock_irqsave(mmux->common.lock, flags);
	if (div >= 0) {
		reg = readl(mmux->common.base + active_div->reg);
		reg = cv1800_clk_regfield_set(reg, div, active_div);
		writel(DIV_SET_EN_DIV_FACTOR(reg),
		       mmux->common.base + active_div->reg);
		readl(mmux->common.base + active_div->reg);
	}
	/*
	 * Read-modify-write the shared selector register: the parked and
	 * unparked halves of a transition do not hold the lock across it,
	 * and neighbouring clocks own the other bits.
	 */
	reg = readl(mmux->common.base + mmux->clk_sel.reg);
	writel(reg ^ BIT(mmux->clk_sel.shift),
	       mmux->common.base + mmux->clk_sel.reg);
	readl(mmux->common.base + mmux->clk_sel.reg);
	writel(mmux->retune_saved_lane, mmux->common.base + park_div->reg);
	mmux->retune_state = CV1800_MMUX_RUNNING;
	spin_unlock_irqrestore(mmux->common.lock, flags);

	return 0;
}

static int mmux_set_rate(struct clk_hw *hw, unsigned long rate,
			 unsigned long parent_rate)
{
	struct cv1800_clk_mmux *mmux = hw_to_cv1800_clk_mmux(hw);
	struct cv1800_clk_regfield *div;
	unsigned long limit;
	unsigned int lane;
	int ret, val;

	switch (mmux->retune_state) {
	case CV1800_MMUX_PARKED:
		/*
		 * A parent-PLL retune left the CPU parked and never sent
		 * POST_RATE_CHANGE, because the PLL's rate did not actually
		 * move. It was therefore never rewritten: go straight back.
		 */
		return cv1800_clk_mmux_unpark(mmux, -1);
	case CV1800_MMUX_STRANDED:
		/*
		 * The mux is not on the lane the framework believes. Refuse
		 * rather than reprogram the lane the CPU is running from.
		 */
		return -EIO;
	default:
		break;
	}

	if (!rate || !parent_rate)
		return -EINVAL;

	if (cv1800_clk_checkbit(&mmux->common, &mmux->bypass))
		return 0;

	lane = cv1800_clk_checkbit(&mmux->common, &mmux->clk_sel) ? 0 : 1;
	div = &mmux->div[lane];
	val = divider_get_val(rate, parent_rate, NULL, div->width, div->flags);
	if (val < 0)
		return val;
	if (val == div_helper_get_clockdiv(&mmux->common, div))
		return 0;

	/*
	 * The CPU clock cannot be gated and its active divider must not be
	 * rewritten, so change it from the other lane. Do not exceed either
	 * endpoint frequency while parked.
	 */
	limit = min(rate, mmux_recalc_rate(hw, parent_rate));
	ret = cv1800_clk_mmux_park(mmux, limit);
	if (ret)
		return ret;

	return cv1800_clk_mmux_unpark(mmux, val);
}

static u8 mmux_get_parent(struct clk_hw *hw)
{
	struct cv1800_clk_mmux *mmux = hw_to_cv1800_clk_mmux(hw);
	struct cv1800_clk_regfield *mux;
	u32 reg;
	s8 clk_sel;

	if (cv1800_clk_checkbit(&mmux->common, &mmux->bypass))
		return 0;

	if (cv1800_clk_checkbit(&mmux->common, &mmux->clk_sel))
		clk_sel = 0;
	else
		clk_sel = 1;
	mux = &mmux->mux[clk_sel];

	reg = readl(mmux->common.base + mux->reg);

	return mmux->sel2parent[clk_sel][cv1800_clk_regfield_get(reg, mux)];
}

static int mmux_set_parent(struct clk_hw *hw, u8 index)
{
	struct cv1800_clk_mmux *mmux = hw_to_cv1800_clk_mmux(hw);
	struct cv1800_clk_regfield *mux;
	unsigned long flags;
	unsigned int mux_index;
	u32 reg;
	s8 clk_sel = mmux->parent2sel[index];

	if (index == 0 || clk_sel == -1)
		return cv1800_clk_setbit(&mmux->common, &mmux->bypass);

	mux = &mmux->mux[clk_sel];
	for (mux_index = 0; mux_index < BIT(mux->width); mux_index++)
		if (mmux->sel2parent[clk_sel][mux_index] == index)
			break;
	if (mux_index == BIT(mux->width))
		return -EINVAL;

	spin_lock_irqsave(mmux->common.lock, flags);

	reg = readl(mmux->common.base + mux->reg);
	reg = cv1800_clk_regfield_set(reg, mux_index, mux);

	writel(reg, mmux->common.base + mux->reg);

	spin_unlock_irqrestore(mmux->common.lock, flags);

	/* Program the chosen lane before selecting it and leaving bypass. */
	if (clk_sel)
		cv1800_clk_clearbit(&mmux->common, &mmux->clk_sel);
	else
		cv1800_clk_setbit(&mmux->common, &mmux->clk_sel);

	return cv1800_clk_clearbit(&mmux->common, &mmux->bypass);
}

const struct clk_ops cv1800_clk_mmux_ops = {
	.disable = mmux_disable,
	.enable = mmux_enable,
	.is_enabled = mmux_is_enabled,

	.determine_rate = mmux_determine_rate,
	.recalc_rate = mmux_recalc_rate,
	.set_rate = mmux_set_rate,

	.set_parent = mmux_set_parent,
	.get_parent = mmux_get_parent,
};

/*
 * Retuning the PLL a CPU is running from hangs that CPU. The common clock
 * framework has no way to express "move off this parent first", so watch
 * the PLL instead: park before it is rewritten, return afterwards, and
 * only once the hardware reports a completed update and a real lock.
 *
 * clk_change_rate() discards set_rate() errors and skips POST_RATE_CHANGE
 * when the recalculated rate is unchanged, so neither the PLL's return
 * value nor the POST callback can be relied on alone. mmux_set_rate() runs
 * unconditionally afterwards and releases a still-parked mux.
 */
static int mmux_retune_notifier(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct cv1800_clk_mmux *mmux = container_of(nb, struct cv1800_clk_mmux,
						    retune_nb);
	struct clk_notifier_data *cnd = data;
	struct clk_hw *hw = &mmux->common.hw;
	unsigned int lane;
	unsigned long limit;
	int ret;

	switch (event) {
	case PRE_RATE_CHANGE:
		/* Only park when this mux is actually fed by that PLL. */
		if (clk_hw_get_parent(hw) != &mmux->retune_pll->common.hw)
			return NOTIFY_DONE;
		/*
		 * The mux divider is untouched across the retune, so the CPU
		 * tracks the PLL through it. Stay at or below both endpoints.
		 */
		lane = cv1800_clk_checkbit(&mmux->common, &mmux->clk_sel) ? 0 : 1;
		limit = min(cnd->old_rate, cnd->new_rate) /
			div_helper_get_clockdiv(&mmux->common, &mmux->div[lane]);

		ret = cv1800_clk_mmux_park(mmux, limit);
		if (ret) {
			pr_err("%s: cannot park CPU off %s, refusing retune\n",
			       clk_hw_get_name(hw),
			       clk_hw_get_name(&mmux->retune_pll->common.hw));
			return notifier_from_errno(ret);
		}
		mmux->retune_saved_pll = readl(mmux->common.base +
					       mmux->retune_pll->pll_reg);
		return NOTIFY_OK;

	case POST_RATE_CHANGE:
		if (mmux->retune_state != CV1800_MMUX_PARKED)
			return NOTIFY_DONE;
		if (!cv1800_clk_pll_poll_lock(mmux->retune_pll)) {
			cv1800_clk_mmux_unpark(mmux, -1);
			return NOTIFY_OK;
		}
		/*
		 * The new setting never locked. Put the PLL back where it
		 * came from, which did lock, and return the CPU to it.
		 */
		writel(mmux->retune_saved_pll,
		       mmux->common.base + mmux->retune_pll->pll_reg);
		ret = cv1800_clk_pll_poll_lock(mmux->retune_pll);
		if (ret) {
			mmux->retune_state = CV1800_MMUX_STRANDED;
			/*
			 * The CPU is staying on the alternate lane, so its
			 * parent really has changed. Tell the framework, or
			 * every reported CPU rate from here on is a lie.
			 */
			clk_hw_reparent(hw, clk_hw_get_parent_by_index(hw,
							mmux_get_parent(hw)));
			pr_err("%s: %s neither locked nor rolled back (%d); staying on the alternate lane\n",
			       clk_hw_get_name(hw),
			       clk_hw_get_name(&mmux->retune_pll->common.hw),
			       ret);
			return NOTIFY_OK;
		}
		pr_warn("%s: %s did not lock; restored its previous rate\n",
			clk_hw_get_name(hw),
			clk_hw_get_name(&mmux->retune_pll->common.hw));
		cv1800_clk_mmux_unpark(mmux, -1);
		return NOTIFY_OK;

	case ABORT_RATE_CHANGE:
		/* The PLL was never rewritten; go straight back. */
		if (mmux->retune_state == CV1800_MMUX_PARKED)
			cv1800_clk_mmux_unpark(mmux, -1);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

int cv1800_clk_mmux_add_retune(struct device *dev,
			       struct cv1800_clk_mmux *mmux)
{
	struct clk *pll;

	if (!mmux->retune_pll)
		return 0;

	pll = devm_clk_hw_get_clk(dev, &mmux->retune_pll->common.hw,
				  "cpu-retune");
	if (IS_ERR(pll))
		return PTR_ERR(pll);

	mmux->retune_nb.notifier_call = mmux_retune_notifier;

	return devm_clk_notifier_register(dev, pll, &mmux->retune_nb);
}

/* AUDIO CLK */
static inline struct cv1800_clk_audio *
hw_to_cv1800_clk_audio(struct clk_hw *hw)
{
	struct cv1800_clk_common *common = hw_to_cv1800_clk_common(hw);

	return container_of(common, struct cv1800_clk_audio, common);
}

static int aclk_enable(struct clk_hw *hw)
{
	struct cv1800_clk_audio *aclk = hw_to_cv1800_clk_audio(hw);

	cv1800_clk_setbit(&aclk->common, &aclk->src_en);
	return cv1800_clk_setbit(&aclk->common, &aclk->output_en);
}

static void aclk_disable(struct clk_hw *hw)
{
	struct cv1800_clk_audio *aclk = hw_to_cv1800_clk_audio(hw);

	cv1800_clk_clearbit(&aclk->common, &aclk->output_en);
	cv1800_clk_clearbit(&aclk->common, &aclk->src_en);
}

static int aclk_is_enabled(struct clk_hw *hw)
{
	struct cv1800_clk_audio *aclk = hw_to_cv1800_clk_audio(hw);

	return cv1800_clk_checkbit(&aclk->common, &aclk->output_en);
}

static int aclk_determine_rate(struct clk_hw *hw,
			       struct clk_rate_request *req)
{
	struct cv1800_clk_audio *aclk = hw_to_cv1800_clk_audio(hw);

	req->rate = aclk->target_rate;

	return 0;
}

static unsigned long aclk_recalc_rate(struct clk_hw *hw,
				      unsigned long parent_rate)
{
	struct cv1800_clk_audio *aclk = hw_to_cv1800_clk_audio(hw);
	u64 rate = parent_rate;
	u64 factor = 2;
	u32 regval;

	if (!cv1800_clk_checkbit(&aclk->common, &aclk->div_en))
		return 0;

	regval = readl(aclk->common.base + aclk->m.reg);
	factor *= cv1800_clk_regfield_get(regval, &aclk->m);

	regval = readl(aclk->common.base + aclk->n.reg);
	rate *= cv1800_clk_regfield_get(regval, &aclk->n);

	return DIV64_U64_ROUND_UP(rate, factor);
}

static void aclk_determine_mn(unsigned long parent_rate, unsigned long rate,
			      u32 *m, u32 *n)
{
	u32 tm = parent_rate / 2;
	u32 tn = rate;
	u32 tcommon = gcd(tm, tn);
	*m = tm / tcommon;
	*n = tn / tcommon;
}

static int aclk_set_rate(struct clk_hw *hw, unsigned long rate,
			 unsigned long parent_rate)
{
	struct cv1800_clk_audio *aclk = hw_to_cv1800_clk_audio(hw);
	unsigned long flags;
	u32 m, n;

	aclk_determine_mn(parent_rate, rate,
			  &m, &n);

	spin_lock_irqsave(aclk->common.lock, flags);

	writel(m, aclk->common.base + aclk->m.reg);
	writel(n, aclk->common.base + aclk->n.reg);

	cv1800_clk_setbit(&aclk->common, &aclk->div_en);
	cv1800_clk_setbit(&aclk->common, &aclk->div_up);

	spin_unlock_irqrestore(aclk->common.lock, flags);

	return 0;
}

const struct clk_ops cv1800_clk_audio_ops = {
	.disable = aclk_disable,
	.enable = aclk_enable,
	.is_enabled = aclk_is_enabled,

	.determine_rate = aclk_determine_rate,
	.recalc_rate = aclk_recalc_rate,
	.set_rate = aclk_set_rate,
};
