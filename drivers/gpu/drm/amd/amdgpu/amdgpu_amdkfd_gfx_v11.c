/*
 * Copyright 2021 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#include <linux/mmu_context.h>
#include "amdgpu.h"
#include "amdgpu_amdkfd.h"
#include "gc/gc_11_0_0_offset.h"
#include "gc/gc_11_0_0_sh_mask.h"
#include "oss/osssys_6_0_0_offset.h"
#include "oss/osssys_6_0_0_sh_mask.h"
#include "soc15_common.h"
#include "soc15d.h"
#include "v11_structs.h"
#include "soc21.h"
#include "soc21_enum.h"
#include <uapi/linux/kfd_ioctl.h>

enum amdkfd_gfx11_pcs_sqcmd_policy {
	/* Variant A: MODE_SINGLE + CHECK_VMID=1 + queue_id=0 + wave_id sweep */
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_Q0 = 0,
	/* Current policy: MODE_SINGLE + CHECK_VMID=1 + queue_id/wave_id sweep */
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP = 1,
	/* Variant B: MODE_BROADCAST, no queue/wave/vmid targeting (gfx12-like) */
	AMDKFD_GFX11_PCS_SQCMD_BROADCAST = 2,
	/* Variant C: MODE_SINGLE + CHECK_VMID=0 + queue_id/wave_id sweep */
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP_NO_VMID_CHECK = 3,
	/* Probe mode: broadcast + low cap + disable after first nonzero post_status */
	AMDKFD_GFX11_PCS_SQCMD_BROADCAST_PROBE = 4,
	/* Queue-targeted broadcast (all waves in one queue) */
	AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE = 5,
	/* Probe mode: broadcast_queue + low cap + disable after first nonzero post_status */
	AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_PROBE = 6,
	/* MODE_SINGLE + CHECK_VMID=1 + fixed queue_id=0 + fixed wave_id=0 */
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_FIXED_Q0_W0 = 7,
	/* MODE_SINGLE + CHECK_VMID=1 + queue_id sweep + fixed wave_id=0 */
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP_W0 = 8,
	/* Control: do not emit SQ_CMD writes (status polling only) */
	AMDKFD_GFX11_PCS_SQCMD_NOOP = 9,
	/* MODE_BROADCAST_QUEUE + CHECK_VMID=0 + queue sweep */
	AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_NO_VMID = 10,
	/* Probe mode: broadcast_queue + CHECK_VMID=0 + low cap + auto-disable */
	AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_NO_VMID_PROBE = 11,
	/* MODE_SINGLE + CHECK_VMID=0 + wave sweep + no queue targeting */
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE_NO_VMID = 12,
	/* MODE_BROADCAST_QUEUE + CHECK_VMID=0 + queue+wave sweep + probe */
	AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_WAVE_NO_VMID_PROBE = 13,
	/* MODE_SINGLE + CHECK_VMID=1 + wave sweep + no queue targeting */
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE = 14,
	/* Probe mode: single + CHECK_VMID=1 + wave sweep + no queue targeting */
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE_PROBE = 15,
};

static int amdkfd_gfx11_pcs_sqcmd_policy =
	AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP;
module_param_named(amdkfd_gfx11_pcs_sqcmd_policy,
		   amdkfd_gfx11_pcs_sqcmd_policy, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_sqcmd_policy,
		 "gfx11 PC sampling SQ_CMD policy: "
		 "0=single_q0, 1=single_queue_sweep, 2=broadcast, "
		 "3=single_queue_sweep_no_vmid_check, 4=broadcast_probe, "
		 "5=broadcast_queue, 6=broadcast_queue_probe, "
		 "7=single_fixed_q0_w0, 8=single_queue_sweep_w0, 9=noop, "
		 "10=broadcast_queue_no_vmid, 11=broadcast_queue_no_vmid_probe, "
		 "12=single_wave_no_queue_no_vmid, 13=broadcast_queue_wave_no_vmid_probe, "
		 "14=single_wave_no_queue, 15=single_wave_no_queue_probe");

static int amdkfd_gfx11_pcs_max_injected_traps = 512;
module_param_named(amdkfd_gfx11_pcs_max_injected_traps,
		   amdkfd_gfx11_pcs_max_injected_traps, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_max_injected_traps,
		 "gfx11 PC sampling: maximum SQ_CMD injections per VMID (1..4096)");

static int amdkfd_gfx11_pcs_post_status_delay_us = 100;
module_param_named(amdkfd_gfx11_pcs_post_status_delay_us,
		   amdkfd_gfx11_pcs_post_status_delay_us, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_post_status_delay_us,
		 "gfx11 PC sampling: delay before post-SQ_CMD hosttrap-status read (0..200000 us)");

static int amdkfd_gfx11_pcs_wave_scan_on_reset = 0;
module_param_named(amdkfd_gfx11_pcs_wave_scan_on_reset,
		   amdkfd_gfx11_pcs_wave_scan_on_reset, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_wave_scan_on_reset,
		 "gfx11 PC sampling debug: dump one wave-state VMID summary when trigger state resets (0=off, 1=on)");

static int amdkfd_gfx11_pcs_wave_scan_max_waves = 32;
module_param_named(amdkfd_gfx11_pcs_wave_scan_max_waves,
		   amdkfd_gfx11_pcs_wave_scan_max_waves, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_wave_scan_max_waves,
		 "gfx11 PC sampling debug: waves per SH to sample in VMID summary scan (1..32)");

static int amdkfd_gfx11_pcs_wave_target_debug = 0;
module_param_named(amdkfd_gfx11_pcs_wave_target_debug,
		   amdkfd_gfx11_pcs_wave_target_debug, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_wave_target_debug,
		 "gfx11 PC sampling debug: for each logged SQ_CMD, report whether requested wave_id appears active (0=off, 1=on)");

static int amdkfd_gfx11_pcs_runtime_reg_readback_on_reset = 1;
module_param_named(amdkfd_gfx11_pcs_runtime_reg_readback_on_reset,
		   amdkfd_gfx11_pcs_runtime_reg_readback_on_reset, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_runtime_reg_readback_on_reset,
		 "gfx11 PC sampling debug: dump runtime TBA/TMA/GDBG readback at trigger reset (0=off, 1=on)");

static int amdkfd_gfx11_pcs_single_wave_use_status_slot = 0;
module_param_named(amdkfd_gfx11_pcs_single_wave_use_status_slot,
		   amdkfd_gfx11_pcs_single_wave_use_status_slot, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_single_wave_use_status_slot,
		 "gfx11 PC sampling debug: MODE_SINGLE WAVE_ID source (0=HW_ID1.wave_id, 1=status-slot index)");

static int amdkfd_gfx11_pcs_sqcmd_cmd_override = -1;
module_param_named(amdkfd_gfx11_pcs_sqcmd_cmd_override,
		   amdkfd_gfx11_pcs_sqcmd_cmd_override, int, 0644);
MODULE_PARM_DESC(amdkfd_gfx11_pcs_sqcmd_cmd_override,
		 "gfx11 PC sampling debug: override SQ_CMD.CMD (-1=default TRAP, 0..15=raw CMD)");

/* Last programmed trap-handler addresses per VMID for trigger-time correlation. */
static u64 amdkfd_gfx11_last_tba_byte[AMDGPU_NUM_VMID];
static u64 amdkfd_gfx11_last_tma_byte[AMDGPU_NUM_VMID];
static u64 amdkfd_gfx11_last_tba_reg[AMDGPU_NUM_VMID];
static u64 amdkfd_gfx11_last_tma_reg[AMDGPU_NUM_VMID];
static bool amdkfd_gfx11_last_trap_valid[AMDGPU_NUM_VMID];
/* Monotonic sequence incremented whenever trap regs are programmed for a VMID. */
static u64 amdkfd_gfx11_trap_prog_seq[AMDGPU_NUM_VMID];

static const char *kgd_gfx_v11_pcs_sqcmd_policy_name(int policy)
{
	switch (policy) {
	case AMDKFD_GFX11_PCS_SQCMD_SINGLE_Q0:
		return "single_q0";
	case AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP:
		return "single_queue_sweep";
	case AMDKFD_GFX11_PCS_SQCMD_BROADCAST:
		return "broadcast";
	case AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP_NO_VMID_CHECK:
		return "single_queue_sweep_no_vmid_check";
	case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_PROBE:
		return "broadcast_probe";
	case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE:
		return "broadcast_queue";
	case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_PROBE:
		return "broadcast_queue_probe";
	case AMDKFD_GFX11_PCS_SQCMD_SINGLE_FIXED_Q0_W0:
		return "single_fixed_q0_w0";
	case AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP_W0:
		return "single_queue_sweep_w0";
	case AMDKFD_GFX11_PCS_SQCMD_NOOP:
		return "noop";
	case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_NO_VMID:
		return "broadcast_queue_no_vmid";
	case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_NO_VMID_PROBE:
		return "broadcast_queue_no_vmid_probe";
	case AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE_NO_VMID:
		return "single_wave_no_queue_no_vmid";
	case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_WAVE_NO_VMID_PROBE:
		return "broadcast_queue_wave_no_vmid_probe";
	case AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE:
		return "single_wave_no_queue";
	case AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE_PROBE:
		return "single_wave_no_queue_probe";
	default:
		return "invalid";
	}
}

enum hqd_dequeue_request_type {
	NO_ACTION = 0,
	DRAIN_PIPE,
	RESET_WAVES,
	SAVE_WAVES
};

static void lock_srbm(struct amdgpu_device *adev, uint32_t mec, uint32_t pipe,
			uint32_t queue, uint32_t vmid)
{
	mutex_lock(&adev->srbm_mutex);
	soc21_grbm_select(adev, mec, pipe, queue, vmid);
}

static void unlock_srbm(struct amdgpu_device *adev)
{
	soc21_grbm_select(adev, 0, 0, 0, 0);
	mutex_unlock(&adev->srbm_mutex);
}

static void acquire_queue(struct amdgpu_device *adev, uint32_t pipe_id,
				uint32_t queue_id)
{
	uint32_t mec = (pipe_id / adev->gfx.mec.num_pipe_per_mec) + 1;
	uint32_t pipe = (pipe_id % adev->gfx.mec.num_pipe_per_mec);

	lock_srbm(adev, mec, pipe, queue_id, 0);
}

static uint64_t get_queue_mask(struct amdgpu_device *adev,
			       uint32_t pipe_id, uint32_t queue_id)
{
	unsigned int bit = pipe_id * adev->gfx.mec.num_queue_per_pipe +
			queue_id;

	return 1ull << bit;
}

static void release_queue(struct amdgpu_device *adev)
{
	unlock_srbm(adev);
}

static void program_sh_mem_settings_v11(struct amdgpu_device *adev, uint32_t vmid,
					uint32_t sh_mem_config,
					uint32_t sh_mem_ape1_base,
					uint32_t sh_mem_ape1_limit,
					uint32_t sh_mem_bases, uint32_t inst)
{
	lock_srbm(adev, 0, 0, 0, vmid);

	WREG32(SOC15_REG_OFFSET(GC, 0, regSH_MEM_CONFIG), sh_mem_config);
	WREG32(SOC15_REG_OFFSET(GC, 0, regSH_MEM_BASES), sh_mem_bases);

	unlock_srbm(adev);
}

static int set_pasid_vmid_mapping_v11(struct amdgpu_device *adev, unsigned int pasid,
					unsigned int vmid, uint32_t inst)
{
	uint32_t value = pasid << IH_VMID_0_LUT__PASID__SHIFT;

	/* Mapping vmid to pasid also for IH block */
	pr_debug("mapping vmid %d -> pasid %d in IH block for GFX client\n",
			vmid, pasid);
	WREG32(SOC15_REG_OFFSET(OSSSYS, 0, regIH_VMID_0_LUT) + vmid, value);

	return 0;
}

static bool get_atc_vmid_pasid_mapping_info_v11(struct amdgpu_device *adev,
					uint8_t vmid, uint16_t *p_pasid)
{
	if (!p_pasid)
		return false;

	*p_pasid = RREG32(SOC15_REG_OFFSET(OSSSYS, 0, regIH_VMID_0_LUT) + vmid) &
		   0xffff;

	return !!(*p_pasid);
}

static int init_interrupts_v11(struct amdgpu_device *adev, uint32_t pipe_id,
				uint32_t inst)
{
	uint32_t mec;
	uint32_t pipe;

	mec = (pipe_id / adev->gfx.mec.num_pipe_per_mec) + 1;
	pipe = (pipe_id % adev->gfx.mec.num_pipe_per_mec);

	lock_srbm(adev, mec, pipe, 0, 0);

	WREG32_SOC15(GC, 0, regCPC_INT_CNTL,
		CP_INT_CNTL_RING0__TIME_STAMP_INT_ENABLE_MASK |
		CP_INT_CNTL_RING0__OPCODE_ERROR_INT_ENABLE_MASK);

	unlock_srbm(adev);

	return 0;
}

static uint32_t get_sdma_rlc_reg_offset(struct amdgpu_device *adev,
				unsigned int engine_id,
				unsigned int queue_id)
{
	uint32_t sdma_engine_reg_base = 0;
	uint32_t sdma_rlc_reg_offset;

	switch (engine_id) {
	case 0:
		sdma_engine_reg_base = SOC15_REG_OFFSET(SDMA0, 0,
				regSDMA0_QUEUE0_RB_CNTL) - regSDMA0_QUEUE0_RB_CNTL;
		break;
	case 1:
		sdma_engine_reg_base = SOC15_REG_OFFSET(SDMA1, 0,
				regSDMA1_QUEUE0_RB_CNTL) - regSDMA0_QUEUE0_RB_CNTL;
		break;
	default:
		BUG();
	}

	sdma_rlc_reg_offset = sdma_engine_reg_base
		+ queue_id * (regSDMA0_QUEUE1_RB_CNTL - regSDMA0_QUEUE0_RB_CNTL);

	pr_debug("RLC register offset for SDMA%d RLC%d: 0x%x\n", engine_id,
			queue_id, sdma_rlc_reg_offset);

	return sdma_rlc_reg_offset;
}

static inline struct v11_compute_mqd *get_mqd(void *mqd)
{
	return (struct v11_compute_mqd *)mqd;
}

static inline struct v11_sdma_mqd *get_sdma_mqd(void *mqd)
{
	return (struct v11_sdma_mqd *)mqd;
}

static int hqd_load_v11(struct amdgpu_device *adev, void *mqd, uint32_t pipe_id,
			uint32_t queue_id, uint32_t __user *wptr,
			uint32_t wptr_shift, uint32_t wptr_mask,
			struct mm_struct *mm, uint32_t inst)
{
	struct v11_compute_mqd *m;
	uint32_t *mqd_hqd;
	uint32_t reg, hqd_base, data;

	m = get_mqd(mqd);

	pr_debug("Load hqd of pipe %d queue %d\n", pipe_id, queue_id);
	acquire_queue(adev, pipe_id, queue_id);

	/* HIQ is set during driver init period with vmid set to 0*/
	if (m->cp_hqd_vmid == 0) {
		uint32_t value, mec, pipe;

		mec = (pipe_id / adev->gfx.mec.num_pipe_per_mec) + 1;
		pipe = (pipe_id % adev->gfx.mec.num_pipe_per_mec);

		pr_debug("kfd: set HIQ, mec:%d, pipe:%d, queue:%d.\n",
			mec, pipe, queue_id);
		value = RREG32(SOC15_REG_OFFSET(GC, 0, regRLC_CP_SCHEDULERS));
		value = REG_SET_FIELD(value, RLC_CP_SCHEDULERS, scheduler1,
			((mec << 5) | (pipe << 3) | queue_id | 0x80));
		WREG32(SOC15_REG_OFFSET(GC, 0, regRLC_CP_SCHEDULERS), value);
	}

	/* HQD registers extend from CP_MQD_BASE_ADDR to CP_HQD_EOP_WPTR_MEM. */
	mqd_hqd = &m->cp_mqd_base_addr_lo;
	hqd_base = SOC15_REG_OFFSET(GC, 0, regCP_MQD_BASE_ADDR);

	for (reg = hqd_base;
	     reg <= SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_WPTR_HI); reg++)
		WREG32(reg, mqd_hqd[reg - hqd_base]);


	/* Activate doorbell logic before triggering WPTR poll. */
	data = REG_SET_FIELD(m->cp_hqd_pq_doorbell_control,
			     CP_HQD_PQ_DOORBELL_CONTROL, DOORBELL_EN, 1);
	WREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_DOORBELL_CONTROL), data);

	if (wptr) {
		/* Don't read wptr with get_user because the user
		 * context may not be accessible (if this function
		 * runs in a work queue). Instead trigger a one-shot
		 * polling read from memory in the CP. This assumes
		 * that wptr is GPU-accessible in the queue's VMID via
		 * ATC or SVM. WPTR==RPTR before starting the poll so
		 * the CP starts fetching new commands from the right
		 * place.
		 *
		 * Guessing a 64-bit WPTR from a 32-bit RPTR is a bit
		 * tricky. Assume that the queue didn't overflow. The
		 * number of valid bits in the 32-bit RPTR depends on
		 * the queue size. The remaining bits are taken from
		 * the saved 64-bit WPTR. If the WPTR wrapped, add the
		 * queue size.
		 */
		uint32_t queue_size =
			2 << REG_GET_FIELD(m->cp_hqd_pq_control,
					   CP_HQD_PQ_CONTROL, QUEUE_SIZE);
		uint64_t guessed_wptr = m->cp_hqd_pq_rptr & (queue_size - 1);

		if ((m->cp_hqd_pq_wptr_lo & (queue_size - 1)) < guessed_wptr)
			guessed_wptr += queue_size;
		guessed_wptr += m->cp_hqd_pq_wptr_lo & ~(queue_size - 1);
		guessed_wptr += (uint64_t)m->cp_hqd_pq_wptr_hi << 32;

		WREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_WPTR_LO),
		       lower_32_bits(guessed_wptr));
		WREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_WPTR_HI),
		       upper_32_bits(guessed_wptr));
		WREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_WPTR_POLL_ADDR),
		       lower_32_bits((uint64_t)wptr));
		WREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_WPTR_POLL_ADDR_HI),
		       upper_32_bits((uint64_t)wptr));
		pr_debug("%s setting CP_PQ_WPTR_POLL_CNTL1 to %x\n", __func__,
			 (uint32_t)get_queue_mask(adev, pipe_id, queue_id));
		WREG32(SOC15_REG_OFFSET(GC, 0, regCP_PQ_WPTR_POLL_CNTL1),
		       (uint32_t)get_queue_mask(adev, pipe_id, queue_id));
	}

	/* Start the EOP fetcher */
	WREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_EOP_RPTR),
	       REG_SET_FIELD(m->cp_hqd_eop_rptr,
			     CP_HQD_EOP_RPTR, INIT_FETCHER, 1));

	data = REG_SET_FIELD(m->cp_hqd_active, CP_HQD_ACTIVE, ACTIVE, 1);
	WREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_ACTIVE), data);

	release_queue(adev);

	return 0;
}

static int hiq_mqd_load_v11(struct amdgpu_device *adev, void *mqd,
			      uint32_t pipe_id, uint32_t queue_id,
			      uint32_t doorbell_off, uint32_t inst)
{
	struct amdgpu_ring *kiq_ring = &adev->gfx.kiq[0].ring;
	struct v11_compute_mqd *m;
	uint32_t mec, pipe;
	int r;

	m = get_mqd(mqd);

	acquire_queue(adev, pipe_id, queue_id);

	mec = (pipe_id / adev->gfx.mec.num_pipe_per_mec) + 1;
	pipe = (pipe_id % adev->gfx.mec.num_pipe_per_mec);

	pr_debug("kfd: set HIQ, mec:%d, pipe:%d, queue:%d.\n",
		 mec, pipe, queue_id);

	spin_lock(&adev->gfx.kiq[0].ring_lock);
	r = amdgpu_ring_alloc(kiq_ring, 7);
	if (r) {
		pr_err("Failed to alloc KIQ (%d).\n", r);
		goto out_unlock;
	}

	amdgpu_ring_write(kiq_ring, PACKET3(PACKET3_MAP_QUEUES, 5));
	amdgpu_ring_write(kiq_ring,
			  PACKET3_MAP_QUEUES_QUEUE_SEL(0) | /* Queue_Sel */
			  PACKET3_MAP_QUEUES_VMID(m->cp_hqd_vmid) | /* VMID */
			  PACKET3_MAP_QUEUES_QUEUE(queue_id) |
			  PACKET3_MAP_QUEUES_PIPE(pipe) |
			  PACKET3_MAP_QUEUES_ME((mec - 1)) |
			  PACKET3_MAP_QUEUES_QUEUE_TYPE(0) | /*queue_type: normal compute queue */
			  PACKET3_MAP_QUEUES_ALLOC_FORMAT(0) | /* alloc format: all_on_one_pipe */
			  PACKET3_MAP_QUEUES_ENGINE_SEL(1) | /* engine_sel: hiq */
			  PACKET3_MAP_QUEUES_NUM_QUEUES(1)); /* num_queues: must be 1 */
	amdgpu_ring_write(kiq_ring,
			PACKET3_MAP_QUEUES_DOORBELL_OFFSET(doorbell_off));
	amdgpu_ring_write(kiq_ring, m->cp_mqd_base_addr_lo);
	amdgpu_ring_write(kiq_ring, m->cp_mqd_base_addr_hi);
	amdgpu_ring_write(kiq_ring, m->cp_hqd_pq_wptr_poll_addr_lo);
	amdgpu_ring_write(kiq_ring, m->cp_hqd_pq_wptr_poll_addr_hi);
	amdgpu_ring_commit(kiq_ring);

out_unlock:
	spin_unlock(&adev->gfx.kiq[0].ring_lock);
	release_queue(adev);

	return r;
}

static int hqd_dump_v11(struct amdgpu_device *adev,
			uint32_t pipe_id, uint32_t queue_id,
			uint32_t (**dump)[2], uint32_t *n_regs, uint32_t inst)
{
	uint32_t i = 0, reg;
#define HQD_N_REGS 56
#define DUMP_REG(addr) do {				\
		if (WARN_ON_ONCE(i >= HQD_N_REGS))	\
			break;				\
		(*dump)[i][0] = (addr) << 2;		\
		(*dump)[i++][1] = RREG32(addr);		\
	} while (0)

	*dump = kmalloc_array(HQD_N_REGS, sizeof(**dump), GFP_KERNEL);
	if (*dump == NULL)
		return -ENOMEM;

	acquire_queue(adev, pipe_id, queue_id);

	for (reg = SOC15_REG_OFFSET(GC, 0, regCP_MQD_BASE_ADDR);
	     reg <= SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_WPTR_HI); reg++)
		DUMP_REG(reg);

	release_queue(adev);

	WARN_ON_ONCE(i != HQD_N_REGS);
	*n_regs = i;

	return 0;
}

static int hqd_sdma_load_v11(struct amdgpu_device *adev, void *mqd,
			     uint32_t __user *wptr, struct mm_struct *mm)
{
	struct v11_sdma_mqd *m;
	uint32_t sdma_rlc_reg_offset;
	unsigned long end_jiffies;
	uint32_t data;
	uint64_t data64;
	uint64_t __user *wptr64 = (uint64_t __user *)wptr;

	m = get_sdma_mqd(mqd);
	sdma_rlc_reg_offset = get_sdma_rlc_reg_offset(adev, m->sdma_engine_id,
					    m->sdma_queue_id);

	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_CNTL,
		m->sdmax_rlcx_rb_cntl & (~SDMA0_QUEUE0_RB_CNTL__RB_ENABLE_MASK));

	end_jiffies = msecs_to_jiffies(2000) + jiffies;
	while (true) {
		data = RREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_CONTEXT_STATUS);
		if (data & SDMA0_QUEUE0_CONTEXT_STATUS__IDLE_MASK)
			break;
		if (time_after(jiffies, end_jiffies)) {
			pr_err("SDMA RLC not idle in %s\n", __func__);
			return -ETIME;
		}
		usleep_range(500, 1000);
	}

	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_DOORBELL_OFFSET,
	       m->sdmax_rlcx_doorbell_offset);

	data = REG_SET_FIELD(m->sdmax_rlcx_doorbell, SDMA0_QUEUE0_DOORBELL,
			     ENABLE, 1);
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_DOORBELL, data);
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_RPTR,
				m->sdmax_rlcx_rb_rptr);
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_RPTR_HI,
				m->sdmax_rlcx_rb_rptr_hi);

	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_MINOR_PTR_UPDATE, 1);
	if (read_user_wptr(mm, wptr64, data64)) {
		WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_WPTR,
		       lower_32_bits(data64));
		WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_WPTR_HI,
		       upper_32_bits(data64));
	} else {
		WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_WPTR,
		       m->sdmax_rlcx_rb_rptr);
		WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_WPTR_HI,
		       m->sdmax_rlcx_rb_rptr_hi);
	}
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_MINOR_PTR_UPDATE, 0);

	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_BASE, m->sdmax_rlcx_rb_base);
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_BASE_HI,
			m->sdmax_rlcx_rb_base_hi);
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_RPTR_ADDR_LO,
			m->sdmax_rlcx_rb_rptr_addr_lo);
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_RPTR_ADDR_HI,
			m->sdmax_rlcx_rb_rptr_addr_hi);

	data = REG_SET_FIELD(m->sdmax_rlcx_rb_cntl, SDMA0_QUEUE0_RB_CNTL,
			     RB_ENABLE, 1);
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_CNTL, data);

	return 0;
}

static int hqd_sdma_dump_v11(struct amdgpu_device *adev,
			     uint32_t engine_id, uint32_t queue_id,
			     uint32_t (**dump)[2], uint32_t *n_regs)
{
	uint32_t sdma_rlc_reg_offset = get_sdma_rlc_reg_offset(adev,
			engine_id, queue_id);
	uint32_t i = 0, reg;
#undef HQD_N_REGS
#define HQD_N_REGS (7+11+1+12+12)

	*dump = kmalloc_array(HQD_N_REGS, sizeof(**dump), GFP_KERNEL);
	if (*dump == NULL)
		return -ENOMEM;

	for (reg = regSDMA0_QUEUE0_RB_CNTL;
	     reg <= regSDMA0_QUEUE0_RB_WPTR_HI; reg++)
		DUMP_REG(sdma_rlc_reg_offset + reg);
	for (reg = regSDMA0_QUEUE0_RB_RPTR_ADDR_HI;
	     reg <= regSDMA0_QUEUE0_DOORBELL; reg++)
		DUMP_REG(sdma_rlc_reg_offset + reg);
	for (reg = regSDMA0_QUEUE0_DOORBELL_LOG;
	     reg <= regSDMA0_QUEUE0_DOORBELL_LOG; reg++)
		DUMP_REG(sdma_rlc_reg_offset + reg);
	for (reg = regSDMA0_QUEUE0_DOORBELL_OFFSET;
	     reg <= regSDMA0_QUEUE0_RB_PREEMPT; reg++)
		DUMP_REG(sdma_rlc_reg_offset + reg);
	for (reg = regSDMA0_QUEUE0_MIDCMD_DATA0;
	     reg <= regSDMA0_QUEUE0_MIDCMD_CNTL; reg++)
		DUMP_REG(sdma_rlc_reg_offset + reg);

	WARN_ON_ONCE(i != HQD_N_REGS);
	*n_regs = i;

	return 0;
}

static bool hqd_is_occupied_v11(struct amdgpu_device *adev, uint64_t queue_address,
				uint32_t pipe_id, uint32_t queue_id, uint32_t inst)
{
	uint32_t act;
	bool retval = false;
	uint32_t low, high;

	acquire_queue(adev, pipe_id, queue_id);
	act = RREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_ACTIVE));
	if (act) {
		low = lower_32_bits(queue_address >> 8);
		high = upper_32_bits(queue_address >> 8);

		if (low == RREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_BASE)) &&
		   high == RREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_PQ_BASE_HI)))
			retval = true;
	}
	release_queue(adev);
	return retval;
}

static bool hqd_sdma_is_occupied_v11(struct amdgpu_device *adev, void *mqd)
{
	struct v11_sdma_mqd *m;
	uint32_t sdma_rlc_reg_offset;
	uint32_t sdma_rlc_rb_cntl;

	m = get_sdma_mqd(mqd);
	sdma_rlc_reg_offset = get_sdma_rlc_reg_offset(adev, m->sdma_engine_id,
					    m->sdma_queue_id);

	sdma_rlc_rb_cntl = RREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_CNTL);

	if (sdma_rlc_rb_cntl & SDMA0_QUEUE0_RB_CNTL__RB_ENABLE_MASK)
		return true;

	return false;
}

static int hqd_destroy_v11(struct amdgpu_device *adev, void *mqd,
				enum kfd_preempt_type reset_type,
				unsigned int utimeout, uint32_t pipe_id,
				uint32_t queue_id, uint32_t inst)
{
	enum hqd_dequeue_request_type type;
	unsigned long end_jiffies;
	uint32_t temp;
	struct v11_compute_mqd *m = get_mqd(mqd);

	acquire_queue(adev, pipe_id, queue_id);

	if (m->cp_hqd_vmid == 0)
		WREG32_FIELD15_PREREG(GC, 0, RLC_CP_SCHEDULERS, scheduler1, 0);

	switch (reset_type) {
	case KFD_PREEMPT_TYPE_WAVEFRONT_DRAIN:
		type = DRAIN_PIPE;
		break;
	case KFD_PREEMPT_TYPE_WAVEFRONT_RESET:
		type = RESET_WAVES;
		break;
	default:
		type = DRAIN_PIPE;
		break;
	}

	WREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_DEQUEUE_REQUEST), type);

	end_jiffies = (utimeout * HZ / 1000) + jiffies;
	while (true) {
		temp = RREG32(SOC15_REG_OFFSET(GC, 0, regCP_HQD_ACTIVE));
		if (!(temp & CP_HQD_ACTIVE__ACTIVE_MASK))
			break;
		if (time_after(jiffies, end_jiffies)) {
			pr_err("cp queue pipe %d queue %d preemption failed\n",
					pipe_id, queue_id);
			release_queue(adev);
			return -ETIME;
		}
		usleep_range(500, 1000);
	}

	release_queue(adev);
	return 0;
}

static int hqd_sdma_destroy_v11(struct amdgpu_device *adev, void *mqd,
				unsigned int utimeout)
{
	struct v11_sdma_mqd *m;
	uint32_t sdma_rlc_reg_offset;
	uint32_t temp;
	unsigned long end_jiffies = (utimeout * HZ / 1000) + jiffies;

	m = get_sdma_mqd(mqd);
	sdma_rlc_reg_offset = get_sdma_rlc_reg_offset(adev, m->sdma_engine_id,
					    m->sdma_queue_id);

	temp = RREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_CNTL);
	temp = temp & ~SDMA0_QUEUE0_RB_CNTL__RB_ENABLE_MASK;
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_CNTL, temp);

	while (true) {
		temp = RREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_CONTEXT_STATUS);
		if (temp & SDMA0_QUEUE0_CONTEXT_STATUS__IDLE_MASK)
			break;
		if (time_after(jiffies, end_jiffies)) {
			pr_err("SDMA RLC not idle in %s\n", __func__);
			return -ETIME;
		}
		usleep_range(500, 1000);
	}

	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_DOORBELL, 0);
	WREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_CNTL,
		RREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_CNTL) |
		SDMA0_QUEUE0_RB_CNTL__RB_ENABLE_MASK);

	m->sdmax_rlcx_rb_rptr = RREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_RPTR);
	m->sdmax_rlcx_rb_rptr_hi =
		RREG32(sdma_rlc_reg_offset + regSDMA0_QUEUE0_RB_RPTR_HI);

	return 0;
}

static int wave_control_execute_v11(struct amdgpu_device *adev,
					uint32_t gfx_index_val,
					uint32_t sq_cmd, uint32_t inst)
{
	uint32_t data = 0;

	mutex_lock(&adev->grbm_idx_mutex);

	WREG32(SOC15_REG_OFFSET(GC, 0, regGRBM_GFX_INDEX), gfx_index_val);
	WREG32(SOC15_REG_OFFSET(GC, 0, regSQ_CMD), sq_cmd);

	data = REG_SET_FIELD(data, GRBM_GFX_INDEX,
		INSTANCE_BROADCAST_WRITES, 1);
	data = REG_SET_FIELD(data, GRBM_GFX_INDEX,
		SA_BROADCAST_WRITES, 1);
	data = REG_SET_FIELD(data, GRBM_GFX_INDEX,
		SE_BROADCAST_WRITES, 1);

	WREG32(SOC15_REG_OFFSET(GC, 0, regGRBM_GFX_INDEX), data);
	mutex_unlock(&adev->grbm_idx_mutex);

	return 0;
}

static void set_vm_context_page_table_base_v11(struct amdgpu_device *adev,
		uint32_t vmid, uint64_t page_table_base)
{
	if (!amdgpu_amdkfd_is_kfd_vmid(adev, vmid)) {
		pr_err("trying to set page table base for wrong VMID %u\n",
		       vmid);
		return;
	}

	/* SDMA is on gfxhub as well for gfx11 adapters */
	adev->gfxhub.funcs->setup_vm_pt_regs(adev, vmid, page_table_base);
}

/*
 * Returns TRAP_EN, EXCP_EN and EXCP_REPLACE.
 *
 * restore_dbg_registers is ignored here but is a general interface requirement
 * for devices that support GFXOFF and where the RLC save/restore list
 * does not support hw registers for debugging i.e. the driver has to manually
 * initialize the debug mode registers after it has disabled GFX off during the
 * debug session.
 */
static uint32_t kgd_gfx_v11_enable_debug_trap(struct amdgpu_device *adev,
					    bool restore_dbg_registers,
					    uint32_t vmid)
{
	uint32_t data = 0;

	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, TRAP_EN, 1);
	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, EXCP_EN, 0);
	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, EXCP_REPLACE, 0);

	return data;
}

/* Returns TRAP_EN, EXCP_EN and EXCP_REPLACE. */
static uint32_t kgd_gfx_v11_disable_debug_trap(struct amdgpu_device *adev,
						bool keep_trap_enabled,
						uint32_t vmid)
{
	uint32_t data = 0;

	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, TRAP_EN, 1);
	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, EXCP_EN, 0);
	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, EXCP_REPLACE, 0);

	return data;
}

static int kgd_gfx_v11_validate_trap_override_request(struct amdgpu_device *adev,
							uint32_t trap_override,
							uint32_t *trap_mask_supported)
{
	*trap_mask_supported &= KFD_DBG_TRAP_MASK_FP_INVALID |
				KFD_DBG_TRAP_MASK_FP_INPUT_DENORMAL |
				KFD_DBG_TRAP_MASK_FP_DIVIDE_BY_ZERO |
				KFD_DBG_TRAP_MASK_FP_OVERFLOW |
				KFD_DBG_TRAP_MASK_FP_UNDERFLOW |
				KFD_DBG_TRAP_MASK_FP_INEXACT |
				KFD_DBG_TRAP_MASK_INT_DIVIDE_BY_ZERO |
				KFD_DBG_TRAP_MASK_DBG_ADDRESS_WATCH |
				KFD_DBG_TRAP_MASK_DBG_MEMORY_VIOLATION;

	if (amdgpu_ip_version(adev, GC_HWIP, 0) >= IP_VERSION(11, 0, 4))
		*trap_mask_supported |= KFD_DBG_TRAP_MASK_TRAP_ON_WAVE_START |
					KFD_DBG_TRAP_MASK_TRAP_ON_WAVE_END;

	if (trap_override != KFD_DBG_TRAP_OVERRIDE_OR &&
			trap_override != KFD_DBG_TRAP_OVERRIDE_REPLACE)
		return -EPERM;

	return 0;
}

static uint32_t trap_mask_map_sw_to_hw(uint32_t mask)
{
	uint32_t trap_on_start = (mask & KFD_DBG_TRAP_MASK_TRAP_ON_WAVE_START) ? 1 : 0;
	uint32_t trap_on_end = (mask & KFD_DBG_TRAP_MASK_TRAP_ON_WAVE_END) ? 1 : 0;
	uint32_t excp_en = mask & (KFD_DBG_TRAP_MASK_FP_INVALID |
			KFD_DBG_TRAP_MASK_FP_INPUT_DENORMAL |
			KFD_DBG_TRAP_MASK_FP_DIVIDE_BY_ZERO |
			KFD_DBG_TRAP_MASK_FP_OVERFLOW |
			KFD_DBG_TRAP_MASK_FP_UNDERFLOW |
			KFD_DBG_TRAP_MASK_FP_INEXACT |
			KFD_DBG_TRAP_MASK_INT_DIVIDE_BY_ZERO |
			KFD_DBG_TRAP_MASK_DBG_ADDRESS_WATCH |
			KFD_DBG_TRAP_MASK_DBG_MEMORY_VIOLATION);
	uint32_t ret;

	ret = REG_SET_FIELD(0, SPI_GDBG_PER_VMID_CNTL, EXCP_EN, excp_en);
	ret = REG_SET_FIELD(ret, SPI_GDBG_PER_VMID_CNTL, TRAP_ON_START, trap_on_start);
	ret = REG_SET_FIELD(ret, SPI_GDBG_PER_VMID_CNTL, TRAP_ON_END, trap_on_end);

	return ret;
}

static uint32_t trap_mask_map_hw_to_sw(uint32_t mask)
{
	uint32_t ret = REG_GET_FIELD(mask, SPI_GDBG_PER_VMID_CNTL, EXCP_EN);

	if (REG_GET_FIELD(mask, SPI_GDBG_PER_VMID_CNTL, TRAP_ON_START))
		ret |= KFD_DBG_TRAP_MASK_TRAP_ON_WAVE_START;

	if (REG_GET_FIELD(mask, SPI_GDBG_PER_VMID_CNTL, TRAP_ON_END))
		ret |= KFD_DBG_TRAP_MASK_TRAP_ON_WAVE_END;

	return ret;
}

/* Returns TRAP_EN, EXCP_EN and EXCP_REPLACE. */
static uint32_t kgd_gfx_v11_set_wave_launch_trap_override(struct amdgpu_device *adev,
					uint32_t vmid,
					uint32_t trap_override,
					uint32_t trap_mask_bits,
					uint32_t trap_mask_request,
					uint32_t *trap_mask_prev,
					uint32_t kfd_dbg_trap_cntl_prev)
{
	uint32_t data = 0;

	*trap_mask_prev = trap_mask_map_hw_to_sw(kfd_dbg_trap_cntl_prev);

	data = (trap_mask_bits & trap_mask_request) | (*trap_mask_prev & ~trap_mask_request);
	data = trap_mask_map_sw_to_hw(data);

	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, TRAP_EN, 1);
	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, EXCP_REPLACE, trap_override);

	return data;
}

static uint32_t kgd_gfx_v11_set_wave_launch_mode(struct amdgpu_device *adev,
					uint8_t wave_launch_mode,
					uint32_t vmid)
{
	uint32_t data = 0;

	data = REG_SET_FIELD(data, SPI_GDBG_PER_VMID_CNTL, LAUNCH_MODE, wave_launch_mode);

	return data;
}

#define TCP_WATCH_STRIDE (regTCP_WATCH1_ADDR_H - regTCP_WATCH0_ADDR_H)
static uint32_t kgd_gfx_v11_set_address_watch(struct amdgpu_device *adev,
					uint64_t watch_address,
					uint32_t watch_address_mask,
					uint32_t watch_id,
					uint32_t watch_mode,
					uint32_t debug_vmid,
					uint32_t inst)
{
	uint32_t watch_address_high;
	uint32_t watch_address_low;
	uint32_t watch_address_cntl;

	watch_address_cntl = 0;
	watch_address_low = lower_32_bits(watch_address);
	watch_address_high = upper_32_bits(watch_address) & 0xffff;

	watch_address_cntl = REG_SET_FIELD(watch_address_cntl,
			TCP_WATCH0_CNTL,
			MODE,
			watch_mode);

	watch_address_cntl = REG_SET_FIELD(watch_address_cntl,
			TCP_WATCH0_CNTL,
			MASK,
			watch_address_mask >> 7);

	watch_address_cntl = REG_SET_FIELD(watch_address_cntl,
			TCP_WATCH0_CNTL,
			VALID,
			1);

	WREG32_RLC((SOC15_REG_OFFSET(GC, 0, regTCP_WATCH0_ADDR_H) +
			(watch_id * TCP_WATCH_STRIDE)),
			watch_address_high);

	WREG32_RLC((SOC15_REG_OFFSET(GC, 0, regTCP_WATCH0_ADDR_L) +
			(watch_id * TCP_WATCH_STRIDE)),
			watch_address_low);

	return watch_address_cntl;
}

static uint32_t kgd_gfx_v11_clear_address_watch(struct amdgpu_device *adev,
						uint32_t watch_id)
{
	return 0;
}

static uint64_t kgd_gfx_v11_hqd_get_pq_addr(struct amdgpu_device *adev,
					    uint32_t pipe_id, uint32_t queue_id,
					    uint32_t inst)
{
	return 0;
}

static uint64_t kgd_gfx_v11_hqd_reset(struct amdgpu_device *adev,
				      uint32_t pipe_id, uint32_t queue_id,
				      uint32_t inst, unsigned int utimeout)
{
	return 0;
}

static uint32_t kgd_gfx_v11_hqd_sdma_get_doorbell(struct amdgpu_device *adev,
						  int engine, int queue)
{
	return 0;
}

static uint32_t kgd_gfx_v11_get_hosttrap_status(struct amdgpu_device *adev,
		uint32_t inst, int *se_idx, int *sh_idx)
{
	uint32_t sq_debug_hosttrap_status = 0x0;
	int i, j;

	if (se_idx)
		*se_idx = -1;
	if (sh_idx)
		*sh_idx = -1;

	mutex_lock(&adev->grbm_idx_mutex);
	for (i = 0; i < adev->gfx.config.max_shader_engines; i++) {
		for (j = 0; j < adev->gfx.config.max_sh_per_se; j++) {
			amdgpu_gfx_select_se_sh(adev, i, j, 0xffffffff, inst);
			sq_debug_hosttrap_status =
				RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_DEBUG_HOST_TRAP_STATUS);

			if (sq_debug_hosttrap_status) {
				if (se_idx)
					*se_idx = i;
				if (sh_idx)
					*sh_idx = j;
				goto out;
			}
		}
	}

out:
	amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
	mutex_unlock(&adev->grbm_idx_mutex);

	return sq_debug_hosttrap_status;
}

static void kgd_gfx_v11_log_hosttrap_status_matrix(struct amdgpu_device *adev,
		uint32_t inst, uint32_t vmid)
{
	int se, sh;
	uint32_t raw;

	mutex_lock(&adev->grbm_idx_mutex);
	for (se = 0; se < adev->gfx.config.max_shader_engines; se++) {
		for (sh = 0; sh < adev->gfx.config.max_sh_per_se; sh++) {
			amdgpu_gfx_select_se_sh(adev, se, sh, 0xffffffff, inst);
			raw = RREG32_SOC15(GC, GET_INST(GC, inst),
					   regSQ_DEBUG_HOST_TRAP_STATUS);
			if (!raw)
				continue;
			dev_info(adev->dev,
				 "trigger_pc_sample_trap: status_matrix vmid=%u se=%d sh=%d raw=0x%x pending_count=%u\n",
				 vmid, se, sh, raw,
				 (unsigned int)(raw & SQ_DEBUG_HOST_TRAP_STATUS__PENDING_COUNT_MASK));
		}
	}
	amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
	mutex_unlock(&adev->grbm_idx_mutex);
}

static uint32_t kgd_gfx_v11_wave_read_ind(struct amdgpu_device *adev,
		uint32_t inst, uint32_t wave, uint32_t address)
{
	WREG32_SOC15(GC, GET_INST(GC, inst), regSQ_IND_INDEX,
		(wave << SQ_IND_INDEX__WAVE_ID__SHIFT) |
		(address << SQ_IND_INDEX__INDEX__SHIFT));
	return RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_IND_DATA);
}

static void kgd_gfx_v11_dump_wave_trap_state(struct amdgpu_device *adev,
		uint32_t inst, int se_idx, int sh_idx, uint32_t vmid)
{
	uint32_t trapsts, ib_dbg1, status, hw_id1, hw_id2, mode;
	uint32_t pc_lo, pc_hi;
	uint32_t ttmp0, ttmp1, ttmp14, ttmp15;
	uint32_t wave_vmid = 0xFFFFFFFF;
	int wave;
	int printed = 0;

	if (se_idx < 0 || sh_idx < 0)
		return;

	mutex_lock(&adev->grbm_idx_mutex);
	amdgpu_gfx_select_se_sh(adev, se_idx, sh_idx, 0xFFFFFFFF, inst);

	for (wave = 0; wave < 32; wave++) {
		trapsts = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_TRAPSTS);
		ib_dbg1 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_IB_DBG1);
		status = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_STATUS);
		hw_id1 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_HW_ID1);
		hw_id2 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_HW_ID2);
		mode = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_MODE);
		pc_lo = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_PC_LO);
		pc_hi = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_PC_HI);
		ttmp0 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_TTMP0);
		ttmp1 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_TTMP1);
		ttmp14 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_TTMP14);
		ttmp15 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_TTMP15);
		if (hw_id2 != 0xbebebeef) {
			wave_vmid = FIELD_GET(SQ_WAVE_HW_ID2__VM_ID_MASK, hw_id2);
			if (wave_vmid != vmid)
				continue;
		} else {
			wave_vmid = 0xFFFFFFFF;
		}

		dev_info(adev->dev,
			 "trigger_pc_sample_trap: wave_dump vmid=%u se=%d sh=%d wave=%d trapsts=0x%x host_trap=%u status=0x%x(valid=%u trap_en=%u trap=%u idle=%u) mode=0x%x pc_hi=0x%x(ht_bit24=%u trap_id23_16=0x%x ht_bit7=%u trap_id15_8=0x%x) pc_lo=0x%x ttmp1=0x%x(ht=%u trap_id=0x%x) ttmp0=0x%x ttmp14=0x%x ttmp15=0x%x hw_id1=0x%x hw_id2=0x%x(wave_vmid=%u) ib_dbg1=0x%x wave_idle=%u\n",
			 vmid, se_idx, sh_idx, wave, trapsts,
			 !!(trapsts & SQ_WAVE_TRAPSTS__HOST_TRAP_MASK),
			 status,
			 !!(status & SQ_WAVE_STATUS__VALID_MASK),
			 !!(status & SQ_WAVE_STATUS__TRAP_EN_MASK),
			 !!(status & SQ_WAVE_STATUS__TRAP_MASK),
			 !!(status & SQ_WAVE_STATUS__IDLE_MASK),
			 mode,
			 pc_hi,
			 !!(pc_hi & BIT(24)),
			 ((pc_hi >> 16) & 0xff),
			 !!(pc_hi & BIT(7)),
			 ((pc_hi >> 8) & 0xff),
			 pc_lo,
			 ttmp1,
			 !!(ttmp1 & BIT(24)),
			 ((ttmp1 >> 16) & 0xff),
			 ttmp0, ttmp14, ttmp15,
			 hw_id1, hw_id2, wave_vmid, ib_dbg1,
			 !!(ib_dbg1 & SQ_WAVE_IB_DBG1__WAVE_IDLE_MASK));
		if (status == 0xbebebeef &&
		    hw_id1 == 0xbebebeef &&
		    hw_id2 == 0xbebebeef &&
		    ib_dbg1 == 0xbebebeef)
			dev_info(adev->dev,
				 "trigger_pc_sample_trap: wave_dump vmid=%u se=%d sh=%d wave=%d has sentinel pattern (SQ_IND_DATA) including HW_ID2; wave selector/context may be invalid/stale\n",
				 vmid, se_idx, sh_idx, wave);
		printed++;
	}

	if (!printed)
		dev_info(adev->dev,
			 "trigger_pc_sample_trap: wave_dump vmid=%u se=%d sh=%d no active/nonzero waves\n",
			 vmid, se_idx, sh_idx);

	amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
	mutex_unlock(&adev->grbm_idx_mutex);
}

static void kgd_gfx_v11_log_wave_presence_summary(struct amdgpu_device *adev,
		uint32_t inst, uint32_t target_vmid)
{
	int vmid, se, sh, wave;
	int max_waves = amdkfd_gfx11_pcs_wave_scan_max_waves;

	if (max_waves < 1)
		max_waves = 1;
	if (max_waves > 32)
		max_waves = 32;

	mutex_lock(&adev->grbm_idx_mutex);
	for (vmid = 0; vmid < 16; vmid++) {
		uint32_t valid_count = 0;
		uint32_t trap_en_count = 0;
		uint32_t trap_count = 0;
		uint32_t host_trap_count = 0;
		uint32_t nonzero_status_count = 0;
		uint32_t sentinel_count = 0;

		for (se = 0; se < adev->gfx.config.max_shader_engines; se++) {
			for (sh = 0; sh < adev->gfx.config.max_sh_per_se; sh++) {
				amdgpu_gfx_select_se_sh(adev, se, sh, vmid, inst);
				for (wave = 0; wave < max_waves; wave++) {
					uint32_t status = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
						ixSQ_WAVE_STATUS);

					if (status == 0xbebebeef) {
						sentinel_count++;
						continue;
					}

					if (status)
						nonzero_status_count++;
					if (status & SQ_WAVE_STATUS__VALID_MASK) {
						uint32_t trapsts = kgd_gfx_v11_wave_read_ind(adev, inst,
							wave, ixSQ_WAVE_TRAPSTS);
						valid_count++;
						if (status & SQ_WAVE_STATUS__TRAP_EN_MASK)
							trap_en_count++;
						if (status & SQ_WAVE_STATUS__TRAP_MASK)
							trap_count++;
						if (trapsts != 0xbebebeef &&
						    (trapsts & SQ_WAVE_TRAPSTS__HOST_TRAP_MASK))
							host_trap_count++;
					}
				}
			}
		}

		if (vmid == target_vmid || valid_count || trap_en_count ||
		    trap_count || host_trap_count || nonzero_status_count) {
			dev_info(adev->dev,
				 "trigger_pc_sample_trap: wave_scan vmid=%d%s valid=%u trap_en=%u trap=%u host_trap=%u nonzero_status=%u sentinel=%u (sampled %d waves/SH)\n",
				 vmid, vmid == target_vmid ? " (target)" : "",
				 valid_count, trap_en_count, trap_count, host_trap_count,
				 nonzero_status_count, sentinel_count, max_waves);
		}
	}
	amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
	mutex_unlock(&adev->grbm_idx_mutex);
}

static void kgd_gfx_v11_log_target_vmid_wave_masks(struct amdgpu_device *adev,
		uint32_t inst, uint32_t vmid)
{
	int se, sh, wave;
	int max_waves = amdkfd_gfx11_pcs_wave_scan_max_waves;

	if (max_waves < 1)
		max_waves = 1;
	if (max_waves > 32)
		max_waves = 32;

	mutex_lock(&adev->grbm_idx_mutex);
	for (se = 0; se < adev->gfx.config.max_shader_engines; se++) {
		for (sh = 0; sh < adev->gfx.config.max_sh_per_se; sh++) {
			uint32_t valid_mask = 0;
			uint32_t trap_en_mask = 0;
			uint32_t trap_mask = 0;
			uint32_t host_mask = 0;
			uint32_t hw_wave_mask = 0;
			uint32_t hw_simd_mask = 0;
			uint32_t nonzero_status = 0;
			uint32_t sentinel = 0;

			amdgpu_gfx_select_se_sh(adev, se, sh, 0xFFFFFFFF, inst);
			for (wave = 0; wave < max_waves; wave++) {
				uint32_t status = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
									 ixSQ_WAVE_STATUS);
				uint32_t hw_id2;
				uint32_t wave_vmid;

				if (status == 0xbebebeef) {
					sentinel++;
					continue;
				}
				if (!status)
					continue;

				nonzero_status++;
				if (status & SQ_WAVE_STATUS__VALID_MASK) {
					uint32_t trapsts;
					uint32_t hw_id1;
					uint32_t hw_wave_id;
					uint32_t hw_simd_id;

					hw_id2 = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
									  ixSQ_WAVE_HW_ID2);
					if (hw_id2 == 0xbebebeef) {
						sentinel++;
						continue;
					}
					wave_vmid = FIELD_GET(SQ_WAVE_HW_ID2__VM_ID_MASK, hw_id2);
					if (wave_vmid != vmid)
						continue;

					valid_mask |= BIT(wave);
					if (status & SQ_WAVE_STATUS__TRAP_EN_MASK)
						trap_en_mask |= BIT(wave);
					if (status & SQ_WAVE_STATUS__TRAP_MASK)
						trap_mask |= BIT(wave);

					trapsts = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
									 ixSQ_WAVE_TRAPSTS);
					if (trapsts != 0xbebebeef &&
					    (trapsts & SQ_WAVE_TRAPSTS__HOST_TRAP_MASK))
						host_mask |= BIT(wave);

					hw_id1 = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
									  ixSQ_WAVE_HW_ID1);
					if (hw_id1 != 0xbebebeef) {
						hw_wave_id = FIELD_GET(SQ_WAVE_HW_ID1__WAVE_ID_MASK, hw_id1);
						hw_simd_id = FIELD_GET(SQ_WAVE_HW_ID1__SIMD_ID_MASK, hw_id1);
						hw_wave_mask |= BIT(hw_wave_id & 0x1f);
						hw_simd_mask |= BIT(hw_simd_id & 0x3);
					}
				}
			}

			if (nonzero_status || sentinel) {
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: wave_mask vmid=%u se=%d sh=%d valid=0x%08x trap_en=0x%08x trap=0x%08x host=0x%08x hw_wave=0x%08x hw_simd=0x%08x nonzero=%u sentinel=%u\n",
					 vmid, se, sh, valid_mask, trap_en_mask, trap_mask, host_mask,
					 hw_wave_mask, hw_simd_mask, nonzero_status, sentinel);
			}
		}
	}
	amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
	mutex_unlock(&adev->grbm_idx_mutex);
}

static void kgd_gfx_v11_log_selected_wave_visibility(struct amdgpu_device *adev,
		uint32_t inst, uint32_t vmid, uint32_t expected_wave_id, uint32_t expected_simd_id)
{
	int se, sh, wave;
	int max_waves = amdkfd_gfx11_pcs_wave_scan_max_waves;
	uint32_t idx_match_valid = 0;
	uint32_t idx_match_trap_en = 0;
	uint32_t hw_match_valid = 0;
	uint32_t hw_match_trap_en = 0;
	uint32_t hw_match_expected_simd = 0;
	uint32_t hw_match_simd_mask = 0;

	if (expected_wave_id > 31)
		return;
	if (max_waves < 1)
		max_waves = 1;
	if (max_waves > 32)
		max_waves = 32;

	mutex_lock(&adev->grbm_idx_mutex);
	for (se = 0; se < adev->gfx.config.max_shader_engines; se++) {
		for (sh = 0; sh < adev->gfx.config.max_sh_per_se; sh++) {
			amdgpu_gfx_select_se_sh(adev, se, sh, 0xFFFFFFFF, inst);
			for (wave = 0; wave < max_waves; wave++) {
				uint32_t status = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
									 ixSQ_WAVE_STATUS);
				uint32_t hw_id2;
				uint32_t wave_vmid;

				if (status == 0xbebebeef || !(status & SQ_WAVE_STATUS__VALID_MASK))
					continue;

				hw_id2 = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
								 ixSQ_WAVE_HW_ID2);
				if (hw_id2 == 0xbebebeef)
					continue;
				wave_vmid = FIELD_GET(SQ_WAVE_HW_ID2__VM_ID_MASK, hw_id2);
				if (wave_vmid != vmid)
					continue;

				if (wave == expected_wave_id) {
					idx_match_valid++;
					if (status & SQ_WAVE_STATUS__TRAP_EN_MASK)
						idx_match_trap_en++;
				}

				{
					uint32_t hw_id1 = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
										   ixSQ_WAVE_HW_ID1);
					if (hw_id1 != 0xbebebeef) {
						uint32_t hw_wave_id =
							FIELD_GET(SQ_WAVE_HW_ID1__WAVE_ID_MASK, hw_id1);
						uint32_t hw_simd_id =
							FIELD_GET(SQ_WAVE_HW_ID1__SIMD_ID_MASK, hw_id1);
						if (hw_wave_id == expected_wave_id) {
							hw_match_valid++;
							hw_match_simd_mask |= BIT(hw_simd_id & 0x3);
							if (status & SQ_WAVE_STATUS__TRAP_EN_MASK)
								hw_match_trap_en++;
							if ((expected_simd_id & 0x3) == (hw_simd_id & 0x3))
								hw_match_expected_simd++;
						}
					}
				}
			}
		}
	}
	amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
	mutex_unlock(&adev->grbm_idx_mutex);

	dev_info(adev->dev,
		 "trigger_pc_sample_trap: wave_target vmid=%u wave_id=%u expected_simd=%u idx_match_valid=%u idx_match_trap_en=%u hw_match_valid=%u hw_match_trap_en=%u hw_match_expected_simd=%u hw_match_simd_mask=0x%x\n",
		 vmid, expected_wave_id, expected_simd_id & 0x3, idx_match_valid,
		 idx_match_trap_en, hw_match_valid, hw_match_trap_en,
		 hw_match_expected_simd, hw_match_simd_mask);
}

/*
 * Snapshot active waves using vmid wildcard selection so we can verify whether
 * the target VMID has runnable trap-enabled waves at trigger time.
 */
static void kgd_gfx_v11_log_global_wave_vmid_queue_summary(struct amdgpu_device *adev,
		uint32_t inst, uint32_t target_vmid)
{
	int se, sh, wave;
	int max_waves = amdkfd_gfx11_pcs_wave_scan_max_waves;
	uint32_t total_nonzero_status = 0;
	uint32_t total_valid = 0;
	uint32_t target_valid = 0;
	uint32_t target_trap_en = 0;
	uint32_t target_trap = 0;
	uint32_t target_host_trap = 0;
	uint32_t sentinel_count = 0;
	uint32_t queue_mask = 0;
	uint32_t pipe_mask = 0;
	uint32_t me_mask = 0;
	uint32_t simd_mask = 0;
	uint32_t wave_mask = 0;
	int detail_budget = 8;

	if (max_waves < 1)
		max_waves = 1;
	if (max_waves > 32)
		max_waves = 32;

	mutex_lock(&adev->grbm_idx_mutex);
	for (se = 0; se < adev->gfx.config.max_shader_engines; se++) {
		for (sh = 0; sh < adev->gfx.config.max_sh_per_se; sh++) {
			amdgpu_gfx_select_se_sh(adev, se, sh, 0xFFFFFFFF, inst);
			for (wave = 0; wave < max_waves; wave++) {
				uint32_t status = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
									 ixSQ_WAVE_STATUS);
				uint32_t hw_id1, hw_id2, trapsts;
				uint32_t vmid, queue_id, pipe_id, me_id;
				uint32_t simd_id, hw_wave_id;

				if (status == 0xbebebeef) {
					sentinel_count++;
					continue;
				}
				if (!status)
					continue;

				total_nonzero_status++;
				if (!(status & SQ_WAVE_STATUS__VALID_MASK))
					continue;

				hw_id1 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_HW_ID1);
				hw_id2 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_HW_ID2);
				if (hw_id1 == 0xbebebeef || hw_id2 == 0xbebebeef) {
					sentinel_count++;
					continue;
				}

				total_valid++;
				vmid = FIELD_GET(SQ_WAVE_HW_ID2__VM_ID_MASK, hw_id2);
				if (vmid != target_vmid)
					continue;

				target_valid++;
				if (status & SQ_WAVE_STATUS__TRAP_EN_MASK)
					target_trap_en++;
				if (status & SQ_WAVE_STATUS__TRAP_MASK)
					target_trap++;

				trapsts = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_TRAPSTS);
				if (trapsts != 0xbebebeef &&
				    (trapsts & SQ_WAVE_TRAPSTS__HOST_TRAP_MASK))
					target_host_trap++;

				queue_id = FIELD_GET(SQ_WAVE_HW_ID2__QUEUE_ID_MASK, hw_id2);
				pipe_id = FIELD_GET(SQ_WAVE_HW_ID2__PIPE_ID_MASK, hw_id2);
				me_id = FIELD_GET(SQ_WAVE_HW_ID2__ME_ID_MASK, hw_id2);
				simd_id = FIELD_GET(SQ_WAVE_HW_ID1__SIMD_ID_MASK, hw_id1);
				hw_wave_id = FIELD_GET(SQ_WAVE_HW_ID1__WAVE_ID_MASK, hw_id1);

				queue_mask |= BIT(queue_id & 0xF);
				pipe_mask |= BIT(pipe_id & 0x3);
				me_mask |= BIT(me_id & 0x7);
				simd_mask |= BIT(simd_id & 0x3);
				wave_mask |= BIT(hw_wave_id & 0x1F);

				if (detail_budget > 0) {
					detail_budget--;
					dev_info(adev->dev,
						 "trigger_pc_sample_trap: global_wave target_vmid=%u se=%d sh=%d status_wave=%d hw_wave=%u simd=%u queue=%u pipe=%u me=%u status=0x%x trapsts=0x%x hw_id1=0x%x hw_id2=0x%x\n",
						 target_vmid, se, sh, wave, hw_wave_id, simd_id, queue_id,
						 pipe_id, me_id, status, trapsts, hw_id1, hw_id2);
				}
			}
		}
	}
	amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
	mutex_unlock(&adev->grbm_idx_mutex);

	dev_info(adev->dev,
		 "trigger_pc_sample_trap: global_wave_summary target_vmid=%u total_valid=%u total_nonzero_status=%u target_valid=%u target_trap_en=%u target_trap=%u target_host_trap=%u queue_mask=0x%x pipe_mask=0x%x me_mask=0x%x simd_mask=0x%x wave_mask=0x%08x sentinel=%u (sampled %d waves/SH)\n",
		 target_vmid, total_valid, total_nonzero_status, target_valid,
		 target_trap_en, target_trap, target_host_trap,
		 queue_mask, pipe_mask, me_mask, simd_mask, wave_mask,
		 sentinel_count, max_waves);
}

/*
 * Pick one active trap-enabled wave for target_vmid and return its hardware
 * queue-id/wave-id. This avoids issuing MODE_SINGLE SQ_CMD to an unrelated
 * queue, which can latch pending_count without servicing host-trap.
 */
static bool kgd_gfx_v11_pick_target_wave_queue(struct amdgpu_device *adev,
		uint32_t inst, uint32_t target_vmid,
		uint32_t *wave_id, uint32_t *queue_id, uint32_t *simd_id,
		uint32_t *status_wave_id, uint32_t *hw_wave_id,
		uint32_t *se_id, uint32_t *sh_id)
{
	int se, sh, wave;
	int max_waves = amdkfd_gfx11_pcs_wave_scan_max_waves;
	bool found = false;
	uint32_t picked_wave = 0, picked_queue = 0, picked_simd = 0;
	uint32_t picked_status_wave = 0, picked_hw_wave = 0;
	uint32_t picked_se = 0, picked_sh = 0;

	if (!wave_id || !queue_id || !simd_id || !status_wave_id || !hw_wave_id ||
	    !se_id || !sh_id)
		return false;

	if (max_waves < 1)
		max_waves = 1;
	if (max_waves > 32)
		max_waves = 32;

	mutex_lock(&adev->grbm_idx_mutex);
	for (se = 0; se < adev->gfx.config.max_shader_engines && !found; se++) {
		for (sh = 0; sh < adev->gfx.config.max_sh_per_se && !found; sh++) {
			amdgpu_gfx_select_se_sh(adev, se, sh, 0xFFFFFFFF, inst);
			for (wave = 0; wave < max_waves; wave++) {
				uint32_t status = kgd_gfx_v11_wave_read_ind(adev, inst, wave,
									 ixSQ_WAVE_STATUS);
				uint32_t hw_id1, hw_id2, vmid;

				if (status == 0xbebebeef)
					continue;
				if (!(status & SQ_WAVE_STATUS__VALID_MASK))
					continue;
				if (!(status & SQ_WAVE_STATUS__TRAP_EN_MASK))
					continue;

				hw_id1 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_HW_ID1);
				hw_id2 = kgd_gfx_v11_wave_read_ind(adev, inst, wave, ixSQ_WAVE_HW_ID2);
				if (hw_id1 == 0xbebebeef || hw_id2 == 0xbebebeef)
					continue;

				vmid = FIELD_GET(SQ_WAVE_HW_ID2__VM_ID_MASK, hw_id2);
				if (vmid != target_vmid)
					continue;

				picked_status_wave = wave & 0x1F;
				picked_hw_wave = FIELD_GET(SQ_WAVE_HW_ID1__WAVE_ID_MASK, hw_id1) & 0x1F;
				picked_wave = picked_hw_wave;
				picked_simd = FIELD_GET(SQ_WAVE_HW_ID1__SIMD_ID_MASK, hw_id1) & 0x3;
				picked_queue = FIELD_GET(SQ_WAVE_HW_ID2__QUEUE_ID_MASK, hw_id2) & 0xF;
				picked_se = se;
				picked_sh = sh;
				found = true;
				break;
			}
		}
	}
	amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
	mutex_unlock(&adev->grbm_idx_mutex);

	if (!found)
		return false;

	*wave_id = picked_wave;
	*queue_id = picked_queue;
	*simd_id = picked_simd;
	*status_wave_id = picked_status_wave;
	*hw_wave_id = picked_hw_wave;
	*se_id = picked_se;
	*sh_id = picked_sh;
	return true;
}

static void kgd_gfx_v11_log_runtime_trap_regs(struct amdgpu_device *adev,
		uint32_t vmid, uint32_t inst)
{
	uint32_t tba_lo, tba_hi, tma_lo, tma_hi, gdbg_cntl;
	uint32_t tba_hi_no_trap;
	u64 tba_reg_addr, tma_reg_addr;
	u64 tba_byte_addr, tma_byte_addr;
	bool have_expected = false;
	u64 expected_tba_byte = 0, expected_tma_byte = 0;
	u64 expected_tba_reg = 0, expected_tma_reg = 0;

	lock_srbm(adev, 0, 0, 0, vmid);
	tba_lo = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TBA_LO);
	tba_hi = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TBA_HI);
	tma_lo = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TMA_LO);
	tma_hi = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TMA_HI);
	gdbg_cntl = RREG32_SOC15(GC, GET_INST(GC, inst), regSPI_GDBG_PER_VMID_CNTL);
	unlock_srbm(adev);

	tba_hi_no_trap = tba_hi & ~(1U << SQ_SHADER_TBA_HI__TRAP_EN__SHIFT);
	tba_reg_addr = ((u64)tba_hi_no_trap << 32) | tba_lo;
	tma_reg_addr = ((u64)tma_hi << 32) | tma_lo;
	tba_byte_addr = tba_reg_addr << 8;
	tma_byte_addr = tma_reg_addr << 8;

	dev_info(adev->dev,
		 "trigger_pc_sample_trap: runtime_regs vmid=%u TBA=0x%x_%x TMA=0x%x_%x reg_tba=0x%llx reg_tma=0x%llx decoded_tba=0x%llx decoded_tma=0x%llx GDBG=0x%x trap_en=%u\n",
		 vmid, tba_hi, tba_lo, tma_hi, tma_lo,
		 tba_reg_addr, tma_reg_addr,
		 tba_byte_addr, tma_byte_addr,
		 gdbg_cntl,
		 !!(tba_hi & (1 << SQ_SHADER_TBA_HI__TRAP_EN__SHIFT)));

	if (vmid < AMDGPU_NUM_VMID && amdkfd_gfx11_last_trap_valid[vmid]) {
		have_expected = true;
		expected_tba_byte = amdkfd_gfx11_last_tba_byte[vmid];
		expected_tma_byte = amdkfd_gfx11_last_tma_byte[vmid];
		expected_tba_reg = amdkfd_gfx11_last_tba_reg[vmid];
		expected_tma_reg = amdkfd_gfx11_last_tma_reg[vmid];
	}

	dev_info(adev->dev,
		 "trigger_pc_sample_trap: runtime_regs vmid=%u expected_valid=%u expected_tba_byte=0x%llx expected_tma_byte=0x%llx expected_tba_reg=0x%llx expected_tma_reg=0x%llx\n",
		 vmid, have_expected, expected_tba_byte, expected_tma_byte,
		 expected_tba_reg, expected_tma_reg);
}

static void kgd_gfx_v11_log_tma_check(struct amdgpu_device *adev,
		uint32_t vmid, uint32_t inst)
{
	uint32_t tma_lo, tma_hi;
	u64 programmed_tma_reg, programmed_tma_byte;
	bool have_expected = false;
	u64 expected_tma_byte = 0, expected_tma_reg = 0;

	if (vmid < AMDGPU_NUM_VMID && amdkfd_gfx11_last_trap_valid[vmid]) {
		have_expected = true;
		expected_tma_byte = amdkfd_gfx11_last_tma_byte[vmid];
		expected_tma_reg = amdkfd_gfx11_last_tma_reg[vmid];
	}

	lock_srbm(adev, 0, 0, 0, vmid);
	tma_lo = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TMA_LO);
	tma_hi = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TMA_HI);
	unlock_srbm(adev);

	programmed_tma_reg = ((u64)tma_hi << 32) | tma_lo;
	programmed_tma_byte = programmed_tma_reg << 8;

	dev_info(adev->dev,
		 "trigger_pc_sample_trap: tma_check vmid=%u expected_valid=%u intended_tma_byte=0x%llx intended_tma_reg=0x%llx programmed_tma_reg=0x%llx programmed_tma_byte=0x%llx\n",
		 vmid, have_expected, expected_tma_byte, expected_tma_reg,
		 programmed_tma_reg, programmed_tma_byte);
}

static void program_trap_handler_settings_v11(struct amdgpu_device *adev,
		uint32_t vmid, uint64_t tba_addr, uint64_t tma_addr,
		uint32_t inst)
{
	uint32_t tba_lo, tba_hi, tma_lo, tma_hi, gdbg_cntl;
	u64 expected_tba_reg = tba_addr >> 8;
	u64 expected_tma_reg = tma_addr >> 8;
	u64 readback_tba_reg, readback_tma_reg;
	u64 readback_tba_byte, readback_tma_byte;

	dev_info(adev->dev, "program_trap_handler_settings_v11: vmid=%u tba=0x%llx tma=0x%llx inst=%u\n",
		 vmid, tba_addr, tma_addr, inst);

	if (vmid < AMDGPU_NUM_VMID) {
		amdkfd_gfx11_last_tba_byte[vmid] = tba_addr;
		amdkfd_gfx11_last_tma_byte[vmid] = tma_addr;
		amdkfd_gfx11_last_tba_reg[vmid] = expected_tba_reg;
		amdkfd_gfx11_last_tma_reg[vmid] = expected_tma_reg;
		amdkfd_gfx11_last_trap_valid[vmid] = true;
		amdkfd_gfx11_trap_prog_seq[vmid]++;
	}

	lock_srbm(adev, 0, 0, 0, vmid);

	/*
	 * Program TBA registers
	 */
	WREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TBA_LO,
		     lower_32_bits(tba_addr >> 8));
	WREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TBA_HI,
		     upper_32_bits(tba_addr >> 8) |
		     (1 << SQ_SHADER_TBA_HI__TRAP_EN__SHIFT));

	/*
	 * Program TMA registers
	 */
	WREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TMA_LO,
		     lower_32_bits(tma_addr >> 8));
	WREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TMA_HI,
		     upper_32_bits(tma_addr >> 8));

	/*
	 * Force enable TRAP_EN in SPI_GDBG_PER_VMID_CNTL.
	 * This is required for HostTrap (SQ_CMD) to work on GFX11.
	 */
	WREG32_SOC15(GC, GET_INST(GC, inst), regSPI_GDBG_PER_VMID_CNTL,
		     REG_SET_FIELD(0, SPI_GDBG_PER_VMID_CNTL, TRAP_EN, 1));

	/* Read back to verify programming (first vmid programmed only) */
	{
		tba_lo = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TBA_LO);
		tba_hi = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TBA_HI);
		tma_lo = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TMA_LO);
		tma_hi = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_SHADER_TMA_HI);
		gdbg_cntl = RREG32_SOC15(GC, GET_INST(GC, inst), regSPI_GDBG_PER_VMID_CNTL);
		readback_tba_reg =
			((u64)(tba_hi & ~(1U << SQ_SHADER_TBA_HI__TRAP_EN__SHIFT)) << 32) |
			tba_lo;
		readback_tma_reg = ((u64)tma_hi << 32) | tma_lo;
		readback_tba_byte = readback_tba_reg << 8;
		readback_tma_byte = readback_tma_reg << 8;
		dev_info(adev->dev,
			 "readback vmid=%u: TBA=0x%x_%x TMA=0x%x_%x GDBG_CNTL=0x%x TRAP_EN=%d\n",
			 vmid, tba_hi, tba_lo, tma_hi, tma_lo, gdbg_cntl,
			 !!(tba_hi & (1 << SQ_SHADER_TBA_HI__TRAP_EN__SHIFT)));
		dev_info(adev->dev,
			 "readback vmid=%u: intended_tba_byte=0x%llx intended_tma_byte=0x%llx intended_tba_reg=0x%llx intended_tma_reg=0x%llx readback_tba_reg=0x%llx readback_tma_reg=0x%llx readback_tba_byte=0x%llx readback_tma_byte=0x%llx\n",
			 vmid, tba_addr, tma_addr, expected_tba_reg, expected_tma_reg,
			 readback_tba_reg, readback_tma_reg,
			 readback_tba_byte, readback_tma_byte);
	}

	unlock_srbm(adev);
}

static uint32_t kgd_gfx_v11_trigger_pc_sample_trap(struct amdgpu_device *adev,
					    uint32_t vmid,
					    uint32_t *target_simd,
					    uint32_t *target_wave_slot,
					    enum kfd_ioctl_pc_sample_method method,
					    uint32_t inst)
{
	static int trigger_count = 0;
	uint32_t max_simd = adev->gfx.cu_info.simd_per_cu;
	uint32_t max_wave_slot = adev->gfx.cu_info.max_waves_per_simd;
	bool log_this_call = false;

	if (!max_simd)
		max_simd = 4;
	if (!max_wave_slot)
		max_wave_slot = 16;
	if (max_wave_slot > 16)
		max_wave_slot = 16;

	if (method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
#if 1 /* Re-enabled: Test 21m */
		static uint32_t last_vmid = 0xFFFFFFFF;
		static uint32_t last_status = 0xFFFFFFFF;
		static bool cap_logged = false;
		static bool probe_disable = false;
		static bool pending_dumped = false;
		static uint32_t pending_dump_status = 0xFFFFFFFF;
		static u64 last_prog_seq;
		static bool last_sqcmd_valid = false;
		static uint32_t last_sqcmd_value;
		static uint32_t last_sqcmd_wave_id;
		static uint32_t last_sqcmd_queue_id;
		static uint32_t last_sqcmd_mode;
		static uint32_t last_sqcmd_check_vmid;
		static bool last_sqcmd_set_wave;
		static bool last_sqcmd_set_queue;
		static bool last_sqcmd_set_vmid;
		int policy = amdkfd_gfx11_pcs_sqcmd_policy;
		const char *policy_name = kgd_gfx_v11_pcs_sqcmd_policy_name(policy);
		const char *reset_reason = NULL;
		uint32_t max_injected_traps = amdkfd_gfx11_pcs_max_injected_traps;
		uint32_t value = 0;
		uint32_t sq_hosttrap_status = 0x0;
		uint32_t sq_pending_count = 0;
		uint32_t cmd = SQ_IND_CMD_CMD_TRAP;
		uint32_t mode = SQ_IND_CMD_MODE_SINGLE;
		uint32_t check_vmid = 1;
		bool set_wave_id = true;
		bool set_queue_id = true;
		bool set_vmid = true;
		bool issue_sqcmd = true;
		bool stop_after_nonzero_post_status = false;
		uint32_t queue_id = 0;
		uint32_t wave_id = 0;
		uint32_t post_status_delay_us = amdkfd_gfx11_pcs_post_status_delay_us;
		uint32_t sq_cmd_readback = 0;
		int cmd_override = amdkfd_gfx11_pcs_sqcmd_cmd_override;
		int status_se = -1;
		int status_sh = -1;
		uint32_t sqcmd_se = 0xFFFFFFFF;
		uint32_t sqcmd_sh = 0xFFFFFFFF;
		bool sqcmd_target_se_sh = false;
		u64 prog_seq = 0;

		if (!max_injected_traps)
			max_injected_traps = 1;
		if (max_injected_traps > 4096)
			max_injected_traps = 4096;
		if (post_status_delay_us > 200000)
			post_status_delay_us = 200000;
		if (cmd_override >= 0 && cmd_override <= 0xF)
			cmd = (uint32_t)cmd_override;

		if (vmid < AMDGPU_NUM_VMID)
			prog_seq = READ_ONCE(amdkfd_gfx11_trap_prog_seq[vmid]);

		sq_hosttrap_status = kgd_gfx_v11_get_hosttrap_status(adev, inst,
					     &status_se, &status_sh);
			if (vmid != last_vmid) {
				reset_reason = "vmid_changed";
			} else if (prog_seq != last_prog_seq) {
				reset_reason = "trap_reprogrammed";
			}
			if (reset_reason) {
				trigger_count = 0;
				*target_simd = 0;
				*target_wave_slot = 0;
				last_vmid = vmid;
				last_status = 0xFFFFFFFF;
				cap_logged = false;
				probe_disable = false;
				pending_dumped = false;
				pending_dump_status = 0xFFFFFFFF;
				last_prog_seq = prog_seq;
				last_sqcmd_valid = false;
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: reset state for vmid=%u policy=%s(%d) reason=%s prog_seq=%llu\n",
					 vmid, policy_name, policy, reset_reason, prog_seq);
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: decode constants TRAPSTS_HOST_MASK=0x%x TRAPSTS_HOST_SHIFT=%u STATUS_TRAP_EN_MASK=0x%x STATUS_TRAP_MASK=0x%x\n",
					 (uint32_t)SQ_WAVE_TRAPSTS__HOST_TRAP_MASK,
					 SQ_WAVE_TRAPSTS__HOST_TRAP__SHIFT,
					 (uint32_t)SQ_WAVE_STATUS__TRAP_EN_MASK,
					 (uint32_t)SQ_WAVE_STATUS__TRAP_MASK);
				if (amdkfd_gfx11_pcs_runtime_reg_readback_on_reset)
					kgd_gfx_v11_log_runtime_trap_regs(adev, vmid, inst);
				if (amdkfd_gfx11_pcs_wave_scan_on_reset) {
					kgd_gfx_v11_log_wave_presence_summary(adev, inst, vmid);
					kgd_gfx_v11_log_target_vmid_wave_masks(adev, inst, vmid);
					kgd_gfx_v11_log_global_wave_vmid_queue_summary(adev, inst, vmid);
				}
			}
			trigger_count++;
			sq_pending_count =
				sq_hosttrap_status & SQ_DEBUG_HOST_TRAP_STATUS__PENDING_COUNT_MASK;

			/* Select SQ_CMD trigger policy for A/B/C isolation. */
			switch (policy) {
		case AMDKFD_GFX11_PCS_SQCMD_SINGLE_Q0:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 1;
			wave_id = ((*target_simd * max_wave_slot) + *target_wave_slot) & 0x1F;
			queue_id = 0;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 1;
			wave_id = ((*target_simd * max_wave_slot) + *target_wave_slot) & 0x1F;
			queue_id = (trigger_count - 1) & 0x7;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_BROADCAST:
			mode = SQ_IND_CMD_MODE_BROADCAST;
			check_vmid = 0;
			set_wave_id = false;
			set_queue_id = false;
			set_vmid = false;
			wave_id = 0;
			queue_id = 0;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_PROBE:
			mode = SQ_IND_CMD_MODE_BROADCAST;
			check_vmid = 0;
			set_wave_id = false;
			set_queue_id = false;
			set_vmid = false;
			stop_after_nonzero_post_status = true;
			if (max_injected_traps > 16)
				max_injected_traps = 16;
			wave_id = 0;
			queue_id = 0;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE:
			mode = SQ_IND_CMD_MODE_BROADCAST_QUEUE;
			check_vmid = 1;
			set_wave_id = false;
			set_queue_id = true;
			set_vmid = true;
			wave_id = 0;
			queue_id = (trigger_count - 1) & 0x7;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_PROBE:
			mode = SQ_IND_CMD_MODE_BROADCAST_QUEUE;
			check_vmid = 1;
			set_wave_id = false;
			set_queue_id = true;
			set_vmid = true;
			stop_after_nonzero_post_status = true;
			if (max_injected_traps > 16)
				max_injected_traps = 16;
			wave_id = 0;
			queue_id = (trigger_count - 1) & 0x7;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP_NO_VMID_CHECK:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 0;
			wave_id = ((*target_simd * max_wave_slot) + *target_wave_slot) & 0x1F;
			queue_id = (trigger_count - 1) & 0x7;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_SINGLE_FIXED_Q0_W0:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 1;
			wave_id = 0;
			queue_id = 0;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP_W0:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 1;
			wave_id = 0;
			queue_id = (trigger_count - 1) & 0x7;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_NOOP:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 0;
			set_wave_id = false;
			set_queue_id = false;
			set_vmid = false;
			issue_sqcmd = false;
			wave_id = 0;
			queue_id = 0;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_NO_VMID:
			mode = SQ_IND_CMD_MODE_BROADCAST_QUEUE;
			check_vmid = 0;
			set_wave_id = false;
			set_queue_id = true;
			set_vmid = false;
			wave_id = 0;
			queue_id = (trigger_count - 1) & 0x7;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_NO_VMID_PROBE:
			mode = SQ_IND_CMD_MODE_BROADCAST_QUEUE;
			check_vmid = 0;
			set_wave_id = false;
			set_queue_id = true;
			set_vmid = false;
			stop_after_nonzero_post_status = true;
			if (max_injected_traps > 16)
				max_injected_traps = 16;
			wave_id = 0;
			queue_id = (trigger_count - 1) & 0x7;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE_NO_VMID:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 0;
			set_wave_id = true;
			set_queue_id = false;
			set_vmid = false;
			wave_id = ((*target_simd * max_wave_slot) + *target_wave_slot) & 0x1F;
			queue_id = 0;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_BROADCAST_QUEUE_WAVE_NO_VMID_PROBE:
			mode = SQ_IND_CMD_MODE_BROADCAST_QUEUE;
			check_vmid = 0;
			set_wave_id = true;
			set_queue_id = true;
			set_vmid = false;
			stop_after_nonzero_post_status = true;
			if (max_injected_traps > 16)
				max_injected_traps = 16;
			wave_id = ((*target_simd * max_wave_slot) + *target_wave_slot) & 0x1F;
			queue_id = (trigger_count - 1) & 0x7;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 1;
			set_wave_id = true;
			set_queue_id = true;
			set_vmid = true;
			wave_id = ((*target_simd * max_wave_slot) + *target_wave_slot) & 0x1F;
			queue_id = 0;
			break;
		case AMDKFD_GFX11_PCS_SQCMD_SINGLE_WAVE_NO_QUEUE_PROBE:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 1;
			set_wave_id = true;
			set_queue_id = true;
			set_vmid = true;
			stop_after_nonzero_post_status = true;
			if (max_injected_traps > 16)
				max_injected_traps = 16;
			wave_id = ((*target_simd * max_wave_slot) + *target_wave_slot) & 0x1F;
			queue_id = 0;
			break;
		default:
			mode = SQ_IND_CMD_MODE_SINGLE;
			check_vmid = 1;
			wave_id = ((*target_simd * max_wave_slot) + *target_wave_slot) & 0x1F;
			queue_id = (trigger_count - 1) & 0x7;
			policy = AMDKFD_GFX11_PCS_SQCMD_SINGLE_QUEUE_SWEEP;
			policy_name = kgd_gfx_v11_pcs_sqcmd_policy_name(policy);
			break;
		}

			log_this_call = (trigger_count <= 20) || (trigger_count % 500 == 0);
			if (log_this_call || (trigger_count % 1000 == 0))
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: count=%d status=0x%x pending_count=%u se=%d sh=%d simd=%u wave_slot=%u max_simd=%u max_wave=%u\n",
					 trigger_count, sq_hosttrap_status, sq_pending_count, status_se, status_sh,
					 *target_simd, *target_wave_slot,
					 max_simd, max_wave_slot);
			if (sq_hosttrap_status != last_status) {
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: status transition vmid=%u 0x%x -> 0x%x\n",
				 vmid, last_status, sq_hosttrap_status);
			last_status = sq_hosttrap_status;
			}
			/* skip when last host trap request is still pending to complete */
			if (sq_pending_count) {
				bool dump_pending_state = (!pending_dumped ||
							  pending_dump_status != sq_hosttrap_status ||
							  trigger_count <= 8);

				if (log_this_call || dump_pending_state)
					dev_info(adev->dev,
						 "trigger_pc_sample_trap: skip SQ_CMD because pending_count=%u (raw_status=0x%x)\n",
						 sq_pending_count, sq_hosttrap_status);
				if (dump_pending_state) {
					dev_info(adev->dev,
						 "trigger_pc_sample_trap: pending latch vmid=%u trigger_count=%d status=0x%x se=%d sh=%d prog_seq=%llu last_prog_seq=%llu last_vmid=%u\n",
						 vmid, trigger_count, sq_hosttrap_status, status_se, status_sh,
						 prog_seq, last_prog_seq, last_vmid);
					if (last_sqcmd_valid)
						dev_info(adev->dev,
							 "trigger_pc_sample_trap: pending latch last_sqcmd=0x%08x mode=%u check_vmid=%u wave_id=%u(set=%u) queue_id=%u(set=%u) vmid_set=%u\n",
							 last_sqcmd_value, last_sqcmd_mode, last_sqcmd_check_vmid,
							 last_sqcmd_wave_id, last_sqcmd_set_wave,
							 last_sqcmd_queue_id, last_sqcmd_set_queue,
							 last_sqcmd_set_vmid);
					kgd_gfx_v11_dump_wave_trap_state(adev, inst,
									 status_se, status_sh,
								 vmid);
					if (amdkfd_gfx11_pcs_wave_scan_on_reset)
						kgd_gfx_v11_log_global_wave_vmid_queue_summary(adev, inst, vmid);
					pending_dumped = true;
					pending_dump_status = sq_hosttrap_status;
				}
			return 0;
		}
		if (!issue_sqcmd) {
			if (log_this_call || (trigger_count % 1000 == 0))
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: NOOP policy active; skipping SQ_CMD write (vmid=%u)\n",
					 vmid);
			return 0;
		}
			if (probe_disable)
				return 0;
			if (trigger_count > max_injected_traps) {
				if (!cap_logged) {
					dev_info(adev->dev,
						 "trigger_pc_sample_trap: injection cap reached at %u for vmid=%u; SQ_CMD disabled for remainder\n",
						 max_injected_traps, vmid);
					cap_logged = true;
				}
				return 0;
			}

			/*
			 * For MODE_SINGLE hosttrap, steer SQ_CMD to a live queue/wave of
			 * target VMID. Implicit queue-id=0 on gfx11 can latch pending_count
			 * when active waves are in a different queue.
			 */
			if (mode == SQ_IND_CMD_MODE_SINGLE && check_vmid && set_wave_id) {
				uint32_t live_wave_id = 0, live_queue_id = 0, live_simd_id = 0;
				uint32_t live_status_wave = 0, live_hw_wave = 0;
				uint32_t live_se = 0, live_sh = 0;
				bool use_status_wave = !!amdkfd_gfx11_pcs_single_wave_use_status_slot;
				bool live_found =
					kgd_gfx_v11_pick_target_wave_queue(adev, inst, vmid,
									 &live_wave_id,
									 &live_queue_id,
									 &live_simd_id,
									 &live_status_wave,
									 &live_hw_wave,
									 &live_se,
									 &live_sh);
				if (!live_found) {
					if (log_this_call || (trigger_count % 1000 == 0))
						dev_info(adev->dev,
							 "trigger_pc_sample_trap: no live trap-enabled wave for vmid=%u; skip SQ_CMD\n",
							 vmid);
					return 0;
				}
				sqcmd_target_se_sh = true;
				sqcmd_se = live_se;
				sqcmd_sh = live_sh;
				if (use_status_wave)
					live_wave_id = live_status_wave;

				if ((wave_id != live_wave_id) || (queue_id != live_queue_id) ||
				    !set_queue_id) {
					if (log_this_call || (trigger_count % 1000 == 0))
						dev_info(adev->dev,
							 "trigger_pc_sample_trap: retarget single mode vmid=%u wave %u->%u queue %u->%u simd=%u se=%u sh=%u (source=%s status_wave=%u hw_wave=%u)\n",
							 vmid, wave_id, live_wave_id, queue_id,
							 live_queue_id, live_simd_id, live_se, live_sh,
							 use_status_wave ? "status_slot" : "hw_wave",
							 live_status_wave, live_hw_wave);
						wave_id = live_wave_id;
						queue_id = live_queue_id;
						set_queue_id = true;
					*target_simd = live_simd_id;
				}
			}

			value = REG_SET_FIELD(value, SQ_CMD, CMD, cmd);
			value = REG_SET_FIELD(value, SQ_CMD, MODE, mode);
		if (set_wave_id)
			value = REG_SET_FIELD(value, SQ_CMD, WAVE_ID, wave_id);
		/* set TrapID 4 for HOSTTRAP */
		value = REG_SET_FIELD(value, SQ_CMD, DATA, 0x4);
		if (set_queue_id)
			value = REG_SET_FIELD(value, SQ_CMD, QUEUE_ID, queue_id);
		if (set_vmid)
			value = REG_SET_FIELD(value, SQ_CMD, VM_ID, vmid);
		value = REG_SET_FIELD(value, SQ_CMD, CHECK_VMID, check_vmid);
		last_sqcmd_valid = true;
		last_sqcmd_value = value;
		last_sqcmd_wave_id = wave_id;
		last_sqcmd_queue_id = queue_id;
		last_sqcmd_mode = mode;
		last_sqcmd_check_vmid = check_vmid;
		last_sqcmd_set_wave = set_wave_id;
		last_sqcmd_set_queue = set_queue_id;
		last_sqcmd_set_vmid = set_vmid;

			if (log_this_call || (trigger_count % 1000 == 0))
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: SQ_CMD=0x%08x vmid=%u cmd=%u mode=%u check_vmid=%u wave_id=%u queue_id=%u simd=%u wave_slot=%u policy=%s scope=%s scope_se=%u scope_sh=%u\n",
					 value, vmid, cmd, mode, check_vmid, wave_id, queue_id,
					 *target_simd, *target_wave_slot, policy_name,
					 sqcmd_target_se_sh ? "single_se_sh" : "broadcast_all",
					 sqcmd_se, sqcmd_sh);
			if (log_this_call)
				kgd_gfx_v11_log_tma_check(adev, vmid, inst);
			if (log_this_call && set_wave_id && amdkfd_gfx11_pcs_wave_target_debug)
				kgd_gfx_v11_log_selected_wave_visibility(adev, inst, vmid,
									 wave_id,
									 *target_simd);

		/* Use direct SQ_CMD write path (same style as gfx12) for host-trap testing. */
		mutex_lock(&adev->grbm_idx_mutex);
		if (sqcmd_target_se_sh)
			amdgpu_gfx_select_se_sh(adev, sqcmd_se, sqcmd_sh, 0xFFFFFFFF, inst);
		else
			amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
		WREG32_SOC15(GC, GET_INST(GC, inst), regSQ_CMD, value);
		sq_cmd_readback = RREG32_SOC15(GC, GET_INST(GC, inst), regSQ_CMD);
		amdgpu_gfx_select_se_sh(adev, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, inst);
		mutex_unlock(&adev->grbm_idx_mutex);
		if (log_this_call || (trigger_count % 1000 == 0))
			dev_info(adev->dev,
				 "trigger_pc_sample_trap: SQ_CMD readback=0x%08x after write\n",
				 sq_cmd_readback);

		/* Check status after sending command (outside lock) */
		if (log_this_call || stop_after_nonzero_post_status) {
			uint32_t post_status;
			uint32_t post_pending_count;
			bool post_transition;
			int post_status_se = -1;
			int post_status_sh = -1;
			if (post_status_delay_us)
				udelay(post_status_delay_us);  /* Let trap status propagate */
			post_status = kgd_gfx_v11_get_hosttrap_status(adev, inst,
								&post_status_se, &post_status_sh);
			post_pending_count =
				post_status & SQ_DEBUG_HOST_TRAP_STATUS__PENDING_COUNT_MASK;
			if (log_this_call || post_status)
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: post_status=0x%x pending_count=%u se=%d sh=%d (0=no pending trap)\n",
					 post_status, post_pending_count,
					 post_status_se, post_status_sh);
			if (post_status && (log_this_call || stop_after_nonzero_post_status))
				kgd_gfx_v11_dump_wave_trap_state(adev, inst,
								 post_status_se,
								 post_status_sh,
								 vmid);
			if (post_status && (log_this_call || stop_after_nonzero_post_status))
				kgd_gfx_v11_log_hosttrap_status_matrix(adev, inst, vmid);
			post_transition = (post_status != last_status);
			if (post_status && post_transition) {
				kgd_gfx_v11_log_runtime_trap_regs(adev, vmid, inst);
				if (amdkfd_gfx11_pcs_wave_scan_on_reset)
					kgd_gfx_v11_log_global_wave_vmid_queue_summary(adev, inst, vmid);
				if (set_wave_id && amdkfd_gfx11_pcs_wave_target_debug)
					kgd_gfx_v11_log_selected_wave_visibility(adev, inst, vmid,
									 wave_id,
									 *target_simd);
			}
			if (post_transition) {
				dev_info(adev->dev,
						 "trigger_pc_sample_trap: post-status transition vmid=%u 0x%x -> 0x%x\n",
						 vmid, last_status, post_status);
				last_status = post_status;
			}
			if (stop_after_nonzero_post_status && post_status) {
				probe_disable = true;
				dev_info(adev->dev,
					 "trigger_pc_sample_trap: probe latched post_status=0x%x; disabling further SQ_CMD for vmid=%u\n",
					 post_status, vmid);
			}
		}

		(*target_wave_slot)++;
		if (*target_wave_slot >= max_wave_slot) {
			*target_wave_slot = 0;
			(*target_simd)++;
			*target_simd %= max_simd;
		}
#else
		/* NO-OP: Skip SQ_CMD entirely. Tests if fault is from setup vs trigger. */
		trigger_count++;
		if (trigger_count <= 5 || (trigger_count % 1000 == 0))
			dev_info(adev->dev,
				 "trigger_pc_sample_trap: NOOP count=%d vmid=%u (SQ_CMD disabled)\n",
				 trigger_count, vmid);
#endif
	} else {
		dev_dbg(adev->dev, "PC Sampling method %d not supported.", method);
		return -EOPNOTSUPP;
	}
	return 0;
}

const struct kfd2kgd_calls gfx_v11_kfd2kgd = {
	.program_sh_mem_settings = program_sh_mem_settings_v11,
	.set_pasid_vmid_mapping = set_pasid_vmid_mapping_v11,
	.init_interrupts = init_interrupts_v11,
	.hqd_load = hqd_load_v11,
	.hiq_mqd_load = hiq_mqd_load_v11,
	.hqd_sdma_load = hqd_sdma_load_v11,
	.hqd_dump = hqd_dump_v11,
	.hqd_sdma_dump = hqd_sdma_dump_v11,
	.hqd_is_occupied = hqd_is_occupied_v11,
	.hqd_sdma_is_occupied = hqd_sdma_is_occupied_v11,
	.hqd_destroy = hqd_destroy_v11,
	.hqd_sdma_destroy = hqd_sdma_destroy_v11,
	.wave_control_execute = wave_control_execute_v11,
	.get_atc_vmid_pasid_mapping_info = get_atc_vmid_pasid_mapping_info_v11,
	.set_vm_context_page_table_base = set_vm_context_page_table_base_v11,
	.enable_debug_trap = kgd_gfx_v11_enable_debug_trap,
	.disable_debug_trap = kgd_gfx_v11_disable_debug_trap,
	.validate_trap_override_request = kgd_gfx_v11_validate_trap_override_request,
	.set_wave_launch_trap_override = kgd_gfx_v11_set_wave_launch_trap_override,
	.set_wave_launch_mode = kgd_gfx_v11_set_wave_launch_mode,
	.set_address_watch = kgd_gfx_v11_set_address_watch,
	.clear_address_watch = kgd_gfx_v11_clear_address_watch,
	.program_trap_handler_settings = program_trap_handler_settings_v11,
	.hqd_get_pq_addr = kgd_gfx_v11_hqd_get_pq_addr,
	.hqd_reset = kgd_gfx_v11_hqd_reset,
	.hqd_sdma_get_doorbell = kgd_gfx_v11_hqd_sdma_get_doorbell,
	.trigger_pc_sample_trap = kgd_gfx_v11_trigger_pc_sample_trap
};
