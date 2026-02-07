// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Copyright 2023 Advanced Micro Devices, Inc.
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

#include "kfd_priv.h"
#include "amdgpu_amdkfd.h"
#include "kfd_pc_sampling.h"
#include "kfd_debug.h"
#include "kfd_device_queue_manager.h"

#include <linux/bitops.h>
/*
 * PC Sampling revision change log
 *
 * 0.1 - Initial revision
 * 0.2 - Support gfx9_4_3 Host Trap PC sampling
 * 0.3 - Fix gfx9_4_3 SQ hang issue
 * 1.1 - Support gfx9_4_3 Stochastic PC sampling
 * 1.2 - Support gfx9_5_0 Host Trap PC sampling
 * 1.3 - Update 1st level trap handler for Stochastic PC sampling
 * 1.4 - Support gfx9_5_0 Stochastic PC sampling
 * 1.5 - Support gfx12_0_0 and gfx12_0_1 Host Trap PC sampling
 */
#define KFD_IOCTL_PCS_MAJOR_VERSION	1
#define KFD_IOCTL_PCS_MINOR_VERSION	5

struct supported_pc_sample_info {
	uint32_t ip_version;
	const struct kfd_pc_sample_info *sample_info;
};

const struct kfd_pc_sample_info sample_info_hosttrap_9_0_0 = {
	0, 1, ~0ULL, 0, KFD_IOCTL_PCS_METHOD_HOSTTRAP, KFD_IOCTL_PCS_TYPE_TIME_US };

const struct kfd_pc_sample_info sample_info_stoch_cycle_9_4_3 = {
	0, 256, (1ULL << 31), KFD_IOCTL_PCS_FLAG_POWER_OF_2,
	    KFD_IOCTL_PCS_METHOD_STOCHASTIC, KFD_IOCTL_PCS_TYPE_CLOCK_CYCLES };

struct supported_pc_sample_info supported_formats[] = {
	{ IP_VERSION(9, 4, 2), &sample_info_hosttrap_9_0_0 },
	{ IP_VERSION(9, 4, 3), &sample_info_hosttrap_9_0_0 },
	{ IP_VERSION(9, 4, 3), &sample_info_stoch_cycle_9_4_3 },
	{ IP_VERSION(9, 5, 0), &sample_info_hosttrap_9_0_0 },
	{ IP_VERSION(9, 5, 0), &sample_info_stoch_cycle_9_4_3 },
	{ IP_VERSION(11, 5, 0), &sample_info_hosttrap_9_0_0 },
	{ IP_VERSION(11, 5, 1), &sample_info_hosttrap_9_0_0 },
	{ IP_VERSION(11, 5, 2), &sample_info_hosttrap_9_0_0 },
	{ IP_VERSION(11, 5, 3), &sample_info_hosttrap_9_0_0 },
	{ IP_VERSION(12, 0, 0), &sample_info_hosttrap_9_0_0 },
	{ IP_VERSION(12, 0, 1), &sample_info_hosttrap_9_0_0 },
};

static uint32_t kfd_pc_sampling_lookup_vmid_by_pasid(struct kfd_node *node,
						      uint32_t owner_pasid)
{
	uint32_t vmid;
	uint32_t valid_count = 0;
	uint32_t first_valid_vmid = 0;
	uint16_t first_valid_pasid = 0;
	uint32_t fallback_valid_count = 0;
	uint32_t fallback_first_valid_vmid = 0;
	uint16_t fallback_first_valid_pasid = 0;

	if (!owner_pasid || !node->kfd2kgd->get_atc_vmid_pasid_mapping_info) {
		if (!owner_pasid)
			pr_warn_ratelimited("pcs vmid lookup skipped: owner_pasid=0 gc_ip=%06x active_count=%u target_vmid=%u\n",
				KFD_GC_VERSION(node),
				READ_ONCE(node->pcs_data.hosttrap_entry.base.active_count),
				READ_ONCE(node->pcs_data.hosttrap_entry.target_vmid));
		if (owner_pasid && !node->kfd2kgd->get_atc_vmid_pasid_mapping_info)
			pr_warn_ratelimited("pcs vmid lookup unavailable: owner_pasid=%u gc_ip=%06x kfd2kgd=%px get_atc=%px trigger=%px (get_atc_vmid_pasid_mapping_info is NULL)\n",
				owner_pasid, KFD_GC_VERSION(node), node->kfd2kgd,
				node->kfd2kgd->get_atc_vmid_pasid_mapping_info,
				node->kfd2kgd->trigger_pc_sample_trap);
		return 0;
	}

	for (vmid = node->vm_info.first_vmid_kfd;
	     vmid <= node->vm_info.last_vmid_kfd; vmid++) {
		uint16_t queried_pasid = 0;
		bool valid;

		valid = node->kfd2kgd->get_atc_vmid_pasid_mapping_info(node->adev,
								     vmid,
								     &queried_pasid);
		if (valid) {
			valid_count++;
			if (!first_valid_vmid) {
				first_valid_vmid = vmid;
				first_valid_pasid = queried_pasid;
			}
		}

		if (valid && queried_pasid == owner_pasid) {
			pr_warn("pcs vmid lookup hit: owner_pasid=%u vmid=%u range=%u-%u valid_count=%u\n",
				owner_pasid, vmid, node->vm_info.first_vmid_kfd,
				node->vm_info.last_vmid_kfd, valid_count);
			return vmid;
		}
	}

	pr_warn_ratelimited("pcs vmid lookup miss: owner_pasid=%u range=%u-%u valid_count=%u first_valid=(vmid=%u pasid=%u)\n",
		owner_pasid, node->vm_info.first_vmid_kfd, node->vm_info.last_vmid_kfd,
		valid_count, first_valid_vmid, first_valid_pasid);

	/* MES can assign a VMID outside [first_vmid_kfd..last_vmid_kfd] on
	 * some paths. Do one full-table fallback scan to confirm.
	 */
	for (vmid = 1; vmid < AMDGPU_NUM_VMID; vmid++) {
		uint16_t queried_pasid = 0;
		bool valid;

		if (vmid >= node->vm_info.first_vmid_kfd &&
		    vmid <= node->vm_info.last_vmid_kfd)
			continue;

		valid = node->kfd2kgd->get_atc_vmid_pasid_mapping_info(node->adev,
								     vmid,
								     &queried_pasid);
		if (valid) {
			fallback_valid_count++;
			if (!fallback_first_valid_vmid) {
				fallback_first_valid_vmid = vmid;
				fallback_first_valid_pasid = queried_pasid;
			}
		}

		if (valid && queried_pasid == owner_pasid) {
			pr_warn("pcs vmid lookup hit(outside_range): owner_pasid=%u vmid=%u kfd_range=%u-%u fallback_valid_count=%u\n",
				owner_pasid, vmid, node->vm_info.first_vmid_kfd,
				node->vm_info.last_vmid_kfd, fallback_valid_count);
			return vmid;
		}
	}

	pr_warn_ratelimited("pcs vmid lookup fallback miss: owner_pasid=%u outside_range_valid_count=%u first_valid=(vmid=%u pasid=%u)\n",
		owner_pasid, fallback_valid_count, fallback_first_valid_vmid,
		fallback_first_valid_pasid);

	return 0;
}

static void kfd_pc_sampling_dump_vmid_maps(struct kfd_node *node,
					   uint32_t owner_pasid,
					   const char *tag)
{
	uint32_t vmid;
	uint32_t sw_match = 0, atc_match = 0;
	uint32_t sw_nonzero = 0, atc_valid = 0;

	pr_warn("pcs vmid dump (%s): owner_pasid=%u range=%u-%u\n",
		tag, owner_pasid, node->vm_info.first_vmid_kfd,
		node->vm_info.last_vmid_kfd);

	for (vmid = node->vm_info.first_vmid_kfd;
	     vmid <= node->vm_info.last_vmid_kfd; vmid++) {
		uint16_t sw_pasid = 0;
		uint16_t atc_pasid = 0;
		bool atc_ok = false;

		if (node->dqm)
			sw_pasid = node->dqm->vmid_pasid[vmid];
		if (node->kfd2kgd->get_atc_vmid_pasid_mapping_info)
			atc_ok = node->kfd2kgd->get_atc_vmid_pasid_mapping_info(
				node->adev, vmid, &atc_pasid);

		if (sw_pasid)
			sw_nonzero++;
		if (atc_ok)
			atc_valid++;
		if (owner_pasid && sw_pasid == owner_pasid)
			sw_match++;
		if (owner_pasid && atc_ok && atc_pasid == owner_pasid)
			atc_match++;

		if (sw_pasid || atc_ok) {
			pr_warn("pcs vmid dump (%s): vmid=%u sw_pasid=%u atc_ok=%u atc_pasid=%u\n",
				tag, vmid, sw_pasid, atc_ok, atc_pasid);
		}
	}

	pr_warn("pcs vmid dump (%s): summary sw_nonzero=%u atc_valid=%u sw_match=%u atc_match=%u\n",
		tag, sw_nonzero, atc_valid, sw_match, atc_match);
}

static int kfd_pc_sample_thread(void *param)
{
	struct amdgpu_device *adev;
	struct kfd_node *node = param;
	uint32_t timeout = 0;
	uint64_t trigger_loop_count = 0;
	ktime_t next_trap_time;
	bool need_wait;
	uint32_t inst;
	uint32_t target_vmid;

	mutex_lock(&node->pcs_data.mutex);
	if (node->pcs_data.hosttrap_entry.base.active_count &&
		node->pcs_data.hosttrap_entry.base.pc_sample_info.interval &&
		node->kfd2kgd->trigger_pc_sample_trap) {
		switch (node->pcs_data.hosttrap_entry.base.pc_sample_info.type) {
		case KFD_IOCTL_PCS_TYPE_TIME_US:
			timeout = (uint32_t)node->pcs_data.hosttrap_entry.base.pc_sample_info.interval;
			break;
		default:
			pr_debug("PC Sampling type %d not supported.",
					node->pcs_data.hosttrap_entry.base.pc_sample_info.type);
		}
	}
	mutex_unlock(&node->pcs_data.mutex);
	if (!timeout) {
		pr_warn("pcs hosttrap: thread init disabled active_count=%u interval=%llu trigger=%px method=%u owner_pasid=%u target_vmid=%u\n",
			READ_ONCE(node->pcs_data.hosttrap_entry.base.active_count),
			node->pcs_data.hosttrap_entry.base.pc_sample_info.interval,
			node->kfd2kgd ? node->kfd2kgd->trigger_pc_sample_trap : NULL,
			node->pcs_data.hosttrap_entry.base.pc_sample_info.method,
			READ_ONCE(node->pcs_data.hosttrap_entry.owner_pasid),
			READ_ONCE(node->pcs_data.hosttrap_entry.target_vmid));
		return -EINVAL;
	}

	pr_warn("pcs hosttrap: thread init active_count=%u interval_us=%u owner_pasid=%u target_vmid=%u method=%u trigger=%px\n",
		READ_ONCE(node->pcs_data.hosttrap_entry.base.active_count),
		timeout,
		READ_ONCE(node->pcs_data.hosttrap_entry.owner_pasid),
		READ_ONCE(node->pcs_data.hosttrap_entry.target_vmid),
		node->pcs_data.hosttrap_entry.base.pc_sample_info.method,
		node->kfd2kgd ? node->kfd2kgd->trigger_pc_sample_trap : NULL);

	adev = node->adev;
	need_wait = false;
	allow_signal(SIGKILL);
	target_vmid = node->pcs_data.hosttrap_entry.target_vmid;

	if (node->kfd2kgd->override_core_cg)
		for_each_inst(inst, node->xcc_mask)
			node->kfd2kgd->override_core_cg(adev, 1, inst);

	while (!kthread_should_stop() &&
			!signal_pending(node->pcs_data.hosttrap_entry.pc_sample_thread)) {
		if (!need_wait) {
			next_trap_time = ktime_add_us(ktime_get_raw(), timeout);
			target_vmid = READ_ONCE(node->pcs_data.hosttrap_entry.target_vmid);
			if (!target_vmid) {
				uint32_t owner_pasid = READ_ONCE(node->pcs_data.hosttrap_entry.owner_pasid);
				uint32_t resolved_vmid =
					kfd_pc_sampling_lookup_vmid_by_pasid(node, owner_pasid);

				if (resolved_vmid) {
					WRITE_ONCE(node->pcs_data.hosttrap_entry.target_vmid, resolved_vmid);
					target_vmid = resolved_vmid;
					pr_warn("pc sample trigger resolved target_vmid=%u from owner_pasid=%u via ATC map\n",
						target_vmid, owner_pasid);
				}
			}
			if (!target_vmid) {
				pr_warn_ratelimited("pc sample trigger skipped: target_vmid=0 owner_pasid=%u active_count=%u (waiting for queue VMID assignment)\n",
					READ_ONCE(node->pcs_data.hosttrap_entry.owner_pasid),
					READ_ONCE(node->pcs_data.hosttrap_entry.base.active_count));
				need_wait = true;
				continue;
			}
			if (node->kfd2kgd->program_trap_handler_settings &&
			    READ_ONCE(node->pcs_data.hosttrap_entry.trap_regs_programmed_vmid) != target_vmid) {
				uint64_t tba_addr = READ_ONCE(node->pcs_data.hosttrap_entry.trap_tba_addr);
				uint64_t tma_addr = READ_ONCE(node->pcs_data.hosttrap_entry.trap_tma_addr);

				if (tba_addr || tma_addr) {
					pr_warn("pcs hosttrap: trigger program trap regs vmid=%u tba=0x%llx tma=0x%llx owner_pasid=%u\n",
						target_vmid, tba_addr, tma_addr,
						READ_ONCE(node->pcs_data.hosttrap_entry.owner_pasid));
					for_each_inst(inst, node->xcc_mask)
						node->kfd2kgd->program_trap_handler_settings(
							adev, target_vmid, tba_addr, tma_addr, inst);
					WRITE_ONCE(node->pcs_data.hosttrap_entry.trap_regs_programmed_vmid,
						   target_vmid);
				} else {
					pr_warn_ratelimited("pcs hosttrap: trigger program trap regs skipped (missing tba/tma) vmid=%u owner_pasid=%u\n",
						target_vmid,
						READ_ONCE(node->pcs_data.hosttrap_entry.owner_pasid));
				}
			}

			for_each_inst(inst, node->xcc_mask) {
				int ret;

				ret = node->kfd2kgd->trigger_pc_sample_trap(adev, target_vmid,
					&node->pcs_data.hosttrap_entry.target_simd,
					&node->pcs_data.hosttrap_entry.target_wave_slot,
					node->pcs_data.hosttrap_entry.base.pc_sample_info.method,
					inst);
				if (ret)
					pr_warn_ratelimited("pc sample trigger failed vmid=%d inst=%u ret=%d method=%u simd=%u wave_slot=%u\n",
						target_vmid, inst, ret,
						node->pcs_data.hosttrap_entry.base.pc_sample_info.method,
						node->pcs_data.hosttrap_entry.target_simd,
						node->pcs_data.hosttrap_entry.target_wave_slot);
			}
			trigger_loop_count++;
			if (trigger_loop_count <= 8 || !(trigger_loop_count % 1000)) {
				pr_warn("pcs hosttrap: trigger loop=%llu owner_pasid=%u target_vmid=%u active_count=%u simd=%u wave_slot=%u\n",
					trigger_loop_count,
					READ_ONCE(node->pcs_data.hosttrap_entry.owner_pasid),
					target_vmid,
					READ_ONCE(node->pcs_data.hosttrap_entry.base.active_count),
					node->pcs_data.hosttrap_entry.target_simd,
					node->pcs_data.hosttrap_entry.target_wave_slot);
			}
			pr_debug_ratelimited("triggered a host trap.");
			need_wait = true;
		} else {
			ktime_t wait_time;
			s64 wait_ns, wait_us;

			wait_time = ktime_sub(next_trap_time, ktime_get_raw());
			wait_ns = ktime_to_ns(wait_time);
			wait_us = ktime_to_us(wait_time);
			if (wait_ns >= 10000) {
				usleep_range(wait_us - 10, wait_us);
			} else {
				schedule();
				if (wait_ns <= 0)
					need_wait = false;
			}
		}
	}
	if (node->kfd2kgd->override_core_cg)
		for_each_inst(inst, node->xcc_mask)
			node->kfd2kgd->override_core_cg(adev, 0, inst);

	node->pcs_data.hosttrap_entry.target_simd = 0;
	node->pcs_data.hosttrap_entry.target_wave_slot = 0;
	node->pcs_data.hosttrap_entry.pc_sample_thread = NULL;

	return 0;
}

static int kfd_pc_sample_thread_start(struct kfd_node *node)
{
	char thread_name[16];
	int ret = 0;

	snprintf(thread_name, 16, "pcs_%d", node->adev->ddev.render->index);
	node->pcs_data.hosttrap_entry.pc_sample_thread =
		kthread_run(kfd_pc_sample_thread, node, thread_name);

	if (IS_ERR(node->pcs_data.hosttrap_entry.pc_sample_thread)) {
		ret = PTR_ERR(node->pcs_data.hosttrap_entry.pc_sample_thread);
		node->pcs_data.hosttrap_entry.pc_sample_thread = NULL;
		pr_warn("Failed to create pc sample thread for %s with ret = %d.",
			thread_name, ret);
	} else {
		pr_warn("pcs hosttrap: thread started name=%s pid=%d owner_pasid=%u target_vmid=%u interval=%llu\n",
			thread_name,
			task_pid_nr(node->pcs_data.hosttrap_entry.pc_sample_thread),
			READ_ONCE(node->pcs_data.hosttrap_entry.owner_pasid),
			READ_ONCE(node->pcs_data.hosttrap_entry.target_vmid),
			node->pcs_data.hosttrap_entry.base.pc_sample_info.interval);
	}

	return ret;
}

static int kfd_pc_sample_query_cap(struct kfd_process_device *pdd,
					struct kfd_ioctl_pc_sample_args __user *user_args)
{
	uint64_t sample_offset;
	int num_method = 0;
	int ret;
	int i;
	const uint32_t user_num_sample_info = user_args->num_sample_info;

	/* use version field to pass back pc sampling revision temporarily, not for upstream */
	user_args->version = KFD_IOCTL_PCS_MAJOR_VERSION << 16 | KFD_IOCTL_PCS_MINOR_VERSION;

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++)
		if (KFD_GC_VERSION(pdd->dev) == supported_formats[i].ip_version)
			num_method++;

	if (!num_method) {
		pr_debug("PC Sampling not supported on GC_HWIP:0x%x.",
			pdd->dev->adev->ip_versions[GC_HWIP][0]);
		return -EOPNOTSUPP;
	}

	ret = 0;
	mutex_lock(&pdd->dev->pcs_data.mutex);
	if (user_args->flags != KFD_IOCTL_PCS_QUERY_TYPE_FULL &&
		(pdd->dev->pcs_data.hosttrap_entry.base.use_count ||
		 pdd->dev->pcs_data.stoch_entry.base.use_count)) {
		user_args->num_sample_info = 0;

		/* If we already have a session, restrict returned list to current method  */
		if (pdd->dev->pcs_data.stoch_entry.base.use_count) {
			user_args->num_sample_info++;
			if (user_args->sample_info_ptr &&
				user_args->num_sample_info <= user_num_sample_info) {
				ret = copy_to_user((void __user *) user_args->sample_info_ptr,
					&pdd->dev->pcs_data.stoch_entry.base.pc_sample_info,
					sizeof(struct kfd_pc_sample_info));
				user_args->sample_info_ptr += sizeof(struct kfd_pc_sample_info);
			}
		}

		if (pdd->dev->pcs_data.hosttrap_entry.base.use_count) {
			user_args->num_sample_info++;
			if (user_args->sample_info_ptr &&
				user_args->num_sample_info <= user_num_sample_info)
				ret |= copy_to_user((void __user *) user_args->sample_info_ptr,
					&pdd->dev->pcs_data.hosttrap_entry.base.pc_sample_info,
					sizeof(struct kfd_pc_sample_info));
		}
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		return ret ? -EFAULT : 0;
	}
	mutex_unlock(&pdd->dev->pcs_data.mutex);

	user_args->num_sample_info = num_method;

	if (!user_args->sample_info_ptr || !user_num_sample_info) {
		/*
		 * User application is querying the size of buffer needed. Application will
		 * allocate required buffer size and send a second query.
		 */
		return 0;
	} else if (user_num_sample_info < num_method) {
		pr_debug("ASIC requires space for %d kfd_pc_sample_info entries.", num_method);
		return -ENOSPC;
	}

	sample_offset = user_args->sample_info_ptr;
	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (KFD_GC_VERSION(pdd->dev) == supported_formats[i].ip_version) {
			ret = copy_to_user((void __user *) sample_offset,
				supported_formats[i].sample_info, sizeof(struct kfd_pc_sample_info));
			if (ret) {
				pr_debug("Failed to copy PC sampling info to user.");
				return -EFAULT;
			}
			sample_offset += sizeof(struct kfd_pc_sample_info);
		}
	}

	return 0;
}

static int kfd_pc_sample_start(struct kfd_process_device *pdd,
					struct pc_sampling_entry *pcs_entry)
{
	bool pc_sampling_start = false;
	int ret = 0;

	pcs_entry->enabled = true;
	mutex_lock(&pdd->dev->pcs_data.mutex);

	kfd_process_set_trap_pc_sampling_flag(&pdd->qpd, pcs_entry->method, true);

	if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
		uint32_t target_vmid;
		if (!pdd->dev->pcs_data.hosttrap_entry.base.active_count)
			pc_sampling_start = true;

		pdd->dev->pcs_data.hosttrap_entry.owner_pasid = pdd->pasid;
		pdd->dev->pcs_data.hosttrap_entry.target_vmid = pdd->qpd.vmid;
		pdd->dev->pcs_data.hosttrap_entry.trap_tba_addr = pdd->qpd.tba_addr;
		pdd->dev->pcs_data.hosttrap_entry.trap_tma_addr = pdd->qpd.tma_addr;
		pdd->dev->pcs_data.hosttrap_entry.trap_regs_programmed_vmid = 0;
		if (!pdd->dev->pcs_data.hosttrap_entry.target_vmid) {
			uint32_t resolved_vmid =
				kfd_pc_sampling_lookup_vmid_by_pasid(
					pdd->dev,
					pdd->dev->pcs_data.hosttrap_entry.owner_pasid);
			if (resolved_vmid) {
				pdd->dev->pcs_data.hosttrap_entry.target_vmid = resolved_vmid;
				pr_warn("pcs hosttrap: start resolved target_vmid=%u from owner_pasid=%u via ATC map\n",
					resolved_vmid, pdd->dev->pcs_data.hosttrap_entry.owner_pasid);
			}
		}

		target_vmid = pdd->dev->pcs_data.hosttrap_entry.target_vmid;
		pr_warn("pcs hosttrap: start enter owner_pasid=%u qpd_vmid=%u target_vmid=%u active_before=%u use_count=%u queues_empty=%u process_ref=%u pid=%d tgid=%d\n",
			pdd->dev->pcs_data.hosttrap_entry.owner_pasid, pdd->qpd.vmid,
			target_vmid, pdd->dev->pcs_data.hosttrap_entry.base.active_count,
			pdd->dev->pcs_data.hosttrap_entry.base.use_count,
			list_empty(&pdd->qpd.queues_list), pdd->process->pc_sampling_ref,
			current->pid, current->tgid);
		/* Under MES + no-CWSR, qpd->vmid can stay 0 and VMID is resolved
		 * dynamically. Program per-VMID trap registers explicitly once VMID
		 * is known so SQ_CMD host-trap has valid TBA/TMA context.
		 */
		if (target_vmid && pdd->dev->kfd2kgd->program_trap_handler_settings) {
			uint32_t xcc_id;
			uint64_t tba_addr = pdd->qpd.tba_addr;
			uint64_t tma_addr = pdd->qpd.tma_addr;

			pr_warn("pcs hosttrap: start program trap regs vmid=%u tba=0x%llx tma=0x%llx owner_pasid=%u\n",
				target_vmid, tba_addr, tma_addr,
				pdd->dev->pcs_data.hosttrap_entry.owner_pasid);
			for_each_inst(xcc_id, pdd->dev->xcc_mask)
				pdd->dev->kfd2kgd->program_trap_handler_settings(
					pdd->dev->adev, target_vmid, tba_addr, tma_addr,
					xcc_id);
			pdd->dev->pcs_data.hosttrap_entry.trap_regs_programmed_vmid = target_vmid;
		}

		pr_warn("pcs hosttrap: start owner_pasid=%u qpd_vmid=%u active_before=%u use_count=%u pid=%d tgid=%d\n",
			pdd->dev->pcs_data.hosttrap_entry.owner_pasid,
			pdd->dev->pcs_data.hosttrap_entry.target_vmid,
			pdd->dev->pcs_data.hosttrap_entry.base.active_count,
			pdd->dev->pcs_data.hosttrap_entry.base.use_count,
			current->pid, current->tgid);
		kfd_pc_sampling_dump_vmid_maps(
			pdd->dev,
			pdd->dev->pcs_data.hosttrap_entry.owner_pasid,
			"start");
		pdd->dev->pcs_data.hosttrap_entry.base.active_count++;
	} else { /* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
		if (!pdd->dev->pcs_data.stoch_entry.base.active_count)
			pc_sampling_start = true;

		pdd->dev->pcs_data.stoch_entry.base.active_count++;
	}
	mutex_unlock(&pdd->dev->pcs_data.mutex);

	while (pc_sampling_start) {
		if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
			/* true means pc_sample_thread stop is in progress */
			if (READ_ONCE(pdd->dev->pcs_data.hosttrap_entry.pc_sample_thread)) {
				usleep_range(1000, 2000);
			} else {
				ret = kfd_pc_sample_thread_start(pdd->dev);
				break;
			}
		} else {/* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
			struct amdgpu_device *adev = pdd->dev->adev;
			struct kfd_node *node = pdd->dev;
			uint64_t interval;
			uint32_t inst;

			interval = node->pcs_data.stoch_entry.base.pc_sample_info.interval;
			if (pdd->dev->kfd2kgd->setup_stoch_sampling)
				for_each_inst(inst, node->xcc_mask)
					pdd->dev->kfd2kgd->setup_stoch_sampling(adev,
					node->compute_vmid_bitmap, true,
					node->pcs_data.stoch_entry.base.pc_sample_info.type,
					interval,
					inst);
			break;
		}
	}
	return ret;
}

static int kfd_pc_sample_stop(struct kfd_process_device *pdd,
					struct pc_sampling_entry *pcs_entry)
{
	bool pc_sampling_stop = false;

	pcs_entry->enabled = false;
	mutex_lock(&pdd->dev->pcs_data.mutex);
	if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
		pdd->dev->pcs_data.hosttrap_entry.base.active_count--;
		if (!pdd->dev->pcs_data.hosttrap_entry.base.active_count) {
			pc_sampling_stop = true;
			pr_warn("pcs hosttrap: stop clear owner_pasid=%u target_vmid=%u use_count=%u pid=%d tgid=%d\n",
				pdd->dev->pcs_data.hosttrap_entry.owner_pasid,
				pdd->dev->pcs_data.hosttrap_entry.target_vmid,
				pdd->dev->pcs_data.hosttrap_entry.base.use_count,
				current->pid, current->tgid);
			pdd->dev->pcs_data.hosttrap_entry.target_vmid = 0;
			pdd->dev->pcs_data.hosttrap_entry.owner_pasid = 0;
			pdd->dev->pcs_data.hosttrap_entry.trap_regs_programmed_vmid = 0;
			pdd->dev->pcs_data.hosttrap_entry.trap_tba_addr = 0;
			pdd->dev->pcs_data.hosttrap_entry.trap_tma_addr = 0;
		}
	} else {/* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
		pdd->dev->pcs_data.stoch_entry.base.active_count--;
		if (!pdd->dev->pcs_data.stoch_entry.base.active_count)
			pc_sampling_stop = true;
	}
	mutex_unlock(&pdd->dev->pcs_data.mutex);

	kfd_process_set_trap_pc_sampling_flag(&pdd->qpd, pcs_entry->method, false);
	remap_queue(pdd->dev->dqm,
		KFD_UNMAP_QUEUES_FILTER_ALL_QUEUES, 0);

	if (pc_sampling_stop) {
		if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
			kthread_stop(pdd->dev->pcs_data.hosttrap_entry.pc_sample_thread);
		} else {/* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
			struct amdgpu_device *adev = pdd->dev->adev;
			struct kfd_node *node = pdd->dev;
			uint32_t inst;

			if (pdd->dev->kfd2kgd->setup_stoch_sampling) {
				for_each_inst(inst, node->xcc_mask)
					pdd->dev->kfd2kgd->setup_stoch_sampling(adev,
					    node->compute_vmid_bitmap, false,
					    node->pcs_data.stoch_entry.base.pc_sample_info.type,
					    0, inst);
			}
		}
	}

	return 0;
}

static int kfd_pc_sample_create(struct kfd_process_device *pdd,
					struct kfd_ioctl_pc_sample_args __user *user_args)
{
	struct kfd_pc_sample_info *supported_format = NULL;
	struct kfd_pc_sample_info user_info;
	struct pc_sampling_entry *pcs_entry;
	int ret;
	int i;

	if (user_args->num_sample_info != 1)
		return -EINVAL;

	ret = copy_from_user(&user_info, (void __user *) user_args->sample_info_ptr,
				sizeof(struct kfd_pc_sample_info));
	if (ret) {
		pr_debug("Failed to copy PC sampling info from user\n");
		return -EFAULT;
	}

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (KFD_GC_VERSION(pdd->dev) == supported_formats[i].ip_version
			&& user_info.method == supported_formats[i].sample_info->method
			&& user_info.type == supported_formats[i].sample_info->type
			&& user_info.interval <= supported_formats[i].sample_info->interval_max
			&& user_info.interval >= supported_formats[i].sample_info->interval_min) {
			supported_format =
				(struct kfd_pc_sample_info *)supported_formats[i].sample_info;
			break;
		}
	}

	if (!supported_format) {
		pr_debug("Sampling format is not supported!");
		return -EOPNOTSUPP;
	}

	if (supported_format->flags == KFD_IOCTL_PCS_FLAG_POWER_OF_2 &&
		user_info.interval & (user_info.interval - 1)) {
		pr_debug("Sampling interval's power is unmatched!");
		return -EINVAL;
	}

	mutex_lock(&pdd->dev->pcs_data.mutex);
	if (supported_format->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
		if (pdd->dev->pcs_data.hosttrap_entry.base.use_count &&
			memcmp(&pdd->dev->pcs_data.hosttrap_entry.base.pc_sample_info,
					&user_info, sizeof(user_info))) {
			ret = copy_to_user((void __user *) user_args->sample_info_ptr,
				&pdd->dev->pcs_data.hosttrap_entry.base.pc_sample_info,
				sizeof(struct kfd_pc_sample_info));
			mutex_unlock(&pdd->dev->pcs_data.mutex);
			return ret ? -EFAULT : -EEXIST;
			}
	} else { /* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
		if (pdd->dev->pcs_data.stoch_entry.base.use_count &&
			memcmp(&pdd->dev->pcs_data.stoch_entry.base.pc_sample_info,
					&user_info, sizeof(user_info))) {
			ret = copy_to_user((void __user *) user_args->sample_info_ptr,
				&pdd->dev->pcs_data.stoch_entry.base.pc_sample_info,
				sizeof(struct kfd_pc_sample_info));
			mutex_unlock(&pdd->dev->pcs_data.mutex);
			return ret ? -EFAULT : -EEXIST;
		}
	}

	pcs_entry = kzalloc(sizeof(*pcs_entry), GFP_KERNEL);
	if (!pcs_entry) {
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		return -ENOMEM;
	}

	i = idr_alloc_cyclic(&pdd->dev->pcs_data.sampling_idr,
				pcs_entry, 1, 0, GFP_KERNEL);
	if (i < 0) {
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		kfree(pcs_entry);
		return i;
	}

	if (supported_format->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
		if (!pdd->dev->pcs_data.hosttrap_entry.base.use_count)
			pdd->dev->pcs_data.hosttrap_entry.base.pc_sample_info = user_info;
		pdd->dev->pcs_data.hosttrap_entry.base.use_count++;
	} else if (supported_format->method == KFD_IOCTL_PCS_METHOD_STOCHASTIC) {
		if (!pdd->dev->pcs_data.stoch_entry.base.use_count)
			pdd->dev->pcs_data.stoch_entry.base.pc_sample_info = user_info;
		pdd->dev->pcs_data.stoch_entry.base.use_count++;
	}

	mutex_unlock(&pdd->dev->pcs_data.mutex);

	pcs_entry->pdd = pdd;
	pcs_entry->method = supported_format->method;
	user_args->trace_id = (uint32_t)i;

	/*
	 * Set SPI_GDBG_PER_VMID_CNTL.TRAP_EN so that TTMP registers are valid in the sampling data
	 * p->runtime_info.ttmp_setup will be cleared when user application calls runtime_disable
	 * on exit.
	 */
	kfd_dbg_enable_ttmp_setup(pdd->process);
	pdd->process->pc_sampling_ref++;
	pr_warn("pcs hosttrap: create session trace_id=0x%x method=%u pasid=%u qpd_vmid=%u pc_sampling_ref=%u pid=%d tgid=%d\n",
		i, pcs_entry->method, pdd->pasid, pdd->qpd.vmid,
		pdd->process->pc_sampling_ref, current->pid, current->tgid);

	pr_debug("alloc pcs_entry = %p, trace_id = 0x%x method = %d on gpu 0x%x",
				pcs_entry, i, pcs_entry->method, pdd->dev->id);

	return 0;
}

static int kfd_pc_sample_destroy(struct kfd_process_device *pdd, uint32_t trace_id,
					struct pc_sampling_entry *pcs_entry)
{
	pr_debug("free pcs_entry = %p, trace_id = 0x%x on gpu 0x%x",
		pcs_entry, trace_id, pdd->dev->id);

	pdd->process->pc_sampling_ref--;
	pr_warn("pcs hosttrap: destroy session trace_id=0x%x method=%u pasid=%u qpd_vmid=%u pc_sampling_ref=%u pid=%d tgid=%d\n",
		trace_id, pcs_entry->method, pdd->pasid, pdd->qpd.vmid,
		pdd->process->pc_sampling_ref, current->pid, current->tgid);
	mutex_lock(&pdd->dev->pcs_data.mutex);
	if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
		pdd->dev->pcs_data.hosttrap_entry.base.use_count--;
		if (!pdd->dev->pcs_data.hosttrap_entry.base.use_count)
			memset(&pdd->dev->pcs_data.hosttrap_entry.base.pc_sample_info, 0x0,
				sizeof(struct kfd_pc_sample_info));
	} else { /* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
		pdd->dev->pcs_data.stoch_entry.base.use_count--;
		if (!pdd->dev->pcs_data.stoch_entry.base.use_count)
			memset(&pdd->dev->pcs_data.stoch_entry.base.pc_sample_info, 0x0,
				sizeof(struct kfd_pc_sample_info));
	}

	idr_remove(&pdd->dev->pcs_data.sampling_idr, trace_id);
	mutex_unlock(&pdd->dev->pcs_data.mutex);

	kfree(pcs_entry);

	return 0;
}

void kfd_pc_sample_release(struct kfd_process_device *pdd)
{
	struct pc_sampling_entry *pcs_entry;
	struct idr *idp;
	uint32_t id;

	/* force to release all PC sampling task for this process */
	idp = &pdd->dev->pcs_data.sampling_idr;
	do {
		pcs_entry = NULL;
		mutex_lock(&pdd->dev->pcs_data.mutex);
		idr_for_each_entry(idp, pcs_entry, id) {
			if (pcs_entry->pdd != pdd)
				continue;
			break;
		}
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		if (pcs_entry) {
			if (pcs_entry->enabled)
				kfd_pc_sample_stop(pdd, pcs_entry);
			kfd_pc_sample_destroy(pdd, id, pcs_entry);
		}
	} while (pcs_entry);
}

int kfd_pc_sample(struct kfd_process_device *pdd,
					struct kfd_ioctl_pc_sample_args __user *args)
{
	struct pc_sampling_entry *pcs_entry;

	if (args->op != KFD_IOCTL_PCS_OP_QUERY_CAPABILITIES &&
		args->op != KFD_IOCTL_PCS_OP_CREATE) {

		mutex_lock(&pdd->dev->pcs_data.mutex);
		pcs_entry = idr_find(&pdd->dev->pcs_data.sampling_idr,
				args->trace_id);
		mutex_unlock(&pdd->dev->pcs_data.mutex);

		/* pcs_entry is only for this pc sampling process,
		 * which has kfd_process->mutex protected here.
		 */
		if (!pcs_entry ||
			pcs_entry->pdd != pdd)
			return -EINVAL;
	} else if (pdd->process->debug_trap_enabled) {
		pr_debug("Cannot have PC Sampling and debug trap simultaneously");
		return -EBUSY;
	}

	switch (args->op) {
	case KFD_IOCTL_PCS_OP_QUERY_CAPABILITIES:
		return kfd_pc_sample_query_cap(pdd, args);

	case KFD_IOCTL_PCS_OP_CREATE:
		return kfd_pc_sample_create(pdd, args);

	case KFD_IOCTL_PCS_OP_DESTROY:
		if (pcs_entry->enabled)
			return -EBUSY;
		else
			return kfd_pc_sample_destroy(pdd, args->trace_id, pcs_entry);

	case KFD_IOCTL_PCS_OP_START:
		if (pcs_entry->enabled)
			return -EALREADY;
		else
			return kfd_pc_sample_start(pdd, pcs_entry);

	case KFD_IOCTL_PCS_OP_STOP:
		if (!pcs_entry->enabled)
			return -EALREADY;
		else
			return kfd_pc_sample_stop(pdd, pcs_entry);
	}

	return -EINVAL;
}
