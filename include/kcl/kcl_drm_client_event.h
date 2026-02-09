/* SPDX-License-Identifier: MIT */
#ifndef KCL_KCL_DRM_CLIENT_EVENT_H
#define KCL_KCL_DRM_CLIENT_EVENT_H

#include <linux/version.h>
#include <drm/drm_fb_helper.h>

#ifndef HAVE_DRM_CLIENT_DEV_RESUME
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 19, 0)
void drm_client_dev_suspend(struct drm_device *dev);
void drm_client_dev_resume(struct drm_device *dev);
#else
void drm_client_dev_suspend(struct drm_device *dev, bool holds_console_lock);
void drm_client_dev_resume(struct drm_device *dev, bool holds_console_lock);
#endif
#endif

#endif
