# SPDX-License-Identifier: GPL-2.0-only
tcc803x-drm-y := tcc803x_drm_drv.o

obj-$(CONFIG_DRM_TCC803X) += tcc803x-drm.o

# Separate module, not linked into tcc803x-drm-y: both files use the
# module_platform_driver()/drm_module_platform_driver() convenience
# macros, which each define a module_init()/module_exit() pair. Two of
# those in one module object would collide (init_module/cleanup_module
# aliasing) when built with CONFIG_DRM_TCC803X=m.
obj-$(CONFIG_DRM_TCC803X) += tcc803x_drm_vioc.o
