CONFIG_MODULE_SIG = n
ifdef CONFIG_VIDEO_CAMERA
KBUILD_CFLAGS += -D_INNO_FOR_NVIDIA_
OBJECTS = inno_common.o inno_common_nv.o inno_common_mode_tbls.o inno_iic.o inno_max_gmsl.o
else
KBUILD_CFLAGS += -D_INNO_FOR_NXP_
OBJECTS = inno_common.o inno_common_nxp.o inno_common_mode_tbls.o inno_iic.o
endif

obj-$(CONFIG_VIDEO_INNO_EV2M_GOM1) += inno_ev2m_gom1.o
inno_ev2m_gom1-objs := inno_ev2m_gom1_core.o $(OBJECTS)

obj-$(CONFIG_VIDEO_INNO_EV2M_OOM1) += inno_ev2m_oom1.o
inno_ev2m_oom1-objs := inno_ev2m_oom1_core.o $(OBJECTS)

obj-$(CONFIG_VIDEO_INNO_EVDM_OOM1) += inno_evdm_oom1.o
inno_evdm_oom1-objs := inno_evdm_oom1_core.o $(OBJECTS)

obj-$(CONFIG_VIDEO_INNO_EV2M_OOM2) += inno_ev2m_oom2.o
inno_ev2m_oom2-objs := inno_ev2m_oom2_core.o $(OBJECTS)

obj-$(CONFIG_VIDEO_INNO_EV8M_OOM1) += inno_ev8m_oom1.o
inno_ev8m_oom1-objs := inno_ev8m_oom1_core.o $(OBJECTS)
