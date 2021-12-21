
virtio-video-objs := \
	virtio_video_driver.o \
	virtio_video_vq.o \
	virtio_video_device.o \
	virtio_video_dec.o \
	virtio_video_enc.o \
	virtio_video_cam.o \
	virtio_video_caps.o \
	virtio_video_helpers.o

obj-$(CONFIG_VIRTIO_VIDEO) += virtio-video.o

ccflags-y += -DDRIVER_VERSION=\"2.3.0\"
