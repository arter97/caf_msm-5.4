/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/list.h>
#include "cam_smmu_api.h"
#include "../isp/msm_buf_mgr.h"

#ifndef SLEEP_MILLI_SEC
#define SLEEP_MILLI_SEC(nMilliSec)\
    do { \
        long timeout = (nMilliSec) * HZ / 1000; \
        while(timeout > 0) \
          { \
              timeout = schedule_timeout(timeout); \
          } \
    }while(0);
#endif

#define  EARLYCAMERA_IS_OPEN_ON_LK              1

#define  ARRIVAL_SIGNAL_GPIO                    37

#define  VIDIOC_MSM_EARLYCAMERA_INIT_BUF        0x1002
#define  VIDIOC_MSM_EARLYCAMERA_REQUEST_BUF     0x1003
#define  VIDIOC_MSM_EARLYCAMERA_RELEASE_BUF     0x1004
#define  VIDIOC_MSM_EARLYCAMERA_GET_SHOW_CAMERA 0x1009
#define  VIDIOC_MSM_EARLYCAMERA_QBUF            0x1010
#define  VIDIOC_MSM_EARLYCAMERA_DQBUF           0x1011

#define MSM_EARLYCAMERA_DRV_NAME     "earlycamera"

#define FRVC_CAMERA_STATUS_REG       0x08600680UL
#define FRVC_CAMERA_IS_ENABLED       0x5a5a5a5a
#define FRVC_CAMERA_IS_WORKING       0xF5F5F5F5
#define FRVC_CAMERA_IS_DONE          0xa5a5a5a5

#define FRVC_CAMERA_NOTIFY_LK_REG      0x08600688UL
#define KERNEL_REQUSET_STOP_FRVC       0xFEFEFEFE
#define KERNEL_REQUEST_PAUSE_FRVC      0xF0F0F0F0
#define FRVC_NOTIFY_KERNEL_WAIT_EXIT   0x8D8D8D8D
#define FRVC_NOTIFY_KERNEL_EXIT_DONE   0x7C7C7C7C

#define FRVC_DISPLAY_STATUS_REG              0x08600684UL
#define FRVC_DISPLAY_IS_ENABLED              0x5a5a5a5a
#define FRVC_DISPLAY_IS_WORKING              0xF5F5F5F5
#define FRVC_DISPLAY_IS_DONE                 0xa5a5a5a5
#define FRVC_NOTIFY_ANDROID_SHOW_CAMERA      0x8B8B8B8B
#define FRVC_NOTIFY_ANDROID_NOT_SHOW_CAMERA  0x7D7D7D7D

#define VFE_BASE                     0x01b14000UL
#define FRVC_CAMERA_PING_ADDR_0      (VFE_BASE + 0x70)
#define FRVC_CAMERA_PING_ADDR_1      (VFE_BASE + 0x94)
#define FRVC_CAMERA_PONG_ADDR_0      (VFE_BASE + 0x74)
#define FRVC_CAMERA_PONG_ADDR_1      (VFE_BASE + 0x98)

#define frvc_read(addr)             readl(addr)
#define frvc_write(value,addr)      writel((value), addr)

static void __iomem *vfe_irq0_base = NULL;
static void __iomem *vfe_irq1_base = NULL;
static void __iomem *vfe_irq0_clear_base = NULL;
static void __iomem *vfe_irq1_clear_base = NULL;
static void __iomem *vfe_pingpong_status_base = NULL;
static void __iomem *vfe_irq_status_base = NULL;
static void __iomem *vfe_ping_addr_0_base = NULL;
static void __iomem *vfe_pong_addr_0_base = NULL;
static void __iomem *vfe_ping_addr_1_base = NULL;
static void __iomem *vfe_pong_addr_1_base = NULL;
static void __iomem *frvc_camera_status_base = NULL;

static void __iomem *frvc_camera_notify_lk_base = NULL;
static void __iomem *frvc_display_status_base = NULL;

struct earlycamera_hal_buffer_cfg{
  int fd;
  int offset;
  int num_planes;
  int pingpong_flag;
  int num_bufs;
  int idx;
  void* pddr;
};

struct earlycamera_isp_buffer{
  int num_planes;
  struct msm_isp_buffer_mapped_info mapped_info[2];
  int buf_idx;
  uint32_t fd;
  uint32_t pingpong_bit;
  struct list_head list;
  int status;
};

struct earlycamera_isp_bufq{
  uint32_t num_bufs;
  int iommu_hdl;
  int ping_status;
  int pong_status;
  uint32_t buffer_is_release_done;
  int gpio_irq_handle;
  struct workqueue_struct *working_queue;
  struct work_struct wq;
  struct mutex mutex;
  struct earlycamera_isp_buffer *bufs;
  struct list_head head;
};

enum earlycamera_buffer_status {
  MSM_BUFFER_EMPTY = 1,
  MSM_BUFFER_SET,
  MSM_BUFFER_DATA_FULL,
  MSM_BUFFER_PING_SET,
  MSM_BUFFER_PONG_SET,
  MSM_BUFFER_BUSY,
  MSM_BUFFER_IDLE,
};

struct earlycamera_kernel_buffer_cfg* buffe_cfg = NULL;
struct earlycamera_isp_bufq isp_bufq;

void __iomem *phy_addrchangetovirtual_addr(unsigned long phy_addr);

static int major = 0;
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static struct class *earlycamera_class = NULL;
static struct device *earlycamera_device = NULL;

static struct task_struct *thead_earlycamera_LK_status_task = NULL;
unsigned long Android_launcher_status = 0;
unsigned long Notify_display_pause = 0;

#define EARLYCAMERA_OPEN   1
#define EARLYCAMERA_CLOSE  0

int check_earlycamera_is_open(unsigned int frvc_camera_status){
  if(frvc_camera_status != FRVC_CAMERA_IS_ENABLED &&
     frvc_camera_status != FRVC_CAMERA_IS_WORKING &&
     frvc_camera_status != FRVC_CAMERA_IS_DONE){
    return EARLYCAMERA_CLOSE;
  }
  else
    return EARLYCAMERA_OPEN;
}

static ssize_t msm_earlycamera_status_show(struct device *dev,struct device_attribute *attr, char *buf)
{
  return scnprintf(buf, PAGE_SIZE, "%ld\n", Android_launcher_status);
}

static ssize_t msm_earlycamera_status_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{

  Android_launcher_status = simple_strtoul(buf, NULL, 10);
  if(Android_launcher_status == 1){
    printk("Android init done,notify stop frvc\n");
    if (frvc_camera_notify_lk_base != NULL) {
       frvc_write(KERNEL_REQUSET_STOP_FRVC,frvc_camera_notify_lk_base);
    }
  }
  return count;
}

static ssize_t msm_earlycamera_lk_notify_display_pause_show(struct device *dev,struct device_attribute *attr, char *buf)
{
  return scnprintf(buf, PAGE_SIZE, "%ld\n", Notify_display_pause);
}

static ssize_t msm_earlycamera_lk_notify_display_pause_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
  unsigned long pause = 0;
  unsigned int display_status = 0;
  int wait_time = 0;
  unsigned int camera_status = 0;

  pause = simple_strtoul(buf, NULL, 10);

  if(pause == 1){
    printk("Android init done,notify pause frvc display\n");
    frvc_write(KERNEL_REQUEST_PAUSE_FRVC,frvc_camera_notify_lk_base);

    camera_status = frvc_read(frvc_camera_status_base);
    if(check_earlycamera_is_open(camera_status) == EARLYCAMERA_CLOSE){
      printk("earlycamera is not open\n");
      Notify_display_pause = pause;
      return count;
    }

    do{
      display_status = frvc_read(frvc_display_status_base);
      wait_time++;
      mdelay(20);
    }while(((display_status != FRVC_NOTIFY_ANDROID_SHOW_CAMERA) &&
            (display_status != FRVC_NOTIFY_ANDROID_NOT_SHOW_CAMERA) &&
            (display_status != FRVC_DISPLAY_IS_ENABLED)) && wait_time < 100);
    Notify_display_pause = pause;
  }

  if(wait_time > 100){
    Notify_display_pause = 0;
    count = 0;
  }

  return count;
}

static ssize_t msm_earlycamera_lk_notify_show_camera_show(struct device *dev,struct device_attribute *attr, char *buf)
{
      uint32_t display_status = 0;
      unsigned int rc = 0;

      if(frvc_display_status_base == NULL){
        printk(" get earlycamera display status ioremap failed\n");
        return -1;
      }
      display_status = frvc_read(frvc_display_status_base);

      if(display_status == FRVC_NOTIFY_ANDROID_SHOW_CAMERA){
        rc = 1;
      }
      else if(display_status == FRVC_NOTIFY_ANDROID_NOT_SHOW_CAMERA){
        rc = 0;
      }
      sprintf(buf,"%d\n", rc);
      return 0;
}

static ssize_t earlycamera_lk_notify_show_camera_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
      return count;
}

static DEVICE_ATTR(earlycamera_status, S_IRUGO | S_IWUSR,msm_earlycamera_status_show,msm_earlycamera_status_store);
static DEVICE_ATTR(earlycamera_lk_notify_display_pause, S_IRUGO | S_IWUSR,msm_earlycamera_lk_notify_display_pause_show,msm_earlycamera_lk_notify_display_pause_store);
static DEVICE_ATTR(earlycamera_lk_notify_show_camera, S_IRUGO | S_IWUSR,msm_earlycamera_lk_notify_show_camera_show,earlycamera_lk_notify_show_camera_store);

int earlycamera_vfe_reg_ioremap(void){

  vfe_irq0_base = phy_addrchangetovirtual_addr(VFE_BASE + 0x38);
  if(vfe_irq0_base == NULL){
    printk("failed to vfe irq0 ioremap\n");
    goto failed_irq0;
  }

  vfe_irq1_base = phy_addrchangetovirtual_addr(VFE_BASE + 0x3c);
  if(vfe_irq1_base == NULL){
    printk("failed to vfe irq1 ioremap\n");
    goto failed_irq1;
  }

  vfe_pingpong_status_base = phy_addrchangetovirtual_addr(VFE_BASE + 0x268);
  if(vfe_pingpong_status_base == NULL){
    printk("failed to vfe irq1 ioremap\n");
    goto failed_pingpong_status;
  }

  vfe_ping_addr_0_base = phy_addrchangetovirtual_addr(FRVC_CAMERA_PING_ADDR_0);
  if(vfe_ping_addr_0_base == NULL){
    printk("failed to vfe vfe_ping_addr_0_base ioremap\n");
    goto failed_ping_addr_0;
  }
  vfe_ping_addr_1_base = phy_addrchangetovirtual_addr(FRVC_CAMERA_PING_ADDR_1);
  if(vfe_ping_addr_1_base == NULL){
    printk("failed to vfe vfe_ping_addr_1_base ioremap\n");
    goto failed_ping_addr_1;
  }
  vfe_pong_addr_0_base = phy_addrchangetovirtual_addr(FRVC_CAMERA_PONG_ADDR_0);
  if(vfe_pong_addr_0_base == NULL){
    printk("failed to vfe vfe_pong_addr_0_base ioremap\n");
    goto failed_pong_addr_0;
  }
  vfe_pong_addr_1_base = phy_addrchangetovirtual_addr(FRVC_CAMERA_PONG_ADDR_1);
  if(vfe_pong_addr_1_base == NULL){
    printk("failed to vfe vfe_pong_addr_1_base ioremap\n");
    goto failed_pong_addr_1;
  }

  vfe_irq_status_base = phy_addrchangetovirtual_addr(VFE_BASE + 0x24);
  if(vfe_irq_status_base == NULL){
    printk("failed to vfe vfe_irq_status_base ioremap\n");
     goto failed_irq_status_addr;
  }

  vfe_irq0_clear_base = phy_addrchangetovirtual_addr(VFE_BASE + 0x30);
  if(vfe_irq0_clear_base == NULL){
    printk("failed to vfe vfe_irq0_clear_base ioremap\n");
    goto failed_irq0_clear_base;
  }

  vfe_irq1_clear_base = phy_addrchangetovirtual_addr(VFE_BASE + 0x34);
  if(vfe_irq1_clear_base == NULL){
    printk("failed to vfe vfe_irq1_clear_base ioremap\n");
    goto failed_irq1_clear_base;
  }
  return 0;

failed_irq1_clear_base:
  iounmap(vfe_irq0_clear_base);
failed_irq0_clear_base:
  iounmap(vfe_irq_status_base);
failed_irq_status_addr:
  iounmap(vfe_pong_addr_1_base);
failed_pong_addr_1:
   iounmap(vfe_pong_addr_0_base);
failed_pong_addr_0:
    iounmap(vfe_ping_addr_1_base);
failed_ping_addr_1:
    iounmap(vfe_ping_addr_0_base);
failed_ping_addr_0:
    iounmap(vfe_pingpong_status_base);
failed_pingpong_status:
    iounmap(vfe_irq1_base);
failed_irq1:
    iounmap(vfe_irq0_base);
failed_irq0:
    return -1;
}

void earlycamera_vfe_reg_iounmap(void){
  if(vfe_pong_addr_1_base != NULL){
    iounmap(vfe_pong_addr_1_base);
    vfe_pong_addr_1_base = NULL;
  }

  if(vfe_pong_addr_0_base != NULL){
    iounmap(vfe_pong_addr_0_base);
    vfe_pong_addr_0_base = NULL;
  }

  if(vfe_ping_addr_1_base != NULL){
    iounmap(vfe_ping_addr_1_base);
    vfe_ping_addr_1_base = NULL;
  }

  if(vfe_ping_addr_0_base != NULL){
    iounmap(vfe_ping_addr_0_base);
    vfe_ping_addr_0_base = NULL;
  }

  if(vfe_pingpong_status_base != NULL){
    iounmap(vfe_pingpong_status_base);
    vfe_pingpong_status_base = NULL;
  }

  if(vfe_irq1_base != NULL){
    iounmap(vfe_irq1_base);
    vfe_irq1_base = NULL;
  }

  if(vfe_irq0_base != NULL){
    iounmap(vfe_irq0_base);
    vfe_irq0_base = NULL;
  }

  if(vfe_irq_status_base != NULL){
    iounmap(vfe_irq_status_base);
    vfe_irq_status_base = NULL;
  }

  if(vfe_irq0_clear_base != NULL){
    iounmap(vfe_irq0_clear_base);
    vfe_irq0_clear_base = NULL;
  }

  if(vfe_irq1_clear_base != NULL){
    iounmap(vfe_irq1_clear_base);
    vfe_irq1_clear_base = NULL;
  }
}

int earlycamera_isp_map_buf(int iommu_hdl,struct msm_isp_buffer_mapped_info *mapped_info, uint32_t fd){
  int ret;
  ret = cam_smmu_get_phy_addr(iommu_hdl,
        fd,
        CAM_SMMU_MAP_RW,
        &(mapped_info->paddr),
        &(mapped_info->len));
  return ret;
}


int earlycameraisp_unmap_buf(int iommu_hdl,uint32_t fd)
{
  cam_smmu_put_phy_addr(iommu_hdl,fd);
  return 0;
}

void earlycamera_update_pingpong_addr(dma_addr_t paddr,int pingpong_status){
  unsigned int paddr32_0,paddr32_1;

  paddr32_0 = (paddr & 0xFFFFFFFF);
  paddr32_1 = ((paddr32_0 + 1920*1080) & 0xFFFFFFFF);

  if(pingpong_status){
    writel(paddr32_0,vfe_ping_addr_0_base);
    writel(paddr32_1,vfe_ping_addr_1_base);
  }
  else{
    writel(paddr32_0,vfe_pong_addr_0_base);
    writel(paddr32_1,vfe_pong_addr_1_base);
  }
}

int earlycamera_release_all_buffer(void){
  int i;
  int rc = 0;
  mutex_lock(&isp_bufq.mutex);

  earlycamera_vfe_reg_iounmap();

  for(i = 0; i < isp_bufq.num_bufs; i++){
    if(isp_bufq.bufs[i].fd != -1){
      rc = earlycameraisp_unmap_buf(isp_bufq.iommu_hdl,isp_bufq.bufs[i].fd);
    }
  }

  if(isp_bufq.iommu_hdl > 0){
      cam_smmu_ops(isp_bufq.iommu_hdl, CAM_SMMU_DETACH);
      cam_smmu_destroy_handle(isp_bufq.iommu_hdl);
      isp_bufq.iommu_hdl = -1;
  }

  if(isp_bufq.bufs != NULL){
    kfree(isp_bufq.bufs);
    isp_bufq.bufs = NULL;
  }
  printk("earlycamera_release_all_buffer.......\n");
  isp_bufq.buffer_is_release_done = 1;
  mutex_unlock(&isp_bufq.mutex);
  return rc;
}

static long earlycamera_ioctl(struct file* filp, unsigned int cmd,unsigned long arg){
  int rc = 0;
  if(EARLYCAMERA_IS_OPEN_ON_LK == 0){
    return rc;
  }
  switch (cmd) {
    case VIDIOC_MSM_EARLYCAMERA_INIT_BUF:
    {
      struct earlycamera_hal_buffer_cfg* cfg = (struct earlycamera_hal_buffer_cfg*)arg;
      int i;

      if(earlycamera_vfe_reg_ioremap() != 0){
        printk("failed to  earlycamera_vfe_reg_ioremap \n");
        rc = -1;
        return -1;
      }
      isp_bufq.num_bufs = cfg->num_bufs;
      isp_bufq.ping_status = MSM_BUFFER_EMPTY;
      isp_bufq.pong_status = MSM_BUFFER_EMPTY;
      isp_bufq.iommu_hdl = -1;
      isp_bufq.buffer_is_release_done = 0;

      mutex_lock(&isp_bufq.mutex);
      isp_bufq.bufs = kmalloc(sizeof(struct earlycamera_isp_buffer) * isp_bufq.num_bufs,GFP_KERNEL);
      mutex_unlock(&isp_bufq.mutex);
      if(!isp_bufq.bufs){
        printk("No free memory for buf info\n");
        rc = -ENOMEM;
        goto malloc_failed;
      }

      for(i = 0; i < isp_bufq.num_bufs; i++){
        isp_bufq.bufs[i].fd = -1;
        isp_bufq.bufs[i].status = MSM_BUFFER_IDLE;
      }

      rc = cam_smmu_get_handle("vfe", &isp_bufq.iommu_hdl);
      if(rc < 0){
        rc = -1;
        goto smmu_handle_failed;
      }
      rc = cam_smmu_ops(isp_bufq.iommu_hdl, CAM_SMMU_ATTACH);
      if(rc < 0){
        rc = -1;
        goto smmu_ops_failed;
      }
      break;

      smmu_ops_failed:
          cam_smmu_ops(isp_bufq.iommu_hdl, CAM_SMMU_DETACH);
      smmu_handle_failed:
          cam_smmu_destroy_handle(isp_bufq.iommu_hdl);
          kfree(isp_bufq.bufs);
      malloc_failed:
          earlycamera_vfe_reg_iounmap();
    }
    break;
    case VIDIOC_MSM_EARLYCAMERA_REQUEST_BUF:
    {
      struct earlycamera_hal_buffer_cfg* cfg = (struct earlycamera_hal_buffer_cfg*)arg;
      struct msm_isp_buffer_mapped_info* mapped_info;

      mutex_lock(&isp_bufq.mutex);
      isp_bufq.bufs[cfg->idx].buf_idx = cfg->idx;
      isp_bufq.bufs[cfg->idx].pingpong_bit = cfg->pingpong_flag;
      isp_bufq.bufs[cfg->idx].num_planes = cfg->num_planes;
      isp_bufq.bufs[cfg->idx].fd = cfg->fd;
      isp_bufq.bufs[cfg->idx].status = MSM_BUFFER_IDLE;

      mapped_info = &isp_bufq.bufs[cfg->idx].mapped_info[0];
      mapped_info->buf_fd = cfg->fd;
      rc = earlycamera_isp_map_buf(isp_bufq.iommu_hdl,mapped_info,mapped_info->buf_fd);
      if(rc < 0){
        printk(" %s: fd %d got phy addr error %d\n", __func__, mapped_info->buf_fd,rc);
        rc = -1;
      }
      mutex_unlock(&isp_bufq.mutex);

    }
    break;
    case VIDIOC_MSM_EARLYCAMERA_QBUF:
    {
      uint32_t notify_camera_status;
      struct earlycamera_hal_buffer_cfg* cfg = (struct earlycamera_hal_buffer_cfg*)arg;

      mutex_lock(&isp_bufq.mutex);
      notify_camera_status = frvc_read(frvc_camera_notify_lk_base);

      if(isp_bufq.bufs  == NULL){
        mutex_unlock(&isp_bufq.mutex);
        return -1;
      }

      isp_bufq.bufs[cfg->idx].status = MSM_BUFFER_EMPTY;
      if(isp_bufq.ping_status == MSM_BUFFER_EMPTY){
        earlycamera_update_pingpong_addr(isp_bufq.bufs[cfg->idx].mapped_info[0].paddr,1);
        isp_bufq.bufs[cfg->idx].status = MSM_BUFFER_PING_SET;
        isp_bufq.ping_status = MSM_BUFFER_SET;
      }
      else if(isp_bufq.pong_status == MSM_BUFFER_EMPTY){
        earlycamera_update_pingpong_addr(isp_bufq.bufs[cfg->idx].mapped_info[0].paddr,0);
        isp_bufq.bufs[cfg->idx].status = MSM_BUFFER_PONG_SET;
        isp_bufq.pong_status = MSM_BUFFER_SET;
      }
      mutex_unlock(&isp_bufq.mutex);
    }
    break;
    case  VIDIOC_MSM_EARLYCAMERA_DQBUF:
    {
      int i;
      uint32_t notify_camera_status;
      struct earlycamera_hal_buffer_cfg* cfg = (struct earlycamera_hal_buffer_cfg*)arg;
      mutex_lock(&isp_bufq.mutex);
      notify_camera_status = frvc_read(frvc_camera_notify_lk_base);

      if(isp_bufq.bufs  == NULL){
        mutex_unlock(&isp_bufq.mutex);
        return -1;
      }

      for(i = 0; i < isp_bufq.num_bufs; i++){
        if(isp_bufq.bufs[i].status == MSM_BUFFER_DATA_FULL){
          isp_bufq.bufs[i].status = MSM_BUFFER_BUSY;
          cfg->idx = i;
          break;
        }
      }
      mutex_unlock(&isp_bufq.mutex);
    }
    break;
    case VIDIOC_MSM_EARLYCAMERA_RELEASE_BUF:
    {
      earlycamera_release_all_buffer();
      rc = 0;
    }
    break;
    case VIDIOC_MSM_EARLYCAMERA_GET_SHOW_CAMERA:
    {
      uint32_t display_status = 0;

      display_status = frvc_read(frvc_display_status_base);

      if(display_status == FRVC_NOTIFY_ANDROID_SHOW_CAMERA){
        rc = 1;
      }
      else{
        rc = 0;
      }
    }
    break;
    default:
      rc = -1;
    break;
  }
  return rc;
}

static const struct file_operations earlycamera_fops = {
  .unlocked_ioctl = earlycamera_ioctl,
	.owner		= THIS_MODULE,
};


static int earlycamera_check_buffer_status(int pingping_status){
  int i;
  int temp;
  if(isp_bufq.bufs == NULL){
    goto out;
  }

  if(pingping_status){
    temp = MSM_BUFFER_PING_SET;
  }else{
    temp = MSM_BUFFER_PONG_SET;
  }

  for(i = 0; i < isp_bufq.num_bufs; i++){
    if(isp_bufq.bufs[i].status == MSM_BUFFER_DATA_FULL){
      isp_bufq.bufs[i].status = MSM_BUFFER_EMPTY;
    }
  }

  for(i = 0; i < isp_bufq.num_bufs; i++){
    if(isp_bufq.bufs[i].status == temp){
      isp_bufq.bufs[i].status = MSM_BUFFER_DATA_FULL;
      break;
    }
  }
  // if(pingping_status){
  //   isp_bufq.ping_status = MSM_BUFFER_EMPTY;
  // }else{
  //   isp_bufq.pong_status = MSM_BUFFER_EMPTY;
  // }

  for(i = 0; i < isp_bufq.num_bufs; i++){
    if(isp_bufq.bufs[i].status == MSM_BUFFER_EMPTY){
      earlycamera_update_pingpong_addr(isp_bufq.bufs[i].mapped_info[0].paddr,pingping_status);
      isp_bufq.bufs[i].status = temp;
      if(pingping_status){
        isp_bufq.ping_status = MSM_BUFFER_SET;
      }else{
        isp_bufq.pong_status = MSM_BUFFER_SET;
      }
      break;
    }
  }
out:
  return 0;
}

static int earlycamera_process_irq(void){
  uint32_t irq0,irq1;
  uint32_t pingpong;
  irq1 = 0;

  do{
    mutex_lock(&isp_bufq.mutex);
    if(isp_bufq.bufs == NULL){
      mdelay(5);
      mutex_unlock(&isp_bufq.mutex);
      return 0;
    }
    if(vfe_irq0_base != NULL)
      irq0 = frvc_read(vfe_irq0_base);
    else{
      mutex_unlock(&isp_bufq.mutex);
      return 0;
    }
    mutex_unlock(&isp_bufq.mutex);
    mdelay(5);
  }while(!((irq0 >> 25) & 0xF));
  mutex_lock(&isp_bufq.mutex);

  if(vfe_irq0_clear_base != NULL &&
    vfe_irq1_clear_base != NULL &&
    vfe_irq_status_base != NULL &&
    vfe_pingpong_status_base != NULL &&
    vfe_irq1_base != NULL){

    pingpong = frvc_read(vfe_pingpong_status_base);
    irq1 = frvc_read(vfe_irq1_base);
    frvc_write(irq0, vfe_irq0_clear_base);
    frvc_write(irq1, vfe_irq1_clear_base);
    frvc_write(1, vfe_irq_status_base);
  }else{
    mutex_unlock(&isp_bufq.mutex);
    return 0;
  }

  earlycamera_check_buffer_status(pingpong & 0x1);
  mutex_unlock(&isp_bufq.mutex);
  return pingpong;
}

static int thead_earlycamera_LK_status(void *arg)
{
  unsigned int frvc_camera_status = 0;
  char* s_c[2];
  uint32_t pingpong = 0;
  unsigned int notify_camera_status = 0;
  struct device *earlycamera_uevent_device = (struct device *)arg;

  while(1){
    frvc_camera_status = frvc_read(frvc_camera_status_base);
    notify_camera_status = frvc_read(frvc_camera_notify_lk_base);
    if(check_earlycamera_is_open(frvc_camera_status) == EARLYCAMERA_CLOSE ||
      EARLYCAMERA_IS_OPEN_ON_LK == 0){
      if(KERNEL_REQUSET_STOP_FRVC == notify_camera_status ||
         FRVC_NOTIFY_KERNEL_WAIT_EXIT == notify_camera_status ||
         FRVC_NOTIFY_KERNEL_EXIT_DONE == notify_camera_status){
        printk("LK_status earlycamera is not open\n");
        break;
      }
      mdelay(10);
      continue;
    }

    if(FRVC_NOTIFY_KERNEL_WAIT_EXIT == notify_camera_status ||
       FRVC_CAMERA_IS_DONE == frvc_camera_status){
      break;
    }

    pingpong = earlycamera_process_irq();
  }

  printk("thead_earlycamera_LK_status wait stop camera ......\n");
  if(frvc_camera_notify_lk_base != NULL){
    frvc_write(FRVC_NOTIFY_KERNEL_EXIT_DONE,frvc_camera_notify_lk_base);
  }

  while(1){
    if(EARLYCAMERA_IS_OPEN_ON_LK == 0)
      break;
    frvc_camera_status = frvc_read(frvc_camera_status_base);
    if(frvc_camera_status == FRVC_CAMERA_IS_DONE){
      break;
    }
    mdelay(5);
  }
  if(EARLYCAMERA_IS_OPEN_ON_LK == 1){
    while(1){
      if(isp_bufq.buffer_is_release_done == 1){
        break;
      }
      mdelay(5);
    }
  }

  printk("thead_earlycamera_LK_status camera is exit......\n");

  s_c[0] = "CAMERA_STATUS=frvc_camera_exit";
  s_c[1] = NULL;
  if(earlycamera_uevent_device != NULL){
    kobject_uevent_env(&earlycamera_uevent_device->kobj,KOBJ_CHANGE, s_c);
  }else{
    kobject_uevent_env(&earlycamera_device->kobj,KOBJ_CHANGE, s_c);
  }

  printk("thead_earlycamera_LK_status enable gpio irq......\n");

  enable_irq(isp_bufq.gpio_irq_handle);
  do_exit(0);
  return 0;
}

static irqreturn_t earlycamera_check_arrival_gpio_irq(int irq, void *data){
  queue_work(isp_bufq.working_queue, &isp_bufq.wq);
  return IRQ_HANDLED;
}

int create_request_arrival_gpio_irq(int gpio_num,struct device *dev){
  int ret;
  if (gpio_is_valid(gpio_num)){
    ret = gpio_request(gpio_num, "earlycamera_irq_gpio");
    if(ret){
      printk("create_request_gpio_irq: gpio_request(%d )failed...\n",gpio_num);
      return -1;
    }
    isp_bufq.gpio_irq_handle = gpio_to_irq(gpio_num);
    if(isp_bufq.gpio_irq_handle < 0){
      printk("create_request_gpio_irq: gpio_to_irq(%d )failed...\n",gpio_num);
      goto err_free_irq_gpio;
    }
  }
  ret = request_irq(isp_bufq.gpio_irq_handle, earlycamera_check_arrival_gpio_irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,"earlycamera_gpio_request",dev);
  disable_irq_nosync(isp_bufq.gpio_irq_handle);
  return 0;

err_free_irq_gpio:
   if (gpio_is_valid(gpio_num)){
    gpio_free(gpio_num);
   }
   return ret;
}

void earlycamera_check_arrival_gpio_work_func(struct work_struct *work)
{
  int gpio_value;
  char* s_c[2];
  gpio_value = gpio_get_value(ARRIVAL_SIGNAL_GPIO);
  if(gpio_value == 1){
    s_c[0] = "REVERSE=1";
  }else{
    s_c[0] = "REVERSE=0";
  }
  s_c[1] = NULL;
  if(earlycamera_device != NULL){
    kobject_uevent_env(&earlycamera_device->kobj,KOBJ_CHANGE, s_c);
  }
}

void __iomem *phy_addrchangetovirtual_addr(unsigned long phy_addr){
    void __iomem *virtual_addr;
    virtual_addr = ioremap(phy_addr, 4);
    return virtual_addr;
}

static int earlycamera_probe(struct platform_device *pdev)
{
  int retval;
  int err;

  frvc_camera_status_base = phy_addrchangetovirtual_addr(FRVC_CAMERA_STATUS_REG);
  if (frvc_camera_status_base == NULL) {
    printk("frvc_camera_status_base ioremap failed \n");
    return -ENOMEM;
  }

  frvc_camera_notify_lk_base = phy_addrchangetovirtual_addr(FRVC_CAMERA_NOTIFY_LK_REG);
  if (frvc_camera_notify_lk_base == NULL) {
    printk("frvc_camera_notify_lk_base ioremap failed \n");
    return -ENOMEM;
  }


  frvc_display_status_base = phy_addrchangetovirtual_addr(FRVC_DISPLAY_STATUS_REG);
  if(frvc_display_status_base == NULL) {
     return -ENOMEM;
  }

  retval = register_chrdev(major, MSM_EARLYCAMERA_DRV_NAME, &earlycamera_fops);
  if (retval < 0) {
    return retval;
  }

  if (major == 0) {
    major = retval;
    printk(KERN_INFO "major number %d\n", major);
  }

  earlycamera_class = class_create(THIS_MODULE, MSM_EARLYCAMERA_DRV_NAME);
  if(earlycamera_class == NULL){
    printk("create earlycamera class failed!\n");
    unregister_chrdev(major, MSM_EARLYCAMERA_DRV_NAME);
    return -1;
  }

  earlycamera_device = device_create(earlycamera_class, NULL, MKDEV(major, 0), NULL, MSM_EARLYCAMERA_DRV_NAME);
  if (earlycamera_device == NULL){
    printk("failed to create device .\n");
    unregister_chrdev(major, MSM_EARLYCAMERA_DRV_NAME);
    return -1;
  }
  retval = device_create_file(earlycamera_device, &dev_attr_earlycamera_status);
  if(retval < 0){
    printk("failed to create earlycamera_status endpoint\n");
  }

  retval = device_create_file(earlycamera_device,&dev_attr_earlycamera_lk_notify_display_pause);
  if(retval < 0){
    printk("failed to create earlycamera_lk_notify_display_pause endpoint\n");
  }

  retval = device_create_file(earlycamera_device,&dev_attr_earlycamera_lk_notify_show_camera);
  if(retval < 0){
    printk("failed to create earlycamera_lk_notify_show_camera endpoint\n");
  }
  isp_bufq.bufs = NULL;
  isp_bufq.iommu_hdl = -1;
  isp_bufq.buffer_is_release_done = 0;
  isp_bufq.gpio_irq_handle = -1;
  mutex_init(&isp_bufq.mutex);

  isp_bufq.working_queue = create_singlethread_workqueue("ealycamera_wq");
  INIT_WORK(&isp_bufq.wq, earlycamera_check_arrival_gpio_work_func);

  create_request_arrival_gpio_irq(ARRIVAL_SIGNAL_GPIO,earlycamera_device);
  thead_earlycamera_LK_status_task = kthread_create(thead_earlycamera_LK_status, earlycamera_device, "thead_earlycamera_LK_status_task");
  if(IS_ERR(thead_earlycamera_LK_status_task)){
    printk("Unable to start kernel thread.\n");
    err = PTR_ERR(thead_earlycamera_LK_status_task);
    thead_earlycamera_LK_status_task = NULL;
    return err;
  }
  wake_up_process(thead_earlycamera_LK_status_task);
  return 0;
}

static int earlycamera_exit(struct platform_device *pdev)
{
  if(frvc_camera_status_base != NULL){
    iounmap(frvc_camera_status_base);
    frvc_camera_status_base = NULL;
  }

  if(frvc_camera_notify_lk_base != NULL){
    iounmap(frvc_camera_notify_lk_base);
    frvc_camera_notify_lk_base = NULL;
  }

  if(frvc_display_status_base != NULL){
    iounmap(frvc_display_status_base);
    frvc_display_status_base = NULL;
  }

  gpio_free(ARRIVAL_SIGNAL_GPIO);
  free_irq(isp_bufq.gpio_irq_handle, earlycamera_device);
  cancel_work_sync(&isp_bufq.wq);
  if(isp_bufq.working_queue){
   destroy_workqueue(isp_bufq.working_queue);
  }
  device_remove_file(earlycamera_device, &dev_attr_earlycamera_status);
  device_remove_file(earlycamera_device, &dev_attr_earlycamera_lk_notify_display_pause);
  device_remove_file(earlycamera_device, &dev_attr_earlycamera_lk_notify_show_camera);
  device_destroy(earlycamera_class, MKDEV(major, 0));
  class_destroy(earlycamera_class);
  unregister_chrdev(major, MSM_EARLYCAMERA_DRV_NAME);
  return 0;
}

static const struct of_device_id earlycamera_dt_match[] = {
	{.compatible = "qcom,earlycamera"},
	{}
};

MODULE_DEVICE_TABLE(of, earlycamera_dt_match);

static struct platform_driver earlycamera_driver = {
	.probe  = earlycamera_probe,
	.remove = earlycamera_exit,
	.driver = {
		.name = MSM_EARLYCAMERA_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = earlycamera_dt_match,
	},
};

static int __init msm_earlycamera_init_module(void)
{
	return platform_driver_register(&earlycamera_driver);
}

static void __exit msm_earlycamera_exit_module(void)
{
	platform_driver_unregister(&earlycamera_driver);
}

module_init(msm_earlycamera_init_module);
module_exit(msm_earlycamera_exit_module);
MODULE_DESCRIPTION("MSM earlycamera notify driver");
MODULE_LICENSE("GPL v2");

