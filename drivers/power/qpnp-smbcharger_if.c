#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/semaphore.h>
#include <linux/types.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/fdtable.h>
#include <linux/rcupdate.h>
#include <linux/eventfd.h>
#include <linux/delay.h>
#include <linux/kthread.h>  // for threads
#include <uapi/linux/qpnp-smbcharger_if.h>

static int tas_batt_mon_open(struct inode *inode, struct file *file);
static int tas_batt_mon_close(struct inode *inode, struct file *file);
static ssize_t tas_batt_mon_read(struct file *file,char __user *bufu, size_t count, loff_t *ppos);
static ssize_t tas_batt_mon_write(struct file *file, const char __user *, size_t count, loff_t *ppos);
static long tas_batt_mon_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int configure_eventfd(void);
void send_smbchg_event(uint64_t event);

extern uint32_t gopro_get_batt_capacity(void);
extern uint32_t gopro_get_batt_temp(void);
extern uint32_t gopro_get_batt_chg_status(void);
extern uint32_t gopro_get_batt_chg_type(void);
extern uint32_t gopro_get_batt_current(void);
extern uint32_t gopro_get_batt_volt(void);

#define MAGIC_NUMBER        		'z'
#define WRITE_GPCAM_PID            _IOW(MAGIC_NUMBER, 1, int)
#define WRITE_EVNT_FD 	           _IOW(MAGIC_NUMBER, 2, int)

#define SUCCESS				0
#define FAILURE				-1

struct tas_battery_params
{
	/* Local references */
	unsigned char opened;
	unsigned char pid_recvd;
	unsigned char evt_prgrs;
	unsigned char running;
	unsigned short gpcam_event_fd;
	unsigned short gpcam_pid;
	int major;

	struct task_struct *userspace_task; 	//...to userspace program's task struct
	struct file *efd_file;          		//...to eventfd's file struct
	struct eventfd_ctx *efd_ctx;        	//...and finally to eventfd context
	struct class *tas_bat_mon_class;
	struct device *tas_bat_mon_dev;	
	struct task_struct *thread;
	struct semaphore battery_mon_sem;
};
struct tas_battery_params *obj_tas_battery_params;

uint64_t pendingEvent;
static long tas_batt_mon_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	printk("Inside %s, Line = %d\n",__func__,__LINE__);
	printk("arg value  %#lX\n",arg);

	switch (cmd)
	{
		case WRITE_EVNT_FD:
			obj_tas_battery_params->gpcam_event_fd = (int) arg;
			printk("obj_tas_battery_params->gpcam_event_fd = %d\n",obj_tas_battery_params->gpcam_event_fd);
			break;

		case WRITE_GPCAM_PID:
			/* Check whether memory is accessible or not */
			obj_tas_battery_params->gpcam_pid = (int) arg;
			printk("obj_tas_battery_params->gpcam_pid = %d\n",obj_tas_battery_params->gpcam_pid);
			break;	

		default:
			printk(KERN_ERR "qpnp_smbcharger_if:Invalid ioctl command \n");
			return -EINVAL;
	}

	if((obj_tas_battery_params->gpcam_event_fd != 0) && 
			(obj_tas_battery_params->gpcam_pid != 0))
	{
		if (configure_eventfd() == SUCCESS)
		{
			obj_tas_battery_params->pid_recvd = 1;
			obj_tas_battery_params->running = 1;
			printk(KERN_INFO "Received PID and event fd and configured eventfd\n");
		}
		else
		{
			obj_tas_battery_params->pid_recvd = 0;
			obj_tas_battery_params->gpcam_event_fd = 0;
			obj_tas_battery_params->gpcam_pid = 0;
			printk(KERN_ERR "qpnp_smbcharger_if: Event fd configuration failed\n");
			return -EINVAL;
		}
	}

	return SUCCESS;
}

static int configure_eventfd(void)
{
	int ret = SUCCESS;
	/* Getting the userspace task details */
	obj_tas_battery_params->userspace_task = pid_task(\
			find_vpid(obj_tas_battery_params->gpcam_pid), \
			PIDTYPE_PID);

	/* Check whether the specified fd has an open file. */
	rcu_read_lock();
	obj_tas_battery_params->efd_file = fcheck_files(obj_tas_battery_params->userspace_task->files,
			obj_tas_battery_params->gpcam_event_fd);
	rcu_read_unlock();

	/* Acquires a reference to the internal eventfd context. */
	obj_tas_battery_params->efd_ctx = eventfd_ctx_fileget(obj_tas_battery_params->efd_file);
	if (!obj_tas_battery_params->efd_ctx) {
		printk(KERN_ERR "Event configuration failed\n");
		return -EINVAL;
	}
	return ret;
}

static ssize_t tas_batt_mon_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	printk("func = %s support is not yet added\n",__func__);
	return 0;
}

static ssize_t tas_batt_mon_read(struct file *file,char __user *buff, size_t count, loff_t *ppos)
{
	struct battery_mon_data battery_data;
	if (sizeof(struct battery_mon_data) != count)
	{
		printk(KERN_ERR "batt_mon: Irrelevant size of data is requested\n");
		return -EFAULT;

	}

	/*clearing memory */
	memset(&battery_data,0,sizeof(struct battery_mon_data));

	/* Get the battery capacity from fg driver */
	battery_data.batt_chg_sts = gopro_get_batt_chg_status();
	battery_data.batt_chg_type = gopro_get_batt_chg_type();
	battery_data.batt_capacity = gopro_get_batt_capacity();
	battery_data.batt_temp = gopro_get_batt_temp();
	battery_data.batt_current = gopro_get_batt_current();
	battery_data.batt_voltage = gopro_get_batt_volt();

	if(copy_to_user(buff, &battery_data,sizeof(struct battery_mon_data))!= 0)
	{
		printk(KERN_ERR "batt_mon: Failed to copy battery details to userspace\n");
		return -EFAULT;
	}
	return (sizeof(struct battery_mon_data));
}

int battery_evnt_handler(void)
{
	printk(KERN_INFO "Enterred into battery event handler thread\n");
	while (!kthread_should_stop())
	{
		down(&obj_tas_battery_params->battery_mon_sem);
		/* releasing the kernel threads */
		if(!obj_tas_battery_params->running)
		{
			/* Clear the events */
			pendingEvent = 0;
			continue;
		}
		/* Sending the event to the userspace */
		eventfd_signal(obj_tas_battery_params->efd_ctx, pendingEvent); 
		printk("Evt_hdlr: data %#llX\n",pendingEvent);
		/* Clear the events */
		pendingEvent = 0;
	}
	return 0;
}

static int tas_batt_mon_open(struct inode *inode, struct file *file)
{
	printk("obj_tas_battery_params->opened = %d\n",obj_tas_battery_params->opened);
	if(obj_tas_battery_params->opened)
	{
		printk( "Device already in use \n");
		return SUCCESS;
	}
	obj_tas_battery_params->thread = kthread_create(battery_evnt_handler,NULL,"Batt evt Hdlr");
	if(obj_tas_battery_params->thread < 0)
	{
		printk(KERN_ERR "Thread creation failed for event handling");
		return -EINVAL;
	}
	wake_up_process(obj_tas_battery_params->thread);
	obj_tas_battery_params->opened = 1;
	obj_tas_battery_params->pid_recvd = 0;
	obj_tas_battery_params->gpcam_event_fd = 0;
	obj_tas_battery_params->gpcam_pid = 0;

	printk( "Device Opened Successfully \n");
	return SUCCESS;
}

static int tas_batt_mon_close(struct inode *inode, struct file *file)
{
	int ret = 0;
	if (!obj_tas_battery_params->opened){
		printk( "Device is not opened already \n");
		return -EINVAL;
	}

	// unblock the semaphore wait from the loop possible down the semaphore as well
	obj_tas_battery_params->opened = 0;
	obj_tas_battery_params->gpcam_event_fd = 0;
	obj_tas_battery_params->gpcam_pid = 0;
	if( obj_tas_battery_params->pid_recvd == 1)
	{
		/* stopping the kernel */
		obj_tas_battery_params->running = 0;
		/* Releasing the semaphore to unblock thread sequence */
		up(&obj_tas_battery_params->battery_mon_sem);
		ret = kthread_stop(obj_tas_battery_params->thread);
		if(ret != 0)
		{
			printk(KERN_ERR "Thread cannot be stopped");
			return -EINVAL;
		}

		obj_tas_battery_params->pid_recvd = 0;
		eventfd_ctx_put(obj_tas_battery_params->efd_ctx);
	}

	printk( "Device is closed successfully\n");
	return 0;
}

/*
 *	Function to send the event to userspace 
 */
void send_smbchg_event(uint64_t event)
{
	if(!obj_tas_battery_params->pid_recvd)
	{
		printk("Evt_hdlr: No efd&pid and ignoring, evt_val:%d\n",event);
		return;
	}
	pendingEvent |= event;

	up(&obj_tas_battery_params->battery_mon_sem);
	return;
}
EXPORT_SYMBOL_GPL(send_smbchg_event);

static struct file_operations tas_batt_mon_fops = {
	.owner          = THIS_MODULE,
	.open	        = tas_batt_mon_open,
	.release        = tas_batt_mon_close,
	.read		= tas_batt_mon_read,
	.write		= tas_batt_mon_write,
	.unlocked_ioctl	= tas_batt_mon_ioctl,
};

static int __init tas_battery_monitor_init(void)
{
	int error = 0;

	/* Allocating the memory for the device context */
	obj_tas_battery_params = kmalloc(sizeof(struct tas_battery_params), GFP_KERNEL);
	if(obj_tas_battery_params == NULL) {
		printk("TAS BAT_PARAM: unable to allocate the memory for driver \n");
		return -ENOMEM; 
	}
	memset(obj_tas_battery_params,0,sizeof(struct tas_battery_params)); 

	/*registering the character device */
	obj_tas_battery_params->major = register_chrdev(0, "batt_monitor", &tas_batt_mon_fops);
	if ( obj_tas_battery_params->major < 0 ) {
		printk("register_chrdev(batt_monitor): FAIL\n");
		return -ENODEV;
	} else {
		printk("register_chrdev(batt_monitor): OK(major = %d)\n", obj_tas_battery_params->major);
	}

	/* Creating the class for device */
	obj_tas_battery_params->tas_bat_mon_class = class_create(THIS_MODULE, "batt_monitor");
	if(IS_ERR(obj_tas_battery_params->tas_bat_mon_class)){
		printk("tas_bat_mon_class fail: FAIL\n");
		return PTR_ERR(obj_tas_battery_params->tas_bat_mon_class);
	} else {
		printk(" TAS_BAT_PARAM: class create: OK\n");
	}


	obj_tas_battery_params->tas_bat_mon_dev = device_create(obj_tas_battery_params->tas_bat_mon_class, NULL, MKDEV(obj_tas_battery_params->major,0), NULL, "batt_monitor");
	if(unlikely(IS_ERR(obj_tas_battery_params->tas_bat_mon_dev))){
		printk("batt_monitor device create: FAIL\n");
		return -EINVAL;
	} else {
		printk("batt_monitor device created successfully\n");
	}

	/*Initializing semaphore and making it to wait */ 
	sema_init(&obj_tas_battery_params->battery_mon_sem, 0);

	pr_debug("Module initialized successfully \n");
	return error;
}

static void __exit tas_battery_monitor_exit(void)
{
	device_destroy(obj_tas_battery_params->tas_bat_mon_class, MKDEV(obj_tas_battery_params->major, 0));
	class_destroy(obj_tas_battery_params->tas_bat_mon_class);
	unregister_chrdev(obj_tas_battery_params->major, "batt_monitor");
	kfree(obj_tas_battery_params);
	printk("Module uninitialized successfully \n");
}

module_init(tas_battery_monitor_init);
module_exit(tas_battery_monitor_exit);

MODULE_DESCRIPTION("Tasmania Battery monitoring driver");
MODULE_AUTHOR("Arulpandiyan Vadivel <arulpandiyanvadivel@gopro.com>");
MODULE_LICENSE("GPL");
