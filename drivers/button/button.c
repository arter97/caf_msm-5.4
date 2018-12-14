/**
 * @file   button.c
 * @author Shekhar Halakatti
 * @date   03 October 2016
 * @brief  A kernel module for controlling a button (or any signal) that is connected to
 * a GPIO. It has full support for interrupts and for sysfs entries so that an interface
 * can be created to the button or the button can be configured from Linux userspace.
 * The sysfs entry appears at /sys/gpkey/
 * @see gopro
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings
#include <linux/time.h>       // Using the clock to measure time between button presses
#include <linux/signal.h>

#define DRIVER_NAME "gp_button"
#define SHUTTER 87
#define SETTING 86
#define DEBOUNCE_TIME 15    // The default bounce time -- 200ms TODO: FIX this time

#define USE_SIGNAL

static ssize_t show_shutter(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_shutter(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count);
static ssize_t show_setting(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_setting(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count);
static ssize_t show_status(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
#ifdef USE_SIGNAL
static int send_button_signal( unsigned int val );
static irq_handler_t button_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);
#endif
static int irqShutter, irqSetting;
static struct gp_button_data{
    struct kobject *button_kobj;
    unsigned int shutter;
    unsigned int setting;
    unsigned int status;
}button_data;

static struct kobj_attribute shutter_attribute =
__ATTR(shutter, 0664, show_shutter, store_shutter);

static struct kobj_attribute setting_attribute =
__ATTR(setting, 0664, show_setting, store_setting);

static struct kobj_attribute status_attribute =
__ATTR(status, 0444, show_status, NULL);

static struct attribute *attrs[] = {
    &shutter_attribute.attr,
    &setting_attribute.attr,
    &status_attribute.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,                    // The attributes array defined just above
};

static ssize_t show_shutter(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", button_data.shutter );
}
static ssize_t show_setting(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", button_data.setting );
}

static ssize_t store_shutter(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count)
{
    unsigned int gpio;
    gpio = simple_strtoul(buf,NULL ,10);
    button_data.shutter = gpio;

    return strlen(buf);
}

static ssize_t store_setting(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count)
{
    unsigned int gpio;
    gpio = simple_strtoul(buf,NULL ,10);
    button_data.setting = gpio;

    return strlen(buf);
}

static ssize_t show_status(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", (gpio_get_value(button_data.shutter) | (gpio_get_value(button_data.setting)<< 1)));
}

static int  button_drv_probe(void)
{
    int retval = 0;

    button_data.shutter = SHUTTER;
    button_data.setting = SETTING;
    button_data.status = 0;
    printk(KERN_INFO "Button: Probe Shutter button %d\n", button_data.shutter);
    printk(KERN_INFO "Button: Probe Setting button %d\n", button_data.setting);

    /* create a kernel object */
    button_data.button_kobj = kobject_create_and_add(DRIVER_NAME, kernel_kobj);
    if (!button_data.button_kobj){
        printk(KERN_ALERT "Button: failed to create kobject mapping\n");
        return -ENOMEM;
    }

    /* Create the files associated with this kobject */
    retval = sysfs_create_group(button_data.button_kobj, &attr_group);
    if (retval){
        printk(KERN_ALERT " Button: failed to create sysfs group[retval=%d]\n",retval);
        kobject_put(button_data.button_kobj);
        return retval;
    }
    
    /* Setup the GPIO */
    gpio_request(button_data.shutter, "gpio 87 connected to shutter");
    gpio_direction_input(button_data.shutter);
    gpio_set_debounce(button_data.shutter, DEBOUNCE_TIME);
    gpio_export(button_data.shutter, false);

    gpio_request(button_data.setting, "gpio 86 connected to shutter");
    gpio_direction_input(button_data.setting);
    gpio_set_debounce(button_data.setting, DEBOUNCE_TIME);
    gpio_export(button_data.setting, false);
#ifdef USE_SIGNAL
    /* Setup the interrupt handler */
    irqShutter = gpio_to_irq(button_data.shutter); // GPIO numbers and IRQ numbers are not the same!
    retval = request_irq(irqShutter,
                        (irq_handler_t) button_irq_handler,
                        (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
                        "_button_handler",
                        NULL);
    irqSetting = gpio_to_irq(button_data.setting); // GPIO numbers and IRQ numbers are not the same!
    retval = request_irq(irqSetting,
                       (irq_handler_t) button_irq_handler,
                       (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
                       "_button_handler",
                       NULL);
#endif
    return retval;
}

static int button_drv_remove(void) {
#ifdef USE_SIGNAL
    free_irq(irqShutter, NULL);
    free_irq(irqSetting, NULL);
#endif
    gpio_unexport(button_data.shutter);
    gpio_free(button_data.shutter);
    gpio_unexport(button_data.setting);
    gpio_free(button_data.setting);
    sysfs_remove_group(button_data.button_kobj, &attr_group);
    kobject_put(button_data.button_kobj);
    printk(KERN_INFO " Button: Module removed!\n");
    return 0;
}

static int __init Button_init(void){
    button_drv_probe();
    return 0;
}

/** The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit Button_exit(void){
    button_drv_remove();
}

#ifdef USE_SIGNAL
/** The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t button_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
    int state = 0;
    state = gpio_get_value(button_data.shutter)| (gpio_get_value(button_data.setting)<< 1);
    printk(KERN_INFO " Button: The button state is currently: %d\n", state);
    send_button_signal(state);
    return (irq_handler_t) IRQ_HANDLED;  // Announce that the IRQ has been handled correctly
}

static int send_button_signal(unsigned int val)
{
    struct siginfo info;
    struct task_struct *t;
    int ret =0;
    int id = 0;
    /* send the signal */
    memset(&info, 0, sizeof(struct siginfo));
    info.si_signo = SIGIO;
    info.si_code = SI_SIGIO;
    info.si_int = val;
    info.si_errno = 0;
    printk("pid %d \n",current->pid);
    rcu_read_lock();
    //t = find_task_by_pid_type(PIDTYPE_PID, pid);  //find the task_struct associated with this pid
    t = pid_task(find_pid_ns(id, &init_pid_ns), PIDTYPE_PID);
    if(t == NULL)
    {
        printk("no such pid\n");
        rcu_read_unlock();
        return -ENODEV;
    }
    ret = send_sig_info(SIGIO, &info, t);    //send the signal
    rcu_read_unlock();
    if (ret < 0)
    {
        printk("error sending signal\n");
        return ret;
    }
    //else
        //printk("Send sig %d val %d pid %d\n", sig, val, id);

    return ret;
}
#endif

MODULE_AUTHOR("Shekhar Halakatti");
MODULE_DESCRIPTION("GP Button driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

module_init(Button_init);
module_exit(Button_exit);
