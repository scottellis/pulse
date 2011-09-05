/*
 Copyright (c) 2010, Scott Ellis
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of the <organization> nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY Scott Ellis ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL Scott Ellis BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <linux/init.h> 
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/moduleparam.h>

#include "pulse.h"

#define PWM8 	0
#define PWM9 	1
#define PWM10 	2
#define PWM11	3
#define NUM_PWM_TIMERS 4


/* default frequency of 1 kHz */
#define DEFAULT_TLDR	0xFFFFFFE0

/* default 50% duty cycle */
/* TMAR = (0xFFFFFFFF - ((0xFFFFFFFF - (DEFAULT_TLDR + 1)) / 2)) */
#define DEFAULT_TMAR	0xFFFFFFEF

/* default TCLR is off state */
#define DEFAULT_TCLR (GPT_TCLR_PT | GPT_TCLR_TRG_OVFL_MATCH | GPT_TCLR_CE \
			| GPT_TCLR_AR) 

#define DEFAULT_PWM_FREQUENCY 2

#define CLOCK_PERIOD_NS 77


static int frequency = DEFAULT_PWM_FREQUENCY;
module_param(frequency, int, S_IRUGO);
MODULE_PARM_DESC(frequency, "The PWM frequency");



#define USER_BUFF_SIZE	128

struct gpt {
	u32 timer_num;
	u32 mux_offset;
	u32 gpt_base;
	u32 input_freq;
	u32 old_mux;
	u32 tldr;
	u32 tmar;
	u32 tclr;
	u32 num_freqs;
	u32 offset;
	void __iomem *base;
};

struct pulse_dev {
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct semaphore sem;
	struct gpt gpt[NUM_PWM_TIMERS];
	char *user_buff;
};

static struct pulse_dev pulse_dev;


static int init_mux(void)
{
	int i;
	void __iomem *base;

	for (i = 0; i < NUM_PWM_TIMERS; i++) {
		base = ioremap(OMAP34XX_PADCONF_START, OMAP34XX_PADCONF_SIZE);
		if (!base) {
			printk(KERN_ALERT "init_mux(): ioremap() failed\n");
			return -1;
		}

		pulse_dev.gpt[i].old_mux = ioread16(base + pulse_dev.gpt[i].mux_offset);
		iowrite16(PWM_ENABLE_MUX, base + pulse_dev.gpt[i].mux_offset);
		iounmap(base);	
	}

	return 0;
}

static int restore_mux(void)
{
	int i;
	void __iomem *base;

	for (i = 0; i < NUM_PWM_TIMERS; i++) {
		if (pulse_dev.gpt[i].old_mux) {
			base = ioremap(OMAP34XX_PADCONF_START, OMAP34XX_PADCONF_SIZE);
			if (!base) {
				printk(KERN_ALERT "restore_mux(): ioremap() failed\n");
				return -1;
			}

			iowrite16(pulse_dev.gpt[i].old_mux, base + pulse_dev.gpt[i].mux_offset);
			iounmap(base);	
		}
	}

	return 0;
}

/* Change PWM10 and PWM11 to use CM_SYS_CLK rather then CM_32K_CLK */
static int use_sys_clk(void)
{
	void __iomem *base;
	u32 val;

	base = ioremap(CLOCK_CONTROL_REG_CM_START, CLOCK_CONTROL_REG_CM_SIZE);

	if (!base) {
		printk(KERN_ALERT "use_sys_clk(): ioremap() failed\n");
		return -1;
	}

	val = ioread32(base + CM_CLKSEL_CORE_OFFSET);
	val |= 0xc0;
	iowrite32(val, base + CM_CLKSEL_CORE_OFFSET);
	iounmap(base);

	return 0;
}

/* Restore PWM10 and PWM11 to using the CM_32K_CLK */
static int restore_32k_clk(void)
{
	void __iomem *base;
	u32 val;

	base = ioremap(CLOCK_CONTROL_REG_CM_START, CLOCK_CONTROL_REG_CM_SIZE);

	if (!base) {
		printk(KERN_ALERT "restore_32k_clk(): ioremap() failed\n");
		return -1;
	}

	val = ioread32(base + CM_CLKSEL_CORE_OFFSET);
	val &= ~0xc0;
	iowrite32(val, base + CM_CLKSEL_CORE_OFFSET);
	iounmap(base);

	return 0;
}

static int map_timers(void)
{
	int i;

	for (i = 0; i < NUM_PWM_TIMERS; i++) {
		pulse_dev.gpt[i].base = ioremap(pulse_dev.gpt[i].gpt_base, 
						GPT_REGS_PAGE_SIZE);

		if (!pulse_dev.gpt[i].base)
			return -1;
	}

	return 0;
}

static void unmap_timers(void)
{
	int i;

	for (i = 0; i < NUM_PWM_TIMERS; i++) {
		if (pulse_dev.gpt[i].base) {
			iounmap(pulse_dev.gpt[i].base);
			pulse_dev.gpt[i].base = NULL;
		}
	}		
}

static int set_pwm_frequency(void)
{
	int i;

	if (frequency < 0) {
		frequency = DEFAULT_PWM_FREQUENCY;
	} else {
		if (frequency > (pulse_dev.gpt[PWM8].input_freq / 2)) 
			frequency = pulse_dev.gpt[PWM8].input_freq / 2;
		else if (frequency == 0)
			frequency = DEFAULT_PWM_FREQUENCY;
	}

	for (i = 0; i < NUM_PWM_TIMERS; i++) {
		/* PWM_FREQ = 32768 / ((0xFFFF FFFF - TLDR) + 1) */
		pulse_dev.gpt[i].tldr = 0xFFFFFFFF - 
			((pulse_dev.gpt[i].input_freq / frequency) - 1);

		/* just for convenience */	
		pulse_dev.gpt[i].num_freqs = 0xFFFFFFFE 
						- pulse_dev.gpt[i].tldr;	

		iowrite32(pulse_dev.gpt[i].tldr, 
				pulse_dev.gpt[i].base + GPT_TLDR);
	}

	return 0;
}

static int pulse_off(void)
{
	int i;

	for (i = 0; i < NUM_PWM_TIMERS; i++) {
		pulse_dev.gpt[i].tclr &= ~GPT_TCLR_ST;
		iowrite32(pulse_dev.gpt[i].tclr, 
				pulse_dev.gpt[i].base + GPT_TCLR); 
	}

	return 0;
}

static int pulse_on(void)
{
	int i;

	for (i = 0; i < NUM_PWM_TIMERS; i++) {
		if (pulse_dev.gpt[i].tmar == 0)
			continue;

		/* set the duty cycle */
		iowrite32(pulse_dev.gpt[i].tmar, 
				pulse_dev.gpt[i].base + GPT_TMAR);
	
		/* initialize TCRR to TLDR, have to start somewhere */
		iowrite32(pulse_dev.gpt[i].tldr + pulse_dev.gpt[i].offset, 
				pulse_dev.gpt[i].base + GPT_TCRR);
		
		/* now turn it on */
		pulse_dev.gpt[i].tclr = ioread32(pulse_dev.gpt[i].base 
								+ GPT_TCLR);
		pulse_dev.gpt[i].tclr |= GPT_TCLR_ST;
	}

	/* try to turn them all on at once */
	for (i = 0; i < NUM_PWM_TIMERS; i++) { 
		iowrite32(pulse_dev.gpt[i].tclr, 
					pulse_dev.gpt[i].base + GPT_TCLR); 
	}

	return 0;
}

static int set_pulse_length(unsigned int timer, unsigned int pulse_length_us) 
{
	unsigned int new_tmar;

	if (timer >= NUM_PWM_TIMERS)
		return -1;

	if (pulse_length_us == 0) {
		pulse_dev.gpt[timer].tmar = 0;
 	}
	else {
		new_tmar = (pulse_length_us * 1000) / CLOCK_PERIOD_NS;

		if (new_tmar == 0) 
			new_tmar = 1;
		else if (new_tmar > pulse_dev.gpt[timer].num_freqs)
			new_tmar = pulse_dev.gpt[timer].num_freqs;
		
		pulse_dev.gpt[timer].tmar = pulse_dev.gpt[timer].tldr 
						+ new_tmar;
	}

	return 0;
}

static ssize_t pulse_read(struct file *filp, char __user *buff, size_t count,
			loff_t *offp)
{
	size_t len;
	unsigned int duty_cycle;
	ssize_t error = 0;

	if (!buff) 
		return -EFAULT;

	/* tell the user there is no more */
	if (*offp > 0) 
		return 0;

	if (down_interruptible(&pulse_dev.sem)) 
		return -ERESTARTSYS;

	if (pulse_dev.gpt[0].tclr & GPT_TCLR_ST) {
		duty_cycle = (10000 * 
			(pulse_dev.gpt[0].tmar - pulse_dev.gpt[0].tldr))
				/ pulse_dev.gpt[0].num_freqs;

		snprintf(pulse_dev.user_buff, USER_BUFF_SIZE,
				"PWM%d Frequency %u Hz Duty Cycle %u\n",
				pulse_dev.gpt[0].timer_num, 
				frequency, 
				duty_cycle);
	}
	else {
		snprintf(pulse_dev.user_buff, USER_BUFF_SIZE,
				"PWM%d Frequency %u Hz Stopped\n",
				pulse_dev.gpt[0].timer_num, frequency);
	}

	len = strlen(pulse_dev.user_buff);
 
	if (len + 1 < count) 
		count = len + 1;

	if (copy_to_user(buff, pulse_dev.user_buff, count))  {
		printk(KERN_ALERT "pwm_read(): copy_to_user() failed\n");
		error = -EFAULT;
	}
	else {
		*offp += count;
		error = count;
	}

	up(&pulse_dev.sem);

	return error;	
}

/*
  Input: pwm8_us[:pwm9_us][:pwm10_us][:pwm11_us]
*/
static void process_pulse_lengths(void)
{
	int i;
	char *pos, *s;
	unsigned int pulse_length_us;

	pos = pulse_dev.user_buff;

	for (i = 0; i < NUM_PWM_TIMERS; i++) {	
		s = strsep(&pos, ":"); 

		if (s) {
			pulse_length_us = simple_strtoul(s, NULL, 0);
			set_pulse_length(i, pulse_length_us);			
		}

		if (!pos)
			break;
	}	

	for (i++; i < NUM_PWM_TIMERS; i++) 
		set_pulse_length(i, 0);
}

/*
  Input: dpwm8_delay:pwm9_delay[:pwm10_delay][:pwm11_delay]
*/
static void process_pulse_delays(void)
{
	int i;
	char *pos, *s;
	unsigned int delay[NUM_PWM_TIMERS], max_delay;

	memset(delay, 0, sizeof(delay));
	max_delay = 0;

	/* skip overo the 'd' */
	pos = &pulse_dev.user_buff[1];

	for (i = 0; i < NUM_PWM_TIMERS; i++) {	
		s = strsep(&pos, ":"); 

		if (s) {
			delay[i] = simple_strtoul(s, NULL, 0);
			
			if (delay[i] > max_delay)
				max_delay = delay[i];
		}

		if (!pos)
			break;
	}	

	/* 
	  Because of the way we are setting up the delays using
	  TCRR = TLDR + an offset, we have to 'reverse' these delays
	  so that they happen sequentially.
	*/
	for (i = 0; i < NUM_PWM_TIMERS; i++)
		pulse_dev.gpt[i].offset = (1000 * (max_delay - delay[i]))
					/ CLOCK_PERIOD_NS;
}

static ssize_t pulse_write(struct file *filp, const char __user *buff, 
			size_t count, loff_t *offp)
{
	size_t len;
	ssize_t status = 0;
	
	if (down_interruptible(&pulse_dev.sem)) 
		return -ERESTARTSYS;

	if (!buff || count < 1) {
		printk(KERN_ALERT "pwm_write(): input check failed\n");
		status = -EFAULT; 
		goto pwm_write_done;
	}
	
	len = USER_BUFF_SIZE - 1;

	/* we are only expecting a small integer, ignore anything else */
	if (count < len)
		len = count;
		
	memset(pulse_dev.user_buff, 0, USER_BUFF_SIZE);

	if (copy_from_user(pulse_dev.user_buff, buff, len)) {
		printk(KERN_ALERT "pwm_write(): copy_from_user() failed\n"); 
		status = -EFAULT; 	
		goto pwm_write_done;
	}

	if (!strncasecmp(pulse_dev.user_buff, "on", 2)) {
		pulse_on();
	}
	else if (!strncasecmp(pulse_dev.user_buff, "off", 3)) {
		pulse_off();
	}
	else {
		pulse_off();

		if (isdigit(pulse_dev.user_buff[0])) 
			process_pulse_lengths();
		else if (pulse_dev.user_buff[0] == 'd')
			process_pulse_delays();
		else 
			printk(KERN_ALERT "pulse_write() unknown command %s\n",
				pulse_dev.user_buff);
	}
	
	/* pretend we ate it all */
	*offp += count;

	status = count;

pwm_write_done:

	up(&pulse_dev.sem);
	
	return status;
}

static long pulse_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) 
{
	printk(KERN_ALERT "pulse_ioctl(%d)\n", cmd);

	return -ENOTTY;
}

static int pulse_open(struct inode *inode, struct file *filp)
{
	int error = 0;

	if (down_interruptible(&pulse_dev.sem)) 
		return -ERESTARTSYS;

	if (!pulse_dev.user_buff) {
		pulse_dev.user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);
		if (!pulse_dev.user_buff)
			error = -ENOMEM;
	}

	up(&pulse_dev.sem);

	return error;	
}

static struct file_operations pulse_fops = {
	.owner = THIS_MODULE,
	.read = pulse_read,
	.write = pulse_write,
	.unlocked_ioctl = pulse_ioctl,
	.open = pulse_open,
};

static int __init pulse_init_cdev(void)
{
	int error;

	error = alloc_chrdev_region(&pulse_dev.devt, 0,	1, "pulse");

	if (error < 0) {
		printk(KERN_ALERT "alloc_chrdev_region() failed: %d \n", 
			error);
		return -1;
	}

	cdev_init(&pulse_dev.cdev, &pulse_fops);
	pulse_dev.cdev.owner = THIS_MODULE;
	
	error = cdev_add(&pulse_dev.cdev, pulse_dev.devt, 1);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: %d\n", error);
		unregister_chrdev_region(pulse_dev.devt, 1);
		return -1;
	}	

	return 0;
}

static int __init pulse_init_class(void)
{
	pulse_dev.class = class_create(THIS_MODULE, "pulse");

	if (!pulse_dev.class) {
		printk(KERN_ALERT "class_create() failed\n");
		return -1;
	}

	if (!device_create(pulse_dev.class, NULL, pulse_dev.devt, NULL, "pulse")) {
		printk(KERN_ALERT "device_create(..., pulse) failed\n");
		class_destroy(pulse_dev.class);
		return -1;
	}

	return 0;
}
 
static int pulse_init_timers(void)
{
	int i;

	if (use_sys_clk()) 
		return -1;
	
	if (init_mux()) {
		restore_32k_clk();
		return -1;
	}

	if (map_timers()) {
		unmap_timers();
		restore_mux();
		restore_32k_clk();
		return -1;
	}

	if (set_pwm_frequency()) {
		unmap_timers();
		restore_mux();
		restore_32k_clk();
		return -1;
	}
	
	for (i = 0; i < NUM_PWM_TIMERS; i++) 
		set_pulse_length(i, 100);

	return 0;
}


static int __init pulse_init(void)
{
	int error = 0;

	memset(&pulse_dev, 0, sizeof(struct pulse_dev));

	pulse_dev.gpt[PWM8].timer_num = 8;
	pulse_dev.gpt[PWM8].mux_offset = GPT8_MUX_OFFSET;
	pulse_dev.gpt[PWM8].gpt_base = PWM8_CTL_BASE;
	pulse_dev.gpt[PWM8].input_freq = CLK_SYS_FREQ;
	pulse_dev.gpt[PWM8].tclr = DEFAULT_TCLR;
	pulse_dev.gpt[PWM8].offset = 1950;

	pulse_dev.gpt[PWM9].timer_num = 9;
	pulse_dev.gpt[PWM9].mux_offset = GPT9_MUX_OFFSET;
	pulse_dev.gpt[PWM9].gpt_base = PWM9_CTL_BASE;
	pulse_dev.gpt[PWM9].input_freq = CLK_SYS_FREQ;
	pulse_dev.gpt[PWM9].tclr = DEFAULT_TCLR;
	pulse_dev.gpt[PWM9].offset = 1300;

	pulse_dev.gpt[PWM10].timer_num = 10;
	pulse_dev.gpt[PWM10].mux_offset = GPT10_MUX_OFFSET;
	pulse_dev.gpt[PWM10].gpt_base = PWM10_CTL_BASE;
	pulse_dev.gpt[PWM10].input_freq = CLK_SYS_FREQ;
	pulse_dev.gpt[PWM10].tclr = DEFAULT_TCLR;
	pulse_dev.gpt[PWM10].offset = 650;

	pulse_dev.gpt[PWM11].timer_num = 11;
	pulse_dev.gpt[PWM11].mux_offset = GPT11_MUX_OFFSET;
	pulse_dev.gpt[PWM11].gpt_base = PWM11_CTL_BASE;
	pulse_dev.gpt[PWM11].input_freq = CLK_SYS_FREQ;
	pulse_dev.gpt[PWM11].tclr = DEFAULT_TCLR;
	pulse_dev.gpt[PWM11].offset = 0;

	sema_init(&pulse_dev.sem, 1);

	if (pulse_init_cdev())
		goto init_fail;

	if (pulse_init_class())
		goto init_fail_2;

	if (pulse_init_timers())
		goto init_fail_3;

	return 0;

init_fail_3:
	device_destroy(pulse_dev.class, pulse_dev.devt);
	class_destroy(pulse_dev.class);

init_fail_2:
	cdev_del(&pulse_dev.cdev);
	unregister_chrdev_region(pulse_dev.devt, 1);

init_fail:
	return error;
}
module_init(pulse_init);

static void __exit pulse_exit(void)
{
	device_destroy(pulse_dev.class, pulse_dev.devt);
	class_destroy(pulse_dev.class);

	cdev_del(&pulse_dev.cdev);
	unregister_chrdev_region(pulse_dev.devt, 1);

	pulse_off();
	unmap_timers();
	restore_mux();
	
	restore_32k_clk();

	if (pulse_dev.user_buff)
		kfree(pulse_dev.user_buff);
}
module_exit(pulse_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Scott Ellis - Jumpnow");
MODULE_DESCRIPTION("Using the Overo PWM outputs to generate some pulses"); 

