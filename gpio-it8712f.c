/*
 *  GPIO interface for IT8712F Super I/O chip
 *
 *  Author: Oleg Malashenko <oma.jc404@gmail.com>
 *
 *  Based on IT8712F EC-LPC I/O Preliminary Specification 0.81
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/ioport.h>

#include <linux/gpio.h>

#define DEFAULT_STARTPIN -1
#define GPIO_NAME "it8712-gpio"

static int startpin = DEFAULT_STARTPIN;

module_param(startpin, int, 0);
MODULE_PARM_DESC(startpin, "First pin number under /sys/class/gpio/, "
		"default = " __MODULE_STRING(DEFAULT_STARTPIN));

typedef enum {
	REG_LOGICAL_DEVICE              = 0x7,
	REG_CHIP_ID_HIGH_BYTE           = 0x20,
	REG_CHIP_ID_LOW_BYTE            = 0x21,
	REG_SIMPLEIO_BASE_HIGH_BYTE     = 0x62,
	REG_SIMPLEIO_BASE_LOW_BYTE      = 0x63,
	REG_GPIO_ENABLE_BASE            = 0x25,
	REG_SIMPLEIO_ENABLE_BASE        = 0xC0,
	REG_SIMPLEIO_ENABLE_OUTPUT_BASE = 0xC8,
} superio_registers_t;

typedef enum {
	LDN_GPIO = 0x7
} superio_devices_t;

static size_t GPIO_IOSIZE = 5;
static int    IT8712F_CHIP_ID = 0x8712;
static size_t GPIO_LINES = 4 * 8 + 6; // 4 sets * 8 lines + 1 set * 6 lines

static u16 gpio_base;
static u8 port;

static u8 superio_read_reg(superio_registers_t reg, u8 port)
{
	outb(reg, port);
	return inb(port + 1);
}

static void superio_write_reg(u8 data, superio_registers_t reg, u8 port)
{
	outb(reg, port);
	outb(data, port + 1);
}

static int superio_enter(u8 port)
{
	if (!request_muxed_region(port, 2, GPIO_NAME))
		return -EBUSY;

	outb(0x87, port);
	outb(0x01, port);
	outb(0x55, port);
	outb((port == 0x2e) ? 0x55 : 0xaa, port);

	return 0;
}

static void superio_exit(u8 port)
{
	outb(0x2, port);
	outb(0x2, port + 1);

	release_region(port, 2);
}

static void superio_enter_gpio_mode(u8 port)
{
	superio_write_reg(LDN_GPIO, REG_LOGICAL_DEVICE, port);
}

static u8 superio_port_discover(void)
{
	static const u8 ports[2] = { 0x2e, 0x4e };

	size_t i;
	int id;
	u8 p = -ENODEV;

	/* chip and port detection */
	for (i = 0; i < ARRAY_SIZE(ports); i++) {
		if (superio_enter(ports[i]) != 0)
		{
			pr_warn("SuperIO port %x is busy\n", ports[i]);
			continue;
		}

		id = (superio_read_reg(REG_CHIP_ID_HIGH_BYTE, ports[i]) << 8) +
			superio_read_reg(REG_CHIP_ID_LOW_BYTE, ports[i]);

		superio_exit(ports[i]);

		if (id == IT8712F_CHIP_ID) {
			p = ports[i];
			break;
		}
	}

	return p;
}

inline static void get_bit_position(unsigned gpio_num, u16* byte, u8* bit)
{
	*byte = gpio_num / 8;
	*bit  = gpio_num % 8;
}

static int it8712f_gpio_enable(unsigned gpio_num, bool on)
{
	u8 bit, curr;
	u16 reg_offset, io_reg;

	if (superio_enter(port) != 0)
		return -EBUSY;

	superio_enter_gpio_mode(port);
	get_bit_position(gpio_num, &reg_offset, &bit);

	/* Enable GPIO pin */
	io_reg = REG_GPIO_ENABLE_BASE + reg_offset;
	curr = superio_read_reg(io_reg, port);
	curr = (on) ? (curr |  (1 << bit))
		    : (curr & ~(1 << bit));

	superio_write_reg(curr, io_reg, port);

	/* Enable SimpleIO mode */
	io_reg = REG_SIMPLEIO_ENABLE_BASE + reg_offset;
	curr = superio_read_reg(io_reg, port);
	curr = (on) ? (curr |  (1 << bit))
		    : (curr & ~(1 << bit));

	superio_write_reg(curr | (1 << bit), io_reg, port);

	superio_exit(port);
	return 0;
}

static int it8712f_gpio_request(struct gpio_chip *gc, unsigned gpio_num)
{
	return it8712f_gpio_enable(gpio_num, true);
}

static void it8712f_gpio_free(struct gpio_chip *gc, unsigned gpio_num)
{
	if (it8712f_gpio_enable(gpio_num, false) != 0)
	{
		pr_warn("%s(): unable to free GPIO line %d",
				__func__, gpio_num);
	}
}

static int it8712f_gpio_get(struct gpio_chip *gc, unsigned gpio_num)
{
	u16 offset;
	u8 bit;

	get_bit_position(gpio_num, &offset, &bit);
	return !!(inb(gpio_base + offset) & (1 << bit));
}

static void it8712f_gpio_set(struct gpio_chip *gc,
				unsigned gpio_num, int val)
{
	u8 curr_vals, bit;
	u16 io_reg;

	/*
	 * Spinlock is required because this function is not protected by
	 * superio_enter()
	 */
	static DEFINE_SPINLOCK(gpio_lock);

	get_bit_position(gpio_num, &io_reg, &bit);
	io_reg += gpio_base;

	spin_lock(&gpio_lock);

	curr_vals = inb(io_reg);
	if (val)
		outb(curr_vals | (1 << bit) , io_reg);
	else
		outb(curr_vals & ~(1 << bit), io_reg);

	spin_unlock(&gpio_lock);
}

static int it8712f_gpio_direction_switch(unsigned gpio_num, bool out)
{
	u8 curr_dirs, need_dirs, bit;
	u16 io_reg;

	get_bit_position(gpio_num, &io_reg, &bit);
	io_reg += REG_SIMPLEIO_ENABLE_OUTPUT_BASE;

	if (superio_enter(port) != 0)
		return -EBUSY;

	superio_enter_gpio_mode(port);

	curr_dirs = superio_read_reg(io_reg, port);
	need_dirs = (out) ? (curr_dirs |  (1 << bit))
			  : (curr_dirs & ~(1 << bit));

	if (curr_dirs != need_dirs)
		superio_write_reg(need_dirs, io_reg, port);

	superio_exit(port);
	return 0;
}

static int it8712f_gpio_direction_in(struct gpio_chip *gc, unsigned gpio_num)
{
	return it8712f_gpio_direction_switch(gpio_num, false);
}

static int it8712f_gpio_direction_out(struct gpio_chip *gc,
					unsigned gpio_num, int val)
{
	it8712f_gpio_set(gc, gpio_num, val);
	return it8712f_gpio_direction_switch(gpio_num, true);
}

static struct gpio_chip it8712f_gpio_chip = {
	.label			= GPIO_NAME,
	.owner			= THIS_MODULE,
	.request		= it8712f_gpio_request,
	.free			= it8712f_gpio_free,
	.get			= it8712f_gpio_get,
	.direction_input	= it8712f_gpio_direction_in,
	.set			= it8712f_gpio_set,
	.direction_output	= it8712f_gpio_direction_out,
};

static int __init it8712f_gpio_init(void)
{
	int err;

	port = superio_port_discover();
	if (port < 0)
		return port;

	/* fetch GPIO base address */
	if (superio_enter(port) != 0)
	{
		pr_warn("SuperIO port %x is busy\n", port);
		return -EBUSY;
	}

	superio_enter_gpio_mode(port);
	gpio_base = (superio_read_reg(REG_SIMPLEIO_BASE_HIGH_BYTE, port) << 8) +
				superio_read_reg(REG_SIMPLEIO_BASE_LOW_BYTE, port);
	superio_exit(port);

	it8712f_gpio_chip.base  = startpin;
	it8712f_gpio_chip.ngpio = GPIO_LINES;

	if (!request_region(gpio_base, GPIO_IOSIZE, GPIO_NAME))
	{
		pr_warn("unable to lock IO region for exclusive use\n");
		gpio_base = 0;
		return -EBUSY;
	}

	err = gpiochip_add(&it8712f_gpio_chip);
	if (err < 0)
	{
		pr_warn("%s(): gpiochip_add() failed, ret=%d\n",
				__func__, err);
		goto gpiochip_add_err;
	}

	pr_info("Chip IT8712F initialised, startpin=%d ngpio=%d\n",
		startpin, it8712f_gpio_chip.ngpio);

	return 0;

gpiochip_add_err:
	release_region(gpio_base, GPIO_IOSIZE);
	gpio_base = 0;
	return err;
}

static void __exit it8712f_gpio_exit(void)
{
	if (gpio_base) {
		int ret = gpiochip_remove(&it8712f_gpio_chip);

		WARN(ret, "%s(): gpiochip_remove() failed, ret=%d\n",
				__func__, ret);

		release_region(gpio_base, GPIO_IOSIZE);
		gpio_base = 0;
	}

	pr_info("IT8712F GPIO shutdown\n");
}
module_init(it8712f_gpio_init);
module_exit(it8712f_gpio_exit);

MODULE_AUTHOR("Oleg Malashenko <oma.jc404@gmail.com>");
MODULE_DESCRIPTION("GPIO interface for IT8712F Super I/O chip");
MODULE_LICENSE("GPL");
