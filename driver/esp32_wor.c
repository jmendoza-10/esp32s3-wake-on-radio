// SPDX-License-Identifier: GPL-2.0
/*
 * esp32_wor - Linux kernel driver for ESP32-S3 Wake-on-Radio
 *
 * Architecture:
 *   The ESP32-S3 firmware drives a GPIO pin HIGH when a wake trigger
 *   is detected (TRIGGER_DETECTED state) and LOW when it returns to
 *   idle/sleep.  This driver requests that GPIO as an interrupt source
 *   and reacts to both edges:
 *
 *     Rising edge  → wake event started  (ESP32 just received a trigger)
 *     Falling edge → wake event ended    (ESP32 going back to idle)
 *
 *   This is a pure hardware-interrupt design — no UART parsing needed
 *   for the wake signal.  UART (serdev) is still available as a
 *   secondary channel for state/telemetry if desired in the future.
 *
 * What's exposed to userspace:
 *   /sys/class/esp32_wor/esp32_wor0/wake_count   — total wake events (r/w)
 *   /sys/class/esp32_wor/esp32_wor0/active        — 1 if GPIO is HIGH now
 *   /sys/class/esp32_wor/esp32_wor0/last_wake_ns  — ktime of last rising edge
 *   /sys/class/esp32_wor/esp32_wor0/last_duration_ns — pulse width of last wake
 *
 * Device Tree binding:
 *   compatible = "custom,esp32-wor";
 *   wake-gpios = <&gpio 17 GPIO_ACTIVE_HIGH>;
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/device.h>

#define DRIVER_NAME "esp32_wor"

/*
 * Debounce: ignore pulses shorter than this.  ESP32 GPIO can glitch
 * during reset/boot even with hold + pull-down.  A real wake trigger
 * produces a pulse of ~2 seconds (firmware active-work delay).  500 ms
 * safely filters all boot/sleep transients.
 */
#define DEBOUNCE_NS  (500 * NSEC_PER_MSEC)

/* ------------------------------------------------------------------ */
/*  Per-device state                                                   */
/* ------------------------------------------------------------------ */

struct esp32_wor {
	struct device    *dev;
	struct gpio_desc *wake_gpio;
	int               irq;

	spinlock_t  lock;
	u32         wake_count;
	bool        active;           /* current GPIO level               */
	ktime_t     last_rise;        /* timestamp of last rising edge    */
	s64         last_duration_ns; /* pulse width: fall - rise         */
	bool        pending;          /* rise seen, waiting for fall to confirm */
};

/* ------------------------------------------------------------------ */
/*  Interrupt handler — runs in hard-IRQ context                       */
/* ------------------------------------------------------------------ */

static irqreturn_t esp32_wor_irq(int irq, void *data)
{
	struct esp32_wor *priv = data;
	int val;
	unsigned long flags;
	s64 pulse_ns;

	val = gpiod_get_value(priv->wake_gpio);

	spin_lock_irqsave(&priv->lock, flags);

	if (val && !priv->pending) {
		/* Rising edge — record time, but don't count yet */
		priv->pending = true;
		priv->active = true;
		priv->last_rise = ktime_get();
	} else if (!val && priv->pending) {
		/* Falling edge — check pulse width for debounce */
		pulse_ns = ktime_to_ns(ktime_sub(ktime_get(), priv->last_rise));
		priv->pending = false;
		priv->active = false;

		if (pulse_ns >= DEBOUNCE_NS) {
			/* Real wake event */
			priv->wake_count++;
			priv->last_duration_ns = pulse_ns;
			dev_dbg(priv->dev,
				"wake #%u, duration=%lld ns\n",
				priv->wake_count, pulse_ns);
		} else {
			dev_dbg(priv->dev,
				"glitch filtered: %lld ns < %lld ns\n",
				pulse_ns, (s64)DEBOUNCE_NS);
		}
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/*  sysfs attributes                                                   */
/* ------------------------------------------------------------------ */

static ssize_t wake_count_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct esp32_wor *priv = dev_get_drvdata(dev);
	u32 count;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	count = priv->wake_count;
	spin_unlock_irqrestore(&priv->lock, flags);

	return sysfs_emit(buf, "%u\n", count);
}

static ssize_t wake_count_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct esp32_wor *priv = dev_get_drvdata(dev);
	u32 val;
	unsigned long flags;

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);
	priv->wake_count = val;
	spin_unlock_irqrestore(&priv->lock, flags);

	return count;
}
static DEVICE_ATTR_RW(wake_count);

static ssize_t active_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct esp32_wor *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n",
			  gpiod_get_value(priv->wake_gpio) ? 1 : 0);
}
static DEVICE_ATTR_RO(active);

static ssize_t last_wake_ns_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct esp32_wor *priv = dev_get_drvdata(dev);
	s64 ns;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	ns = ktime_to_ns(priv->last_rise);
	spin_unlock_irqrestore(&priv->lock, flags);

	return sysfs_emit(buf, "%lld\n", ns);
}
static DEVICE_ATTR_RO(last_wake_ns);

static ssize_t last_duration_ns_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct esp32_wor *priv = dev_get_drvdata(dev);
	s64 ns;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	ns = priv->last_duration_ns;
	spin_unlock_irqrestore(&priv->lock, flags);

	return sysfs_emit(buf, "%lld\n", ns);
}
static DEVICE_ATTR_RO(last_duration_ns);

static struct attribute *esp32_wor_attrs[] = {
	&dev_attr_wake_count.attr,
	&dev_attr_active.attr,
	&dev_attr_last_wake_ns.attr,
	&dev_attr_last_duration_ns.attr,
	NULL,
};
ATTRIBUTE_GROUPS(esp32_wor);

/* ------------------------------------------------------------------ */
/*  Probe / Remove                                                     */
/* ------------------------------------------------------------------ */

static int esp32_wor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32_wor *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	/* ---- GPIO ---- */
	priv->wake_gpio = devm_gpiod_get(dev, "wake", GPIOD_IN);
	if (IS_ERR(priv->wake_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->wake_gpio),
				     "failed to get wake GPIO\n");

	/* ---- IRQ from GPIO ---- */
	priv->irq = gpiod_to_irq(priv->wake_gpio);
	if (priv->irq < 0)
		return dev_err_probe(dev, priv->irq,
				     "failed to map GPIO to IRQ\n");

	/*
	 * Request both edges so we catch:
	 *   - rising  = ESP32 asserted TRIGGER_DETECTED
	 *   - falling = ESP32 de-asserted (going idle/sleep)
	 */
	ret = devm_request_irq(dev, priv->irq, esp32_wor_irq,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       DRIVER_NAME, priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ %d\n",
				     priv->irq);

	/* ---- sysfs ---- */
	ret = devm_device_add_groups(dev, esp32_wor_groups);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create sysfs\n");

	dev_info(dev, "ESP32 Wake-on-Radio: GPIO→IRQ %d, ready\n", priv->irq);
	return 0;
}

/* ------------------------------------------------------------------ */
/*  Device Tree matching                                               */
/* ------------------------------------------------------------------ */

static const struct of_device_id esp32_wor_of_match[] = {
	{ .compatible = "custom,esp32-wor" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, esp32_wor_of_match);

static struct platform_driver esp32_wor_driver = {
	.driver = {
		.name           = DRIVER_NAME,
		.of_match_table = esp32_wor_of_match,
	},
	.probe = esp32_wor_probe,
};
module_platform_driver(esp32_wor_driver);

MODULE_AUTHOR("Jorge Mendoza");
MODULE_DESCRIPTION("ESP32-S3 Wake-on-Radio interrupt-driven GPIO driver");
MODULE_LICENSE("GPL");
