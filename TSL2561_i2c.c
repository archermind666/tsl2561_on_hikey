/******************************************************************************
 * MODULE       : TSL2561_i2c.c
 * FUNCTION     : Driver source for TSL2561 in Candela Sensor IC.
 * REMARKS      :
 *****************************************************************************/
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/slab.h> 
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include "TSL2561_i2c.h"

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1
/******************************* define *******************************/
/* structure of peculiarity to use by system */
typedef struct {
    struct i2c_client  *client;      /* structure pointer for i2c bus            */
	struct regulator *vdd;
    struct hrtimer     timer;        /* structure for timer handler              */
    struct work_struct work;         /* structure for work queue                 */
    struct input_dev   *input_dev;   /* structure pointer for input device       */
    int                delay_time;   /* delay time to set from application       */
    unsigned char      measure_state;
} CANDELA_DATA;

/* logical functions */
static int __init      		candela_init(void);
static void __exit          candela_exit(void);
static int                  candela_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  candela_remove(struct i2c_client *client);
static int                  candela_input_open(struct input_dev *input);
static void                 candela_input_close(struct input_dev *input);
static void                 candela_work_func(struct work_struct *work);
static enum hrtimer_restart candela_timer_func(struct hrtimer *timer);

/* file system */
static ssize_t TSL2561_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t TSL2561_show_enable(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t TSL2561_store_delaytime(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t TSL2561_show_delaytime(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t TSL2561_show_data(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t TSL2561_show_version(struct device *dev, struct device_attribute *attr, char *buf);
/* access function */
static int candela_access_init(struct i2c_client *client);
static int candela_access_exit(struct i2c_client *client);
static int candela_access_write_measurement_state(struct i2c_client *client, unsigned char val);
static int candela_access_read_measurement_data(struct i2c_client *client, unsigned long *val);

/* driver function */
static void make_timer_val(unsigned long data, unsigned long *sec, unsigned long *nsec);

/**************************** variable declaration ****************************/
static const char              TSL2561_driver_ver[] = TSL2561_DRIVER_VER;
static struct workqueue_struct *tsl_workqueue;

/**************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id candela_id[] = {
    { TSL2561_I2C_NAME, 0 }, /* tsl TSL2561 driver */
    { }
};
static const struct of_device_id TSL2561_i2c_of_ids[] = {
	{ .compatible = "tsl,TSL2561_i2c"},
	{ }
};

/*MODULE_DEVICE_TABLE(of, TSL2561_of_ids);*/

/* represent an I2C device driver */
static struct i2c_driver TSL2561_i2c_driver = {
    .driver = {                      /* device driver model driver */
        .name  = TSL2561_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = TSL2561_i2c_of_ids,
    },
    .probe    = candela_probe,         /* callback for device binding */
    .remove   = candela_remove,        /* callback for device unbinding */
    .shutdown = NULL,
    //.suspend  = NULL,
    //.resume   = NULL,
    .id_table = candela_id,            /* list of I2C devices supported by this driver */
};

static DEVICE_ATTR(enable, 0664, TSL2561_show_enable, TSL2561_store_enable);
static DEVICE_ATTR(delay, 0664, TSL2561_show_delaytime, TSL2561_store_delaytime);
static DEVICE_ATTR(data,  S_IRUGO, TSL2561_show_data, NULL);
static DEVICE_ATTR(version,  S_IRUGO, TSL2561_show_version, NULL);

static struct attribute *TSL2561_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_data.attr,
    &dev_attr_delay.attr,
    &dev_attr_version.attr,
    NULL  /* need to NULL terminate the list of attributes */
};

static const struct attribute_group TSL2561_attr_group = {
    .attrs = TSL2561_attributes,
	.name = "TSL2561_attr",
};



/************************************************************
 *                      logic function                      *
 ***********************************************************/
/******************************************************************************
 * NAME       : candela_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
static int __init candela_init(void)
{
    int result;

	printk("[wj hikey960]candela_init begin\n");

    tsl_workqueue = create_singlethread_workqueue("tsl_workqueue");
    if (!tsl_workqueue) {
        return (-ENOMEM);
    }
    result = i2c_add_driver(&TSL2561_i2c_driver);

	printk("[wj hikey960]candela_init end\n");

    return (result);
}
int TSL2561_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
{
	int res = 0;
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_msg msg[2];
#endif
	switch (i2c_flag) {
	case I2C_FLAG_WRITE:
#ifdef CONFIG_MTK_I2C_EXTENSION
		client->addr &= I2C_MASK_FLAG;
		res = i2c_master_send(client, buf, count);
		client->addr &= I2C_MASK_FLAG;
#else
		res = i2c_master_send(client, buf, count);
#endif
		break;

	case I2C_FLAG_READ:
#ifdef CONFIG_MTK_I2C_EXTENSION
		client->addr &= I2C_MASK_FLAG;
		client->addr |= I2C_WR_FLAG;
		client->addr |= I2C_RS_FLAG;
		res = i2c_master_send(client, buf, (count << 8) | 1);
		client->addr &= I2C_MASK_FLAG;
#else
		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = buf;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = count;
		msg[1].buf = buf;
		res = i2c_transfer(client->adapter, msg, sizeof(msg)/sizeof(msg[0]));
#endif
		break;
	default:
		printk("TSL2561_i2c_master_operate i2c_flag command not support!\n");
		break;
	}
	if (res < 0)
		goto EXIT_ERR;
	return res;
EXIT_ERR:
	printk("TSL2561_i2c_master_operate fail res = %d\n", res);
	return res;
}

static int TSL2561_regulators_init(CANDELA_DATA *tsl2561)
{
	int ret;
	struct device *dev = &tsl2561->client->dev;

	tsl2561->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(tsl2561->vdd)) {
		ret = PTR_ERR(tsl2561->vdd);
		dev_err(dev, "TSL2561failed to get vdd regulator %d\n", ret);
		return ret;
	}

	ret = regulator_set_voltage(tsl2561->vdd, 1825000, 1825000);
	if (ret) {
		dev_err(dev, "TSL2561failed to set avdd voltage %d\n", ret);
		return ret;
	}

	ret = regulator_enable(tsl2561->vdd);
	if (ret) {
		dev_err(dev, "TSL2561failed to enable vdd %d\n", ret);
		return ret;
	}
	return 0;
}

/******************************************************************************
 * NAME       : candela_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int candela_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    CANDELA_DATA *candela;
    int        result;

    printk("[wj hikey960]candela_probe begin\n");
	printk("[wj hikey960]called %s of TSL2561!!\n", __func__);
	
    result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
    if (!result) {
        printk("[wj hikey960]%s: need I2C_FUNC_I2C\n", __func__);
        result = -ENODEV;
        goto err_first;
    }
    candela = kzalloc(sizeof(CANDELA_DATA), GFP_KERNEL);
    if (candela == NULL) {
        result = -ENOMEM;
        goto err_first;
    }
    INIT_WORK(&candela->work, candela_work_func);
    candela->client = client;
    i2c_set_clientdata(client, candela);

	/*set power*/
	result = TSL2561_regulators_init(candela);
    if (result) {
        printk("[wj hikey960]%s set power errorX\n", __func__);
        goto err_fourth;
    }
    /* set input device */
    candela->input_dev = input_allocate_device();
    if (!candela->input_dev) {
        result = -ENOMEM;
        dev_err(&candela->client->dev, "input device allocate failed\n");
        goto err_second;
    }
    /* set input event */
    input_set_drvdata(candela->input_dev, candela);
    set_bit(EV_MSC, candela->input_dev->evbit);
    set_bit(EV_SYN, candela->input_dev->evbit);
    set_bit(CANDELA, candela->input_dev->mscbit);

    /* set event name */
    candela->input_dev->name  = TSL2561_INPUT_NAME;
    candela->input_dev->open  = candela_input_open;
    candela->input_dev->close = candela_input_close;

    /* register the device */
    result = input_register_device(candela->input_dev);
    if (result) {
        printk( "[wj hikey960]unable to register input polled device %s: %d\n", candela->input_dev->name, result);
        goto err_third;
    }

    /* Register sysfs hooks */
    result = sysfs_create_group(&client->dev.kobj, &TSL2561_attr_group);
    if (result) {
        printk("[wj hikey960]%s sysfs_create_groupX\n", __func__);
        goto err_fourth;
    }

    /* timer process */
    hrtimer_init(&candela->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    candela->timer.function = candela_timer_func;

    /* initialize static variable */
    candela->delay_time = MIN_DELAY_TIME;
	 printk("[wj hikey960]candela_probe end\n");
    return (0);

err_fourth:
    input_unregister_device(candela->input_dev);
err_third:
    input_free_device(candela->input_dev);
err_second:
    kfree(candela);
err_first:
   
    return (result);

}

/******************************************************************************
 * NAME       : candela_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int candela_remove(struct i2c_client *client)
{
    CANDELA_DATA *candela;
    int        result;

    candela  = i2c_get_clientdata(client);
    result = hrtimer_cancel(&candela->timer);
    if (result == 0) {
        cancel_work_sync(&candela->work);
        sysfs_remove_group(&client->dev.kobj, &TSL2561_attr_group);
        input_unregister_device(candela->input_dev);
        input_free_device(candela->input_dev);
        kfree(candela);
    } else {
        printk("[wj hikey960]%s: Can't cancel timer.\n", __func__);
    }


    return (result);
}

/******************************************************************************
 * NAME       : candela_work_func
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static void candela_work_func(struct work_struct *work)
{
    int           result;
    CANDELA_DATA    *candela;
    long          wait_sec;
    unsigned long wait_nsec;
    unsigned long  data;

	printk("[wj hikey960]candela_work_func begin\n");


    candela = container_of(work, CANDELA_DATA, work);
	printk("[wj hikey960]MEASURE_ON\n");
    if (candela->measure_state == MEASURE_OFF) {
		printk("[wj hikey960]MEASURE_OFF\n");
        return;
    }

    result = candela_access_read_measurement_data(candela->client, &data);
    if (result < 0) {
        printk("[wj hikey960]%s: ERROR! read data function.\n", __func__);
        return;
    }

    input_event(candela->input_dev, EV_MSC, CANDELA, data);
    input_sync(candela->input_dev);

    make_timer_val(candela->delay_time, &wait_sec, &wait_nsec);
    /*result = */
		hrtimer_start(&candela->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
    /*if (result != 0) {
        printk(KERN_ERR "%s: Can't start timer\n", __func__);
        return;
    }*/
	printk("[wj hikey960]candela_work_func end\n");
    return;
}

/******************************************************************************
 * NAME       : candela_timer_func
 * FUNCTION   : call work function (thread of timer)
 * REMARKS    :
 *****************************************************************************/
static enum hrtimer_restart candela_timer_func(struct hrtimer *timer)
{
    CANDELA_DATA *candela;
    int        result;

    printk("[wj hikey960]candela_timer_func begin\n");

    candela  = container_of(timer, CANDELA_DATA, timer);
    result = queue_work(tsl_workqueue, &candela->work);
    if (result == 0) {
        printk(KERN_ERR "%s: Can't register que.\n", __func__);
        printk(KERN_ERR "%s: result = 0x%x\n", __func__, result);
    }

	printk("[wj hikey960]candela_timer_func end\n");
	
    return (HRTIMER_NORESTART);
}

/******************************************************************************
 * NAME       : candela_input_open
 * FUNCTION   : initialize device and open process
 * REMARKS    :
 *****************************************************************************/
static int candela_input_open(struct input_dev *input)
{
    int        result;
    CANDELA_DATA *candela;

	printk("[wj hikey960]candela_input_open begin\n");

    candela = input_get_drvdata(input);

    /* initialize IC */
    result = candela_access_init(candela->client);
    if (result) {
        printk("[wj hikey960]%s: Can't initialize IC. result = 0x%x\n", __func__, result);
    }

	printk("[wj hikey960]candela_input_open end\n");
	
    return (result);
}

/******************************************************************************
 * NAME       : candela_input_close
 * FUNCTION   : stop device and close process
 * REMARKS    :
 *****************************************************************************/
static void candela_input_close(struct input_dev *input)
{
    CANDELA_DATA *candela;

	printk("[wj hikey960]candela_input_close begin\n");

    /* copy client data */
    candela = input_get_drvdata(input);

    /* close IC */
    candela_access_exit(candela->client);

	printk("[wj hikey960]candela_input_close end\n");

    return;
}

/************************************************************
 *                   file system function                   *
 ***********************************************************/
/******************************************************************************
 * NAME       : TSL2561_show_enable
 * FUNCTION   : write to power on or off TSL2561
 * REMARKS    :
 *****************************************************************************/
static ssize_t TSL2561_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client;
    CANDELA_DATA        *candela;
    unsigned long     val;
    int               result;
    long              wait_sec;
    unsigned long     wait_nsec;
	
    printk("[wj hikey960]TSL2561_store_enable begin\n");
    /* initialize data */
    client = to_i2c_client(dev);
    candela  = i2c_get_clientdata(client);
    val    = simple_strtoul(buf, NULL, 10);
    result = -1;

    if ((val == MEASURE_ON) || (val == MEASURE_OFF)) {
        result = candela_access_write_measurement_state(client, (unsigned char)val);
        if (result < 0) {
            printk( "[wj hikey960]%s: Can't write candela_access_write_measurement_state function. Result = 0x%x.\n", __func__, result);
        } else {
            if (val == MEASURE_ON) {
                if (candela->measure_state == MEASURE_OFF) {
                    /* set to start timer to que */
                    make_timer_val(candela->delay_time, &wait_sec, &wait_nsec);
                    /*result = */
						hrtimer_start(&candela->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
                   /* if (result != 0) {
                        printk(KERN_ERR "%s: Can't start timer\n", __func__);
                    }*/
                }
                candela->measure_state = MEASURE_ON;
            } else { // val == MEASURE_OFF
                /* process which stop measurement */
                /* clear to timer que */
                hrtimer_cancel(&candela->timer);
                cancel_work_sync(&candela->work);
                candela->measure_state = MEASURE_OFF;
            }
        }
    } else {
        printk( "[wj hikey960]%s: Can't execute.", __func__);
    }

	printk("[wj hikey960]TSL2561_store_enable end\n");

    return (count);
}


/******************************************************************************
 * NAME       : TSL2561_show_enable
 * FUNCTION   : read to power on or off TSL2561
 * REMARKS    :
 *****************************************************************************/
static ssize_t TSL2561_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client;
    CANDELA_DATA        *candela;
    int               result;

	printk("[wj hikey960]TSL2561_show_enable begin\n");
    /* initialize data */
    client = to_i2c_client(dev);
    candela  = i2c_get_clientdata(client);

    result = sprintf(buf, "%d\n", candela->measure_state);
	printk("[wj hikey960]TSL2561_show_enable end\n");

    return (result);
}

/******************************************************************************
 * NAME       : TSL2561_store_delaytime
 * FUNCTION   : write delay time of TSL2561
 * REMARKS    :
 *****************************************************************************/
static ssize_t TSL2561_store_delaytime(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client;
    CANDELA_DATA        *candela;
    unsigned long     val;

	printk("[wj hikey960]TSL2561_store_delaytime begin\n");
    /* initialize data */
    client = to_i2c_client(dev);
    candela  = i2c_get_clientdata(client);
    val    = simple_strtoul(buf, NULL, 10);

    if (val < MIN_DELAY_TIME) {
        val = MIN_DELAY_TIME;
    }
    candela->delay_time = val;
	
	printk("[wj hikey960]TSL2561_store_delaytime end\n");
    return (count);
}

/******************************************************************************
 * NAME       : TSL2561_show_delaytime
 * FUNCTION   : read to delay time of TSL2561
 * REMARKS    :
 *****************************************************************************/
static ssize_t TSL2561_show_delaytime(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client;
    CANDELA_DATA        *candela;
    int               result;

	printk("[wj hikey960]TSL2561_show_delaytime begin\n");
    /* initialize data */
    client = to_i2c_client(dev);
    candela  = i2c_get_clientdata(client);

    result = sprintf(buf, "%d\n", candela->delay_time);
	printk("[wj hikey960]TSL2561_show_delaytime end\n");

    return (result);
}

/******************************************************************************
 * NAME       : TSL2561_show_data
 * FUNCTION   : read candelaure data from TSL2561
 * REMARKS    :
 *****************************************************************************/
static ssize_t TSL2561_show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client;
    int               result;
    unsigned long      pdata;

	printk("[wj hikey960]TSL2561_show_data begin\n");

    /* initialize data */
    client = to_i2c_client(dev);

    result = candela_access_read_measurement_data(client, &pdata);
    if (result >= 0) {
        result = sprintf(buf, "%ld\n", pdata);
    }
	printk("[wj hikey960]TSL2561_show_data end\n"); 
	
    return (result);
}

/******************************************************************************
 * NAME       : TSL2561_show_version
 * FUNCTION   : read version
 * REMARKS    :
 *****************************************************************************/
static ssize_t TSL2561_show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    int result;

	printk("[wj hikey960]TSL2561_show_version begin\n"); 
    result = sprintf(buf, "driver version %s\n", TSL2561_driver_ver);
	printk("[wj hikey960]TSL2561_show_version end\n");
    return (result);
}

/************************************************************
 *                     access function                      *
 ***********************************************************/
/******************************************************************************
 * NAME       : candela_access_init
 * FUNCTION   : initialize TSL2561
 * REMARKS    :
 *****************************************************************************/
static int candela_access_init(struct i2c_client *client)
{
    u8 databuf[2];
	int res = 0;

	databuf[0] = TSL2561_Control;
	databuf[1] = 0x03;
	res = TSL2561_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	printk("TSL2561 TSL2561_Control poweron command!\n");
	
	databuf[0] = TSL2561_Timing;
	databuf[1] = 0x00;
	res = TSL2561_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	printk("TSL2561 TSL2561_Timing poweron command!\n");	
	
	databuf[0] = TSL2561_Interrupt;
	databuf[1] = 0x00;
	res = TSL2561_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	printk("TSL2561 TSL2561_Interrupt poweron command!\n");
	
EXIT_ERR:	
	printk("[wj hikey960]candela_access_init end\n"); 
    return (res);
}

/******************************************************************************
 * NAME       : candela_access_exit
 * FUNCTION   : shutdown TSL2561
 * REMARKS    :
 *****************************************************************************/
static int candela_access_exit(struct i2c_client *client)
{
    int res;
	u8 databuf[2];
	printk("[wj hikey960]candela_access_exit begin\n");
    

	databuf[0] = TSL2561_Control;
	databuf[1] = 0x00;
	res = TSL2561_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		printk("i2c_master_send function err\n");
	}
	printk("TSL2561 TSL2561_Control powerdown command!\n");
    return (res);
}

/******************************************************************************
 * NAME       : candela_access_write_measurement_state
 * FUNCTION   : measurement on or off to TSL2561
 * REMARKS    : 1 is measurement on, 0 is measurement off
 *****************************************************************************/
static int candela_access_write_measurement_state(struct i2c_client *client, unsigned char val)
{
    int res;
	u8 databuf[2] = { 0 };
	
	printk("[wj hikey960]candela_access_write_measurement_state begin\n");

    if (val == MEASURE_ON) 
	{
		databuf[0] = TSL2561_Control;
		databuf[1] = 0x03;
		res = TSL2561_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			printk("i2c_master_send function err\n");
			goto EXIT_ERR;
		}
		pr_warn("TSL2561 TSL2561_Timing poweron command!\n");	
    } 
	else if (val == MEASURE_OFF) 
	{
		databuf[0] = TSL2561_Control;
		databuf[1] = 0x00;
		res = TSL2561_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			printk("i2c_master_send function err\n");
			goto EXIT_ERR;
		}
		pr_warn("TSL2561 TSL2561_Timing poweron command!\n");	     
    }
	printk("[wj hikey960]candela_access_write_measurement_state end\n");
EXIT_ERR:
	return (res);
}

/******************************************************************************
 * NAME       : calculateLux
 * FUNCTION   : 
 * REMARKS    : 
 *****************************************************************************/

unsigned long calculateLux(unsigned int iGain, unsigned int tInt,int iType)
{
	if(ch1 == 0)
	{ 
		return 0;
	}
	if(ch0/ch1 < 2 && ch0 > 4900)
	{
		return -1;  //ch0 out of range, but ch1 not. the lux is not valid in this situation.
	}
	switch (tInt)
	{
		case 0:  // 13.7 msec
		chScale = CHSCALE_TINT0;
		break;
		case 1: // 101 msec
		chScale = CHSCALE_TINT1;
		break;
		default: // assume no scaling
		chScale = (1 << CH_SCALE);
		break;
	}
	if (!iGain)  
	{
		chScale = chScale << 4; // scale 1X to 16X
	}
	// scale the channel values
	channel0 = (ch0 * chScale) >> CH_SCALE;
	channel1 = (ch1 * chScale) >> CH_SCALE;

	ratio1 = 0;
	if (channel0!= 0)
	{	
		ratio1 = (channel1 << (RATIO_SCALE+1))/channel0;
	}
	// round the ratio value
	ratio = (ratio1 + 1) >> 1;

	switch (iType)
	{
		case 0: // T package
			if ((ratio >= 0) && (ratio <= K1T))
			{b=B1T; m=M1T;}
			else if (ratio <= K2T)
			{b=B2T; m=M2T;}
			else if (ratio <= K3T)
			{b=B3T; m=M3T;}
			else if (ratio <= K4T)
			{b=B4T; m=M4T;}
			else if (ratio <= K5T)
			{b=B5T; m=M5T;}
			else if (ratio <= K6T)
			{b=B6T; m=M6T;}
			else if (ratio <= K7T)
			{b=B7T; m=M7T;}
			else if (ratio > K8T)
			{b=B8T; m=M8T;}
			break;
		case 1:// CS package
			if ((ratio >= 0) && (ratio <= K1C))
			{b=B1C; m=M1C;}
			else if (ratio <= K2C)
			{b=B2C; m=M2C;}
			else if (ratio <= K3C)
			{b=B3C; m=M3C;}
			else if (ratio <= K4C)
			{b=B4C; m=M4C;}
			else if (ratio <= K5C)
			{b=B5C; m=M5C;}
			else if (ratio <= K6C)
			{b=B6C; m=M6C;}
			else if (ratio <= K7C)
			{b=B7C; m=M7C;}
	}
	temp=((channel0*b)-(channel1*m));
	if(temp<0) temp=0;
	temp+=(1<<(LUX_SCALE-1));

	lux=temp>>LUX_SCALE;
	return (lux);
	
}

/******************************************************************************
 * NAME       : candela_access_read_measurement_data
 * FUNCTION   : Read measurement data from TSL2561
 * REMARKS    : 
 *****************************************************************************/
static int candela_access_read_measurement_data(struct i2c_client *client, unsigned long *val)
{
	int res = 0;

	u8 databuf[2] = { 0 };

	databuf[0] = TSL2561_Control;
	databuf[1] = 0x03;
	res = TSL2561_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		printk("i2c_master_send poweron err\n");
		goto EXIT_ERR;
	}
	pr_warn("TSL2561 TSL2561_Timing poweron command!\n");	
	mdelay(1000);


	databuf[0] = TSL2561_Channal0L;
	res = TSL2561_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_READ);
	CH0_LOW = databuf[0];
	if (res <= 0) {
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	pr_warn("TSL2561_Channal0L TSL2561_REG_PS_DATA value value_low = %x\n", databuf[0]);
	
	databuf[0] = TSL2561_Channal0H;
	res = TSL2561_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_READ);
	CH0_HIGH = databuf[0];
	if (res <= 0) {
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	pr_warn("TSL2561_Channal0H TSL2561_REG_PS_DATA value value_low = %x\n", databuf[0]);
	
	databuf[0] = TSL2561_Channal1L;
	res = TSL2561_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_READ);
	CH1_LOW = databuf[0];
	if (res <= 0) {
		printk(" i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	pr_warn("TSL2561_Channal1L TSL2561_REG_PS_DATA value value_low = %x\n", databuf[0]);
	
	databuf[0] = TSL2561_Channal1H;
	res = TSL2561_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_READ);
	CH1_HIGH = databuf[0];
	if (res <= 0) {
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	pr_warn("TSL2561_Channal1H  TSL2561_REG_PS_DATA value value_low = %x\n", databuf[0]);
	
	
	databuf[0] = TSL2561_Control;
	databuf[1] = 0x00;
	res = TSL2561_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	pr_warn("TSL2561 TSL2561_Timing powerdown command!\n");	
	
	ch0 = (CH0_HIGH<<8) | CH0_LOW;
    ch1 = (CH1_HIGH<<8) | CH1_LOW;
	calculateLux(0,0,0);
	pr_warn("[wj hikey960]calculateLux  lux = %ld\n", lux);
	
	*val = lux;
	
EXIT_ERR:
	return (res);
}

/******************************************************************************
 * NAME       : make_timer_val
 * FUNCTION   : Convert from NS time to ktime_set function value. 
 * REMARKS    :
 *****************************************************************************/
static void make_timer_val(unsigned long data, unsigned long *sec, unsigned long *nsec)
{
    
	printk("[wj hikey960]make_timer_val begin\n");
    /* the setting value from application */
    *sec  = (data / SN_TIME_UNIT);
    *nsec = (data - (*sec * SN_TIME_UNIT));
    
	printk("[wj hikey960]make_timer_val end\n");
    return;
}

/******************************************************************************
 * NAME       : candela_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
static void __exit candela_exit(void)
{
	printk("[wj hikey960]candela_exit\n");
    return 0;
}

MODULE_DESCRIPTION("TSL2561 Candela Sensor Driver");
MODULE_LICENSE("GPL");

module_init(candela_init);
module_exit(candela_exit);

