/***************************************************************************
 *   Copyright (C) 2007 by Igor Bykov   *
 *   igrbkv@rambler.ru   *
 ***************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>       /* Specifically, a module */
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>      /* Necessary because we use proc fs */
#include <linux/sched.h>        /* For putting processes to sleep and waking them up */
#include <linux/interrupt.h>
#include <linux/device.h>
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/time.h>

#include "pov.h"
/*
1. Все платы сидят на одном прерывании (один обработчик прерывания)
возможные IRQ - 5,7,10,11,12
2. Все платы работают на одной частоте 1800/3600 Гц
3. Данные из фреймов переписываются в буфер пользователя как есть
4. Параметры: номер перывания - irq (по умолчанию 7), 
маска используемых каналов - pov_mask(по умолчанию 15, ПОВ1 и ПОВ2), 
частота	дискретизации - sample_rate (по умолчанию 1800)
5. Соответствующие каналам названия устройств в драйвере: 
ПОВ1/1 - /dev/pov0, ПОВ1/2 - /dev/pov1,..., ПОВ4/2 - /dev/pov7
*/

#define MAX_BOARDS	4
#define MAX_CHANNELS (MAX_BOARDS*2)

#define FIFO_SIZE 0x2000

#define FRAME_COUNTER_MASK 0x3f
#define MAX_FRAME_COUNTER 0x40
#define POV_FRAME_SIZE 35
#define FRAME_DATA_SIZE 32
#define MSB_MASK 0x7f
#define FRAME_CLOSE_BYTE 0x73
#define READ_COUNTER_TIMEOUT 10
#define AD_OVERFLOW  0x4000
#define COUNTER_MASK  0x1FFF
//Флаг первого отсчета данных
#define START_COUNT_FLAG 1

static void pov_do_work(struct work_struct *w);
struct workqueue_struct *work_queue;
static struct work_struct work;
spinlock_t irq_lock = SPIN_LOCK_UNLOCKED;

#define DEFAULT_IRQ 7	//или 12 (PS/2 мышь)
static int irq = DEFAULT_IRQ;
module_param(irq, int, S_IRUGO);

//Маска задействованных каналов
#define POV11 0x1
#define POV12 0x2
#define POV21 0x4
#define POV22 0x8
#define POV31 0x10
#define POV32 0x20
#define POV41 0x40
#define POV42 0x80
#define POV_MASK	(POV11|POV12|POV21|POV22|POV31|POV32|POV41|POV42)

static int pov_mask = (POV11|POV12|POV21|POV22);
module_param(pov_mask, int, S_IRUGO);

//частота переменного тока
#define AC_FREQUENCY	50
#define BASE_SAMPLE_RATE 1800
#define MAX_SAMPLE_RATE_FACTOR 2
//FIXME Задаваемая частота дискретизации может не совпадать с ПУ-шной
//(см. 2-ой байт заголовка пакета)
int sample_rate = BASE_SAMPLE_RATE;
module_param(sample_rate, int, S_IRUGO);

long long irq_counter;
long long work_counter;

enum dev_states {	//Переходы:
	STATE_UNUSED,	//=init_module=>STATE_CLOSED
 	STATE_CLOSED,	//=pov_open=>STATE_OPENED
 	STATE_ERROR,	//=pov_release=>STATE_CLOSED
 	STATE_OPENED	//=hard_error=>STATE_ERROR  или =pov_release=>STATE_CLOSED
};
enum parse_states {
    PARSE_INITIAL, 
    PARSE_HEAD, 
    PARSE_MSB, 
    PARSE_LSB, 
    PARSE_TAIL
};

struct sample {
    struct timespec timestamp;
    unsigned char data[FRAME_DATA_SIZE];
};

struct pov_dev {
	enum dev_states state;
	int error;
	enum parse_states parse_state;

	//Порты
	int fifo8_port;
	int count_port;
	int state_port;

	int cur_fifo_counter;					//Текущее значение счетчика  fifo
	int last_fifo_counter;					//Предыдущее значение счетчика fifo

	int cur_frame;							//Номер разбираемого фрейма
	int last_frame;							//Номер последнего разобранного фрейма (из заголовка фрейма)
	int idx;								//Текущий lsb-байт фрейма

	long long bytes_read;
	struct mutex mut;
    struct completion compl;
	struct cdev cdev;

	unsigned char *buf;
	int buf_size;

	//статистика
	int err_fifo_overflows;				//число переполнений FIFO
	int err_skipped_frames;				//пропусков кадров
};

struct pov_dev channels[ MAX_CHANNELS ] = {
	//ПОВ1
	[0].fifo8_port = 0x150,
 	[0].count_port = 0x154,
  	[0].state_port = 0x158,

	[1].fifo8_port = 0x151,
  	[1].count_port = 0x156,
  	[1].state_port = 0x158,

   //ПОВ2
   	[2].fifo8_port = 0x15a,
  	[2].count_port = 0x15e,
  	[2].state_port = 0x162,

	[3].fifo8_port = 0x15b,
	[3].count_port = 0x160,
	[3].state_port = 0x162,

 	//ПОВ3
  	[4].fifo8_port = 0x180,
	[4].count_port = 0x184,
	[4].state_port = 0x188,

  	[5].fifo8_port = 0x181,
	[5].count_port = 0x186,
	[5].state_port = 0x188,

  	//ПОВ4
    [6].fifo8_port = 0x18a,
  	[6].count_port = 0x18e,
  	[6].state_port = 0x192,

	[7].fifo8_port = 0x18b,
  	[7].count_port = 0x190,
  	[7].state_port = 0x192,
};

int pov_major, pov_minor;
dev_t dev;
static struct class *pov_class;
struct timespec timestamp;
//Период между отсчетами
s64 count_duration_ns;
//Период между аналогами внутри отсчета 
s64 analog_duration_ns;

static void dev_start(struct pov_dev *dev)
{
    unsigned short count;
	unsigned char state = inb(dev->state_port);
    count = inw(dev->count_port);
    printk(KERN_DEBUG "pov: start cur state:%x count:%x\n", state, count);
	//порт фифо А четный, а В - нечетный
	state |= (dev->fifo8_port%2+1);
	outb(state, dev->state_port);
    count = inw(dev->count_port);
    printk(KERN_DEBUG "pov: start set state:%x count:%x\n", state, count);
    outb(0, dev->fifo8_port);
}

static void dev_stop(struct pov_dev *dev)
{
    unsigned short count;
	unsigned char state = inb(dev->state_port);
    count = inw(dev->count_port);
    printk(KERN_DEBUG "pov: stop cur state:%x count:%x\n", state, count);
	//порт фифо А четный, а В - нечетный
	state &= ~(dev->fifo8_port%2+1);
	outb(state, dev->state_port);
    count = inw(dev->count_port);
    printk(KERN_DEBUG "pov: stop set state:%x count:%x\n", state, count);
    printk(KERN_DEBUG "pov: irq_counter:%lld work_counter:%lld\n", 
        irq_counter, work_counter);
}

static void dev_hard_error(struct pov_dev *dev, int err)
{
	if (dev->state == STATE_OPENED) {
		dev->state = STATE_ERROR;
		dev->error = err;
		dev_stop(dev);
	}
}

static int  dev_calc_data_count(struct pov_dev *dev, int count)
{
	int res;
	res = count - dev->last_fifo_counter;
	if (res < 0)
		res += FIFO_SIZE;
    dev->last_fifo_counter = count;
	return res;
}

static int dev_open( struct pov_dev *dev )
{
	if (dev->state == STATE_UNUSED)
		return -ENODEV;

	if (dev->state != STATE_CLOSED)
		return -EPERM;

	dev->error = 0;

    dev->bytes_read = 0;
	dev->cur_fifo_counter = 0;
	dev->last_fifo_counter = 0;
	dev->parse_state = PARSE_INITIAL;
	dev->last_frame = -1;
    
    mutex_init(&dev->mut);
    dev->buf = 0;
    dev->buf_size = 0;

	dev->err_fifo_overflows = 0;
	dev->err_skipped_frames = 0;

	dev->state = STATE_OPENED;
	dev_start(dev);
	return 0;
}

static int dev_release(struct pov_dev *dev)
{
	if (dev->state == STATE_UNUSED)
		return -ENODEV;

	if (dev->state != STATE_CLOSED)	{
		if (dev->state == STATE_OPENED)
            dev_stop(dev);
        mutex_destroy(&dev->mut);
		dev->state = STATE_CLOSED;
    }

	return 0;
}


static void adjust_timestamp(struct timespec *ts, int bytes_initially, 
    int bytes_rest)
{
    s64 ns1, ns2;
    ns1 = count_duration_ns*bytes_initially;
    ns1 = do_div(ns1, POV_FRAME_SIZE);
    //FIXME Число аналогов не должно быть больше 16, 
    //а bytes_initially - bytes_rest > 35
    ns2 = analog_duration_ns*(bytes_initially - bytes_rest);
    ns2 = do_div(ns2, 16);
    timespec_add_ns(ts, -(ns1+ns2));
}

static void shift_timestamp(struct timespec *ts)
{
    timespec_add_ns(ts, count_duration_ns);
}

static void dev_read(struct pov_dev *dev)
{
    struct timespec ts, *pts;
    int fifo_counter;
    int bytes_initially, bytes_rest;
    
    //Сохранить время и счетчик
    spin_lock_irq(&irq_lock);
    ts = timestamp;
    fifo_counter = dev->cur_fifo_counter;
    spin_unlock_irq(&irq_lock);

    mutex_lock(&dev->mut);
    if (dev->state == STATE_OPENED && dev->buf_size) {
        unsigned char byte;
        int timestamp_adjusted = 0;
        int skipped_frames;
 
        if (fifo_counter & AD_OVERFLOW) {
            dev->err_fifo_overflows++;
            printk(KERN_DEBUG "pov: overflow:%x\n", fifo_counter);
            goto err_exit;
        }
        
        fifo_counter &= COUNTER_MASK;
        bytes_rest = bytes_initially = dev_calc_data_count(dev, fifo_counter);

        while (bytes_rest && dev->buf_size) {
            byte = inb(dev->fifo8_port);
            dev->bytes_read++;
            bytes_rest--; 
retry:
            switch (dev->parse_state) {
                case PARSE_INITIAL:	{
                    if ((byte & 0xC0) == 0x80) {
                        dev->parse_state = PARSE_HEAD;
                        dev->cur_frame = byte & FRAME_COUNTER_MASK;
                    }
                    break;
                }
                case PARSE_HEAD: {
                    if ((byte & 0xCF) == 0x80) {
                        if (!timestamp_adjusted) { 
                            adjust_timestamp(&ts, bytes_initially, bytes_rest);
                            timestamp_adjusted = 1;
                            ts.tv_nsec |= START_COUNT_FLAG;
                        } else {
                            shift_timestamp(&ts);
                            ts.tv_nsec &= ~START_COUNT_FLAG;
                        }

                        if (dev->last_frame == -1)
                            dev->last_frame = dev->cur_frame - 1;
                        skipped_frames = dev->cur_frame - dev->last_frame;
                        if (skipped_frames < 0)
                            skipped_frames += MAX_FRAME_COUNTER;
                        if (--skipped_frames > 0) {
                            dev->err_skipped_frames++;
                            printk(KERN_DEBUG "pov:skipped_frames:%d\n", 
                                skipped_frames);
                            goto err_exit;
                        }        
                        pts = (struct timespec *)dev->buf;
                        put_user(ts, pts);
                        dev->buf += sizeof(struct timespec);
                        dev->buf_size -= sizeof(struct timespec);
                        dev->idx = 0;
                        dev->parse_state = PARSE_MSB;
                    } else if (dev->last_frame == -1) {
                        dev->parse_state = PARSE_INITIAL;
                        goto retry;
                    } else {                               
                        printk(KERN_DEBUG "pov:not head2 byte:%d\n", byte);
                        goto err_exit;
                    }
                    break;
                }
                case PARSE_MSB: {
                    if (byte & ~MSB_MASK) {
                        printk(KERN_DEBUG "pov:not MSB byte:%d\n", byte);
                        goto err_exit;
                    }
                    put_user(byte, dev->buf);
                    dev->buf++;
                    dev->buf_size--;
                    dev->parse_state = PARSE_LSB;
                    break;
                }
                case PARSE_LSB: {
                    put_user(byte, dev->buf);
                    dev->buf++;
                    dev->buf_size--;
                    if (++(dev->idx) == 16) {
                        dev->last_frame = dev->cur_frame;
                        dev->parse_state = PARSE_TAIL;
                    }
                    else
                        dev->parse_state = PARSE_MSB;
                    break;
                }
                case PARSE_TAIL: {
                    if (byte != FRAME_CLOSE_BYTE) {
                        printk(KERN_DEBUG "pov:not frame close byte:%d\n", byte);
                        goto err_exit;
                    }
                    dev->parse_state = PARSE_INITIAL;
                    break;
                }
            }
        }
        if (!dev->buf_size)
            complete(&dev->compl);
    }       
    mutex_unlock(&dev->mut);
    return;

err_exit:
    //FIXME Не выходить с ошибкой, а выйти
    //из функции, чтобы после обновления метки времени
    //начать новый блок данных, не забыв сдвинуться назад 
    //в буфере клиента
    dev_hard_error(dev, -EIO);
    complete(&dev->compl);
    mutex_unlock(&dev->mut);
}
 
static int pov_open(struct inode *inode, struct file *filp)
{
	struct pov_dev *dev; /* device information */

	dev = container_of(inode->i_cdev, struct pov_dev, cdev);
	filp->private_data = dev; /* for other methods */

	return dev_open( dev );

}

static int pov_release(struct inode *inode, struct file *filp)
{
	struct pov_dev *dev; /* device information */

	dev = container_of(inode->i_cdev, struct pov_dev, cdev);
	filp->private_data = dev; /* for other methods */

	return dev_release( dev );
}

irqreturn_t pov_irq_handler(int irq, void *dev_id)
{
    int i = 0;
    irq_counter++;
    
    //Считать текущее время
    getnstimeofday(&timestamp); 

    //Считать счетчики
	for (; i < MAX_CHANNELS; i++) {
		struct pov_dev * dev = &channels[i];
		if (dev->state != STATE_UNUSED) {
  			//Заодно сбросить прерывания
			inb(dev->state_port);
		    dev->cur_fifo_counter = inw(dev->count_port);
        }
    }

	queue_work(work_queue, &work);
	return IRQ_HANDLED;
}

static void pov_do_work(struct work_struct *w)
{
    int i = 0;

    work_counter++;
    for (; i < MAX_CHANNELS; i++) {
		struct pov_dev *dev = &channels[i];
		if (dev->state != STATE_UNUSED)
            dev_read(dev);
    }
}

static ssize_t pov_read( struct file *filp, char *buf, size_t count, loff_t *f_pos )
{
	struct pov_dev *dev = filp->private_data;
	int ret = 0;
    int timeout_in_jiffies = (count+FIFO_SIZE)*HZ/(sizeof(struct sample)*sample_rate);
    
    if (dev->state != STATE_OPENED)
        return -ENODEV;

    count *= sizeof(struct sample)/sizeof(struct sample);
    if (!count)
        return -EINVAL;

    mutex_lock(&dev->mut);
    dev->buf = buf;
    dev->buf_size = count;
    init_completion(&dev->compl);
    mutex_unlock(&dev->mut);
    
    ret = wait_for_completion_interruptible_timeout(&dev->compl, timeout_in_jiffies);
    
    mutex_lock(&dev->mut);
    if (ret < 0)
        dev_hard_error(dev, ret);
    else if (ret == 0 && dev->buf_size)
        dev_hard_error(dev, -ETIMEDOUT);

    if (dev->state != STATE_OPENED)
        return dev->error;
    else
        ret = count - dev->buf_size;
    mutex_unlock(&dev->mut);

    return ret;
}
                          
struct file_operations pov_fops = {
	.owner =    THIS_MODULE,
 	.read =     pov_read,
 	.open =     pov_open,
 	.release =  pov_release
};

static int pov_setup_cdev(struct pov_dev *dev, int index)
{
	int err = 0, devno = MKDEV(pov_major, pov_minor + index);

	cdev_init(&dev->cdev, &pov_fops);
    device_create(pov_class,  0, devno, 0, "pov%d", index);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &pov_fops;
	err = cdev_add(&dev->cdev, devno, 1);

	/* Fail gracefully if need be */
	if (err)
		printk(KERN_WARNING "pov: error %d adding pov%d\n", err, index);
	else
		printk(KERN_INFO "pov: added pov%d (%d %d)\n", index, pov_major, pov_minor + index);
    return err;
}

static int __init pov_init_module(void)
{
	int err = -ENODEV, i;
	int quotient, remainder;
	//проверка параметров
	if (irq != 5 && irq != 7 && irq != 10 && irq != 11 && irq != 12) {
		printk(KERN_WARNING "pov: unsupported irq number %d\n", irq);
		err = -EINVAL;
		goto out_param;
	}

	quotient = sample_rate/BASE_SAMPLE_RATE; 
    remainder = sample_rate%BASE_SAMPLE_RATE;
	if (quotient < 1 || quotient > MAX_SAMPLE_RATE_FACTOR || remainder)	{
		printk(KERN_WARNING "pov: invalid value for parameter 'sample_rate' %d\n", sample_rate);
		err = -EINVAL;
		goto out_param;
	}
	if (!pov_mask || (pov_mask & POV_MASK) != pov_mask)	{
		printk(KERN_WARNING "pov: invalid value for parameter 'pov_mask' %d\n", pov_mask);
		err = -EINVAL;
		goto out_param;
	}

	err = alloc_chrdev_region(&dev, pov_minor, MAX_CHANNELS,  "pov");
	pov_major = MAJOR(dev);
	if (err < 0) {
		printk(KERN_WARNING "pov: can't get major %d\n", pov_major);
		goto out_region;
	}
    pov_class = class_create(THIS_MODULE, "pov");
	if (IS_ERR(pov_class)) {
		err = PTR_ERR(pov_class);
		goto out_class;
	}


	for (i=0; i < MAX_CHANNELS; i++)
		if (pov_mask & BIT(i)) {
			channels[i].state = STATE_CLOSED;
			err = pov_setup_cdev( &channels[i], i );
			if (err) {
				printk(KERN_WARNING "pov: device pov%d setup error\n",  i);
				goto out_dev;
			}
		}
 
    work_queue = create_singlethread_workqueue("pov_wq");
 	if (!work_queue) {
		printk(KERN_WARNING "pov: create_singlethread_workqueue failed\n");
		goto out_dev;
    }

    //Запросить прерывание в совместное пользование, 
    //в качестве уникального идентификатора - pov_major
	err = request_irq( irq, pov_irq_handler, IRQF_SHARED, "pov", &pov_major );
	if (err) {
		printk(KERN_WARNING "pov: installing an interrupt handler for irq %d failed\n", irq);
		goto out_queue;
    }

    spin_lock_init(&irq_lock);
    count_duration_ns = 1000000000/sample_rate;
    analog_duration_ns = count_duration_ns/16;
    
    INIT_WORK(&work, pov_do_work);

    printk(KERN_INFO "pov: init sample_rate=%d irq=%d  pov_mask=%d\n", sample_rate, irq, pov_mask);

    return 0;
out_queue:
    destroy_workqueue(work_queue);
out_dev: 
	class_destroy(pov_class);
 	for (i=0; i < MAX_CHANNELS; i++)
		if (channels[i].state != STATE_UNUSED) { 
            device_destroy(pov_class, MKDEV(pov_major, pov_minor + i));
            cdev_del( &channels[ i ].cdev );    
        }
out_class:
	unregister_chrdev_region(dev, MAX_CHANNELS);
out_region:
out_param:
	return err;
}

static void __exit pov_exit_module(void)
{
	int i;
    for (i = 0; i < MAX_CHANNELS; i++)
        dev_release(&channels[i]);

	free_irq(irq, &pov_major);

    cancel_work_sync(&work);
    flush_workqueue(work_queue);
    destroy_workqueue(work_queue);

	for (i = 0; i < MAX_CHANNELS; i++)
		if (channels[ i ].state != STATE_UNUSED) {
            device_destroy(pov_class, MKDEV(pov_major, pov_minor + i));
			cdev_del( &channels[ i ].cdev );
        }
	class_destroy(pov_class);
    unregister_chrdev_region(dev, MAX_CHANNELS);
	printk( KERN_DEBUG "Module pov exit\n" );
}

module_init(pov_init_module);
module_exit(pov_exit_module);

MODULE_DESCRIPTION("POV driver");
MODULE_AUTHOR("Igor Bykov (igor@parma.spb.ru)");
MODULE_LICENSE("GPL");

