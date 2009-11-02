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
6. 32 байта данных фрейма преобразуются в массив из 16 short.
7. Порядок прихода сигналов 16,1,2,3,...,15 меняется в отсчете
на правильный - 1,2,...,16
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
#define FIRST_SAMPLE_FLAG 1

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
    short data[FRAME_DATA_SIZE/2];
};

struct pov_dev {
    int index;  //dev/pov<index>
	enum dev_states state;
	int error;
	enum parse_states parse_state;

	//Порты
	int fifo8_port;
	int count_port;
	int state_port;

	int cur_fifo_counter;  	    //Текущее значение счетчика  fifo
	int last_fifo_counter; 	    //Предыдущее значение счетчика fifo

	int cur_frame_num;	   	    //Номер разбираемого фрейма
	int last_frame_num;	   	    //Номер последнего разобранного 
                                //фрейма (из заголовка фрейма)
	int data_idx;	            //Текущий индекс фрейма
    int first_sample_flag;      //Флаг начала данных
    struct sample cur_sample;

    struct completion compl;
	struct cdev cdev;

    int bytes_in_fifo;
    //ПУ калибратор
    int pu_freq_period_ns;
    struct timespec begin_time;
    struct timespec end_time;
    long long bytes_from_pu;
    
    int last_ret;               //Значение функции read

    //статистика
    long long read_counter;
	long long bytes_read;
	int err_fifo_overflow;      //ошибок переполнения FIFO
	int err_skipped_frame;	    //ошибок пропуск кадров
    int err_frame_header;       //ошибок начала кадра 
    int err_msb_byte;           //ошибок в данных фрейма
    int err_frame_close_byte;   //ошибок конца кадра
    int err_time_out;           //таймаутов на приеме 
};

struct pov_dev channels[MAX_CHANNELS] = {
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
//Период между отсчетами
int count_duration_ns;
//Период между 16 квантами внутри отсчета
//4(2+2),2,2,2,2,2,2,2,2,2,2,2,2,2,2,3(2+1)
int quantum_duration_ns;

static int pov_proc_output (char *buf)
{
	int i;
	char *p = buf;
    for (i = 0; i < MAX_CHANNELS; i++) {
        struct pov_dev *dev = &channels[i];
        if (dev->state != STATE_UNUSED) {
            p += sprintf(p, "pov\t\t\t:%d\n", i);
            p += sprintf(p, "irq_counter\t\t:%lld\n", irq_counter);
            p += sprintf(p, "read_counter\t\t:%lld\n", dev->read_counter);
            p += sprintf(p, "bytes_read\t\t:%lld\n", dev->bytes_read);
            p += sprintf(p, "err_fifo_overflow\t:%d\n", dev->err_fifo_overflow);
            p += sprintf(p, "err_skipped_frame\t:%d\n", dev->err_skipped_frame);
            p += sprintf(p, "err_frame_header\t:%d\n", dev->err_frame_header);
            p += sprintf(p, "err_msb_byte\t\t:%d\n", dev->err_msb_byte);
            p += sprintf(p, "err_frame_close_byte\t:%d\n", 
                    dev->err_frame_close_byte);
            p += sprintf(p, "err_time_out\t\t:%d\n", dev->err_time_out);
            p += sprintf(p, "\n");
        }
    }

	return p - buf;
}

static int pov_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	int len = pov_proc_output(page);
	if (len <= off+count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}                

static void dev_start(struct pov_dev *dev)
{
    unsigned char state = inb(dev->state_port);
    spin_lock_irq(&irq_lock);
	//Порт фифо А четный, а В - нечетный
	state |= (dev->fifo8_port%2+1);
	outb(state, dev->state_port);
    spin_unlock_irq(&irq_lock);
}

static void dev_stop(struct pov_dev *dev)
{
    unsigned char state = inb(dev->state_port);
    spin_lock_irq(&irq_lock);
    //порт фифо А четный, а В - нечетный
	state &= ~(dev->fifo8_port%2+1);
	outb(state, dev->state_port);
    spin_unlock_irq(&irq_lock);
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

static void reset(struct pov_dev *dev)
{
	dev->cur_fifo_counter = 0;
	dev->last_fifo_counter = 0;
	dev->parse_state = PARSE_INITIAL;
	dev->last_frame_num = -1;
    dev->first_sample_flag = 1;
    dev->bytes_in_fifo = 0;
    dev->bytes_from_pu = -1;
    dev->pu_freq_period_ns = 0;
}

static void restart(struct pov_dev *dev)
{
	dev->parse_state = PARSE_INITIAL;
	dev->last_frame_num = -1;
    dev->bytes_from_pu = -1;
    dev->pu_freq_period_ns = 0;
} 

static int dev_open( struct pov_dev *dev )
{
	if (dev->state == STATE_UNUSED)
		return -ENODEV;

	if (dev->state != STATE_CLOSED)
		return -EPERM;

	dev->error = 0;

    reset(dev);

    dev->last_ret = -1;
    
    dev->read_counter = 0;
    dev->bytes_read = 0;
	dev->err_fifo_overflow = 0;
	dev->err_skipped_frame = 0;
    dev->err_frame_header = 0;
    dev->err_msb_byte = 0; 
    dev->err_frame_close_byte = 0;
    dev->err_time_out = 0;

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
		dev->state = STATE_CLOSED;
    }

	return 0;
}


static void adjust_timestamp(struct timespec *ts, int bytes_in_fifo)
{
    int pending_bytes = bytes_in_fifo%POV_FRAME_SIZE;
    int duration_ns = bytes_in_fifo/POV_FRAME_SIZE*count_duration_ns;
    int pending_quantums;

    if (pending_bytes < 5)
        pending_quantums = 0;
    else
        pending_quantums = (pending_bytes-4+1)/2;
    duration_ns += pending_quantums*quantum_duration_ns;  
    *ts = timespec_sub(*ts, ns_to_timespec(duration_ns));
}

static void shift_timestamp(struct timespec *ts)
{
    timespec_add_ns(ts, count_duration_ns);
}

static void calibrate_pu(int *pu_freq_period_ns, long long bytes_from_pu, 
    struct timespec begin_time, struct timespec end_time)
{
    if (bytes_from_pu) {
        struct timespec sub = timespec_sub(end_time, begin_time);
        s64 ns = timespec_to_ns(&sub)*(s64)(POV_FRAME_SIZE);
        do_div(ns, bytes_from_pu);
        *pu_freq_period_ns = (int)ns;
    }
}



static void dev_read(struct pov_dev *dev)
{
    if (!completion_done(&dev->compl))
        complete(&dev->compl);
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
    struct pov_dev *dev;
    irq_counter++;

    //Сбросить прерывания
 	for (; i < MAX_CHANNELS; i++) {
		dev = &channels[i];
		if (dev->state != STATE_UNUSED)
			inb(dev->state_port);
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
		if (dev->state == STATE_OPENED)
            dev_read(dev);
    }
}

static ssize_t pov_read( struct file *filp, char *buf, size_t size, loff_t *f_pos )
{
	struct pov_dev *dev = filp->private_data;
	int ret = 0;
    int bytes_to_read;
    struct timespec ts;
    int fifo_counter;
    
    if (dev->state != STATE_OPENED)
        return -ENODEV;

    size *= sizeof(struct sample)/sizeof(struct sample);
    if (!size)
        return -EINVAL;

    dev->read_counter++;

    bytes_to_read = size;

    while (1) {
        unsigned char byte;
        int timestamp_adjusted = 0;
        int skipped_frames;
        int timeout_in_jiffies; 
        int new_bytes;

        //Сохранить время и счетчик
        spin_lock_irq(&irq_lock);
        getnstimeofday(&ts); 
        fifo_counter = inw(dev->count_port);
        spin_unlock_irq(&irq_lock);
 
        if (fifo_counter & AD_OVERFLOW) {
            dev->err_fifo_overflow++;
            printk(KERN_WARNING "pov%d: FIFO overflow:%x\n", 
                    dev->index, fifo_counter);
            dev_stop(dev);
            reset(dev);
            dev_start(dev);
            continue;
        }
        
        fifo_counter &= COUNTER_MASK;
        new_bytes = dev_calc_data_count(dev, fifo_counter);
        dev->bytes_in_fifo += new_bytes;

        //Данные для калибровки ПУ
        dev->end_time = ts;
        if (dev->bytes_from_pu == -1) {
            dev->begin_time = ts;
            dev->bytes_from_pu = 0;
        }
        else {
            dev->bytes_from_pu += new_bytes;
            calibrate_pu(&dev->pu_freq_period_ns, dev->bytes_from_pu, 
                dev->begin_time, dev->end_time);
        }
        while (dev->bytes_in_fifo && size) {
            byte = inb(dev->fifo8_port);
            dev->bytes_read++;
            dev->bytes_in_fifo--; 
retry:
            switch (dev->parse_state) {
                case PARSE_INITIAL:	{
                    if ((byte & 0xC0) == 0x80) {
                        dev->parse_state = PARSE_HEAD;
                        dev->cur_frame_num = byte & FRAME_COUNTER_MASK;
                    }
                    break;
                }
                case PARSE_HEAD: {
                    if ((byte & 0xCF) == 0x80) {
                        if (dev->last_frame_num == -1)
                            dev->last_frame_num = dev->cur_frame_num - 1;
                        skipped_frames = dev->cur_frame_num - 
                            dev->last_frame_num;
                        if (skipped_frames < 0)
                            skipped_frames += MAX_FRAME_COUNTER;
                        if (--skipped_frames > 0) {
                            dev->err_skipped_frame++;
                            printk(KERN_WARNING "pov%d: Skipped frames:%d\n", 
                                dev->index, skipped_frames);
                            restart(dev);
                            goto retry;
                        }
                        if (!timestamp_adjusted) { 
                            adjust_timestamp(&ts, dev->bytes_in_fifo+2);
                            timestamp_adjusted = 1;
                        } else
                            shift_timestamp(&ts);
                        
                        if (dev->first_sample_flag) {
                            ts.tv_nsec |= FIRST_SAMPLE_FLAG;
                            dev->first_sample_flag = 0;
                        } else
                            ts.tv_nsec &= ~FIRST_SAMPLE_FLAG;

                        dev->cur_sample.timestamp = ts;
                        dev->data_idx = 0;
                        dev->parse_state = PARSE_MSB;
                    } else if (dev->last_frame_num == -1) {
                        dev->parse_state = PARSE_INITIAL;
                        goto retry;
                    } else {                          
                        dev->err_frame_header++;    
                        printk(KERN_WARNING 
                                "pov%d: Bad frame header second byte:%d\n", 
                                dev->index, byte);
                        restart(dev);
                        goto retry;
                    }
                    break;
                }
                case PARSE_MSB: {
                    if (byte & ~MSB_MASK) {
                        dev->err_msb_byte++;
                        printk(KERN_WARNING "pov%d: Bad msb byte:%d\n", 
                                dev->index, byte);
                        restart(dev);
                        goto retry;
                    }
                    dev->cur_sample.data[dev->data_idx] = byte<<8;
                    dev->parse_state = PARSE_LSB;
                    break;
                }
                case PARSE_LSB: {
                    dev->cur_sample.data[dev->data_idx] |= byte;
                    dev->data_idx++;
                    if (dev->data_idx == 16) {
                        dev->last_frame_num = dev->cur_frame_num;
                        dev->parse_state = PARSE_TAIL;
                    }
                    else
                        dev->parse_state = PARSE_MSB;
                    break;
                }
                case PARSE_TAIL: {
                    struct sample s;
                    int i;
                    if (byte != FRAME_CLOSE_BYTE) {
                        dev->err_frame_close_byte++;
                        printk(KERN_WARNING "pov%d: Bad frame close byte:%d\n", 
                                dev->index, byte);
                        restart(dev);
                        goto retry;
                    }
                    dev->parse_state = PARSE_INITIAL;
                    //Порядок прихода сигналов 16,1,2,...,15
                    //поэтому:
                    s = dev->cur_sample;
                    for (i=1; i < 16; i++)
                        dev->cur_sample.data[i-1] = s.data[i];
                    dev->cur_sample.data[15] = s.data[0];
                    copy_to_user(buf, &dev->cur_sample, sizeof(struct sample));
                    buf += sizeof(struct sample);
                    size -= sizeof(struct sample);
                    break;
                }
            }
        }
        if (!size) {
            ret = bytes_to_read - size;
            break;
        }
        
        timeout_in_jiffies = (size+FIFO_SIZE)*HZ/
            (sizeof(struct sample)*sample_rate);
        init_completion(&dev->compl);
        
        ret = wait_for_completion_interruptible_timeout(&dev->compl, 
            timeout_in_jiffies);
        
        if (ret < 0)
            goto err_exit;
        else if (ret == 0) {
            //ETIMEDOUT
            if (dev->last_ret)
                printk(KERN_WARNING "pov%d: Data read time-out\n", dev->index);
            dev->err_time_out++;
            break;
        }
    }
    goto exit;
err_exit:
    dev_hard_error(dev, ret);
exit:
    dev->last_ret = ret;
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
    device_create(pov_class, 0, devno, 0, "pov%d", index);
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
	struct proc_dir_entry *ent;
	
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
            channels[i].index = i;
			channels[i].state = STATE_CLOSED;
			err = pov_setup_cdev(&channels[i], i);
			if (err) {
				printk(KERN_WARNING "pov: device pov%d setup error\n", i);
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

	ent = create_proc_read_entry("driver/pov", 0, NULL, pov_read_proc, NULL);
	if (!ent)
		printk(KERN_WARNING "pov: unable to create /proc entry.\n");

    spin_lock_init(&irq_lock);
    count_duration_ns = 1000000000/sample_rate;
    quantum_duration_ns = count_duration_ns/16;
    
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
            cdev_del(&channels[i].cdev);    
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
		if (channels[i].state != STATE_UNUSED) {
            device_destroy(pov_class, MKDEV(pov_major, pov_minor + i));
			cdev_del(&channels[i].cdev);
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

