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
#include <linux/kfifo.h>

#include "pov.h"
/*
1. Соответствующие каналам названия устройств в драйвере: 
ПОВ1/1 - /dev/pov0, ПОВ1/2 - /dev/pov1,..., ПОВ4/2 - /dev/pov7
Существующие устройства определяются автоматически(Plug&Play).
Параметры драйвера задаются в файле 
/etc/conf.d/modules

#ifdef USE_POLLING_MECHANISM

2. Прерывания не используются. Драйвер работает по опросу.
Опрос ставится в очередь с задержкой 1/8 времени заполнения
FIFO.

#else

2. Внимание! Платы ПОВ не умеют разделять прерывания с
другими устройствами, в том числе другими платами ПОВ.
#ifdef COMMON_SHARED_IRQ
Все платы сидят на одном прерывании (один обработчик прерывания)
возможные IRQ - 5,7,10,11,12
Номер перывания - параметр irq (по умолчанию 7)  
#else
Каждая плата сидит на своем IRQ. По умолчанию: 
ПОВ1 - IRQ5
ПОВ2 - IRQ7
и т.д.
Для изменения номеров прерываний использовать параметры
brd1_irq, brd2_irq, brd3_irq, brd4_irq

#endif
#endif

2. Все платы работают на одной частоте 1800/3600 Гц
3. Данные из фреймов переписываются в промежуточный буфер 
длительностью 1 с откуда и забираются клиентом.
4. Параметры: 
маска используемых каналов - pov_mask(по умолчанию 15, ПОВ1 и ПОВ2), 
частота	дискретизации - sample_rate (по умолчанию 1800)
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

#define BUFFER_SIZE 2048    //чуть больше 1 с при 1800 Гц
#define PROC_ENTRY "driver/pov"

static DEFINE_SPINLOCK(irq_lock);
static DEFINE_MUTEX(mutex);
static void pov_do_work(struct work_struct *w);
static struct workqueue_struct *work_queue;

#define USE_POLLING_MECHANISM
#ifdef USE_POLLING_MECHANISM
DECLARE_DELAYED_WORK(delayed_work, pov_do_work);
static atomic_t started_refcount;   // = каналов в работе
static long poll_delay_in_js;
#else
static struct work_struct work;
#ifdef COMMON_SHARED_IRQ
#define DEFAULT_IRQ 7	//или 12 (PS/2 мышь)
static int irq = DEFAULT_IRQ;
module_param(irq, int, S_IRUGO);
#else
static int irqmask = 0;
static int brd1_irq = 5;
static int brd2_irq = 7;
static int brd3_irq = 10;
static int brd4_irq = 11;
module_param(brd1_irq, int, S_IRUGO);
module_param(brd2_irq, int, S_IRUGO);
module_param(brd3_irq, int, S_IRUGO);
module_param(brd4_irq, int, S_IRUGO);
#endif
#endif

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

static int pov_mask = POV_MASK;

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
long test_value;

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
    short data[FRAME_DATA_SIZE/sizeof(short)];
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

    struct timespec cur_ts;
    int timestamp_adjusted;
	int cur_fifo_counter;  	    //Текущее значение счетчика  fifo
	int last_fifo_counter; 	    //Предыдущее значение счетчика fifo

	int cur_frame_num;	   	    //Номер разбираемого фрейма
	int last_frame_num;	   	    //Номер последнего разобранного 
                                //фрейма (из заголовка фрейма)
	int data_idx;	            //Текущий индекс фрейма
    int first_sample_flag;      //Флаг начала данных
    struct sample cur_sample;
    DECLARE_KFIFO_PTR(buffer, struct sample);

    struct completion compl;
	struct cdev cdev;

    int bytes_in_fifo;
    int peak_bytes_in_fifo;     //Для статистики
    
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
	int err_buffer_overflow;    //ошибок переполнения буфера
    int buffer_fill_percentage; //заполненность буфера (%)
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
#ifdef USE_POLLING_MECHANISM
            p += sprintf(p, "work_counter\t\t:%lld\n", work_counter);
#else
            p += sprintf(p, "irq_counter\t\t:%lld\n", irq_counter);
#endif
            p += sprintf(p, "peak_bytes_in_fifo\t:%d\n", dev->peak_bytes_in_fifo);
            p += sprintf(p, "read_counter\t\t:%lld\n", dev->read_counter);
            p += sprintf(p, "bytes_read\t\t:%lld\n", dev->bytes_read);
            p += sprintf(p, "err_fifo_overflow\t:%d\n", dev->err_fifo_overflow);
            p += sprintf(p, "err_skipped_frame\t:%d\n", dev->err_skipped_frame);
            p += sprintf(p, "err_frame_header\t:%d\n", dev->err_frame_header);
            p += sprintf(p, "err_msb_byte\t\t:%d\n", dev->err_msb_byte);
            p += sprintf(p, "err_frame_close_byte\t:%d\n", 
                    dev->err_frame_close_byte);
            p += sprintf(p, "err_time_out\t\t:%d\n", dev->err_time_out);
            p += sprintf(p, "err_buffer_overflow\t:%d\n", dev->err_buffer_overflow);
            p += sprintf(p, "buffer_fill_percentage\t:%d\n", 
                    (kfifo_len(&dev->buffer)*100)/BUFFER_SIZE);
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
	//Порт фифо А четный, а В - нечетный
	state |= (dev->fifo8_port%2+1);
	outb(state, dev->state_port);

#ifdef USE_POLLING_MECHANISM
    if (atomic_add_return(1, &started_refcount) == 1)
        queue_delayed_work(work_queue, &delayed_work, poll_delay_in_js);
#endif
}

static void dev_stop(struct pov_dev *dev)
{
    unsigned char state = inb(dev->state_port);
    //порт фифо А четный, а В - нечетный
	state &= ~(dev->fifo8_port%2+1);
	outb(state, dev->state_port);

#ifdef USE_POLLING_MECHANISM
    atomic_sub(1, &started_refcount);
#endif
}

static void dev_hard_error(struct pov_dev *dev, int err)
{
	if (dev->state == STATE_OPENED) {
		dev->state = STATE_ERROR;
		dev->error = err;
		dev_stop(dev);
	}
}

static int  dev_calc_data_count(struct pov_dev *dev)
{
	int res;
	res = dev->cur_fifo_counter - dev->last_fifo_counter;
	if (res < 0)
		res += FIFO_SIZE;
    dev->last_fifo_counter = dev->cur_fifo_counter;
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
    dev->first_sample_flag = 1;
} 

static int dev_open( struct pov_dev *dev )
{
	if (dev->state == STATE_UNUSED)
		return -ENODEV;

	if (dev->state != STATE_CLOSED)
		return -EPERM;

    mutex_lock(&mutex);

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
	dev->err_buffer_overflow = 0;
    dev->buffer_fill_percentage = 0;

	dev->state = STATE_OPENED;
	dev_start(dev);

    mutex_unlock(&mutex);
	
    return 0;
}

static int dev_release(struct pov_dev *dev)
{
	if (dev->state == STATE_UNUSED)
		return -ENODEV;

	if (dev->state != STATE_CLOSED)	{
	    
        mutex_lock(&mutex);

        if (dev->state == STATE_OPENED)
            dev_stop(dev);
		dev->state = STATE_CLOSED;

        mutex_unlock(&mutex);
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

static void dev_pre_read(struct pov_dev *dev, struct timespec *ts)
{
    int new_bytes;

    if (dev->cur_fifo_counter & AD_OVERFLOW) {
        dev->err_fifo_overflow++;
        printk(KERN_WARNING "pov%d: FIFO overflow:%x\n", 
                dev->index, dev->cur_fifo_counter);
        dev_stop(dev);
        reset(dev);
        dev_start(dev);
        return;
    }

    dev->timestamp_adjusted = 0;
    dev->cur_ts = *ts;
    dev->cur_fifo_counter &= COUNTER_MASK;
    new_bytes = dev_calc_data_count(dev);
    dev->bytes_in_fifo += new_bytes;
    dev->peak_bytes_in_fifo = dev->bytes_in_fifo;

    //Данные для калибровки ПУ
    dev->end_time = *ts;
    if (dev->bytes_from_pu == -1) {
        dev->begin_time = *ts;
        dev->bytes_from_pu = 0;
    }
    else {
        dev->bytes_from_pu += new_bytes;
        calibrate_pu(&dev->pu_freq_period_ns, dev->bytes_from_pu, 
            dev->begin_time, dev->end_time);
    }
}

static void dev_parse_byte(struct pov_dev *dev, u8 byte)
{
    
    while (1) {
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
                    int skipped_frames;
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
                        continue;
                    }
                    if (!dev->timestamp_adjusted) { 
                        adjust_timestamp(&dev->cur_ts, dev->bytes_in_fifo+2);
                        dev->timestamp_adjusted = 1;
                    } else
                        shift_timestamp(&dev->cur_ts);
                    
                    if (dev->first_sample_flag) {
                        dev->cur_ts.tv_nsec |= FIRST_SAMPLE_FLAG;
                        dev->first_sample_flag = 0;
                    } else
                        dev->cur_ts.tv_nsec &= ~FIRST_SAMPLE_FLAG;

                    dev->cur_sample.timestamp = dev->cur_ts;
                    dev->data_idx = 0;
                    dev->parse_state = PARSE_MSB;
                } else if (dev->last_frame_num == -1) {
                    dev->parse_state = PARSE_INITIAL;
                    continue;
                } else {                          
                    dev->err_frame_header++;    
                    printk(KERN_WARNING 
                            "pov%d: Bad frame header second byte:%d\n", 
                            dev->index, byte);
                    restart(dev);
                    continue;
                }
                break;
            }
            case PARSE_MSB: {
                if (byte & ~MSB_MASK) {
                    dev->err_msb_byte++;
                    printk(KERN_WARNING "pov%d: Bad msb byte:%d\n", 
                            dev->index, byte);
                    restart(dev);
                    continue;
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
                short v0;
                if (byte != FRAME_CLOSE_BYTE) {
                    dev->err_frame_close_byte++;
                    printk(KERN_WARNING "pov%d: Bad frame close byte:%d\n", 
                            dev->index, byte);
                    restart(dev);
                    continue;
                }
                dev->parse_state = PARSE_INITIAL;
                //Порядок прихода сигналов 16,1,2,...,15
                //поэтому:
                v0 = dev->cur_sample.data[0];
                memcpy(dev->cur_sample.data, &dev->cur_sample.data[1], 
                        15*sizeof(short));
                dev->cur_sample.data[15] = v0;

                if (kfifo_put(&dev->buffer, &dev->cur_sample) == 0) {
                    dev->err_buffer_overflow++;
                    dev->first_sample_flag = 1;
                }
                break;
            }
        }
        break;
    }
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

#ifndef USE_POLLING_MECHANISM
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
#endif

static void pov_do_work(struct work_struct *w)
{
    struct timespec ts;
    unsigned long flags;
    int i;

#ifdef USE_POLLING_MECHANISM
    if (atomic_read(&started_refcount))
        queue_delayed_work(work_queue, &delayed_work, poll_delay_in_js);
#endif

    work_counter++;
    
    mutex_lock(&mutex);
    
    spin_lock_irqsave(&irq_lock, flags);
    //FIXME При инициализации драйвера сделать калибровку по времени
    //функции inw() для корректировки или
    //запрос времени перед каждым inw()
    getnstimeofday(&ts); 
    for (i=0; i < MAX_CHANNELS; i++) {
		struct pov_dev *dev = &channels[i];
		if (dev->state == STATE_OPENED)
            dev->cur_fifo_counter = inw(dev->count_port);
    }
    spin_unlock_irqrestore(&irq_lock, flags);
    
    for (i=0; i < MAX_CHANNELS; i++) {
		struct pov_dev *dev = &channels[i];
		if (dev->state == STATE_OPENED)
            dev_pre_read(dev, &ts);
    }
    
    for (i=0; i < MAX_CHANNELS; i+=2) {
		struct pov_dev *devA = &channels[i];
		struct pov_dev *devB = &channels[i+1];
		if (devA->state == STATE_OPENED && devB->state == STATE_OPENED) {
            int shorts = min(devA->bytes_in_fifo, devB->bytes_in_fifo);
            while (shorts) {
                u16 data = inw(devA->fifo8_port+2);
                
                devA->bytes_read++;
                devA->bytes_in_fifo--;
                dev_parse_byte(devA, data);
                
                devB->bytes_read++;
                devB->bytes_in_fifo--;
                dev_parse_byte(devB, data>>8);
            
                shorts--;
            }
        }
    }

    for (i=0; i < MAX_CHANNELS; i++) {
		struct pov_dev *dev = &channels[i];
		if (dev->state == STATE_OPENED) {
            while (dev->bytes_in_fifo) {
                dev->bytes_in_fifo--;
                dev->bytes_read++;
                dev_parse_byte(dev, inb(dev->fifo8_port));
            }
            if (!completion_done(&dev->compl))
                complete(&dev->compl);
        }
    }
    mutex_unlock(&mutex);

}

static ssize_t pov_read( struct file *filp, char *buf, size_t size, loff_t *f_pos )
{
	struct pov_dev *dev = filp->private_data;
	int ret = 0;
    int bytes_to_read;
    int copied;
    int timeout_in_jiffies;
    
    if (dev->state != STATE_OPENED)
        return -ENODEV;

    size /= sizeof(struct sample);
    if (!size)
        return -EINVAL;
    size *= sizeof(struct sample);

    dev->read_counter++;

    bytes_to_read = size;

    while (1) {
        ret = kfifo_to_user(&dev->buffer, buf, size, &copied);
        if (ret) {
            printk(KERN_WARNING "pov%d: Copy to user failed\n", dev->index);
            goto err_exit;
        }
        buf += copied;
        size -= copied;
        if (!size) {
            ret = bytes_to_read;
            break;
        }
        
        timeout_in_jiffies = (size+FIFO_SIZE)*HZ/
            (sizeof(struct sample)*sample_rate);
        init_completion(&dev->compl);
        
        ret = wait_for_completion_interruptible_timeout(&dev->compl, 
            timeout_in_jiffies);
        
        if (ret < 0) {
            // Signal
            ret = bytes_to_read - size;
            break;
        }
        else if (ret == 0) {
            // Timeout
            if (dev->last_ret != -ENODATA)
                printk(KERN_WARNING "pov%d: Data read time-out\n", dev->index);
                dev->err_time_out++;
            ret = -ENODATA;
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

#ifndef USE_POLLING_MECHANISM    
#ifdef COMMON_SHARED_IRQ
    //проверка параметров
	if (irq != 5 && irq != 7 && irq != 10 && irq != 11 && irq != 12) {
		printk(KERN_WARNING "pov: unsupported irq number %d\n", irq);
		err = -EINVAL;
		goto out_param;
	}
#else
    //Проверить, что номера прерываний у имеющихся ПОВ различны
    int installed_irq = 0;
    int irqs[MAX_BOARDS]; 
    irqs[0] = brd1_irq;
    irqs[1] = brd2_irq;
    irqs[2] = brd3_irq;
    irqs[3] = brd4_irq;
    for (i=0; i < MAX_CHANNELS; i+=2)
        //PnP 13-й бит счетчика всегда 0
        if (~inw(channels[i].count_port) & BIT(13)) {
            int irq = irqs[i/2];
            if (irq != 5 && irq != 7 && irq != 10 && irq != 11 && irq != 12) {
                printk(KERN_WARNING "pov: unsupported irq number %d\n", irq);
                err = -EINVAL;
                goto out_param;
            }
            if (irqmask & BIT(irqs[i/2])) {
                printk(KERN_WARNING "pov: IRQ%d already in use\n", irqs[i/2]);
                err = -EINVAL;
                goto out_param;
            }
            irqmask |= BIT(irqs[i/2]);
		}
#endif
#endif

	quotient = sample_rate/BASE_SAMPLE_RATE; 
    remainder = sample_rate%BASE_SAMPLE_RATE;
	if (quotient < 1 || quotient > MAX_SAMPLE_RATE_FACTOR || remainder)	{
		printk(KERN_WARNING "pov: invalid value for parameter 'sample_rate' %d\n", sample_rate);
		err = -EINVAL;
		goto out_param;
	}

#ifdef USE_POLLING_MECHANISM    
    // Период опроса - 1/4 времени заполнения ФИФО платы
    poll_delay_in_js = HZ*FIFO_SIZE/POV_FRAME_SIZE/sample_rate/4;
#endif

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
        //PnP 13-й бит счетчика всегда 0
        if (~inw(channels[i].count_port) & BIT(13)) {
            channels[i].index = i;
			channels[i].state = STATE_CLOSED;
			err = pov_setup_cdev(&channels[i], i);
			if (err) {
				printk(KERN_WARNING "pov: device pov%d setup error\n", i);
				goto out_dev;
			}
            err = kfifo_alloc(&channels[i].buffer, BUFFER_SIZE, GFP_KERNEL);
            if (err) {
				printk(KERN_WARNING "pov: error to allocate buffer\n");
				goto out_dev;
            }
		}
 
    work_queue = create_singlethread_workqueue("pov_wq");
    // +WQ_HIGHPRI
    //work_queue = alloc_workqueue("pov_wq", WQ_HIGHPRI | WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
 	if (!work_queue) {
		printk(KERN_WARNING "pov: create_singlethread_workqueue failed\n");
		goto out_dev;
    }

#ifndef USE_POLLING_MECHANISM
#ifdef COMMON_SHARED_IRQ
    //Запросить прерывание в совместное пользование, 
    //в качестве уникального идентификатора - pov_major
	err = request_irq(irq, pov_irq_handler, IRQF_SHARED, "pov", &pov_major);
	if (err) {
		printk(KERN_WARNING "pov: installing an interrupt handler for irq %d failed\n", irq);
		goto out_queue;
    }
#else
    for (i = 5; i <= 12; i++) { // IRQ5 - IRQ12
        if (irqmask & BIT(i)) {
            err = request_irq(i, pov_irq_handler, IRQF_SHARED, "pov", &pov_major);
            if (err) {
                printk(KERN_WARNING "pov: installing an interrupt handler for irq %d failed\n", i);
                irqmask = installed_irq;
                goto out_irq;
            }
            installed_irq |= BIT(i);
        }
    }
#endif
#endif

	ent = create_proc_read_entry(PROC_ENTRY, 0, NULL, pov_read_proc, NULL);
	if (!ent)
		printk(KERN_WARNING "pov: unable to create /proc entry.\n");

    spin_lock_init(&irq_lock);
    mutex_init(&mutex);
    count_duration_ns = 1000000000/sample_rate;
    quantum_duration_ns = count_duration_ns/16;
    

#ifdef USE_POLLING_MECHANISM
    printk(KERN_INFO "pov: sample_rate=%d\n", sample_rate);
    return 0;
#else
    INIT_WORK(&work, pov_do_work);
#ifdef COMMON_SHARED_IRQ
    printk(KERN_INFO "pov: init sample_rate=%d irq=%d  pov_mask=%d\n", sample_rate, irq, pov_mask);
#else
    for (i = 0; i < MAX_BOARDS; i++)
        if (irqmask | BIT(irqs[i]))
            printk(KERN_INFO "pov: board=%d init sample_rate=%d irq=%d\n", i, sample_rate, irqs[i]);
#endif
    printk(KERN_INFO "pov: init\n");
    return 0;
#ifdef COMMON_SHARED_IRQ
out_queue:
#else
out_irq:
    for (i = 5; i <= 12; i++)   // IRQ5 - IRQ12
        if (irqmask & BIT(i))
            free_irq(i, NULL);
#endif
#endif  //USE_POLLING_MECHANISM
    destroy_workqueue(work_queue);
out_dev: 
	class_destroy(pov_class);
 	for (i=0; i < MAX_CHANNELS; i++)
		if (channels[i].state != STATE_UNUSED) { 
            kfifo_free(&channels[i].buffer);    
            device_destroy(pov_class, MKDEV(pov_major, pov_minor + i));
            cdev_del(&channels[i].cdev);
        }
out_class:
	unregister_chrdev_region(dev, MAX_CHANNELS);
out_region:
out_param:
    printk(KERN_INFO "pov: init err=%d\n", err);
	return err;
}

static void __exit pov_exit_module(void)
{
	int i;
    printk(KERN_INFO "pov: exit\n");

    remove_proc_entry(PROC_ENTRY, NULL);

    for (i = 0; i < MAX_CHANNELS; i++)
        dev_release(&channels[i]);

#ifdef USE_POLLING_MECHANISM
    cancel_delayed_work_sync(&delayed_work);
#else
#ifdef COMMON_SHARED_IRQ
	free_irq(irq, &pov_major);
#else
    for (i = 5; i <= 12; i++)   // IRQ5 - IRQ12
        if (irqmask & BIT(i))
           free_irq(i, NULL); 
#endif
    cancel_work_sync(&work);
#endif

    flush_workqueue(work_queue);
    destroy_workqueue(work_queue);

	for (i = 0; i < MAX_CHANNELS; i++)
		if (channels[i].state != STATE_UNUSED) {
            kfifo_free(&channels[i].buffer);    
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

