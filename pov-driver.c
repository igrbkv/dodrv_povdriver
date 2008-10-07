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
3. Данные из фреймов переписываются в буфер пользователя как есть, при потере кадра указатель в буфере пользователя
сдвигается на число пропущенных байт данных фрейма
4. Параметры: маска используемых каналов - , irq , pov_mask(по умолчанию включены ПОВ1 и ПОВ2), частота
	дискретизации - sample_rate (по умолчанию 1800)
5. Соответствующие каналам названия устройств в драйвере: ПОВ1/1 - pov0, ПОВ1/2 - pov1, ПОВ2/1 - pov3, 		ПОВ2/2 - pov4 и т.д.
*/

#define MAX_BOARDS	4
#define MAX_CHANNELS (MAX_BOARDS*2)

#define FIFO_SIZE 0x2000
#define BUF_SIZE FIFO_SIZE*2


#define FRAME_COUNTER_MASK			0x3f
#define MAX_FRAME_COUNTER			0x40
#define FRAME_SIZE							35
#define FRAME_DATA_SIZE				32
#define MSB_MASK							0x7f
#define FRAME_CLOSE_BYTE				0x73
#define READ_COUNTER_TIMEOUT 		10
//FIXME Таймаут должен соответствовать размеру буферов
#define READ_TIMEOUT						HZ
#define  AD_OVERFLOW       0x8000
#define COUNTER_MASK   0x1FFF


DECLARE_WAIT_QUEUE_HEAD(pov_queue);

static void pov_do_tasklet( unsigned long unused );
DECLARE_TASKLET(pov_tasklet, pov_do_tasklet, 0);



#define DEFAULT_IRQ 7	//или 12 (PS/2 мышь)
int irq = DEFAULT_IRQ;
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



int pov_mask = (POV11|POV12|POV21|POV22);
module_param(pov_mask, int, S_IRUGO);

//частота переменного тока
#define AC_FREQUENCY	50
#define BASE_SAMPLE_RATE 1800
#define MAX_SAMPLE_RATE_FACTOR 2
//FIXME Задаваемая частота дискретизации может не совпадать с ПУ-шной
//(см. 2-ой байт заголовка пакета)
int sample_rate = BASE_SAMPLE_RATE;
module_param(sample_rate, int, S_IRUGO);

int exit;							//Exit module flag
long long irq_counter;
long long tasklet_counter;

enum dev_state{	//Переходы:
	state_unused,	//=init_module=>state_closed
 	state_closed,		//=pov_open=>state_opened
 	state_error,		//=pov_release=>state_closed
 	state_opened		//=hard_error=>state_error  или =pov_release=>state_closed
};
enum dev_error{ err_overflow, err_start_frame, err_timeout };
enum parse_state{ parse_initial, parse_head, parse_msb, parse_lsb, parse_tail };

struct pov_dev
{
	enum dev_state state;
	enum dev_error error;
	enum parse_state pstate;

	//Порты
	int fifo8_port;
	int fifo16_port;
	int count_port;
	int state_port;

	int count_in_period;      				// Число отсчетов в периоде для канала

	int last_fifo_counter;					//Предыдущее значение счетчика  fifo (-1 изначально)
	int last_frame;							//Номер последнего разобранного фрейма (из заголовка фрейма)
	int cur_frame;							//Номер разбираемого фрейма
	int idx;										//Текущий lsb-байт фрейма
	int last_byte;								//последний прочитанный байт
	int skipped_bytes;						//Пропущено байт из-за сбойных фреймов ( в буфер пользователя будет записан пустой кусок такой длины )

	long long	bytes_read;
	//struct semaphore sem;
	struct cdev cdev;

	//FIXME сделать динмический буфер, размер по умолчанию или параметр модуля
	char buffer[ BUF_SIZE ];
	char *buf_head;
	char *buf_tail;


	//статистика
	int buffer_overflows;				//число переполнений буфера
	int fifo_overflows;					//число переполнений FIFO
	int skipped_frames;				//пропусков кадров
	int err_start_frame_count;
	int err_sckiped_bytes;
};

struct pov_dev channels[ MAX_CHANNELS ] = {
	//ПОВ1
	[0].fifo8_port = 0x150,
	[0].fifo16_port = 0x152,
 	[0].count_port = 0x154,
  	[0].state_port = 0x158,

	[1].fifo8_port = 0x151,
 	[1].fifo16_port = 0x152,
  	[1].count_port = 0x156,
  	[1].state_port = 0x158,

   //ПОВ2
   	[2].fifo8_port = 0x15a,
	[2].fifo16_port = 0x15c,
  	[2].count_port = 0x15e,
  	[2].state_port = 0x162,

	[3].fifo8_port = 0x15b,
 	[3].fifo16_port = 0x15c,
	[3].count_port = 0x160,
	[3].state_port = 0x162,

 	//ПОВ3
  	[4].fifo8_port = 0x180,
 	[4].fifo16_port = 0x182,
	[4].count_port = 0x184,
	[4].state_port = 0x188,

  	[5].fifo8_port = 0x181,
 	[5].fifo16_port = 0x182,
	[4].count_port = 0x186,
	[4].state_port = 0x188,

  	//ПОВ4
    [6].fifo8_port = 0x18a,
 	[6].fifo16_port = 0x18c,
  	[6].count_port = 0x18e,
  	[6].state_port = 0x192,

	[7].fifo8_port = 0x18b,
 	[7].fifo16_port = 0x18c,
  	[7].count_port = 0x190,
  	[7].state_port = 0x192,
};

int pov_major, pov_minor;
dev_t dev;

static void dev_parse_data( struct pov_dev *dev, char *buf, int *size )
{
	char byte;
	char  last_byte = dev->last_byte;
	int idx = dev->idx;
	enum parse_state state = dev->pstate;
	int skipped_bytes = dev->skipped_bytes;
	int skipped_frames;


	while( dev->buf_tail != dev->buf_head && *size )
	{
		byte = *dev->buf_tail;
		retry:
		switch( state )
		{
			case parse_initial:
			{
				if( (byte & 0xC0) == 0x80 )
				{
					state = parse_head;
					dev->cur_frame = byte & FRAME_COUNTER_MASK;
					if( dev->last_frame == -1 )
						dev->last_frame = dev->cur_frame;
				}
				else if( dev->last_frame >= 0 )
					dev->err_sckiped_bytes++;
				break;
			}
			case parse_head:
			{
				if(  (byte & 0xCF) == 0x80 )
				{
					//Вычисляем число пропущенных байт и пропускаем столько же байт  в буфере пользователя
					//FIXME Заполнить пропуск данными прошлого отсчета
					skipped_frames = dev->cur_frame - dev->last_frame;
					if( skipped_frames < 0 )
						skipped_frames += MAX_FRAME_COUNTER;
					if( --skipped_frames > 0 )
						skipped_bytes += skipped_frames*FRAME_DATA_SIZE;
					//Если размер буфера пользователя не кратен FRAME_DATA_SIZE
					//то могут быть остатки из пропущенных байт от предыдущего чтения
					if( skipped_bytes )
					{
						*size -= skipped_bytes;
						if( *size <= 0  )
						{
							skipped_bytes = -*size;
							*size = 0;
							continue;		//Выходим
						}
						else
							skipped_bytes = 0;

					}
					idx = 0;
					state = parse_msb;
				}
				break;
			}
			case parse_msb:
			{
				if( byte & ~MSB_MASK )
				{
					state = parse_initial;
					goto retry;
				}
				put_user( byte, buf++ );
				dev->bytes_read++;
				state = parse_lsb;
				break;
			}
			case parse_lsb:
			{
				put_user( byte, buf++ );
				dev->bytes_read++;
				if( ++idx == 16 )
				{
					dev->last_frame = dev->cur_frame;
					state = parse_tail;
				}
				else
					state = parse_msb;

				break;
			}
			case parse_tail:
			{
				if( byte !=  FRAME_CLOSE_BYTE )
				{
					state = parse_initial;
					goto retry;
				}
				break;
			}
		}
		(*size)--;
		last_byte = byte;
		dev->buf_tail++;
		if( dev->buf_tail - dev->buffer >= BUF_SIZE )
			dev->buf_tail = dev->buffer;
	}
	dev->last_byte = byte;
	dev->idx = idx;
	dev->pstate = state;
	dev->skipped_bytes = skipped_bytes;
}

static unsigned short dev_read_counter( struct pov_dev *dev )
{
	unsigned short count = 0;
	int i;
	for( i=0; i<READ_COUNTER_TIMEOUT; i++ )
	{
		count = inw(dev->count_port);
		if( count == inw(dev->count_port) )
			break;
	}
	return count;
}

static void dev_start( struct pov_dev *dev )
{
	unsigned char state = inb( dev->state_port );
	//порт фифо А четный, а В - нечетный
	state |=  (dev->fifo8_port%2+1);
	outb( state, dev->state_port );
}

static void dev_stop( struct pov_dev *dev )
{
	unsigned char state = inb( dev->state_port );
	//порт фифо А четный, а В - нечетный
	state &= ~(dev->fifo8_port%2+1);
	outb( state, dev->state_port );
}

static void dev_hard_error( struct pov_dev *dev, enum dev_error err )
{
	if( dev->state == state_opened )
	{
		dev->state = state_error;
		dev->error = err;
		dev_stop(  dev );
	}
}

static int  dev_calc_data_count( struct pov_dev *dev, int count  )
{
	int res;
	if( dev->last_fifo_counter == -1 )
		dev->last_fifo_counter = count;
	res = count - dev->last_fifo_counter;
	if(  res < 0 )
		res += FIFO_SIZE;
	return res;
}

static int dev_open( struct pov_dev *dev )
{
	if( dev->state == state_unused )
		return -ENODEV;

	if( dev->state != state_closed )
		return -EPERM;

	dev->bytes_read = 0;
	dev->buf_head = dev->buf_tail = dev->buffer;
	dev->last_fifo_counter = -1;
	dev->pstate = parse_initial;
	dev->last_frame = -1;
	dev->skipped_bytes = 0;

	dev->buffer_overflows = 0;
	dev->fifo_overflows = 0;
	dev->skipped_frames = 0;
	dev->err_start_frame_count = 0;
	dev->err_sckiped_bytes = 0;


	dev->state = state_opened;
	dev_start( dev );
	return 0;          /* success */
}

static int dev_release( struct pov_dev *dev )
{
	if( dev->state == state_unused )
		return -ENODEV;

	if( dev->state == state_opened )
	{
		tasklet_disable( &pov_tasklet );
		dev_stop( dev );
		dev->state = state_closed;
		tasklet_enable( &pov_tasklet );
	}
	else
		dev->state = state_closed;

	return 0;          /* success */
}

static int dev_get_time_by_offset( struct pov_dev *dev,  long long offset, struct timespec *ts_ptr )
{
	struct timespec ts;
	long long cur_offset;
	int counter;
	int data_count;
	int buf_count;
	int bytes_buffered;
	long diff;
	unsigned long flags;
	long sec, rem;
	s64 ns;

	local_irq_save( flags );
	getnstimeofday( &ts );
	counter = dev_read_counter( dev );
	buf_count = dev->buf_head - dev->buf_tail;
	cur_offset = dev->bytes_read;
	local_irq_restore( flags );

	if( counter & AD_OVERFLOW )
		return -EIO;

	counter &= COUNTER_MASK;
	data_count = dev_calc_data_count( dev, counter );
	if( buf_count < 0 )
		buf_count += BUF_SIZE;

	bytes_buffered = data_count+ buf_count;
	cur_offset += bytes_buffered/FRAME_SIZE*FRAME_DATA_SIZE +
			(bytes_buffered%FRAME_DATA_SIZE > 2?  (bytes_buffered%FRAME_DATA_SIZE -2): 0 );

	diff = cur_offset - offset;
	if( diff < 0 )
		return -EINVAL;

	//приведение к функции timespec_add_ns
	sec = diff/sample_rate;
	rem = diff%sample_rate;
	if( rem )
	{
		sec++;
		ns = rem*NSEC_PER_SEC;
		do_div( ns, sample_rate );
		ns = NSEC_PER_SEC - ns;
		timespec_add_ns( &ts, ns);
	}
	ts.tv_sec -= sec;

	*ts_ptr = ts;

	return 0;
}

static int dev_get_offset_by_time( struct pov_dev *dev,  long long *offset, struct timespec time )
{
	struct timespec ts;
	long long cur_offset;
	int counter;
	int data_count;
	int buf_count;
	int bytes_buffered;
	s64 diff;

	local_irq_disable();
	getnstimeofday( &ts );
	counter = dev_read_counter( dev );
	buf_count = dev->buf_head - dev->buf_tail;
	cur_offset = dev->bytes_read;
	local_irq_enable();

	if( counter & AD_OVERFLOW )
		return -EIO;

	counter &= COUNTER_MASK;
	data_count = dev_calc_data_count( dev, counter );
	if( buf_count < 0 )
		buf_count += BUF_SIZE;

	bytes_buffered = data_count+ buf_count;
	cur_offset += bytes_buffered/FRAME_SIZE*FRAME_DATA_SIZE +
			(bytes_buffered%FRAME_DATA_SIZE > 2?  (bytes_buffered%FRAME_DATA_SIZE -2): 0);

	if( timespec_compare( &ts, &time ) < 0 )
		return -EINVAL;

	diff = (timespec_to_ns( &ts ) - timespec_to_ns(  &time ))*sample_rate;
	do_div( diff, NSEC_PER_SEC );

	*offset = cur_offset - diff;

	return 0;

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

irqreturn_t pov_irq_handler( int irq, void *dev_id )
{
	irq_counter++;

	tasklet_schedule( &pov_tasklet );

	return IRQ_HANDLED;
}

static void pov_do_tasklet( unsigned long unused )
{
	int i = 0, j;
	int counter, data_count, diff, buf_free;
	//FIXME Читать из фифо не в промежуточный буфер а прямо в пользовательский буфер

	//Проверка счетчиков и чтение
	for(; i < MAX_CHANNELS; i++ )
	{
		struct pov_dev * dev = &channels[ i ];
		if( dev->state == state_opened )
		{
			inb( dev->state_port ); 			//Сбросить прерывания

			counter = dev_read_counter( dev );
			if( counter & AD_OVERFLOW )
			{
				dev_hard_error( dev, err_overflow );
				continue;
			}
			counter &= COUNTER_MASK;
			data_count = dev_calc_data_count( dev, counter );

			//Сколько свободного места в буфере
			buf_free = dev->buf_tail - dev->buf_head - 1;
			if( buf_free < 0 )
				buf_free += BUF_SIZE;
			//Если места в буфере меньше чем данных в фифо,
			//уменьшаем число считываемых байт из фифо и
			// сохраненное значение счетчика фифо на их разницу
			diff = data_count - buf_free;
			if( diff > 0 )
			{
				data_count =buf_free;
				counter -= diff;
				if( counter < 0 )
					counter += FIFO_SIZE;
			}
			dev->last_fifo_counter = counter;

			//Читаем из фифо в буфер
			//FIXME Проверить, возможно ли строковое чтение из порта функцией insb()
			for(  j = 0; j < data_count; j++ )
			{
				*dev->buf_head++ = inb( dev->fifo8_port );
				if( (dev->buf_head - dev->buffer) >= BUF_SIZE )
					dev->buf_head = dev->buffer;
			}
		}
	}

	wake_up_interruptible(&pov_queue); /* awake any reading process */
}

static ssize_t pov_read( struct file *filp, char *buf, size_t count, loff_t *f_pos )
{
	struct pov_dev *dev = filp->private_data;
	struct timespec ts = { 0, 0 };
	int ret = 0;
	size_t to_read = count;

	while( to_read )
	{
		if ( wait_event_interruptible_timeout( pov_queue, (dev->buf_head != dev->buf_tail) || exit, READ_TIMEOUT))
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */

		if( exit )
			return -ENODEV;

		//Таймаут?
		if( dev->buf_head == dev->buf_tail  )
		{
			//FIXME запрет обработки прерывания может вызвать переполнение fifo в работоспособных каналах
			//Синхронизация с do_pov_tasklet
			tasklet_disable( &pov_tasklet );
			if( dev->state == state_opened )
				dev_hard_error( dev, err_timeout );
			tasklet_enable( &pov_tasklet );
			return -EIO;
		}

		if(  !ts.tv_sec )
		{
			if( (ret = dev_get_time_by_offset( dev, dev->bytes_read, &ts )) < 0 )
				return ret;
			put_user( &ts, buf );
			buf += sizeof( ts );
			to_read -= sizeof( ts );
		}
		dev_parse_data( dev,  buf, &to_read  );

	}

	return count - to_read;
}


static int pov_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	long long offset;
	int ret = -EINVAL;
	struct pov_dev *dev; /* device information */
	struct timespec ts;

	dev = container_of(inode->i_cdev, struct pov_dev, cdev);
	filp->private_data = dev; /* for other methods */
	switch( cmd )
	{
		case POV_IOCXTIMEBYOFFSET:
		{
			ret = copy_from_user( &offset, (void *)arg, sizeof( long long ) );
			if( ret == 0 )
				ret = dev_get_time_by_offset( dev,  offset, &ts );
			if( ret == 0)
				ret = copy_to_user( (void *)arg, &ts, sizeof( ts ) );
			break;
		}
		case POV_IOCXOFFSETBYTIME:
		{
			ret = copy_from_user( &ts, (void *)arg, sizeof( ts ) );
			if( ret == 0 )
				ret = dev_get_offset_by_time( dev,  &offset, ts );
			if( ret == 0)
				ret = copy_to_user( (void *)arg, &offset, sizeof( long long ) );
			break;
		}

	}
	return ret;
}

struct file_operations pov_fops = {
	.owner =    THIS_MODULE,
 	//.llseek =   pov_llseek,
 	.read =     pov_read,
	.ioctl =    pov_ioctl,
 	.open =     pov_open,
 	.release =  pov_release
};

static int pov_setup_cdev(struct pov_dev *dev, int index)
{
	int err = 0, devno = MKDEV(pov_major, pov_minor + index);

	cdev_init(&dev->cdev, &pov_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &pov_fops;
	err = cdev_add (&dev->cdev, devno, 1);

	/* Fail gracefully if need be */
	if (err)
		printk(KERN_WARNING "pov: error %d adding pov%d\n", err, index);
	else
		printk(KERN_INFO "pov: added pov%d (%d %d)\n", index, pov_major, pov_minor + index);
	return err;
}

static int __init pov_init_module(void)
{
	int err = 0, i;
	int quotient, remainder;
	//проверка параметров
	if( irq != 5 && irq != 7 && irq != 10 && irq != 11 && irq != 12 )
	{
		printk(KERN_WARNING "pov: unsupported irq number %d\n", irq);
		err = -EINVAL;
		goto end;
	}

	quotient = sample_rate/BASE_SAMPLE_RATE, remainder = sample_rate%BASE_SAMPLE_RATE;
	if( quotient < 1 || quotient > MAX_SAMPLE_RATE_FACTOR || remainder )
	{
		printk(KERN_WARNING "pov: invalid value for parameter 'sample_rate' %d\n", sample_rate);
		err = -EINVAL;
		goto end;
	}
	if( !pov_mask || (pov_mask & POV_MASK) != pov_mask )
	{
		printk(KERN_WARNING "pov: invalid value for parameter 'pov_mask' %d\n", pov_mask);
		err = -EINVAL;
		goto end;
	}

	err = alloc_chrdev_region(&dev, pov_minor, MAX_CHANNELS,  "pov");
	pov_major = MAJOR(dev);
	if ( err < 0 )
	{
		printk(KERN_WARNING "pov: can't get major %d\n", pov_major);
		goto end;
	}

	//Запросить прерывание в совместное пользование, в качестве уникального идентификатора - pov_major
	err = request_irq( irq, pov_irq_handler, SA_SHIRQ, "pov", &pov_major );
	if( err )
	{
		printk(KERN_WARNING "pov: installing an interrupt handler for irq %d failed\n", irq);
		goto end;
	}


	for(i=0; i < MAX_CHANNELS; i++)
		if( pov_mask&(0x1<<i) )
		{
			err = pov_setup_cdev( &channels[i], i );
			if( err )
			{
				printk(KERN_WARNING "pov: device pov%d setup error\n",  i);
				cdev_del( &channels[ i ].cdev );
				unregister_chrdev_region(dev, MAX_CHANNELS);
				free_irq(  irq, &pov_major );
				break;
			}
			channels[i].state = state_closed;
		}


	if( !err )
		printk(KERN_INFO "pov: init sample_rate=%d pov_mask=%d\n", sample_rate, pov_mask);

end:
	return err;
}



static void __exit pov_exit_module(void)
{
	int i;
	exit = 1;
	for( i = 0; i < MAX_CHANNELS; i++ )
		if( channels[ i ].state != state_opened )
			dev_release( &channels[ i ] );

	free_irq(  irq, &pov_major );
	tasklet_kill( &pov_tasklet );
	for( i = 0; i < MAX_CHANNELS; i++ )
		if( channels[ i ].state != state_unused )
			cdev_del( &channels[ i ].cdev );
	unregister_chrdev_region(dev, MAX_CHANNELS);
	printk( KERN_DEBUG "Module pov exit\n" );
}

module_init(pov_init_module);
module_exit(pov_exit_module);

MODULE_DESCRIPTION("POV driver");
MODULE_AUTHOR("Igor Bykov (igrbkv@rambler.ru)");
MODULE_LICENSE("GPL");
