/***************************************************************************
 *   Copyright (C) 2007 by Igor Bykov   *
 *   igrbkv@rambler.ru   *
 ***************************************************************************/
#ifndef POV_H
#define POV_H

#include <linux/ioctl.h>

#define POV_IOC_MAGIC  'p'

#define POV_IOCXTIMEBYOFFSET _IOWR(POV_IOC_MAGIC, 1, int)
#define POV_IOCXOFFSETBYTIME _IOWR(POV_IOC_MAGIC, 2, int)

#endif
