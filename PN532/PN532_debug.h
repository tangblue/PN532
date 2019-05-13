#ifndef __DEBUG_H__
#define __DEBUG_H__

//#define PN532_DEBUG

#ifdef ARDUINO
#include "Arduino.h"
#endif

#ifdef PN532_DEBUG
#ifdef ARDUINO
#define DMSG(args...)       Serial.print(args)
#define DMSG_STR(str)       Serial.println(str)
#define DMSG_HEX(num)       Serial.print(' '); Serial.print(num, HEX)
#define DMSG_INT(num)       Serial.print(' '); Serial.print(num)
#else /* ARDUINO */
#include <stdio.h>
#define DMSG(args...)       printf("%s", args)
#define DMSG_STR(str)       printf("%s\n", str)
#define DMSG_HEX(num)       printf(" %02X", num)
#define DMSG_INT(num)       printf(" %d", num)
#endif /* ARDUINO */
#else
#define DMSG(args...)
#define DMSG_STR(str)
#define DMSG_HEX(num)
#define DMSG_INT(num)
#endif

#endif
