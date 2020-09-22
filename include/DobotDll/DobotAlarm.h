#ifndef DOBOTALARM_H
#define DOBOTALARM_H

#ifdef _MSC_VER
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned long long uint64_t;
typedef signed long long int64_t;
#else
#include <stdint.h>
#endif

/*********************************************************************************************************
** Data structures
*********************************************************************************************************/

/*********************************************************************************************************
** Common parts
*********************************************************************************************************/
#pragma pack(push)
#pragma pack(1)

struct alarmState {
  uint8_t value[32];
};

// MTest
#endif // DOBOTDLL_H
