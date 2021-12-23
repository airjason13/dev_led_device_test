#ifndef _LED_PARAM_H_
#define _LED_PARAM_H_

//#define LED_1111
#define LED_NUM         1000
#define LED_CHANNELS    3
#define LED_PORTS       8

#define LED_NORMAL_MODE 0
#define LED_AREA_MODE   1

#define LED_WIDTH       80
#define LED_HEIGHT      12

#ifdef LED_1111
#define COLOR_RED	0x400000
#define COLOR_GREEN	0x004000
#define COLOR_BLUE	0x000040
#define COLOR_WHITE	0x404040
#else
#define COLOR_RED	0x004000
#define COLOR_GREEN	0x400000
#define COLOR_BLUE	0x000040
#define COLOR_WHITE	0x404040
#endif
#endif
