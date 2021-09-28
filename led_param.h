#ifndef _LED_PARAM_H_
#define _LED_PARAM_H_

//#define LED_1111

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
