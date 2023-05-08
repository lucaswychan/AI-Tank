#ifndef colorcfg_h
#define colorcfg_h

#include "bsp_ov7725.h"

#define IMG_Y 0
#define IMG_W 240
#define IMG_H 320

#define ALLOW_FAIL_PER       2
#define ITERATER_NUM         8
#define COLOR_RANG           20
#define TRACE_NUM            1
#define COLOR_NUM            3

extern uint8_t global_page;
extern SEARCH_AREA_t area;
extern RESULT_t result[TRACE_NUM];
extern TARGET_CONDITION_t condition[COLOR_NUM];

#endif

