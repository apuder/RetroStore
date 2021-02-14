
#ifndef __SCREEN_H__
#define __SCREEN_H__

#include "defs.h"

#define SCREEN_BASE 0x3c00
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 16

typedef void (*screen_update_range_t)(uint8_t*, uint8_t*);

typedef struct {
  uint8_t  width;
  uint8_t  height;
  uint8_t is_uc;
  uint8_t* screen_base;
  uint8_t* background;
  uint8_t* current;
  screen_update_range_t screen_update_range;
} SCREEN;

extern SCREEN screen;

// Convert character to uppercase if screen is uppercase-only to avoid gibberish.
#define SCREEN_TO_UC(UC, C)  (((UC) && ((C) >= 96 && (C) < 128)) ? ((C)-32) : (C))

void init_hardware();
void set_screen(uint8_t* screen_base,
		uint8_t* background,
		uint8_t width, uint8_t height);
void set_screen_callback(screen_update_range_t screen_update_range);
void set_screen_to_foreground();
void set_screen_to_background();
void screen_show(bool from_left);
void screen_update_full();
void screen_update_range(uint8_t* from, uint8_t* to);

#endif

