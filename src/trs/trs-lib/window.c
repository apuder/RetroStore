
#include "window.h"
#include "screen.h"
#include <string.h>

static const uint8_t scroll_increment = 5;

inline uint8_t* get_screen_pos0(window_t* wnd, uint8_t x, uint8_t y) {
  return (screen.current + (wnd->y + y) * screen.width +
		     (wnd->x + x));
}

inline uint8_t* get_screen_pos(window_t* wnd) {
  return get_screen_pos0(wnd, wnd->cx, wnd->cy);
}

void init_window(window_t* wnd, uint8_t x, uint8_t y, int8_t w, int8_t h) {
  wnd->cx = 0;
  wnd->cy = 0;
  wnd->x = x;
  wnd->y = y;
  wnd->w = (w > 0) ? w : screen.width + w - x;
  wnd->h = (h > 0) ? h : screen.height + h - y;
}

void wnd_cr(window_t* wnd) {
  wnd->cx = 0;
  wnd->cy++;
}

void wnd_goto(window_t* wnd, uint8_t x, uint8_t y) {
  wnd->cx = x;
  wnd->cy = y;
}

void wnd_print(window_t* wnd, bool single_line, const char* str) {
  const char* start_of_word;
  uint8_t len;
  uint8_t* p, from;
  
  while (*str != '\0') {
    const char* next = str;
    uint8_t* from = get_screen_pos(wnd);

    if (*next == '\n') {
      if (wnd->cy + 1 == wnd->h) {
        // We are in the last line. Just exit
        return;
      }
      wnd->cx = 0;
      wnd->cy++;
      str++;
      continue;
    }

    while ((*next != '\0') && (*next == ' ')) {
      next++;
    }

    start_of_word = next;

    while ((*next != '\0') && (*next != ' ') && (*next != '\n')) {
      next++;
    }

    len = next - str;

    // If we are in the last line, add space for 3 for ellipses
    if (wnd->cy + 1 == wnd->h) {
      len += 3;
    }
    
    // Fits in current line?
    if (wnd->cx + len > wnd->w) {
      // No. Do we have another line?
      if (single_line || wnd->cy + 1 == wnd->h) {
	// No. Stop outputting and print "..."
	from = p = get_screen_pos(wnd);
	*p++ = '.';
	*p++ = '.';
	*p++ = '.';
	screen_update_range(from, p);
	return;
      }
      // Yes. Continue on next line
      wnd->cy++;
      wnd->cx = 0;

      str = start_of_word;
    }

    from = p = get_screen_pos(wnd);
  
    while (str != next) {
      *p++ = SCREEN_TO_UC(screen.is_uc, *str);
      str++;
      wnd->cx++;
    }
    screen_update_range(from, p);
  }
} 

void wnd_print_int(window_t* wnd, uint16_t v) {
  char buf[10];
  int i = sizeof(buf);

  buf[--i] = '\0';
  while (v != 0) {
    buf[--i] = v % 10 + '0';
    v = v / 10;
  }
  if (i == sizeof(buf) - 1) {
    buf[--i] = '0';
  }
  wnd_print(wnd, false, buf + i);
}

static void window_screen_update(window_t* wnd)
{
  uint8_t* from = get_screen_pos0(wnd, 0, 0);
  uint8_t* to = get_screen_pos0(wnd, wnd->w - 1, wnd->h - 1) + 1;
  screen_update_range(from, to);
}

void wnd_cls(window_t* wnd) {
  uint8_t x, y;

  for (y = 0; y < wnd->h; y++) {
    uint8_t* p = get_screen_pos0(wnd, 0, y);

    for (x = 0; x < wnd->w; x++) {
      *p++ = ' ';
    }
  }
  wnd->cx = 0;
  wnd->cy = 0;
  window_screen_update(wnd);
}  

void wnd_clear_eol(window_t* wnd)
{
  uint8_t x;

  uint8_t* from = get_screen_pos(wnd);
  uint8_t* to = from;
  for (x = wnd->cx; x < wnd->w; x++) {
    *to++ = ' ';
  }
  screen_update_range(from, to);  
}

void wnd_scroll_up(window_t* wnd) {
  int8_t x, y;
  uint8_t* clear = NULL;

  for (y = 0; y < wnd->h - 1; y++) {
    uint8_t* to = get_screen_pos0(wnd, 0, y);
    uint8_t* from = to + screen.width;
    clear = from;
    
    for (x = 0; x < wnd->w; x++) {
      *to++ = *from++;
    }
  }
 
  for (x = 0; x < wnd->w; x++) {
    *clear++ = ' ';
  }

  wnd->cx = 0;
  wnd->cy = wnd->h - 1;
  window_screen_update(wnd);
}

void wnd_scroll_down(window_t* wnd) {
  int8_t x, y;
  uint8_t* clear = NULL;

  for (y = wnd->h - 1; y > 0; y--) {
    uint8_t* to = get_screen_pos0(wnd, 0, y);
    uint8_t* from = to - screen.width;
    clear = from;
    
    for (x = 0; x < wnd->w; x++) {
      *to++ = *from++;
    }
  }
 
  for (x = 0; x < wnd->w; x++) {
    *clear++ = ' ';
  }

  wnd->cx = 0;
  wnd->cy = 0;
  window_screen_update(wnd);
}

void wnd_popup(const char* msg) {
  int len = strlen(msg) + 2;
  int cx = (screen.width - len) / 2;
  int cy = screen.height / 2 - 1;
  uint8_t* p0 = screen.current + cy * screen.width + cx;
  uint8_t* p1 = p0 + screen.width;
  uint8_t* p2 = p1 + screen.width;
  uint8_t* p0_from = p0;
  uint8_t* p1_from = p1;
  uint8_t* p2_from = p2;
  int x, y;
  
  for (x = 0; x < len; x++) {
    *p0++ = 176;
    *p2++ = 131;
    if (x == 0 || x == len - 1) {
      *p1 = 191;
    } else {
      *p1 = SCREEN_TO_UC(screen.is_uc, msg[x - 1]);
    }
    p1++;
  }
  screen_update_range(p0_from, p0);
  screen_update_range(p1_from, p2);
  screen_update_range(p2_from, p2);  
}
