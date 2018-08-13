// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "../../tools/rgb.h"
#include "../../tools/framebuffer.h"
  
#ifdef CONFIG_LCD_COLORSPACE_R8G8B8
   typedef struct R8G8B8_STRUCT color_t;
#endif

#ifdef CONFIG_LCD_COLORSPACE_R5G6B5
   typedef struct R5G6B5_STRUCT color_t;
#endif

typedef struct LCD
{
    void (*deinit)(void);
    void (*reset)(void);
    uint32_t (*get_x_resolution)(void);
    uint32_t (*get_y_resolution)(void);
    color_t* (*get_frame_buffer)(void);
    void (*set_pixel)(int xpos,int ypos,color_t color); 
    void (*fill_horiz_line)(int xpos,int ypos,int xpos2,color_t color);
    void (*fill_rect)(int xpos,int ypos,int xpos2,int ypos2,color_t color);
    void (*put_image)(int xpos, int ypos, int xpos2, int ypos2, fb_t* src);
}lcd_t;

void lcd_init(lcd_t* lcd);

#ifdef __cplusplus
}
#endif
