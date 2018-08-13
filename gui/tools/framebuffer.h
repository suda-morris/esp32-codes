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

#include <stdint.h>
#include <stdbool.h>

typedef struct FRAMEBUFFER_STRUCT
{
    uint16_t    width;
    uint16_t    height;
    uint8_t     colorspace;
    uint8_t    *buffer;    
}fb_t;
    
    
bool frame_buffer_create(uint8_t colorspace, int width, int height, struct FRAMEBUFFER_STRUCT *ret_fb);
void frame_buffer_destroy(struct FRAMEBUFFER_STRUCT *fb);
bool frame_buffer_is_valid(struct FRAMEBUFFER_STRUCT *fb);
uint32_t frame_buffer_calculate_byte_size(struct FRAMEBUFFER_STRUCT *fb);
void frame_buffer_clone(struct FRAMEBUFFER_STRUCT *fb_src, struct FRAMEBUFFER_STRUCT *ret_fb);
void frame_buffer_copy(struct FRAMEBUFFER_STRUCT *fb_src, struct FRAMEBUFFER_STRUCT *fb_dst);
bool frame_buffer_are_equal_w_h_cs(struct FRAMEBUFFER_STRUCT *fb_src, struct FRAMEBUFFER_STRUCT *fb_dst);
bool frame_buffer_are_equal(struct FRAMEBUFFER_STRUCT *fb_src, struct FRAMEBUFFER_STRUCT *fb_dst);
    

#ifdef __cplusplus
}
#endif
