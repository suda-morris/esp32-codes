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
#include <string.h>
#include <assert.h>

#include "framebuffer.h"
#include "rgb.h"

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool frame_buffer_create(uint8_t colorspace, int width, int height, struct FRAMEBUFFER_STRUCT *ret_fb)
{
    if (ret_fb==NULL)
        return false;
    
    ret_fb->buffer = NULL;
    ret_fb->colorspace = colorspace;
    ret_fb->width = width;
    ret_fb->height = height;

    int memSizeInBytes = frame_buffer_calculate_byte_size(ret_fb);
    if (memSizeInBytes==0)
        return false;
            
    ret_fb->buffer = malloc(memSizeInBytes);
    if (ret_fb->buffer==NULL)
        return false;
    
    return true;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void frame_buffer_destroy(struct FRAMEBUFFER_STRUCT *fb)
{
    if (fb==NULL)
        return;
    
    if (fb->buffer!=NULL)
    {
        free(fb->buffer);
        fb->buffer = NULL;
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool frame_buffer_is_valid(struct FRAMEBUFFER_STRUCT *fb)
{
    if ( (fb==NULL) ||
         (fb->width==0) || 
         (fb->height==0) ||
         (fb->buffer==NULL) ||
         (colorspace2size(fb->colorspace)==0) )
        return false;
                
    return true;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool frame_buffer_is_valid_w_h_cs(struct FRAMEBUFFER_STRUCT *fb)
{
    if ( (fb==NULL) ||
         (fb->width==0) || 
         (fb->height==0) ||
         (colorspace2size(fb->colorspace)==0) )
        return false;
                
    return true;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

uint32_t frame_buffer_calculate_byte_size(struct FRAMEBUFFER_STRUCT *fb)
{
    if (!frame_buffer_is_valid_w_h_cs(fb)) 
        return 0;
    
    int colorSizeInBytes = colorspace2size(fb->colorspace);
    int byteSize = colorSizeInBytes*fb->width*fb->height;
    return byteSize;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void frame_buffer_clone(struct FRAMEBUFFER_STRUCT *fb_src, struct FRAMEBUFFER_STRUCT *ret_fb)
{   
    if (ret_fb==NULL)
        return;
    
    memset(ret_fb, 0, sizeof(*ret_fb));
    
    if (frame_buffer_is_valid(fb_src) == false)
        return;

    bool created = frame_buffer_create(fb_src->colorspace, fb_src->width,  fb_src->height, ret_fb);
    if (created == false)
        return;
    
    int sizeInBytes = frame_buffer_calculate_byte_size(ret_fb);
    memcpy(ret_fb->buffer, fb_src->buffer, sizeInBytes);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void frame_buffer_copy(struct FRAMEBUFFER_STRUCT *fb_src, struct FRAMEBUFFER_STRUCT *fb_dst)
{
    assert(0);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool frame_buffer_are_equal_w_h_cs(struct FRAMEBUFFER_STRUCT *fb_src, struct FRAMEBUFFER_STRUCT *fb_dst)
{
    if ( (fb_src==NULL) || 
         (fb_dst==NULL) )
        return false;
    
    return (fb_src->width == fb_dst->width) && (fb_src->height == fb_dst->height) && (fb_src->colorspace == fb_dst->colorspace);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool frame_buffer_are_equal(struct FRAMEBUFFER_STRUCT *fb_src, struct FRAMEBUFFER_STRUCT *fb_dst)
{
    if (frame_buffer_are_equal_w_h_cs(fb_src, fb_dst)==false)
        return false;
    
    if ( (fb_src->buffer==NULL) || 
         (fb_dst->buffer==NULL) || 
         (fb_src->buffer==fb_dst->buffer) )
        return false; 
    
    int szInBytes = frame_buffer_calculate_byte_size(fb_src);
    int equal = memcmp(fb_src->buffer, fb_dst->buffer, szInBytes);
    return (equal == 0);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


