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

    
typedef enum RGB_COLOR_SPACE
{ 
    COLORSPACE_R5G6B5,
    COLORSPACE_R8G8B8,
    COLORSPACE_UNKNOW
}rgb_color_space_t;   


#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)  
typedef struct R5G6B5_STRUCT
{
    union
    {
        struct
        {
            uint16_t B : 5;
            uint16_t G : 6;
            uint16_t R : 5;
        };
        struct
        {
            uint8_t byte0;
            uint8_t byte1;
        };
        uint16_t color;        
    };
}rgb565_t;

typedef struct R8G8B8_STRUCT
{
    union
    {
        struct
        {
            uint8_t B : 8;
            uint8_t G : 8;
            uint8_t R : 8;
        };
        struct
        {
            uint8_t byte0;
            uint8_t byte1;
            uint8_t byte2;
        };
        uint32_t color;
    };
}rgb888_t;

typedef struct A8R8G8B8_STRUCT
{
    union
    {
        struct
        {
            uint8_t B : 8;
            uint8_t G : 8;
            uint8_t R : 8;
            uint8_t A : 8;
        };
        struct
        {
            uint8_t byte0;
            uint8_t byte1;
            uint8_t byte2;
            uint8_t byte3;
        };
        uint32_t color;
    };
}argb8888_t;
#pragma pack(pop)  /* restore alignment */

uint32_t rgb565_to_rgb888(uint16_t rgb565);
uint16_t rgb888_to_rgb565(uint32_t rgb888);
rgb888_t uint32_to_rgb888(uint32_t color32);
uint8_t colorspace2size(rgb_color_space_t colorspace);

#ifdef __cplusplus
}
#endif
