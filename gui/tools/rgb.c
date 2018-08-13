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

#include "rgb.h"

static const uint8_t Table5[] = {  0,   8,  16,  25,  33,  41,  49,  58, 
                     66,  74,  82,  90,  99, 107, 115, 123, 
                    132, 140, 148, 156, 165, 173, 181, 189, 
                    197, 206, 214, 222, 230, 239, 247, 255 };

static const uint8_t Table6[] = {  0,   4,   8,  12,  16,  20,  24,  28, 
                     32,  36,  40,  45,  49,  53,  57,  61, 
                     65,  69,  73,  77,  81,  85,  89,  93, 
                     97, 101, 105, 109, 113, 117, 121, 125, 
                    130, 134, 138, 142, 146, 150, 154, 158, 
                    162, 166, 170, 174, 178, 182, 186, 190, 
                    194, 198, 202, 206, 210, 215, 219, 223, 
                    227, 231, 235, 239, 243, 247, 251, 255 };

uint32_t rgb565_to_rgb888(uint16_t rgb565)
{
    rgb565_t c565;
    c565.color = rgb565;

    uint32_t r = Table5[c565.R];
    uint32_t g = Table6[c565.G];
    uint32_t b = Table5[c565.B];

    uint32_t rgb888 = (r << 16) | (g << 8) | b;  
    return rgb888;
}

uint16_t rgb888_to_rgb565(uint32_t rgb888)
{
    argb8888_t c8888;
    c8888.color = rgb888;

    uint32_t r5 = ( c8888.R * 249 + 1014 ) >> 11;
    uint32_t g6 = ( c8888.G * 253 +  505 ) >> 10;
    uint32_t b5 = ( c8888.B * 249 + 1014 ) >> 11;
    
    uint16_t rgb565 = (r5<<11) | (g6<<5) | b5;
    return rgb565;
}

rgb888_t uint32_to_rgb888(uint32_t color32)
{
    struct R8G8B8_STRUCT *col = (struct R8G8B8_STRUCT*)(&color32);
    return *col;
}

uint8_t colorspace2size(rgb_color_space_t colorspace) 
{
    int sizeInBytes = 0;
    switch (colorspace)
    {
        case COLORSPACE_R5G6B5:
            sizeInBytes = sizeof(struct R5G6B5_STRUCT);
            break;
        case COLORSPACE_R8G8B8:
            sizeInBytes = sizeof(struct R8G8B8_STRUCT);
            break;
        default:
            sizeInBytes = 0;
            break;
    }
    return sizeInBytes;
}
