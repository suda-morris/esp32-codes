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
#include <stdlib.h>
#include "framebuffer.h"
#include "rgb.h"

void graphics_blur4x4_framebuffer_R5G6G5(fb_t *fb)
{
    if (fb == NULL)
        return;
    
    struct R5G6B5_STRUCT c565_0;
    struct R5G6B5_STRUCT c565_1;
    struct R5G6B5_STRUCT c565_2;
    struct R5G6B5_STRUCT c565_3;
	
    int w = fb->width;
    int h = fb->height;
	
    int r=0;
    int g=0;
    int b=0;
    uint16_t* buf16 = fb->buffer;
    for (int y=0; y<h; y++)
    {
        int addr = w*y;
        uint8_t* buf8 = (uint8_t*)buf16;
        for (int x=0; x<w; x++)
        {			
            if (x==0)
                c565_0.color = buf16[addr+w-1];
            else
                c565_0.color = buf16[addr+x-1];
			
            if (x==(w-1))
                c565_1.color = buf16[addr];
            else	
                c565_1.color = buf16[addr+x+1];

            if (y==0)
                c565_2.color = buf16[addr+x];
            else
                c565_2.color = buf16[addr+x-w];

            if (y==(h-1))
                c565_3.color = buf16[addr+x];
            else
                c565_3.color = buf16[addr+x+w];
			
            r = (int)(c565_0.R) + (int)(c565_1.R) + (int)(c565_2.R) + (int)(c565_3.R);
            g = (int)(c565_0.G) + (int)(c565_1.G) + (int)(c565_2.G) + (int)(c565_3.G);
            b = (int)(c565_0.B) + (int)(c565_1.B) + (int)(c565_2.B) + (int)(c565_3.B);

            r = r >> 2;
            g = g >> 2;
            b = b >> 2;

            c565_0.R = r;
            c565_0.G = g;
            c565_0.B = b;
			
            buf16[addr+x] = c565_0.color;	 
        }
    }
}                      