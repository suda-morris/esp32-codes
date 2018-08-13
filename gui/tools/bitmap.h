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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
    
#define BI_RGB 0L

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)  
struct BITMAPFILEHEADER_STRUCT 
{
  uint16_t  wType;
  uint32_t  dwSize;
  uint16_t  wReserved1;
  uint16_t  wReserved2;
  uint32_t  dwOffBits;
};

struct BITMAPINFOHEADER_STRUCT
{
  uint32_t  dwSize;
  int32_t   lWidth;
  int32_t   lHeight;
  uint16_t  wPlanes;
  uint16_t  wBitCount;
  uint32_t  dwCompression;
  uint32_t  dwSizeImage;
  int32_t   lXPelsPerMeter;
  int32_t   lYPelsPerMeter;
  uint32_t  dwClrUsed;
  uint32_t  dwClrImportant;
};
#pragma pack(pop)  /* restore alignment */


uint32_t    bmp_compute_imageR8B8G8_size_bytes(int width, int height);
bool        bmp_write_to_file_bmp_buffer_from_rgb(FILE *file_handle, uint8_t *rgb_buf_bytes, uint8_t rgb_buf_colorspace, int width, int height);
bool        bmp_create_file_from_rgb_buf(uint8_t* rgb_buffer, uint8_t rgb_buf_colorspace, int width, int height, const char* bmp_file_name);
bool        bmp_create_file_from_r5g6b5_buf(uint8_t* rgb_buffer, int width, int height, const char* bmp_file_name);
bool        bmp_create_file_from_r8g8b8_buf(uint8_t* rgb_buffer, int width, int height, const char* bmp_file_name);

#ifdef __cplusplus
}
#endif
