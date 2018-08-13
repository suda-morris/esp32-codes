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
#include "bitmap.h"
#include "rgb.h"

#include <string.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// computed byte size for R8G8B8 data format
uint32_t bmp_compute_imageR8G8B8_size_bytes(int width, int height)
{
    int padding = 0;
    int scanlinebytes = width * 3;
    while ((scanlinebytes + padding) % 4 != 0) // DWORD = 4 bytes
            padding++;
    // get the padded scanline width
    int psw = scanlinebytes + padding;
    return psw*height;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// writes only RGB bitmap data without headers and other thechnical infos
bool bmp_write_to_file_bmp_buffer_from_rgb(FILE *file_handle, uint8_t *rgb_buf_bytes, uint8_t rgb_buf_colorspace, int width, int height)
{
    if (file_handle==NULL)
        return false;
 
    // now we have to find with how many bytes
    // we have to pad for the next DWORD boundary
    int padding = 0;
    int scanlinebytes = width * 3;
    while ((scanlinebytes + padding) % 4 != 0) // DWORD = 4 bytes
        padding++;
    
    // get the padded scanline width
    int psw = scanlinebytes + padding;

    // we can already store the size of the new padded buffer
    int line_size_bytes = psw;
    
    // and create new buffer for 1 line only
    uint8_t* line_buf = malloc(line_size_bytes);
    
    if (line_buf==NULL) // !!! ERROR: memory allocation failed
        return false;
    
    // fill the buffer with zero bytes then we dont have to add
    // extra padding zero bytes later on
    // memset(newbuf, 0, *newsize);
    
    uint8_t success_storing = true;
    // now we loop trough all bytes of the original buffer, 
    // swap the R and B bytes and the scanlines
    long bufpos = 0;   
    
    int szOfColor_bytes = 3;
    switch (rgb_buf_colorspace)
    {
        case COLORSPACE_R5G6B5:
            szOfColor_bytes = 2;
            break;
        case COLORSPACE_R8G8B8:
            szOfColor_bytes = 3;
            break;
        default:
            szOfColor_bytes = 3;
            break;
    }
  
    struct A8R8G8B8_STRUCT color8888;
//    uint32_t color888 = 0; 
//    uint8_t* color_buf = &color888;
    uint16_t color565 = 0;

    for (int y = 0; y < height; y++)
    {
        bufpos = szOfColor_bytes*((height-y-1)*width);  // position in original buffer to start scanline
        
        for (int x = 0; x < 3*width; x+=3, bufpos+=szOfColor_bytes)
        {                    
            switch(rgb_buf_colorspace)
            {
                case COLORSPACE_R5G6B5:
                    color565 = *((uint16_t*)(rgb_buf_bytes+bufpos));
                    color8888.color = rgb565_to_rgb888(color565);
                    break;
                case COLORSPACE_R8G8B8:
                    color8888.R = rgb_buf_bytes[bufpos+2]; // swap r and b
                    color8888.G = rgb_buf_bytes[bufpos+1]; // g stays
                    color8888.B = rgb_buf_bytes[bufpos+0]; // swap b and r
                    break;
                default:
                    break;
            }

            line_buf[x]     = color8888.byte0;                 
            line_buf[x + 1] = color8888.byte1; 
            line_buf[x + 2] = color8888.byte2;
        }

        // write one scanline
        long need_to_write_bytes = line_size_bytes;
        long written_bytes = fwrite(line_buf, sizeof(uint8_t), line_size_bytes, file_handle);    
        if (need_to_write_bytes != written_bytes) // ERROR: unxpected bytes has been written
        {
            success_storing = false;
            break;
        }
    }
       
    free(line_buf);
   
    return success_storing;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool bmp_create_file_from_rgb_buf(uint8_t* rgb_buffer, uint8_t rgb_buf_colorspace, int width, int height, const char* bmp_file_name)
{
        uint32_t paddedsize = bmp_compute_imageR8G8B8_size_bytes(width, height);
        
	struct BITMAPFILEHEADER_STRUCT bmfhs;
	struct BITMAPINFOHEADER_STRUCT infos;
	
	memset(&bmfhs, 0, sizeof(bmfhs));
	memset(&infos, 0, sizeof(infos));
	
	bmfhs.wType = 0x4d42;           // 0x4d42 = 'BM' signature
	bmfhs.wReserved1 = 0;
	bmfhs.wReserved2 = 0;
	bmfhs.dwOffBits = sizeof(bmfhs) + sizeof(infos); // number of bytes to start of bitmap bits
	bmfhs.dwSize = bmfhs.dwOffBits + paddedsize;
	
	infos.dwSize = sizeof(infos);
	infos.lWidth = width;
	infos.lHeight = height;
	infos.wPlanes = 1;              // we only have one bitplane
	infos.wBitCount = 24;           // RGB mode is 24 bits
	infos.dwCompression = BI_RGB;	
	infos.dwSizeImage = 0;          // can be 0 for 24 bit images
	infos.lXPelsPerMeter = 0x0ec4;  // paint and PSP use this values
	infos.lYPelsPerMeter = 0x0ec4;     
	infos.dwClrUsed = 0;            // we are in RGB mode and have no palette
	infos.dwClrImportant = 0;       // all colors are required

	// now we open the file to write to
//	HANDLE file = CreateFile ( bmpfile , GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL );
	        
        FILE* fp = fopen(bmp_file_name,"wb"); // write binary

        if (fp == NULL)
        {
            fclose(fp);
            return false;
        }

        long need_to_write_bytes = sizeof(bmfhs);
        long written_bytes = fwrite(&bmfhs, sizeof(uint8_t), need_to_write_bytes, fp);
        if (written_bytes != need_to_write_bytes)
        {
            fclose(fp);
            return false;
        }
        
        need_to_write_bytes = sizeof(infos);
        written_bytes = fwrite(&infos, sizeof(uint8_t), need_to_write_bytes, fp);
        if (written_bytes != need_to_write_bytes)
        {
            fclose(fp);
            return false;
        }

        if (bmp_write_to_file_bmp_buffer_from_rgb(fp, rgb_buffer, rgb_buf_colorspace, width, height) == false)
        {
            fclose(fp);
            return false;
        }
       
        fclose(fp);
	return true;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool bmp_create_file_from_r5g6b5_buf(uint8_t* rgb_buffer, int width, int height, const char* bmp_file_name)
{
    bool success = bmp_create_file_from_rgb_buf(rgb_buffer, COLORSPACE_R5G6B5, width, height, bmp_file_name);
    return success;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool bmp_create_file_from_r8g8b8_buf(uint8_t* rgb_buffer, int width, int height, const char* bmp_file_name)
{
    bool success = bmp_create_file_from_rgb_buf(rgb_buffer, COLORSPACE_R8G8B8, width, height, bmp_file_name);
    return success;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
