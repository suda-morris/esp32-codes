/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   graphics.h
 * Author: remote
 *
 * Created on 7 мая 2018 г., 16:25
 */

#ifndef GRAPHICS_H
#define GRAPHICS_H

#ifdef __cplusplus
extern "C" {
#endif

void graphics_blur4x4_framebuffer_R5G6G5(struct FRAMEBUFFER_STRUCT *fb);

#ifdef __cplusplus
}
#endif

#endif /* GRAPHICS_H */

