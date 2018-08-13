/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   mathext.h
 * Author: remote
 *
 * Created on 16 апреля 2018 г., 13:11
 */

#ifndef MATHEXT_H
#define MATHEXT_H

#ifdef __cplusplus
extern "C" {
#endif

void math_swap_if_arg0_bigger_arg1_int(int* arg0, int* arg1);
void math_swap_if_arg0_smaller_arg1_int(int* arg0, int* arg1);
int math_max_of_args_int(int arg0, int arg1);
int math_min_of_args_int(int arg0, int arg1);



#ifdef __cplusplus
}
#endif

#endif /* MATHEXT_H */

