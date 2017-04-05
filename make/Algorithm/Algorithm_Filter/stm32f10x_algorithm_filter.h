/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_filter.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.05
Description: declare the filter function
Others:      none
Function List:
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.11  modify the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_FILTER_H__
#define __STM32F10X_ALGORITHM_FILTER_H__

extern void LPF2pSetCutOffFreq_1(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_2(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_3(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_4(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_5(float sample_freq, float cutoff_freq);
extern void LPF2pSetCutOffFreq_6(float sample_freq, float cutoff_freq);

extern float LPF2pApply_1(float sample);
extern float LPF2pApply_2(float sample);
extern float LPF2pApply_3(float sample);
extern float LPF2pApply_4(float sample);
extern float LPF2pApply_5(float sample);
extern float LPF2pApply_6(float sample);

#endif
