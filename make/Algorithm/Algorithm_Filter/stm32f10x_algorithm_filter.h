/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_filter.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.05
Description: Declare the filter function
Others:      none
Function List:
             1.  void  Filter_SetLPF2pCutoffFreq_1(float sample_freq,
                                                   float cutoff_freq);
             2.  void  Filter_SetLPF2pCutoffFreq_2(float sample_freq,
                                                   float cutoff_freq);
             3.  void  Filter_SetLPF2pCutoffFreq_3(float sample_freq,
                                                   float cutoff_freq);
             4.  void  Filter_SetLPF2pCutoffFreq_4(float sample_freq,
                                                   float cutoff_freq);
             5.  void  Filter_SetLPF2pCutoffFreq_5(float sample_freq,
                                                   float cutoff_freq);
             6.  void  Filter_SetLPF2pCutoffFreq_6(float sample_freq,
                                                   float cutoff_freq);
             7.  float Filter_ApplyLPF2p_1(float sample);
             8.  float Filter_ApplyLPF2p_2(float sample);
             9.  float Filter_ApplyLPF2p_3(float sample);
             10. float Filter_ApplyLPF2p_4(float sample);
             11. float Filter_ApplyLPF2p_5(float sample);
             12. float Filter_ApplyLPF2p_6(float sample);
History:
<author>    <date>        <desc>
maksyuki    2017.01.11    Modify the module
myyerrol    2017.04.30    Format the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_FILTER_H__
#define __STM32F10X_ALGORITHM_FILTER_H__

extern void  Filter_SetLPF2pCutoffFreq_1(float sample_freq, float cutoff_freq);
extern void  Filter_SetLPF2pCutoffFreq_2(float sample_freq, float cutoff_freq);
extern void  Filter_SetLPF2pCutoffFreq_3(float sample_freq, float cutoff_freq);
extern void  Filter_SetLPF2pCutoffFreq_4(float sample_freq, float cutoff_freq);
extern void  Filter_SetLPF2pCutoffFreq_5(float sample_freq, float cutoff_freq);
extern void  Filter_SetLPF2pCutoffFreq_6(float sample_freq, float cutoff_freq);
extern float Filter_ApplyLPF2p_1(float sample);
extern float Filter_ApplyLPF2p_2(float sample);
extern float Filter_ApplyLPF2p_3(float sample);
extern float Filter_ApplyLPF2p_4(float sample);
extern float Filter_ApplyLPF2p_5(float sample);
extern float Filter_ApplyLPF2p_6(float sample);

#endif
