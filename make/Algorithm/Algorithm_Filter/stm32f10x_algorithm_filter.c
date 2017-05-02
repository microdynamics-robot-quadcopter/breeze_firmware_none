/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_filter.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.05
Description: Implement the filter function
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

#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_filter.h"

static float cutoff_freq1;
static float a11;
static float a21;
static float b01;
static float b11;
static float b21;
static float delay_element_11;
static float delay_element_21;

void Filter_SetLPF2pCutoffFreq_1(float sample_freq, float cutoff_freq)
{
    float fr  = 0;
    float ohm = 0;
    float c   = 0;

    fr  = sample_freq / cutoff_freq;
    ohm = tanf(M_PI / fr);
    c   = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

    cutoff_freq1 = cutoff_freq;

    if (cutoff_freq1 > 0.0f)
    {
        b01 = ohm * ohm / c;
        b11 = 2.0f * b01;
        b21 = b01;
        a11 = 2.0f * (ohm * ohm - 1.0f) / c;
        a21 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    }
}

float Filter_ApplyLPF2p_1(float sample)
{
    float delay_element_0 = 0;
    float output = 0;

    if (cutoff_freq1 <= 0.0f)
    {
        // Do not the filtering.
        return sample;
    }
    else
    {
        delay_element_0 = sample - delay_element_11 * a11 - delay_element_21 *
            a21;
        // Do the filtering.
        if (isnan(delay_element_0) || isinf(delay_element_0))
        {
            // Don't allow bad values to propogate via the filter.
            delay_element_0 = sample;
        }
        output = delay_element_0 * b01 + delay_element_11 * b11 +
            delay_element_21 * b21;
        delay_element_21 = delay_element_11;
        delay_element_11 = delay_element_0;
        return output;
    }
}

static float cutoff_freq2;
static float a12;
static float a22;
static float b02;
static float b12;
static float b22;
static float delay_element_12;
static float delay_element_22;

void Filter_SetLPF2pCutoffFreq_2(float sample_freq, float cutoff_freq)
{
    float fr  = 0;
    float ohm = 0;
    float c   = 0;

    fr  = sample_freq / cutoff_freq;
    ohm = tanf(M_PI / fr);
    c   = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

    cutoff_freq2 = cutoff_freq;

    if (cutoff_freq2 > 0.0f)
    {
        b02 = ohm * ohm / c;
        b12 = 2.0f *b02;
        b22 = b02;
        a12 = 2.0f * (ohm * ohm - 1.0f) / c;
        a22 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    }
}

float Filter_ApplyLPF2p_2(float sample)
{
    float delay_element_0 = 0;
    float output = 0;

    if (cutoff_freq2 <= 0.0f)
    {
        // Do not the filtering.
        return sample;
    }
    else
    {
        delay_element_0 = sample - delay_element_12 * a12 - delay_element_22 *
            a22;
        // Do the filtering.
        if (isnan(delay_element_0) || isinf(delay_element_0))
        {
            // Don't allow bad values to propogate via the filter.
            delay_element_0 = sample;
        }
        output = delay_element_0 * b02 + delay_element_12 * b12 +
            delay_element_22 * b22;
        delay_element_22 = delay_element_12;
        delay_element_12 = delay_element_0;
        return output;
    }
}

static float  cutoff_freq3;
static float  a13;
static float  a23;
static float  b03;
static float  b13;
static float  b23;
static float  delay_element_13;
static float  delay_element_23;

void Filter_SetLPF2pCutoffFreq_3(float sample_freq, float cutoff_freq)
{
    float fr  = 0;
    float ohm = 0;
    float c   = 0;

    fr  = sample_freq / cutoff_freq;
    ohm = tanf(M_PI / fr);
    c   = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

    cutoff_freq3 = cutoff_freq;

    if (cutoff_freq3 > 0.0f)
    {
        b03 = ohm * ohm / c;
        b13 = 2.0f * b03;
        b23 = b03;
        a13 = 2.0f * (ohm * ohm - 1.0f) / c;
        a23 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    }
}

float Filter_ApplyLPF2p_3(float sample)
{
    float delay_element_0 = 0;
    float output = 0;

    if (cutoff_freq3 <= 0.0f)
    {
        // Do not the filtering.
        return sample;
    }
    else
    {
        delay_element_0 = sample - delay_element_13 * a13 - delay_element_23 *
            a23;
        // Do the filtering.
        if (isnan(delay_element_0) || isinf(delay_element_0))
        {
            // Don't allow bad values to propogate via the filter.
            delay_element_0 = sample;
        }
        output = delay_element_0 * b03 + delay_element_13 * b13 +
            delay_element_23 * b23;
        delay_element_23 = delay_element_13;
        delay_element_13 = delay_element_0;
        return output;
    }
}

static float cutoff_freq4;
static float a14;
static float a24;
static float b04;
static float b14;
static float b24;
static float delay_element_14;
static float delay_element_24;

void Filter_SetLPF2pCutoffFreq_4(float sample_freq, float cutoff_freq)
{
    float fr  = 0;
    float ohm = 0;
    float c   = 0;

    fr  = sample_freq / cutoff_freq;
    ohm = tanf(M_PI / fr);
    c   = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

    cutoff_freq4 = cutoff_freq;

    if (cutoff_freq4 > 0.0f)
    {
        b04 = ohm * ohm / c;
        b14 = 2.0f * b04;
        b24 = b04;
        a14 = 2.0f * (ohm * ohm - 1.0f) / c;
        a24 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    }
}

float Filter_ApplyLPF2p_4(float sample)
{
    float delay_element_0 = 0;
    float output = 0;

    if (cutoff_freq4 <= 0.0f)
    {
        // Do not the filtering.
        return sample;
    }
    else
    {
        delay_element_0 = sample - delay_element_14 * a14 - delay_element_24 *
            a24;
        // Do the filtering.
        if (isnan(delay_element_0) || isinf(delay_element_0))
        {
            // Don't allow bad values to propogate via the filter.
            delay_element_0 = sample;
        }
        output = delay_element_0 * b04 + delay_element_14 * b14 +
            delay_element_24 * b24;
        delay_element_24 = delay_element_14;
        delay_element_14 = delay_element_0;
        return output;
    }
}

static float cutoff_freq5;
static float a15;
static float a25;
static float b05;
static float b15;
static float b25;
static float delay_element_15;
static float delay_element_25;

void Filter_SetLPF2pCutoffFreq_5(float sample_freq, float cutoff_freq)
{
    float fr  = 0;
    float ohm = 0;
    float c   = 0;

    fr  = sample_freq / cutoff_freq;
    ohm = tanf(M_PI / fr);
    c   = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

    cutoff_freq5 = cutoff_freq;

    if (cutoff_freq5 > 0.0f)
    {
        b05 = ohm * ohm / c;
        b15 = 2.0f * b05;
        b25 = b05;
        a15 = 2.0f * (ohm * ohm - 1.0f) / c;
        a25 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    }
}

float Filter_ApplyLPF2p_5(float sample)
{
    float delay_element_0 = 0;
    float output = 0;

    if (cutoff_freq5 <= 0.0f)
    {
        // Do not the filtering.
        return sample;
    }
    else
    {
        delay_element_0 = sample - delay_element_15 * a15 - delay_element_25 *
            a25;
        // Do the filtering.
        if (isnan(delay_element_0) || isinf(delay_element_0))
        {
            // Don't allow bad values to propogate via the filter.
            delay_element_0 = sample;
        }
        output = delay_element_0 * b05 + delay_element_15 * b15 +
            delay_element_25 * b25;
        delay_element_25 = delay_element_15;
        delay_element_15 = delay_element_0;
        return output;
    }
}

static float cutoff_freq6;
static float a16;
static float a26;
static float b06;
static float b16;
static float b26;
static float delay_element_16;
static float delay_element_26;

void Filter_SetLPF2pCutoffFreq_6(float sample_freq, float cutoff_freq)
{
    float fr  = 0;
    float ohm = 0;
    float c   = 0;

    fr  = sample_freq / cutoff_freq;
    ohm = tanf(M_PI / fr);
    c   = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

    cutoff_freq6 = cutoff_freq;

    if (cutoff_freq6 > 0.0f)
    {
        b06 = ohm * ohm / c;
        b16 = 2.0f * b06;
        b26 = b06;
        a16 = 2.0f * (ohm * ohm - 1.0f) / c;
        a26 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    }
}

float Filter_ApplyLPF2p_6(float sample)
{
    float delay_element_0 = 0;
    float output = 0;

    if (cutoff_freq6 <= 0.0f)
    {
        // Do not the filtering.
        return sample;
    }
    else
    {
        delay_element_0 = sample - delay_element_16 * a16 - delay_element_26 *
            a26;
        // Do the filtering.
        if (isnan(delay_element_0) || isinf(delay_element_0))
        {
            // Don't allow bad values to propogate via the filter.
            delay_element_0 = sample;
        }
        output = delay_element_0 * b06 + delay_element_16 * b16 +
            delay_element_26 * b26;
        delay_element_26 = delay_element_16;
        delay_element_16 = delay_element_0;
        return output;
    }
}
