/*
 * ButterWorthFilter.h
 *
 *  Created on: Aug 6, 2025
 *      Author: LuuLinhK65
 */

#ifndef _BUTTERWORTHFILTER_H_
#define _BUTTERWORTHFILTER_H_

typedef struct {
    float a0, a1, a2;     // Hệ số đầu vào
    float b1, b2;         // Hệ số phản hồi
    float x0, x1, x2;     // Đầu vào hiện tại và quá khứ
    float y0, y1, y2;     // Đầu ra hiện tại và quá khứ
} LOWPASSFILTER2;

typedef LOWPASSFILTER2* LOWPASSFILTER2_handle;

#define LOWPASSFILTER2_DEFAULTS { \
    0, 0, 0, 0, 0,                /* Hệ số */ \
    0, 0, 0,                     /* x0, x1, x2 */ \
    0, 0, 0                      /* y0, y1, y2 */ \
}

#define LOWPASSFILTER2_MACRO(v)                        \
    float fc = 100;                           \
    float fs = 10000;                          \
    float omega = 2.0f * 3.1415926f * fc / fs;  \
    float cos_omega = cosf(omega);                \
    float sin_omega = sinf(omega);                   \
    float alpha = sin_omega / (2.0f * sqrtf(2.0f));  \
    float a0 = 1.0f + alpha;                         \
    v.a0 = (1.0f - cos_omega) / 2.0f / a0;           \
    v.a1 = (1.0f - cos_omega) / a0;                  \
    v.a2 = v.a0;                                      \
    v.b1 = -2.0f * cos_omega / a0;                     \
    v.b2 = (1.0f - alpha) / a0;                        \
    v.y0 = v.a0*v.x0 + v.a1*v.x1 + v.a2*v.x2 - v.b1*v.y1 - v.b2*v.y2;    \
    v.x2 = v.x1;                                       \
    v.x1 = v.x0;                                       \
    v.y2 = v.y1;                                       \
    v.y1 = v.y0;                                       \

#define LOWPASSFILTER2_INT(v)            \
    v.a0 = 0;   \
    v.a1 = 0;   \
    v.a2 = 0;   \
    v.b1 = 0;   \
    v.b2 = 0;   \
    v.x0 = 0;   \
    v.x1 = 0;   \
    v.x2 = 0;   \
    v.y0 = 0;   \
    v.y1 = 0;   \
    v.y2 = 0;   \

#endif /* _BUTTERWORTHFILTER_H_ */
