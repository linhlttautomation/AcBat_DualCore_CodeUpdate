/*
 * SVM3D_DQZ_CLA.h
 *
 *  Created on: May 5, 2024
 *      Author: Linh
 */

#ifndef SVM3D_DQZ_H_
#define SVM3D_DQZ_H_

typedef struct  { float  Ualpha;            // Input: reference alpha-axis phase voltage
                  float  Ubeta;             // Input: reference beta-axis phase voltage
                  float  Ugamma;            // Input: reference gamma-axis phase voltage
                  float  Udc;               // Input: reference Udc voltage
                  float  da;                // Output: reference phase-a switching function
                  float  db;                // Output: reference phase-b switching function
                  float  dc;                // Output: reference phase-c switching function
                  float  dn;                // Output: reference phase-n switching function
                  float  Ua;
                  float  Ub;
                  float  Uc;
                  float  d1;
                  float  d2;
                  float  d3;
                  float  d0;
                  float  inver_Udc;
                  Uint16 Prism;
                } SVM3D_DQZ_GEN_CLA;

#define SVM3D_DQZ_GEN_CLA_MACRO(v)                                                                                          \
    /* Inverse Clarke Transformation */                                                                                     \
    v.Ua = v.Ualpha + v.Ugamma;                                                                                             \
    v.Ub = -0.5*v.Ualpha + 0.8660254037844386*v.Ubeta + v.Ugamma;                                                           \
    v.Uc = -0.5*v.Ualpha - 0.8660254037844386*v.Ubeta + v.Ugamma;                                                           \
    /* Normalization */                                                                                                     \
    v.inver_Udc = (Us_max*1.0/Udc_max)*(1.0/v.Udc);                                                                       \
    /* Prism Identification */                                                                                              \
    if ((v.Ualpha*v.Ubeta >= 0) && (v.Ubeta >= 0) && (fabs(v.Ualpha) >= fabs(v.Ubeta)/1.732050807568877)) v.Prism = 1;      \
    if ((v.Ualpha*v.Ubeta >= 0) && (v.Ubeta >= 0) && (fabs(v.Ualpha) < fabs(v.Ubeta)/1.732050807568877)) v.Prism = 2;       \
    if ((v.Ualpha*v.Ubeta >= 0) && (v.Ubeta < 0) && (fabs(v.Ualpha) >= fabs(v.Ubeta)/1.732050807568877)) v.Prism = 4;       \
    if ((v.Ualpha*v.Ubeta >= 0) && (v.Ubeta < 0) && (fabs(v.Ualpha) < fabs(v.Ubeta)/1.732050807568877)) v.Prism = 5;        \
    if ((v.Ualpha*v.Ubeta < 0) && (v.Ubeta >= 0) && (fabs(v.Ualpha) >= fabs(v.Ubeta)/1.732050807568877)) v.Prism = 3;       \
    if ((v.Ualpha*v.Ubeta < 0) && (v.Ubeta >= 0) && (fabs(v.Ualpha) < fabs(v.Ubeta)/1.732050807568877)) v.Prism = 2;        \
    if ((v.Ualpha*v.Ubeta < 0) && (v.Ubeta < 0) && (fabs(v.Ualpha) >= fabs(v.Ubeta)/1.732050807568877)) v.Prism = 6;        \
    if ((v.Ualpha*v.Ubeta < 0) && (v.Ubeta < 0) && (fabs(v.Ualpha) < fabs(v.Ubeta)/1.732050807568877)) v.Prism = 5;         \
    /* v.da = 0.7; v.db = 0.7; v.dc = 0.7; v.dn = 0.7; */                                                                   \
    /* Duty Cycle Calculation */                                                                                            \
    switch(v.Prism)                                                                                                         \
    {                                                                                                                       \
    case 0:                                                                                                                 \
        v.da = 0;                                                                                                           \
        v.db = 0;                                                                                                           \
        v.dc = 0;                                                                                                           \
        v.dn = 0;                                                                                                           \
        break;                                                                                                              \
    case 1:                                                                                                                 \
        if ((v.Ua >= 0) && (v.Ub >= 0) && (v.Uc >= 0))                                                                      \
        {                                                                                                                   \
            /* Prism 1 Tetra 3 */                                                                                           \
            v.d1 = (1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d2 = (1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                 \
            v.d3 = (-0.5*v.Ualpha - 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.db = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
            v.dc = 0.5*v.d0 + v.d3;                                                                                         \
            v.dn = 0.5*v.d0;                                                                                                \
        }                                                                                                                   \
        else if ((v.Ua >= 0) && (v.Ub >= 0) && (v.Uc < 0))                                                                  \
        {                                                                                                                   \
            /* Prism 1 Tetra 2 */                                                                                           \
            v.d1 = (1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d2 = (-0.5*v.Ualpha + 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d3 = (0.5*v.Ualpha + 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.db = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
            v.dc = 0.5*v.d0;                                                                                                \
            v.dn = 0.5*v.d0 + v.d3;                                                                                         \
        }                                                                                                                   \
        else if ((v.Ua >= 0) && (v.Ub < 0) && (v.Uc <= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 1 Tetra 1 */                                                                                           \
            v.d1 = (v.Ualpha + v.Ugamma)*v.inver_Udc;                                                                       \
            v.d2 = (0.5*v.Ualpha - 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d3 = (1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                 \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.db = 0.5*v.d0 + v.d3;                                                                                         \
            v.dc = 0.5*v.d0;                                                                                                \
            v.dn = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub <= 0) && (v.Uc <= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 1 Tetra 4 */                                                                                           \
            v.d1 = (1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d2 = (1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                 \
            v.d3 = (-v.Ualpha - v.Ugamma)*v.inver_Udc;                                                                      \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.db = 0.5*v.d0 + v.d2;                                                                                         \
            v.dc = 0.5*v.d0;                                                                                                \
            v.dn = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
        }                                                                                                                   \
        break;                                                                                                              \
    case 2:                                                                                                                 \
        if ((v.Ua >= 0) && (v.Ub >= 0) && (v.Uc >= 0))                                                                      \
        {                                                                                                                   \
            /* Prism 2 Tetra 3 */                                                                                           \
            v.d1 = (1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d2 = (-1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d3 = (-0.5*v.Ualpha - 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d3;                                                                                  \
            v.db = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dc = 0.5*v.d0 + v.d3;                                                                                         \
            v.dn = 0.5*v.d0;                                                                                                \
        }                                                                                                                   \
        else if ((v.Ua >= 0) && (v.Ub >= 0) && (v.Uc < 0))                                                                  \
        {                                                                                                                   \
            /* Prism 2 Tetra 1 */                                                                                           \
            v.d1 = (v.Ualpha + v.Ugamma)*v.inver_Udc;                                                                       \
            v.d2 = (0.5*v.Ualpha + 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d3 = (-1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.db = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dc = 0.5*v.d0;                                                                                                \
            v.dn = 0.5*v.d0 + v.d2;                                                                                         \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub >= 0) && (v.Uc <= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 2 Tetra 2 */                                                                                           \
            v.d1 = (1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d2 = (-0.5*v.Ualpha + 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d3 = (-v.Ualpha - v.Ugamma)*v.inver_Udc;                                                                      \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1;                                                                                         \
            v.db = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dc = 0.5*v.d0;                                                                                                \
            v.dn = 0.5*v.d0 + v.d1 + v.d3;                                                                                  \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub < 0) && (v.Uc <= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 2 Tetra 4 */                                                                                           \
            v.d1 = (1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d2 = (-1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d3 = (0.5*v.Ualpha - 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1;                                                                                         \
            v.db = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.dc = 0.5*v.d0;                                                                                                \
            v.dn = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
        }                                                                                                                   \
        break;                                                                                                              \
    case 3:                                                                                                                 \
        if ((v.Ua >= 0) && (v.Ub >= 0) && (v.Uc >= 0))                                                                      \
        {                                                                                                                   \
            /* Prism 3 Tetra 3 */                                                                                           \
            v.d1 = (1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                 \
            v.d2 = (-1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d3 = (v.Ualpha + v.Ugamma)*v.inver_Udc;                                                                       \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d3;                                                                                         \
            v.db = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dc = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
            v.dn = 0.5*v.d0;                                                                                                \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub >= 0) && (v.Uc >= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 3 Tetra 2 */                                                                                           \
            v.d1 = (1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                 \
            v.d2 = (-0.5*v.Ualpha - 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d3 = (-v.Ualpha - v.Ugamma)*v.inver_Udc;                                                                      \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0;                                                                                                \
            v.db = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dc = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
            v.dn = 0.5*v.d0 + v.d3;                                                                                         \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub >= 0) && (v.Uc < 0))                                                                   \
        {                                                                                                                   \
            /* Prism 3 Tetra 1 */                                                                                           \
            v.d1 = (-0.5*v.Ualpha + 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d2 = (0.5*v.Ualpha + 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d3 = (-1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0;                                                                                                \
            v.db = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dc = 0.5*v.d0 + v.d3;                                                                                         \
            v.dn = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub < 0) && (v.Uc <= 0))                                                                   \
        {                                                                                                                   \
            /* Prism 3 Tetra 4 */                                                                                           \
            v.d1 = (1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                 \
            v.d2 = (-1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d3 = (-0.5*v.Ualpha + 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0;                                                                                                \
            v.db = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.dc = 0.5*v.d0 + v.d2;                                                                                         \
            v.dn = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
        }                                                                                                                   \
        break;                                                                                                              \
    case 4:                                                                                                                 \
        if ((v.Ua >= 0) && (v.Ub >= 0) && (v.Uc >= 0))                                                                      \
        {                                                                                                                   \
            /* Prism 4 Tetra 3 */                                                                                           \
            v.d1 = (-1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d2 = (-1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                 \
            v.d3 = (v.Ualpha + v.Ugamma)*v.inver_Udc;                                                                       \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d3;                                                                                         \
            v.db = 0.5*v.d0 + v.d1 + v.d3;                                                                                  \
            v.dc = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dn = 0.5*v.d0;                                                                                                \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub >= 0) && (v.Uc >= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 4 Tetra 1 */                                                                                           \
            v.d1 = (-0.5*v.Ualpha + 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d2 = (-v.Ualpha - v.Ugamma)*v.inver_Udc;                                                                      \
            v.d3 = (-1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0;                                                                                                \
            v.db = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.dc = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dn = 0.5*v.d0 + v.d2;                                                                                         \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub < 0) && (v.Uc >= 0))                                                                   \
        {                                                                                                                   \
            /* Prism 4 Tetra 2 */                                                                                           \
            v.d1 = (-1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d2 = (-0.5*v.Ualpha - 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d3 = (0.5*v.Ualpha - 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0;                                                                                                \
            v.db = 0.5*v.d0 + v.d1;                                                                                         \
            v.dc = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dn = 0.5*v.d0 + v.d1 + v.d3;                                                                                  \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub < 0) && (v.Uc < 0))                                                                    \
        {                                                                                                                   \
            /* Prism 4 Tetra 4 */                                                                                           \
            v.d1 = (-1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d2 = (-1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                \
            v.d3 = (0.5*v.Ualpha + 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0;                                                                                                \
            v.db = 0.5*v.d0 + v.d1;                                                                                         \
            v.dc = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.dn = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
        }                                                                                                                   \
        break;                                                                                                              \
    case 5:                                                                                                                 \
        if ((v.Ua >= 0) && (v.Ub >= 0) && (v.Uc >= 0))                                                                      \
        {                                                                                                                   \
            /* Prism 5 Tetra 3 */                                                                                           \
            v.d1 = (-1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d2 = (1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d3 = (-0.5*v.Ualpha + 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
            v.db = 0.5*v.d0 + v.d3;                                                                                         \
            v.dc = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dn = 0.5*v.d0;                                                                                                \
        }                                                                                                                   \
        else if ((v.Ua >= 0) && (v.Ub < 0) && (v.Uc >= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 5 Tetra 2 */                                                                                           \
            v.d1 = (-1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d2 = (v.Ualpha + v.Ugamma)*v.inver_Udc;                                                                       \
            v.d3 = (0.5*v.Ualpha - 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
            v.db = 0.5*v.d0;                                                                                                \
            v.dc = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dn = 0.5*v.d0 + v.d3;                                                                                         \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub <= 0) && (v.Uc >= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 5 Tetra 1 */                                                                                           \
            v.d1 = (-0.5*v.Ualpha - 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                            \
            v.d2 = (-v.Ualpha - v.Ugamma)*v.inver_Udc;                                                                      \
            v.d3 = (1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d3;                                                                                         \
            v.db = 0.5*v.d0;                                                                                                \
            v.dc = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.dn = 0.5*v.d0 + v.d2 + v.d3;                                                                                  \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub <= 0) && (v.Uc < 0))                                                                   \
        {                                                                                                                   \
            /* Prism 5 Tetra 4 */                                                                                           \
            v.d1 = (-1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                      \
            v.d2 = (1.5*v.Ualpha - 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d3 = (0.5*v.Ualpha + 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d2;                                                                                         \
            v.db = 0.5*v.d0;                                                                                                \
            v.dc = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.dn = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
        }                                                                                                                   \
        break;                                                                                                              \
    case 6:                                                                                                                 \
        if ((v.Ua >= 0) && (v.Ub >= 0) && (v.Uc >= 0))                                                                      \
        {                                                                                                                   \
            /* Prism 6 Tetra 3 */                                                                                           \
            v.d1 = (-1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                \
            v.d2 = (1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d3 = (-0.5*v.Ualpha + 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.db = 0.5*v.d0 + v.d3;                                                                                         \
            v.dc = 0.5*v.d0 + v.d1 + v.d3;                                                                                  \
            v.dn = 0.5*v.d0;                                                                                                \
        }                                                                                                                   \
        else if ((v.Ua >= 0) && (v.Ub < 0) && (v.Uc >= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 6 Tetra 1 */                                                                                           \
            v.d1 = (-0.5*v.Ualpha - 0.8660254038*v.Ubeta + v.Ugamma)*v.inver_Udc;                                           \
            v.d2 = (0.5*v.Ualpha - 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d3 = (1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.db = 0.5*v.d0;                                                                                                \
            v.dc = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.dn = 0.5*v.d0 + v.d2;                                                                                         \
        }                                                                                                                   \
        else if ((v.Ua >= 0) && (v.Ub < 0) && (v.Uc < 0))                                                                   \
        {                                                                                                                   \
            /* Prism 6 Tetra 2 */                                                                                           \
            v.d1 = (-1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                \
            v.d2 = (v.Ualpha + v.Ugamma)*v.inver_Udc;                                                                       \
            v.d3 = (0.5*v.Ualpha + 0.8660254038*v.Ubeta - v.Ugamma)*v.inver_Udc;                                            \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
            v.db = 0.5*v.d0;                                                                                                \
            v.dc = 0.5*v.d0 + v.d1;                                                                                         \
            v.dn = 0.5*v.d0 + v.d1 + v.d3;                                                                                  \
        }                                                                                                                   \
        else if ((v.Ua < 0) && (v.Ub <= 0) && (v.Uc <= 0))                                                                  \
        {                                                                                                                   \
            /* Prism 6 Tetra 4 */                                                                                           \
            v.d1 = (-1.732050807568877*v.Ubeta)*v.inver_Udc;                                                                \
            v.d2 = (1.5*v.Ualpha + 0.8660254038*v.Ubeta)*v.inver_Udc;                                                       \
            v.d3 = (-v.Ualpha - v.Ugamma)*v.inver_Udc;                                                                      \
            v.d0 = 1.0 - v.d1 - v.d2 - v.d3;                                                                                \
            v.da = 0.5*v.d0 + v.d1 + v.d2;                                                                                  \
            v.db = 0.5*v.d0;                                                                                                \
            v.dc = 0.5*v.d0 + v.d1;                                                                                         \
            v.dn = 0.5*v.d0 + v.d1 + v.d2 + v.d3;                                                                           \
        }                                                                                                                   \
        break;                                                                                                              \
    }                                                                                                                       \
    /* Limit v.da,v.db,v.dc,v.dn 0:1 */                                                                                     \
    if (v.da < 0.0) v.da = 0.0; else if(v.da > 0.98) v.da = 0.98;                                                           \
    if (v.db < 0.0) v.db = 0.0; else if(v.db > 0.98) v.db = 0.98;                                                           \
    if (v.dc < 0.0) v.dc = 0.0; else if(v.dc > 0.98) v.dc = 0.98;                                                           \
    if (v.dn < 0.0) v.dn = 0.0; else if(v.dn > 0.98) v.dn = 0.98;                                                           \

#define SVM3D_DQZ_GEN_CLA_INT(v)                                                                                            \
        v.Ualpha = 0;                                                                                                       \
        v.Ugamma = 0;                                                                                                       \
        v.Ubeta = 0;                                                                                                        \
        v.Udc = 0;                                                                                                          \
        v.da = 0;                                                                                                           \
        v.db = 0;                                                                                                           \
        v.dc = 0;                                                                                                           \
        v.dn = 0;                                                                                                           \
        v.Ua = 0;                                                                                                           \
        v.Ub = 0;                                                                                                           \
        v.Uc = 0;                                                                                                           \
        v.d1 = 0;                                                                                                           \
        v.d2 = 0;                                                                                                           \
        v.d3 = 0;                                                                                                           \
        v.d0 = 0;                                                                                                           \
        v.inver_Udc = 0;                                                                                                    \
        v.Prism = 0;                                                                                                        \

#endif /* SVM3D_DQZ_H_ */
