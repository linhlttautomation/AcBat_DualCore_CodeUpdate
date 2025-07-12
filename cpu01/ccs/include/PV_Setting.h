/*
 * PV_Setting.h
 *
 *  Created on: Oct 22, 2024
 *      Author: 84339
 */

#ifndef _PV_SETTING_H_
#define _PV_SETTING_H_

// Define Level
#define LEVEL1        1           // Vong ho doc lap
#define LEVEL2        2           // Vong ho IFB
#define LEVEL3        3           // Vong dong doc lap
#define LEVEL4        4           // Vong ap doc lap
#define LEVEL5        5           // Vong Dong IFB
#define LEVEL6        6           // Vong AP IFB
#define LEVEL7        7           // Vong dong noi luoi 3P4W
#define LEVEL8        8           // Vong ap noi luoi 3P4W

// ---------------------------------------------------
#define BUILDLEVEL    LEVEL1
// ---------------------------------------------------

#define SINGLE_PHASE_MODE       1
#define THREE_PHASE_MODE        3

// ---------------------------------------------------
#define SET_MODE_RUN            THREE_PHASE_MODE
// ---------------------------------------------------

#define READ_VOLTAGE_AC_BEFORE_LPF         1
#define READ_VOLTAGE_AC_AFTER_LPF          2
#define READ_VOLTAGE_DC                    3
#define READ_CURRENT                       4

#define READ_VOLTAGE_AC_AFTER_KALMAN       5

#define READ_OUTPUT_CURRENT_CONTROLLER     6

// ---------------------------------------------------
#define SET_MODE_READ           READ_VOLTAGE_AC_AFTER_LPF
// ---------------------------------------------------

// ---------------------------------------------------
#define TUNNING_ADC              2 // 1: Manual; 2: No
#define DATA1_VG_RMS             220.0
#define DATA1_IG_RMS             1.0
// ---------------------------------------------------

#define ALLOW_IPC_CPU               1
#define ALLOW_WATCHDOG_TIMER        0
#define ALLOW_EXTERNAL_INTERRUPT    0
#define ALLOW_BUTTON                0
#define ALLOW_CALIB_PI              0
#define ALLOW_TIMER0                1

// ---------------------------------------------------
#define MODE_MODULATION_SVM2D       2
#define MODE_MODULATION_SVM3D       3

// ---------------------------------------------------
#define MODE_MODULATION             MODE_MODULATION_SVM3D
// ---------------------------------------------------

// ---------------------------------------------------
#define ADC_TRIGGER_ZERO            1
#define ADC_TRIGGER_PRD             2
#define ADC_TRIGGER_CMPB            3
#define ADC_TRIGGER_CMPA            4

// ---------------------------------------------------
#define ADC_TRIGGER_MODE            ADC_TRIGGER_ZERO
// ---------------------------------------------------

#if(BUILDLEVEL == LEVEL4)

// Define thong so PI cho tung muc dien ap Vac dat khac nhau khi khong tai
// ---------------------------------------------------
#define VAC_REF                 90
#define MODE_LOAD               64 // 64 OR NO_LOAD
#define VAC_LOAD_64             0 // 100 chay duoc
#define VAC_LOAD_64_GHEP_NOI    0 // 100 chay duoc
#define VAC_LOAD_32_GHEP_NOI    103 //
//-----------------------------------------------------

#endif

// Define toán học
#define PI 3.14159265358979
#define can2 1.414213562f
#define can3 1.732050808f
#define can6 2.449489743f

// Define đọc ADC
#define UDC_HCPL        AdcbResultRegs.ADCRESULT2

#define VaG_HCPL        AdcaResultRegs.ADCRESULT5
#define VbG_HCPL        AdcaResultRegs.ADCRESULT2
//#define VbG_HCPL        AdcaResultRegs.ADCRESULT3 //Vb_test

#define VcG_HCPL        AdcbResultRegs.ADCRESULT5

#define IA_INV_LEM      AdcaResultRegs.ADCRESULT1
#define IB_INV_LEM      AdcaResultRegs.ADCRESULT0
#define IC_INV_LEM      AdcaResultRegs.ADCRESULT4
#define IZ_INV_LEM      AdcbResultRegs.ADCRESULT4

#define T_Us             0.002 // Time sample voltage
#define Ti               0.00002 // Time sample current 0.0000154
#define T                0.00002             // time sample
#define T_Udc            0.0002              // time sample voltage

#define LEM_1(A)     (2048.0*A/10.0)

#if (BUILDLEVEL == LEVEL3)

    #define KP_CURR_LOOP            3.0 // L filter 330uH: 3.3
    #define KI_CURR_LOOP            1500.0 // L filter 330uH: 132000.0

    #define KP_CURR_LOOP_Z          3.0 // L filter 330uH: 13.2
    #define KI_CURR_LOOP_Z          1500.0 // L filter 330uH: 528000.0

#endif

#if(BUILDLEVEL == LEVEL4)

    #if(VAC_REF == 25) // ok van Sic
        #define KP_CURR_LOOP_1            2.0*2
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 35) // ok van Sic
        #define KP_CURR_LOOP_1            2.0*2.5
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 50) // ok van Sic
        #define KP_CURR_LOOP_1            2.0*3.5
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 75) // ok van Sic
        #define KP_CURR_LOOP_1            2.0*5.25
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 90 && MODE_LOAD == NO_LOAD) // ok van Si Vdc = 300V chay tai 64Ohm
        #define KP_CURR_LOOP_1            2.0*6.9
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 100) //ok van Sic
        #define KP_CURR_LOOP_1            2.0*6.9
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 110 && MODE_LOAD == 64) // ok van Si Vdc = 340V chay co tai 64
        #define KP_CURR_LOOP_1            2.0*11.0
        #define KI_CURR_LOOP_1            0.18/8.5
    #endif

    #if(VAC_REF == 110 && MODE_LOAD == NO_LOAD) // ok van Si Vdc = 350V chay khong tai
        #define KP_CURR_LOOP_1            2.0*6.9
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 115)
        #define KP_CURR_LOOP_1            2.0*6.9
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 130)
        #define KP_CURR_LOOP_1            2.0*8.8
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 150)
        #define KP_CURR_LOOP_1            2.0*2
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 175)
        #define KP_CURR_LOOP_1            2.0*2
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 200)
        #define KP_CURR_LOOP_1            2.0*2
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_REF == 220)
        #define KP_CURR_LOOP_1            2.0*2
        #define KI_CURR_LOOP_1            0.18/2
    #endif

    #if(VAC_LOAD_64 == 15 && ALLOW_CALIB_PI == 0) // Chay duoc tuy nhien ramp chua dep lam
        #define KP_CURR_LOOP_1            2.0
        #define KI_CURR_LOOP_1            200.0
    #endif

    #if(VAC_LOAD_64 == 80 && ALLOW_CALIB_PI == 0) // ok Vac dep
        #define KP_CURR_LOOP_1            2.0
        #define KI_CURR_LOOP_1            200.0
    #endif

    #if(VAC_LOAD_64 == 100 && ALLOW_CALIB_PI == 0) // ok Vac dep
        #define KP_CURR_LOOP_1            2.0
        #define KI_CURR_LOOP_1            200.0
    #endif

    #if(VAC_LOAD_64_GHEP_NOI == 100 && ALLOW_CALIB_PI == 0) //
        #define KP_CURR_LOOP_1            1.0
        #define KI_CURR_LOOP_1            200.0
    #endif

    #if(VAC_LOAD_32_GHEP_NOI == 100 && ALLOW_CALIB_PI == 0) //chay duoc nhung ra 110V va hoi xau
        #define KP_CURR_LOOP_1            1.0
        #define KI_CURR_LOOP_1            200.0
    #endif

    #if(VAC_LOAD_32_GHEP_NOI == 103 && ALLOW_CALIB_PI == 0) // Chay ok 1KW
        #define KP_CURR_LOOP_1            1.0
        #define KI_CURR_LOOP_1            200.0
    #endif

    #if(VAC_LOAD_32_GHEP_NOI == 92 && ALLOW_CALIB_PI == 1)
        extern volatile float KP_CURR_LOOP_1;
        extern volatile float KI_CURR_LOOP_1;
    #endif

    #define KP_VOLT_US_LOOP           0.00001
    #define KI_VOLT_US_LOOP           0.1

#endif

#if (BUILDLEVEL == LEVEL5)

#define KP_CURR_LOOP_PR        1 //2.953;
#define KI_CURR_LOOP_PR        1000; //6159.173;

#define KP_CURR_LOOP_PI         3.4
#define KI_CURR_LOOP_PI         800.0
#endif
#if (BUILDLEVEL == LEVEL6)
#define KP_CURR_LOOP_PR        1; //2.953;
#define KI_CURR_LOOP_PR        1000; //6159.173;

#define KP_voltage_LOOP_PR        0.2;
#define KI_voltage_LOOP_PR        200;
#endif

#if (BUILDLEVEL == LEVEL7)

    //#define KP_CURR_LOOP            1.15       // L filter 1.7mH: 17.0   --- L Filter 115uH: 1.15
    //#define KI_CURR_LOOP            800.0      // L filter 1.7mH: 800.0  --- L Filter 115uH: 800.0
    //
    #define KP_CURR_LOOP_Z          3.0       // L filter 1.7mH: 68.0   --- L Filter 115uH: 4.60
    #define KI_CURR_LOOP_Z          1500.0     // L filter 1.7mH: 3200.0 --- L Filter 115uH: 3200.0

    #define KP_CURR_LOOP            3.0       // L filter 1.7mH: 17.0   --- L Filter 115uH: 1.15
    #define KI_CURR_LOOP            1500.0      // L filter 1.7mH: 800.0  --- L Filter 115uH: 800.0

    //#define KP_CURR_LOOP_Z          4.60       // L filter 1.7mH: 68.0   --- L Filter 115uH: 4.60
    //#define KI_CURR_LOOP_Z          3200.0     // L filter 1.7mH: 3200.0 --- L Filter 115uH: 3200.0

    #define KP_PLL                  4.0
    #define KI_PLL                  100.0

#endif

#if (BUILDLEVEL == LEVEL8)

    #define KP_CURR_LOOP            3.0
    #define KI_CURR_LOOP            1500.0

    #define KP_CURR_LOOP_Z          3.0
    #define KI_CURR_LOOP_Z          1500.0

    #define KP_VOLT_UDC_LOOP        0.1
    #define KI_VOLT_UDC_LOOP        5.0

#endif
// Define the base quantities for PU system conversion
#define NORMAL_FREQ     50.0
#define BASE_FREQ       150.0               // Base electrical frequency (Hz)
#define Udc_max         800.0               // Max DC Voltage (V)
#define Us_max          400.0               // Max Phase Voltage (V)
#define Is_max          10.0               // Base Peak Phase Current (A)
#define Wref            (2.0*PI*NORMAL_FREQ)
#define Wmax            (2.0*PI*BASE_FREQ)

// CMPSS FLC Permission
#define CMPSS_PROTECT_UDC_UPPER         1 // Da test co the bao ve duoc, bv ok

#define CMPSS_PROTECT_VaG_UPPER         0 // Da test co the bao ve duoc
#define CMPSS_PROTECT_VaG_LOWER         0 // Da test co the bao ve duoc

#define CMPSS_PROTECT_VbG_UPPER         1 // Da test co the bao ve duoc, bv ok
#define CMPSS_PROTECT_VbG_LOWER         1 // Da test co the bao ve duoc, bv ok

#define CMPSS_PROTECT_VcG_UPPER         0 //
#define CMPSS_PROTECT_VcG_LOWER         0 //

#define CMPSS_PROTECT_Ia_inv_UPPER      0
#define CMPSS_PROTECT_Ia_inv_LOWER      0

#define CMPSS_PROTECT_Ib_inv_UPPER      0
#define CMPSS_PROTECT_Ib_inv_LOWER      0

#define CMPSS_PROTECT_Ic_inv_UPPER      1 // Da test co the bao ve duoc, bv ok
#define CMPSS_PROTECT_Ic_inv_LOWER      1 // Da test co the bao ve duoc, bv ok

// CMPSS FLC Setting
extern volatile float CMPSS_Udc_New_Protecion;

extern volatile float CMPSS_Udc_Offset_New_Protecion;
extern volatile float CMPSS_Vg_Offset_New_Protecion;

extern volatile float CMPSS_Ig_inv_New_Protecion;

// CMPSS TPC Setting

#endif /* _PV_SETTING_H_ */
