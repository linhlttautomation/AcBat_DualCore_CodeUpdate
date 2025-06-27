/*
 * CFDAB_Setting.h
 *
 *  Created on: Oct 22, 2024
 *      Author: 84339
 */

#ifndef _CFDAB_SETTING_H_
#define _CFDAB_SETTING_H_

// Define Level
#define LEVEL1          1           // Vong ho discharge
#define LEVEL5          5           // Vong kin discharge

#define BUILDLEVEL      LEVEL5

// Define Mode_Run
#define CFDAB_MODE      1
#define VFDAB_MODE      2

#define TPC_MODE_RUN    CFDAB_MODE


#define     NOT_ALLOW_SWITCH_BDK      0
#define     ALLOW_SWITCH_BDK          1

#define     SWITCH_BDK_MODE    ALLOW_SWITCH_BDK
//
#define PV_INVERTER_ADC 1                                           // 0 HIL , 1 = Exp
#define ADC_PU_SCALE_FACTOR_16BIT        0.0000152587890625         //1/2^12=0.000244140625
#define ADC_PU_SCALE_FACTOR_12BIT        0.000244140625             //1/2^12=0.000244140625
#define ADC_PU_PPB_SCALE_FACTOR          0.000488281250             //1/2^11
#define SD_PU_SCALE_FACTOR               0.000030517578125
#define Scale                            1.0

#define PI 3.14159265358979
#define DUTY_MAX        1.0

#define Udc_max         800.0
#define Ubat_max        200.0
#define Ibat_max        50.0
#define Uc_max          600.0

#define T                   0.00002                                 // time sample
//#define MEAUBAT(A) (A/Ubat_max*4096.0+43)

#if (PV_INVERTER_ADC == 0)
//
#endif
//
#if (PV_INVERTER_ADC == 1)

#define UDC_HCPL    AdcbResultRegs.ADCRESULT2
#define UBAT_HCPL   AdccResultRegs.ADCRESULT2
#define VC_HCPL     AdccResultRegs.ADCRESULT5
#define IHV_LEM     AdccResultRegs.ADCRESULT3
#define ILV_LEM     AdccResultRegs.ADCRESULT4
#endif

// LEM    1.0pu current ==> 50.0A -> 2048 counts
#define LEM_2(A)     2024-132+(2048.0*A/Ibat_max)
#define LEML_2(A)    2024+(2048.0*A/Ibat_max/1.1)-165
#define MEAUBAT(A)   (A/Ubat_max*4096.0)
#define MEAUDC(A)    (4096.0*A/Udc_max+10)
#define MEAUC(A)     (A/Uc_max*4096.0+7)

// Discharge Close_Loop
#if (BUILDLEVEL == LEVEL5)

// PRECIOUS PARAMETERS

//#define KP_VOLT_UDC_LOOP        0.0002
//#define KI_VOLT_UDC_LOOP        0.05
//#define KP_VOLT_VC_LOOP         0.000001
//#define KI_VOLT_VC_LOOP         0.00003
//#define KD_VOLT_VC_LOOP         0

//12V_Vbat

//#define KP_VOLT_UDC_LOOP        0.0002
//#define KI_VOLT_UDC_LOOP        0.05
//#define KP_VOLT_VC_LOOP         0.00001
//#define KI_VOLT_VC_LOOP         0.00005
//#define KD_VOLT_VC_LOOP         0

//14V_Vbat

//#define KP_VOLT_UDC_LOOP        0.00002
//#define KI_VOLT_UDC_LOOP        0.05
//#define KP_VOLT_VC_LOOP         0.000025
//#define KI_VOLT_VC_LOOP         0.00003
//#define KD_VOLT_VC_LOOP         0

////29V_Vbat_ source

//#define KP_VOLT_UDC_LOOP        0.00002
//#define KI_VOLT_UDC_LOOP        0.05
//#define KP_VOLT_VC_LOOP         0.000025
//#define KI_VOLT_VC_LOOP         0.00003
//#define KD_VOLT_VC_LOOP         0

//120V_Vbat

//#define KP_VOLT_UDC_LOOP        0.00002*2
//#define KI_VOLT_UDC_LOOP        0.05/6
//#define KP_VOLT_VC_LOOP         0.000025
//#define KI_VOLT_VC_LOOP         0.00003
//#define KD_VOLT_VC_LOOP         0

//#define KP_VOLT_UDC_LOOP          0.00002*2 //source
//#define KI_VOLT_UDC_LOOP          0.05/6 //source
//
//#define KP_VOLT_VC_LOOP           0.000025
//#define KI_VOLT_VC_LOOP           0.00003
//#define KD_VOLT_VC_LOOP           0

#define KP_VOLT_UDC_LOOP          0.00002*3.0 //source
#define KI_VOLT_UDC_LOOP          0.05/6.0 //source

#define KP_VOLT_VC_LOOP           0.0000265
#define KI_VOLT_VC_LOOP           0.0000285
#define KD_VOLT_VC_LOOP           0


#define KP_VOLT_UDC_LOOP_LOAD          0.00002*5.0  //source
#define KI_VOLT_UDC_LOOP_LOAD          0.05/12.0    //source

#define KP_VOLT_VC_LOOP_LOAD           0.0000265
#define KI_VOLT_VC_LOOP_LOAD           0.0000285
#define KD_VOLT_VC_LOOP_LOAD           0

//5V_Vbat

//#define KP_VOLT_UDC_LOOP        0.0002
//#define KI_VOLT_UDC_LOOP        0.0002
//#define KP_VOLT_VC_LOOP         0.000001
//#define KI_VOLT_VC_LOOP         0.00003
//#define KD_VOLT_VC_LOOP         0


//#define KP_VOLT_UDC_LOOP        0.00001   //_12_18   //_5v
//#define KP_VOLT_UDC_LOOP        0.0008 //10V

//#define KI_VOLT_UDC_LOOP        0.2860
//#define KI_VOLT_UDC_LOOP        0.01

//#define KI_VOLT_UDC_LOOP        0.0001  //_5v
//#define KI_VOLT_UDC_LOOP        0.008 //10V

//#define KP_VOLT_VC_LOOP         0.00001
//#define KP_VOLT_VC_LOOP         0.000005 _//10V

//#define KI_VOLT_VC_LOOP         0.000015

#endif

// SETTING CFDAB

#define CFDAB_Power                    2500
#define CFDAB_Voltage                  400
#define CFDAB_MaxCharge_Current        10
#define CFDAB_MaxDischarge_Current     10

#define CFDAB_UdcRef                    42
#define CFDAB_VcRef                     28

#define CFDAB_UbatRef                   5
#define CFDAB_IbatRef                   5.2

#define CFDAB_Udc_Max                   460
#define CFDAB_Udc_Min                   350

#define CFDAB_Vc_Max                    460
#define CFDAB_Vc_Min                    0

#define CFDAB_Ubat_Max                  160
#define CFDAB_Ubat_Min                  0

#define CFDAB_Ibat_Max                  10

// CMPSS TPC Permission
#define CMPSS_PROTECT_Ubat_UPPER        1

#define CMPSS_PROTECT_Ihv_UPPER         0

#define CMPSS_PROTECT_Ilv               1

#define CMPSS_PROTECT_Uclamp_UPPER      1

#define CMPSS_PROTECT_Udc_UPPER         1

#endif /* _CFDAB_SETTING_H_ */
