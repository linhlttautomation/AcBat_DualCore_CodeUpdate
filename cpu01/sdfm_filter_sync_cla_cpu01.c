/*
    Code Dual Core 2 CPU 2 CLA viet boi LUU TA TRUONG LINH K65
 */

//
// Included Files
//
#include "F28x_Project.h"
#include "cla_sdfm_filter_sync_shared.h"
#include "F2837xD_sdfm_drivers.h"
#include "F2837xD_struct.h"
#include "PV_Variables.h"
#include "F2837xD_GlobalPrototypes.h"
#include "PV_Setting.h"
#include "stdio.h"

//
// Defines
//
#define MAX_SAMPLES               1024
#define SDFM_PIN_MUX_OPTION1      1
#define SDFM_PIN_MUX_OPTION2      2
#define SDFM_PIN_MUX_OPTION3      3
#define WAITSTEP                  asm(" RPT #255 || NOP")

//
// Globals
//
Uint16 gPeripheralNumber;
//
// Function Prototypes
//
void Sdfm_configurePins(Uint16);
void Cla_initMemoryMap(void);
void CLA_initCpu1Cla(void);

// Khai bao cac bien share CPU --> CLA
extern volatile CPU_TO_CLA CpuToCLA;

// Khai bao cac bien share CPU --> CLA
extern volatile CLA_TO_CPU ClaToCPU;

static inline void dlog1(Uint16 value);
static inline void dlog2(Uint16 value);
static inline void dlog3(Uint16 value);

#define DLOG_SIZE_1 1000
Uint16 DataLog1[DLOG_SIZE_1];
#pragma DATA_SECTION(DataLog1, "DLOG");

#define DLOG_SIZE_2 1000
Uint16 DataLog2[DLOG_SIZE_2];
#pragma DATA_SECTION(DataLog2, "DLOG");

#define DLOG_SIZE_3 1000
Uint16 DataLog3[DLOG_SIZE_3];
#pragma DATA_SECTION(DataLog3, "DLOG");

Uint16 ndx1 = 0;
Uint16 ndx2 = 0;
Uint16 ndx3 = 0;

Uint16 Task1_Isr = 0;
Uint16 Task8_Isr = 0;

Uint16 START = 0;

Uint16 START_1 = 0;

typedef enum {
    OFF_NORMAL,
    ON_NORMAL
} START_FLC;

START_FLC b_START_FLC = OFF_NORMAL;

Uint16 ON_RELAY = 0;

PROTECT_CHANEL protect_chanel;

Uint16 wd_count = 0;  // Đếm số lần WDT ngắt

Uint16 ClrPrtFlg = 0;

Uint16 ClrPrtFLg_Fst = 0;

Uint16 RunTask8Flag = 0;

Uint16 FLC_RstFlg = 0;

/*volatile float CMPSS_Udc_New_Protecion = 480.0;
volatile float CMPSS_Udc_Offset_New_Protecion = 0.0;
volatile float CMPSS_Vg_Offset_New_Protecion = 224.0;
volatile float CMPSS_Ig_inv_New_Protecion = 8.0;*/ // Bao ve 300Vdc-92Vrms-Tai32Ohm-1kW

volatile float CMPSS_Udc_New_Protecion = 350.0;
volatile float CMPSS_Udc_Offset_New_Protecion = 0.0;
volatile float CMPSS_Vg_Offset_New_Protecion = 0.0;
volatile float CMPSS_Ig_inv_New_Protecion = 8.0; // Bao ve 320Vdc-103Vrms-Tai32Ohm-1kw

#if(ALLOW_CALIB_PI == 1)
    volatile float KP_CURR_LOOP_1 = 1.0;
    volatile float KI_CURR_LOOP_1 = 200.0;
#endif

volatile Uint32 seconds_counter_cmpss = 0;
volatile Uint32 CMPSS_Protect_Time = 0;

// CMPSS parameters for Over Current Protection FLC
Uint16  clkPrescale_1 = 6,
        sampwin_1     = 30,
        thresh_1      = 18,
        LEM_curHi_1   = LEM_1(40.0),
        LEM_curLo_1   = LEM_1(40.0);

void DelayUs(unsigned long us)
{
    unsigned long count;
    for(count = 0; count < (us * 200); count++) {
        __asm(" NOP");
    }
}

void DelayMs(unsigned long ms)
{
    unsigned long count = 0;
    for(count = 0; count < ms; count++)
    {
        DelayUs(1000);
    }
}

void DelayS(unsigned long s)
{
    unsigned long count = 0;
    for(count = 0; count < s; count++)
    {
        DelayMs(1000);
    }
}

void InitCpuTimer0(void)
{
    CpuTimer0Regs.TCR.bit.TSS = 1;           // Stop timer
    CpuTimer0Regs.PRD.all     = 200000000UL - 1;  // Set period for 1s (200MHz)
    CpuTimer0Regs.TPR.all     = 0;           // No prescaler
    CpuTimer0Regs.TPRH.all    = 0;

    CpuTimer0Regs.TCR.bit.TRB = 1;           // Reload timer
    CpuTimer0Regs.TCR.bit.TIE = 1;           // Enable timer interrupt
    CpuTimer0Regs.TCR.bit.TSS = 0;           // Start timer
}

__interrupt void Cpu_Timer0_ISR(void)
{
    if(START == 1 || b_START_FLC == ON_NORMAL)
    {
        seconds_counter_cmpss++;  // Ngat 1s
    }

    CpuTimer0Regs.TCR.bit.TIF = 1;          // Clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt
}

//void InitWatchdog(void)
//{
//    EALLOW;
//    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0x2; // Chia SYSCLKOUT cho 4 (nếu SYSCLKOUT = 200MHz, thì LSPCLK = 50MHz)
//    WdRegs.WDCR.all = 0x0028 | (0x5 << 3);  // Bật WDT, Prescaler = 512, ghi khóa bảo vệ
//    WdRegs.SCSR.bit.WDENINT = 1; // Chế độ ngắt, không reset hệ thống
//    EDIS;
//}
//
//void ResetWatchdog(void)
//{
//    EALLOW;
//    WdRegs.WDKEY.all = 0x0055;  // Ghi giá trị 0x55 trước
//    WdRegs.WDKEY.all = 0x00AA;  // Ghi giá trị 0xAA để reset bộ đếm
//    EDIS;
//}
//
//__interrupt void Watchdog_ISR(void)
//{
//    wd_count++;
//    START = 0;
//
//    EALLOW;
//    //WdRegs.SCSR.bit.WDINTS = 1; // Xóa cờ ngắt WDT
//    WdRegs.SCSR.all |= (1 << 3);  // Xóa cờ ngắt Watchdog
//    EDIS;
//
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}

void Init_ADC_A()
{
    Uint16 i;

    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    //AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    // Cấu hình độ phân giải và chế độ tín hiệu cho ADC A
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;  // 12-bit resolution
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;  // Single-ended mode

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for > 1ms to allow ADC time to power up
    //
    for(i = 0; i < 1000; i++)
    {
        asm("   RPT#255 || NOP");
    }
    EDIS;

    EALLOW;
    //
    //Select the channels to convert and end of conversion flag ADCA
    //
    // Ib_adc
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;          //SOC0 will convert pin A0 -> Ib_adc
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

    // Ia_adc
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;          //SOC1 will convert pin A1 -> Ia_adc
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

    // VbN
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;          //SOC2 will convert pin A2 -> VbN
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

//    AdcaRegs.ADCSOC6CTL.bit.CHSEL = 2;          //SOC2 will convert pin A2 -> VbN
//    AdcaRegs.ADCSOC6CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
//    AdcaRegs.ADCSOC6CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

    // VbN_test
//    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;          //SOC3 will convert pin A2 -> VbN
//    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
//    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

    // Ic_adc
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;          //SOC4 will convert pin A4 -> Iz_adc
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

    // VaN
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;          //SOC5 will convert pin A5 -> VaN
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

    // Trigger CLA
    AdcaRegs.ADCINTSOCSEL1.all = 0x0000;          // No ADCInterrupt will trigger SOCx
    AdcaRegs.ADCINTSOCSEL2.all = 0x0000;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;      // EOC1 is trigger for ADCINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable ADC interrupt 1
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;     // ADCINT1 pulses are generated whenever an EOC pulse is generated irrespective of whether the flag bit is cleared or not.
                                                // 0 No further ADCINT2 pulses are generated until ADCINT2 flag (in ADCINTFLG register) is cleared by user.
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //make sure INT1 flag is cleared
    EDIS;
}

// Init ADC B
void Init_ADC_B()
{
    Uint16 i;

    EALLOW;

    //
    //write configurations
    //
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    //AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    // Cấu hình độ phân giải và chế độ tín hiệu cho ADC B
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;  // 12-bit resolution
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;  // Single-ended mode
    //
    //Set pulse positions to late
    //
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for > 1ms to allow ADC time to power up
    //
    for(i = 0; i < 1000; i++)
    {
        asm("   RPT#255 || NOP");
    }
    EDIS;

    EALLOW;

    // Udc
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;          //SOC2 will convert pin B2 -> Udc
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

    //Iz_adc
    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 4;          //SOC4 will convert pin B4 -> Iz_inv
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM1 SOCA/C

    // VcN
    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 5;          //SOC5 will convert pin B5 -> VcN
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

//    // VbN_test
//    AdcbRegs.ADCSOC7CTL.bit.CHSEL = 3;          //SOC3 will convert pin B5 -> VcN
//    AdcbRegs.ADCSOC7CTL.bit.ACQPS = 19;         //sample window is 20 SYSCLK cycles
//    AdcbRegs.ADCSOC7CTL.bit.TRIGSEL = 0x0B;     //trigger on ePWM4 SOCA/C

    EDIS;
}

void CMPSS_Protection_FLC(void)
{
    EALLOW;

    // Cmpss Protect for TPC
    EPwmXbarRegs.TRIP5MUX0TO15CFG.all  = 0x0000;
    EPwmXbarRegs.TRIP5MUX16TO31CFG.all = 0x0000;
    EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX10  = 0;
    EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX12  = 0;
    EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX14  = 0;
    EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX8  = 0;

    EPwmXbarRegs.TRIP5MUXENABLE.all = 0x0000;
    EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX10  = 1;
    EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX12  = 1;
    EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX14  = 1;
    EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX8  = 1;

    #if(CMPSS_PROTECT_UDC_UPPER == 1)
        Cmpss3Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss3Regs.COMPCTL.bit.COMPHSOURCE = 0;
        Cmpss3Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss3Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // Udc Upper protection
        Cmpss3Regs.DACHVALS.bit.DACVAL = (CpuToCLA.ADCoffset_Udc + ((CMPSS_Udc_New_Protecion + CMPSS_Udc_Offset_New_Protecion)/800.0)*(4096.0 - CpuToCLA.ADCoffset_Udc)*1.0 + 0)/1.1;

        Cmpss3Regs.COMPCTL.bit.COMPHINV = 0;
        Cmpss3Regs.COMPCTL.bit.CTRIPHSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX4 = 0; // Cmpss3 trip H
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX4  = 1; // VDCH

        Cmpss3Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss3Regs.CTRIPHFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss3Regs.CTRIPHFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss3Regs.CTRIPHFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss3Regs.COMPSTSCLR.bit.HLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_VaG_UPPER == 1)
        Cmpss2Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss2Regs.COMPCTL.bit.COMPHSOURCE = 1;
        Cmpss2Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // VaG Upper protection
        Cmpss2Regs.DACHVALS.bit.DACVAL = (2598 + (((30.0/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - 2598) - 10)/1.1;
        Cmpss2Regs.COMPCTL.bit.COMPHINV = 0;
        Cmpss2Regs.COMPCTL.bit.CTRIPHSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2 = 0 ; // Cmpss2 trip H
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2 = 1; // VagH

        Cmpss2Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss2Regs.CTRIPHFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss2Regs.CTRIPHFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss2Regs.CTRIPHFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_VaG_LOWER == 1)
        Cmpss2Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss2Regs.COMPCTL.bit.COMPLSOURCE = 1;
        Cmpss2Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // VaG Lower protecion
        Cmpss2Regs.DACLVALS.bit.DACVAL = (2598 - (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - 2598) + 0)/1.1;
        Cmpss2Regs.COMPCTL.bit.COMPLINV = 1;
        Cmpss2Regs.COMPCTL.bit.CTRIPLSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3 = 0 ; // Cmpss2 trip L
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX3  = 1; // VagL

        Cmpss2Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss2Regs.CTRIPLFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss2Regs.CTRIPLFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss2Regs.CTRIPLFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_VbG_UPPER == 1)
        Cmpss1Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss1Regs.COMPCTL.bit.COMPHSOURCE = 0;
        Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // VbG Upper protection
        Cmpss1Regs.DACHVALS.bit.DACVAL = (CpuToCLA.ADCoffset_VbG + (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - CpuToCLA.ADCoffset_VbG) + 25)/1.1;

        Cmpss1Regs.COMPCTL.bit.COMPHINV = 0;
        Cmpss1Regs.COMPCTL.bit.CTRIPHSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0 = 0 ; // Cmpss1 trip H
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0  = 1; // VbgH

        Cmpss1Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss1Regs.CTRIPHFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss1Regs.CTRIPHFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss1Regs.CTRIPHFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss1Regs.COMPSTSCLR.bit.HLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_VbG_LOWER == 1)
        Cmpss1Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss1Regs.COMPCTL.bit.COMPLSOURCE = 0;
        Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // VbG Lower protecion
        Cmpss1Regs.DACLVALS.bit.DACVAL = (CpuToCLA.ADCoffset_VbG - (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - CpuToCLA.ADCoffset_VbG) + 30)/1.1;

        Cmpss1Regs.COMPCTL.bit.COMPLINV = 1;
        Cmpss1Regs.COMPCTL.bit.CTRIPLSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX1 = 0 ; // Cmpss1 trip L
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX1  = 1; // VbgL

        Cmpss1Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss1Regs.CTRIPLFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss1Regs.CTRIPLFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss1Regs.CTRIPLFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss1Regs.COMPSTSCLR.bit.LLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_VcG_UPPER == 1)
        Cmpss1Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss1Regs.COMPCTL.bit.COMPHSOURCE = 1;
        Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // VcG Upper protection
        Cmpss1Regs.DACHVALS.bit.DACVAL = (CpuToCLA.ADCoffset_VcG + (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - CpuToCLA.ADCoffset_VcG) + 25)/1.1;

        Cmpss1Regs.COMPCTL.bit.COMPHINV = 1;
        Cmpss1Regs.COMPCTL.bit.CTRIPHSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0 = 0 ; // Cmpss1 trip H
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0  = 1; // VcgH

        Cmpss1Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss1Regs.CTRIPHFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss1Regs.CTRIPHFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss1Regs.CTRIPHFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss1Regs.COMPSTSCLR.bit.HLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_VcG_LOWER == 1)
        Cmpss1Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss1Regs.COMPCTL.bit.COMPHSOURCE = 1;
        Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // VcG Lower protection
        Cmpss1Regs.DACLVALS.bit.DACVAL = (CpuToCLA.ADCoffset_VcG - (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - CpuToCLA.ADCoffset_VcG) - 1000)/1.1;

        Cmpss1Regs.COMPCTL.bit.COMPHINV = 1;
        Cmpss1Regs.COMPCTL.bit.CTRIPHSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0 = 0 ; // Cmpss1 trip H
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0  = 1; // VcgH

        Cmpss1Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss1Regs.CTRIPHFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss1Regs.CTRIPHFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss1Regs.CTRIPHFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss1Regs.COMPSTSCLR.bit.HLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_Ia_inv_UPPER == 1)
        Cmpss2Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss2Regs.COMPCTL.bit.COMPHSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // Ia Upper protection
        Cmpss2Regs.DACHVALS.bit.DACVAL = (2093 + (CMPSS_Ig_inv_New_Protecion/81.3)*2093 - 230 + 169)/1.1;
        Cmpss2Regs.COMPCTL.bit.COMPHINV = 0;
        Cmpss2Regs.COMPCTL.bit.CTRIPHSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2 = 0 ; // Cmpss2 trip H
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2  = 1; // IaH

        // High protect
        Cmpss2Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss2Regs.CTRIPHFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss2Regs.CTRIPHFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss2Regs.CTRIPHFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_Ia_inv_LOWER == 1)
        Cmpss2Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss2Regs.COMPCTL.bit.COMPLSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // Ia Lower protecion
        Cmpss2Regs.DACLVALS.bit.DACVAL = (2093 - (CMPSS_Ig_inv_New_Protecion/81.3)*2093 - 230 - 169)/1.1;
        Cmpss2Regs.COMPCTL.bit.COMPLINV = 1;
        Cmpss2Regs.COMPCTL.bit.CTRIPLSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3 = 0; // Cmpss2 trip L
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX3  = 1; // IaL

        Cmpss2Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss2Regs.CTRIPLFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss2Regs.CTRIPLFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss2Regs.CTRIPLFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_Ib_inv_UPPER == 1)
        Cmpss4Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss4Regs.COMPCTL.bit.COMPHSOURCE = 0;
        Cmpss4Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss4Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // Ib Upper protection
        Cmpss4Regs.DACHVALS.bit.DACVAL = (2105 + (CMPSS_Ig_inv_New_Protecion/81.3)*2105 - 240 + 180)/1.1;
        Cmpss4Regs.COMPCTL.bit.COMPHINV = 0;
        Cmpss4Regs.COMPCTL.bit.CTRIPHSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX6 = 0 ; // Cmpss4 trip H
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX6  = 1; // IbH

        // High protect
        Cmpss4Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss4Regs.CTRIPHFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss4Regs.CTRIPHFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss4Regs.CTRIPHFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss4Regs.COMPSTSCLR.bit.HLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_Ib_inv_LOWER == 1)
        Cmpss4Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss4Regs.COMPCTL.bit.COMPLSOURCE = 0;
        Cmpss4Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss4Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // Ib Lower protecion
        Cmpss4Regs.DACLVALS.bit.DACVAL = (2105 - (CMPSS_Ig_inv_New_Protecion/81.3)*2105 - 232 - 180)/1.1 ;
        Cmpss4Regs.COMPCTL.bit.COMPLINV = 1;
        Cmpss4Regs.COMPCTL.bit.CTRIPLSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX7 = 0 ; // Cmpss4 trip L
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX7  = 1; // IbL

        Cmpss4Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss4Regs.CTRIPLFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss4Regs.CTRIPLFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss4Regs.CTRIPLFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss4Regs.COMPSTSCLR.bit.LLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_Ic_inv_UPPER == 1)
        Cmpss2Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss2Regs.COMPCTL.bit.COMPHSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // Ic Upper protection
        Cmpss2Regs.DACHVALS.bit.DACVAL = (CpuToCLA.ADCoffset_Ic_inv + (CMPSS_Ig_inv_New_Protecion/10.0)*CpuToCLA.ADCoffset_Ic_inv - 51)/1.1;
        Cmpss2Regs.COMPCTL.bit.COMPHINV = 0;
        Cmpss2Regs.COMPCTL.bit.CTRIPHSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2 = 0 ; // Cmpss2 trip H
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2  = 1; // IcH

        // High protect
        Cmpss2Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss2Regs.CTRIPHFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss2Regs.CTRIPHFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss2Regs.CTRIPHFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------
    #if(CMPSS_PROTECT_Ic_inv_LOWER == 1)
        Cmpss2Regs.COMPCTL.bit.COMPDACE = 1;
        Cmpss2Regs.COMPCTL.bit.COMPLSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.DACSOURCE = 0;
        Cmpss2Regs.COMPDACCTL.bit.SWLOADSEL = 0;

        // Ic Lower protecion
        Cmpss2Regs.DACLVALS.bit.DACVAL = (CpuToCLA.ADCoffset_Ic_inv - (CMPSS_Ig_inv_New_Protecion/10.0)*CpuToCLA.ADCoffset_Ic_inv + 35)/1.1;
        Cmpss2Regs.COMPCTL.bit.COMPLINV = 1;
        Cmpss2Regs.COMPCTL.bit.CTRIPLSEL = 2;

        EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3 = 0 ; // Cmpss2 trip L
        EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX3  = 1; // IcL

        Cmpss2Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale_1; // Set time between samples, max : 1023
        Cmpss2Regs.CTRIPLFILCTL.bit.SAMPWIN        = sampwin_1; // # Of samples in window, max : 31
        Cmpss2Regs.CTRIPLFILCTL.bit.THRESH         = thresh_1; // Recommended : thresh > sampwin/2
        Cmpss2Regs.CTRIPLFILCTL.bit.FILINIT        = 1; // Init samples to filter input value
        Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR = 1; // Clear the status register for latched comparator events
    #endif
    //-----------------------------------------------------

    // DC Trip select
    EPwm4Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3 ; // Tripin4
    EPwm4Regs.TZDCSEL.bit.DCAEVT1 = 4 ; // DCAL high , DCAH don't care
    EPwm4Regs.DCTRIPSEL.bit.DCALCOMPSEL = 3 ; // Tripin4

    EPwm5Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3 ; // Tripin4
    EPwm5Regs.TZDCSEL.bit.DCAEVT1 = 4 ; // DCAL high , DCAH don't care
    EPwm5Regs.DCTRIPSEL.bit.DCALCOMPSEL = 3 ; // Tripin4

    EPwm6Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3 ; // Tripin4
    EPwm6Regs.TZDCSEL.bit.DCAEVT1 = 4 ; // DCAL high , DCAH don't care
    EPwm6Regs.DCTRIPSEL.bit.DCALCOMPSEL = 3 ; // Tripin4

    EPwm8Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3 ; // Tripin4
    EPwm8Regs.TZDCSEL.bit.DCAEVT1 = 4 ; // DCAL high , DCAH don't care
    EPwm8Regs.DCTRIPSEL.bit.DCALCOMPSEL = 3 ; // Tripin4

    // Tripzone Select
    EPwm4Regs.TZSEL.bit.DCAEVT1 = 1;
    EPwm5Regs.TZSEL.bit.DCAEVT1 = 1;
    EPwm6Regs.TZSEL.bit.DCAEVT1 = 1;
    EPwm8Regs.TZSEL.bit.DCAEVT1 = 1;

    EPwm4Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO; // EPWMxA will go low
    EPwm4Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_LO; // EPWMxB will go low

    EPwm5Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO; // EPWMxA will go low
    EPwm5Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_LO; // EPWMxB will go low

    EPwm6Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO; // EPWMxA will go low
    EPwm6Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_LO; // EPWMxB will go low

    EPwm8Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO; // EPWMxA will go low
    EPwm8Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_LO; // EPWMxB will go low

    // Clear any spurious OV trip
    EPwm4Regs.TZCLR.bit.DCAEVT1 = 1;
    EPwm5Regs.TZCLR.bit.DCAEVT1 = 1;
    EPwm6Regs.TZCLR.bit.DCAEVT1 = 1;
    EPwm8Regs.TZCLR.bit.DCAEVT1 = 1;

    EDIS;
}

void ClearProtectFlagFcn(void)
{
    EALLOW;

    // Clear CMPSS1 High Latched Fault and Low Latched Fault for VbG
    Cmpss1Regs.COMPSTSCLR.bit.HLATCHCLR  = 1;  // Clear latched status
    Cmpss1Regs.COMPSTSCLR.bit.LLATCHCLR  = 1;  // Clear latched status

    // Clear CMPSS2 High Latched Fault and Low Latched Fault for Ic_inv
    Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR  = 1;  // Clear latched status
    Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR  = 1;  // Clear latched status

    // Clear CMPSS3 High Latched Fault for Udc
    Cmpss3Regs.COMPSTSCLR.bit.HLATCHCLR  = 1;  // Clear latched status

    // Clear EPWM4 Trip Zone flags
    EPwm4Regs.TZCLR.bit.OST    = 1;    // One-shot trip clear
    EPwm4Regs.TZCLR.bit.CBC    = 1;    // Cycle-by-cycle trip clear
    EPwm4Regs.TZCLR.bit.INT    = 1;    // Interrupt clear
    EPwm4Regs.TZCLR.bit.DCAEVT1 = 1;   // Digital Compare A Event 1 clear

    // Clear EPWM5 Trip Zone flags
    EPwm5Regs.TZCLR.bit.OST    = 1;
    EPwm5Regs.TZCLR.bit.CBC    = 1;
    EPwm5Regs.TZCLR.bit.INT    = 1;
    EPwm5Regs.TZCLR.bit.DCAEVT1 = 1;

    // Clear EPWM6 Trip Zone flags
    EPwm6Regs.TZCLR.bit.OST    = 1;
    EPwm6Regs.TZCLR.bit.CBC    = 1;
    EPwm6Regs.TZCLR.bit.INT    = 1;
    EPwm6Regs.TZCLR.bit.DCAEVT1 = 1;

    // Clear EPWM8 Trip Zone flags
    EPwm8Regs.TZCLR.bit.OST    = 1;
    EPwm8Regs.TZCLR.bit.CBC    = 1;
    EPwm8Regs.TZCLR.bit.INT    = 1;
    EPwm8Regs.TZCLR.bit.DCAEVT1 = 1;

    EPwm4Regs.TZCLR.all = 0xFFFF;
    EPwm5Regs.TZCLR.all = 0xFFFF;
    EPwm6Regs.TZCLR.all = 0xFFFF;
    EPwm8Regs.TZCLR.all = 0xFFFF;

    EDIS;
}

void UpdateProtectValue(void)
{
    EALLOW;

    #if (CMPSS_PROTECT_UDC_UPPER == 1)
        Cmpss3Regs.DACHVALS.bit.DACVAL = (4 + ((CMPSS_Udc_New_Protecion + CMPSS_Udc_Offset_New_Protecion)/800.0)*(4096.0 - 4)*1.0 + 0)/1.1;
    #endif

    #if (CMPSS_PROTECT_VbG_UPPER == 1)
        Cmpss1Regs.DACHVALS.bit.DACVAL = (2646 + (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - 2646) + 25)/1.1;
    #endif

    #if (CMPSS_PROTECT_VbG_LOWER == 1)
        Cmpss1Regs.DACLVALS.bit.DACVAL = (2646 - (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - 2646) + 30)/1.1;
    #endif

//    #if (CMPSS_PROTECT_VcG_UPPER == 1)
//        Cmpss1Regs.DACHVALS.bit.DACVAL = (2695 + (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - 2695) + 25)/1.1;
//    #endif
//
//    #if (CMPSS_PROTECT_VcG_LOWER == 1)
//        Cmpss1Regs.DACLVALS.bit.DACVAL = (2695 - (((CMPSS_Udc_New_Protecion/can3)+ CMPSS_Vg_Offset_New_Protecion)/400.0)*(4096.0 - 2695) + 30)/1.1;
//    #endif

    #if (CMPSS_PROTECT_Ic_inv_UPPER == 1)
    Cmpss2Regs.DACHVALS.bit.DACVAL = (2070 + (CMPSS_Ig_inv_New_Protecion/10.0)*2070 - 51)/1.1;
#endif

    #if (CMPSS_PROTECT_Ic_inv_LOWER == 1)
    Cmpss2Regs.DACLVALS.bit.DACVAL = (2070 - (CMPSS_Ig_inv_New_Protecion/10.0)*2070 + 35)/1.1;
#endif

    EDIS;
}

// Main
//
int main(void)
{
    Uint16  pinMuxoption;
    Uint16  HLT, LLT;

    int period = 1000;

    #if(SET_MODE_RUN == THREE_PHASE_MODE)
        int deadtime = 60;
    #endif

    #if(SET_MODE_RUN == SINGLE_PHASE_MODE)
        int deadtime = 40;
    #endif

    InitSysCtrl();

    #if(ALLOW_IPC_CPU == 1)

        InitIpc();
        // Reset trạng thái IPC trước khi bắt đầu
        IpcRegs.IPCACK.all = 0xFFFFFFFF;   // Xóa tất cả cờ IPC
        IpcRegs.IPCSET.all = 0;            // Đảm bảo không còn cờ set treo

    #endif

    for(ndx1=0; ndx1<DLOG_SIZE_1; ndx1++)
    {
        DataLog1[ndx1] = 0;
    }
    ndx1 = 0;

    for(ndx2=0; ndx2<DLOG_SIZE_2; ndx2++)
    {
        DataLog2[ndx2] = 0;
    }
    ndx2 = 0;

    for(ndx3=0; ndx3<DLOG_SIZE_3; ndx3++)
    {
        DataLog3[ndx3] = 0;
    }
    ndx3 = 0;

    Init_ADC_A();
    Init_ADC_B();

    EALLOW;

    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;

    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;

    CpuSysRegs.PCLKCR14.bit.CMPSS1 = 1;
    CpuSysRegs.PCLKCR14.bit.CMPSS2 = 1;
    CpuSysRegs.PCLKCR14.bit.CMPSS3 = 1;
    CpuSysRegs.PCLKCR14.bit.CMPSS4 = 1;

    //CpuSysRegs.PCLKCR0.bit.WD = 1;  // Cấp clock cho Watchdog Timer

    EDIS;

    EALLOW;

    // Cấp quyền truy cập ePWM cho CPU2
    DevCfgRegs.CPUSEL0.bit.EPWM1 = 1; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL0.bit.EPWM2 = 1; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL0.bit.EPWM3 = 1; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL0.bit.EPWM10 = 1; // 1: CPU2, 0: CPU1

    DevCfgRegs.CPUSEL11.bit.ADC_A = 0; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL11.bit.ADC_B = 0; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL11.bit.ADC_C = 1; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL11.bit.ADC_D = 1; // 1: CPU2, 0: CPU1

    DevCfgRegs.CPUSEL12.bit.CMPSS1 = 0; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL12.bit.CMPSS2 = 0; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL12.bit.CMPSS3 = 0; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL12.bit.CMPSS4 = 0; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL12.bit.CMPSS5 = 1; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL12.bit.CMPSS6 = 1; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL12.bit.CMPSS7 = 1; // 1: CPU2, 0: CPU1
    DevCfgRegs.CPUSEL12.bit.CMPSS8 = 1; // 1: CPU2, 0: CPU1
    // Đóng khóa

    EDIS;

    EALLOW;

//  GPIO-00 - PIN FUNCTION = PWM1A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;    // if GMUX = 0: 0=GPIO, 1= EPWM1A , 2=Resv , 3=Resv
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     // 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;      // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;

    //--------------------------------------------------------------------------------------
//  GPIO-01 - PIN FUNCTION = PWM1B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;    // if GMUX = 0: 0=GPIO, 1= EPWM1A , 2=Resv , 3=Resv
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     // 0=GPIO,  1=EPWM1B,  2=Resv,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;      // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;

    //--------------------------------------------------------------------------------------
//  GPIO-02 - PIN FUNCTION = PWM2A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;    // if GMUX = 0: 0=GPIO, 1= EPWM2A , 2=Resv , 3=Resv
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     // 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;      // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-03 - PIN FUNCTION = PWM2B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;     // 0=GPIO,  1=EPWM2B,  2=SPISOMI-A,  3=COMP2OUT
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-04 - PIN FUNCTION = PWM3A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;     // 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;      // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-05 - PIN FUNCTION = PWM3B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;     // 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;      // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-06 - PIN FUNCTION = PWM4A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;   //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;    // 2=CANTXB,
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;     // 1=OUTput
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-07 - PIN FUNCTION = PWM4B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;   //
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;    // 2=CANXB,
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;     //  0 = INput, 1 = Output
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-8 - PIN FUNCTION = PWM 5A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;   //
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;    // 0=GPIO,
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-9 - PIN FUNCTION = PWM 5B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;   //
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;    // 0=GPIO,
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-10 - PIN FUNCTION = PWM 6A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;      //
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;    //
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-11 - PIN FUNCTION = PWM 6B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;      //
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-12 - PIN FUNCTION = PWM 7A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;      //
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-13 - PIN FUNCTION = PWM 7B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;      //
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-14 - PIN FUNCTION = PWM 8A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;      //
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-15 - PIN FUNCTION = PWM 8B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;      //
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-16 - PIN FUNCTION = PWM 9A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0;      //
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;     // 1=OUTput,  0=INput
//--------------------------------------------------------------------------------------
//  GPIO-17 - PIN FUNCTION = PWM 9B
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0;      //
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;     // 1=OUTput,  0=INput
//--------------------------------------------------------------------------------------
//  GPIO-18 - PIN FUNCTION = PWM 10A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 1;      //
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;

//--------------------------------------------------------------------------------------
//  GPIO-19 - PIN FUNCTION = PWM 10B
    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 1;      //
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;

//--------------------------------------------------------------------------------------
//    //  GPIO-20 - PIN FUNCTION = PWM 11A
//    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 1;      //
//    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;
//    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;     // 1=OUTput,  0=INput
//    //--------------------------------------------------------------------------------------
//    //  GPIO-21 - PIN FUNCTION = PWM 11B
//    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 1;      //
//    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;
//    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;     // 1=OUTput,  0=INput
//    //--------------------------------------------------------------------------------------
//    //  GPIO-22 - PIN FUNCTION = PWM 12A
//    GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = 1;      //
//    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;
//    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;     // 1=OUTput,  0=INput
//    //--------------------------------------------------------------------------------------
//    //  GPIO-23 - PIN FUNCTION = PWM 12B
//    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 1;      //
//    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;
//    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;     // 1=OUTput,  0=INput

    // Cấu hình GPIO27 làm output cho Relay 1
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;  // Chọn chức năng GPIO cho chân GPIO32
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;   // Cấu hình GPIO32 làm output
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // Khởi tạo ở mức thấp (relay tắt)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;

    // Cấu hình GPIO25 làm output cho Relay 2
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;  // Chọn chức năng GPIO cho chân GPIO32
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;   // Cấu hình GPIO32 làm output
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1; // Khởi tạo ở mức thấp (relay tắt)
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;

    GpioCtrlRegs.GPCGMUX1.bit.GPIO73 = 0;      //
    GpioCtrlRegs.GPCDIR.bit.GPIO73 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3;  // Chọn chế độ XCLKOUT cho GPIO73
    EDIS;

    #if(ALLOW_BUTTON == 1)
        EALLOW;
        // Cấu hình GPIO32 làm đầu vào
        GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
        GpioCtrlRegs.GPBPUD.bit.GPIO32 = 1;   // Tắt pull-up (nếu có điện trở ngoài)
        GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;   // Cấu hình làm input
        GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
        EDIS;
    #endif

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

#if(SET_MODE_RUN == THREE_PHASE_MODE)
    EALLOW;

    /* Time-Base Control (TBCTL) */
    EPwm4Regs.TBCTL.bit.FREE_SOFT = 3;
    EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.SWFSYNC = 0;
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm4Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    EPwm6Regs.TBCTL.bit.FREE_SOFT = 3;
    EPwm6Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm6Regs.TBCTL.bit.SWFSYNC = 0;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm6Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    EPwm5Regs.TBCTL.bit.FREE_SOFT = 3;
    EPwm5Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm5Regs.TBCTL.bit.SWFSYNC = 0;
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm5Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    EPwm8Regs.TBCTL.bit.FREE_SOFT = 3;
    EPwm8Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm8Regs.TBCTL.bit.SWFSYNC = 0;
    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm8Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    /* Initialization */
    EPwm4Regs.CMPA.bit.CMPA = period;
    EPwm4Regs.TBPHS.bit.TBPHS = 0;
    EPwm4Regs.TBCTR = 0;
    EPwm4Regs.TBPRD = period;

    EPwm6Regs.CMPA.bit.CMPA = period;
    EPwm6Regs.TBPHS.bit.TBPHS = 0;
    EPwm6Regs.TBCTR = 0;
    EPwm6Regs.TBPRD = period;

    EPwm5Regs.CMPA.bit.CMPA = period;
    EPwm5Regs.TBPHS.bit.TBPHS = 0;
    EPwm5Regs.TBCTR = 0;
    EPwm5Regs.TBPRD = period;

    EPwm8Regs.CMPA.bit.CMPA = period;
    EPwm8Regs.TBPHS.bit.TBPHS = 0;
    EPwm8Regs.TBCTR = 0;
    EPwm8Regs.TBPRD = period;

    /* Counter-Compare (CC) */
    EPwm4Regs.CMPCTL.all = 0x000C;
    EPwm6Regs.CMPCTL.all = 0x000C;
    EPwm5Regs.CMPCTL.all = 0x000C;
    EPwm8Regs.CMPCTL.all = 0x000C;

    /* Action-Qualifier (AQ) */
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;

    /* Dead Band (DB) */
    EPwm4Regs.DBCTL.all = 0x03CB;
    EPwm4Regs.DBFED.bit.DBFED = deadtime;
    EPwm4Regs.DBRED.bit.DBRED = deadtime;

    EPwm6Regs.DBCTL.all = 0x03CB;
    EPwm6Regs.DBFED.bit.DBFED = deadtime;
    EPwm6Regs.DBRED.bit.DBRED = deadtime;

    EPwm5Regs.DBCTL.all = 0x03CB;
    EPwm5Regs.DBFED.bit.DBFED = deadtime;
    EPwm5Regs.DBRED.bit.DBRED = deadtime;

    EPwm8Regs.DBCTL.all = 0x03CB;
    EPwm8Regs.DBFED.bit.DBFED = deadtime;
    EPwm8Regs.DBRED.bit.DBRED = deadtime;


    /* Event Trigger (ET) */

    #if(ADC_TRIGGER_MODE == ADC_TRIGGER_CMPB)
        EPwm4Regs.CMPB.bit.CMPB = period/2;
    #endif

    EPwm4Regs.ETSEL.bit.SOCAEN = 1;

    #if(ADC_TRIGGER_MODE == ADC_TRIGGER_ZERO)
        EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    #elif(ADC_TRIGGER_MODE == ADC_TRIGGER_PRD)
        EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;
    #elif(ADC_TRIGGER_MODE == ADC_TRIGGER_CMPB)
        EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPB; // or ET_CTRD_CMPB
    #elif(ADC_TRIGGER_MODE == ADC_TRIGGER_CMPA)
        EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPA;  // ET_CTRD_CMPA
    #endif

    EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;                // Generate pulse on 2nd event
    EPwm4Regs.ETCLR.bit.SOCA = 1;
    EPwm4Regs.ETPS.bit.SOCACNT = ET_1ST;

    // Enable CNT_zero interrupt using EPWM4 Time-base
    EPwm4Regs.ETSEL.bit.INTEN = 1;                      // enable EPWM4INT generation
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;        // enable interrupt CNT_zero event
    EPwm4Regs.ETPS.bit.INTPRD = ET_1ST;                 // generate interrupt on the 2nd event
    EPwm4Regs.ETPS.bit.INTCNT = ET_1ST;
    EPwm4Regs.ETCLR.bit.INT = 1;                        // enable more interrupts

    EDIS;

    #endif

    #if(SET_MODE_RUN == SINGLE_PHASE_MODE)
        EALLOW;

           /* Time-Base Control (TBCTL) */
           EPwm4Regs.TBCTL.bit.FREE_SOFT = 3;
           EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN;
           EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
           EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
           EPwm4Regs.TBCTL.bit.SWFSYNC = 0;
           EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
           EPwm4Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
           EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;
           EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

           EPwm6Regs.TBCTL.bit.FREE_SOFT = 3;
           EPwm6Regs.TBCTL.bit.PHSDIR = TB_DOWN;
           EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;
           EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
           EPwm6Regs.TBCTL.bit.SWFSYNC = 0;
           EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
           EPwm6Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
           EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;
           EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

           EPwm5Regs.TBCTL.bit.FREE_SOFT = 3;
           EPwm5Regs.TBCTL.bit.PHSDIR = TB_DOWN;
           EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
           EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
           EPwm5Regs.TBCTL.bit.SWFSYNC = 0;
           EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
           EPwm5Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
           EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;
           EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

           EPwm7Regs.TBCTL.bit.FREE_SOFT = 3;
           EPwm7Regs.TBCTL.bit.PHSDIR = TB_DOWN;
           EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;
           EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
           EPwm7Regs.TBCTL.bit.SWFSYNC = 0;
           EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
           EPwm7Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
           EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;
           EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

           EPwm8Regs.TBCTL.bit.FREE_SOFT = 3;
           EPwm8Regs.TBCTL.bit.PHSDIR = TB_DOWN;
           EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;
           EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
           EPwm8Regs.TBCTL.bit.SWFSYNC = 0;
           EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
           EPwm8Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
           EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;
           EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

           /* Initialization */
           EPwm4Regs.CMPA.bit.CMPA = period;
           EPwm4Regs.TBPHS.bit.TBPHS = 0;
           EPwm4Regs.TBCTR = 0;
           EPwm4Regs.TBPRD = period;

           EPwm6Regs.CMPA.bit.CMPA = period;
           EPwm6Regs.TBPHS.bit.TBPHS = 0;
           EPwm6Regs.TBCTR = 0;
           EPwm6Regs.TBPRD = period;

           EPwm5Regs.CMPA.bit.CMPA = period;
           EPwm5Regs.TBPHS.bit.TBPHS = period;
           EPwm5Regs.TBCTR = 0;
           EPwm5Regs.TBPRD = period;

           EPwm8Regs.CMPA.bit.CMPA = period;
           EPwm8Regs.TBPHS.bit.TBPHS = period;
           EPwm8Regs.TBCTR = 0;
           EPwm8Regs.TBPRD = period;

           /* Counter-Compare (CC) */
           EPwm4Regs.CMPCTL.all = 0x000C;
           EPwm6Regs.CMPCTL.all = 0x000C;
           EPwm5Regs.CMPCTL.all = 0x000C;
           EPwm8Regs.CMPCTL.all = 0x000C;

           EPwm4Regs.CMPCTL.bit.LOADAMODE = 0x01;
           EPwm6Regs.CMPCTL.bit.LOADAMODE = 0x01;
           EPwm5Regs.CMPCTL.bit.LOADAMODE = 0x01;
           EPwm8Regs.CMPCTL.bit.LOADAMODE = 0x01;

           /* Action-Qualifier (AQ) */
           EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;
           EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;

           EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;
           EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;

           EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;
           EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;

           EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;
           EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR;

           /* Dead Band (DB) */
           EPwm4Regs.DBCTL.all = 0x03CB;
           EPwm4Regs.DBFED.bit.DBFED = deadtime;
           EPwm4Regs.DBRED.bit.DBRED = deadtime;

           EPwm6Regs.DBCTL.all = 0x03CB;
           EPwm6Regs.DBFED.bit.DBFED = deadtime;
           EPwm6Regs.DBRED.bit.DBRED = deadtime;

           EPwm5Regs.DBCTL.all = 0x03CB;
           EPwm5Regs.DBFED.bit.DBFED = deadtime;
           EPwm5Regs.DBRED.bit.DBRED = deadtime;

           EPwm8Regs.DBCTL.all = 0x03CB;
           EPwm8Regs.DBFED.bit.DBFED = deadtime;
           EPwm8Regs.DBRED.bit.DBRED = deadtime;

           /* Event Trigger (ET) */
           EPwm4Regs.ETSEL.bit.SOCAEN = 1;
           EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;           // CTR = 0
           EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;                // Generate pulse on 2nd event
           EPwm4Regs.ETCLR.bit.SOCA = 1;
           EPwm4Regs.ETPS.bit.SOCACNT = ET_1ST;

           // Enable CNT_zero interrupt using EPWM1 Time-base
           EPwm4Regs.ETSEL.bit.INTEN = 1;                      // enable EPWM1INT generation
           EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;            // enable interrupt CNT_zero event
           EPwm4Regs.ETPS.bit.INTPRD = ET_1ST;                 // generate interrupt on the 2nd event
           EPwm4Regs.ETPS.bit.INTCNT = ET_1ST;
           EPwm4Regs.ETCLR.bit.INT = 1;                        // enable more interrupts

           EDIS;
    #endif

    CpuToCLA.ADCoffset_Udc = 4;  //
    CpuToCLA.ADCoffset_VaG = 2711; //
    CpuToCLA.ADCoffset_VbG = 2646; //
    CpuToCLA.ADCoffset_VcG = 2695; //
    CpuToCLA.ADCoffset_Ia_inv = 2062; //
    CpuToCLA.ADCoffset_Ib_inv = 2030; //
    CpuToCLA.ADCoffset_Ic_inv = 2070; //

    CpuToCLA.ADCgain_Udc = 1.0;  //
    CpuToCLA.ADCgain_VaG = 0.75999999;
    CpuToCLA.ADCgain_VbG = 0.850000024; //
    CpuToCLA.ADCgain_VcG = 0.75; //
    CpuToCLA.ADCgain_Ia_inv = 1.51999998; //
    CpuToCLA.ADCgain_Ib_inv = 1.50999999; //
    CpuToCLA.ADCgain_Ic_inv = 1.47899997; //

    CMPSS_Protection_FLC();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    InitCpuTimer0();

// Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts
//
    DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
   InitPieCtrl();

//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_SysCtrl.c.
// This function is found in F2837xD_SysCtrl.c.
//
   InitPieVectTable();

//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
   EALLOW;
//   PieVectTable.SD1_INT = &Sdfm1_ISR;
//   PieVectTable.SD2_INT = &Sdfm2_ISR;
   EDIS;

    #if(ALLOW_WATCHDOG_TIMER == 1)
       // 2. Cấu hình ngắt Watchdog
       EALLOW;
       PieVectTable.WAKE_INT = &Watchdog_ISR;  // Gán hàm xử lý ngắt Watchdog
       EDIS;
    #endif

   EALLOW;
//
// Enable CPU INT5 which is connected to SDFM INT
//
   IER |= M_INT11;

    #if(ALLOW_TIMER0 == 1)
       EALLOW;
       PieVectTable.TIMER0_INT = &Cpu_Timer0_ISR;  // ISR for Timer0
       EDIS;
    #endif

    #if(ALLOW_WATCHDOG_TIMER == 1)
       PieCtrlRegs.PIECTRL.bit.ENPIE = 1;     // Bật PIE
       PieCtrlRegs.PIEIER1.bit.INTx8 = 1;     // Cho phép ngắt Watchdog (WAKEINT)
       IER |= M_INT1;
    #endif

    #if(ALLOW_TIMER0 == 1)
        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // Timer0 interrupt = Group 1, INT7
        IER |= M_INT1;                      // Enable group 1 interrupt
        EINT;                               // Enable global interrupt
    #endif

//
// Enable SDFM INTn in the PIE: Group 5 __interrupt 9-10
//
   PieCtrlRegs.PIEIER5.bit.INTx9 = 1;  // SDFM1 interrupt enabled
   PieCtrlRegs.PIEIER5.bit.INTx10 = 1; // SDFM2 interrupt enabled
   EINT;

#ifdef CPU1
    pinMuxoption = SDFM_PIN_MUX_OPTION1;

//
// Configure GPIO pins as SDFM pins
//
    //Sdfm_configurePins(pinMuxoption);
    EALLOW;
    CPU1_CLA1(ENABLE);    //Enable CPU1.CLA module
    CPU2_CLA1(ENABLE);    //Enable CPU2.CLA module

    CONNECT_SD1(TO_CPU1);         //Connect SDFM1 to CPU1
    CONNECT_SD2(TO_CPU1);         //Connect SDFM2 to CPU1
    VBUS32_1(CONNECT_TO_CLA1);    //Connect VBUS32_1 (SDFM bus) to CPU1
    EDIS;
#endif

//
// Configure the CLA memory spaces
//
    Cla_initMemoryMap();

#ifdef CPU1
//
// Configure the CLA task vectors for CPU1
//
    CLA_initCpu1Cla();
#endif
#ifdef CPU2
//
// Configure the CLA task vectors for CPU2
//
    CLA_initCpu2Cla();
#endif

    Cla1ForceTask8andWait();
    WAITSTEP;

    EALLOW;

// Trigger Source for TASK1 of CLA1 = SDFM1
//    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = CLA_TRIG_SD1INT;
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 1;

// Trigger Source for TASK1 of CLA1 = SDFM2
//

    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 1;

//
// Lock CLA1TASKSRCSEL1 register
//
    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL1 = 1;
    EDIS;

    gPeripheralNumber = SDFM1; // Select SDFM1

//
// Input Control Module
//
//Configure Input Control Mode: Modulator Clock rate = Modulator data rate
//
    Sdfm_configureInputCtrl(gPeripheralNumber, FILTER1, MODE_0);
    Sdfm_configureInputCtrl(gPeripheralNumber, FILTER2, MODE_0);
    Sdfm_configureInputCtrl(gPeripheralNumber, FILTER3, MODE_0);
    Sdfm_configureInputCtrl(gPeripheralNumber, FILTER4, MODE_0);

//
// Comparator Module
//
    HLT = 0x7FFF;    // Over value threshold settings
    LLT = 0x0000;    // Under value threshold settings

//
// Configure Comparator module's comparator filter type and comparator's OSR
// value, higher threshold, lower threshold
//
    Sdfm_configureComparator(gPeripheralNumber, FILTER1, SINC3, OSR_32,
                             HLT, LLT);
    Sdfm_configureComparator(gPeripheralNumber, FILTER2, SINC3, OSR_32,
                             HLT, LLT);
    Sdfm_configureComparator(gPeripheralNumber, FILTER3, SINC3, OSR_32,
                             HLT, LLT);
    Sdfm_configureComparator(gPeripheralNumber, FILTER4, SINC3, OSR_32,
                             HLT, LLT);

//
// Data filter Module
//
// Configure Data filter modules filter type, OSR value and
// enable / disable data filter
//
    Sdfm_configureData_filter(gPeripheralNumber, FILTER1, FILTER_ENABLE, SINC3,
                              OSR_256, DATA_16_BIT, SHIFT_9_BITS);
    Sdfm_configureData_filter(gPeripheralNumber, FILTER2, FILTER_ENABLE, SINC3,
                              OSR_256, DATA_16_BIT, SHIFT_9_BITS);
    Sdfm_configureData_filter(gPeripheralNumber, FILTER3, FILTER_ENABLE, SINC3,
                              OSR_256, DATA_16_BIT, SHIFT_9_BITS);
    Sdfm_configureData_filter(gPeripheralNumber, FILTER4, FILTER_ENABLE, SINC3,
                              OSR_256, DATA_16_BIT, SHIFT_9_BITS);

//
// Enable Master filter bit: Unless this bit is set none of the filter modules
// can be enabled.
// All the filter modules are synchronized when master filter bit is enabled
// after individual filter modules are enabled.
//
    Sdfm_enableMFE(gPeripheralNumber);

//
// PWM11.CMPC, PWM11.CMPD, PWM12.CMPC and PWM12.CMPD signals cannot synchronize
// the filters. This option is not being used in this example.
//
    Sdfm_configureExternalreset(gPeripheralNumber,FILTER_1_EXT_RESET_DISABLE,
                                FILTER_2_EXT_RESET_DISABLE,
                                FILTER_3_EXT_RESET_DISABLE,
                                FILTER_4_EXT_RESET_DISABLE);

//
// Enable interrupts
//
// Following SDFM interrupts can be enabled / disabled using this function.
//  Enable / disable comparator high threshold
//  Enable / disable comparator low threshold
//  Enable / disable modulator clock failure
//  Enable / disable filter acknowledge
//
    Sdfm_configureInterrupt(gPeripheralNumber, FILTER1, IEH_DISABLE,
                            IEL_DISABLE, MFIE_ENABLE, AE_ENABLE);
    Sdfm_configureInterrupt(gPeripheralNumber, FILTER2, IEH_DISABLE,
                            IEL_DISABLE, MFIE_ENABLE, AE_ENABLE);
    Sdfm_configureInterrupt(gPeripheralNumber, FILTER3, IEH_DISABLE,
                            IEL_DISABLE, MFIE_ENABLE, AE_ENABLE);
    Sdfm_configureInterrupt(gPeripheralNumber, FILTER4, IEH_DISABLE,
                            IEL_DISABLE, MFIE_ENABLE, AE_ENABLE);

//
// Enable master interrupt so that any of the filter interrupts can trigger by
// SDFM interrupt to CPU
//
    //SDFM_MASTER_INTERRUPT_ENABLE(gPeripheralNumber);
    Sdfm_enableMIE(gPeripheralNumber);

    EINT;
    ERTM;

    #if(ALLOW_WATCHDOG_TIMER == 1)
       InitWatchdog();  // Khởi tạo Watchdog
    #endif

    CpuToCLA.EnableADC = 0;
    DelayMs(100);
    CpuToCLA.EnableADC = 1;
    CpuToCLA.EnableFlag = 0;

#if(BUILDLEVEL == LEVEL4)
    #if(VAC_LOAD_32_GHEP_NOI)
        CpuToCLA.VdTesting = VAC_LOAD_32_GHEP_NOI;
    #endif
#endif

    CpuToCLA.IdTesting = 0.20;
    CpuToCLA.ADCoffset_Udc = 4;  //
    CpuToCLA.ADCoffset_VaG = 2711; //
    CpuToCLA.ADCoffset_VbG = 2646; //
    CpuToCLA.ADCoffset_VcG = 2695; //
    CpuToCLA.ADCoffset_Ia_inv = 2062; //
    CpuToCLA.ADCoffset_Ib_inv = 2030; //
    CpuToCLA.ADCoffset_Ic_inv = 2070; //

    CpuToCLA.ADCgain_Udc = 1.0;  //
    CpuToCLA.ADCgain_VaG = 0.75999999;
    CpuToCLA.ADCgain_VbG = 0.850000024; //
    CpuToCLA.ADCgain_VcG = 0.75; //
    CpuToCLA.ADCgain_Ia_inv = 1.51999998; //
    CpuToCLA.ADCgain_Ib_inv = 1.50999999; //
    CpuToCLA.ADCgain_Ic_inv = 1.47899997; //

    DelayMs(100);

    while(1)
    {
//        if(START == 1)
//        {
//            b_START_FLC = ON_NORMAL;
//        }
//        else if(START == 0)
//        {
//            b_START_FLC = OFF_NORMAL;
//        }
//        if(b_START_FLC == ON_NORMAL)
//        {
//            START = 1;
//        }
//        else if(b_START_FLC == OFF_NORMAL)
//        {
//            START = 0;
//        }

        if(START == 1 || b_START_FLC == ON_NORMAL)
        {
            #if(BUILDLEVEL == LEVEL1 ||BUILDLEVEL == LEVEL2|| BUILDLEVEL == LEVEL3 || BUILDLEVEL == LEVEL4||BUILDLEVEL == LEVEL5||BUILDLEVEL == LEVEL6||BUILDLEVEL == LEVEL7||BUILDLEVEL == LEVEL8)
                CpuToCLA.EnableFlag = 1;
            #endif
        }
        else
        {
            CpuToCLA.EnableFlag = 0;
        }

        #if(ALLOW_WATCHDOG_TIMER == 1)
           ResetWatchdog();
        #endif

        // Neu co su kien bao ve thi khong cho phep START = 1
        if(EPwm4Regs.TZFLG.bit.OST == 1 || EPwm6Regs.TZFLG.bit.OST == 1 || EPwm5Regs.TZFLG.bit.OST == 1 || EPwm8Regs.TZFLG.bit.OST == 1)
        {
            START = 0;
            CpuToCLA.EnableFlag = 0;
        }

        // Hien thi kenh bao ve
        if(Cmpss3Regs.COMPSTS.bit.COMPHLATCH == 1 && EPwm4Regs.TZFLG.bit.OST == 1 && CMPSS_PROTECT_UDC_UPPER == 1)
        {
            protect_chanel.Udc_upper = 1;
        }

        else if(Cmpss3Regs.COMPSTS.bit.COMPHLATCH == 0 && EPwm4Regs.TZFLG.bit.OST == 0)
        {
            protect_chanel.Udc_upper = 0;
            ClrPrtFlg = 0;
        }

        if(Cmpss1Regs.COMPSTS.bit.COMPHLATCH == 1 && EPwm4Regs.TZFLG.bit.OST == 1 && CMPSS_PROTECT_VbG_UPPER == 1)
        {
            protect_chanel.VbG_upper = 1;
        }

        else if(Cmpss1Regs.COMPSTS.bit.COMPHLATCH == 0 && EPwm4Regs.TZFLG.bit.OST == 0)
        {
            protect_chanel.VbG_upper = 0;
            ClrPrtFlg = 0;
        }

        if(Cmpss1Regs.COMPSTS.bit.COMPLLATCH == 1 && EPwm4Regs.TZFLG.bit.OST == 1 && CMPSS_PROTECT_VbG_LOWER == 1)
        {
            protect_chanel.VbG_lower = 1;
        }

        else if(Cmpss1Regs.COMPSTS.bit.COMPLLATCH == 0 && EPwm4Regs.TZFLG.bit.OST == 0)
        {
            protect_chanel.VbG_lower = 0;
            ClrPrtFlg = 0;

        }

        if(Cmpss2Regs.COMPSTS.bit.COMPHLATCH == 1 && EPwm4Regs.TZFLG.bit.OST == 1 && CMPSS_PROTECT_Ic_inv_UPPER == 1)
        {
            protect_chanel.Ic_upper = 1;
        }

        else if(Cmpss2Regs.COMPSTS.bit.COMPHLATCH == 0 && EPwm4Regs.TZFLG.bit.OST == 0)
        {
            protect_chanel.Ic_upper = 0;
            ClrPrtFlg = 0;
        }

        if(Cmpss2Regs.COMPSTS.bit.COMPLLATCH == 1 && EPwm4Regs.TZFLG.bit.OST == 1 && CMPSS_PROTECT_Ic_inv_LOWER == 1)
        {
            protect_chanel.Ic_lower = 1;
        }

        else if(Cmpss2Regs.COMPSTS.bit.COMPLLATCH == 0 && EPwm4Regs.TZFLG.bit.OST == 0)
        {
            protect_chanel.Ic_lower = 0;
            ClrPrtFlg = 0;
        }
        if(EPwm4Regs.TZFLG.bit.OST == 1)
        {
            CMPSS_Protect_Time = seconds_counter_cmpss;
        }
        else
        {
            CMPSS_Protect_Time = 0;
        }

        if(FLC_RstFlg == 1) // Flag reset FLC
        {
            ClrPrtFlg = 1;
            RunTask8Flag = 1;
            UpdateProtectValue();
            seconds_counter_cmpss = 0;
            FLC_RstFlg = 0;
        }

        if(ClrPrtFLg_Fst == 0) // Clear protect CMPSS first after debug
        {
            ClearProtectFlagFcn();
            ClrPrtFLg_Fst = 1;
        }

        if(ClrPrtFlg == 1) // Flag reset protect CMPSS
        {
            ClearProtectFlagFcn();
        }

        #if(BUILDLEVEL == LEVEL4)
        // Nếu điện áp Udc không đủ để điều chế ra Vac đặt thì hiện thị cảnh báo và tắt START
        if(ClaToCPU.Udc_under_modulation == 1)
        {
            protect_chanel.Udc_under_modulation = 1;
    //        START = 0;
    //        CpuToCLA.EnableFlag = 0;
        }
        else
        {
            protect_chanel.Udc_under_modulation = 0;
        }
        #endif

        if(RunTask8Flag == 1)
        {
            EALLOW;
            Cla1Regs.MIFRC.bit.INT8 = 1; // Force Task8 run
            EDIS;
            RunTask8Flag = 0;
        }

        #if(ALLOW_BUTTON == 1)
            if(GpioDataRegs.GPBDAT.bit.GPIO32 == 1)
            {
                START = 0;
                CpuToCLA.EnableFlag = 0;
            }
        #endif

        #if(SET_MODE_RUN == THREE_PHASE_MODE)

            ON_RELAY = 1;

            if (ON_RELAY == 1)
            {
                GpioDataRegs.GPASET.bit.GPIO27 = 1; // Relay 1
                GpioDataRegs.GPASET.bit.GPIO25 = 1; // Relay 2
            }
            else
            {
                GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // Relay 1
                GpioDataRegs.GPACLEAR.bit.GPIO25 = 1; // Relay 2
            }

        #endif

        #if(SET_MODE_RUN == SINGLE_PHASE_MODE)

            if(ON_RELAY == 0)
            { GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // Relay 1
            GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;} // Relay 2
            else
            { GpioDataRegs.GPASET.bit.GPIO27 = 1; // Relay 1
              GpioDataRegs.GPASET.bit.GPIO25 = 1;}
          //  while(GpioDataRegs.GPADAT.bit.GPIO27 != 0 && GpioDataRegs.GPADAT.bit.GPIO25 != 0)
           // {
           //     START = 0;

           // }

        #endif

        #if(ALLOW_IPC_CPU == 1)

            if (START_1 == 1)
            {
                IpcRegs.IPCSENDDATA = 1;
                IpcRegs.IPCSET.bit.IPC0 = 1;
                while (IpcRegs.IPCFLG.bit.IPC0 == 1);
            }
            else
            {
                IpcRegs.IPCSENDDATA = 0;
                IpcRegs.IPCSET.bit.IPC0 = 1;
                while (IpcRegs.IPCFLG.bit.IPC0 == 1);
            }

        #endif
    }
}

// Sdfm_configurePins - Configure SDFM pin muxing GPIOs
//

void Sdfm_configurePins(Uint16 sdfmPinOption)
{
    Uint16 pin;
    switch (sdfmPinOption)
    {
        case SDFM_PIN_MUX_OPTION1:
            for(pin = 16; pin <= 31; pin++)
            {
                GPIO_SetupPinOptions(pin, GPIO_INPUT, GPIO_ASYNC);
                GPIO_SetupPinMux(pin,GPIO_MUX_CPU1,7);
            }
            break;

        case SDFM_PIN_MUX_OPTION2:
            for(pin = 48; pin <= 63; pin++)
            {
                GPIO_SetupPinOptions(pin, GPIO_INPUT, GPIO_ASYNC);
                GPIO_SetupPinMux(pin,GPIO_MUX_CPU1,7);
            }
            break;

        case SDFM_PIN_MUX_OPTION3:
            for(pin = 122; pin <= 137; pin++)
            {
                GPIO_SetupPinOptions(pin, GPIO_INPUT, GPIO_ASYNC);
                GPIO_SetupPinMux(pin,GPIO_MUX_CPU1,7);
            }
            break;
    }
}

//
// Cla_initMemoryMap - Initialize CLA memory map
//
void Cla_initMemoryMap(void)
{
    EALLOW;

    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

    //
    // Select LS1 and LS2 RAM to be the programming space for the CLA
    // Select LS5 to be data RAM for the CLA
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;

//    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;
//
//    //
//    // Filter1 and Filter2 data memory LS0
//    //
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS2 = 1; //LS2RAM is shared between CPU and CLA
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS2 = 0; //LS2RAM setup as data memory
//
//    //
//    // Filter3 and Filter4 data memory LS3
//    //
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1; //LS3RAM is shared between CPU and CLA
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 0; //LS3RAM setup as data memory
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 1;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS2 = 1; //LS2RAM is shared between CPU and CLA
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS2 = 1; //LS2RAM setup as data memory

    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1; //LS3RAM is shared between CPU and CLA
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 1; //LS3RAM setup as data memory

    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1; //LS4RAM is shared between CPU and CLA
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 0; //LS4RAM setup as data memory
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1; //LS4RAM setup as data memory

    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1; //LS5RAM is shared between CPU and CLA
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1; //LS5RAM setup as data memory
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1; //LS5RAM setup as data memory


    EDIS;
}

//
// CLA_initCpu1Cla - Initialize CLA tasks and end of task ISRs
//
void CLA_initCpu1Cla(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
    Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
    Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
    Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
    Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
    Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
    Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);

    //
    // Enable IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks
    //
    asm("   RPT #3 || NOP");
    Cla1Regs.MCTL.bit.IACKE = 1;
    Cla1Regs.MIER.all = 0x0083;

    //
    // Configure the vectors for the end-of-task interrupt for all
    // 8 tasks
    //
    PieVectTable.CLA1_1_INT = &cla1Isr1;
    PieVectTable.CLA1_2_INT = &cla1Isr2;
    PieVectTable.CLA1_3_INT = &cla1Isr3;
    PieVectTable.CLA1_4_INT = &cla1Isr4;
    PieVectTable.CLA1_5_INT = &cla1Isr5;
    PieVectTable.CLA1_6_INT = &cla1Isr6;
    PieVectTable.CLA1_7_INT = &cla1Isr7;
    PieVectTable.CLA1_8_INT = &cla1Isr8;

    //
    // Enable CLA interrupts at the group and subgroup levels
    //
    PieCtrlRegs.PIEIER11.all = 0xFFFF;
    IER |= (M_INT11 );
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    EDIS;
}

// cla1Isr1 - CLA 1 ISR 1
//
interrupt void cla1Isr1 ()
{
     //asm(" ESTOP0");
    Task1_Isr++;
    PieCtrlRegs.PIEACK.all = M_INT11;
    dlog1((ClaToCPU.ADC_CPU.datalog1+1)*10000);
    dlog2((ClaToCPU.ADC_CPU.datalog2+1)*10000);
    dlog3((ClaToCPU.ADC_CPU.datalog3+1)*10000);
}

//
// cla1Isr2 - CLA 1 ISR 2
//
interrupt void cla1Isr2 ()
{
     //asm(" ESTOP0");
     PieCtrlRegs.PIEACK.all = M_INT11;
}

//
// cla1Isr3 - CLA 1 ISR 3
//
interrupt void cla1Isr3 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr4 - CLA 1 ISR 4
//
interrupt void cla1Isr4 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr5 - CLA 1 ISR 5
//
interrupt void cla1Isr5 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr6 - CLA 1 ISR 6
//
interrupt void cla1Isr6 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr7 - CLA 1 ISR 7
//
interrupt void cla1Isr7 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr8 - CLA 1 ISR 8
//
interrupt void cla1Isr8 ()
{
    // asm(" ESTOP0");
    Task8_Isr++;
    PieCtrlRegs.PIEACK.all = M_INT11;
}

static inline void dlog1(Uint16 value)
{
    DataLog1[ndx1] = value;
    if(++ndx1 == DLOG_SIZE_1)
    {
        ndx1 = 0;
    }
}

static inline void dlog2(Uint16 value)
{
    DataLog2[ndx2] = value;
    if(++ndx2 == DLOG_SIZE_2)
    {
        ndx2 = 0;
    }
}
static inline void dlog3(Uint16 value)
{
    DataLog3[ndx3] = value;
    if(++ndx3 == DLOG_SIZE_3)
    {
      ndx3 = 0;
    }
}

//
// End of file
//
