//RAMLS1: origin = 0x008800, length = 0x000800
//RAMLS2: origin = 0x009000, length = 0x000800
//RAMLS3: origin = 0x009800, length = 0x000800
//RAMLS4: origin = 0x00A000, length = 0x000800
//RAMLS5: origin = 0x00A800, length = 0x000800

// The user must define CLA_C in the project linker settings if using the
// CLA C compiler
// Project Properties -> C2000 Linker -> Advanced Options -> Command File
// Preprocessing -> --define
#ifdef CLA_C
// Define a size for the CLA scratchpad area that will be used
// by the CLA compiler for local symbols and temps
// Also force references to the special symbols that mark the
// scratchpad are.
CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start
#endif //CLA_C

MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */

   BEGIN           	: origin = 0x000000, length = 0x000002
   RAMM0           	: origin = 0x000122, length = 0x0002DE
   RAMD0           	: origin = 0x00B000, length = 0x000800
   RAMD1            : origin = 0x00B800, length = 0x000800
   //RAMD01            : origin = 0x00B000, length = 0x001000
   //RAMLS4      		: origin = 0x00A000, length = 0x000800
   //RAMLS5           : origin = 0x00A800, length = 0x00800
   //RAMLS45          : origin = 0x00A000, length = 0x001000
   //RAMLS345         : origin = 0x009800, length = 0x001800
   //RAMLS2345        : origin = 0x009000, length = 0x002000
   RAMLS12345      : origin = 0x008800, length = 0x002800
   RAMGS1           : origin = 0x00D000, length = 0x001000
   RESET           	: origin = 0x3FFFC0, length = 0x000002

PAGE 1 :

   BOOT_RSVD       : origin = 0x000002, length = 0x000121     /* Part of M0, BOOT rom will use this for stack */
   RAMM1           : origin = 0x000400, length = 0x0003F8     /* on-chip RAM block M1 */
//   RAMM1_RSVD      : origin = 0x0007F8, length = 0x000008     /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RAMLS0          	: origin = 0x008000, length = 0x000800
   //RAMLS1          	: origin = 0x008800, length = 0x000800
   //RAMLS01          	: origin = 0x008000, length = 0x001000
   RAMLS2_1      	: origin = 0x009000, length = 0x000400
   RAMLS2_2         : origin = 0x009400, length = 0x000400
   RAMLS3_1      	: origin = 0x009800, length = 0x000400
   RAMLS3_2			: origin = 0x009C00, length = 0x000400

   RAMGS0           : origin = 0x00C000, length = 0x001000

   RAMGS2           : origin = 0x00E000, length = 0x001000
   RAMGS3           : origin = 0x00F000, length = 0x001000
   RAMGS4           : origin = 0x010000, length = 0x001000
   RAMGS5           : origin = 0x011000, length = 0x001000
   RAMGS6           : origin = 0x012000, length = 0x001000
   RAMGS7           : origin = 0x013000, length = 0x001000
   RAMGS8           : origin = 0x014000, length = 0x001000
   RAMGS9           : origin = 0x015000, length = 0x001000
   RAMGS10     : origin = 0x016000, length = 0x001000

//   RAMGS11     : origin = 0x017000, length = 0x000FF8   /* Uncomment for F28374D, F28376D devices */

//   RAMGS11_RSVD : origin = 0x017FF8, length = 0x000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RAMGS11     : origin = 0x017000, length = 0x001000     /* Only Available on F28379D/_, F28377D/F28377S, F28375D/F28375S devices. Remove line on other devices. */
   RAMGS12     : origin = 0x018000, length = 0x001000     /* Only Available on F28379D/_, F28377D/F28377S, F28375D/F28375S devices. Remove line on other devices. */
   RAMGS13     : origin = 0x019000, length = 0x001000     /* Only Available on F28379D/_, F28377D/F28377S, F28375D/F28375S devices. Remove line on other devices. */
   RAMGS14     : origin = 0x01A000, length = 0x001000     /* Only Available on F28379D/_, F28377D/F28377S, F28375D/F28375S devices. Remove line on other devices. */
   RAMGS15     : origin = 0x01B000, length = 0x000FF8     /* Only Available on F28379D/_, F28377D/F28377S, F28375D/F28375S devices. Remove line on other devices. */
   
//   RAMGS15_RSVD : origin = 0x01BFF8, length = 0x000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
                                                            /* Only on F28379D/_, F28377D/F28377S, F28375D/F28375S devices. Remove line on other devices. */

   CLA1_MSGRAMLOW   : origin = 0x001480, length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500, length = 0x000080
   
}


SECTIONS
{
   codestart        : > BEGIN,     PAGE = 0
   .text            : >> RAMM0|RAMD0|RAMD1|RAMGS1,   PAGE = 0
   //.text            : >> RAMD01|RAMLS4|RAMM0|RAMGS1,   PAGE = 0
   .cinit           : > RAMM0,     PAGE = 0
   .pinit           : > RAMM0,     PAGE = 0
   .switch          : > RAMM0,     PAGE = 0
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   .stack           : > RAMM1,     PAGE = 1
   .ebss            : > RAMGS2,    PAGE = 1
   .econst          : > RAMGS3,    PAGE = 1
   .esysmem         : > RAMGS3,    PAGE = 1
   Filter_RegsFile  : > RAMGS0,	   PAGE = 1
   CLA1mathTables   : > RAMLS0,    PAGE = 1

 #ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
       .TI.ramfunc      : > RAMM0      PAGE = 0
    #else
       ramfuncs         : > RAMM0      PAGE = 0
    #endif
#endif

    /* CLA specific sections */
   //Cla1Prog         : > RAMLS2345, PAGE=0
   Cla1Prog         : > RAMLS12345, PAGE=0

   CLADataLS0		: > RAMLS0, PAGE=1
   //CLADataLS0		: > RAMLS01, PAGE=1
   //CLADataLS1		: > RAMLS1, PAGE=1

   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW,   PAGE = 1
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH,  PAGE = 1

   /* The following section definition are for SDFM examples */
   Filter1_RegsFile : > RAMLS2_1,	PAGE = 1, fill=0x1111
   Filter2_RegsFile : > RAMLS2_2,	PAGE = 1, fill=0x2222
   Filter3_RegsFile : > RAMLS3_1,	PAGE = 1, fill=0x3333
   Filter4_RegsFile : > RAMLS3_2,	PAGE = 1, fill=0x4444
   
#ifdef CLA_C
   /* CLA C compiler sections */
   //
   // Must be allocated to memory the CLA has write access to
   //
  /* CLAscratch       :
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) } >  RAMLS1,  PAGE = 1

   .scratchpad      : > RAMLS1,       PAGE = 1
   .bss_cla		    : > RAMLS1,       PAGE = 1
   .const_cla	    : > RAMLS1,       PAGE = 1  */
     CLAscratch       :
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) } >  RAMLS0,  PAGE = 1
//                     *.obj(CLAscratch_end) } >  RAMLS01,  PAGE = 1
   .scratchpad      : > RAMLS0,       PAGE = 1
   .bss_cla		    : > RAMLS0,       PAGE = 1
   .const_cla	    : > RAMLS0,       PAGE = 1
//   .scratchpad      : > RAMLS01,       PAGE = 1
//   .bss_cla		    : > RAMLS01,       PAGE = 1
//   .const_cla	    : > RAMLS01,       PAGE = 1
#endif //CLA_C
 /* Test specific sections */
   IOBuffer         : > RAMLS0,     PAGE = 1
//   IOBuffer         : > RAMLS01,     PAGE = 1
   .reset           : > RESET,      PAGE = 0, TYPE = DSECT /* not used, */

   .cio             : > RAMGS7,     PAGE = 1
   .sysmem          : > RAMGS7,     PAGE = 1

   .stack           : > RAMGS2,     PAGE = 1 /* Needs to be in lower 64K memory */
   .ebss            : > RAMLS2_1,     PAGE = 1
   .esysmem         : > RAMGS7,     PAGE = 1
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
