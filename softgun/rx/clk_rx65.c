/**
 ************************************************************
 * Clock generation Circuit of RX65N
 ************************************************************
 */

#include <stdint.h>
#include <inttypes.h>
#include "bus.h"
#include "sglib.h"
#include "sgstring.h"
#include "clk_rx65.h"
#include "clock.h"
#include "signode.h"
#include "configfile.h"


#define REG_SCKCR(base)     0x00080020
#define REG_ROMWT(base)     0x0008101c
#define REG_SCKCR2(base)    0x00080024
#define REG_SCKCR3(base)    0x00080026
#define REG_PLLCR(base)     0x00080028
#define REG_PLLCR2(base)    0x0008002a
#define     PLLCR2_PLLEN    (1 << 0)
#define REG_BCKCR(base)     0x00080030
#define REG_MOSCCR(base)    0x00080032
#define REG_SOSCCR(base)    0x00080033
#define REG_LOCOCR(base)    0x00080034
#define REG_ILOCOCR(base)   0x00080035
#define REG_HOCOCR(base)    0x00080036
#define REG_HOCOCR2(base)   0x00080037
#define REG_OSCOVFSR(base)  0x0008003c
#define     OSCOVFSR_MOOVF      (1 << 0)
#define     OSCOVFSR_SOOVF      (1 << 1)
#define     OSCOVFSR_PLOVF      (1 << 2)
#define     OSCOVFSR_HCOVF      (1 << 3)
#define     OSCOVFSR_ILCOVF     (1 << 4)
#define REG_OSTDCR(base)    0x00080040
#define REG_OSTDSR(base)    0x00080041
#define REG_MOSCWTCR(base)  0x000800a2
#define REG_SOSCWTCR(base)  0x000800a3
#define REG_MOFCR(base)     0x0008c293
#define REG_HOCOPCR(base)   0x0008c294

typedef struct RxClk {
    BusDevice bdev;
    bool isInit;
    uint32_t subOscFreq;
    uint32_t mainOscFreq;
    uint32_t iwdtOscFreq;
    uint32_t locoOscFreq;
    uint32_t hocoOscFreq;
    //SigNode *sigMstp[128];
    Clock_t *clkMainOsc;
    Clock_t *clkSubOsc;
    Clock_t *clkHoco;
    Clock_t *clkLoco;
    Clock_t *clkPllOut;
    Clock_t *clkFDivIn;
    Clock_t *clkDiv1;
    Clock_t *clkDiv2;
    Clock_t *clkDiv4;
    Clock_t *clkDiv8;
    Clock_t *clkDiv16;
    Clock_t *clkDiv32;
    Clock_t *clkDiv64;
    Clock_t *clkDiv5;
    Clock_t *clkDiv3;

    /* The output clocks */
    Clock_t *clkFCLK;
    Clock_t *clkICLK;
    Clock_t *clkPCKA;
    Clock_t *clkPCKB;
    Clock_t *clkPCKC;
    Clock_t *clkPCKD;
    Clock_t *clkBCLK;
    Clock_t *clkBCLKPin;
    Clock_t *clkUCLK;
    //Clock_t *clkGLCDC; /*  same as PLL Out */
    Clock_t *clkCANMCLK;
    Clock_t *clkIWDTCLK;
    Clock_t *clkRTCSCLK;
    Clock_t *clkRTCMCLK;

    uint32_t regSCKCR;
    uint8_t regROMWT;
    uint16_t regSCKCR2;
    uint16_t regSCKCR3;
    uint16_t regPLLCR;
    uint8_t regPLLCR2;
    uint8_t regBCKCR;
    uint8_t regMOSCCR;
    uint8_t regSOSCCR;
    uint8_t regLOCOCR;
    uint8_t regILOCOCR;
    uint8_t regHOCOCR;
    uint8_t regHOCOCR2;
    uint8_t regOSCOVFSR;
    uint8_t regOSTDCR;
    uint8_t regOSTDSR;
    uint8_t regMOSCWTCR;
    uint8_t regSOSCWTCR;
    uint8_t regMOFCR;
    uint8_t regHOCOPCR;
} RxClk;

/*
 *********************************************************************
 * \fn static void romwt_verify(RxClk *rc); 
 *********************************************************************
 */
static void
romwt_verify(RxClk *rc) 
{
    unsigned int div;
    uint64_t iclkfreq; 
    uint32_t flashFreq;
    switch (rc->regROMWT) {
        case 0:  
            div = 1;
            break;
        case 1:  
            div = 2;
            break;
        case 2:  
            div = 3;
            break;
        default:
            fprintf(stderr, "Illegal Wait states for Flash in ROMWT register\n");
            __builtin_trap();
            break;
    }
    iclkfreq = Clock_Freq(rc->clkICLK);
    flashFreq = iclkfreq / div;
    if (flashFreq > (50 * 1000 * 1000)) {
        fprintf(stderr, "Error: ROMWT Flash Frequency to high\n");
        __builtin_trap();
    }
}
 
/**
 ***********************************************************************************************
 * \fn static void update_pllclock(RxClk *rc) 
 ***********************************************************************************************
 */
static void
update_pllclock(RxClk *rc) 
{
    unsigned int plldiv, pllsrcsel, stc;
    unsigned int div, mul;
    uint64_t freq;
    Clock_t *pllInClk;
    plldiv = rc->regPLLCR & 0xf;
    pllsrcsel = (rc->regPLLCR >> 4) & 1;
    stc = (rc->regPLLCR >> 8) & 0x3f;
        switch (plldiv) {
            case 0:
                div = 1;
                break;
            case 1:
                div = 2;
                break;
            case 2:
                div = 3;
                break;
            default:
                div = 1;
                fprintf(stderr, "Illegal PLLCR value 0x%04x\n", rc->regPLLCR);
                __builtin_trap();
        }
        if ((stc > 0x3b) || (stc < 0x13)) {
                fprintf(stderr, "Illegal PLLCR STC value %u (0x%04x)\n",stc,  rc->regPLLCR);
                __builtin_trap();
        }
        mul = 20 + (stc - 0x13);
        if (pllsrcsel) {
            pllInClk = rc->clkHoco;
        } else {
            pllInClk = rc->clkMainOsc;
        }
        if (rc->regPLLCR2 & 1) {
            rc->regOSCOVFSR &= ~OSCOVFSR_PLOVF;
            /* If PLL is disabled then the clock is 0 */
            Clock_MakeDerived(rc->clkPllOut, pllInClk, 0, 1);
        } else {
            freq = Clock_Freq(pllInClk);
            if (((freq / div) > 24000000) || ((freq / div) < 8000000)) {
                fprintf(stderr, "PLL: Divided Frequency out of range: %" PRIu64 "\n", freq / div);
            }
            Clock_MakeDerived(rc->clkPllOut, pllInClk, mul, 2 * div);
            freq = Clock_Freq(rc->clkPllOut);
            if ((freq > 240000000) || (freq < 120000000)) {
                //if (!rc->isInit) {
                fprintf(stderr, "PLL: Output Frequency out of range %" PRIu64 "\n", freq);
                //}
            }
            rc->regOSCOVFSR |= OSCOVFSR_PLOVF;
        }
}

/**
 **********************************************************************************
 * Clock Trace proc for ICLK, verifies flash access speed on every clock change
 **********************************************************************************
 */
static void 
ICLK_TraceProc(struct Clock *clock, void *clientData)
{
    RxClk *rc = clientData;
    romwt_verify(rc);
}

static void
sckcr_derive_clock(RxClk *rc, Clock_t *derivedClk, uint8_t selection) 
{
    switch (selection) {
        case 0: 
            Clock_MakeDerived(derivedClk, rc->clkDiv1, 1, 1);
            break;

        case 1: 
            Clock_MakeDerived(derivedClk, rc->clkDiv2, 1, 1);
            break;

        case 2: 
            Clock_MakeDerived(derivedClk, rc->clkDiv4, 1, 1);
            break;

        case 3: 
            Clock_MakeDerived(derivedClk, rc->clkDiv8, 1, 1);
            break;

        case 4: 
            Clock_MakeDerived(derivedClk, rc->clkDiv16, 1, 1);
            break;

        case 5: 
            Clock_MakeDerived(derivedClk, rc->clkDiv32, 1, 1);
            break;

        case 6: 
            Clock_MakeDerived(derivedClk, rc->clkDiv64, 1, 1);
            break;

        default:
            fprintf(stderr, "Illegal CLK selection in SCKCR\n");
            Clock_MakeDerived(derivedClk, rc->clkDiv1, 0, 1);
            break;
    }
}

static uint32_t
sckcr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regSCKCR;
}

static void
sckcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    uint32_t fck, ick, pstop, bck, pcka, pckb, pckc, pckd; 
    rc->regSCKCR = value & 0xffcfffff;
    fck = (value >> 28) & 0xf;
    ick = (value >> 24) & 0xf;
    pstop = (value >> 22) & 3;
    bck = (value >> 16) & 0xf; 
    pcka = (value >> 12) & 0xf;
    pckb = (value >> 8) & 0xf;
    pckc = (value >> 4) & 0xf;
    pckd = (value & 0xf);
    sckcr_derive_clock(rc, rc->clkFCLK, fck);
    sckcr_derive_clock(rc, rc->clkICLK, ick);
    sckcr_derive_clock(rc, rc->clkBCLK, bck);
    sckcr_derive_clock(rc, rc->clkPCKA, pcka);
    sckcr_derive_clock(rc, rc->clkPCKB, pckb);
    sckcr_derive_clock(rc, rc->clkPCKC, pckc);
    sckcr_derive_clock(rc, rc->clkPCKD, pckd);
}

static uint32_t
romwt_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regROMWT;
}

static void
romwt_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regROMWT = value & 3;
    romwt_verify(rc); 
}

static uint32_t
sckcr2_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regSCKCR2 | 1;
}

static void
sckcr2_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    unsigned int uck;
    rc->regSCKCR2 = value & (0xf << 4);
    uck = (value >> 4) & 0xf;
    if ((value & 0xff0f) != 0x0001) {
        fprintf(stderr, "SCKCR2: Bad Reserved Bits: 0x%04x\n", value);
        __builtin_trap();
    }
    switch (uck) {
       case 1:
            Clock_MakeDerived(rc->clkUCLK, rc->clkDiv2, 1, 1);
            break;
        case 2:
            Clock_MakeDerived(rc->clkUCLK, rc->clkDiv3, 1, 1);
            break;
        case 3:
            Clock_MakeDerived(rc->clkUCLK, rc->clkDiv4, 1, 1);
            break;
        case 4:
            Clock_MakeDerived(rc->clkUCLK, rc->clkDiv5, 1, 1);
            break;
        default:
            fprintf(stderr, "Bad UCK selection %d\n", uck);
            Clock_MakeDerived(rc->clkUCLK, rc->clkDiv1, 0, 1);
            break;
    }
}

static uint32_t
sckcr3_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regSCKCR3; 
}

static void
sckcr3_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regSCKCR3 = value & 0x0700;
    unsigned int cksel = (value >> 8) & 7;
    if ((value & 0xF8FF) != 0) {
        fprintf(stderr, "SCKCR3: Write illegal value 0x%04x\n", value);
        __builtin_trap();
    }
    switch (cksel) {
        case 0: /* LOCO */
            Clock_MakeDerived(rc->clkFDivIn, rc->clkLoco, 1, 1);
            break;

        case 1: /* HOCO */
            Clock_MakeDerived(rc->clkFDivIn, rc->clkHoco, 1, 1);
            break;
        case 2: /* Main OSC */
            Clock_MakeDerived(rc->clkFDivIn, rc->clkMainOsc, 1, 1);
            break;
        case 3: /* Sub OSC */
            Clock_MakeDerived(rc->clkFDivIn, rc->clkSubOsc, 1, 1);
            break;
        case 4: /* PLL */
            Clock_MakeDerived(rc->clkFDivIn, rc->clkPllOut, 1, 1);
            break;
        default:
            fprintf(stderr, "SCKCR3: illegal cksel %u\n", cksel);
            __builtin_trap();
    }
}

static uint32_t
pllcr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regPLLCR;
}

static void
pllcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regPLLCR = value & 0x3f13;
    update_pllclock(rc);
}

static uint32_t
pllcr2_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regPLLCR2;
}

static void
pllcr2_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    uint8_t diffEn = (value & 1) ^ (rc->regPLLCR2 & 1);
    rc->regPLLCR2 = value & 1;
    if (diffEn) {
        update_pllclock(rc);
    }
}

static uint32_t
bckcr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regBCKCR;
}

static void
bckcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regBCKCR = value & 1;
    fprintf(stderr, "External bus clock pin not implemented\n");
}

static uint32_t
mosccr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regMOSCCR;
}

static void
mosccr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    if (value & 1) {
        rc->regOSCOVFSR &= ~OSCOVFSR_MOOVF;
        Clock_SetFreq(rc->clkMainOsc, 0);
    } else {
        Clock_SetFreq(rc->clkMainOsc, rc->mainOscFreq);
        rc->regOSCOVFSR |= OSCOVFSR_MOOVF;
    }
    rc->regMOSCCR = value & 1;
}

static uint32_t
sosccr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regSOSCCR;
}

static void
sosccr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    /* Clock also runs when RTCEN is set, this is missing here */
    if (value & 1) {
        rc->regOSCOVFSR &= ~OSCOVFSR_SOOVF;
        Clock_SetFreq(rc->clkSubOsc, 0);
    } else {
        rc->regOSCOVFSR |= OSCOVFSR_SOOVF;
        Clock_SetFreq(rc->clkSubOsc, rc->subOscFreq);
    }
    rc->regSOSCCR = value & 1;
}

static uint32_t
lococr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regLOCOCR;
}

static void
lococr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    if (value & 1) {
        Clock_SetFreq(rc->clkLoco, 0);
    } else {
        Clock_SetFreq(rc->clkLoco, rc->locoOscFreq);
    }
    rc->regLOCOCR = value & 1;
}

static uint32_t
ilococr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regILOCOCR;
}

static void
ilococr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    if (value & 1) {
        rc->regOSCOVFSR &= ~OSCOVFSR_ILCOVF;
        Clock_SetFreq(rc->clkIWDTCLK, 0);
    } else {
        rc->regOSCOVFSR |= OSCOVFSR_ILCOVF;
        Clock_SetFreq(rc->clkIWDTCLK, rc->iwdtOscFreq);
    }
    rc->regILOCOCR = value & 1;
}

static uint32_t
hococr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regHOCOCR;
}

static void
hococr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    if (value & 1) {
        rc->regOSCOVFSR |= OSCOVFSR_HCOVF;
        Clock_SetFreq(rc->clkHoco, 0);
    } else {
        Clock_SetFreq(rc->clkHoco, rc->hocoOscFreq);
    }
    rc->regHOCOCR = value & 1;
}

static uint32_t
hococr2_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regHOCOCR2;
}

static void
hococr2_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    uint8_t hcfrq = value & 3;
    if ((rc->regHOCOCR & 1) == 0) {
        fprintf(stderr, "Prohibited writing to HOCOCR2 while Oscillator is enabled\n");
        __builtin_trap();
    }
    switch (hcfrq) {
        case 0:
            rc->hocoOscFreq = 16 * 1000 * 1000; /* Just the default */
            break;
        case 1:
            rc->hocoOscFreq = 18 * 1000 * 1000; /* Just the default */
            break;
        case 2:
            rc->hocoOscFreq = 20 * 1000 * 1000; /* Just the default */
            break;
        default:
            fprintf(stderr, "Illegal HOCO Frequence selection\n");
            __builtin_trap();
            break;
    }
}

static uint32_t
oscovfsr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regOSCOVFSR;
}

static void
oscovfsr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    fprintf(stderr, "OSCOVFSR_WRITE\n");
}

static uint32_t
ostdcr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regOSTDCR; 
}

static void
ostdcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regOSTDCR = value & 0x81; 
}

static uint32_t
ostdsr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regOSTDSR;
}

static void
ostdsr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regOSTDSR = rc->regOSTDSR & value;
}

static uint32_t
moscwtcr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regMOSCWTCR;    
}

static void
moscwtcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regMOSCWTCR = value & 0xff;
}

static uint32_t
soscwtcr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regSOSCWTCR;    
}

static void
soscwtcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regSOSCWTCR = value & 0xff;
}

static uint32_t
mofcr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regMOFCR; 
}

static void
mofcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regMOFCR = value & 0x71; 
    fprintf(stderr, "Main Osc driving strenth\n");
}

static uint32_t
hocopcr_read(void *clientData, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    return rc->regHOCOPCR; 
}

static void
hocopcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    RxClk *rc = clientData;
    rc->regHOCOPCR = value & 1; 
    if (value & 1) {
        fprintf(stderr, "Turning off HOCO power supply not implemented\n");
    }
}

static void
RxClk_Unmap(void *owner, uint32_t base, uint32_t mask)
{
    IOH_Delete32(REG_SCKCR(base));
    IOH_Delete8(REG_ROMWT(base)); 
    IOH_Delete16(REG_SCKCR2(base));
    IOH_Delete16(REG_SCKCR3(base));
    IOH_Delete16(REG_PLLCR(base));
    IOH_Delete8(REG_PLLCR2(base));
    IOH_Delete8(REG_BCKCR(base));
    IOH_Delete8(REG_MOSCCR(base));
    IOH_Delete8(REG_SOSCCR(base));
    IOH_Delete8(REG_LOCOCR(base));
    IOH_Delete8(REG_ILOCOCR(base));
    IOH_Delete8(REG_HOCOCR(base));
    IOH_Delete8(REG_HOCOCR2(base));
    IOH_Delete8(REG_OSCOVFSR(base));
    IOH_Delete8(REG_OSTDCR(base));
    IOH_Delete8(REG_OSTDSR(base));
    IOH_Delete8(REG_MOSCWTCR(base));
    IOH_Delete8(REG_SOSCWTCR(base));
    IOH_Delete8(REG_MOFCR(base));
    IOH_Delete8(REG_HOCOPCR(base));
}

static void
RxClk_Map(void *owner, uint32_t base, uint32_t mask, uint32_t mapflags)
{
    RxClk *rc = owner;
    IOH_New32(REG_SCKCR(base), sckcr_read, sckcr_write, rc);
    IOH_New8(REG_ROMWT(base), romwt_read, romwt_write, rc); 
    IOH_New16(REG_SCKCR2(base), sckcr2_read, sckcr2_write, rc);
    IOH_New16(REG_SCKCR3(base), sckcr3_read, sckcr3_write, rc);
    IOH_New16(REG_PLLCR(base), pllcr_read, pllcr_write, rc);
    IOH_New8(REG_PLLCR2(base), pllcr2_read, pllcr2_write, rc);
    IOH_New8(REG_BCKCR(base), bckcr_read, bckcr_write, rc);
    IOH_New8(REG_MOSCCR(base), mosccr_read, mosccr_write, rc);
    IOH_New8(REG_SOSCCR(base), sosccr_read, sosccr_write, rc);
    IOH_New8(REG_LOCOCR(base), lococr_read, lococr_write, rc);
    IOH_New8(REG_ILOCOCR(base), ilococr_read, ilococr_write, rc);
    IOH_New8(REG_HOCOCR(base), hococr_read, hococr_write, rc);
    IOH_New8(REG_HOCOCR2(base), hococr2_read, hococr2_write, rc);
    IOH_New8(REG_OSCOVFSR(base), oscovfsr_read, oscovfsr_write, rc);
    IOH_New8(REG_OSTDCR(base), ostdcr_read, ostdcr_write, rc);
    IOH_New8(REG_OSTDSR(base), ostdsr_read, ostdsr_write, rc);
    IOH_New8(REG_MOSCWTCR(base), moscwtcr_read, moscwtcr_write, rc);
    IOH_New8(REG_SOSCWTCR(base), soscwtcr_read, soscwtcr_write, rc);
    IOH_New8(REG_MOFCR(base), mofcr_read, mofcr_write, rc);
    IOH_New8(REG_HOCOPCR(base), hocopcr_read, hocopcr_write, rc);
}

BusDevice *
Rx65Clk_New(const char *name)
{
    RxClk *rc = sg_new(RxClk);

    rc->bdev.first_mapping = NULL;
    rc->bdev.Map = RxClk_Map;
    rc->bdev.UnMap = RxClk_Unmap;
    rc->bdev.owner = rc;
    rc->bdev.hw_flags = MEM_FLAG_WRITABLE | MEM_FLAG_READABLE;

    rc->isInit = true;
    rc->mainOscFreq = 12 * 1000 * 1000;
    rc->subOscFreq = 32768;
    rc->iwdtOscFreq = 120 * 1000;
    rc->locoOscFreq = 240 * 1000;
    rc->hocoOscFreq = 16 * 1000 * 1000; /* Just the default */

    rc->regMOSCWTCR = 0x53;
    rc->regSOSCWTCR = 0x21;
    rc->regPLLCR = 0x1d00;
    rc->regPLLCR2= 0x1;
    rc->regILOCOCR = 1;
    rc->regHOCOCR = 1;

    Config_ReadUInt32(&rc->mainOscFreq, "global", "crystal");
    Config_ReadUInt32(&rc->subOscFreq, "global", "subclk");

    rc->clkMainOsc = Clock_New("%s.extal", name);
    rc->clkSubOsc = Clock_New("%s.subosc", name);
    rc->clkHoco = Clock_New("%s.hoco", name);
    rc->clkLoco = Clock_New("%s.loco", name);
    Clock_SetFreq(rc->clkLoco, rc->locoOscFreq);
    //Clock_SetFreq(rc->clkHoco, rc->hocoOscFreq);

    rc->clkPllOut = Clock_New("%s.pllout", name);
    rc->clkFDivIn = Clock_New("%s.fdivin", name);
    rc->clkDiv1 = Clock_New("%s.div1", name);
    rc->clkDiv2 = Clock_New("%s.div2", name);
    rc->clkDiv4 = Clock_New("%s.div4", name);
    rc->clkDiv8 = Clock_New("%s.div8", name);
    rc->clkDiv16 = Clock_New("%s.div16", name);
    rc->clkDiv32 = Clock_New("%s.div32", name);
    rc->clkDiv64 = Clock_New("%s.div64", name);
    rc->clkDiv5 = Clock_New("%s.div5", name);
    rc->clkDiv3 = Clock_New("%s.div3", name);

    Clock_MakeDerived(rc->clkDiv1, rc->clkFDivIn, 1, 1);
    Clock_MakeDerived(rc->clkDiv2, rc->clkFDivIn, 1, 2);
    Clock_MakeDerived(rc->clkDiv4, rc->clkFDivIn, 1, 4);
    Clock_MakeDerived(rc->clkDiv8, rc->clkFDivIn, 1, 8);
    Clock_MakeDerived(rc->clkDiv16, rc->clkFDivIn, 1, 16);
    Clock_MakeDerived(rc->clkDiv32, rc->clkFDivIn, 1, 32);
    Clock_MakeDerived(rc->clkDiv64, rc->clkFDivIn, 1, 64);
    Clock_MakeDerived(rc->clkDiv5, rc->clkFDivIn, 1, 5);
    Clock_MakeDerived(rc->clkDiv3, rc->clkFDivIn, 1, 3);

    /* The output clocks */
    rc->clkFCLK = Clock_New("%s.fclk", name);
    rc->clkICLK = Clock_New("%s.iclk", name);
    Clock_Trace(rc->clkICLK, ICLK_TraceProc, rc);

    rc->clkPCKA = Clock_New("%s.pcka", name);
    rc->clkPCKB = Clock_New("%s.pckb", name);
    rc->clkPCKC = Clock_New("%s.pckc", name);
    rc->clkPCKD = Clock_New("%s.pckd", name);
    rc->clkBCLK = Clock_New("%s.bclk", name);
    rc->clkBCLKPin = Clock_New("%s.bclkpin", name);
    rc->clkUCLK = Clock_New("%s.uclk", name);
    //Clock_t *clkGLCDC; /*  same as PLL Out */
    rc->clkCANMCLK = Clock_New("%s.canmclk", name);
    rc->clkIWDTCLK = Clock_New("%s.iwdtclk", name);
    rc->clkRTCSCLK = Clock_New("%s.rtcsclk", name);
    rc->clkRTCMCLK = Clock_New("%s.rtcmclk", name);
    return &rc->bdev;
}
