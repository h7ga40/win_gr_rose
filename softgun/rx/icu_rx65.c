/**
 *********************************************************************************************
 * RX65 Interrupt controller
 *********************************************************************************************
 */ 
#include "bus.h"
#include "sgstring.h"
#include "signode.h"
#include "icu_rx65.h"
#include "cycletimer.h"
//#include "cpu_rx.h"

 #define RX_ICU_RX65    (3)

#define REG_IR(base,irq)    ((base) + (irq))
#define     IR_IR   (1<<0)
#define REG_DTCER(base,dtc)   ((base) + 0x100 + (dtc))
#define     DTCER_DTCE    (1 << 0)
#define REG_IER(base,ier)   ((base) + 0x200 + (ier))
#define     IER_IEN0    (1 << 0)
#define     IER_IEN1    (1 << 1)
#define     IER_IEN2    (1 << 2)
#define     IER_IEN3    (1 << 3)
#define     IER_IEN4    (1 << 4)
#define     IER_IEN5    (1 << 5)
#define     IER_IEN6    (1 << 6)
#define     IER_IEN7    (1 << 7)
#define REG_SWINTR(base)    ((base) + 0x2E0)
#define     SWINTR_SWINT    (1 << 0)
#define REG_SWINT2R(base)   ((base) + 0x2E1)
#define     SWINT2R_SWINT2  (1 << 0)
#define REG_FIR(base)       ((base) + 0x2F0)
#define     FIR_FVCT_MSK    (0xff)
#define     FIR_FIEN    (1 << 15)
#define REG_IPR(base,ipr)   ((base) + 0x300 + (ipr))
#define     IPR_IPR_MSK (0x0f)
#define REG_DMRSR(base, m) ((base) + 0x400 + ((m) << 2))
#define REG_IRQCR(base,idx) ((base) + 0x500 + (idx))
#define     IRQCR_IRQMD_MSK     (3 << 2)
#define     IRQCR_IRQMD_SHIFT   (2)
#define         IRQMD_LO_LVL    (0)
#define         IRQMD_N_EDGE    (1 << 2)
#define         IRQMD_P_EDGE    (2 << 2)
#define         IRQMD_BOTH_EDGE (3 << 2)
#define REG_IRQFLTE0(base)  ((base) + 0x520)
#define REG_IRQFLTE1(base)  ((base) + 0x521)
#define REG_IRQFLTC0(base)  ((base) + 0x528)
#define REG_IRQFLTC1(base)  ((base) + 0x52A)
#define REG_NMISR(base)     ((base) + 0x580)
#define     NMISR_NMIST     (1 << 0)
#define     NMISR_OSTST     (1 << 1)
#define     NMISR_WDTST     (1 << 2)
#define     NMISR_IWDTST    (1 << 3)
#define     NMISR_LVD1ST    (1 << 4)
#define     NMISR_LVD2ST    (1 << 5)
#define     NMISR_RAMST     (1 << 6)
#define REG_NMIER(base)     ((base) + 0x581)
#define     NMIER_NMIEN     (1 << 0)
#define     NMIER_OSTEN     (1 << 1)
#define     NMIER_WDTEN     (1 << 2)
#define     NMIER_IWDTEN    (1 << 3)
#define     NMIER_LVD1EN    (1 << 4)
#define     NMIER_LVD2EN    (1 << 5)
#define     NMIER_RAMEN     (1 << 6)
#define REG_NMICLR(base)    ((base) + 0x582)
#define     NMICLR_NMICLR   (1 << 0)
#define     NMICLR_OSTCLR   (1 << 1)
#define     NMICLR_WDTCLR   (1 << 2)
#define     NMICLR_IWDTCLR  (1 << 3)
#define     NMICLR_LVD1CLR  (1 << 4)
#define     NMICLR_LVD2CLR  (1 << 5)
#define REG_NMICR(base)     ((base) + 0x583)
#define     NMICR_NMIMD     (1 << 3)
#define REG_NMIFLTE(base)   ((base) + 0x590)
#define     NMIFLTE_NFLTEN  (1 << 0)
#define REG_NMIFLTC(base)   ((base) + 0x594)
#define     NMIFLTC_NFCLKSEL_MSK    (3)

typedef struct ICU ICU;
typedef struct Irq {
    ICU *icu;
    uint8_t irqNr;
    uint8_t regIR;
    uint8_t regDTCER;
    uint8_t regIRQCR;
    SigNode *sigIrq;
    uint8_t ipl;
    struct Irq *prev;
    struct Irq *next;
} Irq;

typedef struct IprToIrq {
    struct IprToIrq *next;
    int16_t irqNr;
} IprToIrq;

struct ICU {
    BusDevice bdev;
    int variant;
    Irq irq[256];
    Irq *firstActiveIrq;
    SigNode *sigIrqAck;
    SigNode *sigSWINT;
    SigNode *sigSWINT2R;
    int8_t regIER[32];
    uint8_t regIPR[256];
    const int16_t *irqToIpr;
    IprToIrq *iprToIrqHead[256]; /* Reverse mapping */
    uint8_t regDMRSR[8];
    uint16_t regIRQFLTE0;
    uint16_t regIRQFLTE1;
    uint16_t regIRQFLTC0;
    uint16_t regIRQFLTC1;
    uint8_t regNMISR;
    uint8_t regNMIER;
    uint8_t regNMICLR;
    uint8_t regNMICR;
    uint8_t regNMIFLTE;
    uint8_t regNMIFLTC;
    uint16_t regFIR;
}; 

static const uint8_t rx65Irqcr[256] = {
/*   0  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*   4  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*   8  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  12  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  16  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  20  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_N_EDGE,
/*  24  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  28  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  32  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  36  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  40  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  44  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  48  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  52  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  56  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  60  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  64  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  68  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  72  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  76  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  80  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  84  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/*  88  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_LO_LVL, IRQMD_LO_LVL,
/*  92  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_LO_LVL, IRQMD_N_EDGE,
/*  96  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 100  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 104  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_LO_LVL, IRQMD_LO_LVL,
/* 108  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_LO_LVL, IRQMD_LO_LVL,
/* 112  */ IRQMD_LO_LVL, IRQMD_LO_LVL, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 116  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_LO_LVL, IRQMD_LO_LVL,
/* 120  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 124  */ IRQMD_LO_LVL, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 128  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 132  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 136  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 140  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 144  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 148  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 152  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 156  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 160  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 164  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 168  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 172  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 176  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 180  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 184  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 188  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 192  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 196  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 200  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 204  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 208  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 212  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 216  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 220  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 224  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 228  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 232  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 236  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 240  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 244  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 248  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
/* 252  */ IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE, IRQMD_N_EDGE,
};

static const int16_t rx65IrqToIpr[256] = {
/*   0 */  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
/*  16 */   0, -1,  0, -1, -1,  1, -1,  2, -1, -1,  3,  3,  4,  5,  6,  7,
/*  32 */  -1, -1, 34, 35, -1, -1, 38, 39, 40, 41, 42, 43, 44, 45, -1, -1,
/*  48 */  -1, -1, 50, 51, 52, 53, 54, 55, -1, -1, 58, 59, 60, 61, 62, 63,
/*  64 */  64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
/*  80 */  80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, -1, 92, 93, -1, 95,
/*  96 */  96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
/* 112 */ 112, 113, 114, 115, 116, 117,  -1,  -1, 120, 121, 122, 123, 124, 125, 126, 127,
/* 128 */ 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 
/* 144 */ 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 157, 159,
/* 160 */ 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
/* 176 */ 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 
/* 192 */ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207,
/* 208 */ 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223,
/* 224 */ 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
/* 240 */ 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255
};

static void
PostInterruptPL(ICU *icu)
{
    Irq *irq = icu->firstActiveIrq;
    if (irq == NULL) {
        RX_PostInterrupt(0, 0);
    } else {
        RX_PostInterrupt(irq->ipl, irq->irqNr);
    }
}

static void
update_interrupt(Irq *irq)
{
    ICU *icu = irq->icu;
    Irq *cursor, *prev;
    int16_t ipl;
    bool ien;
    int16_t ipr_idx;
    if (irq->prev) {
        if(irq->next) {
            irq->next->prev = irq->prev;
        }
        irq->prev->next = irq->next;
        irq->prev = NULL;
        irq->next = NULL;
    } else if(icu->firstActiveIrq == irq) {
        if (irq->next) {
            irq->next->prev = NULL;
        }
        icu->firstActiveIrq = irq->next;
        irq->prev = NULL;
        irq->next = NULL;
    }
    ien = (icu->regIER[irq->irqNr >> 3] >> (irq->irqNr & 7)) & 1; 
    ipr_idx = icu->irqToIpr[irq->irqNr];
    if ((ipr_idx < 0) || (ipr_idx >= array_size(icu->regIPR))) {
        fprintf(stderr, "Bug: No interrupt priority for irq %u\n", irq->irqNr);
    }
    /*
     *****************************************************************************
     * Check if this is the fast interrupt
     * irqN must match, there must be an interrupt request and fien and ien
     * must be both true.
     *****************************************************************************
     */
    if ((irq->irqNr == (icu->regFIR & 0xff)) && (icu->regFIR & FIR_FIEN) 
        && (irq->regIR & IR_IR) && ien)
    {
        fprintf(stderr, "Post firq %u at %lu\n", irq->irqNr, CycleCounter_Get());
        icu->firstActiveIrq = irq;
        irq->ipl = 0xff;
        PostInterruptPL(icu);
        return;
    }
    ipl = icu->regIPR[ipr_idx] & 0xf;
    if ((ien == 0) || !(irq->regIR & IR_IR) || (ipl == 0)) {
        PostInterruptPL(icu);
        return;
    }
    irq->ipl = ipl;
    if (icu->firstActiveIrq == NULL) {
        irq->next = NULL;
        irq->prev = NULL;
        icu->firstActiveIrq = irq;
    } else {
        for (prev = NULL, cursor = icu->firstActiveIrq; cursor;
            prev = cursor, cursor = cursor->next) 
        {
            if ((irq->ipl > cursor->ipl) ||
                ((irq->ipl == cursor->ipl) && (irq->irqNr > cursor->irqNr))) {
                irq->next = cursor;
                irq->prev = cursor->prev;
                cursor->prev = irq;
                if (prev) {
                    prev->next = irq;
                } else {
                    icu->firstActiveIrq = irq;
                }
                break;
            }     
        }
        if (prev && !cursor) {
            irq->next = NULL;
            irq->prev = prev;
            prev->next = irq;
        }
    }
    PostInterruptPL(icu);
}

/**
 *****************************************************************************************************
 * AckInterrupt is called by the CPU. It acknowledges the first active IRQ because 
 * this was the last one which was postet to the CPU
 * The interrupt is acknoledged on negative edge currently only
 *****************************************************************************************************
 */
static void
AckInterrupt(SigNode *sig, int value, void *eventData)
{
    ICU *icu = eventData;
    Irq *irq = icu->firstActiveIrq;
    if (value != SIG_LOW) {
        return;
    }
    if (!irq) {
        fprintf(stderr, "Bug: CPU acked non posted interrupt\n");
        return;
    }
    switch (irq->regIRQCR & IRQCR_IRQMD_MSK) {
            case IRQMD_N_EDGE:
            case IRQMD_P_EDGE:
            case IRQMD_BOTH_EDGE:
                irq->regIR &= ~IR_IR;
                update_interrupt(irq);
                break;
    }
}

static void
update_level_interrupt(Irq *irq) 
{
    switch(irq->regIRQCR & IRQCR_IRQMD_MSK) {
        case IRQMD_LO_LVL:
        if(SigNode_Val(irq->sigIrq) == SIG_LOW) {
            irq->regIR |= IR_IR;
        } else {
            irq->regIR &= ~IR_IR;
        } 
        update_interrupt(irq);
        break;
    }
}

static uint32_t
ir_read(void *clientData, uint32_t address, int rqlen)
{
    Irq *irq = clientData;
    return irq->regIR;
}

static void 
ir_write(void *clientData, uint32_t value, uint32_t address, int rqlen) 
{
    Irq *irq = clientData;
    switch (irq->regIRQCR & IRQCR_IRQMD_MSK)
    {
        case IRQMD_LO_LVL:
            break;
        case IRQMD_N_EDGE:
        case IRQMD_P_EDGE:
        case IRQMD_BOTH_EDGE:
            irq->regIR &= value & 1;
            update_interrupt(irq);
            break; 
    }
}

static uint32_t
dtcer_read(void *clientData, uint32_t address, int rqlen)
{
    Irq *irq = clientData;
    return irq->regDTCER;
}

static void
dtcer_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    Irq *irq = clientData;
    irq->regDTCER = value & DTCER_DTCE;
}

static uint32_t
ier_read(void *clientData, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    unsigned int idx = address % array_size(icu->regIER);
    return icu->regIER[idx];
}

static void
ier_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    int i;
    unsigned int idx = address % array_size(icu->regIER);
    uint8_t diff = icu->regIER[idx] ^ value;
    icu->regIER[idx] = value;
    for (i = 0; i < 8; i++) {
        if (diff & (1 << i)) {
            Irq *irq = &icu->irq[8 * idx + i];
            update_interrupt(irq);
        }
    }
}

static uint32_t
swintr_read(void *clientData, uint32_t address, int rqlen) 
{
    return 0;
}

static void
swintr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    if (value & SWINTR_SWINT) {
        SigNode_Set(icu->sigSWINT, SIG_LOW);
        SigNode_Set(icu->sigSWINT, SIG_HIGH);
    }
}

static uint32_t
swint2r_read(void *clientData, uint32_t address, int rqlen) 
{
    return 0;
}

static void
swint2r_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    if (value & SWINT2R_SWINT2) {
        SigNode_Set(icu->sigSWINT2R, SIG_LOW);
        SigNode_Set(icu->sigSWINT2R, SIG_HIGH);
    }
}

static uint32_t
fir_read(void *clientData, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    return icu->regFIR;
}

static void
fir_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    Irq *irq, *oldirq;
    bool ien;
    fprintf(stderr, "*** FIR ena %u fir %u\n", !!(value & FIR_FIEN), value & 0xff);
    irq = &icu->irq[value & 0xff];
    ien = (icu->regIER[irq->irqNr >> 3] >> (irq->irqNr & 1)) & 1;
    if (icu->regFIR & FIR_FIEN) {
        oldirq = &icu->irq[icu->regFIR & 0xff];
        icu->regFIR = value & 0x80ff;
        update_interrupt(oldirq);
    }
    icu->regFIR = value & 0x80ff;
    if (value & FIR_FIEN) {
        update_interrupt(irq);
    }
}

static uint32_t
ipr_read(void *clientData, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    unsigned int idx = address & 0xff;
    if (idx > array_size(icu->regIPR)) {
        fprintf(stderr, "Illegal index %d of IPR register\n", idx);
        return 0;
    }
    return icu->regIPR[idx];
}

static void
ipr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    IprToIrq *cursor;
    unsigned int idx = address & 0xff;
    if (idx > array_size(icu->regIPR)) {
        fprintf(stderr, "Illegal index %d of IPR register\n", idx);
        return;
    }
    icu->regIPR[idx] = value;
    for (cursor = icu->iprToIrqHead[idx]; cursor; cursor = cursor->next) {
        int16_t irqNr = cursor->irqNr;
        Irq *irq; 
        if ((irqNr < 0) || (irqNr > 255)) {
            fprintf(stderr, "Bug: Bad irqList\n");
            __builtin_trap();
        }
        irq = &icu->irq[irqNr];
        fprintf(stderr, "IPR write IRQ %u addr %08x\n", irq->irqNr, address);
        update_level_interrupt(irq);
    }
}

static uint32_t
dmrsr_read(void *clientData, uint32_t address, int rqlen)
{
    return 0;
}

static void
dmrsr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
}

static uint32_t
irqcr_read(void *clientData, uint32_t address, int rqlen)
{
    Irq *irq = clientData;
    return irq->regIRQCR;
}

static void
irqcr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    Irq *irq = clientData;
    irq->regIRQCR = value;
    update_level_interrupt(irq);
}


static uint32_t
irqflte0_read(void *clientData, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    return icu->regIRQFLTE0;
}

static void 
irqflte0_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    icu->regIRQFLTE0 = value;
}

static uint32_t
irqflte1_read(void *clientData, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    return icu->regIRQFLTE1;
}

static void 
irqflte1_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    icu->regIRQFLTE1 = value;
}

static uint32_t
irqfltc0_read(void *clientData, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    return icu->regIRQFLTC0;
}

static void 
irqfltc0_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    icu->regIRQFLTC0 = value;
}

static uint32_t
irqfltc1_read(void *clientData, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    return icu->regIRQFLTC1;
}

static void 
irqfltc1_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    ICU *icu = clientData;
    icu->regIRQFLTC1 = value;
}

static uint32_t
nmisr_read(void *clientData, uint32_t address, int rqlen)
{
    return 0;
}
static void
nmisr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    return;
}

static uint32_t
nmier_read(void *clientData, uint32_t address, int rqlen)
{
    return 0;
}
static void
nmier_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    return;
}

static uint32_t
nmiclr_read(void *clientData, uint32_t address, int rqlen)
{
    return 0;
}
static void
nmiclr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    return;
}

static uint32_t
nmicr_read(void *clientData, uint32_t address, int rqlen)
{
    return 0;
}
static void
nmicr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    return;
}

static uint32_t
nmiflte_read(void *clientData, uint32_t address, int rqlen)
{
    return 0;
}
static void
nmiflte_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    return;
}

static uint32_t
nmifltc_read(void *clientData, uint32_t address, int rqlen)
{
    return 0;
}
static void
nmifltc_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
    return;
}

static void
SigIrqTraceProc(SigNode *sig, int value, void *eventData)
{
    Irq *irq = eventData;
    uint8_t old = irq->regIR;
    uint8_t diff;
    switch (irq->regIRQCR & IRQCR_IRQMD_MSK) {
        case IRQMD_LO_LVL:
            if (value == SIG_LOW) {
                irq->regIR |= IR_IR;
            } else { 
                irq->regIR &= ~IR_IR;
            }
            break;

        case IRQMD_N_EDGE:
            if (value == SIG_LOW) {
                irq->regIR |= IR_IR;
            }
            break;

        case IRQMD_P_EDGE:
            if (value == SIG_HIGH) {
                irq->regIR |= IR_IR;
            }
            break;

        case IRQMD_BOTH_EDGE:
            irq->regIR |= IR_IR;
            break;
    }
    diff = old ^ irq->regIR;
    if (diff) {
        update_interrupt(irq);
    }
}

static void
ICU_UnMap(void *module_owner, uint32_t base, uint32_t mapsize)
{
    ICU *icu = module_owner;
    unsigned int i;
    for (i = 0; i < array_size(icu->irq); i++) {
        IOH_Delete8(REG_IR(base, i));
        IOH_Delete8(REG_DTCER(base, i));
    }
    for (i = 0; i < array_size(icu->regIER); i++) {
        IOH_Delete8(REG_IER(base, i));
    }
    IOH_Delete8(REG_SWINTR(base));
    IOH_Delete8(REG_SWINT2R(base));
    IOH_Delete8(REG_FIR(base));
    for (i = 0; i < array_size(icu->regIPR); i++) {
        IOH_Delete8(REG_IPR(base, i));
    }
    for (i = 0; i < array_size(icu->regDMRSR); i++) {
        IOH_Delete8(REG_DMRSR(base, i));
    }
    for (i = 0; i < 16; i++) {
        IOH_Delete8(REG_IRQCR(base, i));
    }
    if (icu->variant == RX_ICU_RX65) {
        IOH_Delete8(REG_IRQFLTE0(base)); 
        IOH_Delete8(REG_IRQFLTE1(base)); 
        IOH_Delete16(REG_IRQFLTC0(base)); 
        IOH_Delete16(REG_IRQFLTC1(base)); 
        IOH_Delete8(REG_NMIFLTE(base)); 
        IOH_Delete8(REG_NMIFLTC(base)); 
    }
    IOH_Delete8(REG_NMISR(base));
    IOH_Delete8(REG_NMIER(base));
    IOH_Delete8(REG_NMICLR(base)); 
    IOH_Delete8(REG_NMICR(base));
}

static void
ICU_Map(void *module_owner, uint32_t base, uint32_t mapsize, uint32_t flags)
{
    ICU *icu = module_owner;
    unsigned int i;
    for (i = 0; i < array_size(icu->irq); i++) {
        Irq *irq = &icu->irq[i];
        IOH_New8(REG_IR(base, i), ir_read, ir_write, irq);
        IOH_New8(REG_DTCER(base, i), dtcer_read, dtcer_write, irq);
    }
    for (i = 0; i < array_size(icu->regIER); i++) {
        IOH_New8(REG_IER(base, i), ier_read, ier_write, icu);
    }
    IOH_New8(REG_SWINTR(base), swintr_read, swintr_write, icu);
    IOH_New8(REG_SWINT2R(base), swint2r_read, swint2r_write, icu);
    IOH_New8(REG_FIR(base), fir_read, fir_write, icu);
    for (i = 0; i < array_size(icu->regIPR); i++) {
        IOH_New8(REG_IPR(base, i), ipr_read, ipr_write, icu);
    }
    for (i = 0; i < array_size(icu->regDMRSR); i++) {
        IOH_New8(REG_DMRSR(base, i), dmrsr_read, dmrsr_write, icu);
    }
    for (i = 0; i < 16; i++) {
        IOH_New8(REG_IRQCR(base, i), irqcr_read, irqcr_write, &icu->irq[i + 64]);
    }
    if (icu->variant == RX_ICU_RX65) {
        IOH_New8(REG_IRQFLTE0(base), irqflte0_read, irqflte0_write, icu); 
        IOH_New8(REG_IRQFLTE1(base), irqflte1_read, irqflte1_write, icu); 
        IOH_New16(REG_IRQFLTC0(base), irqfltc0_read, irqfltc0_write, icu); 
        IOH_New16(REG_IRQFLTC1(base), irqfltc1_read, irqfltc1_write, icu); 
        IOH_New8(REG_NMIFLTE(base), nmiflte_read, nmiflte_write, icu); 
        IOH_New8(REG_NMIFLTC(base), nmifltc_read, nmifltc_write, icu); 
    }
    IOH_New8(REG_NMISR(base), nmisr_read, nmisr_write, icu);
    IOH_New8(REG_NMIER(base), nmier_read, nmier_write, icu);
    IOH_New8(REG_NMICLR(base), nmiclr_read, nmiclr_write, icu); 
    IOH_New8(REG_NMICR(base), nmicr_read, nmicr_write, icu);
}

BusDevice *
RX65ICU_New(const char *name, int variant)
{
    int i;
    ICU *icu = sg_new(ICU);
    icu->variant = variant;
    switch (variant) 
    {
        case RX_ICU_RX65:
            icu->irqToIpr = rx65IrqToIpr; 
            break;
        default:
            fprintf(stderr, "Unknown RX ICU variant %d\n", variant);
            __builtin_trap();
    }
    icu->bdev.first_mapping = NULL;
    icu->bdev.Map = ICU_Map;
    icu->bdev.UnMap = ICU_UnMap;
    icu->bdev.owner = icu;
    icu->bdev.hw_flags = MEM_FLAG_READABLE | MEM_FLAG_WRITABLE;
    for(i = 0; i < 256; i ++) {
         IprToIrq *iprToIrq;
         int ipr;

         ipr = icu->irqToIpr[i];
         if(ipr < 0) {
             continue;
         }
         //fprintf(stderr,"Irq %u ipr %u\n",i,ipr);
         iprToIrq = sg_new(IprToIrq);
         iprToIrq->irqNr = i;
         /* Insert to linked list */
         iprToIrq->next = icu->iprToIrqHead[ipr];
         icu->iprToIrqHead[ipr] = iprToIrq;
    }
    for (i = 0; i < 256; i++) {
        Irq *irq = &icu->irq[i];
        irq->icu = icu;
        irq->irqNr = i;
        switch (variant) {
            case RX_ICU_RX65:
                irq->regIRQCR = rx65Irqcr[i];
                break;
            default:
                fprintf(stderr, "Unknown RX ICU variant %d\n", variant);
                __builtin_trap();
        }
        irq->sigIrq = SigNode_New("%s.irq%d", name, i);
        if (!irq->sigIrq) {
            fprintf(stderr, "Can not create interrupt line\n");
            __builtin_trap();
        }
        SigNode_Trace(irq->sigIrq, SigIrqTraceProc, irq);
    }
    icu->sigIrqAck = SigNode_New("%s.irqAck", name);
    icu->sigSWINT = SigNode_New("%s.swint", name);
    icu->sigSWINT2R = SigNode_New("%s.swint2", name);
    if (!icu->sigIrqAck || !icu->sigSWINT) {
        fprintf(stderr, "%s: can not create IRQ line\n", name); 
        __builtin_trap();
    }
    SigNode_Trace(icu->sigIrqAck, AckInterrupt, icu);
    fprintf(stderr, "Created RX65 Interrupt Control Unit\n");
    return &icu->bdev;
}
