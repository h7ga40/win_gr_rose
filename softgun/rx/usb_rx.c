/**
 */
#include "sgstring.h"
#include "bus.h"
#include "usb_rx.h"
#include "cycletimer.h"
#include "signode.h"
#include "usbdevice.h"
#include "usbstdrq.h"
#include <assert.h>

#define REG_SYSCFG(base)	((base) + 0x00)
#define		SYSCFG_SCKE	(1 << 10)
#define		SYSCFG_DCFM	(1 << 6)
#define		SYSCFG_DRPD	(1 << 5)
#define		SYSCFG_DPRPU	(1 << 4)
#define		SYSCFG_USBE	(1 << 0)
#define REG_SYSSTS0(base)	((base) + 0x04)
#define		SYSSTS0_OVCMON_MSK	(3 << 14)
#define		SYSSTS0_HTACT		(1 << 6)
#define		SYSSTS0_IDMON		(1 << 2)
#define		SYSSTS0_LNST		(3 << 0)
#define REG_DVSTCTR0(base)	((base) + 0x08)
#define		DVSTCTR0_NNPBTOA	(1 << 11)
#define		DVSTCTR0_EXICEN		(1 << 10)
#define		DVSTCTR0_VBUSEN		(1 << 9)
#define 	DVSTCTR0_WKUP		(1 << 8)
#define		DVSTCTR0_RWUPE		(1 << 7)
#define		DVSTCTR0_USBRST		(1 << 6)
#define		DVSTCTR0_RESUME		(1 << 5)
#define		DVSTCTR0_UACT		(1 << 4)
#define		DVSTCTR0_RHST_MSK	(7)

#define REG_CFIFO(base)		((base) + 0x14)
#define REG_D0FIFO(base)	((base) + 0x18)
#define REG_D1FIFO(base)	((base) + 0x1c)
#define REG_CFIFOSEL(base)	((base) + 0x20)
#define		FIFOSEL_RCNT		(1 << 15)
#define		FIFOSEL_REW		(1 << 14)
#define		FIFOSEL_MBW		(1 << 10)
#define 	FIFOSEL_BIGEND		(1 << 8)
#define		FIFOSEL_ISEL		(1 << 5)
#define		FIFOSEL_CURPIPE_MSK	(0xf)

#define REG_CFIFOCTR(base)	((base) + 0x22)
#define		FIFOCTR_BVAL	(1 << 15)
#define		FIFOCTR_BCLR	(1 << 14)
#define		FIFOCTR_FRDY	(1 << 13)
#define		FIFOCTR_DTLN_MSK	(0x1ff)

#define REG_D0FIFOSEL(base)	((base) + 0x28)
#define REG_D0FIFOCTR(base)	((base) + 0x2a)
#define REG_D1FIFOSEL(base)	((base) + 0x2c)
#define REG_D1FIFOCTR(base)	((base) + 0x2e)
#define REG_INTENB0(base)	((base) + 0x30)
#define		INTENB0_VBSE	(1 << 15)
#define		INTENB0_RSME	(1 << 14)
#define		INTENB0_SOFE	(1 << 13)
#define		INTENB0_DVSE	(1 << 12)
#define		INTENB0_CTRE	(1 << 11)
#define		INTENB0_BEMPE	(1 << 10)
#define		INTENB0_NRDYE	(1 << 9)
#define		INTENB0_BRDYE	(1 << 8)
#define REG_INTENB1(base)	((base) + 0x32)
#define		INTENB1_OVRCHRE	(1 << 15)
#define		INTENB1_BCHGE	(1 << 14)
#define		INTENB1_DTCHE	(1 << 12)
#define		INTENB1_ATTCHE	(1 << 11)
#define		INTENB1_EOFERRE	(1 << 6)
#define		INTENB1_SIGNE	(1 << 5)
#define		INTENB1_SACKE	(1 << 4)

#define REG_BRDYENB(base)	((base) + 0x36)
#define REG_NRDYENB(base)	((base) + 0x38)
#define REG_BEMPENB(base)	((base) + 0x3a)
#define		BEMPE_PIPO0	(1 << 0)
#define		BEMPE_PIPO1	(1 << 1)
#define		BEMPE_PIPO2	(1 << 2)
#define		BEMPE_PIPO3	(1 << 3)
#define		BEMPE_PIPO4	(1 << 4)
#define		BEMPE_PIPO5	(1 << 5)
#define		BEMPE_PIPO6	(1 << 6)
#define		BEMPE_PIPO7	(1 << 7)
#define		BEMPE_PIPO8	(1 << 8)
#define		BEMPE_PIPO9	(1 << 9)

#define REG_SOFCFG(base)	((base) + 0x3c)
#define		SOCFG_TRNENSEL	(1 << 8)
#define		SOCFG_BRDYM	(1 << 6)
#define 	SOCFG_EDGESTS	(1 << 4)

#define REG_INTSTS0(base)	((base) + 0x40)
#define		INTSTS0_VBINT		(1 << 15)
#define		INTSTS0_RESM		(1 << 14)
#define		INTSTS0_SOFR		(1 << 13)
#define		INTSTS0_DVST		(1 << 12)
#define		INTSTS0_CTRT		(1 << 11)
#define		INTSTS0_BEMP		(1 << 10)
#define		INTSTS0_NRDY		(1 << 9)
#define		INTSTS0_BRDY		(1 << 8)
#define		INTSTS0_VBSTS		(1 << 7)
#define		INTSTS0_DVSQ_OFS	(4)
#define		INTSTS0_DVSQ_MSK	(7 << INTSTS0_DVSQ_OFS)
#define		INTSTS0_VALID		(1 << 3)
#define		INTSTS0_CTSQ_OFS	(0)
#define		INTSTS0_CTSQ_MSK	(7 << INTSTS0_CTSQ_OFS)

#define		DVSQ_POWS	(0)	/* Powered state */
#define		DVSQ_DEFS	(1)	/* Default state */
#define		DVSQ_ADRS	(2)	/* Address state */
#define		DVSQ_CFGS	(3)	/* Configured state */
#define		DVSQ_SUSF	(4)	/* Suspended state */

#define		CTSQ_IDST	(0)	/* Idle or setup stage */
#define		CTSQ_RDDS	(1)	/* Ctrl read data stage */
#define		CTSQ_RDSS	(2)	/* Ctrl read status stage */
#define   	CTSQ_WRDS	(3)	/* Ctrl write data stage */
#define   	CTSQ_WRSS	(4)	/* Ctrl write status stage */
#define 	CTSQ_WRNDSS	(5)	/* Ctrl write (no data) status stage */
#define		CTSQ_TSQERR	(6)	/* Ctrl transfer sequence error */

#define REG_INTSTS1(base)	((base) + 0x42)
#define		INTSTS1_OVRCR		(1 << 15)
#define		INTSTS1_BCHG		(1 << 14)
#define		INTSTS1_DTCH		(1 << 12)
#define		INTSTS1_ATTCH		(1 << 11)
#define		INTSTS1_EOFERR		(1 << 6)
#define		INTSTS1_SIGN		(1 << 5)
#define		INTSTS1_SACK		(1 << 4)

#define REG_BRDYSTS(base)	((base) + 0x46)
#define REG_NRDYSTS(base)	((base) + 0x48)
#define REG_BEMPSTS(base)	((base) + 0x4a)
#define REG_FRMNUM(base)	((base) + 0x4c)
#define		FRMNUM_OVRN	(1 << 15)
#define		FRMNUM_CRCE	(1 << 14)

#define REG_DVCHGR(base)	((base) + 0x4e)
#define		DVCHGR_DVCHG	(1 << 15)

#define REG_USBADDR(base)	((base) + 0x50)
#define		USBADDR_STSRECONV_MSK	(0xf << 8)
#define		USBADDR_ADDR_MSK	(0x7f)

#define REG_USBREQ(base)	((base) + 0x54)
#define REG_USBVAL(base)	((base) + 0x56)
#define REG_USBINDX(base)	((base) + 0x58)
#define REG_USBLENG(base)	((base) + 0x5a)
#define REG_DCPCFG(base)	((base) + 0x5c)
#define		DCPCFG_SHTNAK	(1 << 7)
#define		DCPCFG_DIR	(1 << 4)
#define REG_DCPMAXP(base)	((base) + 0x5e)
#define		DCPMAXP_DEVSEL_MSK	(0xf << 12)
#define		DCPMAXP_MXPS_MSK	(0x7f)
#define REG_DCPCTR(base)	((base) + 0x60)
#define		DCPCTR_BSTS		(1 << 15)
#define		DCPCTR_SUREQ	(1 << 14)
#define		DCPCTR_SUREQCLR	(1 << 11)
#define		DCPCTR_SQCLR	(1 << 8)
#define		DCPCTR_SQSET	(1 << 7)
#define		DCPCTR_SQMON	(1 << 6)
#define		DCPCTR_PBUSY	(1 << 5)
#define		DCPCTR_CCPL		(1 << 2)
#define		DCPCTR_PID_MSK	(3)

#define		PID_NAK			(0)
#define		PID_BUF			(1)
#define		PID_STALL		(2)

#define REG_PIPESEL(base)	((base) + 0x64)
#define		PIPESEL_PIPESEL_MSK	(0xf)
#define REG_PIPECFG(base)	((base) + 0x68)
#define		PIPECFG_TYPE_OFS	(14)
#define		PIPECFG_TYPE_MSK	(3 << PIPECFG_TYPE_OFS)
#define		PIPECFG_BFRE		(1 << 10)
#define		PIPECFG_DBLB		(1 << 9)
#define		PIPECFG_SHTNAK		(1 << 7)
#define		PIPECFG_DIR			(1 << 4)
#define		PIPECFG_EPNUM_OFS	(0)
#define		PIPECFG_EPNUM_MSK	(0xf << PIPECFG_EPNUM_OFS)
#define REG_PIPEMAXP(base)	((base) + 0x6c)
#define		PIPEMAXP_MXPS_MSK	(0x1ff)
#define		PIPEMAXP_DEVSEL_MSK	(0xf << 12)

#define		TYPE_BULK			(1)
#define		TYPE_INTERRUPT		(2)
#define		TYPE_ISOCHRONOUS	(3)

#define REG_PIPEPERI(base)	((base) + 0x6e)
#define		PIPEPERI_IFIS	(	(1 << 12)
#define		PIPEPERI_IITV_MSK	(0x7)
#define REG_PIPE1CTR(base)	((base) + 0x70)
#define		PIPE15CTR_BSTS		(1 << 15)
#define		PIPE15CTR_INBUFM	(1 << 14)
#define		PIPE15CTR_ATREPM	(1 << 10)
#define		PIPE15CTR_ACLRM		(1 << 9)
#define		PIPE15CTR_SQCLR		(1 << 8)
#define		PIPE15CTR_SQSET		(1 << 7)
#define		PIPE15CTR_SQMON		(1 << 6)
#define		PIPE15CTR_PBUSY		(1 << 5)
#define		PIPE15CTR_PID_MSK	(3)
#define REG_PIPE2CTR(base)	((base) + 0x72)
#define REG_PIPE3CTR(base)	((base) + 0x74)
#define REG_PIPE4CTR(base)	((base) + 0x76)
#define REG_PIPE5CTR(base)	((base) + 0x78)
#define REG_PIPE6CTR(base)	((base) + 0x7a)
#define		PIPE69CTR_BSTS		(1 << 15)
#define		PIPE69CTR_ACLRM		(1 << 9)
#define		PIPE69CTR_SQCLR		(1 << 8)
#define		PIPE69CTR_SQSET		(1 << 7)
#define		PIPE69CTR_SQMON		(1 << 6)
#define		PIPE69CTR_PBUSY		(1 << 5)
#define		PIPE69CTR_PID_MSK	(3)

#define REG_PIPE7CTR(base)	((base) + 0x7c)
#define REG_PIPE8CTR(base)	((base) + 0x7e)
#define REG_PIPE9CTR(base)	((base) + 0x80)
#define REG_PIPE1TRE(base)	((base) + 0x90)
#define		PIPETRE_TRENB	(1 << 9)
#define		PIPETRE_TRCLR	(1 << 8)

#define REG_PIPE1TRN(base)	((base) + 0x92)
#define REG_PIPE2TRE(base)	((base) + 0x94)
#define REG_PIPE2TRN(base)	((base) + 0x96)
#define REG_PIPE3TRE(base)	((base) + 0x98)
#define REG_PIPE3TRN(base)	((base) + 0x9a)
#define REG_PIPE4TRE(base)	((base) + 0x9c)
#define REG_PIPE4TRN(base)	((base) + 0x9e)
#define REG_PIPE5TRE(base)	((base) + 0xa0)
#define REG_PIPE5TRN(base)	((base) + 0xa2)
#define REG_DEVADD0(base)	((base) + 0xd0)
#define		DEVADDR_USBSPD_MSK	(3 << 6)
#define REG_DEVADD1(base)	((base) + 0xd2)
#define REG_DEVADD2(base)	((base) + 0xd4)
#define REG_DEVADD3(base)	((base) + 0xd6)
#define REG_DEVADD4(base)	((base) + 0xd8)
#define REG_DEVADD5(base)	((base) + 0xda)

#define REG_DPUSR0R		0x000a0400
#define		DPUSR0R_DVBSTS1	(1 << 31)
#define		DPUSR0R_DOVCB1	(1 << 29)
#define		DPUSR0R_DOVCA1	(1 << 28)
#define		DPUSR0R_DM1	(1 << 25)
#define		DPUSR0R_DP1	(1 << 24)
#define		DPUSR0R_DVBSTS0	(1 << 23)
#define		DPUSR0R_DOVCB0	(1 << 21)
#define		DPUSR0R_DOVCA0	(1 << 20)
#define		DPUSR0R_DM0	(1 << 17)
#define		DPUSR0R_DP0	(1 << 16)
#define		DPUSR0R_FIXPHY1	(1 << 12)
#define		DPUSR0R_SRPC1	(1 << 8)
#define		DPUSR0R_FIXPHY0	(1 << 4)
#define		DPUSR0R_SRPC0	(1 << 0)
#define REG_DPUSR1R		0x000a0404
#define		DPUSR1R_DVBINT1		(1 << 31)
#define		DPUSR1R_DOVRCRB1	(1 << 29)
#define		DPUSR1R_DOVRCRA1	(1 << 28)
#define		DPUSR1R_DMINT1		(1 << 25)
#define		DPUSR1R_DPINT1		(1 << 24)
#define		DPUSR1R_DVBINT0		(1 << 23)
#define		DPUSR1R_DOVRCRB0	(1 << 21)
#define		DPUSR1R_DOVRCRA0	(1 << 20)
#define		DPUSR1R_DMINT0		(1 << 25)
#define		DPUSR1R_DPINT0		(1 << 16)

#define		DPUSR1R_DVBSE1		(1 << 15)
#define		DPUSR1R_DOVRCRBE1	(1 << 13)
#define		DPUSR1R_DOVRCRAE1	(1 << 12)
#define		DPUSR1R_DMINTE1	(1 << 9)
#define		DPUSR1R_DPINTE1	(1 << 8)
#define		DPUSR1R_DVBSE0		(1 << 7)
#define		DPUSR1R_DOVRCRBE0	(1 << 5)
#define		DPUSR1R_DOVRCRAE0	(1 << 4)
#define		DPUSR1R_DMINTE0	(1 << 1)
#define		DPUSR1R_DPINTE0	(1 << 0)

#define USB_FIFO_BUF_SZ 256

typedef struct usb_fifo_t {
	unsigned char buf[USB_FIFO_BUF_SZ];
	int pos_w;
	int pos_r;
} usb_fifo_t;

typedef struct RXUsbPipe {
	uint16_t regPIPECFG;
	uint16_t regPIPEMAXP;
	uint16_t regPIPEPERI;
	uint16_t regPIPECTR;
	uint16_t regPIPETRE;
	uint16_t regPIPETRN;
	bool isel;
	usb_fifo_t fifo;
	bool frdy;
	bool brdy;
	bool nrdy;
	bool bemp;
} RXUsbPipe;

typedef struct RXUsb {
	BusDevice bdev;
	uint16_t regSYSCFG;
	uint16_t regSYSSTS0;
	uint16_t regDVSTCTR0;

	uint16_t regCFIFOSEL;
	uint16_t regCFIFOCTR;
	uint16_t regD0FIFOSEL;
	uint16_t regD0FIFOCTR;
	uint16_t regD1FIFOSEL;
	uint16_t regD1FIFOCTR;
	uint16_t regINTENB0;
	uint16_t regINTENB1;
	uint16_t regBRDYENB;
	uint16_t regNRDYENB;
	uint16_t regBEMPENB;
	uint16_t regSOFCFG;
	uint16_t regINTSTS0;
	uint16_t regINTSTS1;
	uint16_t regFRMNUM;
	uint16_t regDVCHGR;
	uint16_t regUSBADDR;
	uint16_t regUSBREQ;
	uint16_t regUSBVAL;
	uint16_t regUSBINDX;
	uint16_t regUSBLENG;

	uint16_t regPIPESEL;
	RXUsbPipe Pipe[10];

	uint16_t regDEVADD0;
	uint16_t regDEVADD1;
	uint16_t regDEVADD2;
	uint16_t regDEVADD3;
	uint16_t regDEVADD4;
	uint16_t regDEVADD5;

	SigNode *sigIrq;
	CycleTimer sofTimer;
	bool int_req;

	int Counter;
	uint16_t DevSeq;
	uint16_t CtrSeq;
	uint16_t HostFrmSeq;
	uint16_t HostPipe;
	uint16_t HostStupSeq;
	bool HostInRecv;
} RXUsb;

static void
usb_fifo_clear(usb_fifo_t *fifo)
{
	fifo->pos_r = fifo->pos_w = 0;
}

static int
usb_fifo_length(usb_fifo_t *fifo)
{
	int pos_w = fifo->pos_w;
	int pos_r = fifo->pos_r;
	if (pos_r > pos_w)
		pos_w += sizeof(fifo->buf);
	return pos_w - pos_r;
}

static bool
usb_fifo_is_writeble(usb_fifo_t *fifo)
{
	int pos_w = fifo->pos_w + 1;
	if (pos_w >= sizeof(fifo->buf))
		pos_w = 0;
	return fifo->pos_r != pos_w;
}

static void
usb_fifo_write(usb_fifo_t *fifo, uint8_t c)
{
	fifo->buf[fifo->pos_w++] = c;
	if (fifo->pos_w >= sizeof(fifo->buf))
		fifo->pos_w = 0;
}

static bool
usb_fifo_is_readable(usb_fifo_t *fifo)
{
	return fifo->pos_r != fifo->pos_w;
}

static uint8_t
usb_fifo_read(usb_fifo_t *fifo)
{
	uint8_t c = fifo->buf[fifo->pos_r++];
	if (fifo->pos_r >= sizeof(fifo->buf))
		fifo->pos_r = 0;
	return c;
}

/*
 *******************************************************************************
 * Registers Initialized when USBE is written 0 in function mode
 * (docu says on write, not on edge but i should check the real hardware)
 * Table 28.4
 *******************************************************************************
 */
#if 0
static void
usbe0_function(RXUsb *ru)
{
	//ru->regSYSSTS0 LNST
	// RHST
	//DVSQ
	// USBADDR
	//USBREQ BREQUEST,BMREQUESTTYPE
	//WVALUE
	//WINDEX
	//WLENGHT
}
#endif
/**
 ***********************************************************************
 * Registers initialized when USBE is written 0 in host mode
 ***********************************************************************
 */
#if 0
static void
usbe0_host(RXUsb *ru)
{
	//RHST
	//FRNM
}
#endif

/**
 ********************************************************************
 * Bit 0: USBE USB Module enable
 * Bit 4: DPRPU D+ Line Pullup Control
 * Bit 5: DRPD D+/D- Pulldown control
 * Bit 6: DCFM Controller function select 0 = function 1 = host
 * Bit 10: SCKE	USB Module Clock Enable
 *********************************************************************
 */
static uint32_t
syscfg_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regSYSCFG;
}

static void
syscfg_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regSYSCFG = value;

	if (ru->regSYSCFG & SYSCFG_USBE) {
		CycleTimer_Mod(&ru->sofTimer, CyclesToMilliseconds(1));
	}
	else {
		CycleTimer_Remove(&ru->sofTimer);
	}
}

/*
 ************************************************************************
 * Register SYSSTS0
 * Bits 14+15: OVCMON Overcurrent monitor
 * Bit 6: HTACT Host sequencer active 0 = stopped 1 = not stopped
 * Bit 2: IDMON Status of the ID pin
 * Bits 0 + 1: Linestatus 0 = SE0, 1 = J, 2 = K, 3 = SE1
 ************************************************************************
 */
static uint32_t
syssts0_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regSYSSTS0;
}

static void
syssts0_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regSYSSTS0 = value;
}

/**
 **************************************************************************************
 * DVSTCTR0 Device status control register
 * RHST 0-2: 0 Speed not determined, 1 = LS, 2 = FS. FUN: 2 = FS 4 = Reset in progress
 * Bit 4: UACT 0 = Disa downstream, 1 = enable downstream (SOF)
 * Bit 5: RESUME 1 = output a resume signal
 * Bit 6: USBRST 1 = output a ru reset signal
 * Bit 7: RWUPE 1 = Downstream wakeup enabled
 * Bit 8: WKUP 1 = output a remote wake up signal.
 * Bit 9: VBUSEN The VBUSEN
 * Bit 10: EXICEN The Status of the external exicen pin.
 * Bit 11: Host negotiation protocol
 **************************************************************************************
 */
static uint32_t
dvstctr0_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regDVSTCTR0;
}

static void
dvstctr0_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regDVSTCTR0 = value;
}

/**
 *************************************************************************************************
 * Register CFIFO
 * 8/16 Bit control fifo register. The assignment to a pipe is done with the cfifosel register.
 *************************************************************************************************
 */
static uint32_t
cfifo_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = 0;
	int curpipe = ru->regCFIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (usb_fifo_is_readable(&pipe->fifo))
		result |= usb_fifo_read(&pipe->fifo);

	// 16bit
	if ((rqlen > 1) && (ru->regCFIFOSEL & FIFOSEL_MBW)) {
		if (ru->regCFIFOSEL & FIFOSEL_BIGEND) {
			result <<= 8;
			if (usb_fifo_is_readable(&pipe->fifo))
				result |= usb_fifo_read(&pipe->fifo);
		}
		else {
			if (usb_fifo_is_readable(&pipe->fifo))
				result |= usb_fifo_read(&pipe->fifo) << 8;
		}
	}

	return result;
}

static void
cfifo_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regCFIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	// 16bit
	if ((rqlen > 1) && (ru->regCFIFOSEL & FIFOSEL_MBW)) {
		if (ru->regCFIFOSEL & FIFOSEL_BIGEND) {
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, (value >> 8) & 0xFF);
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, value & 0xFF);
		}
		else {
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, value & 0xFF);
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, (value >> 8) & 0xFF);
		}
	}
	else {
		if (usb_fifo_is_writeble(&pipe->fifo))
			usb_fifo_write(&pipe->fifo, value & 0xFF);
	}
}

/**
 ****************************************************************
 * Register D0FIFO
 * One of two data fifos. The assignement to a Pipe is done
 * with the D0FIFOSEL register.
 ****************************************************************
 */
static uint32_t
d0fifo_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = 0;
	int curpipe = ru->regD0FIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (usb_fifo_is_readable(&pipe->fifo))
		result |= usb_fifo_read(&pipe->fifo);

	// 16bit
	if ((rqlen > 1) && (ru->regD0FIFOSEL & FIFOSEL_MBW)) {
		if (ru->regD0FIFOSEL & FIFOSEL_BIGEND) {
			result <<= 8;
			if (usb_fifo_is_readable(&pipe->fifo))
				result |= usb_fifo_read(&pipe->fifo);
		}
		else {
			if (usb_fifo_is_readable(&pipe->fifo))
				result |= usb_fifo_read(&pipe->fifo) << 8;
		}
	}

	return result;
}

static void
d0fifo_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regD0FIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	// 16bit
	if ((rqlen > 1) && (ru->regD0FIFOSEL & FIFOSEL_MBW)) {
		if (ru->regD0FIFOSEL & FIFOSEL_BIGEND) {
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, (value >> 8) & 0xFF);
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, value & 0xFF);
		}
		else {
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, value & 0xFF);
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, (value >> 8) & 0xFF);
		}
	}
	else {
		if (usb_fifo_is_writeble(&pipe->fifo))
			usb_fifo_write(&pipe->fifo, value & 0xFF);
	}
}

/**
 *****************************************************************
 * Register D1FIFO
 * One of two data fifos. The assignment to a pipe is done
 * with the D1FIFOSEL register.
 *****************************************************************
 */
static uint32_t
d1fifo_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = 0;
	int curpipe = ru->regD1FIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (usb_fifo_is_readable(&pipe->fifo))
		result |= usb_fifo_read(&pipe->fifo);

	// 16bit
	if ((rqlen > 1) && (ru->regD1FIFOSEL & FIFOSEL_MBW)) {
		if (ru->regD1FIFOSEL & FIFOSEL_BIGEND) {
			result <<= 8;
			if (usb_fifo_is_readable(&pipe->fifo))
				result |= usb_fifo_read(&pipe->fifo);
		}
		else {
			if (usb_fifo_is_readable(&pipe->fifo))
				result |= usb_fifo_read(&pipe->fifo) << 8;
		}
	}

	return result;
}

static void
d1fifo_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regD1FIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	// 16bit
	if ((rqlen > 1) && (ru->regD1FIFOSEL & FIFOSEL_MBW)) {
		if (ru->regD1FIFOSEL & FIFOSEL_BIGEND) {
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, (value >> 8) & 0xFF);
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, value & 0xFF);
		}
		else {
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, value & 0xFF);
			if (usb_fifo_is_writeble(&pipe->fifo))
				usb_fifo_write(&pipe->fifo, (value >> 8) & 0xFF);
		}
	}
	else {
		if (usb_fifo_is_writeble(&pipe->fifo))
			usb_fifo_write(&pipe->fifo, value & 0xFF);
	}
}

/**
 *******************************************************************************************
 * Register xFIFOSEL registers
 * Do the assignent between a pipe and the control fifo access port CFIFO
 * Bits 0 - 3: Access pipe number (0 == DCP ... 9 == PIPE9)
 * Bit  5: ISEL 0 = Read from fifo 1 = write to fifo
 * Bit  8: BIGEND Fifo Port endianess:  0 = little endian, 1 = bigendian
 * Bit 10: MBW access width: 0 = 8Bit 1 = 16Bit
 * Bit 14: Buffer pointer rewind.
 * Bit 15: RCNT Read count mode: 0 Clear DTLN when read is complete. 1 = Decrement DTLN
 *****************************************************************************************
 */
static uint32_t
cfifosel_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = ru->regCFIFOSEL;
	int curpipe = result & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (pipe->isel)
		result |= FIFOSEL_ISEL;
	else
		result &= ~FIFOSEL_ISEL;

	return result;
}

static void
cfifosel_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = value & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (value & FIFOSEL_ISEL)
		pipe->isel = true;
	else
		pipe->isel = false;

	ru->regCFIFOSEL = value;
}

/**
 ***************************************************************************
 * Register xFIFOCR
 * Bit 0 - 8 DTLN Indicates the length of recieve data. Depends on RCNT
 * Bit 13 Fifo Port Ready 0 = port access disabled, 1 = enabled
 * Bit 14: 1 = Clear the buffer memory
 * Bit 15: 1 = Buffer Memory valid flag 1 = Writing ended
 ***************************************************************************
 */
static uint32_t
cfifoctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regCFIFOCTR;
}

static void
cfifoctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regCFIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (value & FIFOCTR_BCLR)
		usb_fifo_clear(&pipe->fifo);
	value &= ~(FIFOCTR_DTLN_MSK | FIFOCTR_FRDY);
	value |= usb_fifo_length(&pipe->fifo) & FIFOCTR_DTLN_MSK;
	if (pipe->frdy)
		value |= FIFOCTR_FRDY;
	ru->regCFIFOCTR = value;
}

static uint32_t
d0fifosel_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = ru->regD0FIFOSEL;
	int curpipe = result & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (pipe->isel)
		result |= FIFOSEL_ISEL;
	else
		result &= ~FIFOSEL_ISEL;

	return result;
}

static void
d0fifosel_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = value & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (value & FIFOSEL_ISEL)
		pipe->isel = true;
	else
		pipe->isel = false;

	ru->regD0FIFOSEL = value;
}

static uint32_t
d0fifoctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regD0FIFOCTR;
}

static void
d0fifoctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regD0FIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	value &= ~(FIFOCTR_DTLN_MSK | FIFOCTR_FRDY);
	value |= usb_fifo_length(&pipe->fifo);
	if (pipe->frdy)
		value |= FIFOCTR_FRDY;
	ru->regD0FIFOCTR = value;
}

static uint32_t
d1fifosel_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = ru->regD1FIFOSEL;
	int curpipe = result & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (pipe->isel)
		result |= FIFOSEL_ISEL;
	else
		result &= ~FIFOSEL_ISEL;

	return result;
}

static void
d1fifosel_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = value & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	if (value & FIFOSEL_ISEL)
		pipe->isel = true;
	else
		pipe->isel = false;

	ru->regD1FIFOSEL = value;
}

static uint32_t
d1fifoctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regD1FIFOCTR;
}

static void
d1fifoctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regD1FIFOSEL & FIFOSEL_CURPIPE_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];

	value &= ~(FIFOCTR_DTLN_MSK | FIFOCTR_FRDY);
	value |= usb_fifo_length(&pipe->fifo);
	if (pipe->frdy)
		value |= FIFOCTR_FRDY;
	ru->regD1FIFOCTR = value;
}

/**
 ***************************************************************
 * Register INTENB0:
 * Interrupt enable register 0
 * Bit 8: BRDYE	Buffer ready interrupt enable.
 * Bit 9: NRDYE Buffer not ready respones interrupt enable
 * Bit 10: BEMPE Buffer empty interrupt enable.
 * Bit 11: Control transfer stage transition interrupt enable
 * Bit 12: DVSE Device state transition interrupt enable.
 * Bit 13: SOFE Start of frame interrupt enable
 * Bit 14: RSME Resume interrupt enable
 * Bit 15: VBSE VBUS interrupt enable
 **************************************************************
 */
static uint32_t
intenb0_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regINTENB0;
}

static void
intenb0_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint16_t intsts0 = ru->regINTSTS0 & (INTSTS0_VBINT | INTSTS0_RESM | INTSTS0_SOFR | INTSTS0_DVST | INTSTS0_CTRT | INTSTS0_BEMP | INTSTS0_NRDY | INTSTS0_BRDY);
	value &= (INTENB0_VBSE | INTENB0_RSME | INTENB0_SOFE | INTENB0_DVSE | INTENB0_CTRE | INTENB0_BEMPE | INTENB0_NRDYE | INTENB0_BRDYE);
	ru->regINTENB0 = value;
	if (intsts0 & value) {
		ru->int_req = 1;
		CycleTimer_Mod(&ru->sofTimer, 1);
	}
}

/**
 *******************************************************************
 * Register INTENB1:
 * Interrupt enable register 1. ONLY Host controller mode !
 * Bit 4: SACKE Setup transaction normale response interrupt enable.
 * Bit 5: SIGNE Setup transaction error interrupt enable.
 * Bit 6: EOFERRE Error detection interrupt enable.
 * Bit 11: ATTCHE Connection detection interrupt enable.
 * Bit 12: DTCHE Disconnect detection interrupt enable.
 * Bit 14: BCHGE Interrupt output disabled.
 * Bit 15: OVRCRE Overcurrent Change interrupt eneble
 ******************************************************************
 */
static uint32_t
intenb1_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regINTENB1;
}

static void
intenb1_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regINTENB1 = value;
}

/**
 ********************************************************************
 * Register BRDYENB
 * Bufer Ready Interrupt enables
 ********************************************************************
 */
static uint32_t
brdyenb_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regBRDYENB;
}

static void
brdyenb_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regBRDYENB = value;
}

/**
 ********************************************************************
 * Register NRDYENB
 * Bufer not Ready Interrupt enables
 ********************************************************************
 */
static uint32_t
nrdyenb_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regNRDYENB;
}

static void
nrdyenb_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regNRDYENB = value;
}

/**
 ***************************************************************
 * Buffer empty interrupt enables.
 ***************************************************************
 */
static uint32_t
bempenb_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regBEMPENB;
}

static void
bempenb_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regBEMPENB = value;
}

/**
 *************************************************************************************
 * Start of frame output configuration register.
 * Bit 4: EDGESTS ? Was ist edge processing ?????
 * Bit 5: BRDYM set this bit to 0
 * Bit 6: TRNSEL Transaction enabled time select. Set to 0 in function mode !
 ************************************************************************************
 */
static uint32_t
sofcfg_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regSOFCFG;
}

static void
sofcfg_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regSOFCFG = value;
}

/**
 ******************************************************************
 * Register INTSTS0
 * Interrupt status register.
 * Some of the bits are write 0 to clear.
 * Bits 0 - 2 CTSQ Control transfer stage.
 * Bit 3: VALID 1 = setup packet reception.
 * Bit 4 - 6: Device state
 * Bit 7: VBus Input status
 * Bit 8: BRDY Buffer ready interrupt status.
 * Bit 9: NRDY Buffer not ready interrupt status.
 * Bit 10: BEMP Buffer empty interrupt status.
 * Bit 11: CTRT Control transfer stage transition interrupt status.
 * Bit 12: DVST device state transition interrupt status.
 * Bit 13: SOFR Start of frame interrupt status.
 * Bit 14: RESM Resume interrupt status
 * Bit 15: VBINT VBUS interrupt status.
 ******************************************************************
 */
static uint32_t
intsts0_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint16_t intsts0 = ru->regINTSTS0;
	intsts0 &= ~(INTSTS0_DVSQ_MSK | INTSTS0_CTSQ_MSK);
	intsts0 |= ru->DevSeq << INTSTS0_DVSQ_OFS;
	intsts0 |= ru->CtrSeq << INTSTS0_CTSQ_OFS;
	return intsts0;
}

static void
intsts0_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;

	value |= ~(INTSTS0_VBINT | INTSTS0_RESM | INTSTS0_SOFR | INTSTS0_DVST | INTSTS0_CTRT | INTSTS0_BEMP | INTSTS0_NRDY | INTSTS0_BRDY);

	//assert(((~ru->regINTSTS0) & (~value)) == 0);

	ru->regINTSTS0 &= value;
}

/**
 *************************************************************************
 * Register INTSTS1
 * Interrupt status register 1. Host mode only
 * Bit 15: OVRCR  Overrcurrent interrupt status.
 * Bit 14: BCHG USB Bus Change interrupt Status.
 * Bit 12: DTCH disconnection interrupt status
 * Bit 11: ATTCH Connect interrupt status.
 * Bit 6: EOFERR EOF error detection interrupt status.
 * Bit 5: SIGN Setup Transaction interrupt status.
 * Bit 4: SACK Setup transaction normal response interrupt status.
 *************************************************************************
 */
static uint32_t
intsts1_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regINTSTS1;
}

static void
intsts1_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regINTSTS1 = value;
}

/**
 *************************************************************
 * BRDYSTS
 * Buffer ready interrupt status register
 *************************************************************
 */
static uint32_t
brdysts_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = 0;
	uint32_t b = 0x001;
	for (int i = 0; i < 9; i++, b <<= 1)
		if (ru->Pipe[i].brdy)
			result |= b;
	return result;
}

static void
brdysts_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t b = 0x001;
	for (int i = 0; i < 9; i++, b <<= 1)
		if ((value & b) == 0)
			ru->Pipe[i].brdy = false;
}

/**
 ******************************************************************
 * Buffer Not ready interrupt status register
 ******************************************************************
 */
static uint32_t
nrdysts_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = 0;
	uint32_t b = 0x001;
	for (int i = 0; i < 9; i++, b <<= 1)
		if (ru->Pipe[i].nrdy)
			result |= b;
	return result;
}

static void
nrdysts_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t b = 0x001;
	for (int i = 0; i < 9; i++, b <<= 1)
		if ((value & b) == 0)
			ru->Pipe[i].nrdy = false;
}

/**
 ****************************************************************
 * BEMPTSTS
 * Buffer empty interrupt status register.
 * BEMTS is not set if buffer is emptied by BCLR.
 ****************************************************************
 */
static uint32_t
bempsts_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t result = 0;
	uint32_t b = 0x001;
	for (int i = 0; i < 9; i++, b <<= 1)
		if (ru->Pipe[i].bemp)
			result |= b;
	return result;
}

static void
bempsts_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	uint32_t b = 0x001;
	for (int i = 0; i < 9; i++, b <<= 1)
		if ((value & b) == 0)
			ru->Pipe[i].bemp = false;
}

/**
 ****************************************************************
 * Register  FRMNUM
 * latest framenumber
 * Bits 0 - 10 Framenumber of last received or sent SOF packet.
 * Bit 14: CRCE Receive data error.
 * Bit 15: Overrun/Underrun detection status.
 ****************************************************************
 */
static uint32_t
frmnum_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regFRMNUM;
}

static void
frmnum_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regFRMNUM = value;
}

/**
 ***************************************************************
 * Register DVCHGR
 * Device state change register.
 * Bit 15
 ***************************************************************
 */
static uint32_t
dvchgr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regDVCHGR;
}

static void
dvchgr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regDVCHGR = value;
}

/**
 **************************************************************************
 * Register USBADDR
 * Bits 0-6: Indicates the assigned USB device address (Function mode only)
 * Bits 8-11: Status recovery.
 **************************************************************************
 */
static uint32_t
usbaddr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regUSBADDR;
}

static void
usbaddr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regUSBADDR = value;
}

/**
 ******************************************************************
 * USB request
 * Host: USB request to send, Function: USB request received.
 ******************************************************************
 */
static uint32_t
usbreq_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regUSBREQ;
}

static void
usbreq_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regUSBREQ = value;
}

/**
 ***********************************************************************
 * Register USBVAL
 * Stores the request value.
 ***********************************************************************
 */
static uint32_t
usbval_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regUSBVAL;
}

static void
usbval_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regUSBVAL = value;
}

/**
 ***************************************************************
 * Register USBINDX
 ***************************************************************
 */
static uint32_t
usbindx_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regUSBINDX;
}

static void
usbindx_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regUSBINDX = value;
}

/**
 *******************************************************
 * USB request length.
 *******************************************************
 */
static uint32_t
usbleng_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regUSBLENG;
}

static void
usbleng_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regUSBLENG = value;
}

/**
 ***********************************************************************
 * DCPCFG
 * Bit 4: DIR Transfer Direction
 * Bit 7: SHTNAK Pipe is disabled at end of tranfer (following packets
 *	  will be NAKed)
 ***********************************************************************
 */
static uint32_t
dcpcfg_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[0];

	return pipe->regPIPECFG;
}

static void
dcpcfg_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[0];

	pipe->regPIPECFG = value;

	if (value & PIPECFG_DIR) {
		pipe->isel = true;
	}
	else {
		pipe->isel = false;
	}
}

/**
 *******************************************************************
 * DCPMAXP
 * DCP maximum packet size register
 * Bits 0-6: Maximum packet size.
 * Bits 12-15: Device select 0-5
 *******************************************************************
 */
static uint32_t
dcpmaxp_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[0];

	return pipe->regPIPEMAXP;
}

static void
dcpmaxp_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[0];

	pipe->regPIPEMAXP = value;
}

/**
 *************************************************************
 * DCP Control Register DCPCTR
 * Bit 0-1: PID Respnse PID 00 = NAK, 01 = BUF, 10,11 = STALL
 * Bit 2: CCPL Control Tranfer End Enable
 * Bit 5: PBUSY Pipe Busy
 * Bit 6: SQMON Sequence Toggle Bit monitor
 * Bit 7: SQSET Toggle Bit Set
 * Bit 8: SQCLR Toggle Bit Clear
 * Bit 11: SUREQCLR SUREQ Bit clear
 * Bit 14: SUREQ Setup Token Transmission
 * Bit 15: BSTS Buffer Status
 *************************************************************
 */
static uint32_t
dcpctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[0];
	uint32_t result = pipe->regPIPECTR;

	if (pipe->isel) {
		if (true)
			result |= DCPCTR_BSTS;
		else
			result &= ~DCPCTR_BSTS;
	}
	else {
		if (true)
			result |= DCPCTR_BSTS;
		else
			result &= ~DCPCTR_BSTS;
	}

	return result;
}

static void
dcpctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[0];

	pipe->regPIPECTR = value;
}

/**
 ***************************************************************
 * Register PIPESEL
 * Pipe Window Select Register, 0 = No pipe, 1-9
 ***************************************************************
 */
static uint32_t
pipesel_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regPIPESEL;
}

static void
pipesel_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regPIPESEL = value;
}

/**
 ******************************************************************
 * Register PIECFG
 * Bit 0 - 3: Endpoint Number
 * Bit 4: Transfer Direction
 * Bit 7: SHTNAK Disable pipe at end of transfer
 * Bit 9: DBLB Double buffer mode
 * Bit 10: BFRE BRDY Interrupt operation specification
 * Bit 14 - 15: Transfer Type 00: unused 01: Bulk, 11: Isochronous
 ******************************************************************
 */
static uint32_t
pipecfg_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regPIPESEL & PIPESEL_PIPESEL_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];
	if ((curpipe <= 0) || (curpipe > 9))
		return 0;
	return pipe->regPIPECFG;
}

static void
pipecfg_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regPIPESEL & PIPESEL_PIPESEL_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];
	if ((curpipe <= 0) || (curpipe > 9))
		return;
	pipe->regPIPECFG = value;

	if (value & PIPECFG_DIR) {
		pipe->isel = true;
	}
	else {
		pipe->isel = false;
	}
}

static uint32_t
pipemaxp_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regPIPESEL & PIPESEL_PIPESEL_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];
	if ((curpipe <= 0) || (curpipe > 9))
		return 0;
	return pipe->regPIPEMAXP;
}

static void
pipemaxp_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regPIPESEL & PIPESEL_PIPESEL_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];
	if ((curpipe <= 0) || (curpipe > 9))
		return;
	pipe->regPIPEMAXP = value;
}

static uint32_t
pipeperi_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regPIPESEL & PIPESEL_PIPESEL_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];
	if ((curpipe <= 0) || (curpipe > 9))
		return 0;
	return pipe->regPIPEPERI;
}

static void
pipeperi_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	int curpipe = ru->regPIPESEL & PIPESEL_PIPESEL_MSK;
	RXUsbPipe *pipe = &ru->Pipe[curpipe];
	if ((curpipe <= 0) || (curpipe > 9))
		return;
	pipe->regPIPEPERI = value;
}

static uint32_t
pipe1ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe1ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe2ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe2ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe3ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe3ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe4ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe4ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe5ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe5ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe6ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe6ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe7ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe7ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe8ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe8ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe9ctr_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe9ctr_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe1tre_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe1tre_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe1trn_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRN;
}

static void
pipe1trn_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRN = value;
}

static uint32_t
pipe2tre_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe2tre_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe2trn_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRN;
}

static void
pipe2trn_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRN = value;
}

static uint32_t
pipe3tre_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe3tre_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe3trn_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRN;
}

static void
pipe3trn_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRN = value;
}

static uint32_t
pipe4tre_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe4tre_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe4trn_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRN;
}

static void
pipe4trn_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRN = value;
}

static uint32_t
pipe5tre_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRE;
}

static void
pipe5tre_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRE = value;
}

static uint32_t
pipe5trn_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	return pipe->regPIPETRN;
}

static void
pipe5trn_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	RXUsbPipe *pipe = &ru->Pipe[1];
	pipe->regPIPETRN = value;
}

static uint32_t
devadd0_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regDEVADD0;
}

static void
devadd0_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regDEVADD0 = value;
}

static uint32_t
devadd1_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regDEVADD1;
}

static void
devadd1_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regDEVADD1 = value;
}

static uint32_t
devadd2_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regDEVADD2;
}

static void
devadd2_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regDEVADD2 = value;
}

static uint32_t
devadd3_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regDEVADD3;
}

static void
devadd3_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regDEVADD3 = value;
}

static uint32_t
devadd4_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regDEVADD4;
}

static void
devadd4_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regDEVADD4 = value;
}

static uint32_t
devadd5_read(void *clientData, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	return ru->regDEVADD5;
}

static void
devadd5_write(void *clientData, uint32_t value, uint32_t address, int rqlen)
{
	RXUsb *ru = (RXUsb *)clientData;
	ru->regDEVADD5 = value;
}

static void
RxUsbFun_Unmap(void *owner, uint32_t base, uint32_t mask)
{
	IOH_Delete16(REG_SYSCFG(base));
	IOH_Delete16(REG_SYSSTS0(base));
	IOH_Delete16(REG_DVSTCTR0(base));
	IOH_Delete16(REG_CFIFO(base));
	IOH_Delete16(REG_D0FIFO(base));
	IOH_Delete16(REG_D1FIFO(base));
	IOH_Delete16(REG_CFIFOSEL(base));
	IOH_Delete16(REG_CFIFOCTR(base));
	IOH_Delete16(REG_D0FIFOSEL(base));
	IOH_Delete16(REG_D0FIFOCTR(base));
	IOH_Delete16(REG_D1FIFOSEL(base));
	IOH_Delete16(REG_D1FIFOCTR(base));
	IOH_Delete16(REG_INTENB0(base));
	IOH_Delete16(REG_INTENB1(base));
	IOH_Delete16(REG_BRDYENB(base));
	IOH_Delete16(REG_NRDYENB(base));
	IOH_Delete16(REG_BEMPENB(base));
	IOH_Delete16(REG_SOFCFG(base));
	IOH_Delete16(REG_INTSTS0(base));
	IOH_Delete16(REG_INTSTS1(base));
	IOH_Delete16(REG_BRDYSTS(base));
	IOH_Delete16(REG_NRDYSTS(base));
	IOH_Delete16(REG_BEMPSTS(base));
	IOH_Delete16(REG_FRMNUM(base));
	IOH_Delete16(REG_DVCHGR(base));
	IOH_Delete16(REG_USBADDR(base));
	IOH_Delete16(REG_USBREQ(base));
	IOH_Delete16(REG_USBVAL(base));
	IOH_Delete16(REG_USBINDX(base));
	IOH_Delete16(REG_USBLENG(base));
	IOH_Delete16(REG_DCPCFG(base));
	IOH_Delete16(REG_DCPMAXP(base));
	IOH_Delete16(REG_DCPCTR(base));
	IOH_Delete16(REG_PIPESEL(base));
	IOH_Delete16(REG_PIPECFG(base));
	IOH_Delete16(REG_PIPEMAXP(base));
	IOH_Delete16(REG_PIPEPERI(base));
	IOH_Delete16(REG_PIPE1CTR(base));
	IOH_Delete16(REG_PIPE2CTR(base));
	IOH_Delete16(REG_PIPE3CTR(base));
	IOH_Delete16(REG_PIPE4CTR(base));
	IOH_Delete16(REG_PIPE5CTR(base));
	IOH_Delete16(REG_PIPE6CTR(base));
	IOH_Delete16(REG_PIPE7CTR(base));
	IOH_Delete16(REG_PIPE8CTR(base));
	IOH_Delete16(REG_PIPE9CTR(base));
	IOH_Delete16(REG_PIPE1TRE(base));
	IOH_Delete16(REG_PIPE1TRN(base));
	IOH_Delete16(REG_PIPE2TRE(base));
	IOH_Delete16(REG_PIPE2TRN(base));
	IOH_Delete16(REG_PIPE3TRE(base));
	IOH_Delete16(REG_PIPE3TRN(base));
	IOH_Delete16(REG_PIPE4TRE(base));
	IOH_Delete16(REG_PIPE4TRN(base));
	IOH_Delete16(REG_PIPE5TRE(base));
	IOH_Delete16(REG_PIPE5TRN(base));
	IOH_Delete16(REG_DEVADD0(base));
	IOH_Delete16(REG_DEVADD1(base));
	IOH_Delete16(REG_DEVADD2(base));
	IOH_Delete16(REG_DEVADD3(base));
	IOH_Delete16(REG_DEVADD4(base));
	IOH_Delete16(REG_DEVADD5(base));
#if 0
	IOH_Delete32(REG_DPUSR0R);
	IOH_Delete32(REG_DPUSR1R);
#endif
}

static void
RxUsbFun_Map(void *owner, uint32_t base, uint32_t mask, uint32_t flags)
{
	void *usbfn = owner;
	IOH_New16(REG_SYSCFG(base), syscfg_read, syscfg_write, usbfn);
	IOH_New16(REG_SYSSTS0(base), syssts0_read, syssts0_write, usbfn);
	IOH_New16(REG_DVSTCTR0(base), dvstctr0_read, dvstctr0_write, usbfn);
	IOH_New16(REG_CFIFO(base), cfifo_read, cfifo_write, usbfn);
	IOH_New16(REG_D0FIFO(base), d0fifo_read, d0fifo_write, usbfn);
	IOH_New16(REG_D1FIFO(base), d1fifo_read, d1fifo_write, usbfn);
	IOH_New16(REG_CFIFOSEL(base), cfifosel_read, cfifosel_write, usbfn);
	IOH_New16(REG_CFIFOCTR(base), cfifoctr_read, cfifoctr_write, usbfn);
	IOH_New16(REG_D0FIFOSEL(base), d0fifosel_read, d0fifosel_write, usbfn);
	IOH_New16(REG_D0FIFOCTR(base), d0fifoctr_read, d0fifoctr_write, usbfn);
	IOH_New16(REG_D1FIFOSEL(base), d1fifosel_read, d1fifosel_write, usbfn);
	IOH_New16(REG_D1FIFOCTR(base), d1fifoctr_read, d1fifoctr_write, usbfn);
	IOH_New16(REG_INTENB0(base), intenb0_read, intenb0_write, usbfn);
	IOH_New16(REG_INTENB1(base), intenb1_read, intenb1_write, usbfn);
	IOH_New16(REG_BRDYENB(base), brdyenb_read, brdyenb_write, usbfn);
	IOH_New16(REG_NRDYENB(base), nrdyenb_read, nrdyenb_write, usbfn);
	IOH_New16(REG_BEMPENB(base), bempenb_read, bempenb_write, usbfn);
	IOH_New16(REG_SOFCFG(base), sofcfg_read, sofcfg_write, usbfn);
	IOH_New16(REG_INTSTS0(base), intsts0_read, intsts0_write, usbfn);
	IOH_New16(REG_INTSTS1(base), intsts1_read, intsts1_write, usbfn);
	IOH_New16(REG_BRDYSTS(base), brdysts_read, brdysts_write, usbfn);
	IOH_New16(REG_NRDYSTS(base), nrdysts_read, nrdysts_write, usbfn);
	IOH_New16(REG_BEMPSTS(base), bempsts_read, bempsts_write, usbfn);
	IOH_New16(REG_FRMNUM(base), frmnum_read, frmnum_write, usbfn);
	IOH_New16(REG_DVCHGR(base), dvchgr_read, dvchgr_write, usbfn);
	IOH_New16(REG_USBADDR(base), usbaddr_read, usbaddr_write, usbfn);
	IOH_New16(REG_USBREQ(base), usbreq_read, usbreq_write, usbfn);
	IOH_New16(REG_USBVAL(base), usbval_read, usbval_write, usbfn);
	IOH_New16(REG_USBINDX(base), usbindx_read, usbindx_write, usbfn);
	IOH_New16(REG_USBLENG(base), usbleng_read, usbleng_write, usbfn);
	IOH_New16(REG_DCPCFG(base), dcpcfg_read, dcpcfg_write, usbfn);
	IOH_New16(REG_DCPMAXP(base), dcpmaxp_read, dcpmaxp_write, usbfn);
	IOH_New16(REG_DCPCTR(base), dcpctr_read, dcpctr_write, usbfn);
	IOH_New16(REG_PIPESEL(base), pipesel_read, pipesel_write, usbfn);
	IOH_New16(REG_PIPECFG(base), pipecfg_read, pipecfg_write, usbfn);
	IOH_New16(REG_PIPEMAXP(base), pipemaxp_read, pipemaxp_write, usbfn);
	IOH_New16(REG_PIPEPERI(base), pipeperi_read, pipeperi_write, usbfn);
	IOH_New16(REG_PIPE1CTR(base), pipe1ctr_read, pipe1ctr_write, usbfn);
	IOH_New16(REG_PIPE2CTR(base), pipe2ctr_read, pipe2ctr_write, usbfn);
	IOH_New16(REG_PIPE3CTR(base), pipe3ctr_read, pipe3ctr_write, usbfn);
	IOH_New16(REG_PIPE4CTR(base), pipe4ctr_read, pipe4ctr_write, usbfn);
	IOH_New16(REG_PIPE5CTR(base), pipe5ctr_read, pipe5ctr_write, usbfn);
	IOH_New16(REG_PIPE6CTR(base), pipe6ctr_read, pipe6ctr_write, usbfn);
	IOH_New16(REG_PIPE7CTR(base), pipe7ctr_read, pipe7ctr_write, usbfn);
	IOH_New16(REG_PIPE8CTR(base), pipe8ctr_read, pipe8ctr_write, usbfn);
	IOH_New16(REG_PIPE9CTR(base), pipe9ctr_read, pipe9ctr_write, usbfn);
	IOH_New16(REG_PIPE1TRE(base), pipe1tre_read, pipe1tre_write, usbfn);
	IOH_New16(REG_PIPE1TRN(base), pipe1trn_read, pipe1trn_write, usbfn);
	IOH_New16(REG_PIPE2TRE(base), pipe2tre_read, pipe2tre_write, usbfn);
	IOH_New16(REG_PIPE2TRN(base), pipe2trn_read, pipe2trn_write, usbfn);
	IOH_New16(REG_PIPE3TRE(base), pipe3tre_read, pipe3tre_write, usbfn);
	IOH_New16(REG_PIPE3TRN(base), pipe3trn_read, pipe3trn_write, usbfn);
	IOH_New16(REG_PIPE4TRE(base), pipe4tre_read, pipe4tre_write, usbfn);
	IOH_New16(REG_PIPE4TRN(base), pipe4trn_read, pipe4trn_write, usbfn);
	IOH_New16(REG_PIPE5TRE(base), pipe5tre_read, pipe5tre_write, usbfn);
	IOH_New16(REG_PIPE5TRN(base), pipe5trn_read, pipe5trn_write, usbfn);
	IOH_New16(REG_DEVADD0(base), devadd0_read, devadd0_write, usbfn);
	IOH_New16(REG_DEVADD1(base), devadd1_read, devadd1_write, usbfn);
	IOH_New16(REG_DEVADD2(base), devadd2_read, devadd2_write, usbfn);
	IOH_New16(REG_DEVADD3(base), devadd3_read, devadd3_write, usbfn);
	IOH_New16(REG_DEVADD4(base), devadd4_read, devadd4_write, usbfn);
	IOH_New16(REG_DEVADD5(base), devadd5_read, devadd5_write, usbfn);
#if 0
	IOH_New32(REG_DPUSR0R);
	IOH_New32(REG_DPUSR1R);
#endif
}

static uint16_t
control_seq(RXUsb *ru, bool setup)
{
	RXUsbPipe *pipe = &ru->Pipe[0];

	switch (ru->HostStupSeq) {
	case 0:
		if (!setup) {
			ru->regINTSTS0 |= INTSTS0_VALID;
			pipe->regPIPETRE &= ~(DCPCTR_PID_MSK | DCPCTR_CCPL); /*NAK*/
			ru->regUSBREQ = /*bRequest*/(USBRQ_GET_DESCRIPTOR << 8) | /*bmRequestType*/(0x80);
			ru->regUSBVAL = /*bDescriptorType*/(USB_DT_DEVICE << 8) | /*Descriptor Index*/(0x00);
			ru->regUSBINDX = /*Language Id*/0;
			ru->regUSBLENG = /*wLength*/18;
		}
		return CTSQ_RDDS;
	case 2:
		if (!setup) {
			ru->regINTSTS0 |= INTSTS0_VALID;
			pipe->regPIPETRE &= ~(DCPCTR_PID_MSK | DCPCTR_CCPL); /*NAK*/
			ru->regUSBREQ = /*bRequest*/(USBRQ_GET_DESCRIPTOR << 8) | /*bmRequestType*/(0x80);
			ru->regUSBVAL = /*bDescriptorType*/(USB_DT_CONFIG << 8) | /*Descriptor Index*/(0x00);
			ru->regUSBINDX = /*Language Id*/0;
			ru->regUSBLENG = /*wLength*/100;
		}
		return CTSQ_RDDS;
	case 4:
		if (!setup) {
			ru->regINTSTS0 |= INTSTS0_VALID;
			pipe->regPIPETRE &= ~(DCPCTR_PID_MSK | DCPCTR_CCPL); /*NAK*/
			ru->regUSBREQ = /*bRequest*/(USBRQ_SET_CONFIGURATION << 8) | /*bmRequestType*/(0x00);
			ru->regUSBVAL = /*bConfigurationValue*/1;
			ru->regUSBINDX = /*wIndex*/0;
			ru->regUSBLENG = /*wLength*/0;
		}
		return CTSQ_WRDS;
	default:
		return CTSQ_IDST;
	}
}

static void
timer_event(void *eventData)
{
	RXUsb *ru = (RXUsb *)eventData;

	if (ru->int_req) {
		ru->int_req = 0;
		SigNode_Set(ru->sigIrq, SIG_LOW);
		SigNode_Set(ru->sigIrq, SIG_HIGH);
		CycleTimer_Mod(&ru->sofTimer, 1);
		return;
	}

	uint16_t intsts0_now = ru->regINTSTS0;
	uint16_t intsts0_int = 0, intenb0 = 0;
	uint16_t timeout = 0;
	uint16_t ctrseq = ru->CtrSeq;

	intsts0_now &= (INTSTS0_VBINT | INTSTS0_RESM | INTSTS0_SOFR | INTSTS0_DVST | INTSTS0_CTRT | INTSTS0_BEMP | INTSTS0_NRDY | INTSTS0_BRDY);

	do {
		uint16_t curpipe = ru->HostPipe;
		RXUsbPipe *pipe = &ru->Pipe[curpipe];
		uint16_t pipecfg = pipe->regPIPECFG;
		uint16_t pipemaxp = pipe->regPIPEMAXP;
		uint16_t pipeperi = pipe->regPIPEPERI;
		uint16_t *pipectr = &pipe->regPIPECTR;

		switch (ru->HostFrmSeq) {
		case 0:
			// SOF
			ru->regFRMNUM = (ru->regFRMNUM + 1) & 0x07FF;
			intsts0_int |= INTSTS0_SOFR;
			intenb0 |= INTENB0_SOFE;
			timeout = CyclesToMicroseconds(125);
			ru->HostFrmSeq = 1;
			ru->HostPipe = 1;
			break;
		case 1:
			// Cyclic Trans
			if (ru->DevSeq == DVSQ_CFGS) {
				if (pipecfg & PIPECFG_EPNUM_MSK) {
					if ((pipecfg & PIPECFG_TYPE_MSK) == TYPE_ISOCHRONOUS) {
						if (((*pipectr & DCPCTR_PID_MSK) == PID_BUF) && ((pipeperi & PIPEPERI_IITV_MSK) != 0)) {
							if ((pipecfg & PIPECFG_DIR) != 0/*OUT*/) {
								// (HOST)データ受信
								intsts0_int |= INTSTS0_BRDY;
								intenb0 |= INTENB0_BRDYE;
								timeout = CyclesToMicroseconds(125);
								break;
							}
							else {
								// (HOST)データ送信
								intsts0_int |= INTSTS0_BRDY;
								intenb0 |= INTENB0_BRDYE;
								timeout = CyclesToMicroseconds(125);
								break;
							}
						}
					}
					else if ((pipecfg & PIPECFG_TYPE_MSK) == TYPE_INTERRUPT) {
						if ((pipecfg & PIPECFG_DIR) != 0/*OUT*/) {
							if (((*pipectr & DCPCTR_PID_MSK) == PID_BUF) && ((pipeperi & PIPEPERI_IITV_MSK) != 0)) {
								// (HOST)データ受信
								intsts0_int |= INTSTS0_BRDY;
								intenb0 |= INTENB0_BRDYE;
								timeout = CyclesToMicroseconds(125);
								break;
							}
						}
						else {
							if (((*pipectr & DCPCTR_PID_MSK) == PID_BUF) && ((pipeperi & PIPEPERI_IITV_MSK) != 0)) {
								// (HOST)データ送信
								intsts0_int |= INTSTS0_BRDY;
								intenb0 |= INTENB0_BRDYE;
								timeout = CyclesToMicroseconds(125);
								break;
							}
						}
					}
				}
				if (ru->HostPipe == 9) {
					ru->HostFrmSeq = 2;
					ru->HostPipe = 0;
				}
				else {
					ru->HostPipe++;
				}
			}
			else {
				ru->HostFrmSeq = 2;
				ru->HostPipe = 0;
			}
			break;
		case 2:
			// Control Setup
			if (ctrseq == CTSQ_IDST) {
				switch (ru->DevSeq) {
					/* Powered state */
				case DVSQ_POWS:
					if (ru->Counter >= 1000) {
						ru->Counter = 0;
						intsts0_int |= INTSTS0_VBINT | INTSTS0_DVST;
						intenb0 |= INTENB0_VBSE | INTENB0_DVSE;
						timeout = CyclesToMicroseconds(125);
						ru->DevSeq = DVSQ_DEFS;
						ru->regINTSTS0 |= INTSTS0_VBSTS;
					}
					else {
						ru->Counter++;
					}
					break;
					/* Default state */
				case DVSQ_DEFS:
					intsts0_int |= INTSTS0_DVST;
					intenb0 |= INTENB0_DVSE;
					timeout = CyclesToMicroseconds(125);
					ru->DevSeq = DVSQ_ADRS;
					ru->regUSBADDR = (0x0800 | (ru->DevSeq << 8))/*STSRECOV*/ | 1/*USBADDR*/;
					break;
					/* Address state */
				case DVSQ_ADRS:
					ru->HostStupSeq = 0;
					intsts0_int |= INTSTS0_DVST;
					intenb0 |= INTENB0_DVSE;
					timeout = CyclesToMicroseconds(125/*TODO*/);
					ru->DevSeq = DVSQ_CFGS;
					control_seq(ru, false);
					ru->HostStupSeq++;
					if (ru->regUSBREQ & 0x80)
						ru->HostInRecv = true;
					ctrseq = CTSQ_RDDS;
					break;
					/* Configured state */
				case DVSQ_CFGS:
					if (control_seq(ru, false) != CTSQ_IDST) {
						ru->HostStupSeq++;
						if (ru->regUSBREQ & 0x80)
							ru->HostInRecv = true;
						timeout = CyclesToMicroseconds(125/*TODO*/);
						ctrseq = CTSQ_RDDS;
					}
					else if (ru->HostInRecv) {
						timeout = CyclesToMicroseconds(125/*TODO*/);
						ctrseq = CTSQ_WRDS;
					}
					else if (pipe->frdy) {
						// (HOST)データ受信
						timeout = CyclesToMicroseconds(125/*TODO*/);
						ctrseq = CTSQ_RDDS;
					}
					break;
					/* Suspended state */
				default:
					break;
				}
			}
			ru->HostFrmSeq = 3;
			ru->HostPipe = 0;
			break;
		case 3:
			// Control Data/State
			if (ru->DevSeq == DVSQ_CFGS) {
				switch (ctrseq) {
				case CTSQ_IDST:
					if (ru->HostPipe == 9) {
						ru->HostFrmSeq = 4;
						ru->HostPipe = 1;
					}
					else {
						ru->HostPipe++;
					}
					break;
				case CTSQ_WRDS:
					// (HOST)データ送信
					if ((curpipe == 0) || (pipecfg & PIPECFG_EPNUM_MSK)) {
						if ((*pipectr & DCPCTR_PID_MSK) == PID_BUF) {
							if ((*pipectr & DCPCTR_CCPL) == 0) {
								bool fifo_bval = false;
								bool fifo_full = false;
								uint16_t *fifoctr;
								if (pipe == 0) {
									fifoctr = &ru->regCFIFOCTR;
								}
								else if ((*pipectr & DCPCTR_SQMON) == 0) {
									fifoctr = &ru->regD0FIFOCTR;
								}
								else {
									fifoctr = &ru->regD1FIFOCTR;
								}
								fifo_bval = (*fifoctr & FIFOCTR_BVAL) != 0;
								fifo_full = usb_fifo_length(&pipe->fifo) == (pipemaxp & DCPMAXP_MXPS_MSK);
								if (fifo_bval || fifo_full) {
									*fifoctr &= ~FIFOCTR_DTLN_MSK;
									*fifoctr |= usb_fifo_length(&pipe->fifo) & FIFOCTR_DTLN_MSK;
									usb_fifo_clear(&pipe->fifo);
									if (fifo_bval) {
										ru->HostStupSeq++;
										ru->HostInRecv = false;
									}
									else {
										ru->HostInRecv = true;
									}
								}
								timeout = CyclesToMicroseconds(125);
								intsts0_int |= INTSTS0_BRDY;
								intenb0 |= INTENB0_BRDYE;
								pipe->brdy = true;
								ctrseq = CTSQ_WRSS;
							}
							else {
								ru->HostStupSeq++;
								ru->HostInRecv = false;
								timeout = CyclesToMicroseconds(125);
								intsts0_int |= INTSTS0_BRDY;
								intenb0 |= INTENB0_BRDYE;
								pipe->brdy = true;
								ctrseq = CTSQ_WRSS;
							}
						}
					}
					break;
				case CTSQ_WRSS:
					// (HOST)ステータス受信
					timeout = CyclesToMicroseconds(125);
					ctrseq = CTSQ_IDST;
					*pipectr &= ~(DCPCTR_PID_MSK | DCPCTR_CCPL);
					break;
				case CTSQ_WRNDSS:
					// (HOST)ノーデータステータス受信
					timeout = CyclesToMicroseconds(125);
					ctrseq = CTSQ_IDST;
					*pipectr &= ~(DCPCTR_PID_MSK | DCPCTR_CCPL);
					break;
				case CTSQ_RDDS:
					// (HOST)データ送信
					if ((*pipectr & DCPCTR_PID_MSK) == PID_BUF) {
						timeout = CyclesToMicroseconds(125/*TODO*/);
						ctrseq = CTSQ_RDSS;
						intsts0_int |= INTSTS0_BRDY;
						intenb0 |= INTENB0_BRDYE;
						pipe->brdy = true;
					}
					break;
				case CTSQ_RDSS:
					// (HOST)ステータス送信
					timeout = CyclesToMicroseconds(125);
					ctrseq = CTSQ_IDST;
					*pipectr &= ~(DCPCTR_PID_MSK | DCPCTR_CCPL);
					break;
				}
			}
			if (timeout == 0) {
				//ctrseq = CTSQ_IDST;
				ru->HostFrmSeq = 4;
				ru->HostPipe = 1;
			}
			break;
		case 4:
			// Bulk Data
			if (ru->DevSeq == DVSQ_CFGS) {
				if (pipecfg & PIPECFG_EPNUM_MSK) {
					if ((pipecfg & PIPECFG_TYPE_MSK) == TYPE_BULK) {
						if (((pipecfg & PIPECFG_DIR) != 0/*OUT*/) && ((*pipectr & DCPCTR_PID_MSK) == PID_BUF) && ((pipeperi & PIPEPERI_IITV_MSK) == 0)) {
							// (HOST)データ受信
							intsts0_int |= INTSTS0_BRDY;
							intenb0 |= INTENB0_BRDYE;
							timeout = CyclesToMicroseconds(125);
						}
					}
					else if (ctrseq == CTSQ_RDDS) {
						if (((pipecfg & PIPECFG_DIR) == 0/*IN*/) && ((*pipectr & DCPCTR_PID_MSK) == PID_BUF) && ((pipeperi & PIPEPERI_IITV_MSK) == 0)) {
							// (HOST)データ送信
							intsts0_int |= INTSTS0_BRDY;
							intenb0 |= INTENB0_BRDYE;
							timeout = CyclesToMicroseconds(125);
						}
					}
				}
				if (ru->HostPipe == 9) {
					ru->HostFrmSeq = 5; /* 1msに収まっていれば、4の繰り返し */
					ru->HostPipe = 1;
				}
				else {
					ru->HostPipe++;
				}
			}
			else {
				ru->HostFrmSeq = 5;
				ru->HostPipe = 1;
			}
			break;
		default:
			// Idle
			ru->HostFrmSeq = 0;
			ru->HostPipe = 1;
			timeout = CyclesToMilliseconds(1);
			break;
		}
	} while (timeout == 0);

	if (ru->CtrSeq != ctrseq) {
		ru->CtrSeq = ctrseq;
		intsts0_int |= INTSTS0_CTRT;
		intenb0 |= INTENB0_CTRE;
	}
	ru->regINTSTS0 |= intsts0_int;

	if ((intsts0_now ^ intsts0_int) && (ru->regINTENB0 & intenb0)) {
		SigNode_Set(ru->sigIrq, SIG_LOW);
		SigNode_Set(ru->sigIrq, SIG_HIGH);
	}

	CycleTimer_Mod(&ru->sofTimer, timeout);
}

/*
 *******************************************************
 * \fn BusDevice * RX_UsbFnNew(const char *name)
 * Create a new USB function module.
 *******************************************************
 */
BusDevice *
RX_UsbNew(const char *name)
{
	RXUsb *ru = sg_new(RXUsb);
	ru->bdev.owner = ru;
	ru->bdev.first_mapping = NULL;
	ru->bdev.Map = RxUsbFun_Map;
	ru->bdev.UnMap = RxUsbFun_Unmap;
	ru->bdev.hw_flags = MEM_FLAG_WRITABLE | MEM_FLAG_READABLE;

	ru->sigIrq = SigNode_New("%s.irq", name);
	CycleTimer_Init(&ru->sofTimer, timer_event, ru);

	return &ru->bdev;
}
