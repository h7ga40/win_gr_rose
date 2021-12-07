#include <lwip/sio.h>
#include "device/usbd_pvt.h"
#include <tusb.h>
#include <windows.h>
#include "configfile.h"
#include "bus.h"
#include "signode.h"
#include "sdhci.h"
#include "mmcard.h"
#include "cycletimer.h"
#include "icu_rx65.h"
#include "clk_rx65.h"
#include "mpc_rx65.h"
#include "ioport_rx65n.h"
#include "usb_rx.h"
#include "cmt_rx65n.h"
#include "sci_rx65n.h"
#include "serial.h"
#include "iodefine.h"
#include "interrupt_handlers.h"

INT PageFaultExceptionFilter(PEXCEPTION_POINTERS ExInfo);

sio_fd_t sio_open(u8_t devnum)
{
	return (sio_fd_t)1;
}

u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
	if (fd != (sio_fd_t)1)
		return 0;

	return 0;
}

void sio_send(u8_t c, sio_fd_t fd)
{
	if (fd != (sio_fd_t)1)
		return;
}

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
	return 0;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
	return false;
}

uint8_t const *tud_descriptor_bos_cb(void)
{
	return NULL;
}

uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
	return 0;
}

uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
	return NULL;
}

bool tud_hid_set_idle_cb(uint8_t instance, uint8_t idle_rate)
{
	return false;
}

void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol)
{
}

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const *request)
{
}

void sleep(int ms)
{
}

void usleep(int us)
{
}

typedef struct RX_Cpu {
	//RX_Instruction *instr;
	uint32_t icode;
	uint32_t regR[16];
	uint32_t regISP;
	uint32_t regUSP;
	uint32_t regINTB;
	uint32_t regPC;
	uint32_t regPSW;
	uint32_t regBPC;
	uint32_t regBPSW;
	uint32_t regFINTV;
	uint32_t regFPSW;
	uint64_t regACC;
	uint32_t pendingIpl;
	uint32_t pendingIrqNo;
	uint32_t pendingFirqNo;
	uint32_t signals;
	//SoftFloatContext *floatContext;
	SigNode *sigIrqAck;	/* Acknowledge output for the Interrupt controller */
	//Debugger *debugger;
	//DebugBackendOps dbgops;
	//jmp_buf restart_idec_jump;
	//RX_DebugState dbg_state;
	int dbg_steps;
	/* Throttling cpu to real speed */
	//Throttle *throttle;
} RX_Cpu;

#define RX_SIG_IRQ	(1 << 0)
#define RX_SIG_FIRQ	(1 << 1)
#define RX_SIG_DBG	(1 << 2)

#define PSW_C	(UINT32_C(1) << 0)
#define PSW_Z	(UINT32_C(1) << 1)
#define PSW_S	(UINT32_C(1) << 2)
#define PSW_O	(UINT32_C(1) << 3)
#define PSW_I	(UINT32_C(1) << 16)
#define PSW_U	(UINT32_C(1) << 17)
#define PSW_PM	(UINT32_C(1) << 20)
#define PSW_IPL_MSK	(UINT32_C(0xf) << 24)
#define PSW_IPL_SHIFT	(24)

#define FPSW_RM_MSK	(3)
#define FPSW_CV		(1 << 2)
#define FPSW_CO		(1 << 3)
#define FPSW_CZ		(1 << 4)
#define FPSW_CU		(1 << 5)
#define FPSW_CX		(1 << 6)
#define FPSW_CE		(1 << 7)
#define FPSW_DN		(1 << 8)
#define FPSW_EV		(1 << 10)
#define FPSW_EO		(1 << 11)
#define FPSW_EZ		(1 << 12)
#define FPSW_EU		(1 << 13)
#define FPSW_EX		(1 << 14)
#define FPSW_FV		(1 << 26)
#define FPSW_FO		(1 << 27)
#define FPSW_FZ		(1 << 28)
#define FPSW_FU		(1 << 29)
#define FPSW_FX		(1 << 30)
#define FPSW_FS		(1 << 31)

#define ICODE   (g_RXCpu.icode)
#if 1
#define ICODE8()  (be32_to_host(ICODE) >> 24)
#define ICODE16() (be32_to_host(ICODE) >> 16)
#define ICODE24() (be32_to_host(ICODE) >> 8)
#define ICODE32() (be32_to_host(ICODE))
#define ICODE_BE (ICODE);
#else
#define ICODE8()  (ICODE >> 24)
#define ICODE16() (ICODE >> 16)
#define ICODE24() (ICODE >> 8)
#define ICODE32() (ICODE >> 0)
#endif
#define INSTR   (g_RXCpu.instr)

#define RX_REG_PC	(g_RXCpu.regPC)
#define RX_REG_FPSW	(g_RXCpu.regFPSW)
#define RX_REG_PSW	(g_RXCpu.regPSW)
#define RX_REG_BPSW	(g_RXCpu.regBPSW)
#define RX_REG_BPC	(g_RXCpu.regBPC)
#define RX_REG_FINTV	(g_RXCpu.regFINTV)
#define RX_REG_INTB	(g_RXCpu.regINTB)
#define RX_FLOAT_CONTEXT (g_RXCpu.floatContext)

RX_Cpu g_RXCpu;
bool mainloop_event_pending;
bool mainloop_event_io;

const char softgun_config[] = "[global]\nimagedir: .\\\n[card0]\ntype: auto_mmc";
BusDevice *RxICU;
BusDevice *Rx65Clk;
BusDevice *Rx65Mpc;
BusDevice *Rx65nIoPorts;
BusDevice *RxCMT;
BusDevice *RxSCI1;

BusDevice *UsbRx;

HANDLE g_SimThread;
DWORD g_SimThreadId;
HANDLE g_SimMutex;
HANDLE g_StartEvent;
HANDLE g_EndEvent;
BOOL g_ExitFlag;

void CheckSignals();

unsigned long sim_thread(void *arg)
{
	unsigned long result = 0;
	LARGE_INTEGER freq;
	LARGE_INTEGER btime, ntime, dtime;
	DWORD ret, ret2;
	uint64_t next;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&ntime);

	while (!g_ExitFlag && result == 0) {
		if (firstCycleTimerTimeout == ~0ULL) {
			ret = WaitForSingleObject(g_StartEvent, INFINITE);
		}
		else {
			ret = WaitForSingleObject(g_StartEvent, CyclesToMilliseconds(firstCycleTimerTimeout - CycleCounter));
		}

		if (g_ExitFlag || ((ret != WAIT_OBJECT_0) && (ret != WAIT_TIMEOUT)))
			break;

		if (ret == WAIT_TIMEOUT) {
			for (;;) {
				ret2 = WaitForSingleObject(g_SimMutex, 0);
				if (ret2 == WAIT_OBJECT_0)
					break;
				if (g_ExitFlag || (ret2 != WAIT_TIMEOUT)) {
					result = GetLastError();
					break;
				}

				ret = WaitForSingleObject(g_StartEvent, 0);
				if (ret == WAIT_OBJECT_0)
					break;
				if (g_ExitFlag || (ret != WAIT_TIMEOUT)) {
					result = GetLastError();
					break;
				}
			}
		}
		else {
			ret2 = WAIT_TIMEOUT;
		}

		btime = ntime;

		QueryPerformanceCounter(&ntime);

		if (ntime.QuadPart >= btime.QuadPart) {
			dtime.QuadPart = ntime.QuadPart - btime.QuadPart;
		}
		else {
			dtime.QuadPart = btime.QuadPart - ntime.QuadPart;
		}

		next = CycleCounter + (CycleTimerRate * dtime.QuadPart / freq.QuadPart);

		if (firstCycleTimerTimeout < next) {
			CycleCounter = firstCycleTimerTimeout;

			__try
			{
				CheckSignals();
				CycleTimers_Check();
			}
			__except (PageFaultExceptionFilter(GetExceptionInformation()))
			{
				result = GetLastError();
			}
		}
		else {
			CycleCounter = next;
		}

		if (ret2 == WAIT_OBJECT_0) {
			ReleaseMutex(g_SimMutex);
		}

		if (ret == WAIT_OBJECT_0) {
			SetEvent(g_EndEvent);
		}
	}

	return 0;
}

void SimLock(void)
{
	if (GetCurrentThreadId() == g_SimThreadId)
		return;

	WaitForSingleObject(g_SimMutex, INFINITE);

	SetEvent(g_StartEvent);
	WaitForSingleObject(g_EndEvent, INFINITE);
}

void SimUnlock(void)
{
	if (GetCurrentThreadId() == g_SimThreadId)
		return;

	SetEvent(g_StartEvent);
	WaitForSingleObject(g_EndEvent, INFINITE);

	ReleaseMutex(g_SimMutex);
}

uint32_t disable_interrupt()
{
	SimLock();
	uint32_t pswi = (RX_REG_PSW & PSW_I);
	RX_REG_PSW &= ~(PSW_I);
	return pswi;
}

void enable_interrupt(uint32_t pswi)
{
	RX_REG_PSW |= (pswi & PSW_I);
	SimUnlock();
}

static inline void
RX_UpdateIrqSignal(void)
{
	uint32_t ipl = (RX_REG_PSW & PSW_IPL_MSK) >> PSW_IPL_SHIFT;
	if ((RX_REG_PSW & PSW_I) && (g_RXCpu.pendingIpl > ipl)) {
		g_RXCpu.signals |= RX_SIG_IRQ;
		mainloop_event_pending = 1;
	}
	else {
		g_RXCpu.signals &= ~RX_SIG_IRQ;
	}
}

static inline void
RX_SET_REG_PSW(uint32_t value)
{
	uint32_t diff = value ^ g_RXCpu.regPSW;
	if (diff & PSW_U) {
		if (value & PSW_U) {
			g_RXCpu.regISP = g_RXCpu.regR[0];
			g_RXCpu.regR[0] = g_RXCpu.regUSP;
		}
		else {
			g_RXCpu.regUSP = g_RXCpu.regR[0];
			g_RXCpu.regR[0] = g_RXCpu.regISP;
		}
	}
#if 0
	static CycleCounter_t tStamp;
	if ((value & PSW_IPL_MSK) == (0xf << PSW_IPL_SHIFT)) {
		tStamp = CycleCounter_Get();
	}
	else {
		fprintf(stderr, "%lu\n", CycleCounter_Get() - tStamp);
	}
#endif
	g_RXCpu.regPSW = value;
	if (diff & (PSW_I | PSW_IPL_MSK)) {
		RX_UpdateIrqSignal();
	}
}

static void
RX_FastInterrupt(void)
{
#if 0
	g_RXCpu.dbg_state = RXDBG_STOP;
	RX_SigDebugMode(1);
#endif

	fprintf(stderr, "Calling FINTV %08x at %lu\n", RX_REG_FINTV, CycleCounter_Get());
	RX_REG_BPSW = RX_REG_PSW;
	RX_SET_REG_PSW((RX_REG_PSW & ~(PSW_U | PSW_I | PSW_PM | PSW_IPL_MSK)) |
		(15 << PSW_IPL_SHIFT));
	RX_REG_BPC = RX_REG_PC;
	RX_REG_PC = RX_REG_FINTV;
	SigNode_Set(g_RXCpu.sigIrqAck, SIG_LOW);
	SigNode_Set(g_RXCpu.sigIrqAck, SIG_HIGH);
	CycleCounter += 5;
}

static void
RX_Interrupt(unsigned int irq_no, unsigned int ipl)
{
	uint32_t vector_addr;
	//fprintf(stderr,"RX irq %d level %d\n",irq_no,ipl);
	uint32_t Sp;
	vector_addr = RX_REG_INTB + (irq_no << 2);
	uint32_t saved_psw = RX_REG_PSW;
	if (unlikely(ipl == 255)) {
		RX_FastInterrupt();
	}
	else {
		RX_SET_REG_PSW((RX_REG_PSW & ~(PSW_U | PSW_I | PSW_PM | PSW_IPL_MSK)) |
			(ipl << PSW_IPL_SHIFT));

		switch (irq_no) {
		case VECT_CMT0_CMI0:
			INT_Excep_CMT0_CMI0();
			break;
		case VECT_SCI1_TXI1:
			INT_Excep_SCI1_TXI1();
			break;
		case VECT_SCI1_RXI1:
			INT_Excep_SCI1_RXI1();
			break;
		case VECT_ICU_GROUPBL0:
			INT_Excep_ICU_GROUPBL0();
			break;
		case VECT_PERIB_INTB185:
			INT_Excep_PERIB_INTB185();
			break;
		}

		//fprintf(stderr,"PC set to %08x, SP %08x\n",RX_REG_PC,Sp);
		/* This acks the interrupt if */
		SigNode_Set(g_RXCpu.sigIrqAck, SIG_LOW);
		SigNode_Set(g_RXCpu.sigIrqAck, SIG_HIGH);
	}
}

void
RX_PostInterrupt(uint8_t ipl, uint8_t irqNo)
{
	RX_Cpu *rx = &g_RXCpu;
	rx->pendingIpl = ipl;
	rx->pendingIrqNo = irqNo;
	//fprintf(stderr,"pending IPL %d, IRQ %d, CPU IPL %08x %08lld\n",ipl,irqNo, (RX_REG_PSW >> PSW_IPL_SHIFT) & 15, CycleCounter_Get() );
	RX_UpdateIrqSignal();
}

void
FIO_HandleInput()
{
}

void
Do_Debug()
{
}

void
CheckSignals()
{
	if (unlikely(mainloop_event_pending)) {
		mainloop_event_pending = 0;
		if (mainloop_event_io) {
			FIO_HandleInput();
		}
		if (g_RXCpu.signals) {
			if (likely(g_RXCpu.signals & RX_SIG_IRQ)) {
				//fprintf(stderr, "SigIrq\n");
				RX_Interrupt(g_RXCpu.pendingIrqNo, g_RXCpu.pendingIpl);
			}
#if 0
			if (g_RXCpu.signals & RX_SIG_FIRQ) {
				RX_FastInterrupt(g_RXCpu.pendingFirqNo);
			}
#endif
			if (unlikely(g_RXCpu.signals & RX_SIG_DBG)) {
				Do_Debug();
			}
		}
	}
}

const SIZE_T IORegisterSize = 0x100000;
unsigned char *IORegister;

int AnalizeCode(unsigned char *_opecode, void *_addr, PCONTEXT ContextRecord)
{
	if ((_addr < (void *)IORegister) || (_addr > (void *)&IORegister[IORegisterSize]))
		return 0;

	uint32_t addr = (uint32_t)(_addr - (void *)IORegister) + 0x80000;
	unsigned char *opecode = _opecode;
	BOOL mode16bit = FALSE;
	uint64_t value = 0;
	unsigned char code = *opecode++;

	if (code == 0x66) {
		mode16bit = TRUE;
		code = *opecode++;
	}

	uint8_t r_m = *opecode;
	uint8_t mod = (r_m >> 6) & 0x3;
	uint8_t reg = (r_m >> 3) & 0x7;
	r_m = (r_m >> 0) & 0x7;

	switch (code) {
	case 0x88: {
		// MOV Eb Gb
		switch (mod) {
		case 0:
			// mov [reg], Gb
			opecode += 1;
			break;
		case 1:
			// mov [reg+Ib], Gb
			opecode += 2;
			break;
		case 2:
			// mov [reg+Iv], Gb
			opecode += 5;
			break;
		case 3:
			// mov reg, Gb
			opecode += 1;
			break;
		}

		switch (reg) {
		case 0:
			value = ContextRecord->Rax;
			break;
		case 1:
			value = ContextRecord->Rcx;
			break;
		case 2:
			value = ContextRecord->Rdx;
			break;
		case 3:
			value = ContextRecord->Rbx;
			break;
		case 4:
			value = ContextRecord->Rsp;
			break;
		case 5:
			value = ContextRecord->Rbp;
			break;
		case 6:
			value = ContextRecord->Rsi;
			break;
		case 7:
			value = ContextRecord->Rdi;
			break;
		}

		*((uint8_t *)_addr) = value;
		IO_Write8(value, addr);
		break;
	}
	case 0x89: {
		// MOV Ev Gv
		switch (mod) {
		case 0:
			// mov [reg], Gv
			opecode += 1;
			break;
		case 1:
			// mov [reg+Ib], Gv
			opecode += 2;
			break;
		case 2:
			// mov [reg+Iv], Gv
			opecode += 5;
			break;
		case 3:
			// mov reg, Gv
			opecode += 1;
			break;
		}

		switch (reg) {
		case 0:
			value = ContextRecord->Rax;
			break;
		case 1:
			value = ContextRecord->Rcx;
			break;
		case 2:
			value = ContextRecord->Rdx;
			break;
		case 3:
			value = ContextRecord->Rbx;
			break;
		case 4:
			value = ContextRecord->Rsp;
			break;
		case 5:
			value = ContextRecord->Rbp;
			break;
		case 6:
			value = ContextRecord->Rsi;
			break;
		case 7:
			value = ContextRecord->Rdi;
			break;
		}

		if (mode16bit) {
			*((uint16_t *)_addr) = value;
			IO_Write16(value, addr);
		}
		else {
			*((uint32_t *)_addr) = value;
			IO_Write32(value, addr);
		}
		break;
	}
	case 0x8A: {
		// MOV Gb Eb
		switch (mod) {
		case 0:
			// mov Gb, [reg]
			opecode += 1;
			break;
		case 1:
			// mov Gb, [reg+Ib]
			opecode += 2;
			break;
		case 2:
			// mov Gb, [reg+Iv]
			opecode += 5;
			break;
		case 3:
			// mov Gb, reg
			opecode += 1;
			break;
		}

		value = IO_Read8(addr);
		*((uint8_t *)_addr) = value;

		switch (reg) {
		case 0:
			ContextRecord->Rax = value;
			break;
		case 1:
			ContextRecord->Rcx = value;
			break;
		case 2:
			ContextRecord->Rdx = value;
			break;
		case 3:
			ContextRecord->Rbx = value;
			break;
		case 4:
			ContextRecord->Rsp = value;
			break;
		case 5:
			ContextRecord->Rbp = value;
			break;
		case 6:
			ContextRecord->Rsi = value;
			break;
		case 7:
			ContextRecord->Rdi = value;
			break;
		}
		break;
	}
	case 0x8B: {
		// MOV Gv Ev
		switch (mod) {
		case 0:
			// mov Gv, [reg]
			opecode += 1;
			break;
		case 1:
			// mov Gv, [reg+Ib]
			opecode += 2;
			break;
		case 2:
			// mov Gv, [reg+Iv]
			opecode += 5;
			break;
		case 3:
			// mov Gv, reg
			opecode += 1;
			break;
		}

		if (mode16bit) {
			value = IO_Read16(addr);
			*((uint16_t *)_addr) = value;
		}
		else {
			value = IO_Read32(addr);
			*((uint32_t *)_addr) = value;
		}

		switch (reg) {
		case 0:
			ContextRecord->Rax = value;
			break;
		case 1:
			ContextRecord->Rcx = value;
			break;
		case 2:
			ContextRecord->Rdx = value;
			break;
		case 3:
			ContextRecord->Rbx = value;
			break;
		case 4:
			ContextRecord->Rsp = value;
			break;
		case 5:
			ContextRecord->Rbp = value;
			break;
		case 6:
			ContextRecord->Rsi = value;
			break;
		case 7:
			ContextRecord->Rdi = value;
			break;
		}
		break;
	}
	case 0xC6: {
		// MOV Eb Ib
		switch (mod) {
		case 0:
			// mov [reg], Ib
			opecode += 1;
			break;
		case 1:
			// mov [reg+Ib], Ib
			opecode += 2;
			break;
		case 2:
			// mov [reg+Iv], Ib
			opecode += 5;
			break;
		case 3:
			// mov reg, Ib
			opecode += 1;
			break;
		}
		value = *opecode++;

		if (mode16bit) {
			*((uint16_t *)_addr) = value;
			IO_Write16(value, addr);
		}
		else {
			*((uint32_t *)_addr) = value;
			IO_Write32(value, addr);
		}
		break;
	}
	case 0xC7: {
		// MOV Ev Iv
		switch (mod) {
		case 0:
			// mov [reg], Iv
			opecode += 1;
			break;
		case 1:
			// mov [reg+Ib], Iv
			opecode += 2;
			break;
		case 2:
			// mov [reg+Iv], Iv
			opecode += 5;
			break;
		case 3:
			// mov reg, Iv
			opecode += 1;
			break;
		}

		if (mode16bit) {
			value = (uint64_t)(opecode[0]) | (uint64_t)(opecode[1] << 8);
			opecode += 2;
			*((uint16_t *)_addr) = value;
			IO_Write16(value, addr);
		}
		else {
			value = (uint64_t)(opecode[0]) | (uint64_t)(opecode[1] << 8) | (uint64_t)(opecode[2] << 16) | (uint64_t)(opecode[3] << 24);
			opecode += 4;
			*((uint32_t *)_addr) = value;
			IO_Write32(value, addr);
		}

		break;
	}
	case 0xCC: {
		// INT
		return -1;
	}
	default:
		return 0;
	}

	return (int)(opecode - _opecode);
}

INT PageFaultExceptionFilter(PEXCEPTION_POINTERS ExInfo)
{
	INT result;
	if (ExInfo->ExceptionRecord->ExceptionCode != EXCEPTION_GUARD_PAGE)
		return EXCEPTION_CONTINUE_SEARCH;

	SimLock();
	int codesize = AnalizeCode(ExInfo->ExceptionRecord->ExceptionAddress, (void *)ExInfo->ExceptionRecord->ExceptionInformation[1], ExInfo->ContextRecord);
	if (codesize == -1) {
		result = EXCEPTION_CONTINUE_EXECUTION;
		goto exit;
	}
	if (codesize == 0) {
		result = EXCEPTION_EXECUTE_HANDLER;
		goto exit;
	}

	ExInfo->ContextRecord->Rip += codesize;

	DWORD oldAttr = 0;
	BOOL ok = VirtualProtect(IORegister, IORegisterSize, PAGE_READWRITE | PAGE_GUARD, &oldAttr);
	if (!ok) {
		result = EXCEPTION_EXECUTE_HANDLER;
		goto exit;
	}

	result = EXCEPTION_CONTINUE_EXECUTION;
exit:
	SimUnlock();

	return result;
}

int tusb_main(void);

static void
interrupt_sig(struct SigNode *node, int value, void *clientData)
{
	UINT intNo = (UINT)clientData;

	switch (intNo) {
	case VECT_CMT0_CMI0:
		if (value == SIG_HIGH) {
			if (GetCurrentThreadId() == g_SimThreadId)
				INT_Excep_CMT0_CMI0();
			else
				IR(CMT0, CMI0) = 1;
		}
		break;
	case VECT_SCI1_TXI1:
		if (value == SIG_HIGH) {
			if (GetCurrentThreadId() == g_SimThreadId)
				INT_Excep_SCI1_TXI1();
			else
				IR(SCI1, TXI1) = 1;
		}
		break;
	case VECT_SCI1_RXI1:
		if (value == SIG_HIGH) {
			if (GetCurrentThreadId() == g_SimThreadId)
				INT_Excep_SCI1_RXI1();
			else
				IR(SCI1, RXI1) = 1;
		}
		break;
	case VECT_ICU_GROUPBL0:
		if (value == SIG_HIGH) {
			if (GetCurrentThreadId() == g_SimThreadId)
				INT_Excep_ICU_GROUPBL0();
			else
				IR(ICU, GROUPBL0) = 1;
		}
		break;
	case VECT_PERIB_INTB185:
		if (value == SIG_HIGH) {
			if (GetCurrentThreadId() == g_SimThreadId)
				INT_Excep_PERIB_INTB185();
			else
				IR(PERIB, INTB185) = 1;
		}
		break;
	}
}

static void
usb0irq_sig(struct SigNode *node, int value, void *clientData)
{
	struct SigNode *dnode = (struct SigNode *)clientData;
	SigNode_Set(dnode, value);
}

int null_device_uart_cmd(struct SerialDevice *serial, UartCmd *cmd)
{
	return 0;
}

void null_device_start_rx(struct SerialDevice *serial)
{
}

void null_device_stop_rx(struct SerialDevice *serial)
{
}

int null_device_write(struct SerialDevice *serial, const UartChar *buf, int count)
{
	return count;
}

int null_device_read(struct SerialDevice *serial, UartChar *buf, int count)
{
	return 0;
}

SerialDevice *NullSerialDevice_Constructor(const char *name)
{
	SerialDevice *result = malloc(sizeof(SerialDevice));

	result->owner = result;
	result->write = null_device_write;
	result->read = null_device_read;

	return result;
}

__declspec(dllexport) int __stdcall device_main(void);

int device_main(void)
{
	int ret;

	IORegister = VirtualAlloc(NULL, IORegisterSize, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE | PAGE_GUARD);
	if (IORegister == NULL)
		return -1;

	g_SimThread = CreateThread(NULL, 0, sim_thread, NULL, CREATE_SUSPENDED, &g_SimThreadId);
	if (g_SimThread == NULL) {
		VirtualFree(IORegister, 0, MEM_RELEASE);
		return -1;
	}

	g_SimMutex = CreateMutex(NULL, FALSE, NULL);
	if (g_SimMutex == NULL) {
		VirtualFree(IORegister, 0, MEM_RELEASE);
		CloseHandle(g_SimThread);
		return -1;
	}

	g_StartEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (g_StartEvent == NULL) {
		VirtualFree(IORegister, 0, MEM_RELEASE);
		CloseHandle(g_SimThread);
		CloseHandle(g_SimMutex);
		return -1;
	}

	g_EndEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (g_EndEvent == NULL) {
		VirtualFree(IORegister, 0, MEM_RELEASE);
		CloseHandle(g_SimThread);
		CloseHandle(g_SimMutex);
		CloseHandle(g_StartEvent);
		return -1;
	}

	uint32_t cpu_clock = 120000000;
	const char *instancename = "rx65n";

	Config_AddString(softgun_config);

	Bus_Init(NULL, 1 * 1024);
	SignodesInit();
	ClocksInit();
	CycleTimers_Init(instancename, cpu_clock);
	SerialModule_Register("null", NullSerialDevice_Constructor);

	RxICU = RX65ICU_New("ICU", RX_ICU_RX65);
	Mem_AreaAddMapping(RxICU, 0x87000, sizeof(struct st_icu), MEM_FLAG_WRITABLE | MEM_FLAG_READABLE);
	Rx65Clk = Rx65Clk_New(instancename);
	Mem_AreaAddMapping(Rx65Clk, 0x80000, 0x10000, MEM_FLAG_WRITABLE | MEM_FLAG_READABLE);
	Rx65Mpc = Rx65Mpc_New("MPC");
	Mem_AreaAddMapping(Rx65Mpc, 0x8C000, 0x400, MEM_FLAG_WRITABLE | MEM_FLAG_READABLE);
	Rx65nIoPorts = Rx65nIoPorts_New("PORT");
	Mem_AreaAddMapping(Rx65nIoPorts, 0x8C000, 0x100, MEM_FLAG_WRITABLE | MEM_FLAG_READABLE);
	RxCMT = CMTMod_New("CMT", 0);
	Mem_AreaAddMapping(RxCMT, 0x88000, 0x20, MEM_FLAG_WRITABLE | MEM_FLAG_READABLE);
	RxSCI1 = SCI_New("SCI1");
	Mem_AreaAddMapping(RxSCI1, 0x8A020, 0x20, MEM_FLAG_WRITABLE | MEM_FLAG_READABLE);
	UsbRx = RX_UsbNew("USB0");
	Mem_AreaAddMapping(UsbRx, 0xA0000, 0x800, MEM_FLAG_WRITABLE | MEM_FLAG_READABLE);

	Clock_t *clkCMT0 = Clock_Find("CMT0.clk");
	Clock_SetFreq(clkCMT0, 60000000);

	SigNode_Trace(SigNode_Find("CMT0.irq"), interrupt_sig, (void *)VECT_CMT0_CMI0);
	SigNode_Trace(SigNode_Find("SCI1.irqTXI"), interrupt_sig, (void *)VECT_SCI1_TXI1);
	SigNode_Trace(SigNode_Find("SCI1.irqRXI"), interrupt_sig, (void *)VECT_SCI1_RXI1);
	//SigNode_Trace(SigNode_Find("ICU.irq110"), interrupt_sig, (void *)VECT_ICU_GROUPBL0);
	//SigNode_Trace(SigNode_Find("ICU.irq185"), interrupt_sig, (void *)VECT_PERIB_INTB185);
	//SigNode_Trace(SigNode_Find("USB0.irq"), usb0irq_sig, (void *)SigNode_Find("ICU.irq185"));
	SigNode_Trace(SigNode_Find("USB0.irq"), interrupt_sig, (void *)VECT_PERIB_INTB185);

	ResumeThread(g_SimThread);

	RX_REG_PSW |= PSW_I;

	__try
	{
		ret = tusb_main();
	}
	__except (PageFaultExceptionFilter(GetExceptionInformation()))
	{
		return GetLastError();
	}

	VirtualFree(IORegister, 0, MEM_RELEASE);
	CloseHandle(g_EndEvent);
	CloseHandle(g_StartEvent);
	CloseHandle(g_SimMutex);
	CloseHandle(g_SimThread);

	return ret;
}
