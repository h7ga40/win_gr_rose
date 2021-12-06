/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Koji Kitayama
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include <sys/types.h>
#include "bsp/board.h"
#include "iodefine.h"
#include "interrupt_handlers.h"

#define IRQ_PRIORITY_CMT0     5
#define IRQ_PRIORITY_USBI0    6
#define IRQ_PRIORITY_SCI1     5

#define SYSTEM_PRCR_PRC1      (1<<1)
#define SYSTEM_PRCR_PRKEY     (0xA5u<<8)

#define CMT_PCLK              60000000
#define CMT_CMCR_CKS_DIV_128  2
#define CMT_CMCR_CMIE         (1<<6)
#define MPC_PFS_ISEL          (1<<6)

#define SCI_PCLK              60000000
#define SCI_SSR_FER           (1<<4)
#define SCI_SSR_ORER          (1<<5)

#define SCI_SCR_TEIE          (1u<<2)
#define SCI_SCR_RE            (1u<<4)
#define SCI_SCR_TE            (1u<<5)
#define SCI_SCR_RIE           (1u<<6)
#define SCI_SCR_TIE           (1u<<7)
#define INT_Excep_SCI1_TEI1   INT_Excep_ICU_GROUPBL0

#define IRQ_USB0_USBI0        62
#define SLIBR_USBI0           SLIBR185
#define IPR_USB0_USBI0        IPR_PERIB_INTB185
#define INT_Excep_USB0_USBI0  INT_Excep_PERIB_INTB185

void HardwareSetup(void)
{
  FLASH.ROMCIV.WORD = 1;
  while (FLASH.ROMCIV.WORD) ;
  FLASH.ROMCE.WORD = 1;
  while (!FLASH.ROMCE.WORD) ;

  SYSTEM.PRCR.WORD = 0xA503u;
  if (!SYSTEM.RSTSR1.BYTE) {
    RTC.RCR4.BYTE = 0;
    RTC.RCR3.BYTE = 12;
    while (12 != RTC.RCR3.BYTE) ;
  }

  SYSTEM.MOFCR.BYTE = 0x20;
  SYSTEM.MOSCWTCR.BYTE = 0x53;
  SYSTEM.MOSCCR.BYTE = 0x0;
  while (SYSTEM.MOSCCR.BIT.MOSTP) ;

  SYSTEM.PLLCR.WORD  = 0x2700u; /* HOCO x 20 */
  SYSTEM.PLLCR2.BYTE = 0;
  while (!SYSTEM.OSCOVFSR.BIT.PLOVF) ;

  SYSTEM.SCKCR.LONG  = 0x21C11222u;
  SYSTEM.SCKCR2.WORD = 0x0041u;
  SYSTEM.ROMWT.BYTE  = 0x02u;
  while (0x02u != SYSTEM.ROMWT.BYTE) ;
  SYSTEM.SCKCR3.WORD = 0x400u;
  SYSTEM.PRCR.WORD   = 0xA500u;
}

//--------------------------------------------------------------------+
// SCI handling
//--------------------------------------------------------------------+
typedef struct {
  uint8_t *buf;
  uint32_t cnt;
} sci_buf_t;
static volatile sci_buf_t sci_buf[2];

void INT_Excep_SCI1_TXI1(void)
{
  uint8_t *buf = sci_buf[0].buf;
  uint32_t cnt = sci_buf[0].cnt;
  
  if (!buf || !cnt) {
    SCI1.SCR.BYTE &= ~(SCI_SCR_TEIE | SCI_SCR_TE | SCI_SCR_TIE);
    return;
  }
  SCI1.TDR = *buf;
  if (--cnt) {
    ++buf;
  } else {
    buf = NULL;
    SCI1.SCR.BIT.TIE  = 0;
    SCI1.SCR.BIT.TEIE = 1;
  }
  sci_buf[0].buf = buf;
  sci_buf[0].cnt = cnt;
}

void INT_Excep_SCI1_TEI1(void)
{
  SCI1.SCR.BYTE &= ~(SCI_SCR_TEIE | SCI_SCR_TE | SCI_SCR_TIE);
}

void INT_Excep_SCI1_RXI1(void)
{
  uint8_t *buf = sci_buf[1].buf;
  uint32_t cnt = sci_buf[1].cnt;

  if (!buf || !cnt ||
      (SCI1.SSR.BYTE & (SCI_SSR_FER | SCI_SSR_ORER))) {
    sci_buf[1].buf = NULL;
    SCI1.SSR.BYTE   = 0;
    SCI1.SCR.BYTE  &= ~(SCI_SCR_RE | SCI_SCR_RIE);
    return;
  }
  *buf = SCI1.RDR;
  if (--cnt) {
    ++buf;
  } else {
    buf = NULL;
    SCI1.SCR.BYTE &= ~(SCI_SCR_RE | SCI_SCR_RIE);
  }
  sci_buf[1].buf = buf;
  sci_buf[1].cnt = cnt;
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void INT_Excep_USB0_USBI0(void)
{
#ifdef tud_int_handler
  tud_int_handler(0);
#endif
#ifdef tuh_int_handler
  tuh_int_handler(0);
#endif
}

void board_init(void)
{
  /* setup software configurable interrupts */
  ICU.SLIBR_USBI0.BYTE = IRQ_USB0_USBI0;
  ICU.SLIPRCR.BYTE     = 1;

#if CFG_TUSB_OS == OPT_OS_NONE
  /* Enable CMT0 */
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY | SYSTEM_PRCR_PRC1;
  MSTP(CMT0)       = 0;
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY;
  /* Setup 1ms tick timer */
  CMT0.CMCNT      = 0;
  CMT0.CMCOR      = CMT_PCLK / 1000 / 128;
  CMT0.CMCR.WORD  = CMT_CMCR_CMIE | CMT_CMCR_CKS_DIV_128;
  IR(CMT0, CMI0)  = 0;
  IPR(CMT0, CMI0) = IRQ_PRIORITY_CMT0;
  IEN(CMT0, CMI0) = 1;
  CMT.CMSTR0.BIT.STR0 = 1;
#endif

  /* Unlock MPC registers */
  MPC.PWPR.BIT.B0WI  = 0;
  MPC.PWPR.BIT.PFSWE = 1;
  // SW PD7
  PORTD.PMR.BIT.B7 = 0U;
  PORTD.PDR.BIT.B7 = 0U;
  // LED PA0, PA1
  PORTA.PODR.BIT.B0 = 0U;
  PORTA.PODR.BIT.B1 = 0U;
  PORTA.PMR.BIT.B0  = 0U;
  PORTA.PMR.BIT.B1  = 0U;
  PORTA.PDR.BIT.B0  = 1U;
  PORTA.PDR.BIT.B1  = 1U;
  /* UART TXD1 => P26, RXD1 => P30 */
  PORT2.PMR.BIT.B6 = 1U;
  PORT2.PDR.BIT.B6 = 1U;
  PORT2.PCR.BIT.B6 = 1U;
  MPC.P26PFS.BYTE  = 0b01010;
  PORT3.PMR.BIT.B0 = 1U;
  PORT3.PDR.BIT.B0 = 0U;
  MPC.P30PFS.BYTE  = 0b01010;
  /* USB VBUS -> P16 */
  PORT1.PMR.BIT.B6 = 1U;
  MPC.P16PFS.BYTE  = MPC_PFS_ISEL | 0b10001;
  /* Lock MPC registers */
  MPC.PWPR.BIT.PFSWE = 0;
  MPC.PWPR.BIT.B0WI  = 1;

  /* Enable SCI1 */
  SYSTEM.PRCR.WORD   = SYSTEM_PRCR_PRKEY | SYSTEM_PRCR_PRC1;
  MSTP(SCI1)         = 0;
  SYSTEM.PRCR.WORD   = SYSTEM_PRCR_PRKEY;
  SCI1.BRR           = (SCI_PCLK / (32 * 115200)) - 1;
  IR(SCI1,  RXI1)    = 0;
  IR(SCI1,  TXI1)    = 0;
  IS(SCI1,  TEI1)    = 0;
  IR(ICU, GROUPBL0)  = 0;
  IPR(SCI1, RXI1)    = IRQ_PRIORITY_SCI1;
  IPR(SCI1, TXI1)    = IRQ_PRIORITY_SCI1;
  IPR(ICU,GROUPBL0)  = IRQ_PRIORITY_SCI1;
  IEN(SCI1, RXI1)    = 1;
  IEN(SCI1, TXI1)    = 1;
  IEN(ICU,GROUPBL0)  = 1;
  EN(SCI1, TEI1)     = 1;

  /* setup USBI0 interrupt. */
  IR(USB0, USBI0)  = 0;
  IPR(USB0, USBI0) = IRQ_PRIORITY_USBI0;
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  PORTA.PODR.BIT.B0 = state ? 0 : 1;
}

uint32_t board_button_read(void)
{
  return PORTD.PIDR.BIT.B7 ? 0 : 1;
}

int board_uart_read(uint8_t* buf, int len)
{
  sci_buf[1].buf = buf;
  sci_buf[1].cnt = len;
  SCI1.SCR.BYTE |= SCI_SCR_RE | SCI_SCR_RIE;
  while (SCI1.SCR.BIT.RE) ;
  return len - sci_buf[1].cnt;
}

int board_uart_write(void const *buf, int len)
{
  sci_buf[0].buf = (uint8_t*)buf;
  sci_buf[0].cnt = len;
  SCI1.SCR.BYTE |= SCI_SCR_TE | SCI_SCR_TIE;
  while (SCI1.SCR.BIT.TE) ;
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void INT_Excep_CMT0_CMI0(void)
{
  ++system_ticks;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#else
uint32_t SystemCoreClock = 120000000;
#endif

int close(int fd)
{
    (void)fd;
    return -1;
}
int fstat(int fd, void *pstat)
{
    (void)fd;
    (void)pstat;
    return 0;
}
off_t lseek(int fd, off_t pos, int whence)
{
    (void)fd;
    (void)pos;
    (void)whence;
    return 0;
}
int isatty(int fd)
{
    (void)fd;
    return 1;
}
