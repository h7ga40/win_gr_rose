#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "mmcdev.h"
#include "sgstring.h"
#include "cycletimer.h"

static int infprintf(const char *fmt, ...)
{
	int result;
	va_list ap;

	va_start(ap, fmt);

#if 0
	result = vfprintf(stderr, fmt, ap);
#else
	result = 0;
#endif

	va_end(ap);

	return result;
}

//typedef int MMCDev_DataSink(void *dev,const uint8_t *data,int count);
void
MMCDev_AddListener(MMCDev * mmcdev, void *dev, int maxpkt, MMCDev_DataSink * proc)
{
	MMC_Listener *li = sg_new(MMC_Listener);
	li->device = dev;
	li->maxpkt = maxpkt;
	li->dataSink = proc;
	li->next = mmcdev->listener_head;
	if (li->next) {
		infprintf("Bug: currently only one listener per MMC card allowed\n");
	}
	mmcdev->listener_head = li;
}

void
MMCDev_RemoveListener(MMCDev * mmcdev, void *dev)
{
	MMC_Listener *li = mmcdev->listener_head;
	if (li->device != dev) {
		infprintf("MMCard: Removing wrong device from listeners list\n");
	}
	mmcdev->listener_head = NULL;
	free(li);
}

static void
MMCDev_DoTransmission(void *clientData)
{
	MMCDev *mmcdev = (MMCDev *) clientData;
	MMC_Listener *li = mmcdev->listener_head;
	int len;
	int result;
	uint64_t cycles;
	uint32_t freq;

	if (!li) {
		/* Old style Read Interface */
		return;
	}
	if (CycleTimer_IsActive(&mmcdev->transmissionTimer)) {
		infprintf("Error: Card Transmission timer is already running\n");
		return;
	}
	len = li->maxpkt < sizeof(li->buf) ? li->maxpkt : sizeof(li->buf);
	result = MMCDev_Read(mmcdev, li->buf, len);
	if (result != len) {
		//return;
		infprintf("MMCDev_Read %d\n", result);
		if (result < len)
			memset(li->buf, result, len - result);
	}
//   fprintf(stderr,"MMCard: Do the transmission len %d, transfer cnt %d\n",result,card->transfer_count); // jk
	//MMC_CRC16Init(&dataBlock.crc,0);
	//MMC_CRC16(&dataBlock.crc,dataBlock.data,dataBlock.datalen);
	li->dataSink(li->device, li->buf, len);
	freq = Clock_Freq(mmcdev->clock);
	if (!freq) {
		freq = 1;
		infprintf("Error: MMCard used with clock of 0 HZ\n");
	}
#if 0
	if (mmcdev->type == CARD_TYPE_MMC) {
		cycles = NanosecondsToCycles(1000000000 / freq * 10 * result);
	} else {
		cycles = NanosecondsToCycles((1000000000 / 4) / freq * 10 * result);
	}
#else
	cycles = NanosecondsToCycles((1000000000 / 4) / freq * 10 * result);
#endif
	if (CycleTimer_IsActive(&mmcdev->transmissionTimer)) {
		infprintf("MMCDev: Bug, transmission timer is already running !\n");
	} else {
		CycleTimer_Mod(&mmcdev->transmissionTimer, cycles);
	}
}

void
MMCDev_Init(MMCDev * card, const char *name)
{
#if 0
	card->clock = Clock_New("%s.clk", name);
	Clock_SetFreq(card->clock, 16 * 1000 * 1000);	/* Bad, the clock should come from controller */
#endif
	CycleTimer_Init(&card->transmissionTimer, MMCDev_DoTransmission, card);
}
