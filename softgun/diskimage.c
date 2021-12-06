/*
 **************************************************************************************************
 * diskimage.c
 *	Library for diskimages for use in nonvolatile memory device
 *	emulators like amdflash or serial eeproms and MMC/SD-Cards
 *
 * (C) 2005 Jochen Karrer
 *    Author: Jochen Karrer
 *
 * Status: Working 
 *
 * Copyright 2010 Jochen Karrer. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list
 *       of conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY Jochen Karrer ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those of the
 * authors and should not be interpreted as representing official policies, either expressed
 * or implied, of Jochen Karrer.
 *
 *************************************************************************************************
 */

#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
//#include <sys/file.h>
//#include <sys/mman.h>
#include <sys/types.h>
//#include <sys/ioctl.h>
#include <io.h>
#include <assert.h>
#include "diskimage.h"
#include "sgstring.h"

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

uint8_t *disk_image;
uint64_t disk_image_size;

#define S_IFBLK (~S_IFMT)
/*
 * ------------------------------------------------------------------------
 * Fill a diskimage with emptyval. This is intended for 
 * new non-sparse diskimages
 * ------------------------------------------------------------------------
 */
static int
diskimage_fill(DiskImage * di, uint8_t emptyval)
{
	memset(di->map, emptyval, di->size);
	return 0;
}

static int
diskimage_sparse_fill(DiskImage * di, uint8_t emptyval)
{
	uint8_t *pos = (uint8_t *)di->map;
	if (di->size < 1) {
		infprintf("Sparse diskimage is < 1 Byte\n");
		return -1;
	}
	if (pos == NULL) {
		perror("lseek64 on flashfile failed");
		return -1;
	}
	pos[di->size - 1] = emptyval;

	return 0;
}

static int
check_fill(DiskImage * di, int flags)
{
	uint8_t emptyval;
	if (flags & DI_CREAT_FF) {
		emptyval = 0xff;
	} else {
		emptyval = 0x00;
	}
	if (flags & DI_SPARSE) {
		if (diskimage_sparse_fill(di, emptyval) < 0) {
			infprintf("Filling the sparse diskimage failed\n");
			return -1;
		}
	} else {
		if (diskimage_fill(di, emptyval) < 0) {
			infprintf("Filling the diskimage failed\n");
			return -1;
		}
	}
	return 0;
}

/**
 ************************************************************************
 * Return the size of a block device in bytes
 ************************************************************************
 */
static uint64_t
blockdev_get_size(DiskImage * di)
{
#if 0
	uint64_t size;
	ioctl(di->fd, BLKGETSIZE64, &size);
	return size;
#endif
	return 0;
}

DiskImage *
DiskImage_Open(const char *name, uint64_t size, int flags)
{
	DiskImage *di;
	di = sg_new(DiskImage);
	di->size = size;
	(void)flags;
	di->map = malloc(size);
	if (di->map == NULL) {
		infprintf("Can't open image \"%s\" ", name);
		perror("");
		free(di);
		return NULL;
	}
	disk_image = di->map;
	disk_image_size = di->size;
	return di;
}

int
DiskImage_Read(DiskImage * di, off_t ofs, uint8_t * buf, int count)
{
	int cnt = count;
	uint8_t *pos = di->map;

	if (cnt > di->size - ofs)
		cnt = di->size - ofs;

	memcpy(buf , &pos[ofs], cnt);

	return cnt;
}

int
DiskImage_Write(DiskImage * di, off_t ofs, const uint8_t * buf, int count)
{
	int cnt = count;
	uint8_t *pos = di->map;

	if (cnt > di->size - ofs)
		cnt = di->size - ofs;

	memcpy(&pos[ofs], buf, cnt);

	return cnt;
}

void *
DiskImage_Mmap(DiskImage * di)
{
	return di->map;
}

void
DiskImage_Close(DiskImage * di)
{
	free(di->map);
	free(di);
}
