/**
 * drv.c
 * Copyright (c) 2021 zhangzhao <zhangzhao@ihep.ac.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**********************************************************************
 *                            Header Files                            *
 **********************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "drv.h"

/**********************************************************************
 *                          Macro Definitions                         *
 **********************************************************************/

/* the default memory page size of the Linux Kernel is 4KB */
#define PAGE_SIZE 4096UL

/* page mask */
#define PAGE_MASK (PAGE_SIZE - 1)

/* base address of the register map. */
#define BASE_ADDR 0x43c0000

/* print error message. */
#define FATAL                                                              \
    do {                                                                   \
        fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", __LINE__, \
                __FILE__, errno, strerror(errno));                         \
        exit(1);                                                           \
    } while (0)

/**********************************************************************
 *                        Hardware Definitions                        *
 **********************************************************************/
/*         Register         Address Offset  No.    Description*/

#define    Control                0x00    /*00*/
#define    EventRamAddr           0x04    /*01*/
#define    EventRamData           0x08    /*02*/
#define    OutputPulseEnables     0x0c    /*03*/
#define    OutputLevelEnables     0x10    /*04*/
#define    TriggerEventEnables    0x14    /*05*/
#define    EventCounterLo         0x18    /*06*/
#define    EventCounterHi         0x1c    /*07*/
#define    TimeStampLo            0x20    /*08*/
#define    TimeStampHi            0x24    /*09*/
#define    EventFifo              0x28    /*10     lsb = event #, msb  lsb of time stamp counter */
#define    EventTimeHi            0x2c    /*11     bits 23-8 of the time stamp counter */
#define    DelayPulseEnables      0x30    /*12*/
#define    DelayPulseSelect       0x34    /*13     OTP pulses have delays and widths. See HW technical reference */
#define    PulseDelay             0x38    /*14*/
#define    PulseWidth             0x3c    /*15*/
#define    IrqVector              0x40    /*16*/
#define    IrqEnables             0x44    /*17*/

/************************
*  Extended registers  *
************************/

#define    DBusEnables            0x48    /*18     Distributed bus enable register */
#define    DBusData               0x4c    /*19     Distributed bus data register (read only) */
#define    Resvd1                 0x58    /*22*/
#define    DelayPrescaler         0x50    /*20     Programmable delay pulse clock prescaler */
#define    FPGAVersion            0x5c    /*23     FPGA Firmware version number */
#define    EventCounterClock      0x54    /*21     Event counter clock prescaler */
#define    Resvd3                 0x60    /*24*/
#define    Resvd4                 0x64    /*25*/
#define    Resvd5                 0x68    /*26*/
#define    Resvd6                 0x6c    /*27*/
#define    Resvd7                 0x70    /*28*/
#define    Resvd8                 0x74    /*29*/
#define    Resvd9                 0x78    /*30*/
#define    Resvd10                0x7c    /*31*/
#define    FP0Map                 0x80    /*32     Front panel output mapping register */
#define    FP1Map                 0x84    /*33     Front panel output mapping register */
#define    FP2Map                 0x88    /*34     Front panel output mapping register */
#define    FP3Map                 0x8c    /*35     Front panel output mapping register */
#define    FP4Map                 0x90    /*36     Front panel output mapping register */
#define    FP5Map                 0x94    /*37     Front panel output mapping register */
#define    FP6Map                 0x98    /*38     Front panel output mapping register */
#define    uSecDivider            0x9c    /*39     Resvd11*/
#define    ExtEventCode           0xa0    /*40*/
#define    ClockControl           0xa4    /*41*/
#define    TSSec                  0xac    /*43*/
#define    SecondsSR              0xa8    /*42*/
#define    Resvd12                0xb0    /*44*/
#define    EvFIFOSec              0xb4    /*45*/
#define    EvFIFOEvCnt            0xb8    /*46*/
#define    OutputPol              0xbc    /*47*/
#define    ExtDelay               0xc0    /*48     extended/32-bit delay register*/
#define    ExtWidth               0xc4    /*49     extended/32-bit width register*/
#define    Presc0                 0xc8    /*50     Front panel clock #0 presaler */
#define    Presc1                 0xcc    /*51     Front panel clock #1 presaler */
#define    Presc2                 0xd0    /*52     Front panel clock #2 presaler */
/*define   Resvd13                0x7a*/
#define    DataBufCtrl            0xd4    /*53     leige */
#define    RFpattern              0xd8    /*54     leige set RFpattern */
#define    FracDiv                0xdc    /*55     leige */

/**********************************************************************
* These registers are only of special interest and left outside EPICS *
* support for the time being. TK, 25-JUL-05.                          *
**********************************************************************/

/* #define    RfDelay                0xe0 */
/* #define    RxDelay                0xe4 */
/* #define    Resvd14                0x00/1* 0x8c leige *1/ */
/* #define    FBRFFrac               0x00/1* 0x90 leige *1/ */
/* #define    FbRxFrac               0x00 */
/* #define    RFDelyInit             0x00 */
/* #define    RxDelyInit             0x00/1* 09C *1/ */
/* #define    CML4Pat00              0x00/1* 0A0 *1/ */
/* #define    CML4Pat01              0x00/1* 0A4 *1/ */
/* #define    CML4Pat10              0x00/1* 0A8 *1/ */
/* #define    CML4Pat11              0x00/1* 0AC *1/ */
/* #define    CML4Ena                0x00/1* 0B0 *1/ */
/* #define    CML4EnaResv_1          0x00/1* 0B4 *1/ */
/* #define    CML4EnaResv_2          0x00/1* 0B8 *1/ */
/* #define    CML4EnaResv_3          0x00/1* 0BC *1/ */
/* #define    CML5Pat00              0x00/1* 0C0 *1/ */
/* #define    CML5Pat01              0x00/1* 0C4 *1/ */
/* #define    CML5Pat10              0x00/1* 0C8 *1/ */
/* #define    CML5Pat11              0x00/1* 0CC *1/ */
/* #define    CML5Ena                0x00/1* 0D0 *1/ */
/* #define    CML5EnaResv_1          0x00/1* 0B4 *1/ */
/* #define    CML5EnaResv_2          0x00/1* 0B8 *1/ */
/* #define    CML5EnaResv_3          0x00/1* 0BC *1/ */
/* #define    CML6Pat00              0x00/1* 0E0 *1/ */
/* #define    CML6Pat01              0x00/1* 0E4 *1/ */
/* #define    CML6Pat10              0x00/1* 0E8 *1/ */
/* #define    CML6Pat11              0x00/1* 0EC *1/ */
/* #define    CML6Ena                0x00/1* 0F0 *1/ */
/* #define    CML6EnaResv_1          0x00/1* 0B4 *1/ */
/* #define    CML6EnaResv_2          0x00/1* 0B8 *1/ */
/* #define    CML6EnaResv_3          0x00/1* 0BC *1/ */


/**********************************************************************
 *                             Functions                              *
 **********************************************************************/

/**********************************************************************
 *                             open_evr()                              *
 ***********************************************************************
 *   Map physical address to virtual address, return virtual address   *
 **********************************************************************/

int* open_evr(off_t map_addr) {

    int fd;
    void *page_addr;

    /* open an image of the main memory */
    if ((fd = open("/dev/uio0", O_RDWR | O_SYNC)) == -1) FATAL;
    printf("/dev/mem opened.\n");

    /* map one page */
    page_addr = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                map_addr & ~PAGE_MASK);
    if (page_addr == (void *)-1) FATAL;

    printf("One page mapped at address %p.\n", page_addr);

    close(fd);

    return(page_addr + ((map_addr)&PAGE_MASK));
}

/***********************************************************************
 *                         close_evr()                            *
 ***********************************************************************
 *   Free the mapped page.	                                           *
 **********************************************************************/

void unmap_pageaddr(void *page_addr) {
    if (munmap(page_addr, PAGE_SIZE) == -1) FATAL;
    printf("The address has been freed.\n");
}

/***********************************************************************
 *                             read_32()                               *
 ***********************************************************************
 *   Read 32-bit date from mapped address.                             *
 **********************************************************************/

void read_32(off_t src, unsigned int *dest) {
    void *page_addr, *virt_addr;

    phyaddr_mapto_pageaddr(src, &page_addr);

    virt_addr = PAGEADDR2VIRTADDR(src, page_addr);

    /* read 4 bytes from the virtual address and put them in *dest. */
    *dest = *((unsigned int *)virt_addr);

    printf("read 32-bits data %08x from %p\n.", *dest, virt_addr);
    unmap_pageaddr(page_addr);
}

/***********************************************************************
 *                             get_32()                                *
 ***********************************************************************
 *   Get 32-bit date of mapped address and return it. 	               *
 **********************************************************************/

unsigned int get_32(off_t src) {
    void *page_addr, *virt_addr;

    phyaddr_mapto_pageaddr(src, &page_addr);

    virt_addr = PAGEADDR2VIRTADDR(src, page_addr);

    /* read 4 bytes from the virtual address and return it. */
    return (*((unsigned int *)virt_addr));

    unmap_pageaddr(page_addr);
}

/***********************************************************************
 *                             write_32()                              *
 ***********************************************************************
 *   Write 32-bit date of mapped address. 	                           *
 **********************************************************************/

void write_32(unsigned int src, off_t dest) {
    void *page_addr, *virt_addr;

    phyaddr_mapto_pageaddr(dest, &page_addr);

    virt_addr = PAGEADDR2VIRTADDR(dest, page_addr);

    /* write 4 bytes of src to the virtual address. */
    *((unsigned int *)virt_addr) = src;

    printf("write 32-bits data %08x at %p\n.", *((unsigned int *)virt_addr), virt_addr);
    unmap_pageaddr(page_addr);
}
