/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_MYPL011_H
#define HW_MYPL011_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_MYPL011 "MYPL011"
OBJECT_DECLARE_SIMPLE_TYPE(MyPL011State, MyPL011);

/* This shares the same struct (and cast macro) as the base pl011 device */
#define TYPE_MYPL011_LUMINARY "mypl011_luminary"

/* Depth of UART FIFO in bytes, when FIFO mode is enabled (else depth == 1) */
#define MYPL011_FIFO_DEPTH 1

/* UARTDR; */
#define UARTDR_RESET_VAL 0
#define UARTDR_DATA 0xFF
#define UARTDR_FE BIT(8)
#define UARTDR_PE BIT(9)
#define UARTDR_BE BIT(10)
#define UARTDR_OE BIT(11)

/* UARTRSR_ECR; */
#define UARTRSR_ECR_RESET_VAL 0
#define UARTRSR_ECR_FE BIT(0)
#define UARTRSR_ECR_PE BIT(1)
#define UARTRSR_ECR_BE BIT(2)
#define UARTRSR_ECR_OE BIT(3)
#define UARTRSR_ECR_ECR 0xFF << 8

/* UARTFR; */
#define UARTFR_CTS BIT(0)
#define UARTFR_DSR BIT(1)
#define UARTFR_DCD BIT(2)
#define UARTFR_BUSY BIT(3)
#define UARTFR_RXFE BIT(4)
#define UARTFR_TXFF BIT(5)
#define UARTFR_RXFF BIT(6)
#define UARTFR_TXFE BIT(7)
#define UARTFR_RI BIT(8)
#define UARTFR_RESET_VAL UARTFR_RXFE | UARTFR_TXFE

/* UARTILPR; */
#define UARTILPR_ILPDVSR 0xFF
#define UARTILPR_RESET_VAL 0

/* UARTIBRD; */
#define UARTIBRD_BAUD_DIVINT  0xFFFF
#define UARTIBRD_RESET_VAL  0

/* UARTFBRD;     */
#define UARTFBRD_RESET_VAL 0
#define UARTFBRD_BAUD_DIVFRAC 0x3F


/* UARTLCR_H; */
#define UARTLCR_H_RESET_VAL 0
#define UARTLCR_H_BRK BIT(0)
#define UARTLCR_H_PEN BIT(1)
#define UARTLCR_H_EPS BIT(2)
#define UARTLCR_H_STP2 BIT(3)
#define UARTLCR_H_FEN BIT(4)
#define UARTLCR_H_WLEN 3 << 5
#define UARTLCR_H_SPS BIT(7)

/* UARTCR; */
#define UARTCR_RESET_VAL 0x0300
#define UARTCR_UARTEN BIT(0)
#define UARTCR_SIREN BIT(1)
#define UARTCR_SIRLP BIT(2)
/* 6:3 Reserved */
#define UARTCR_LBE BIT(7)
#define UARTCR_TXE BIT(8)
#define UARTCR_RXE BIT(9)
#define UARTCR_DTR BIT(10)
#define UARTCR_RTS BIT(11)
#define UARTCR_Out1 BIT(12)
#define UARTCR_Out2 BIT(13)
#define UARTCR_RTSEn BIT(14)
#define UARTCR_CTSEn BIT(15)

/* UARTIFLS; */
#define UARTIFLS_TXIFLSEL 0x7 << 0
#define UARTIFLS_RXIFLSEL 0x7 << 3
#define UARTIFLS_RESET_VAL (unsigned short)0b010010

/* UARTIMSC; */
#define UARTIMSC_RESET_VAL 0
#define UARTIMSC_RIMIM BIT(0)
#define UARTIMSC_CTSMIM BIT(1)
#define UARTIMSC_DCDMIM BIT(2)
#define UARTIMSC_DSRMIM BIT(3)
#define UARTIMSC_RXIM BIT(4)
#define UARTIMSC_TXIM BIT(5)
#define UARTIMSC_RTIM BIT(6)
#define UARTIMSC_FEIM BIT(7)
#define UARTIMSC_PEIM BIT(8)
#define UARTIMSC_BEIM BIT(9)
#define UARTIMSC_OEIM BIT(10)

/* UARTRIS; */
#define UARTRIS_RESET_VAL 0
#define UARTRIS_RIRMIS BIT(0)
#define UARTRIS_CTSRMIS BIT(1)
#define UARTRIS_DCDRMIS BIT(2)
#define UARTRIS_DSRRMIS BIT(3)
#define UARTRIS_RXRIS BIT(4)
#define UARTRIS_TXRIS BIT(5)
#define UARTRIS_RTRIS BIT(6)
#define UARTRIS_FERIS BIT(7)
#define UARTRIS_PERIS BIT(8)
#define UARTRIS_BERIS BIT(9)
#define UARTRIS_OERIS BIT(10)

/* UARTMIS; */
#define UARTMIS_RESET_VAL 0 // except 3:0
#define UARTMIS_RIMMIS BIT(0)
#define UARTMIS_CTSMMIS BIT(1)
#define UARTMIS_DCDMMIS BIT(2)
#define UARTMIS_DSRMMIS BIT(3)
#define UARTMIS_RXMIS BIT(4)
#define UARTMIS_TXMIS BIT(5)
#define UARTMIS_RTMIS BIT(6)
#define UARTMIS_FEMIS BIT(7)
#define UARTMIS_PEMIS BIT(8)
#define UARTMIS_BEMIS BIT(9)
#define UARTMIS_OEMIS BIT(10)

/* UARTICR; */
#define UARTICR_RIMIC BIT(0)
#define UARTICR_CTSMIC BIT(1)
#define UARTICR_DCDMIC BIT(2)
#define UARTICR_DSRMIC BIT(3)
#define UARTICR_RXIC BIT(4)
#define UARTICR_TXIC BIT(5)
#define UARTICR_RTIC BIT(6)
#define UARTICR_FEIC BIT(7)
#define UARTICR_PEOC BIT(8)
#define UARTICR_BEIC BIT(9)
#define UARTICR_OEIC BIT(10)

/* UARTDMACR; */
#define UARTDMACR_RESET_VAL 0
#define UARTDMACR_RXDMAE  BIT(0)
#define UARTDMACR_TXDMAE BIT(1)
#define UARTDMACR_DMAONERR BIT(2)


/* const unsigned char pl011_id_arm[8] = */
/*   { 0x11, 0x10, 0x14, 0x00, 0x0d, 0xf0, 0x05, 0xb1 }; */

/* UARTPeriphID; */
#define UARTPeriphID0 0x11
#define UARTPeriphID1 0x10
#define UARTPeriphID2 0x14
/* #define UARTPeriphID3 0x00 */
/* #define UARTPeriphID (UARTPeriphID0 << 12) | (UARTPeriphID1 << 8) | (UARTPeriphID2 << 4) | (UARTPeriphID3 << 0) */
    

/* UARTCellID;    */
#define UARTCellID0 0x0d
#define UARTCellID1 0xf0
#define UARTCellID2 0x05
#define UARTCellID3 0xb1

typedef uint32_t reg32;

struct MyPL011State {
    /* QEMU emulation state */
    struct {
        SysBusDevice parent_obj;
        MemoryRegion iomem;
        CharBackend chr;
        qemu_irq irq[6];
        Clock *clk;
        bool migrate_clk;
    };

    /* Device State */
    struct {
        reg32 UARTDR;
        reg32 UARTRSR_ECR;
        reg32 UARTFR;
        reg32 UARTILPR;
        reg32 UARTIBRD;
        reg32 UARTFBRD;    
        reg32 UARTLCR_H;
        reg32 UARTCR;
        reg32 UARTIFLS;
        reg32 UARTIMSC;
        reg32 UARTRIS;
        reg32 UARTMIS;
        reg32 UARTICR;
        reg32 UARTDMACR;
        reg32 UARTPeriphID; /* RO: 0x1110_400 */
        reg32 UARTCellID;   /* RO: 0x0DF005B1 */
    };
    
    struct {
        bool int_enabled;
        reg32 int_level;
    };
    
    /* FIFO */
    struct {
        reg32 read_fifo[MYPL011_FIFO_DEPTH];
        int read_pos;
        int read_count;
        int read_trigger;
    };
    /* const unsigned char *id; */
};

DeviceState *mypl011_create(hwaddr addr, qemu_irq irq, Chardev *chr);

#endif
