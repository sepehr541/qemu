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
#include "qemu/fifo32.h"
#include "qom/object.h"

#define TYPE_MYPL011 "MYPL011"
OBJECT_DECLARE_SIMPLE_TYPE(MyPL011State, MYPL011)

/* Depth of UART FIFO in bytes, when FIFO mode is enabled (else depth == 1) */
#define MYPL011_FIFO_DEPTH 32

/* UARTDR; */
#define UARTDR_OFFSET 0x0
#define UARTDR_RESET_VAL 0u
#define UARTDR_DATA 0xFFu
#define UARTDR_FE BIT(8)
#define UARTDR_PE BIT(9)
#define UARTDR_BE BIT(10)
#define UARTDR_OE BIT(11)

/* UARTRSR_ECR; */
#define UARTRSR_ECR_OFFSET 0x4
#define UARTRSR_ECR_RESET_VAL 0u
#define UARTRSR_ECR_FE BIT(0)
#define UARTRSR_ECR_PE BIT(1)
#define UARTRSR_ECR_BE BIT(2)
#define UARTRSR_ECR_OE BIT(3)
#define UARTRSR_ECR_ECR 0xFF << 8u

/* UARTFR; */
#define UARTFR_OFFSET 0x18
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
#define UARTILPR_OFFSET 0x20 
#define UARTILPR_ILPDVSR 0xFFu
#define UARTILPR_RESET_VAL 0u

/* UARTIBRD; */
#define UARTIBRD_OFFSET 0x24
#define UARTIBRD_BAUD_DIVINT  0xFFFFu
#define UARTIBRD_RESET_VAL  0u

/* UARTFBRD;     */
#define UARTFBRD_OFFSET 0x28
#define UARTFBRD_RESET_VAL 0u
#define UARTFBRD_BAUD_DIVFRAC 0x3Fu


/* UARTLCR_H; */
#define UARTLCR_H_OFFSET 0x2C
#define UARTLCR_H_RESET_VAL 0u
#define UARTLCR_H_BRK BIT(0)
#define UARTLCR_H_PEN BIT(1)
#define UARTLCR_H_EPS BIT(2)
#define UARTLCR_H_STP2 BIT(3)
#define UARTLCR_H_FEN BIT(4)
#define UARTLCR_H_WLEN 3 << 5
#define UARTLCR_H_SPS BIT(7)

/* UARTCR; */
#define UARTCR_OFFSET 0x30
#define UARTCR_RESET_VAL 0x0300u
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
#define UARTIFLS_OFFSET 0x34
#define UARTIFLS_TXIFLSEL 0x7u << 0
#define UARTIFLS_RXIFLSEL_SHIFT 3
#define UARTIFLS_RXIFLSEL_MASK 0x7u << 3
#define UARTIFLS_RESET_VAL 0x12

#define UARTIFLS_1_8 0u
#define UARTIFLS_1_4 1u
#define UARTIFLS_1_2 2u
#define UARTIFLS_3_4 3u
#define UARTIFLS_7_8 4u

/* Interrupt Bit Masks */
#define RIINTR BIT(0)
#define CTSINTR BIT(1)
#define DCDINTR BIT(2)
#define DSRINTR BIT(3)
#define MSINTR (RIINTR | CTSINTR | DCDINTR | DSRINTR)

#define RXINTR BIT(4)
#define TXINTR BIT(5)
#define RTINTR BIT(6)

#define FEINTR BIT(7)
#define PEINTR BIT(8)
#define BEINTR BIT(9)
#define OEINTR BIT(10)
#define EINTR_MASK (FEINTR | PEINTR | BEINTR | OEINTR)


/* UARTIMSC; */
#define UARTIMSC_OFFSET 0x38
#define UARTIMSC_RESET_VAL 0u

/* UARTRIS; */
#define UARTRIS_OFFSET 0x3C
#define UARTRIS_RESET_VAL 0u

/* UARTMIS; */
#define UARTMIS_OFFSET 0x40
#define UARTMIS_RESET_VAL 0u // except 3:0

/* UARTICR; */
#define UARTICR_OFFSET 0x44
#define UARTICR_RESET_VAL 0u

/* UARTDMACR; */
#define UARTDMACR_OFFSET 0x48
#define UARTDMACR_RESET_VAL 0u
#define UARTDMACR_RXDMAE  BIT(0)
#define UARTDMACR_TXDMAE BIT(1)
#define UARTDMACR_DMAONERR BIT(2)


/* const unsigned char pl011_id_arm[8] = */
/*   { 0x11, 0x10, 0x14, 0x00, 0x0d, 0xf0, 0x05, 0xb1 }; */
/* UARTPeriphID; */
#define UARTPeriphID0_OFFSET 0xFE0
#define UARTPeriphID1_OFFSET 0xFE4
#define UARTPeriphID2_OFFSET 0xFE8
#define UARTPeriphID3_OFFSET 0xFEC

/* UARTCellID;    */
#define UARTCellID0_OFFSET 0xFF0
#define UARTCellID1_OFFSET 0xFF4
#define UARTCellID2_OFFSET 0xFF8
#define UARTCellID3_OFFSET 0xFFC

typedef uint32_t reg32;
typedef uint16_t reg16;

struct MyPL011State {
    /* QEMU emulation state */
    struct {
        SysBusDevice parent_obj;
        /* MMIO region */
        MemoryRegion iomem;
        CharBackend chr;
        Clock *clk;
        bool migrate_clk;
    };

    /* IRQ lines */
    struct {
        qemu_irq UARTINTR;   /* combined interrupt line */
        qemu_irq UARTRXINTR; /* receive FIFO interrupt line */
        qemu_irq UARTTXINTR; /* transmit FIFO interrupt line */
        qemu_irq UARTRTINTR; /* receive timeout interrupt line */
        qemu_irq UARTMSINTR; /* modem status interrupt line */
        qemu_irq UARTEINTR;  /* error interrupt line */
    };

    /* Device State */
    struct {
        reg16 rsr_ecr;
        reg16 overrun;
        reg16 interrupts;
        reg16 baudrate_int;
        reg16 baudrate_frac;
        reg16 isBaudrateSet;
        reg16 lcr_h;
        reg16 cr;
        reg16 fifolevels;
        reg16 interruptmasks;
        reg16 txThresholdEverCrossed;
    };

    /*********************
      Data Register
    *********************/
    Fifo32 rxfifo;
    Fifo32 txfifo;
    
};

typedef struct MyPL011State PL011State;

DeviceState *mypl011_create(hwaddr addr, qemu_irq irq, Chardev *chr);

#endif
