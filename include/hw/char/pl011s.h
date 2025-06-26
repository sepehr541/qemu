#ifndef HW_PL011S_H
#define HW_PL011S_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"
#include "qemu/fifo32.h"

#define TYPE_PL011S "pl011s"
OBJECT_DECLARE_SIMPLE_TYPE(PL011State, PL011S)

/* This shares the same struct (and cast macro) as the base pl011 device */

/* Depth of UART FIFO in bytes, when FIFO mode is enabled (else depth == 1) */
#define PL011_FIFO_DEPTH 32

struct PL011State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    CharBackend be;
    Clock *clk;
    bool migrate_clk;
    Fifo32 rxfifo;
    Fifo32 txfifo;
    uint16_t rsr_ecr;
    uint16_t overrun;
    uint16_t interrupts;
    uint16_t baudrate_int;
    uint16_t baudrate_frac;
    uint16_t isBaudrateSet;
    uint16_t lcr_h;
    uint16_t cr;
    uint16_t fifolevels;
    uint16_t interruptmasks;
    uint16_t txThresholdEverCrossed;
    qemu_irq UARTRXINTR;
    qemu_irq UARTTXINTR;
    qemu_irq UARTRTINTR;
    qemu_irq UARTEINTR;
    qemu_irq UARTINTR;
};

typedef struct PL011State PL011State;

DeviceState *pl011_create(hwaddr addr, qemu_irq irq, Chardev *chr);

#endif
