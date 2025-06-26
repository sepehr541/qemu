#include <stdint.h>
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/fifo32.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "trace.h"

#include "hw/char/pl011s.h"

/***************************
 *         Helpers         *
 ***************************/

static uint32_t rx_threshold(void) {
    return 0;
}


static uint32_t tx_threshold(void) {
    return 1;
}

/***************************
 * Synthesized Operations *
 ***************************/

static uint64_t readUARTCR(PL011State *s) {
    return s->cr;
}


static uint64_t readUARTCellID0(PL011State *s) {
    return 0xd;
}


static uint64_t readUARTCellID1(PL011State *s) {
    return 0xf0;
}


static uint64_t readUARTCellID2(PL011State *s) {
    return 0x5;
}


static uint64_t readUARTCellID3(PL011State *s) {
    return 0xb1;
}


static uint64_t readUARTDR(PL011State *s) {
    assert(!(fifo32_is_empty(&s->rxfifo)));
    s->rsr_ecr = fifo32_pop(&s->rxfifo);
    uint16_t tmp = s->rsr_ecr;
    s->rsr_ecr >>= 0x8;
    if (fifo32_num_used(&s->rxfifo) >= rx_threshold()) {
        s->interrupts |= 0x10;
    } else {
        s->interrupts &= 0x7ef;
    }

    return tmp;
}


static uint64_t readUARTFBRD(PL011State *s) {
    return s->baudrate_frac;
}


static uint64_t readUARTFR(PL011State *s) {
    uint16_t value = 0x0;
    if (fifo32_is_full(&s->rxfifo)) {
        value |= 0x40;
    }

    if (fifo32_is_empty(&s->txfifo)) {
        value |= 0x80;
    }

    if (fifo32_is_full(&s->txfifo)) {
        value |= 0x20;
    }

    if (fifo32_is_empty(&s->rxfifo)) {
        value |= 0x10;
    }

    return value;
}


static uint64_t readUARTIBRD(PL011State *s) {
    return s->baudrate_int;
}


static uint64_t readUARTIFLS(PL011State *s) {
    return s->fifolevels;
}


static uint64_t readUARTIMSC(PL011State *s) {
    return s->interruptmasks;
}


static uint64_t readUARTLCR_H(PL011State *s) {
    return s->lcr_h;
}


static uint64_t readUARTMIS(PL011State *s) {
    uint16_t tmp = s->interrupts;
    tmp &= s->interruptmasks;
    return tmp;
}


static uint64_t readUARTPeriphID0(PL011State *s) {
    return 0x11;
}


static uint64_t readUARTPeriphID1(PL011State *s) {
    return 0x10;
}


static uint64_t readUARTPeriphID2(PL011State *s) {
    return 0xe;
}


static uint64_t readUARTPeriphID3(PL011State *s) {
    return 0x0;
}


static uint64_t readUARTRIS(PL011State *s) {
    return s->interrupts;
}


static uint64_t readUARTRSR_UARTECR(PL011State *s) {
    return s->rsr_ecr;
}


static void receive(PL011State* s, uint16_t value) {
    assert((s->cr & 0x201) == 0x201);
    assert(s->isBaudrateSet == 0x1);
    if (fifo32_is_full(&s->rxfifo)) {
        s->overrun = 0x1;
        s->interrupts |= 0x400;
    } else {
        s->overrun = 0x0;
        fifo32_push(&s->rxfifo, value);
        value &= 0x700;
        value >>= 0x1;
        s->interrupts |= value;
        if (fifo32_num_used(&s->rxfifo) >= rx_threshold()) {
            s->interrupts |= 0x10;
        } else {
            s->interrupts &= 0x7ef;
        }

    }

}


static void transmit(PL011State* s) {
    assert((!(fifo32_is_empty(&s->txfifo)) && ((s->cr & 0x201) == 0x201 && s->isBaudrateSet == 0x1)));
    uint16_t tmp = 0x0;
    tmp = fifo32_pop(&s->txfifo);
    qemu_chr_fe_write_all(&s->be, (const uint8_t*)&tmp, 1);
    if (fifo32_num_used(&s->txfifo) <= tx_threshold()) {
        s->interrupts |= 0x20;
        s->txThresholdEverCrossed = 0x1;
    } else {
        s->interrupts &= 0xffdf;
    }

}


static void updateINTR(PL011State* s) {
    uint16_t intr = 0x0;
    uint16_t tmp = s->interrupts;
    tmp &= s->interruptmasks;
    if ((tmp & 0x10) == 0x10) {
        qemu_set_irq(s->UARTRXINTR, 0x1);
        intr |= 0x100;
    } else {
        qemu_set_irq(s->UARTRXINTR, 0x0);
    }

    if ((tmp & 0x20) == 0x20) {
        qemu_set_irq(s->UARTTXINTR, 0x1);
        intr |= tmp;
    } else {
        qemu_set_irq(s->UARTTXINTR, 0x0);
    }

    if ((tmp & 0x780) != 0) {
        qemu_set_irq(s->UARTEINTR, 0x1);
        intr |= 0x10;
    } else {
        qemu_set_irq(s->UARTEINTR, 0x0);
    }

    qemu_set_irq(s->UARTINTR, intr);
}


static void writeUARTCR(PL011State *s, uint64_t value) {
    s->cr = value;
}


static void writeUARTDR(PL011State *s, uint64_t value) {
    assert(!(fifo32_is_full(&s->txfifo)));
    fifo32_push(&s->txfifo, value);
    if (((s->interrupts & 0x0) == 0x0 && fifo32_num_used(&s->txfifo) <= tx_threshold())) {

    } else {
        s->interrupts &= 0x7df;
    }

}


static void writeUARTFBRD(PL011State *s, uint64_t value) {
    assert(((((s->baudrate_int & 0x0) == 0x0 && s->cr != 0x5) && !((s->cr & 0x1) == 0x1)) && !((s->baudrate_int == 0xffff && value != 0x0))));
    s->baudrate_frac = value;
}


static void writeUARTIBRD(PL011State *s, uint64_t value) {
    assert(((value != 0x0 && s->cr != 0x1) && !((s->cr & 0x1) == 0x1)));
    s->baudrate_int = value;
}


static void writeUARTICR(PL011State *s, uint64_t value) {
    value &= s->interrupts;
    s->interrupts ^= value;
}


static void writeUARTIFLS(PL011State *s, uint64_t value) {
    s->fifolevels = value;
    if (fifo32_num_used(&s->rxfifo) >= rx_threshold()) {
        s->interrupts |= 0x10;
    } else {
        s->interrupts &= 0x7cf;
    }

    if ((s->txThresholdEverCrossed == 0x1 && fifo32_num_used(&s->txfifo) <= tx_threshold())) {
        s->interrupts |= 0x20;
    } else {
        s->interrupts &= 0x8fdf;
    }

}


static void writeUARTIMSC(PL011State *s, uint64_t value) {
    s->interruptmasks = value;
}


static void writeUARTLCR_H(PL011State *s, uint64_t value) {
    assert(!((s->cr & 0x1) == 0x1));
    s->lcr_h = value;
    uint16_t a = value;
    a &= 0x10;
    if (a == 0x0) {
        fifo32_reset(&s->rxfifo);
        fifo32_reset(&s->txfifo);
    }

    if ((s->baudrate_int == 0x0 || (s->baudrate_int == 0xffff && s->baudrate_frac != 0x0))) {
        s->isBaudrateSet = 0x0;
    } else {
        s->isBaudrateSet = 0x1;
    }

}

static void writeUARTRSR_UARTECR(PL011State *s, uint64_t value) {
    s->rsr_ecr = 0x0;
    return;
}

/***************************
 * MemoryRegion Operations *
 ***************************/


static uint64_t pl011s_read(void* opaque, hwaddr offset, unsigned size) {
    PL011State* s = (PL011State*) opaque;
    uint64_t res = 0;
    switch (offset) {
    case 0:
        res = readUARTDR(s);
        break;
    case 4:
        res = readUARTRSR_UARTECR(s);
        break;
    case 24:
        res = readUARTFR(s);
        break;
    case 32:
        res = 0;
        break;
    case 36:
        res = readUARTIBRD(s);
        break;
    case 40:
        res = readUARTFBRD(s);
        break;
    case 44:
        res = readUARTLCR_H(s);
        break;
    case 48:
        res = readUARTCR(s);
        break;
    case 52:
        res = readUARTIFLS(s);
        break;
    case 56:
        res = readUARTIMSC(s);
        break;
    case 60:
        res = readUARTRIS(s);
        break;
    case 64:
        res = readUARTMIS(s);
        break;
    case 72:
        res = 0;
        break;
    case 4064:
        res = readUARTPeriphID0(s);
        break;
    case 4068:
        res = readUARTPeriphID1(s);
        break;
    case 4072:
        res = readUARTPeriphID2(s);
        break;
    case 4076:
        res = readUARTPeriphID3(s);
        break;
    case 4080:
        res = readUARTCellID0(s);
        break;
    case 4084:
        res = readUARTCellID1(s);
        break;
    case 4088:
        res = readUARTCellID2(s);
        break;
    case 4092:
        res = readUARTCellID3(s);
        break;
    default:
        res = 0;
    }
    updateINTR(s);
    return res;
}


static void pl011s_write(void* opaque, uint64_t offset, uint64_t value, unsigned size) {
    PL011State* s = (PL011State*) opaque;
    switch (offset) {
    case 0:
        writeUARTDR(s, value);
        transmit(s);
        break;
    case 4:
        writeUARTRSR_UARTECR(s, value);
        break;
    case 32:
        /* writeUARTILPR(s, value); */
        break;
    case 36:
        writeUARTIBRD(s, value);
        break;
    case 40:
        writeUARTFBRD(s, value);
        break;
    case 44:
        writeUARTLCR_H(s, value);
        break;
    case 48:
        writeUARTCR(s, value);
        break;
    case 52:
        writeUARTIFLS(s, value);
        break;
    case 56:
        writeUARTIMSC(s, value);
        break;
    case 68:
        writeUARTICR(s, value);
        break;
    case 72:
        /* writeUARTMACR(s, value); */
        break;
    default:
        break;
    }
    updateINTR(s);
}

/**
 * @var mypl011_mem_ops
 * @brief Register Callbacks with QEMU for read/write to PL011
 */
static const MemoryRegionOps pl011_mem_ops = {
    .read = pl011s_read,
    .write = pl011s_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.max_access_size = 4,
    .impl.min_access_size = 4
};



/***************************
 * Lifecycle Callbacks  *
 ***************************/

static int pl011s_can_receive(void *opaque)
{
    PL011State *s = (PL011State *)opaque;
    return  !fifo32_is_full(&s->rxfifo);
}

static void pl011s_receive(void *opaque, const uint8_t *buf, int size)
{
    PL011State *s = PL011S(opaque);
    
    for (int i = 0; i < size; i++)
        receive(s, buf[i]);

    updateINTR(s);
}

static void pl011s_event(void *opaque, QEMUChrEvent event)
{
    PL011State *s = PL011S(opaque);
    if (event == CHR_EVENT_BREAK) {
        receive(s, 0x0400);
    }
}

/**
 * @brief Reset DeviceState
 * @details Emulate resetting the device
 * @param[in,out] DeviceState of PL011
 */
static void pl011s_reset(DeviceState *dev) {
    PL011State *pl011 = PL011S(dev);

    /* Reset FIFOs */
    fifo32_reset(&pl011->rxfifo);
    fifo32_reset(&pl011->txfifo);
    
    /* Reset Registers */
    pl011->rsr_ecr = 0;
    pl011->overrun = 0;
    pl011->interrupts = 0;
    pl011->baudrate_int = 0;
    pl011->baudrate_frac = 0;
    pl011->isBaudrateSet = 0;
    pl011->lcr_h = 0;
    pl011->cr = 0x0300;
    pl011->fifolevels = 0x12;
    pl011->interruptmasks = 0;
    pl011->txThresholdEverCrossed = 0;

    /* Reset IRQs */
    qemu_set_irq(pl011->UARTINTR, false);
    qemu_set_irq(pl011->UARTRXINTR, false);
    qemu_set_irq(pl011->UARTTXINTR, false);
    qemu_set_irq(pl011->UARTRTINTR, false);
    qemu_set_irq(pl011->UARTEINTR, false);
}


/**
 * @brief Summary
 * @details Description
 * @param[in,out] dev Description
 * @param[out] errp Description
 */
static void pl011s_realize(DeviceState *dev, Error **errp) {
    
    PL011State *pl011 = PL011S(dev);
    fifo32_create(&pl011->rxfifo, 32);
    fifo32_create(&pl011->txfifo, 32);
    pl011s_reset(dev);

    qemu_chr_fe_set_handlers(
        &pl011->be,
        pl011s_can_receive,
        pl011s_receive,
        pl011s_event,
        NULL,
        pl011,
        NULL,
        true);   
}


static void pl011s_unrealize(DeviceState *dev) {
    PL011State *pl011 = PL011S(dev);
    fifo32_destroy(&pl011->rxfifo);
    fifo32_destroy(&pl011->txfifo);
}



/*
  QOM and QDEV setup
 */

/**
 * @brief DeviceState Init
 * @details QEMU required
 * @param[in,out] obj Object instance, cast required
 */
static void pl011s_init(Object *obj) {
    
    /* Cast to Device */
    PL011State *pl011 = PL011S(obj);
    SysBusDevice *sysbusdev = SYS_BUS_DEVICE(obj);

    /* QEMU: Get MMIO memory region for PL011 */
    memory_region_init_io(&pl011->iomem, obj, &pl011_mem_ops, pl011, TYPE_PL011S, 0x1000);

    /* initialize mmio region in DeviceState */
    sysbus_init_mmio(sysbusdev, &pl011->iomem);

    /* QEMU: Initialize Interrupt Lines for PL011 */
    sysbus_init_irq(sysbusdev, &pl011->UARTINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTRXINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTTXINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTRTINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTEINTR);
}

static Property pl011s_properties[] = {
    DEFINE_PROP_CHR("chardev", PL011State, be),
    DEFINE_PROP_END_OF_LIST()
};

/**
 * @brief DeviceClass Init
 * @details QEMU Required
 * @param[in,out] klass ObjectClass
 * @param[out] class_data N/A
 */
static void pl011s_class_init(ObjectClass *klass, void *class_data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = pl011s_realize;
    dc->unrealize = pl011s_unrealize;
    device_class_set_legacy_reset(dc, pl011s_reset);
    device_class_set_props(dc, pl011s_properties);
}

/**
 * @var my_device_info
 * @brief TypeInfo, QEMU required
 */
static const TypeInfo pl011s_info = {
    .name = TYPE_PL011S,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PL011State),
    .instance_init = pl011s_init,
    .class_init = pl011s_class_init,
};

static void pl011s_register_types(void) {
    type_register_static(&pl011s_info);
}

type_init(pl011s_register_types);
