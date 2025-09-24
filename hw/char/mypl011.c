#include "qemu/osdep.h"
#include "qemu/fifo32.h"
#include "qapi/error.h"
#include "hw/char/mypl011.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "trace.h"
#include <stdint.h>

#define MyPL011_REG_SIZE 0x1000
#define RX_TIMEOUT_MS 10

/*********************
 * Constant State    *
 *********************/

/* Peripheral Identification Registers */ 
/* static unsigned char UARTPeriphID[] = {0x11, 0x10, 0x14, 0x00}; */
/* PrimeCell Identification Registers */ 
/* static unsigned char UARTCellID[] = {0x0d, 0xf0, 0x05, 0xb1}; */

static const char *pl011_regname(hwaddr offset)
{
    /* trace_mypl011_call(__func__); */
    static const char *const rname[] = {
        [0] = "DR", [1] = "RSR", [6] = "FR", [8] = "ILPR", [9] = "IBRD",
        [10] = "FBRD", [11] = "LCRH", [12] = "CR", [13] = "IFLS", [14] = "IMSC",
        [15] = "RIS", [16] = "MIS", [17] = "ICR", [18] = "DMACR",
    };
    unsigned idx = offset >> 2;

    if (idx < ARRAY_SIZE(rname) && rname[idx]) {
        return rname[idx];
    }
    if (idx >= 0x3f8 && idx <= 0x400) {
        return "ID";
    }
    return "UNKN";
}

/*********************
 *  Flags            *
 *********************/

static inline bool pl011_loopback_enabled(MyPL011State *pl011)
{
    trace_mypl011_call(__func__);
    return !!(pl011->cr & UARTCR_LBE);
}

static inline bool pl011_is_fifo_enabled(MyPL011State *pl011)
{
    trace_mypl011_call(__func__);
    return (pl011->lcr_h & UARTLCR_H_FEN) != 0;
}


__attribute__((unused))
static unsigned int pl011_get_baudrate(const MyPL011State *pl011)
{
    trace_mypl011_call(__func__);
    uint64_t clk;

    if (pl011->baudrate_int == 0) {
        return 0;
    }

    clk = clock_get_hz(pl011->clk);
    return (clk / ((pl011->baudrate_int << 6) + pl011->baudrate_frac)) << 2;
}

__attribute__((unused))
static void pl011_trace_baudrate_change(const MyPL011State *pl011)
{
    trace_mypl011_call(__func__);
    trace_mypl011_baudrate_change(pl011_get_baudrate(pl011),
                                  clock_get_hz(pl011->clk),
                                  pl011->baudrate_int,
                                  pl011->baudrate_frac);
}

/*********************
 *  Interrupts       *
 *********************/
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
    
    if ((tmp & 0x40) == 0x40) {
        qemu_set_irq(s->UARTRTINTR, 0x1);
        intr |= tmp;
    } else {
        qemu_set_irq(s->UARTRTINTR, 0x0);
    }

    if ((tmp & 0x780) != 0) {
        qemu_set_irq(s->UARTEINTR, 0x1);
        intr |= 0x10;
    } else {
        qemu_set_irq(s->UARTEINTR, 0x0);
    }

    qemu_set_irq(s->UARTINTR, intr);
}




/***************************
 * I/O Operations *
 ***************************/
enum fifo_enum {
    RXFIFO,
    TXFIFO
};

static bool fifo_full(PL011State *s, enum fifo_enum fifo) {
    uint16_t max_length = 0x0;
    if ((s->lcr_h & 0x10) == 0x10) {
        max_length = MYPL011_FIFO_DEPTH;
    } else {
        max_length = 0x1;
    }

    uint16_t fifo_length = 0x0;
    if (fifo == RXFIFO) {
        fifo_length = fifo32_num_used(&s->rxfifo);
    } else {
        fifo_length = fifo32_num_used(&s->txfifo);
    }

    if (max_length == fifo_length) {
        return 0x1;
    } else {
        return 0x0;
    }

}

static uint16_t fifo_threshold(PL011State *s, enum fifo_enum fifo) {
    if (!((s->lcr_h & 0x10) == 0x10)) {
        return 0x1;
    }

    uint16_t level = s->fifolevels;
    if (fifo == RXFIFO) {
        level &= 0x38;
        level >>= 0x3;
    } else {
        level &= 0x7;
    }

    uint16_t numer = 0x0;
    if (level == 0x0) {
        numer = 0x1;
    }

    if (level == 0x1) {
        numer = 0x2;
    }

    if (level == 0x2) {
        numer = 0x4;
    }

    if (level == 0x3) {
        numer = 0x6;
    }

    if (level == 0x4) {
        numer = 0x7;
    }

    numer *= MYPL011_FIFO_DEPTH;
    numer >>= 0x3;
    return numer;
}

static void receive(PL011State* s, uint16_t value) {
    assert(((s->cr & 0x201) == 0x201 && s->isBaudrateSet == 0x1));
    if (fifo_full(s, RXFIFO)) {
        s->overrun = 0x1;
        s->interrupts |= 0x400;
    } else {
        fifo32_push(&s->rxfifo, value);
        s->overrun = 0x0;
        value &= 0x700;
        value >>= 0x1;
        s->interrupts |= value;
        if (fifo32_num_used(&s->rxfifo) >= fifo_threshold(s, RXFIFO)) {
            s->interrupts |= 0x10;
        } else {
            s->interrupts &= 0x7ef;
        }

    }

}


static void transmit(PL011State* s) {
    assert(((s->isBaudrateSet != 0x0 && !(fifo32_is_empty(&s->txfifo))) && (s->isBaudrateSet == 0x1 && (s->cr & 0x201) == 0x201)));
    uint8_t tmp = 0x0;
    tmp = (uint8_t)fifo32_pop(&s->txfifo);
    qemu_chr_fe_write_all(&s->chr, &tmp, 1);
    if (fifo32_num_used(&s->txfifo) <= fifo_threshold(s, TXFIFO)) {
        s->interrupts |= 0x20;
        s->txThresholdEverCrossed = 0x20;
    } else {
        s->interrupts &= 0xffdf;
    }

}


/**
 * @brief Can_receive predicate
 * @details 
 * @param[in,out] opaque Description
 * @return number of bytes the device can receive
 */
static int pl011_can_receive(void *opaque) {
    trace_mypl011_call(__func__);
    
    MyPL011State *pl011 = (MyPL011State *)opaque;

    int free_slots = fifo32_num_free(&pl011->rxfifo);
    trace_mypl011_can_receive(pl011->lcr_h, free_slots, free_slots);
    if ((pl011->cr & UARTCR_RXE) == 0 ||
        (pl011->cr & UARTCR_UARTEN) == 0)
        return 0;
    
    return free_slots;
}

/**
 * @brief Summary
 * @details Description
 * @param[in,out] opaque Description
 * @param[in] buf Pointer to buffer to ?
 * @param[in] size Description
 */
static void pl011_receive(void *opaque, const uint8_t *buf, int size) {
    trace_mypl011_call(__func__);
    
    /*
     * In loopback mode, the RX input signal is internally disconnected
     * from the entire receiving logics; thus, all inputs are ignored,
     * and BREAK detection on RX input signal is also not performed.
     */
    if (pl011_loopback_enabled(opaque)) {
        return;
    }


    MyPL011State *pl011 = MYPL011(opaque);
    
    timer_del(&pl011->rx_timeout);

    for (int i = 0; i < size; i++) {
        receive(opaque, buf[i]);
    }

    int64_t next_expire = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + RX_TIMEOUT_MS;
    timer_mod(&pl011->rx_timeout, next_expire);
    updateINTR(opaque);
}

/**
 * @brief Summary
 * @details Description
 * @param[in,out] opaque Description
 * @param[in] event Description
 */
static void pl011_event(void *opaque, QEMUChrEvent event) {
    trace_mypl011_call(__func__);
    trace_mypl011_event(event);
    MyPL011State *pl011 = (MyPL011State *)opaque;
    /// TODO: handle serial break 
    if (event == CHR_EVENT_BREAK && !pl011_loopback_enabled(pl011)) {
        receive(pl011, UARTDR_BE);
    }
}


/*********************
 *  Loopback         *
 *********************/
static void pl011_loopback_tx(MyPL011State *pl011, uint32_t value) {

    trace_mypl011_call(__func__);
    
    if (!pl011_loopback_enabled(pl011)) {
        return;
    }

    /*
     * Caveat:
     *
     * In real hardware, TX loopback happens at the serial-bit level
     * and then reassembled by the RX logics back into bytes and placed
     * into the RX fifo. That is, loopback happens after TX fifo.
     *
     * Because the real hardware TX fifo is time-drained at the frame
     * rate governed by the configured serial format, some loopback
     * bytes in TX fifo may still be able to get into the RX fifo
     * that could be full at times while being drained at software
     * pace.
     *
     * In such scenario, the RX draining pace is the major factor
     * deciding which loopback bytes get into the RX fifo, unless
     * hardware flow-control is enabled.
     *
     * For simplicity, the above described is not emulated.
     */
    receive(pl011, value);
}

__attribute__((unused))
static void pl011_loopback_break(MyPL011State *pl011, int brk_enable) {
    trace_mypl011_call(__func__);
    
    if (brk_enable) {
        pl011_loopback_tx(pl011, UARTDR_BE);
    }
}




/***************************
 * Read Registers          *
 ***************************/

typedef uint64_t (*read_reg)(MyPL011State*);

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
    if (fifo32_num_used(&s->rxfifo) >= fifo_threshold(s, RXFIFO)) {
        s->interrupts |= 0x10;
    } else {
        s->interrupts &= 0x17ef;
    }

    if (fifo32_is_empty(&s->rxfifo)) {
        timer_del(&s->rx_timeout);
        s->interrupts &= 0x7bf;
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
    return 0x14;
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


read_reg read_funcs[] = {
    [UARTDR_OFFSET] = readUARTDR,
    [UARTRSR_ECR_OFFSET] = readUARTRSR_UARTECR,
    [UARTFR_OFFSET] = readUARTFR,
    [UARTIBRD_OFFSET] = readUARTIBRD,
    [UARTFBRD_OFFSET] = readUARTFBRD,
    [UARTLCR_H_OFFSET] = readUARTLCR_H,
    [UARTCR_OFFSET] = readUARTCR,
    [UARTIFLS_OFFSET] = readUARTIFLS,
    [UARTIMSC_OFFSET] = readUARTIMSC,
    [UARTRIS_OFFSET] = readUARTRIS,
    [UARTMIS_OFFSET] = readUARTMIS,
    [UARTPeriphID0_OFFSET] = readUARTPeriphID0,
    [UARTPeriphID1_OFFSET] = readUARTPeriphID1,
    [UARTPeriphID2_OFFSET] = readUARTPeriphID2,
    [UARTPeriphID3_OFFSET] = readUARTPeriphID3,
    [UARTCellID0_OFFSET] = readUARTCellID0,
    [UARTCellID1_OFFSET] = readUARTCellID1,
    [UARTCellID2_OFFSET] = readUARTCellID2,
    [UARTCellID3_OFFSET] = readUARTCellID3,
};

/***************************
 * Write Registers         *
 ***************************/
typedef void (*write_reg)(MyPL011State*, uint64_t value);


static void writeUARTCR(PL011State *s, uint64_t value) {
    s->cr = value;
}


static void writeUARTDR(PL011State *s, uint64_t value) {
    assert(!(fifo_full(s, TXFIFO)));
    fifo32_push(&s->txfifo, value);
    if (((s->interrupts & 0x0) == 0x0 && fifo32_num_used(&s->txfifo) <= fifo_threshold(s, TXFIFO))) {

    } else {
        s->interrupts &= 0xffdf;
    }

    // always transmit immediately on writeUARTDR 
    transmit(s);

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
    if (fifo32_num_used(&s->rxfifo) >= fifo_threshold(s, RXFIFO)) {
        s->interrupts |= 0x10;
    } else {
        s->interrupts &= 0x6fcf;
    }

    if ((s->txThresholdEverCrossed == 0x1 && fifo32_num_used(&s->txfifo) <= fifo_threshold(s, TXFIFO))) {
        s->interrupts |= 0x20;
    } else {
        s->interrupts &= 0x7df;
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


write_reg write_funcs[] = {
    [UARTDR_OFFSET] = writeUARTDR,
    [UARTRSR_ECR_OFFSET] = writeUARTRSR_UARTECR,
    [UARTIBRD_OFFSET] = writeUARTIBRD,
    [UARTFBRD_OFFSET] = writeUARTFBRD,
    [UARTLCR_H_OFFSET] = writeUARTLCR_H,
    [UARTCR_OFFSET] = writeUARTCR,
    [UARTIFLS_OFFSET] = writeUARTIFLS,
    [UARTIMSC_OFFSET] = writeUARTIMSC,
    [UARTICR_OFFSET] = writeUARTICR,
};



/***************************
 * MemoryRegion Operations *
 ***************************/
static uint64_t mypl011_read(void* opaque, hwaddr offset, unsigned size) {
    MyPL011State *pl011 = (MyPL011State *)opaque;
    uint64_t retval = (uint64_t)read_funcs[offset](pl011);
    trace_mypl011_read(offset, retval, pl011_regname(offset));
    updateINTR(pl011);
    return retval;
}

static void mypl011_write(void* opaque, hwaddr offset, uint64_t value, unsigned size) {
    trace_mypl011_write(offset, value, pl011_regname(offset));
    MyPL011State *pl011 = (MyPL011State *)opaque;
    write_funcs[offset](pl011, value);
    updateINTR(pl011);
}

/**
 * @var mypl011_mem_ops
 * @brief Register Callbacks with QEMU for read/write to PL011
 */
static const MemoryRegionOps mypl011_mem_ops = {
    .read = mypl011_read,
    .write = mypl011_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.max_access_size = 4,
    .impl.min_access_size = 4
};



/***************************
 * Lifecycle Callbacks  *
 ***************************/
static void mypl011_reset(DeviceState *dev);
static void mypl011_receive_timeout_handler(void *opaque) {
    MyPL011State *pl011 = MYPL011(opaque);
    trace_mypl011_rt_callback(fifo32_num_used(&pl011->rxfifo));
    if (!fifo32_is_empty(&pl011->rxfifo)) {
        pl011->interrupts |= 0x40;
    }
    updateINTR(pl011);
}


/**
 * @brief Summary
 * @details Description
 * @param[in,out] dev Description
 * @param[out] errp Description
 */
static void mypl011_realize(DeviceState *dev, Error **errp) {
    trace_mypl011_call(__func__);
    
    MyPL011State *pl011 = MYPL011(dev);
    fifo32_create(&pl011->rxfifo, MYPL011_FIFO_DEPTH);
    fifo32_create(&pl011->txfifo, MYPL011_FIFO_DEPTH);
    mypl011_reset(dev);

    timer_init_ms(&pl011->rx_timeout,
                  QEMU_CLOCK_VIRTUAL,
                  mypl011_receive_timeout_handler,
                  dev);
    
    qemu_chr_fe_set_handlers(
        &pl011->chr,
        pl011_can_receive,
        pl011_receive,
        pl011_event,
        NULL,
        pl011,
        NULL,
        true);
}


/**
 * @brief Reset DeviceState
 * @details Emulate resetting the device
 * @param[in,out] DeviceState of PL011
 */
static void mypl011_reset(DeviceState *dev) {
    trace_mypl011_call(__func__);
    MyPL011State *pl011 = MYPL011(dev);

    /* Reset Registers */
    pl011->rsr_ecr = 0;
    pl011->overrun = 0;
    pl011->interrupts = 0;
    pl011->baudrate_int = 0;
    pl011->baudrate_frac = 0;
    pl011->isBaudrateSet = 0;
    pl011->lcr_h = 0;
    pl011->cr = 0x300;
    pl011->fifolevels = 0x12;
    pl011->interruptmasks = 0;
    pl011->txThresholdEverCrossed = 0;
    
    /* Reset RX FIFO */
    fifo32_reset(&pl011->rxfifo);
    fifo32_reset(&pl011->txfifo);
    

    /* Reset IRQs */
    qemu_set_irq(pl011->UARTINTR, false);
    qemu_set_irq(pl011->UARTRXINTR, false);
    qemu_set_irq(pl011->UARTTXINTR, false);
    qemu_set_irq(pl011->UARTRTINTR, false);
    qemu_set_irq(pl011->UARTMSINTR, false);
    qemu_set_irq(pl011->UARTEINTR, false);

    /* cancel rx_timeout timer */
    timer_del(&pl011->rx_timeout);
}

static void pl011_clock_update(void *opaque, ClockEvent event) {
    trace_mypl011_call(__func__);
    
    MyPL011State *pl011 = MYPL011(opaque);
    pl011_trace_baudrate_change(pl011);
}

/**
 * @brief DeviceState Init
 * @details QEMU required
 * @param[in,out] obj Object instance, cast required
 */
static void mypl011_init(Object *obj) {

    trace_mypl011_init();
    
    /* Cast to Device */
    MyPL011State *pl011 = MYPL011(obj);
    SysBusDevice *sysbusdev = SYS_BUS_DEVICE(obj);

    /* QEMU: Get MMIO memory region for PL011 */
    memory_region_init_io(&pl011->iomem, obj, &mypl011_mem_ops, pl011, TYPE_MYPL011, MyPL011_REG_SIZE);

    /* initialize mmio region in DeviceState */
    sysbus_init_mmio(sysbusdev, &pl011->iomem);

    /* QEMU: Initialize Interrupt Lines for PL011 */
    sysbus_init_irq(sysbusdev, &pl011->UARTINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTRXINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTTXINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTRTINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTMSINTR);
    sysbus_init_irq(sysbusdev, &pl011->UARTEINTR);

    /* Initialize clock into PL011  */
    pl011->clk = qdev_init_clock_in(DEVICE(obj), "clk", pl011_clock_update, pl011, ClockUpdate);
    
}

/**
 * @brief Instantiate a DeviceState for PL011
 * @details Allocate and register DeviceState, irq lines, and the character backend
 * @param[in] addr base address for device
 * @param[in] irq ??
 * @param[in] chr Character backend
 * @return instantiated DeviceState
 */
DeviceState* mypl011_create(hwaddr addr, qemu_irq irq, Chardev *chr) {
    DeviceState *dev;
    SysBusDevice *sysbusdev;
    
    /* Allocate a DeviceState on the heap */
    dev = qdev_new(TYPE_MYPL011);

    /* Cast to a sysbus device */
    sysbusdev = SYS_BUS_DEVICE(dev);

    /*  ?? Set chr property */
    qdev_prop_set_chr(dev, "chardev", chr);

    /*  realize the device */
    sysbus_realize_and_unref(sysbusdev, &error_fatal);

    /* set address of mmio region in SysBusDevice */
    sysbus_mmio_map(sysbusdev, 0, addr);

    /* connect to irq line */
    sysbus_connect_irq(sysbusdev, 0, irq);

    return dev;
}

/*********************
 * VMState           *
 *********************/

static bool pl011_clock_needed(void *opaque) {
    trace_mypl011_call(__func__);
    
    MyPL011State *pl011 = MYPL011(opaque);

    return pl011->migrate_clk;
}

static const VMStateDescription vmstate_pl011_clock = {
    .name = "pl011/clock",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = pl011_clock_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, MyPL011State),
        VMSTATE_END_OF_LIST()
    }
};

static int pl011_post_load(void *opaque, int version_id) {
    trace_mypl011_call(__func__);
    
    /* MyPL011State* s = MYPL011(opaque); */

    /* /\* Sanity-check input state *\/ */
    /* if (s->read_pos >= ARRAY_SIZE(s->read_fifo) || */
    /*     s->read_count > ARRAY_SIZE(s->read_fifo)) { */
    /*     return -1; */
    /* } */

    /* if (!pl011_is_fifo_enabled(s) && s->read_count > 0 && s->read_pos > 0) { */
    /*     /\* */
    /*      * Older versions of PL011 didn't ensure that the single */
    /*      * character in the FIFO in FIFO-disabled mode is in */
    /*      * element 0 of the array; convert to follow the current */
    /*      * code's assumptions. */
    /*      *\/ */
    /*     s->read_fifo[0] = s->read_fifo[s->read_pos]; */
    /*     s->read_pos = 0; */
    /* } */

    return 0;
}

static const VMStateDescription vmstate_mypl011 = {
    .name = "pl011",
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = pl011_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(rsr_ecr, MyPL011State),
        VMSTATE_UINT16(overrun, MyPL011State),
        VMSTATE_UINT16(interrupts, MyPL011State),
        VMSTATE_UINT16(baudrate_int, MyPL011State),
        VMSTATE_UINT16(baudrate_frac, MyPL011State),
        VMSTATE_UINT16(isBaudrateSet, MyPL011State),
        VMSTATE_UINT16(lcr_h, MyPL011State),
        VMSTATE_UINT16(cr, MyPL011State),
        VMSTATE_UINT16(fifolevels, MyPL011State),
        VMSTATE_UINT16(interruptmasks, MyPL011State),
        VMSTATE_UINT16(txThresholdEverCrossed, MyPL011State),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_pl011_clock,
        NULL
    }
};



/**
 * @var mypl011_properties
 * @brief Properties of PL011 DeviceClass
 */
static Property mypl011_properties[] = {
    DEFINE_PROP_CHR("chardev", MyPL011State, chr),
    DEFINE_PROP_BOOL("migrate-clk", MyPL011State, migrate_clk, true),
    DEFINE_PROP_END_OF_LIST(),
};

/**
 * @brief DeviceClass Init
 * @details QEMU Required
 * @param[in,out] klass ObjectClass
 * @param[out] class_data N/A
 */
static void mypl011_class_init(ObjectClass *klass, void *class_data) {
    trace_mypl011_call(__func__);
    
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = mypl011_realize;
    dc->reset = mypl011_reset;
    dc->vmsd = &vmstate_mypl011;
    device_class_set_props(dc, mypl011_properties);
}

/**
 * @var my_device_info
 * @brief TypeInfo, QEMU required
 */
static const TypeInfo mypl011_info = {
    .name = TYPE_MYPL011,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MyPL011State),
    .instance_init = mypl011_init,
    .class_init = mypl011_class_init,
};

static void pl011_register_types(void) {
    type_register_static(&mypl011_info);
}

type_init(pl011_register_types);
