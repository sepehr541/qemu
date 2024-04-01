#include "qemu/osdep.h"
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
#include "trace.h"

#define MyPL011_REG_SIZE 0x1000

/*********************
 * Constant State    *
 *********************/

/* Peripheral Identification Registers */ 
static unsigned char UARTPeriphID[] = {0x11, 0x10, 0x14, 0x00};
/* PrimeCell Identification Registers */ 
static unsigned char UARTCellID[] = {0x0d, 0xf0, 0x05, 0xb1};

/*********************
 *  Flags            *
 *********************/

static inline bool pl011_loopback_enabled(MyPL011State *pl011)
{
    return !!(pl011->UARTCR & UARTCR_LBE);
}

static inline bool pl011_is_fifo_enabled(MyPL011State *pl011)
{
    return (pl011->UARTLCR_H & UARTLCR_H_FEN) != 0;
}


__attribute__((unused))
static unsigned int pl011_get_baudrate(const MyPL011State *pl011)
{
    uint64_t clk;

    if (pl011->UARTIBRD == 0) {
        return 0;
    }

    clk = clock_get_hz(pl011->clk);
    return (clk / ((pl011->UARTIBRD << 6) + pl011->UARTFBRD)) << 2;
}

__attribute__((unused))
static void pl011_trace_baudrate_change(const MyPL011State *pl011)
{
    trace_pl011_baudrate_change(pl011_get_baudrate(pl011),
                                clock_get_hz(pl011->clk),
                                pl011->UARTIBRD,
                                pl011->UARTFBRD);
}

/*********************
 *  Interrupts       *
 *********************/

/* UARTINTR (combined masked) */
/*
  asserted if *any* of the individual interrupts are asserted and enabled.
*/
static inline void assert_INTR(MyPL011State *pl011) {  
    qemu_set_irq(pl011->UARTINTR, pl011->UARTMIS != 0);
}

static void set_interrupt(MyPL011State *pl011, uint32_t irq_mask) {
    /* set RIS */
    pl011->UARTRIS &= irq_mask;
    /* check mask */
    if (pl011->UARTIMSC & irq_mask) {
        /* set MIS */
        pl011->UARTMIS &= irq_mask;
    }
}

static void clear_interrupt(MyPL011State *pl011, uint32_t irq_mask) {
    /* clear RIS */
    pl011->UARTRIS &= ~irq_mask;
    /* check mask */
    if (pl011->UARTIMSC & irq_mask) {
        /* clear MIS */
        pl011->UARTMIS &= ~irq_mask;        
    }
}

static void trigger_interrupt(qemu_irq irq_line, int level) {
    qemu_set_irq(irq_line, level);
}

/* UARTRXINTR */
/*
  - If the FIFOs are enabled and the receive FIFO reaches the programmed trigger
  level. When this happens, the receive interrupt is asserted HIGH. 
  - If the FIFOs are disabled (have a depth of one location) and data is received
  thereby filling the location, the receive interrupt is asserted HIGH. 
*/
static void set_RXINTR(MyPL011State *pl011) {
    set_interrupt(pl011, RXINTR);
    trigger_interrupt(pl011->UARTRXINTR, true);
}

/*
  - The receive interrupt is cleared by reading data from the receive FIFO until it becomes less than the trigger level,
  or by clearing the interrupt.
  - The receive interrupt is cleared by performing a single read of the receive FIFO, or by clearing the interrupt.
*/
static void clear_RXINTR(MyPL011State *pl011) {
    clear_interrupt(pl011, RXINTR);
    trigger_interrupt(pl011->UARTRXINTR, false);
}

/* UARTTXINTR */
/*
  The transmit interrupt changes state when one of the following events occurs:

  - If the FIFOs are enabled and the transmit FIFO is equal to or lower than the
  programmed trigger level then the transmit interrupt is asserted HIGH. The
  transmit interrupt is cleared by writing data to the transmit FIFO until it becomes
  greater than the trigger level, or by clearing the interrupt.

  - If the FIFOs are disabled (have a depth of one location) and there is no data
  present in the transmitters single location, the transmit interrupt is asserted HIGH.
  It is cleared by performing a single write to the transmit FIFO, or by clearing the
  interrupt.
  To update the transmit FIFO you must:

  - Write data to the transmit FIFO, either prior to enabling the UART and the
  interrupts, or after enabling the UART and interrupts.

  -- Note --
  The transmit interrupt is based on a transition through a level, rather than on the level
  itself. When the interrupt and the UART is enabled before any data is written to the
  transmit FIFO the interrupt is not set. The interrupt is only set, after written data leaves
  the single location of the transmit FIFO and it becomes empty.
*/
static void set_TXINTR(MyPL011State *pl011) {
    set_interrupt(pl011, TXINTR);
}

__attribute__((unused))
static void clear_TXINTR(MyPL011State *pl011) {
    clear_interrupt(pl011, TXINTR);
}

/* UARTRTINTR */
__attribute__((unused))
static void set_RTINTR(MyPL011State *pl011) {
    set_interrupt(pl011, RTINTR);
}

__attribute__((unused))
static void clear_RTINTR(MyPL011State *pl011) {
    clear_interrupt(pl011, RTINTR);
}

/* UARTFEINTR */
__attribute__((unused))
static void set_EINTR(MyPL011State *pl011, uint32_t irq_mask) {
    
}

__attribute__((unused))
static void clear_EINTR(MyPL011State *pl011, uint32_t irq_mask) {

}

/* UARTMSINTR */
/* TODO: DCE Mode -- Ignored for now */
__attribute__((unused))
static void set_MSINTR(MyPL011State *pl011, uint32_t irq_mask) {
    /* set_interrupt(pl011, irq_mask); */
    /* trigger_interrupt(pl011->UARTMSINTR, true); */
}

__attribute__((unused))
static void clear_MSINTR(MyPL011State *pl011, uint32_t irq_mask) {
    clear_interrupt(pl011, irq_mask);
    /* TODO */
    if (false) {
        trigger_interrupt(NULL, false);
        assert_INTR(pl011);
    }
}




/*********************
 *  FIFO             *
 *********************/
static inline unsigned pl011_get_fifo_depth(MyPL011State *pl011)
{
    /* Note: FIFO depth is expected to be power-of-2 */
    return pl011_is_fifo_enabled(pl011) ? MYPL011_FIFO_DEPTH : 1;
}

static inline void pl011_reset_fifo(MyPL011State *pl011)
{
    pl011->read_count = 0;
    pl011->read_pos = 0;

    /* Reset FIFO flags */
    pl011->UARTFR &= ~(UARTFR_RXFF |UARTFR_TXFF);
    pl011->UARTFR |= UARTFR_RXFE | UARTFR_TXFE;
}

static void threshold_fraction(uint32_t status, uint32_t *numer, uint32_t *denom) {
    switch(status) {
    case UARTIFLS_1_8:
        *denom = 8;
        break;
    case UARTIFLS_1_4:
        *denom = 4;
        break;
    case UARTIFLS_1_2:
        *denom = 2;
        break;
    case UARTIFLS_3_4:
        *numer = 3;
        *denom = 4;
        break;
    case UARTIFLS_7_8:
        *numer = 7;
        *denom = 8;
        break;
    default:
        assert(0);
        break;
    }
}

static bool mypl011_RX_FIFO_reached_threshold(MyPL011State *pl011) {
    uint32_t numer = 1, denom = 1;
    uint32_t RXIFLSEL = (pl011->UARTIFLS & UARTIFLS_RXIFLSEL_MASK) >> UARTIFLS_RXIFLSEL_SHIFT;
    threshold_fraction(RXIFLSEL, &numer, &denom);
    return pl011->read_count >= (MYPL011_FIFO_DEPTH / denom) * numer;
}

/**
 * @brief Push char onto FIFO
 * @details Asserts that FIFO is not full
 * @param[in,out] pl011 DeviceState
 */
static void mypl011_RX_FIFO_push(MyPL011State *pl011, uint32_t val) {
    /* TODO: FIX to have the overrun bit set */
    assert(pl011->read_count < MYPL011_FIFO_DEPTH);

    /* push value onto fifo */
    uint32_t pipe_depth = pl011_get_fifo_depth(pl011);
    int slot = (pl011->read_pos + pl011->read_count) & (pipe_depth - 1);
   
    pl011->read_fifo[slot] = val;
    pl011->read_count++;

    /* Receive FIFO is not empty */
    pl011->UARTFR &= ~UARTFR_RXFE;

    /* Check if FIFO is full  */
    if (pl011->read_count == MYPL011_FIFO_DEPTH) {
        /* set Receive FIFO full */
        pl011->UARTFR |= UARTFR_RXFF;
    }

    if (mypl011_RX_FIFO_reached_threshold(pl011)) {
        set_RXINTR(pl011);
    }
}

/**
 * @brief Summary
 * @detail get value at the head of the RX FIFO
 * @param[in,out] pl011 Description
 * @return Description
 */
static uint32_t mypl011_RX_FIFO_pop(MyPL011State *pl011) {
    /* TODO: FIX to have the overrun bit set */
    assert(pl011->read_count < MYPL011_FIFO_DEPTH);

    /* pop value off FIFO */
    uint32_t val = pl011->read_fifo[pl011->read_pos];
    uint32_t pipe_depth = pl011_get_fifo_depth(pl011);
    pl011->read_pos = (pl011->read_pos + 1) % pipe_depth;
    pl011->read_count--;
    
    /* Receive FIFO is not full */
    pl011->UARTFR &= ~UARTFR_RXFF;

    /* Check if FIFO is empty  */
    if (pl011->read_count == 0) {
        /* set Receive FIFO empty */
        pl011->UARTFR |= UARTFR_RXFE;
    }

    /* clear RXINTR if less than threshold */
    if (!mypl011_RX_FIFO_reached_threshold(pl011)) {
        clear_RXINTR(pl011);
    }

    /* update UARTRSR to have the status of popped value */
    pl011->UARTRSR_ECR = val >> 8;

    return val;
}


/* static bool mypl011_TX_FIFO_reached_threshold(MyPL011State *pl011) { */
/*     uint32_t numer = 1, denom = 1; */
/*     uint32_t TXIFLSEL = (pl011->UARTIFLS & UARTIFLS_TXIFLSEL); */
/*     threshold_fraction(TXIFLSEL, &numer, &denom); */
/*     return pl011->read_count >= (MYPL011_FIFO_DEPTH / denom) * numer; */
/* } */



/**
 * @brief Can_receive predicate
 * @details 
 * @param[in,out] opaque Description
 * @return number of bytes the device can receive
 */
static int pl011_can_receive(void *opaque)
{
    MyPL011State *pl011 = (MyPL011State *)opaque;
    int r;

    /// TODO: fix to return the number of bytes it can read
    r = pl011->read_count < pl011_get_fifo_depth(pl011);
    trace_pl011_can_receive(pl011->UARTLCR_H, pl011->read_count, r);
    return r;
}

/**
 * @brief Summary
 * @details Description
 * @param[in,out] opaque Description
 * @param[in] buf Pointer to buffer to ?
 * @param[in] size Description
 */
static void pl011_receive(void *opaque, const uint8_t *buf, int size)
{
    /*
     * In loopback mode, the RX input signal is internally disconnected
     * from the entire receiving logics; thus, all inputs are ignored,
     * and BREAK detection on RX input signal is also not performed.
     */
    if (pl011_loopback_enabled(opaque)) {
        return;
    }

    mypl011_RX_FIFO_push(opaque, *buf);
}

/**
 * @brief Summary
 * @details Description
 * @param[in,out] opaque Description
 * @param[in] event Description
 */
static void pl011_event(void *opaque, QEMUChrEvent event)
{
    MyPL011State *pl011 = (MyPL011State *)opaque;
    /// TODO: handle serial break 
    if (event == CHR_EVENT_BREAK && !pl011_loopback_enabled(pl011)) {
        mypl011_RX_FIFO_push(pl011, UARTDR_BE);
    }
}



/***************************
 * Read Registers          *
 ***************************/

typedef uint64_t (*read_reg)(MyPL011State*);

static uint64_t read_UARTDR(MyPL011State *pl011) {
    uint32_t value = mypl011_RX_FIFO_pop(pl011);
    qemu_chr_fe_accept_input(&pl011->chr);
    return value;
}
static uint64_t read_UARTRSR_ECR(MyPL011State *pl011) {
    return pl011->UARTRSR_ECR;
}

static uint64_t read_UARTFR(MyPL011State *pl011) {
    return pl011->UARTFR;
}

static uint64_t read_UARTILPR(MyPL011State *pl011) {
    return pl011->UARTILPR;
}

static uint64_t read_UARTIBRD(MyPL011State *pl011) {
    return pl011->UARTIBRD;
}

static uint64_t read_UARTFBRD(MyPL011State *pl011) {
    return pl011->UARTFBRD;
}

static uint64_t read_UARTLCR_H(MyPL011State *pl011) {
    return pl011->UARTLCR_H;
}

static uint64_t read_UARTCR(MyPL011State *pl011) {
    return pl011->UARTCR;
}

static uint64_t read_UARTIFLS(MyPL011State *pl011) {
    return pl011->UARTIFLS;
}

static uint64_t read_UARTIMSC(MyPL011State *pl011) {
    return pl011->UARTIMSC;
}

static uint64_t read_UARTRIS(MyPL011State *pl011) {
    return pl011->UARTRIS;
}

static uint64_t read_UARTMIS(MyPL011State *pl011) {
    return pl011->UARTMIS;
}

static uint64_t read_UARTICR(MyPL011State *pl011) {
    assert(0);
    return 0;
}

static uint64_t read_UARTDMACR(MyPL011State *pl011) {
    return pl011->UARTDMACR;
}

static uint64_t read_UARTPeriphID0(MyPL011State *pl011) { return UARTPeriphID[0]; }
static uint64_t read_UARTPeriphID1(MyPL011State *pl011) { return UARTPeriphID[1]; }
static uint64_t read_UARTPeriphID2(MyPL011State *pl011) { return UARTPeriphID[2]; }
static uint64_t read_UARTPeriphID3(MyPL011State *pl011) { return UARTPeriphID[3]; }

static uint64_t read_UARTCellID0(MyPL011State *pl011) { return UARTCellID[0]; }
static uint64_t read_UARTCellID1(MyPL011State *pl011) { return UARTCellID[1]; }
static uint64_t read_UARTCellID2(MyPL011State *pl011) { return UARTCellID[2]; }
static uint64_t read_UARTCellID3(MyPL011State *pl011) { return UARTCellID[3]; }


read_reg read_funcs[] = {
    [UARTDR_OFFSET] = read_UARTDR,
    [UARTRSR_ECR_OFFSET] = read_UARTRSR_ECR,
    [UARTFR_OFFSET] = read_UARTFR,
    [UARTILPR_OFFSET] = read_UARTILPR,
    [UARTIBRD_OFFSET] = read_UARTIBRD,
    [UARTFBRD_OFFSET] = read_UARTFBRD,
    [UARTLCR_H_OFFSET] = read_UARTLCR_H,
    [UARTCR_OFFSET] = read_UARTCR,
    [UARTIFLS_OFFSET] = read_UARTIFLS,
    [UARTIMSC_OFFSET] = read_UARTIMSC,
    [UARTRIS_OFFSET] = read_UARTRIS,
    [UARTMIS_OFFSET] = read_UARTMIS,
    [UARTICR_OFFSET] = read_UARTICR,
    [UARTDMACR_OFFSET] = read_UARTDMACR,
    [UARTPeriphID0_OFFSET] = read_UARTPeriphID0,
    [UARTPeriphID1_OFFSET] = read_UARTPeriphID1,
    [UARTPeriphID2_OFFSET] = read_UARTPeriphID2,
    [UARTPeriphID3_OFFSET] = read_UARTPeriphID3,
    [UARTCellID0_OFFSET] = read_UARTCellID0,
    [UARTCellID1_OFFSET] = read_UARTCellID1,
    [UARTCellID2_OFFSET] = read_UARTCellID2,
    [UARTCellID3_OFFSET] = read_UARTCellID3,
};

/***************************
 * Write Registers         *
 ***************************/
typedef void (*write_reg)(MyPL011State*, uint64_t value);

static void write_UARTDR(MyPL011State *pl011, uint64_t value) {
    /* ??? Check if transmitter is enabled.  */
    unsigned char ch = value;
    /* XXX this blocks entire thread. Rewrite to use
     * qemu_chr_fe_write and background I/O callbacks */
    qemu_chr_fe_write_all(&pl011->chr, &ch, 1);
    /* pl011_loopback_tx(s, ch); */
    /* s->int_level |= INT_TX; */
    set_TXINTR(pl011);
    /* ?? other INTRs ?? */
}



static void write_UARTRSR_ECR(MyPL011State *pl011, uint64_t _) {
    /* A write to the UARTECR Register clears the framing, parity, break, and overrun errors. */
    pl011->UARTRSR_ECR = 0;
}

static void write_UARTFR(MyPL011State *pl011, uint64_t value) {
    /* No-op */
}

static void write_UARTILPR(MyPL011State *pl011, uint64_t value) {
    pl011->UARTILPR = value;
}

static void write_UARTIBRD(MyPL011State *pl011, uint64_t value) {
    pl011->UARTIBRD = value;
}

static void write_UARTFBRD(MyPL011State *pl011, uint64_t value) {
    pl011->UARTFBRD = value;
}

static void write_UARTLCR_H(MyPL011State *pl011, uint64_t value) {
    /* Reset the FIFO state on FIFO enable or disable */
    if ((pl011->UARTLCR_H ^ value) & UARTLCR_H_FEN) {
        pl011_reset_fifo(pl011);
    }
    
    if ((pl011->UARTLCR_H ^ value) & UARTLCR_H_BRK) {
        int break_enable = value & UARTLCR_H_BRK;
        qemu_chr_fe_ioctl(&pl011->chr, CHR_IOCTL_SERIAL_SET_BREAK,
                          &break_enable);
        /* pl011_loopback_break(s, break_enable); */
    }
    pl011->UARTLCR_H = value;
}

static void write_UARTCR(MyPL011State *pl011, uint64_t value) {
    /* TODO: ??? Need to implement the enable bit.  */
    pl011->UARTCR = value;
    /* pl011_loopback_mdmctrl(s); */
}

static void write_UARTIFLS(MyPL011State *pl011, uint64_t value) {
    pl011->UARTIFLS = value;
}

static void write_UARTIMSC(MyPL011State *pl011, uint64_t value) {
    pl011->UARTIMSC = value;
}

static void write_UARTRIS(MyPL011State *pl011, uint64_t value) {
    /* Read-only */
}

static void write_UARTMIS(MyPL011State *pl011, uint64_t value) {
    /* Read-only */
}

static void write_UARTICR(MyPL011State *pl011, uint64_t value) {
    pl011->UARTRIS &= ~value;
    pl011->UARTMIS &= ~value;
    /* TODO: retrigger interrupts */
}

static void write_UARTDMACR(MyPL011State *pl011, uint64_t value) {
    pl011->UARTDMACR = value;
}

/* Read-only  */
static void write_UARTPeriphID0(MyPL011State *pl011, uint64_t value) {}
static void write_UARTPeriphID1(MyPL011State *pl011, uint64_t value) {}
static void write_UARTPeriphID2(MyPL011State *pl011, uint64_t value) {}
static void write_UARTPeriphID3(MyPL011State *pl011, uint64_t value) {}
static void write_UARTCellID0(MyPL011State *pl011, uint64_t value) {}
static void write_UARTCellID1(MyPL011State *pl011, uint64_t value) {}
static void write_UARTCellID2(MyPL011State *pl011, uint64_t value) {}
static void write_UARTCellID3(MyPL011State *pl011, uint64_t value) {}


write_reg write_funcs[] = {
    [UARTDR_OFFSET] = write_UARTDR,
    [UARTRSR_ECR_OFFSET] = write_UARTRSR_ECR,
    [UARTFR_OFFSET] = write_UARTFR,
    [UARTILPR_OFFSET] = write_UARTILPR,
    [UARTIBRD_OFFSET] = write_UARTIBRD,
    [UARTFBRD_OFFSET] = write_UARTFBRD,
    [UARTLCR_H_OFFSET] = write_UARTLCR_H,
    [UARTCR_OFFSET] = write_UARTCR,
    [UARTIFLS_OFFSET] = write_UARTIFLS,
    [UARTIMSC_OFFSET] = write_UARTIMSC,
    [UARTRIS_OFFSET] = write_UARTRIS,
    [UARTMIS_OFFSET] = write_UARTMIS,
    [UARTICR_OFFSET] = write_UARTICR,
    [UARTDMACR_OFFSET] = write_UARTDMACR,
    [UARTPeriphID0_OFFSET] = write_UARTPeriphID0,
    [UARTPeriphID1_OFFSET] = write_UARTPeriphID1,
    [UARTPeriphID2_OFFSET] = write_UARTPeriphID2,
    [UARTPeriphID3_OFFSET] = write_UARTPeriphID3,
    [UARTCellID0_OFFSET] = write_UARTCellID0,
    [UARTCellID1_OFFSET] = write_UARTCellID1,
    [UARTCellID2_OFFSET] = write_UARTCellID2,
    [UARTCellID3_OFFSET] = write_UARTCellID3,
};


/***************************
 * MemoryRegion Operations *
 ***************************/
static uint64_t mypl011_read(void* opaque, hwaddr offset, unsigned size) {
    MyPL011State *pl011 = (MyPL011State *)opaque;
    uint64_t retval = read_funcs[offset](pl011);
    return retval;
}

static void mypl011_write(void* opaque, hwaddr offset, uint64_t value, unsigned size) {
    MyPL011State *pl011 = (MyPL011State *)opaque;
    write_funcs[offset](pl011, value);
}

/**
 * @var mypl011_mem_ops
 * @brief Register Callbacks with QEMU for read/write to PL011
 */
static const MemoryRegionOps mypl011_mem_ops = {
    .read = mypl011_read,
    .write = mypl011_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.max_access_size = 4,
    .valid.min_access_size = 4
};



/***************************
 * Lifecycle Callbacks  *
 ***************************/

/**
 * @brief Summary
 * @details Description
 * @param[in,out] dev Description
 * @param[out] errp Description
 */
static void mypl011_realize(DeviceState *dev, Error **errp) {
    MyPL011State *pl011 = MYPL011(dev);

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
    MyPL011State *pl011 = MYPL011(dev);

    /* Reset Registers */
    pl011->UARTRSR_ECR = UARTRSR_ECR_RESET_VAL;
    pl011->UARTFR = UARTFR_RESET_VAL;
    pl011->UARTILPR = UARTILPR_RESET_VAL;
    pl011->UARTIBRD = UARTIBRD_RESET_VAL;
    pl011->UARTFBRD = UARTFBRD_RESET_VAL;
    pl011->UARTLCR_H = UARTLCR_H_RESET_VAL;
    pl011->UARTCR = UARTCR_RESET_VAL;
    pl011->UARTIFLS = UARTIFLS_RESET_VAL;
    pl011->UARTIMSC = UARTIMSC_RESET_VAL;
    pl011->UARTRIS = UARTRIS_RESET_VAL;
    pl011->UARTMIS = UARTMIS_RESET_VAL;
    pl011->UARTICR = UARTICR_RESET_VAL;
    pl011->UARTDMACR = UARTDMACR_RESET_VAL;

    /* Reset RX FIFO */
    pl011_reset_fifo(pl011);

    /* Reset IRQs */
    qemu_set_irq(pl011->UARTINTR, false);
    qemu_set_irq(pl011->UARTRXINTR, false);
    qemu_set_irq(pl011->UARTTXINTR, false);
    qemu_set_irq(pl011->UARTRTINTR, false);
    qemu_set_irq(pl011->UARTMSINTR, false);
    qemu_set_irq(pl011->UARTEINTR, false);

}



static void pl011_clock_update(void *opaque, ClockEvent event)
{
    /* TODO */
    /* PL011State *s = PL011(opaque); */
    /* pl011_trace_baudrate_change(s); */
}

/**
 * @brief DeviceState Init
 * @details QEMU required
 * @param[in,out] obj Object instance, cast required
 */
static void mypl011_init(Object *obj) {

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

    /* dev->id = pl011_id_arm; */
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
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = mypl011_realize;
    dc->reset = mypl011_reset;
    /* dc->vmsd = &mypl011_vmstate; */

    /* Device Composition and Backlink through setting the properties */
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
