#include "hw/char/pl011_2.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "hw/char/pl011.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/osdep.h"
#include "qemu/typedefs.h"
#include "trace.h"

#define MyPL011_REG_SIZE 0x1000

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
    dev = qdev_new(TYPE_PL011);

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


static inline bool pl011_is_fifo_enabled(MyPL011State *pl011)
{
    return (pl011->UARTLCR_H & UARTLCR_H_FEN) != 0;
}

static inline unsigned pl011_get_fifo_depth(MyPL011State *pl011)
{
    /* Note: FIFO depth is expected to be power-of-2 */
    return pl011_is_fifo_enabled(pl011) ? PL011_FIFO_DEPTH : 1;
}

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

    pl011_put_fifo(opaque, *buf);
}

/**
 * @brief Summary
 * @details Description
 * @param[in,out] opaque Description
 * @param[in] event Description
 */
static void pl011_event(void *opaque, QEMUChrEvent event)
{
    if (event == CHR_EVENT_BREAK && !pl011_loopback_enabled(opaque)) {
        pl011_put_fifo(opaque, DR_BE);
    }
}


/**
 * @brief Summary
 * @details Description
 * @param[in,out] dev Description
 * @param[out] errp Description
 */
static void pl011_realize(DeviceState *dev, Error **errp) {
    MyPL011State *pl011 = MyPL011(dev);

    qemu_chr_fe_set_handlers(&pl011->chr,
                             pl011_can_receive,
                             pl011_receive,
                             pl011_event,
                             NULL,
                             pl011,
                             NULL,
                             true);
}


static void mypl011_reset(DeviceState *device) {
    MyPL011State *pl011 = MyPL011(dev);
    pl011->UARTDR = UARTDR_RESET_VAL;
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
    pl011->UART
}

static uint64_t mypl011_read(void* opaque, hwaddr offset, unsigned size) {
    /// TODO
}

static void mypl011_write(void* opaque, hwaddr offset, uint64_t data, unsigned size) {
    /// TODO
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

/**
 * @brief DeviceState Init
 * @details QEMU required
 * @param[in,out] obj Object instance, cast required
 */
static void mypl011_init(Object *obj) {

    /* Cast to Device */
    MyPL011State *dev = MyPL011(obj);
    SysBusDevice *sysbusdev = SYS_BUS_DEVICE(obj);

    /* QEMU: Get MMIO memory region for PL011 */
    memory_region_init_io(&dev->iomem, obj, &mypl011_mem_ops, dev, TYPE_MYPL011, MyPL011_REG_SIZE);

    /* initialize mmio region in DeviceState */
    sysbus_init_mmio(sysbusdev, &dev->iomem);

    /* QEMU: Initialize Interrupt Lines for PL011 */
    for (unsigned i = 0; i < ARRAY_SIZE(dev->irq); i++) {
        sysbus_init_irq(sysbusdev, &dev->irq[i]);
    }

    /* Initialize clock into PL011  */
    dev->clk = qdev_init_clock_in(DEVICE(obj), "clk", pl011_clock_update, dev, ClockUpdate);

    /* dev->id = pl011_id_arm; */
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
void mypl011_class_init(ObjectClass *klass, void *class_data) {
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
static const TypeInfo my_device_info = {
    .name = TYPE_MYPL011,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MyPL011State),
    .instance_init = mypl011_init,
    .class_init = mypl011_class_init,
};

static void pl011_register_types(void) {
    type_register_static(&pl011_arm_info);
}

type_init(pl011_register_types);
