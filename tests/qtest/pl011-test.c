/*
 * QTest testcase for PL011
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/registerfields.h"

#define PL011_BASE_ADDR 0x3f201000
#define CLK 48000000

/* See include/hw/char/pl011.h for definitions */
REG32(UARTDR, 0x000)
    FIELD(UARTDR, OE, 11, 1)
    FIELD(UARTDR, BE, 10, 1)
    FIELD(UARTDR, PE, 9, 1)
    FIELD(UARTDR, FE, 8, 1)
    FIELD(UARTDR, DATA, 0, 8)

REG32(UARTRSRECR, 0x004)
    FIELD(UARTRSRECR, OE, 3, 1)
    FIELD(UARTRSRECR, BE, 2, 1)
    FIELD(UARTRSRECR, PE, 1, 1)
    FIELD(UARTRSRECR, FE, 0, 1)
    FIELD(UARTRSRECR, CLEAR_ERRORS, 7, 1)

REG32(UARTFR, 0x018)
    FIELD(UARTFR, RI, 8, 1)
    FIELD(UARTFR, TXFE, 7, 1)
    FIELD(UARTFR, RXFF, 6, 1)
    FIELD(UARTFR, TXFF, 5, 1)
    FIELD(UARTFR, RXFE, 4, 1)
    FIELD(UARTFR, BUSY, 3, 1)
    FIELD(UARTFR, DCD, 2, 1)
    FIELD(UARTFR, DSR, 1, 1)
    FIELD(UARTFR, CTS, 0, 1)

REG32(UARTILPR, 0x020)

REG32(UARTIBRD, 0x024)
    FIELD(UARTIBRD, BAUD_DIVINT, 0, 16)

REG32(UARTFBRD, 0x028)
    FIELD(UARTFBRD, BAUD_DIVFRAC, 0, 6)

REG32(UARTLCR_H, 0x02C)
    FIELD(UARTLCR_H, SPS, 7, 1)
    FIELD(UARTLCR_H, WLEN, 5, 2)
    FIELD(UARTLCR_H, FEN, 4, 1)
    FIELD(UARTLCR_H, STP2, 3, 1)
    FIELD(UARTLCR_H, EPS, 2, 1)
    FIELD(UARTLCR_H, PEN, 1, 1)
    FIELD(UARTLCR_H, BRK, 0, 1)

REG32(UARTCR, 0x030)
    FIELD(UARTCR, CTSEN, 15, 1)
    FIELD(UARTCR, RTSEN, 14, 1)
    FIELD(UARTCR, OUT2, 13, 1)
    FIELD(UARTCR, OUT1, 12, 1)
    FIELD(UARTCR, RTS, 11, 1)
    FIELD(UARTCR, DTR, 10, 1)
    FIELD(UARTCR, RXE, 9, 1)
    FIELD(UARTCR, TXE, 8, 1)
    FIELD(UARTCR, LBE, 7, 1)
    FIELD(UARTCR, RESERVED, 3, 4)
    FIELD(UARTCR, SIRLP, 2, 1)
    FIELD(UARTCR, SIREN, 1, 1)
    FIELD(UARTCR, UARTEN, 0, 1)


REG32(UARTIFLS, 0x034)
    FIELD(UARTIFLS, RXIFLSEL, 3, 3)
    FIELD(UARTIFLS, TXIFLSEL, 0, 3)

REG32(UARTIMSC, 0x038)
    FIELD(UARTIMSC, OEIM, 10, 1)
    FIELD(UARTIMSC, BEIM, 9, 1)
    FIELD(UARTIMSC, PEIM, 8, 1)
    FIELD(UARTIMSC, FEIM, 7, 1)
    FIELD(UARTIMSC, RTIM, 6, 1)
    FIELD(UARTIMSC, TXIM, 5, 1)
    FIELD(UARTIMSC, RXIM, 4, 1)
    FIELD(UARTIMSC, DSRMIM, 3, 1)
    FIELD(UARTIMSC, DCDMIM, 2, 1)
    FIELD(UARTIMSC, CTSMIM, 1, 1)
    FIELD(UARTIMSC, RIMIM, 0, 1)

REG32(UARTRIS, 0x03C)
    FIELD(UARTRIS, OERIS, 10, 1)
    FIELD(UARTRIS, BERIS, 9, 1)
    FIELD(UARTRIS, PERIS, 8, 1)
    FIELD(UARTRIS, FERIS, 7, 1)
    FIELD(UARTRIS, RTRIS, 6, 1)
    FIELD(UARTRIS, TXRIS, 5, 1)
    FIELD(UARTRIS, RXRIS, 4, 1)
    FIELD(UARTRIS, DSRRMIS, 3, 1)
    FIELD(UARTRIS, DCDRMIS, 2, 1)
    FIELD(UARTRIS, CTSRMIS, 1, 1)
    FIELD(UARTRIS, RIRMIS, 0, 1)

REG32(UARTMIS, 0x040)
    FIELD(UARTMIS, OEMIS, 10, 1)
    FIELD(UARTMIS, BEMIS, 9, 1)
    FIELD(UARTMIS, PEMIS, 8, 1)
    FIELD(UARTMIS, FEMIS, 7, 1)
    FIELD(UARTMIS, RTMIS, 6, 1)
    FIELD(UARTMIS, TXMIS, 5, 1)
    FIELD(UARTMIS, RXMIS, 4, 1)
    FIELD(UARTMIS, DSRMMIS, 3, 1)
    FIELD(UARTMIS, DCDMMIS, 2, 1)
    FIELD(UARTMIS, CTSMMIS, 1, 1)
    FIELD(UARTMIS, RIMMIS, 0, 1)

REG32(UARTICR, 0x044)
    FIELD(UARTICR, OEIC, 10, 1)
    FIELD(UARTICR, BEIC, 9, 1)
    FIELD(UARTICR, PEIC, 8, 1)
    FIELD(UARTICR, FEIC, 7, 1)
    FIELD(UARTICR, RTIC, 6, 1)
    FIELD(UARTICR, TXIC, 5, 1)
    FIELD(UARTICR, RXIC, 4, 1)
    FIELD(UARTICR, DSRMIC, 3, 1)
    FIELD(UARTICR, DCDMIC, 2, 1)
    FIELD(UARTICR, CTSMIC, 1, 1)
    FIELD(UARTICR, RIMIC, 0, 1)

REG32(UARTDMACR, 0x048)
    FIELD(UARTDMACR, DMAONERR, 2, 1)
    FIELD(UARTDMACR, TXDMAE, 1, 1)
    FIELD(UARTDMACR, RXDMAE, 0, 1)

REG8(UARTPeriphID0, 0xFE0)
REG8(UARTPeriphID1, 0xFE4)
REG8(UARTPeriphID2, 0xFE8)
REG8(UARTPeriphID3, 0xFEC)
REG8(UARTPCellID0, 0xFF0)
REG8(UARTPCellID1, 0xFF4)
REG8(UARTPCellID2, 0xFF8)
REG8(UARTPCellID3, 0xFFC)



#define REG_PL011(r) (PL011_BASE_ADDR + r)


/*
 * Fixture
 */
typedef struct {
    int sock_fd;
    QTestState *qts;
} Fixture;

static void fixture_set_up(Fixture *fixture,
                           gconstpointer user_data)
{
    int sock_fd = -1;
    QTestState *qts = qtest_init_with_serial("-M raspi3b", &sock_fd);
    
    fixture->sock_fd = sock_fd;
    fixture->qts = qts;
}

static void fixture_tear_down(Fixture *fixture,
                              gconstpointer user_data)
{
    close(fixture->sock_fd);
    qtest_quit(fixture->qts);
}



/*
 * Helpers
 */
#define TEST(case) static void case(Fixture *fixture, gconstpointer user_data)

static bool usart_wait_for_flag(QTestState *qts, uint32_t event_addr,
                                uint32_t flag)
{
    while (true) {
        if ((qtest_readl(qts, event_addr) & flag)) {
            return true;
        }
        g_usleep(1000);
    }

    return false;
}

static void init_uart(QTestState *qts)
{
    // enable trasmit and receive
    uint32_t cr = (1 << R_UARTCR_TXE_SHIFT) | (1 << R_UARTCR_RXE_SHIFT);
    qtest_writel(qts, REG_PL011(A_UARTCR), cr);
    
    // set the bit format to 8 bits
    uint32_t lcr_h = (0b11 << R_UARTLCR_H_WLEN_SHIFT);
    // enable FIFOs
    lcr_h |= (1 << R_UARTLCR_H_FEN_SHIFT);

    qtest_writel(qts, REG_PL011(A_UARTLCR_H), lcr_h);
    
    // enable UART
    cr = (1 << R_UARTCR_UARTEN_SHIFT);
    qtest_writel(qts, REG_PL011(A_UARTCR), cr);
}

/*
 *
 * -- UARTDR --
 *
 */
// Write
TEST(UARTDR_W_TXE_SC) {}
TEST(UARTDR_W_TXE_MC) {}
TEST(UARTDR_W_TXE_SC_OVERRUN_FIFO) {}
TEST(UARTDR_W_TXDisabled) {}
TEST(UARTDR_W_UARTDisabled) {}
// Read
TEST(UARTDR_R_RXE_FIFO_EMPTY) {}
TEST(UARTDR_R_RXE_FIFO_NOT_EMPTY_SC) {}
TEST(UARTDR_R_RXE_FIFO_NOT_EMPTY_MC) {}

#define write_register(r, v) qtest_writel(fixture->qts, REG_PL011(r), v)
#define read_register(r) qtest_readl(fixture->qts, REG_PL011(r))

#define io_sock(op, buf, len) g_assert_true(op(fixture->sock_fd, buf, len, 0) == len)
#define read_sock_into(buf, len)  io_sock(recv, buf, len)
#define write_sock_from(buf, len) io_sock(send, buf, len)

// Sample test
static void test_write_read(Fixture *fixture,
                            gconstpointer user_data)
{

    // init uart
    init_uart(fixture->qts);
    
    char data[2]; 

    // TX
    write_register(A_UARTDR, 'a');
    read_sock_into(data, 1);
    g_assert_true(data[0] == 'a');

    // RX
    write_sock_from("b", 1);
    usart_wait_for_flag(fixture->qts, REG_PL011(A_UARTRIS), R_UARTRIS_RXRIS_MASK);
    /* g_debug("# This is a debug message: %u", read_register(A_UARTDR)); */
    
    g_assert_true(read_register(A_UARTDR) == 'b');
}


#define add_test(name, test) qtest_add(name, Fixture, NULL, fixture_set_up, test, fixture_tear_down)

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    add_test("pl011/testing", test_write_read);
    /*
     * UARTDR
     */
    // read
    add_test("pl011/UARTDR/read/rx-enabled/fifo-empty/single-charater", UARTDR_R_RXE_FIFO_EMPTY);
    add_test("pl011/UARTDR/read/rx-enabled/fifo-not-empty/single-character", UARTDR_R_RXE_FIFO_NOT_EMPTY_SC);
    add_test("pl011/UARTDR/read/rx-enabled/fifo-not-empty/multiple-character", UARTDR_R_RXE_FIFO_NOT_EMPTY_MC);
    
    // write
    add_test("pl011/UARTDR/write/tx-enabled/single-character", UARTDR_W_TXE_SC);
    add_test("pl011/UARTDR/write/tx-enabled/multiple-characters", UARTDR_W_TXE_MC);
    add_test("pl011/UARTDR/write/tx-enabled/fifo-full/overrun", UARTDR_W_TXE_SC_OVERRUN_FIFO);
    add_test("pl011/UARTDR/write/tx-disabled", UARTDR_W_TXDisabled);
    add_test("pl011/UARTDR/write/uart-disabled", UARTDR_W_UARTDisabled);
    
    ret = g_test_run();

    return ret;
}

