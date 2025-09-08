#if 1

/* main.c
   MSP430F6779 + PN532 (SPI) — continuous tag polling
   - SPI: P4.5 MOSI, P4.4 MISO, P4.6 SCK, P4.7 CS
   - IRQ: P1.2 (active-low)
   - RST: P6.1 (active-low)
   - UART: UCA1 P3.4 RX, P3.5 TX @ 9600 @ 16 MHz
   - Correct InListPassiveTarget parsing (UID extraction fixed)
   - Prints UID as AA:BB:CC:DD and only when it changes
*/

#include <msp430f6779.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------- Pin map ---------- */
#define SPI_SOMI_PIN    BIT4    // P4.4 MISO
#define SPI_SIMO_PIN    BIT5    // P4.5 MOSI
#define SPI_CLK_PIN     BIT6    // P4.6 SCK
#define SPI_CS_PIN      BIT7    // P4.7 CS

#define PN532_RST_PIN   BIT1    // P6.1 RST
#define PN532_IRQ_PIN   BIT2    // P1.2 IRQ

/* PN532 SPI opcodes */
#define PN532_SPI_SR    0x02
#define PN532_SPI_DW    0x01
#define PN532_SPI_DR    0x03

/* ---------- UART helpers ---------- */
static void uart_init(void) {
    // P3.4 RX, P3.5 TX
    P3SEL0 |= BIT4 | BIT5;
    P3DIR  |= BIT5;  // TX
    P3DIR  &= ~BIT4; // RX

    UCA1CTL1 |= UCSWRST;
    UCA1CTL1 |= UCSSEL_2;       // SMCLK
    UCA1BRW   = 109;            // 9600 @ 16 MHz
    UCA1MCTLW = 0;
    UCA1CTL1 &= ~UCSWRST;
}
static void uart_putc(char c) {
    while (!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = (uint8_t)c;
}
static void uart_print(const char *s) { while (*s) uart_putc(*s++); }
static void uart_print_hex(uint8_t b) {
    const char hex[] = "0123456789ABCDEF";
    uart_putc(hex[(b >> 4) & 0xF]);
    uart_putc(hex[b & 0xF]);
}

/* ---------- Delay helpers ---------- */
static void delay_us(unsigned us) {
    while (us--) __delay_cycles(16);  // ~1µs @ 16 MHz
}
static void delay_ms(unsigned ms) {
    while (ms--) __delay_cycles(16000); // ~1ms @ 16 MHz
}

/* ---------- SPI bit-bang (Mode 0, LSB-first) ---------- */
static inline void cs_low(void)  { P4OUT &= ~SPI_CS_PIN; }
static inline void cs_high(void) { P4OUT |=  SPI_CS_PIN; }
static inline void sck_low(void) { P4OUT &= ~SPI_CLK_PIN; }
static inline void sck_high(void){ P4OUT |=  SPI_CLK_PIN; }
static inline void mosi_low(void){ P4OUT &= ~SPI_SIMO_PIN; }
static inline void mosi_high(void){ P4OUT |=  SPI_SIMO_PIN; }
static inline uint8_t miso_read(void){ return (P4IN & SPI_SOMI_PIN) ? 1 : 0; }

static uint8_t spi_xfer_byte_lsbfirst(uint8_t b_out) {
    uint8_t b_in = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        if (b_out & 0x01) mosi_high(); else mosi_low();
        __delay_cycles(2);           // small setup time (literal)
        sck_high();
        b_in >>= 1;
        if (miso_read()) b_in |= 0x80;
        sck_low();
        b_out >>= 1;
    }
    return b_in;
}
static void spi_write_bytes(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) spi_xfer_byte_lsbfirst(buf[i]);
}
static void spi_read_bytes(uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) buf[i] = spi_xfer_byte_lsbfirst(0x00);
}

/* ---------- PN532 helpers ---------- */
static uint8_t calc_lcs(uint8_t len) { return (uint8_t)(~len + 1); }
static uint8_t calc_dcs(const uint8_t *data, uint8_t n) {
    uint16_t s = 0;
    for (uint8_t i = 0; i < n; ++i) s += data[i];
    return (uint8_t)(~s + 1);
}

/* ---------- IRQ (P1.2) ---------- */
static volatile uint8_t pn532_irq_flag = 0;
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
    if (P1IFG & PN532_IRQ_PIN) {
        P1IFG &= ~PN532_IRQ_PIN;
        pn532_irq_flag = 1;
    }
}

/* ---------- PN532 low-level ---------- */
static uint8_t pn532_read_status(void) {
    uint8_t b;
    cs_low();
    delay_us(5);
    spi_xfer_byte_lsbfirst(PN532_SPI_SR);
    b = spi_xfer_byte_lsbfirst(0x00);
    cs_high();
    return b;
}
static bool pn532_wait_ready(uint16_t timeout_ms) {
    while (timeout_ms--) {
        if (pn532_irq_flag) { pn532_irq_flag = 0; return true; }
        uint8_t st = pn532_read_status();
        if (st & 0x01) return true;    // RDY bit
        delay_ms(1);
    }
    return false;
}
static void pn532_send_frame(const uint8_t *payload, uint8_t len) {
    uint8_t hdr[4] = {0x00, 0xFF, len, calc_lcs(len)};
    cs_low();
    delay_us(5);
    spi_xfer_byte_lsbfirst(PN532_SPI_DW);
    spi_write_bytes(hdr, 4);
    spi_write_bytes(payload, len);
    spi_xfer_byte_lsbfirst(calc_dcs(payload, len));
    spi_xfer_byte_lsbfirst(0x00);
    cs_high();
}
static bool pn532_read_ack(uint16_t timeout_ms) {
    if (!pn532_wait_ready(timeout_ms)) return false;
    uint8_t ack[6];
    cs_low();
    delay_us(5);
    spi_xfer_byte_lsbfirst(PN532_SPI_DR);
    spi_read_bytes(ack, 6);
    cs_high();
    // expected ACK: 00 00 FF 00 FF 00
    return (ack[0]==0x00 && ack[1]==0x00 && ack[2]==0xFF &&
            ack[3]==0x00 && ack[4]==0xFF && ack[5]==0x00);
}
static int pn532_read_response(uint8_t *out, uint8_t out_max, uint16_t timeout_ms) {
    if (!pn532_wait_ready(timeout_ms)) return -1;
    uint8_t hdr[5];
    cs_low();
    delay_us(5);
    spi_xfer_byte_lsbfirst(PN532_SPI_DR);
    spi_read_bytes(hdr, 5);
    if (!(hdr[0]==0x00 && hdr[1]==0x00 && hdr[2]==0xFF)) { cs_high(); return -2; }
    uint8_t len = hdr[3];
    uint8_t lcs = hdr[4];
    if ((uint8_t)(len + lcs) != 0x00) { cs_high(); return -3; }
    if (len + 2 > out_max) { cs_high(); return -4; }
    spi_read_bytes(out, len + 2); // out[0..len-1] = TFI + payload
    cs_high();
    return len; // number of TFI+payload bytes placed in out[0..len-1]
}

/* ---------- PN532 commands ---------- */
static bool pn532_sam_configuration(void) {
    uint8_t payload[] = {0xD4, 0x14, 0x01};
    pn532_send_frame(payload, sizeof(payload));
    if (!pn532_read_ack(200)) return false;
    uint8_t resp[16];
    int len = pn532_read_response(resp, sizeof(resp), 300);
    return (len >= 2 && resp[0]==0xD5 && resp[1]==0x15);
}
static bool pn532_get_firmware_version(uint8_t *ic, uint8_t *ver, uint8_t *rev, uint8_t *sup) {
    uint8_t payload[] = {0xD4, 0x02};
    pn532_send_frame(payload, sizeof(payload));
    if (!pn532_read_ack(200)) return false;
    uint8_t resp[32];
    int len = pn532_read_response(resp, sizeof(resp), 400);
    if (len < 0 || resp[0]!=0xD5 || resp[1]!=0x03) return false;
    *ic=resp[2]; *ver=resp[3]; *rev=resp[4]; *sup=resp[5];
    return true;
}

/* ---------- FIXED: InListPassiveTarget parsing (correct UID extraction) ----------
   Response TFI+payload layout (when reading one tag) in 'resp[]':
   resp[0] = TFI (0xD5)
   resp[1] = Response code (0x4B)
   resp[2] = Nb of targets (N)
   resp[3] = Tg (target number)
   resp[4..5] = SENS_RES (ATQA) (2 bytes)
   resp[6] = SEL_RES  (SAK) (1 byte)
   resp[7] = NFCID length (uid length)
   resp[8..8+UIDlen-1] = UID bytes
*/
static bool pn532_in_list_passive_target(uint8_t *uid, uint8_t *uid_len) {
    uint8_t payload[] = {0xD4, 0x4A, 0x01, 0x00}; // MaxTg=1, BrTy=106 kbps type A
    pn532_send_frame(payload, sizeof(payload));
    if (!pn532_read_ack(500)) return false;
    uint8_t resp[128];
    int len = pn532_read_response(resp, sizeof(resp), 1200);
    if (len < 0) return false;
    if (resp[0] != 0xD5 || resp[1] != 0x4B) return false;
    if (len < 8) return false;            // minimum bytes before UID length field
    uint8_t nb = resp[2];
    if (nb < 1) return false;
    // UID length is at resp[7], UID starts at resp[8]
    uint8_t ulen = resp[7];
    if (ulen < 1 || ulen > 10) return false;
    if ((uint8_t)(8 + ulen) > (uint8_t)len) return false; // bounds
    for (uint8_t i = 0; i < ulen; ++i) uid[i] = resp[8 + i];
    *uid_len = ulen;
    return true;
}

/* ---------- Init ---------- */
static void io_init(void) {
    P4DIR |= (SPI_CS_PIN | SPI_CLK_PIN | SPI_SIMO_PIN);
    P4DIR &= ~SPI_SOMI_PIN;
    P4OUT |= SPI_CS_PIN;
    P4OUT &= ~(SPI_CLK_PIN | SPI_SIMO_PIN);
    P6DIR |= PN532_RST_PIN; P6OUT |= PN532_RST_PIN;
    P1DIR &= ~PN532_IRQ_PIN; P1REN |= PN532_IRQ_PIN; P1OUT |= PN532_IRQ_PIN;
    P1IES |= PN532_IRQ_PIN; P1IFG &= ~PN532_IRQ_PIN; P1IE |= PN532_IRQ_PIN;
}
static void pn532_reset(void) {
    P6OUT &= ~PN532_RST_PIN; delay_ms(10);
    P6OUT |= PN532_RST_PIN;  delay_ms(120); // allow module bootup
}

/* ---------- Main ---------- */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    io_init();
    uart_init();
    __enable_interrupt();

    uart_print("\r\nPN532 SPI demo (fixed UID parse)\r\n");
    pn532_reset();

    if (pn532_sam_configuration()) uart_print("SAMConfiguration OK\r\n");
    else uart_print("SAMConfiguration FAILED\r\n");

    uint8_t ic, ver, rev, sup;
    if (pn532_get_firmware_version(&ic,&ver,&rev,&sup)) {
        uart_print("PN532 Detected\r\n");
        if (ic == 0x32) uart_print(" IC: PN532\r\n");
        else { uart_print(" IC: Unknown (0x"); uart_print_hex(ic); uart_print(")\r\n"); }

        // Firmware major.minor (print as decimal if small)
        uart_print(" Firmware: ");
        uart_putc('0' + ver);
        uart_putc('.');
        if (rev < 10) uart_putc('0' + rev);
        else uart_print_hex(rev);
        uart_print("\r\n");

        uart_print(" Features: ");
        if (sup & 0x01) uart_print("ISO14443A ");
        if (sup & 0x02) uart_print("ISO14443B ");
        if (sup & 0x04) uart_print("NFC ");
        uart_print("\r\n");
    } else {
        uart_print("GetFirmwareVersion FAILED\r\n");
    }

    /* Continuous poll and print UID only when it changes */
    uint8_t last_uid[10] = {0};
    uint8_t last_len = 0;

    while (1) {
        uint8_t uid[10], uidlen = 0;
        
        if (pn532_in_list_passive_target(uid, &uidlen)) {
    uart_print("UID = ");
    for (uint8_t i = 0; i < uidlen; i++) {
        uart_print_hex(uid[i]);
        if (i < uidlen - 1) uart_putc(':');
    }
    uart_print("\r\n");

    // small delay to avoid spamming if tag is held in place
    delay_ms(300);
}
// else: do nothing when no tag

//        if (pn532_in_list_passive_target(uid, &uidlen)) {
//            bool same = (uidlen == last_len);
//            if (same) {
//                for (uint8_t i=0;i<uidlen;i++) if (uid[i] != last_uid[i]) { same = false; break; }
//            }
//            if (!same) {
//                uart_print("UID = ");
//                for (uint8_t i=0;i<uidlen;i++){
//                    uart_print_hex(uid[i]);
//                    if (i < uidlen-1) uart_putc(':');
//                }
//                uart_print("\r\n");
//                // save as last
//                last_len = uidlen;
//                for (uint8_t i=0;i<uidlen;i++) last_uid[i] = uid[i];
//            }
//        } else {
//            // no tag -> clear last so the same tag will be printed next time it appears
//            last_len = 0;
//        }
       // delay_ms(500);
    }
}





#endif
