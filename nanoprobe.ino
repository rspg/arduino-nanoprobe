#include <avr/wdt.h>
#include <SoftwareSerial.h>

typedef long            int32_t;
typedef unsigned long   uint32_t;

#define PIN_SWDCLK 2        // inverted
#define PIN_SWDIO  3
#define PIN_RX     4
#define PIN_TX     5

SoftwareSerial MonitorSerial(PIN_RX, PIN_TX);

#define DIV_ROUND_UP(m, n)      (((m) + (n) - 1) / (n))


#if 0
#define picoprobe_dump(...)     do{ aprintf(__VA_ARGS__); }while(0)
#else
#define picoprobe_dump(...)     ((void)0)
#endif

#if 0
#define picoprobe_debug(...)    do{ aprintf(__VA_ARGS__); }while(0)
#else
#define picoprobe_debug(...)    ((void)0)
#endif

#define SET_BITS(x, y)           ((x) |= (y))
#define CLR_BITS(x, y)           ((x) &= ~(y))

template<typename T>
void print_value(T value)
{
    MonitorSerial.print(value);
}

void aprintf() 
{
    MonitorSerial.println();
}

template<typename T, typename ...Args>
void aprintf(T value, Args... args)
{
    print_value(value);
    aprintf(args...);
}

enum _dbg_pins {
    DBG_PIN_WRITE = 1,
    DBG_PIN_WRITE_WAIT = 2,
    DBG_PIN_READ = 4,
    DBG_PIN_PKT = 8,
};

#define PROBE_BUF_SIZE (512+256)
struct _probe {
    // Total length
    uint32_t tx_len;
    // Data back to host
    uint8_t tx_buf[PROBE_BUF_SIZE];

    // CMD / Data RX'd from
    uint32_t rx_len;
    uint8_t rx_buf[PROBE_BUF_SIZE];

    // PIO offset
    uint32_t offset;
};

struct _probe probe;

#define ENABLE_LATENCY 0
#if ENABLE_LATENCY
int latency_us = 5;
#define LATENCY_DELAY  delayMicroseconds(latency_us)
#else
#define LATENCY_DELAY  ((void)0)
#endif



enum PROBE_CMDS {
    PROBE_INVALID      = 0, // Invalid command
    PROBE_WRITE_BITS   = 1, // Host wants us to write bits
    PROBE_READ_BITS    = 2, // Host wants us to read bits
    PROBE_SET_FREQ     = 3, // Set TCK
    PROBE_RESET        = 4, // Reset all state
    PROBE_TARGET_RESET = 5, // Reset target
};

struct __attribute__((__packed__)) probe_cmd_hdr {
	uint8_t id;
    uint8_t cmd;
    uint32_t bits;
};

struct __attribute__((__packed__)) probe_pkt_hdr {
    uint32_t total_packet_length;
};

void probe_set_swclk_freq(uint32_t freq_khz) 
{

}

void probe_assert_reset(bool state)
{
    wdt_disable();
    wdt_enable(WDTO_15MS);
    while (1) {}
}

void probe_tick(uint8_t on)
{
    const uint8_t mask = _BV(PIN_SWDCLK)|_BV(PIN_SWDIO);
    //SET_BITS(PORTD, mask);
    PORTD = (PORTD&~mask) | _BV(PIN_SWDCLK) | on;
    LATENCY_DELAY;
    CLR_BITS(PORTD, _BV(PIN_SWDCLK));
    LATENCY_DELAY;
}

void probe_tick()
{
    SET_BITS(PORTD, _BV(PIN_SWDCLK));
    LATENCY_DELAY;
    CLR_BITS(PORTD, _BV(PIN_SWDCLK));
    LATENCY_DELAY;
}

void probe_write_bits(uint32_t bit_count, uint8_t data_byte) 
{
    uint8_t wrote_byte = 0;

    int n = bit_count;
    while(n-- > 0){
        const uint8_t bit = data_byte & 1;
        wrote_byte |= bit << (bit_count - n - 1);
        probe_tick((data_byte&1)<<PIN_SWDIO);
        data_byte >>= 1;
    }

    picoprobe_dump("Write ", bit_count, "bits ", wrote_byte);
}

uint8_t probe_read_bits(uint32_t bit_count) 
{
    int n = bit_count;
    uint8_t data = 0;
    while(n-- > 0){
        data |= (PIND&_BV(PIN_SWDIO))>>PIN_SWDIO<<(bit_count - n - 1);
        probe_tick(0);
    }

    picoprobe_dump("Read ", bit_count, "bits ", data);

    return data;
}

void probe_read_mode(void) 
{
    // pio_sm_exec(pio0, PROBE_SM, pio_encode_jmp(probe.offset + probe_offset_in_posedge));
    // while(pio0->dbg_padoe & (1 << PROBE_PIN_SWDIO));
    CLR_BITS(DDRD, _BV(PIN_SWDIO));
    SET_BITS(DDRD, _BV(PIN_SWDCLK));
}

void probe_write_mode(void) 
{
    // pio_sm_exec(pio0, PROBE_SM, pio_encode_jmp(probe.offset + probe_offset_out_negedge));
    // while(!(pio0->dbg_padoe & (1 << PROBE_PIN_SWDIO)));
    SET_BITS(DDRD, _BV(PIN_SWDCLK) | _BV(PIN_SWDIO));
}

void probe_init() 
{
    Serial.begin(115200);
    probe_write_mode();

    picoprobe_debug("Initialine done");
}

void probe_handle_read(uint32_t total_bits) 
{
    picoprobe_debug("Read ", total_bits, " bits");
    probe_read_mode();

    uint32_t chunk;
    uint32_t bits = total_bits;
    while (bits > 0) {
        if (bits > 8) {
            chunk = 8;
        } else {
            chunk = bits;
        }
        probe.tx_buf[probe.tx_len] = probe_read_bits(chunk);
        probe.tx_len++;
        // Decrement remaining bits
        bits -= chunk;
    }
}

void probe_handle_write(uint8_t *data, uint32_t total_bits) 
{
    picoprobe_debug("Write ", total_bits, " bits");

    probe_write_mode();

    uint32_t chunk;
    uint32_t bits = total_bits;
    while (bits > 0) {
        if (bits > 8) {
            chunk = 8;
        } else {
            chunk = bits;
        }

        probe_write_bits(chunk, *data++);
        bits -= chunk;
    }
}

void probe_prepare_read_header(struct probe_cmd_hdr *hdr) 
{
    // We have a read so need to prefix the data with the cmd header
    if (probe.tx_len == 0) {
        // Reserve some space for probe_pkt_hdr
        probe.tx_len += sizeof(struct probe_pkt_hdr);
    }

    memcpy((void*)&probe.tx_buf[probe.tx_len], hdr, sizeof(struct probe_cmd_hdr));
    probe.tx_len += sizeof(struct probe_cmd_hdr);
}

void probe_handle_pkt(void) 
{
    uint8_t *pkt = &probe.rx_buf[0] + sizeof(struct probe_pkt_hdr);
    uint32_t remaining = probe.rx_len - sizeof(struct probe_pkt_hdr);

    digitalWrite(LED_BUILTIN, HIGH);

    picoprobe_debug("Processing packet of length ", probe.rx_len);

    probe.tx_len = 0;
    while (remaining) {
        struct probe_cmd_hdr *hdr = (struct probe_cmd_hdr*)pkt;
        uint32_t data_bytes = DIV_ROUND_UP(hdr->bits, 8);
        pkt += sizeof(struct probe_cmd_hdr);
        remaining -= sizeof(struct probe_cmd_hdr);

        if (hdr->cmd == PROBE_WRITE_BITS) {
            uint8_t *data = pkt;
            probe_handle_write(data, hdr->bits);
            pkt += data_bytes;
            remaining -= data_bytes;
        } else if (hdr->cmd == PROBE_READ_BITS) {
            probe_prepare_read_header(hdr);
            probe_handle_read(hdr->bits);
        } else if (hdr->cmd == PROBE_SET_FREQ) {
            probe_set_swclk_freq(hdr->bits);
        } else if (hdr->cmd == PROBE_RESET) {
            // TODO: Is there anything to do after a reset?
            // tx len and rx len should already be 0
            ;
        } else if (hdr->cmd == PROBE_TARGET_RESET) {
            probe_assert_reset(hdr->bits);
        }
    }
    probe.rx_len = 0;

    if (probe.tx_len) {
        // Fill in total packet length before sending
        struct probe_pkt_hdr *tx_hdr = (struct probe_pkt_hdr*)&probe.tx_buf[0];
        tx_hdr->total_packet_length = probe.tx_len;
        Serial.write(&probe.tx_buf[0], probe.tx_len);
        picoprobe_debug("Picoprobe wrote ", probe.tx_len, " response bytes");
    }
    probe.tx_len = 0;

    digitalWrite(LED_BUILTIN, LOW);
}

void probe_task(void) 
{
    while ( Serial.available() ) 
    {
        probe.rx_buf[probe.rx_len] = Serial.read();
        probe.rx_len ++;

        if (probe.rx_len >= sizeof(struct probe_pkt_hdr)) {
            struct probe_pkt_hdr *pkt_hdr = (struct probe_pkt_hdr*)&probe.rx_buf[0];
            if (pkt_hdr->total_packet_length == probe.rx_len) {
                picoprobe_debug("probe_handle_pkt ", pkt_hdr->total_packet_length, " ", probe.rx_len);
                probe_handle_pkt();
            }
        }
    }
}

// ----------------------------------------------------------------------------

void setup()
{
    MonitorSerial.begin(115200);

    probe_init();
}

void loop()
{
    probe_task();
    delay(1);
}