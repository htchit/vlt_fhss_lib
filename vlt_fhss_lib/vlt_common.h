#define FHSS_TIME 5000
#define fhss_rx_timeout_us 1400
#define fhss_sync 50
#define fhss_nSync 100
#define fhss_loss_max 15
#define fhss_con_min 10
#define fhss_bind_timeout_ms 200

//TX Buffer related
static unsigned char __attribute__((aligned(4))) tx_buffer[64] = {0};

unsigned char tx_payload[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0X09, 0X0A};

//RX Buffer related
#define RX_BUF_LEN 64
#define RX_BUF_NUM 4

volatile static unsigned char rx_buf[RX_BUF_LEN * RX_BUF_NUM] __attribute__((aligned(4))) = {};
volatile static unsigned char rx_ptr = 0;
volatile static unsigned char rx_flag = 0;
volatile static unsigned char rx_timeout_flag = 0;
volatile static unsigned char *rx_packet = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned char rssi = 0;
volatile static unsigned int rx_timestamp = 0;
volatile static unsigned int rx_test_cnt = 0;

//VLT_RC related
volatile unsigned char sync_word[4] = {'b', 'i', 'n', 'd'};

volatile unsigned char flash_mid[2] = {0};
volatile unsigned char flash_uid[16] = {0};
volatile int bind_ok = 0;

volatile unsigned char rx_data[10] = {0};

volatile unsigned int rx_next_time = 0;
volatile unsigned int rx_last_time = 0;

const char fhss_channel[53] =
    {3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63, 66, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 123, 126, 129, 132, 135, 138, 141, 144, 147, 150, 153, 156, 159};

volatile unsigned int fhss_ch = 0;
volatile unsigned int fhss_code = 0;
volatile unsigned int fhss_loss = 0;
volatile unsigned int fhss_link = 0;

//self defined structs
typedef enum
{
    RF_wait_con,
    RF_con,
    RF_loss,
} RF_state_enum;

typedef enum
{
    RF_rx_crc_ok,
    RF_rx_nrx,
    RF_rx_tx_done,
    RF_rx_wait,
} RF_rx_state_enum;

typedef enum
{
    RF_setting_none,
    RF_setting_reverse,
    RF_setting_trim,
    RF_setting_FS,
    RF_setting_stop,
} RF_setting_state_enum;

volatile RF_state_enum RF_state = RF_wait_con;
volatile RF_rx_state_enum RF_rx_state = RF_rx_wait;
volatile RF_setting_state_enum RF_setting_state = RF_setting_none;

struct VLT_RX_param
{
    unsigned char sync_word[4];
    unsigned short CH_fs[32];
    unsigned short CH_trim[32];
    unsigned char CH_rev[32];
};

struct VLT_packet
{
    char fhss_code;
    char packet_type;
    unsigned short CH[32];
    unsigned char CH_SW[32];
    unsigned char setting_type;
    unsigned char setting_chanel;
    unsigned char setting_payload[7];
};

struct VLT_packet rf_packet;
struct VLT_RX_param flash_storage;

void rf_packet_gen(unsigned char packet_type)
{
    switch (packet_type)
    {
    case 0:
        tx_payload[0] = (fhss_code << 2) + 0;
        for (unsigned char i = 0; i < 3; i++)
        {
            tx_payload[3 * i + 1] = rf_packet.CH[2 * i] & 0xff;
            tx_payload[3 * i + 2] = (rf_packet.CH[2 * i] >> 8) + ((rf_packet.CH[2 * i + 1] & 0b1111) << 4);
            tx_payload[3 * i + 3] = rf_packet.CH[2 * i + 1] >> 4;
        }
        break;

    case 1:
        tx_payload[0] = (fhss_code << 2) + 1;
        for (unsigned char i = 0; i < 3; i++)
        {
            tx_payload[3 * i + 1] = rf_packet.CH[6 + 2 * i] & 0xff;
            tx_payload[3 * i + 2] = (rf_packet.CH[6 + 2 * i] >> 8) + ((rf_packet.CH[6 + 2 * i + 1] & 0b1111) << 4);
            tx_payload[3 * i + 3] = rf_packet.CH[6 + 2 * i + 1] >> 4;
        }
        break;

    case 2:
        tx_payload[0] = (fhss_code << 2) + 2;
        for (unsigned char i = 0; i < 2; i++)
        {
            tx_payload[3 * i + 1] = rf_packet.CH[12 + 2 * i] & 0xff;
            tx_payload[3 * i + 2] = (rf_packet.CH[12 + 2 * i] >> 8) + ((rf_packet.CH[12 + 2 * i + 1] & 0b1111) << 4);
            tx_payload[3 * i + 3] = rf_packet.CH[12 + 2 * i + 1] >> 4;
        }
        for (unsigned i = 0; i < 8; i++)
        {
            tx_payload[7] += rf_packet.CH_SW[i] << i;
            tx_payload[8] += rf_packet.CH_SW[i + 8] << i;
            tx_payload[9] += rf_packet.CH_SW[i + 16] << i;
        }
        break;

    case 3:
        tx_payload[0] = (fhss_code << 2) + 3;
        tx_payload[1] = rf_packet.setting_type;
        tx_payload[2] = rf_packet.setting_chanel;
        memcpy(&tx_payload[3], rf_packet.setting_payload, sizeof(rf_packet.setting_payload));

    default:
        break;
    }
}

void rf_packet_parse()
{
    rf_packet.fhss_code = (rx_payload[0] & 0b11111100) >> 2;
    fhss_code = rf_packet.fhss_code;
    rf_packet.packet_type = rx_payload[0] & 0b00000011;
    switch (rf_packet.packet_type)
    {
    case 0:
        for (unsigned char i = 0; i < 3; i++)
        {
            rf_packet.CH[2 * i] = tx_payload[3 * i + 1];
            rf_packet.CH[2 * i] = rf_packet.CH[2 * i] + (tx_payload[3 * i + 2] & 0b00001111) << 8;
            rf_packet.CH[2 * i + 1] = ((tx_payload[3 * i + 2] & 0b11110000) >> 4) + (tx_payload[3 * i + 3] << 8);
        }
        break;

    case 1:
        for (unsigned char i = 0; i < 3; i++)
        {
            rf_packet.CH[2 * i + 6] = tx_payload[3 * i + 1];
            rf_packet.CH[2 * i + 6] = rf_packet.CH[2 * i] + (tx_payload[3 * i + 2] & 0b00001111) << 8;
            rf_packet.CH[2 * i + 7] = ((tx_payload[3 * i + 2] & 0b11110000) >> 4) + tx_payload[3 * i + 3] << 8;
        }
        break;

    case 2:
        for (unsigned char i = 0; i < 2; i++)
        {
            rf_packet.CH[2 * i] = tx_payload[3 * i + 1];
            rf_packet.CH[2 * i] = rf_packet.CH[2 * i] + (tx_payload[3 * i + 2] & 0b00001111) << 8;
            rf_packet.CH[2 * i + 1] = ((tx_payload[3 * i + 2] & 0b11110000) >> 4) + (tx_payload[3 * i + 3] << 8);
        }
        break;

    case 3:
        /* code */
        break;

    default:
        break;
    }
}

//VLT rf functions
void VLT_RF_init()
{
    //generic FSK Link Layer configuratioin
    gen_fsk_datarate_set(GEN_FSK_DATARATE_250KBPS); //Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(8);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); //set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0);                //enable pipe0's reception
    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0);              //set pipe0 as the TX pipe
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, sizeof(tx_payload));
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_10DBM);
    gen_fsk_rx_buffer_set(rx_buf + rx_ptr * RX_BUF_LEN, RX_BUF_LEN);
    gen_fsk_channel_set(0);                      //set rf freq as 2400MHz
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); //set transceiver to basic TX state
    gen_fsk_tx_settle_set(149);
}

void VLT_set_palna(unsigned char pa_enable)
{
    if (pa_enable)
    {
        gpio_write(GPIO_LNA_PIN, 0);
        gpio_write(GPIO_PA_PIN, 1);
    }
    else
    {
        gpio_write(GPIO_PA_PIN, 0);
        gpio_write(GPIO_LNA_PIN, 1);
    }
}
