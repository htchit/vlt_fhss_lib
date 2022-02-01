#include "../../drivers.h"
#include "./genfsk_ll/genfsk_ll.h"
#include "vlt_common.h"

//VLT rf functions

void VLT_RF_bind()
{
    tx_buffer[0] = sizeof(tx_payload);
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    tx_payload[0] = fhss_code << 2;
    tx_payload[1] = sync_word[0];
    tx_payload[2] = sync_word[1];
    tx_payload[3] = sync_word[2];
    tx_payload[4] = sync_word[3];

    memcpy(&tx_buffer[4], tx_payload, sizeof(tx_payload));
    device_led_setup(led_fhss_bind);
    if (gpio_read(GPIO[2].pin_no) == 0)
    {
        gen_fsk_stx2rx_start(tx_buffer, clock_time() + 50 * 16, 5000);
        while (bind_ok == 0)
        {
            device_led_process();
            if (RF_rx_state == RF_rx_crc_ok)
            {
                RF_rx_state = RF_rx_wait;
                bind_ok = 1;
            }
            else if (RF_rx_state == RF_rx_nrx)
            {
                RF_rx_state = RF_rx_wait;
                gen_fsk_stx2rx_start(tx_buffer, clock_time() + 50 * 16, 5000);
            }
            else if (RF_rx_state == RF_rx_tx_done)
            {
                RF_rx_state = RF_rx_wait;
            }
        }
    }
}

void VLT_init()
{
    cpu_wakeup_init();
    clock_init(SYS_CLK_24M_Crystal);

    random_generator_init();
    fhss_code = rand() % sizeof(fhss_channel);

    flash_read_mid_uid_with_check(flash_mid, flash_uid);

    VLT_GPIO_init();

    VLT_RF_init();

    sync_word[0] = flash_uid[0];
    sync_word[1] = flash_uid[1];
    sync_word[2] = flash_uid[2];
    sync_word[3] = flash_uid[3];

    //irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_RX_TIMEOUT);
    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
    irq_enable();                      //enable general irq

    VLT_RF_bind();

    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word);
    rx_last_time = clock_time() + 50 * 16;
    gen_fsk_stx2rx_start(tx_buffer, clock_time() + 50 * 16, 1 * 1000);
}

void fhss_tx()
{
    rf_packet_gen(0);
    fhss_ch = fhss_ch + 1 + fhss_code;
    if (fhss_ch > 52)
    {
        fhss_ch -= 53;
    }
    gen_fsk_channel_set(fhss_channel[fhss_ch]);
    memcpy(&tx_buffer[4], tx_payload, sizeof(tx_payload));
    rx_next_time = rx_last_time + FHSS_TIME * 16;
    rx_last_time = rx_next_time;
    gen_fsk_stx2rx_start(tx_buffer, rx_next_time, fhss_rx_timeout_us);
}

void VLT_FHSS_service()
{
    if (RF_rx_state == RF_rx_crc_ok)
    {
        VLT_set_palna(1);
        RF_rx_state = RF_rx_wait;
        fhss_loss = 0;
        fhss_link++;
        if (fhss_link > fhss_con_min)
        {
            RF_state = RF_con;
            device_led_setup(led_good);
        }
        rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
        rssi = (gen_fsk_rx_packet_rssi_get(rx_packet) + 110);
        rx_timestamp = gen_fsk_rx_timestamp_get(rx_packet);
        for (unsigned char i = 0; i < 10; i++)
        {
            rx_data[i] = *rx_payload;
            rx_payload += 1;
        }
        fhss_tx();
    }
    else if (RF_rx_state == RF_rx_nrx)
    {
        VLT_set_palna(1);
        RF_rx_state = RF_rx_wait;
        fhss_loss++;
        fhss_link = 0;
        if (fhss_loss > fhss_loss_max)
        {
            RF_state = RF_loss;
            device_led_setup(led_fhss_loss);
        }
        fhss_tx();
    }
    else if (RF_rx_state == RF_rx_tx_done)
    {
        RF_rx_state = RF_rx_wait;
        VLT_set_palna(0);
    }
}
