#include "vlt_common.h"

//VLT rf functions

void VLT_RF_bind()
{
    gen_fsk_srx2tx_start(tx_buffer, clock_time() + 50 * 16, fhss_bind_timeout_ms * 1000);
    while (RF_rx_state == RF_rx_wait)
    {
    }
    if (RF_rx_state == RF_rx_crc_ok)
    {
        RF_rx_state = RF_rx_wait;
        rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
        rssi = (gen_fsk_rx_packet_rssi_get(rx_packet) + 110);
        rx_timestamp = gen_fsk_rx_timestamp_get(rx_packet);
        for (unsigned char i = 0; i < 23; i++)
        {
            rx_data[i] = *rx_payload;
            rx_payload += 1;
        }
        sync_word[0] = rx_data[1];
        sync_word[1] = rx_data[2];
        sync_word[2] = rx_data[3];
        sync_word[3] = rx_data[4];
        flash_erase_sector(0x40000);
        WaitMs(5);
        flash_write_page(0x40000, 4, sync_word);
        WaitMs(10);
        reg_pwdn_ctrl = FLD_PWDN_CTRL_REBOOT;
    }
    else if (RF_rx_state == RF_rx_nrx)
    {
        RF_rx_state = RF_rx_wait;
    }
    else if (RF_rx_state == RF_rx_tx_done)
    {
        RF_rx_state = RF_rx_wait;
    }
}

void VLT_init()
{
    cpu_wakeup_init();
    clock_init(SYS_CLK_24M_Crystal);

    VLT_GPIO_init();

    VLT_RF_init();

    //irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_FIRST_TIMEOUT);
    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
    irq_enable();                      //enable general irq

    //fill the DMA tx buffer
    tx_buffer[0] = sizeof(tx_payload);
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    memcpy(&tx_buffer[4], tx_payload, sizeof(tx_payload));

    VLT_RF_bind();
    flash_read_page(0x40000, 4, sync_word);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word);
    gen_fsk_channel_set(fhss_channel[0]);
    rx_next_time = clock_time() + 50 * 16;
    rx_last_time = rx_next_time;
    gen_fsk_srx2tx_start(tx_buffer, rx_next_time, fhss_rx_timeout_us);
}

void VLT_FHSS_service()
{
    if (RF_rx_state == RF_rx_crc_ok)
    {
        RF_rx_state = RF_rx_wait;
        VLT_set_palna(1);
        fhss_loss = 0;
        fhss_link++;
        RF_state = RF_con;
        device_led_setup(led_good);
        rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
        rssi = (gen_fsk_rx_packet_rssi_get(rx_packet) + 110);
        rx_timestamp = gen_fsk_rx_timestamp_get(rx_packet);
        rf_packet_parse();
        rx_next_time = rx_last_time + 5000 * 16;
    }
    else if (RF_rx_state == RF_rx_nrx)
    {
        RF_rx_state = RF_rx_wait;
        VLT_set_palna(0);
        fhss_loss++;
        fhss_link = 0;
        if (fhss_loss > fhss_loss_max)
        {
            RF_state = RF_loss;
            device_led_setup(led_fhss_loss);
        }
        if (RF_state == RF_con)
        {
            fhss_ch = fhss_ch + 1 + fhss_code;
            if (fhss_ch > (sizeof(fhss_channel) - 1))
            {
                fhss_ch -= sizeof(fhss_channel);
            }
            gen_fsk_channel_set(fhss_channel[fhss_ch]);
            rx_next_time = rx_last_time + 5000 * 16;
            rx_last_time = rx_next_time;
            gen_fsk_srx2tx_start(tx_buffer, rx_next_time, 2500);
        }
        else
        {
            rx_next_time = rx_last_time + 6300 * 16;
            rx_last_time = rx_next_time;
            gen_fsk_srx2tx_start(tx_buffer, rx_next_time, 5000);
        }
    }
    else if (RF_rx_state == RF_rx_tx_done)
    {
        RF_rx_state = RF_rx_wait;
        VLT_set_palna(0);
        fhss_ch = fhss_ch + 1 + fhss_code;
        if (fhss_ch > (sizeof(fhss_channel) - 1))
        {
            fhss_ch -= sizeof(fhss_channel);
        }
        gen_fsk_channel_set(fhss_channel[fhss_ch]);
        rx_last_time = rx_next_time;
        gen_fsk_srx2tx_start(tx_buffer, rx_next_time, 2500);
    }
}
