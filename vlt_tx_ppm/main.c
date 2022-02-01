#include "vlt_config.h"
#include "../../vlt_fhss_lib/tx.h"

volatile unsigned long ppm_last_time = 0;
volatile unsigned long ppm_this_time = 0;
unsigned int ppm_timer[18];
unsigned char ppm_count = 0;

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();
    if (irq_src & FLD_IRQ_ZB_RT_EN) //if rf irq occurs
    {
        if (rf_irq_src & FLD_RF_IRQ_RX) //if rf rx irq occurs
        {
            rf_irq_clr_src(FLD_RF_IRQ_RX);
            rx_packet = rx_buf + rx_ptr * RX_BUF_LEN; //set to next rx_buf
            rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;
            gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
            if (gen_fsk_is_rx_crc_ok(rx_packet))
            {
                RF_rx_state = RF_rx_crc_ok;
            }
            else
            {
                RF_rx_state = RF_rx_nrx;
            }
        }

        if (rf_irq_src & FLD_RF_IRQ_RX_TIMEOUT) //if rf tx irq occurs
        {
            rf_irq_clr_src(FLD_RF_IRQ_FIRST_TIMEOUT);
            RF_rx_state = RF_rx_nrx;
        }

        if (rf_irq_src & FLD_RF_IRQ_TX) //if rf tx irq occurs
        {
            rf_irq_clr_src(FLD_RF_IRQ_TX);
            RF_rx_state = RF_rx_tx_done;
        }
    }

    if (irq_src & FLD_IRQ_GPIO_EN)
    {
        ppm_last_time = ppm_this_time;
        ppm_this_time = clock_time();
    }

    irq_clr_src2(FLD_IRQ_ALL);
}

_attribute_ram_code_sec_noinline_ int main(void)
{
    VLT_init();
    while (1)
    {
        VLT_FHSS_service();
        device_led_process();

        if ((ppm_this_time - ppm_last_time) > 900 * 16)
        {
            ppm_count = 0;
        }
        else
        {
            ppm_timer[ppm_count] = ppm_this_time - ppm_last_time;
            rf_packet.CH[ppm_count] = 4095 * (ppm_timer[ppm_count] - 1000) / 1000;
            ppm_count++;
        }
    }
}
