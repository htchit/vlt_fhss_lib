#define VLT_TX
#include "../../drivers.h"
#include "../common/blt_led.h"

led_cfg_t led_clean = {1, 1, 1, 1};
led_cfg_t led_power_on = {100, 100, 5, 5};
led_cfg_t led_fhss_bind = {100, 10, 5, 5};
led_cfg_t led_fhss_loss = {200, 800, 1, 0};
led_cfg_t led_good = {1000, 1000, 1, 0};
led_cfg_t led_low_bat = {500, 500, 1, 0};
led_cfg_t led_test = {200, 200, 5, 0};

//GPIO settings
#define GPIO_LED_PIN GPIO_PB6
#define GPIO_BIND_PIN GPIO_PC7
#define GPIO_PPM_PIN GPIO_PA0
#define GPIO_PA_PIN GPIO_PD1
#define GPIO_LNA_PIN GPIO_PD0

struct VLT_GPIO_param
{
    GPIO_PinTypeDef pin_no;
    GPIO_FuncTypeDef function;
    GPIO_PullTypeDef pull;
    unsigned int dir;
};

struct VLT_GPIO_param GPIO[5];

void VLT_GPIO_init()
{
    GPIO[0].pin_no = GPIO_LED_PIN;
    GPIO[0].dir = 1;
    GPIO[0].function = AS_GPIO;
    GPIO[0].pull = PM_PIN_PULLUP_10K;

    GPIO[1].pin_no = GPIO_PPM_PIN;
    GPIO[1].dir = 0;
    GPIO[1].function = AS_GPIO;
    GPIO[1].pull = PM_PIN_PULLUP_10K;

    GPIO[2].pin_no = GPIO_BIND_PIN;
    GPIO[2].dir = 0;
    GPIO[2].function = AS_GPIO;
    GPIO[2].pull = PM_PIN_PULLUP_10K;

    GPIO[3].pin_no = GPIO_PA_PIN;
    GPIO[3].dir = 1;
    GPIO[3].function = AS_GPIO;
    GPIO[3].pull = PM_PIN_PULLDOWN_100K;

    GPIO[4].pin_no = GPIO_LNA_PIN;
    GPIO[4].dir = 1;
    GPIO[4].function = AS_GPIO;
    GPIO[4].pull = PM_PIN_PULLDOWN_100K;

    for (char i = 0; i < 5; i++)
    {
        gpio_set_func(GPIO[i].pin_no, GPIO[i].function);
        gpio_set_output_en(GPIO[i].pin_no, GPIO[i].dir);
        gpio_set_input_en(GPIO[i].pin_no, GPIO[i].dir ^ 1);
        if (GPIO[i].dir == 1)
        {
            gpio_write(GPIO[i].pin_no, 0);
        }
        gpio_setup_up_down_resistor(GPIO[i].pin_no, GPIO[i].pull);
    }
    gpio_set_interrupt(GPIO_PPM_PIN, pol_falling);
    irq_enable_type(FLD_IRQ_GPIO_EN);
    device_led_init(GPIO_LED_PIN, 0);
    device_led_setup(led_power_on);
}
