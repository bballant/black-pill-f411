#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>

static void clock_setup(void) {
  // https://github.com/libopencm3/libopencm3/pull/1303/files
  struct rcc_clock_scale clock_scale = {
      .pllm = 12,
      .plln = 96,
      .pllp = 2,
      .pllq = 4,
      .pllr = 0,
      .pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
      .hpre = RCC_CFGR_HPRE_NODIV,
      .ppre1 = RCC_CFGR_PPRE_DIV2,
      .ppre2 = RCC_CFGR_PPRE_NODIV,
      .voltage_scale = PWR_SCALE1,
      .flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
      .ahb_frequency = 100000000,
      .apb1_frequency = 50000000,
      .apb2_frequency = 100000000,
  };
  rcc_clock_setup_pll(&clock_scale);
}

static void task1(void *args __attribute__((unused))) {
  for (;;) {
    gpio_toggle(GPIOC, GPIO13);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

int main(void) {
  clock_setup();

  // PC13:
  rcc_periph_clock_enable(RCC_GPIOC);

  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);

  xTaskCreate(task1, "task1", 100, NULL, configMAX_PRIORITIES - 1, NULL);
  vTaskStartScheduler();

  for (;;)
    ;
  return 0;
}
