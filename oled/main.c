/* Demo program for OLED 128x64 SSD1306 controller
 * Warren Gay   Sun Dec 17 22:49:14 2017
 *
 * Important!  	You must have a pullup resistor on the NSS
 * 	       	line in order that the NSS (/CS) SPI output
 *		functions correctly as a chip select. The
 *		SPI peripheral configures NSS pin as an
 *		open drain output.
 *
 * OLED		4-Wire SPI
 *
 * PINS:
 *	PC13	LED
 *	PA15	/CS (NSS, with 10k pullup)
 *	PB3	SCK
 *	PB5	MOSI (MISO not used)
 *	PB10	D/C
 *	PB11	/Reset
 */

#include "intelhex.h"
//#include "mcuio.h"
#include "miniprintf.h"
#include "uartlib.h"

#include "FreeRTOS.h"
#include "meter.h"
#include "oled.h"
#include "queue.h"
#include "task.h"

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>

void oled_command(uint8_t byte) {
  gpio_clear(GPIOA, GPIO10);
  spi_enable(SPI1);
  spi_xfer(SPI1, byte);
  spi_disable(SPI1);
}

void oled_command2(uint8_t byte, uint8_t byte2) {
  gpio_clear(GPIOA, GPIO10);
  spi_enable(SPI1);
  spi_xfer(SPI1, byte);
  spi_xfer(SPI1, byte2);
  spi_disable(SPI1);
}

void oled_data(uint8_t byte) {
  gpio_set(GPIOA, GPIO10);
  spi_enable(SPI1);
  spi_xfer(SPI1, byte);
  spi_disable(SPI1);
}

static void oled_reset(void) {
  gpio_clear(GPIOA, GPIO11);
  vTaskDelay(1);
  gpio_set(GPIOA, GPIO11);
}

static void oled_init(void) {
  static uint8_t cmds[] = {0xAE, 0x00, 0x10, 0x40, 0x81, 0xCF, 0xA1, 0xA6,
                           0xA8, 0x3F, 0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1,
                           0xDA, 0x12, 0xDB, 0x40, 0x8D, 0x14, 0xAF, 0xFF};

  gpio_clear(GPIOC, GPIO13);
  oled_reset();
  for (unsigned ux = 0; cmds[ux] != 0xFF; ++ux)
    oled_command(cmds[ux]);
  gpio_set(GPIOC, GPIO13);
}

/*
 * Monitor task:
 */
static void monitor_task(void *arg __attribute((unused))) {
  struct Meter m1;
  float v = 2.5;


  oled_init();
  meter_init(&m1, 3.5);

  meter_set_value(&m1, v);
  meter_update();

  //bool menuf = true;
  //char ch;
  //std_printf("\nMonitor Task Started.\n");
  for (;;) {
	  //if (menuf) {
	  //  std_printf("\nTest Menu:\n"
	  //             "  0 .. set to 0.0 volts\n"
	  //             "  1 .. set to 1.0 volts\n"
	  //             "  2 .. set to 2.0 volts\n"
	  //             "  3 .. set to 3.0 volts\n"
	  //             "  4 .. set to 3.5 volts\n"
	  //             "  + .. increase by 0.1 volts\n"
	  //             "  - .. decrease by 0.1 volts\n");
	  //}
	  //menuf = false;

	  //std_printf("\n: ");
	  //ch = std_getc();

	  //if (isalpha(ch))
	  //  ch = toupper(ch);
	  //std_printf("%c\n", ch);

	  //switch (ch) {
	  //case '?':
	  //case '\r':
	  //case '\n':
	  //  menuf = true;
	  //  break;
	  //case '+':
	  //  v += 0.1;
	  //  meter_set_value(&m1, v);
	  //  meter_update();
	  //  break;
	  //case '-':
	  //  v -= 0.1;
	  //  meter_set_value(&m1, v);
	  //  meter_update();
	  //  break;
	  //case '0':
	  //  v = 0.0;
	  //  meter_set_value(&m1, v);
	  //  meter_update();
	  //  break;
	  //case '1':
	  //  v = 1.0;
	  //  meter_set_value(&m1, v);
	  //  meter_update();
	  //  break;
	  //case '2':
	  //  v = 2.0;
	  //  meter_set_value(&m1, v);
	  //  meter_update();
	  //  break;
	  //case '3':
	  //  v = 3.0;
	  //  meter_set_value(&m1, v);
	  //  meter_update();
	  //  break;
	  //case '4':
	  //  v = 3.5;
	  //  meter_set_value(&m1, v);
	  //  meter_update();
	  //  break;
	  //default:
	  //  std_printf(" ???\n");
	  //  menuf = true;
	  //}
  }
}

static void uart_setup(void) {

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_USART1);

  // UART TX on PA9 (GPIO_USART1_TX)
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

  gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_enable(USART1);
}

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

int main(void) {

  clock_setup();
  uart_setup();
  // SPI
  rcc_periph_clock_enable(RCC_SPI1);
  // PC13:
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
  //
  // PB0:
  rcc_periph_clock_enable(RCC_GPIOA);
  // PA10 -> D/C; PA11 -> RES
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10 | GPIO11);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
                          GPIO10 | GPIO11);
  // activate OLED reset
  gpio_clear(GPIOA, GPIO11);
  //
  // PA7=MOSI, PA5=SCK, PA4=NSS/CS
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5 | GPIO7);
  gpio_set_af(GPIOA, GPIO_AF5, GPIO4 | GPIO5 | GPIO7);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                          GPIO4 | GPIO5 | GPIO7);

  spi_reset(SPI1);

  spi_init_master(
      SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_disable_software_slave_management(SPI1);
  spi_enable_ss_output(SPI1);

  //usb_start(1, 1);
  //std_set_device(mcu_usb);   // Use USB for std I/O
  gpio_clear(GPIOC, GPIO13); // PC13 = off

  xTaskCreate(monitor_task, "monitor", 500, NULL, 1, NULL);
  vTaskStartScheduler();
  for (;;)
    ;
  return 0;
}

// End
