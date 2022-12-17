/* Task based USART demo:
 * Warren Gay VE3WWG
 *
 * This simple demonstration runs from task1, writing 012...XYZ lines
 * one after the other, at a rate of 5 characters/second. This demo
 * uses usart_send_blocking() to write characters.
 *
 * STM32F103C8T6:
 *  TX: A9  <====> RX of TTL serial
 *  RX: A10 <====> TX of TTL serial
 *  CTS:    A11 (not used)
 *  RTS:    A12 (not used)
 *  Config: 8N1
 *  Baud:   38400
 * Caution:
 *  Not all GPIO pins are 5V tolerant, so be careful to
 *  get the wiring correct.
 */
#include <math.h>

#include <FreeRTOS.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <task.h>

#include "uartlib.h"

static char *show_binary(int width, int n) {
  int count = (sizeof n) * 8;
  count = width < count ? width : count;
  static char res[((sizeof n) * 8) + 1];
  for (int i = 0; i < count; i++) {
    res[i] = '0' | ((n >> (count - 1 - i)) & 1);
  }
  res[count] = 0;
  return res;
}

static void println_binary(int width, int n) {
    uart1_printf("%s\n", show_binary(width, n));
}

// elementary automation rule 30
static unsigned int ea_thirty[8] = {
    0, 1, 1, 1, 1, 0, 0, 0,
};

static int ea_next(int width, int board) {
  int next_board = 0;
  for (int i = 0; i < width; i++) {
    unsigned int neighbor_bits = 0;
    if (i == 0) {
      // left bit = word's rightmost bit
      unsigned int left_bit = (board & 1) << 2; // [100]
      // shift next two bits all the way to right and mask
      unsigned int right_bits = (board >> (width - i - 2)) & 3;
      neighbor_bits = left_bit | right_bits;
    } else if (i == width - 1) {
      // right bit = word's leftmost bit
      unsigned int right_bit = (board >> (width - 1)) & 1;
      unsigned int left_bits = (board & 3) << 1;
      neighbor_bits = left_bits | right_bit;
    } else {
      // neighbor_bits
      neighbor_bits = (board >> (width - i - 2)) & 7;
    }
    // ensure less than 8
    neighbor_bits = neighbor_bits & 7;
    unsigned int c = ea_thirty[neighbor_bits];
    next_board = (next_board << 1) | c;
  }
  return next_board;
}

static void task1(void *args __attribute__((unused))) {
  static int width = 4;
  int board = 1 << (width / 2);
  println_binary(width, board);
  GPIO_BSRR(GPIOB) = ~board << 16 | board;
  for (;;) {
    gpio_toggle(GPIOC, GPIO13);
    // ensure it's correct width
    board = ea_next(width, board);
    int b = board & ((int) pow((double) 2 , width) - 1);
    println_binary(width, b);
    GPIO_BSRR(GPIOB) = ~b << 16 | b;
    vTaskDelay(pdMS_TO_TICKS(666));
  }
}

/*********************************************************************
 * Setup the UART
 *********************************************************************/
static void uart_setup(void) {

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_USART1);

  // UART TX on PA9 (GPIO_USART1_TX)
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

  gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

  usart_set_baudrate(USART1, 38400);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_enable(USART1);
}


/*********************************************************************
 * Main program
 *********************************************************************/
int main(void) {

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

  // PC13:
  rcc_periph_clock_enable(RCC_GPIOC);

  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);

  // PB0:
  rcc_periph_clock_enable(RCC_GPIOB);

  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  GPIO0 | GPIO1 | GPIO2 | GPIO3);

  uart_setup();

  xTaskCreate(task1, "task1", 100, NULL, configMAX_PRIORITIES - 1, NULL);
  vTaskStartScheduler();

  for (;;)
    ;
  return 0;
}

// End
