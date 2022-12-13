/* Task based USART demo:
 * Warren Gay VE3WWG
 *
 * This simple demonstration runs from task1, writing 012...XYZ lines
 * one after the other, at a rate of 5 characters/second. This demo
 * uses usart_send_blocking() to write characters.
 *
 * STM32F103C8T6:
 *	TX:	A9  <====> RX of TTL serial
 *	RX:	A10 <====> TX of TTL serial
 *	CTS:	A11 (not used)
 *	RTS:	A12 (not used)
 *	Config:	8N1
 *	Baud:	38400
 * Caution:
 *	Not all GPIO pins are 5V tolerant, so be careful to
 *	get the wiring correct.
 */
#include <FreeRTOS.h>
#include <task.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>

/*********************************************************************
 * Setup the UART
 *********************************************************************/
static void
uart_setup(void) {

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	// UART TX on PA9 (GPIO_USART1_TX)
	gpio_mode_setup(
		GPIOA,
		GPIO_MODE_AF,
		GPIO_PUPD_NONE,
		GPIO9
    );

	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

	usart_set_baudrate(USART1,38400);
	usart_set_databits(USART1,8);
	usart_set_stopbits(USART1,USART_STOPBITS_1);
	usart_set_mode(USART1,USART_MODE_TX);
	usart_set_parity(USART1,USART_PARITY_NONE);
	usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}
//
///*********************************************************************
// * Send one character to the UART
// *********************************************************************/
static inline void
uart_putc(char ch) {
	usart_send_blocking(USART1,ch);
}

/*********************************************************************
 * Send characters to the UART, slowly
 *********************************************************************/
static void
task1(void *args __attribute__((unused))) {
	int c = '0' - 1;

	for (;;) {
		gpio_toggle(GPIOC,GPIO13);
		vTaskDelay(pdMS_TO_TICKS(500));
		if ( ++c >= 'Z' ) {
			uart_putc(c);
			uart_putc('\r');
			uart_putc('\n');
			c = '0' - 1;
		} else	{
			uart_putc(c);
		}
	}
}

/*********************************************************************
 * Main program
 *********************************************************************/
int
main(void) {

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
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_3WS,
		.ahb_frequency  = 100000000,
		.apb1_frequency = 50000000,
		.apb2_frequency = 100000000,
    };

	rcc_clock_setup_pll(&clock_scale);

	// PC13:
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_mode_setup(
		GPIOC,
		GPIO_MODE_OUTPUT,
		GPIO_PUPD_NONE,
		GPIO13
    );

//	gpio_set_output_options(
//		GPIOC,
//		GPIO_OTYPE_PP,
//		GPIO_OSPEED_2MHZ,
//		GPIO13
//    );

	uart_setup();

	xTaskCreate(task1,"task1",100,NULL,configMAX_PRIORITIES-1,NULL);
	vTaskStartScheduler();

	for (;;);

//	int c = '0' - 1;
//	int i;
//
//	for (;;) {
//		gpio_toggle(GPIOC,GPIO13);
//		if ( ++c >= 'Z' ) {
//			uart_putc(c);
//			uart_putc('\r');
//			uart_putc('\n');
//			c = '0' - 1;
//		} else	{
//			uart_putc(c);
//		}
//		for (i = 0; i < 2000000; i++)	/* Wait a bit. */
//			__asm__("nop");
//	}
	return 0;
}

// End
