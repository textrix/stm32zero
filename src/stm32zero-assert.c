/**
 * STM32ZERO Assert Implementation
 *
 * Provides __assert and __assert_func for newlib's assert.h.
 * Uses STM32ZERO_SIO_REG for direct UART register access (no HAL, no interrupts).
 */

#include "main.h"
#include "stm32zero-sio-reg.h"

static void assert_int_to_str_(int value, char buffer[12])
{
	char tmp[12];
	int i = 0;
	int j = 0;
	int negative = 0;

	if (value == 0) {
		buffer[0] = '0';
		buffer[1] = '\0';
		return;
	}

	if (value < 0) {
		negative = 1;
		value = -value;
	}

	while (value > 0) {
		tmp[i++] = '0' + (value % 10);
		value /= 10;
	}

	if (negative) {
		buffer[j++] = '-';
	}

	while (i--) {
		buffer[j++] = tmp[i];
	}

	buffer[j] = '\0';
}

static void assert_putc_(char c)
{
	while (!(STM32ZERO_SIO_REG->ISR & USART_ISR_TXE_TXFNF)) {
	}
	STM32ZERO_SIO_REG->TDR = c;
}

static void assert_print_(const char* s)
{
	while (*s) {
		assert_putc_(*s++);
	}
}

static void assert_delay_(volatile uint32_t cycles)
{
	while (cycles--) {
		__NOP();
	}
}

static const char msg_[] = "\r\n\r\n--------------- ASSERTION FAILED ---------------\r\n";
static const char crlf_[] = "\r\n";
static char line_text_[12]; /* sign + 10 digits + null */

__NO_RETURN void __assert(const char* filename, int line, const char* expr)
{
	__DSB();
	__disable_irq();
	__ISB();

	assert_int_to_str_(line, line_text_);

	while (1) {
		assert_print_(msg_);
		assert_print_(crlf_);
		assert_print_(filename);
		assert_print_(crlf_);
		assert_print_(line_text_);
		assert_print_(crlf_);
		assert_print_(expr);
		assert_print_(crlf_);

		assert_delay_(0x07FFFFFF);
	}
}

__NO_RETURN void __assert_func(const char* filename, int line,
			       const char* funcname, const char* expr)
{
	__DSB();
	__disable_irq();
	__ISB();

	assert_int_to_str_(line, line_text_);

	while (1) {
		assert_print_(msg_);
		assert_print_(crlf_);
		assert_print_(filename);
		assert_print_(crlf_);
		assert_print_(line_text_);
		assert_print_(crlf_);
		assert_print_(funcname);
		assert_print_(crlf_);
		assert_print_(expr);
		assert_print_(crlf_);

		assert_delay_(0x07FFFFFF);
	}
}
