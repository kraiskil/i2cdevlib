/* An adaptation layer for using
 * the device drivers in i2cdevlib/Arduino/
 * with the avr_libc-implmenetation of I2Cdev
 */
#include <util/delay.h>

/* Delay (busy loop) the given amount of milliseconds */
void delay(unsigned ms)
{
	/* _delay_ms() expects a compile-time constant */
	unsigned i;
	for(i=0; i<ms; i++)
		_delay_ms(1);
}

