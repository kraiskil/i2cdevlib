/* An adaptation layer for using
 * the device drivers in i2cdevlib/Arduino/
 * with the avr_libc-implmenetation of I2Cdev.
 * Force-include this file with the gcc '-include' 
 * command line option.
 */
#ifdef __cplusplus
extern "C" {
#endif

/* Delay (busy loop) the given amount of milliseconds */
void delay(unsigned ms);

#ifdef __cplusplus
};
#endif

