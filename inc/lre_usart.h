/*
 * lre_usart.h
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#ifndef LRE_USART_H_
#define LRE_USART_H_

void lre_usart_init();
void send_usart_string(char str[]);
void hreadable_floats(float number, char *buf);

#ifndef FALSE
#define FALSE 0;
#define TRUE 1;
#endif

#endif /* LRE_USART_H_ */
