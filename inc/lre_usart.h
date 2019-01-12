/*
 * lre_usart.h
 *
 *  Created on: 24.12.2018
 *      Author: Nicolas Klenert
 */

#ifndef LRE_USART_H_
#define LRE_USART_H_

void usart_init();
void send_usart_string(char str[]);
void send_usart_letter(char letter);
void hreadable_floats(float number, char *buf);

#endif /* LRE_USART_H_ */
