/*
 * OSC.h
 *
 *  Created on: 02.10.2017
 *      Author: anonymous
 */

#ifndef OSC_H_
#define OSC_H_

#include <Arduino.h>

void OSC_init();

void OSC_MsgSend(char *c, unsigned char msgSize);
void OSC_MsgSend(char *c, unsigned char msgSize, float p);

void OSC_MsgRead();




#endif /* OSC_H_ */
