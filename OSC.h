/*
 * OSC.h
 *
 *  Created on: 02.10.2017
 *      Author: anonymous
 */

#ifndef OSC_H_
#define OSC_H_

#include <Arduino.h>

uint8_t OSCpage;
uint8_t OSCnewMessage;
float OSCfader[4];
float OSCxy1_x;
float OSCxy1_y;
uint8_t OSCpush[4];
uint8_t OSCtoggle[2];


void OSC_init();

void OSC_MsgSend(char *c, unsigned char msgSize);
void OSC_MsgSend(char *c, unsigned char msgSize, float p);

void OSC_MsgRead();




#endif /* OSC_H_ */
