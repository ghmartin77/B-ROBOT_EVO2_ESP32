/*
 * OSC.cpp
 *
 *  Created on: 02.10.2017
 *      Author: anonymous
 */

#include "OSC.h"
#include <WiFiUdp.h>
#include "globals.h"

// for DEBUG uncomment this lines...
//#define OSCDEBUG 0

#define MAX_BUFFER 32
unsigned char UDPBuffer[MAX_BUFFER]; // input message buffer

// OSC message internal variables
unsigned char OSCtouchMessage;
WiFiUDP udp;
IPAddress telemetryServer(192, 168, 4, 2);

// ------- OSC functions -----------------------------------------

// Aux functions
float OSC_extractParamFloat(uint8_t pos) {
	union {
		unsigned char Buff[4];
		float d;
	} u;

	u.Buff[0] = UDPBuffer[pos + 3];
	u.Buff[1] = UDPBuffer[pos + 2];
	u.Buff[2] = UDPBuffer[pos + 1];
	u.Buff[3] = UDPBuffer[pos];

	return (u.d);
}

int16_t OSC_extractParamInt(uint8_t pos) {
	union {
		unsigned char Buff[2];
		int16_t d;
	} u;

	u.Buff[1] = UDPBuffer[pos + 1];
	u.Buff[0] = UDPBuffer[pos];

	return (u.d);
}

void OSC_init() {
	udp.begin(2222);

	OSCfader[0] = 0.5;
	OSCfader[1] = 0.5;
	OSCfader[2] = 0.5;
	OSCfader[3] = 0.5;
}

void OSC_MsgSend(char *c, unsigned char msgSize, float p) {
	uint8_t i;
	union {
		unsigned char Buff[4];
		float d;
	} u;

	// We copy the param in the last 4 bytes
	u.d = p;
	c[msgSize - 4] = u.Buff[3];
	c[msgSize - 3] = u.Buff[2];
	c[msgSize - 2] = u.Buff[1];
	c[msgSize - 1] = u.Buff[0];

	OSC_MsgSend(c, msgSize);
}

void OSC_MsgSend(char *c, unsigned char msgSize) {
	udp.beginPacket(telemetryServer, 2223);
	udp.write((uint8_t*) c, msgSize);
	udp.endPacket();
}

void OSC_MsgRead() {
	int packetSize = udp.parsePacket();

	if (packetSize <= MAX_BUFFER && packetSize > 0) {
		if (udp.read(UDPBuffer, MAX_BUFFER)) {
#ifdef OSCDEBUG
			Serial.println(UDPBuffer);
#endif
			// We look for an OSC message start like /x/
			if ((UDPBuffer[0] == '/') && (UDPBuffer[2] == '/')
					&& ((UDPBuffer[1] == '1') || (UDPBuffer[1] == '2'))) {
				OSCnewMessage = 1;
				OSCpage = UDPBuffer[1] - '0';  // Convert page to int
				OSCtouchMessage = 0;

				float value = OSC_extractParamFloat(16);

				if ((UDPBuffer[5] == 'd') && (UDPBuffer[6] == 'e')
						&& (UDPBuffer[7] == 'r')) {
					// Fader    /1/fader1 ,f  xxxx

					OSCfader[UDPBuffer[8] - '0' - 1] = value;
					return;
				} else if ((UDPBuffer[4] == 'u') && (UDPBuffer[5] == 's')
						&& (UDPBuffer[6] == 'h')) {
					// Push message

					if (value == 0)
						OSCpush[UDPBuffer[7] - '0' - 1] = 0;
					else
						OSCpush[UDPBuffer[7] - '0' - 1] = 1;

					return;
				} else if ((UDPBuffer[6] == 'g') && (UDPBuffer[7] == 'l')
						&& (UDPBuffer[8] == 'e')) {
					// Toggle message
					if (value == 0)
						OSCtoggle[UDPBuffer[9] - '0' - 1] = 0;
					else
						OSCtoggle[UDPBuffer[9] - '0' - 1] = 1;
					return;
				}
			}
		}
	}
}

