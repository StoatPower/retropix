#ifndef __ARDUCAM_SLOT_H
#define __ARDUCAM_SLOT_H
#include <Arduino.h>

#define SerialBegin(baudRate)       Serial.begin(baudRate)
#define SerialWrite(ch)             Serial.write(ch)
#define SerialWriteBuff(buf, len)   Serial.write(buf, len)
#define SerialPrintf(str)           Serial.print(str)
#define SerialAvailable()           Seiral.available()
#define SerialRead()                Serial.read()
#define dalayUs(us)                 delayMicroseconds(us)
#endif