
#include <Arduino.h>
#include <MPU9250.h>
#include <quaternionFilters.h>
#include <util/delay.h>


MPU9250 MPU;

void setup()
{
    Serial.begin(9600);
    delay(100);

    
}
void loop()
{
    byte c = 0;
    c = MPU.readByte(0x68,0x75);
    _delay_us(100);
    Serial.println(c, HEX);
}
