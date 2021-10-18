#include <wiringPi.h>

#define pin 7

int main()
{
    wiringPiSetup();
    pinMode(pin, OUTPUT);

    for(;;)
    {
        digitalWrite(pin, HIGH);
        delay(500);
        digitalWrite(pin, LOW);
        delay(500);
    }
    return 0;
}