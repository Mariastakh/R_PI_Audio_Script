#include <wiringPi.h>
#include <ads1115.h>
#include <iostream>

int main(void)
{
  wiringPiSetup();
  ads1115Setup(70, 0x48);


    for(;;)
    {
       int ch0 = analogRead(70+0);
       Std::cout << ch0 << std::endl;
    }
}
