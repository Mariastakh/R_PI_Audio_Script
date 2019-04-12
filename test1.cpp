
#include "maximilian.h"
#include <wiringPi.h>
#include <ads1115.h>

vector<double> delayBuffer(441000, 0.0);

int counter, counter2, counter3 = 0;
int numSeconds, maxNumSamples;
int delayTime = 1 * 44100;
int delayTime2 = 22050;
int ch0;

maxiOsc mySine;//One oscillator - can be called anything. Can be any of the available waveforms.

int map_value(int inputMin, int inputMax, int outputMin, int outputMax, int value)
{
 int inMin = inputMin / 1000;
 int inMax = inputMax / 1000;
 int outMin = outputMin / 1000;
 int outMax = outputMax / 1000;

 int val = value / 1000;

 int input_range = inMax - inMin;
 int output_range = outMax - outMin;

 int output = (val - inMin) * output_range / input_range + outMin;
 return output*1000;
}


void setup() {//some inits
numSeconds = 10;
maxNumSamples = numSeconds * 44100;    //nothing to go here this time
wiringPiSetup();
ads1115Setup(70, 0x48);

}

void getPotValue()
{
//std::cout << "hi bitch"<< std::endl;

        ch0 = analogRead(70+0);
        delayTime = map_value(0, 26440, 1, 441000, ch0);
//      std::cout <<delayTime<<std::endl;
}

void play(double *output, double inputBufferValue) {

   delayBuffer[counter] = inputBufferValue;
   counter++;
   if(counter >= maxNumSamples)
   { counter = 0; }

   counter2 = counter-delayTime;
   if(counter2 >= 0 && counter2 < maxNumSamples){
        counter2=counter2;
   }else if(counter2 >= maxNumSamples){
        counter2=counter2-maxNumSamples;
   }else{
        counter2=counter2+maxNumSamples;}

  double tap1 = delayBuffer[counter2];

    output[0]= (inputBufferValue*0.5) + (tap1 * 0.5); //mySine.sinewave(440);
    output[1]=output[0];
}
