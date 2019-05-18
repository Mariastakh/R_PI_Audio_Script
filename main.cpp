// Include the Maximilian and WiringPi library, plus the mcp3004 script included in the WiringPi library
#include "maximilian.h"
#include <wiringPi.h>
#include <mcp3004.h>
#include <iostream>

// Delay Line Variables:
int bufferLen = 88200;
vector<double> delayBuffer(bufferLen);
double taps[16]={0.0}; int dt[16]={0};
double ind[16]={0.0}; double interpTap[16]={0.0};
int writeHead = 0;

// Parameter Variables:
int maxRange = 1023; double delayTime = 0.5; int bpm = 120;
double prevDelayTime = 0.5; int prevSelection = 0;
int numberOfTaps, numTaps, rhythmRead, ww, volumeRead, pitchFreq, pitchAmp = 0;
double feedbackGain = 0.0; int wa = 1;; double wf = 1.0; double drywet = 0.5; double previousWa = 1.0;
double previousFeedbackGain = 0.0; double previousDrywet = 0.5; double previousWf = 1.0;
int previousPitchAmp = 0; int previousPitchFreq = 0;
double ch[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Fixed values for the Wavetable Amplitude and Frequency Presets:
double pitchFreqArray[4][8] = { 
			         {20.48, 10.24, 5.12, 2.56, 1.28, 0.64, 0.32,0.16},
				 {0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.64, 1.28},
				 {11.2, 12.3, 5.5, 12.5, 8.7, 5.6, 12.8, 9.9},
				 {0.01, 2.0, 1.0, 0.73, 0.06, 0.97, 1.3, 2.3} };

double pitchAmpArray[4][8] = { {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0},

				{503.0, 30.0, 12.0, 5231.9, 2801.7, 3583.6, 30.0, 500.0},
				{10.0, 20.0, 300.0, 25.0, 35.0, 65.0, 45.0, 75.0},
				{200.0, 100.0, 255.0, 203.0, 207.0, 205.0, 100.0,97.0}
 };

// Wavetable Samples:
//double wv[16]={0.0};
double wv[16] = {0.0};
int singleWavetableLength; int last_interrupt_time = 0;
int selection = 0; int preset = 0;	
// Tap Scalars:
double tapVolumes[16]={0.0};
// Rhythm Volume, and Time Waveforms:
double rs[8400]; double vs[8400];
maxiSample rhythmSample, volumeSample, timeSample, s1;
vector<maxiSample> samples;

double secondsInOneBeat = 0.5;
double dry, outputSignal = 0.0;

int shiftPin = 6;
bool pageTwo = false; bool changeAmpValue = true; bool changeFreqValue = true;
bool changeFeedbackValue = true; bool changeDelayTime = true;


// Audio function declarations:
int tapTime(int tapIndex); 
double interpolate(double ind);
double tapScalars(int tapIndex);
double getWaveSamples(int tapIndex);

// This function maps the incoming potentionmeter values to the ranges required by the parameters:
int map_value(int inputMin, int inputMax, int outputMin, int outputMax, int value)
{
//std::cout<< "Mapping Next Value..."<<std::endl;
 int inMin = inputMin;
 int inMax = inputMax;
 int outMin = outputMin;
 int outMax = outputMax;
 
 int val = value;

 int input_range = inMax - inMin;
 int output_range = outMax - outMin;
/*
std::cout<< inMin<<std::endl;
std::cout<<inMax<<std::endl;
std::cout<<outMin<<std::endl;
std::cout<<outMax<<std::endl;
std::cout<<val<<std::endl;
std::cout<<input_range<<std::endl;
std::cout<<output_range<<std::endl;
*/int output = (((val - inMin) * output_range) / input_range) + outMin;
return output;
}

// A double version of the mapping function for values that need to be floating points:
double map_double_val(double inputMin, double inputMax, double outputMin, double outputMax, double value)
{
double inMin = inputMin;
double inMax = inputMax;
double outMin = outputMin;
double outMax = outputMax;

double val = value;

double input_range = inMax - inMin;
double output_range = outMax - outMin;

double output = (val - inMin) * output_range / input_range + outMin;

}

// Maximilian Setup Function:
void setup() {
std::cout<<"SETUP STARTING..."<<std::endl;
wiringPiSetup();
mcp3004Setup(70, 0);
// Load samples:
rhythmSample.load("wav_files/fifteen_wavetables_from_AWF_001.wav");
volumeSample.load("wav_files/fifteen_wavetables_from_AWF_001.wav");
timeSample.load("wav_files/fifteen_wavetables_from_AWF_001.wav");
s1.load("wav_files/wood.wav");
samples.resize(5);
samples[0].load("samples/pad.wav");
samples[1].load("samples/bongo.wav");
samples[2].load("samples/click.wav");
samples[3].load("samples/triangle.wav");
samples[4].load("samples/pluck.wav");

singleWavetableLength =  timeSample.getLength() / 14;

	for(int i = 0; i < 8400; i++)
	{
		// Store the waveforms that will be used to get the rhythmic and amplitude values in arrays for later access:
		// Offset the waveform so it ranges between 0 and 1
		rs[i] = (rhythmSample.play() + 1) / 2;
		vs[i] = (volumeSample.play() + 1) / 2;
	}

// Shift Button GPIO:
pinMode(shiftPin ,INPUT);
pullUpDnControl(shiftPin, PUD_UP);
std::cout << "SETUP FINISHED" << std::endl;
}

// Define any non-audio stream functions here,
// These need to be initialised and run in player.cpp

// This function looks out for the shift butotn getting toggled:
void getButtonValue()
{
	if(digitalRead(shiftPin)){
	} else {
		int interrupt_time = millis();
		if(interrupt_time - last_interrupt_time > 200)
		{	
			std::cout<<"SHIFTTTT"<<std::endl;
			pageTwo=!pageTwo;
			std::cout<<pageTwo<<std::endl;	
		}	
	last_interrupt_time = interrupt_time;	

	}
}

// This function uses the wiringPi function analogRead to gran the pot values from the ADC:
void getPotValue()
{
//	std::cout <<"grabbing channels" <<std::endl;
	for(int i = 0; i < 8; i++)
	{
		ch[i] = analogRead(70+i);
//		std::cout << ch[i] << std::endl;
	}
//	std::cout<< "end of channels" <<std::endl;
	
	// Map the pot values to the required ones:
	numTaps = map_value(0, maxRange, 0, 8, ch[1]);
	preset = map_value(0, maxRange, 0, 4, ch[6]);
	rhythmRead = map_value(0, maxRange, 0, 13, ch[5]);
	ww = map_value(0, maxRange, 0, 13, ch[4]);
	drywet = map_double_val(0, maxRange, 0, 1.0, ch[3]);
	wa = map_value(0, maxRange, 0, 2000, ch[2]);
	wf = map_double_val(0, maxRange, 0.001, 5, ch[0]);
	delayTime = map_double_val(0, maxRange, 0.001, 0.5, ch[7]);
	
	/**/

double beatsToMs = (60000 / bpm) * 1.0; // 1.0 represents quarter note
secondsInOneBeat  = 60.0 / bpm;
}

// Play function (where the real-time DSP happens)
void play(double *output, double inputBufferValue)
{
//std::cout << "HI"<<std::endl;
// Store dry signal:
dry = inputBufferValue;
//dryLeft = samples[selection].play();
//delayBuffer[writeHead] = samples[selection].play()+(delayBuffer[writeHead]*feedbackGain);
delayBuffer[writeHead]=inputBufferValue;

// reset output:
outputSignal = 0.0;
double mix = 0.0;
// Taps Loop:
//----------------------
if(numTaps>0){
	// For every tap:
	for(int i = 0; i < numTaps; i++)
	{
		// Grab current waveform function (this value will be used to traverse the delay buffer)
		wv[i] = getWaveSamples(i);
		// Set the time of this tap:
		dt[i] = tapTime(i);
		// Work out the location of the tap along the delay line, using the waveform sample from above, and the tap time:
		double index = (writeHead + wv[i]) - (dt[i]);
		//Wraparound logic:
		if(index < 0){index = bufferLen+index;}
		index = fmod(index, bufferLen);
		// Set the reading index of this tap:
		ind[i] = index;
		// Use interpolation to get the correct value from the delay buffer:
		taps[i] = interpolate(ind[i]);
		// Add it to the output signal:
		outputSignal+=taps[i];
	}

	// Gain control:
	outputSignal = outputSignal/numTaps;
	
    mix = (outputSignal * drywet) + (dry * (1.0 - drywet));
	}else{ mix = dry*(1.0-drywet);}

//std::cout<<dry*(1.0-drywet)<<std::endl;
    output[0]=mix;
    output[1]=output[0];
    
    writeHead += 1;
    if(writeHead >= bufferLen)
    {
	writeHead = 0;
    }
	
}

// Define any functions you will use in the audio stream here:

// This function works out the time at which a tap should occur, in other words, its location along the delay buffer
// This value is yielded by the rhythm waveform stored during setup. 
int tapTime(int tapIndex)
{
	// Work out where to sample the waveform, using the current tap count: 
	// (The higher the count, the further along the waveform we will move)
	int indexInRs = ((singleWavetableLength / numTaps) * tapIndex);
	// Read the amplitude value of the waveform at that point:
	double rcAmpVal = rs[indexInRs + (rhythmRead*600)];
	// Work out the time at which this tap should occur based on the delayTime setting and the tap count:
	double numDivisionSamples = (delayTime * 44100) * tapIndex;
	// Map the amplitude value from the waveform from 0-1 to 0 - the time at which the tap is due to occur:
	double mappedAmp = map_double_val(rcAmpVal, 0.0,1.0,0.0,numDivisionSamples);
	// Add that value as an offset to the tap time:
	double output = numDivisionSamples+mappedAmp;
	// Wrap-around:
	output = fmod(output, bufferLen);
	return output;

}

//Interpolation code from : Richard Boulanger and Victor Lazzarini, The Audio Programming Book, Chapter 6, pp. 495-496
double interpolate(double ind)
{
	// Convert the current index to an integer (i.e. the closest previous existing index)
	double interpTap = int(ind);
	// Acquire the decimal point value difference between the integer value and floating point value:
	double frac = ind - interpTap;
	double next = 0.0;
	// Wrap- around logic:
	if(interpTap != (bufferLen - 1))
	{
		// if we don't need to wrap then next is equal to the value next to the current value in the delay line:
		next = delayBuffer[interpTap + 1];
		// Else wrap:
		} else { next = delayBuffer[0];
	}
	
	// The interpolated value is the result of (the value of the known previous index) + the amount we've moved 
	// multiplied by (the current value subtracted from the next value)
	double interpolatedValue = delayBuffer[interpTap] + frac * 
	(next - delayBuffer[interpTap]);
	double out = interpolatedValue;
	return out;
}

// This function works out the amplitude of each tap. 
// This value is yielded by the volume waveform stored during setup. 
double tapScalars(int tapIndex)
{
	// Work out where to sample the waveform, using the current tap count: 
	// (The higher the count, the further along the waveform we will move)
	int indexInVs = (volumeRead*600) + ((singleWavetableLength / numTaps) * 
	tapIndex);
	// Use the amplitude of the waveform at that index to set the amplitude of the tap:
	double vol = vs[indexInVs];
	return vol;
}

// Yield the waveform sample that is added to the delain line tap index:
double getWaveSamples(int tapIndex)
{
double sample = 0.0;
	
	// If we are not using the presets:
	if(preset == 0)
	{
	// Use the global frequency and amplitude settings when sampling the waveform:
	 sample = ((timeSample.play(wf, (ww*600), (ww*600)+600)+
		1.0f) / 2.0f) * wa;
		
	// Else if we are using the presets, use the preset frequency and amplitude values
	// when sampling the waveform:
	} else if(preset>0)
{
	sample = ((timeSample.play(pitchFreqArray[preset-1][tapIndex],
		(ww*600), (ww*600)+ 600) + 1.0f) / 2.0f) * 
		pitchAmpArray[preset-1][tapIndex];
//std:cout << "pitch frequency array!"<<std::endl;

}
return sample;
}
