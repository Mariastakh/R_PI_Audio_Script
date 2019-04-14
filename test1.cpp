#include <wiringPi.h>
#include <mcp3004.h>
#include "maximilian.h"
#include <iostream>

// Delay line variables:
double oldBuffer[441000] = {0.0}
double taps[16] = {0.0};  double dt[16] = {0.0};
double ind[16] = {0,0}; double interpTap[16] = {0.0};
int writeHead = 0;

// Parameter variables:
int maxRange = 26440;  int bpm = 120;
int numTaps, rhythm, ww, vel, pitchFreq, pitchAmp = 0;
double feedbackGain = 0.0; double wa, wf = 1.0; double drywet = 0.5;
double ch[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Wavetable samples:
double wv[16] = {0.0};
// Tap Scalars
double tapVolumes[16] = {0.0};

// Rhythm Volume, and Time Waveforms:
double rs[8400]; double vs[8400];
maxiSample rhythmSample, volumeSample, timeSample;

double secondsInOneBeat = 0.5;
double dryLeft, dryRight, outputSignal = 0.0;

// Audio function declarations:
double tapTime(int tapIndex);
double interpolate(double ind);
double tapScalars(int tapIndex);
// wv is an array, so the of this function needs to be compatible:
double* getWaveSamples();

void setup() {
wiringPiSetup();
mcp3004Setup(70, 0);

  for(int i = 0; i < 8400; i++)
  {
    rs[i] = (rhythmSample.play() + 1) / 2;
    vs[i] = (volumeSample.play() + 1) / 2;
  }

}

// define any non-audio stream functions here
// These need to be initalised and run in player.cpp
void getPotValue()
{
  for(int i = 0; i<8; i++)
  {
    ch[i] = analogRead(70+i);
  }
        // Params Page 1:
        numTaps = map_value(0, maxRange, 0, 16, ch[0]);
        rhythm = map_value(0, maxRange, 0, 16, ch[1]);
        bpm = map_value(0, maxRange, 20, 220, ch[2]);
        feedbackGain = map_value(0, maxRange, 0.0, 0.9, ch[3]);
        wa =  map_value(0, maxRange, 0.0, 1200, ch[4]);
        wf = map_value(0, maxRange, 0.001, 30, ch[5]);
        ww = map_value(0, maxRange, 0, 14, ch[6]);
        vel = map_value(0, maxRange, 0, 16, ch[7]);

        // Params Page 2:
        drywet = map_value(0, maxRange, 0.0, 1.0, ch[8]);
        pitchAmp = map_value(0, maxRange, 0, 2, ch[9]);
        pitchFreq = map_value(0, maxRange, 0, 2, ch[10

        // Convert BPM value to milliseconds:
        //----------------------------------------------------
        double beatsToMs = (60000 / bpm) * 1.0; // 1.0 represents quarter note
        secondsInOneBeat = 60.0 / bpm;

        // Calculate Volume Scalars:
        //--------------------------------------
        double velSum = 0.0;
        for(int i = 0; i < numTaps; i++)
        {
          tapVolumes[i] = tapScalars(j);
          velSum + tapVolumes[i];
        }

        for(int i = 0; i < numTaps; i++)
        {
          tapVolumes[i] = (tapVolumes[i] / volSum) * 1;
        }
}

void play(double *output, double inputBufferValue){

    // Store dry signal:
    dryLeft =  inputBufferValue;
    // Delay Line writeHead and Feedback: is this correct feedback calc??
    delayBuffer[writeHead] = inputBufferValue + (delayBuffer[writeHead] * feedbackGain)
    // Fill the wavetable Samples array: - fix function first
    wv = getWaveSamples();
    // reset output:
    outputSignal = 0.0;

    // Taps Loop:
    //----------------------------------
    for(int i = 0; i < numTaps; i++)
    {
      dt[i] = tapTime(i);
      double index = (writeHead + wv[i]) - (dt[i]);
      if(index < 0){index = 441000+index};
      index = fmod(index, 441000);
      ind[i] = index;
      taps[i] = interpolate(ind[i]);
      taps[i] *= tapVolumes[i];
      outputSignal += taps[i]
    }
    //----------------------------------
    double mix = (outputSignal * drywet) + (dryLeft * (1-drywet));

    output[0]= mix;
    output[1]=output[0];

    writeHead += 1;
    if(writeHead >= 441000)
    {
      writeHead = 0;
    }
}

// Define any functions you will use in the audio stream here:
double tapTime(int tapIndex)
{
  // first you want to space numTaps along the rhythm wavetable buffer:, so divide the waveform buffer size by the number of taps.
  int indexInRs = ((singleWavetableLength / numTaps)* tapIndex);
  // Find the real amplitude value in rs:
  double rcAmpVal = rs[indexInRs + readStartRhythm];
  // Re-scale that difference so it matches the audio buffer scale:
  double limit = secondsInOneBeat * 441999.0;
  double output = jmap(rcAmpVal, 0.0, 1.0, 0.0, limit);
  return output;
}

double interpolate(double ind)
{
    // Interpolation : truncate the index so we have a whole number:
    double interpTap = int(ind);
    // find the remainder:
    double frac = ind - interpTap;
    double next = 0.0;
    // if we dont need to wrap around, then go ahead and read the next sample:
    if (interpTap!= (numSamples - 1))
    {
      next = oldBuffer[interpTap + 1];
    } else { next = oldBuffer[0]; }

    double interpolatedValue = oldBuffer[interpTap] + frac * (next - oldBuffer[interpTap]);
    //double outS = interpolatedValue;
    double out = interpolatedValue;// interpolatedValue;// oldBuffer[ind[i]];

    return out;
}
double tapScalars(int tapIndex)
{
  // space readpointers along the volume wavetable buffer:, so divide the waveform buffer size by the number of taps.
  //int indexInVs = (vs.size() / numTaps) * j;
  int indexInVs = readStart + ((singleWavetableLength / numTaps)* tapIndex);
  // Find the amplitude value in vs at that index, map it to the correct range:
  double vol = vs[indexInVs];
  return vol;
}

// wv is an array, so the of this function needs to be compatible:
double* getWaveSamples()
{

  if (pitchAmp == 0 && pitchFreq == 0)
{
  for (int i = 0; i < wv.size(); i++)
  {
    wv[i] = ((timeSample.play(wf, whichWave, whichWave + 600) + 1.0f) / 2.0f) * wa;
  }
}
else if (pitchAmp == 0 && pitchFreq != 0)
{
  // The global freq and amp controls set all the taps:
  for (int i = 0; i < wv.size(); i++)
  {
    wv[i] = ((timeSample.play(pitchFreqArray[pitchFreq - 1][i], ww, ww + 600) + 1.0f) / 2.0f) * wa;
  }
}
else if (pitchAmp != 0 && pitchFreq == 0)
{
  for (int i = 0; i < numTaps; i++)
  {
    wv[i] = ((timeSample.play(wf, ww, ww + 600) + 1.0f) / 2.0f) * pitchAmpArray[pitchAmp - 1][i];
  }
}
else if (pitchAmp != 0 && pitchFreq != 0)
{
  for (int i = 0; i < numTaps; i++)
  {
    wv[i] = ((timeSample.play(pitchFreqArray[pitchFreq - 1][i], ww, ww + 600) + 1.0f) / 2.0f) * pitchAmpArray[pitchAmp - 1][i];
  }
}

  return wv;
}
