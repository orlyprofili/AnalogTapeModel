#ifndef TONECONTROLDYNAMIC_H_INCLUDED
#define TONECONTROLDYNAMIC_H_INCLUDED

#include <JuceHeader.h>
#include "EnvelopeFollower.h"

using SmoothGain = SmoothedValue<float, ValueSmoothingTypes::Multiplicative>;

struct ToneStageDynamic
{
    std::vector<chowdsp::ShelfFilter<float>> tone;
    std::vector<SmoothGain> lowGain, highGain, tFreq;
    float fs = 44100.0f;

    EnvelopeFollower envelopeFollower;  // Declare an instance of EnvelopeFollower

    ToneStageDynamic();

    void prepare(double sampleRate, int numChannels);
    void processBlock(AudioBuffer<float>& buffer);
    void setLowGain(float lowGainDB);
    void setHighGain(float highGainDB);
    void setTransFreq(float newTFreq);
};

class ToneControlDynamic
{
public:
    ToneControlDynamic(AudioProcessorValueTreeState& vts);

    static void createParameterLayout(std::vector<std::unique_ptr<RangedAudioParameter>>& params);
    void prepare(double sampleRate, int numChannels);
    void setDBScale(float newDBScale) { dbScale = newDBScale; };

    void processBlockIn(AudioBuffer<float>& buffer);
    void processBlockOut(AudioBuffer<float>& buffer);

private:
    ToneStageDynamic toneIn, toneOut;

    std::atomic<float>* onOffParam = nullptr;
    chowdsp::FloatParameter* bassParam = nullptr;
    chowdsp::FloatParameter* trebleParam = nullptr;
    chowdsp::FloatParameter* tFreqParam = nullptr;

    float dbScale = 1.0f;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(ToneControlDynamic)
};

#endif // TONECONTROLDYNAMIC_H_INCLUDED
