#pragma once

#include <cmath>

class EnvelopeFollower
{
public:
    EnvelopeFollower();

    // Set the attack time in milliseconds
    void setAttackTime(float attackTimeMs);

    // Set the release time in milliseconds
    void setReleaseTime(float releaseTimeMs);

    // Set the signal dependence parameter
    void setSignalDependence(float a);

    // Prepare the envelope follower with the given sample rate
    void prepareToPlay(double sampleRate);

    // Process a single audio sample and return the envelope value
    float processSample(float inputSample);

private:
    float fs = 48000.0f;         // Sample rate
    float T = 1.0f / fs;         // Time period per sample
    float attackTime = 10.0f;    // Attack time in milliseconds
    float releaseTime = 100.0f;  // Release time in milliseconds
    float A = 1.5f;              // Signal-dependence parameter
    float z = 0.0f;              // Previous output

    // Update internal time constants if necessary
    void updateTimeConstants();
};