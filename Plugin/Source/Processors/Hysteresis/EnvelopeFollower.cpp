//This is from medium article: https://jatinchowdhury18.medium.com/developing-a-signal-dependent-envelope-follower-9cb5b89f9595

#include "EnvelopeFollower.h"
#include <cmath>

namespace
{
    constexpr float defaultAttackTimeMs = 10.0f; // Default attack time in milliseconds
    constexpr float defaultReleaseTimeMs = 100.0f; // Default release time in milliseconds
    constexpr float defaultSignalDependence = 1.5f; // Default signal dependence parameter
    constexpr float convergenceThreshold = 1.0e-3f; // Convergence threshold for the iterative solver
    constexpr int maxIterations = 100; // Maximum iterations for Newton-Raphson solver
} // namespace

EnvelopeFollower::EnvelopeFollower() = default;

void EnvelopeFollower::setAttackTime(float attackTimeMs)
{
    attackTime = attackTimeMs;
    updateTimeConstants();
}

void EnvelopeFollower::setReleaseTime(float releaseTimeMs)
{
    releaseTime = releaseTimeMs;
    updateTimeConstants();
}

void EnvelopeFollower::setSignalDependence(float a)
{
    A = a;
}

void EnvelopeFollower::prepareToPlay(double sampleRate)
{
    fs = static_cast<float>(sampleRate);
    T = 1.0f / fs;
    z = 0.0f;
}

float EnvelopeFollower::processSample(float inputSample)
{
    const float absInput = std::abs(inputSample);
    const bool isAttack = absInput > z;

    // Choose appropriate time constant based on attack or release phase
    const float G = isAttack ? attackTime : releaseTime;

    auto lambdaFunctionLite = [this, G](float l, float z, float c) -> float
    {
        return c + std::exp(-T / G) * (z - c) - l;
    };

    auto lambdaFunctionFull = [this, G](float l, float z, float c) -> float
    {
        // Compute f(λ) and f'(λ), this can be optimized a lot :)
        float f = G * std::exp(A * l);
        return c + std::exp(-T / f) * (z - c) - l;
    };


    float l = z;
    float delta = 100.0f;

    // Refined Newton-Raphson loop
    int iteration = 0;
    while (std::abs(delta) > convergenceThreshold && iteration < maxIterations)
    {
        // Compute f(λ) and f'(λ)
        float fl = G * std::exp(A * l);
        float flp = G * A * std::exp(A * l);

        // Compute the function value F and its derivative F'
        float F = lambdaFunctionFull(l, z, absInput);
        float F_prime = flp * (T / (fl * fl)) * std::exp(-T / fl) * (z - absInput) - 1.0f;
        
        // Update the value of λ using the Newton-Raphson formula
        delta = F / F_prime;
        l -= delta;

        ++iteration;
    }

    z = l;
    return z;
}

void EnvelopeFollower::updateTimeConstants()
{
    // Placeholder for complex time constant updates, if needed in future.
}