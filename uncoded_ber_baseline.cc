#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include <cmath>
#include <complex>
#include <vector>
#include <random>
#include <algorithm>
#include <bitset>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OfdmaChannelEstimationSimulation");

// OFDM parameters
const int NUM_SUBCARRIERS = 1024;
const int CP_LENGTH = NUM_SUBCARRIERS / 8;
const double SUBCARRIER_SPACING = 9765.625; // Hz
const double SYMBOL_DURATION = 1 / SUBCARRIER_SPACING;
const double TOTAL_SYMBOL_DURATION = SYMBOL_DURATION + (CP_LENGTH * SYMBOL_DURATION / NUM_SUBCARRIERS);

// Pilot structure
const int FREQ_PILOT_SPACING = 8;
const int TIME_PILOT_SPACING = 7;

// Frame structure
const int SYMBOLS_PER_FRAME = 70;

// Channel model
const int NUM_TAPS = 1;
const double MAX_DELAY_SPREAD = 5e-6; // 5 µs

// Simulation parameters
const int NUM_FRAMES = 100;
const double CENTER_FREQ = 2e9; // 2 GHz

std::complex<double> QpskModulate(int symbol)
{
    switch(symbol)
    {
        case 0: return std::complex<double>(1/std::sqrt(2), 1/std::sqrt(2));
        case 1: return std::complex<double>(-1/std::sqrt(2), 1/std::sqrt(2));
        case 2: return std::complex<double>(1/std::sqrt(2), -1/std::sqrt(2));
        case 3: return std::complex<double>(-1/std::sqrt(2), -1/std::sqrt(2));
        default: return std::complex<double>(0, 0);
    }
}

int QpskDemodulate(const std::complex<double>& signal)
{
    if (signal.real() >= 0 && signal.imag() >= 0) return 0;
    if (signal.real() < 0 && signal.imag() >= 0) return 1;
    if (signal.real() >= 0 && signal.imag() < 0) return 2;
    return 3;
}

std::vector<std::complex<double>> GenerateChannelTaps(int numTaps, double maxDelaySpread)
{
    std::vector<std::complex<double>> taps(numTaps);
    std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> distribution(0.0, 1.0);

    double totalPower = 0;
    for (int i = 0; i < numTaps; ++i)
    {
        double delay = (numTaps > 1) ? (i * maxDelaySpread / (numTaps - 1)) : 0;
        double tapPower = std::exp(-delay / (maxDelaySpread / 3));
        totalPower += tapPower;

        double real = distribution(generator) * std::sqrt(tapPower / 2);
        double imag = distribution(generator) * std::sqrt(tapPower / 2);
        taps[i] = std::complex<double>(real, imag);
    }

    // Normalize taps
    if (totalPower > 0)
    {
        for (auto& tap : taps)
        {
            tap /= std::sqrt(totalPower);
        }
    }
    else
    {
        taps[0] = std::complex<double>(1, 0);
    }

    return taps;
}

std::vector<std::complex<double>> EvolveChannelTaps(const std::vector<std::complex<double>>& taps, double dopplerShift, double time)
{
    std::vector<std::complex<double>> evolvedTaps(taps.size());
    for (size_t i = 0; i < taps.size(); ++i)
    {
        double angle = 2 * M_PI * dopplerShift * time;
        evolvedTaps[i] = taps[i] * std::exp(std::complex<double>(0, angle));
    }
    return evolvedTaps;
}

std::vector<std::complex<double>> ApplyChannel(const std::vector<std::complex<double>>& signal, const std::vector<std::complex<double>>& channelTaps)
{
    std::vector<std::complex<double>> output(signal.size(), std::complex<double>(0, 0));
    for (size_t i = 0; i < signal.size(); ++i)
    {
        for (size_t j = 0; j < channelTaps.size() && i + j < signal.size(); ++j)
        {
            output[i + j] += signal[i] * channelTaps[j];
        }
    }
    return output;
}

std::vector<std::complex<double>> OfdmModulate(const std::vector<std::complex<double>>& input)
{
    std::vector<std::complex<double>> output(NUM_SUBCARRIERS + CP_LENGTH);
    std::vector<std::complex<double>> ifft_in(NUM_SUBCARRIERS, std::complex<double>(0, 0));

    for (size_t i = 0; i < input.size(); ++i)
    {
        ifft_in[i] = input[i];
    }

    // Perform IFFT
    std::vector<std::complex<double>> ifft_out(NUM_SUBCARRIERS);
    for (size_t k = 0; k < NUM_SUBCARRIERS; ++k)
    {
        std::complex<double> sum(0, 0);
        for (size_t n = 0; n < NUM_SUBCARRIERS; ++n)
        {
            double angle = 2 * M_PI * k * n / NUM_SUBCARRIERS;
            sum += ifft_in[n] * std::exp(std::complex<double>(0, angle));
        }
        ifft_out[k] = sum / std::sqrt(static_cast<double>(NUM_SUBCARRIERS));
    }

    // Add cyclic prefix
    std::copy(ifft_out.end() - CP_LENGTH, ifft_out.end(), output.begin());
    std::copy(ifft_out.begin(), ifft_out.end(), output.begin() + CP_LENGTH);

    return output;
}

std::vector<std::complex<double>> OfdmDemodulate(const std::vector<std::complex<double>>& input)
{
    std::vector<std::complex<double>> output(NUM_SUBCARRIERS);
    std::vector<std::complex<double>> fft_in(NUM_SUBCARRIERS);

    // Remove cyclic prefix
    std::copy(input.begin() + CP_LENGTH, input.end(), fft_in.begin());

    // Perform FFT
    for (size_t k = 0; k < NUM_SUBCARRIERS; ++k)
    {
        std::complex<double> sum(0, 0);
        for (size_t n = 0; n < NUM_SUBCARRIERS; ++n)
        {
            double angle = -2 * M_PI * k * n / NUM_SUBCARRIERS;
            sum += fft_in[n] * std::exp(std::complex<double>(0, angle));
        }
        output[k] = sum / std::sqrt(static_cast<double>(NUM_SUBCARRIERS));
    }

    return output;
}

std::vector<std::complex<double>> EstimateChannel(const std::vector<std::vector<std::complex<double>>>& receivedSymbols, const std::vector<std::vector<std::complex<double>>>& pilotSymbols)
{
    std::vector<std::complex<double>> channelEstimate(NUM_SUBCARRIERS);
    std::vector<bool> pilotPositions(NUM_SUBCARRIERS, false);

    // Perform LS estimation at pilot positions
    for (size_t t = 0; t < receivedSymbols.size(); ++t)
    {
        if (t % TIME_PILOT_SPACING == 0)
        {
            for (size_t f = 0; f < NUM_SUBCARRIERS; f += FREQ_PILOT_SPACING)
            {
                if (std::abs(pilotSymbols[t][f]) > 0)
                {
                    channelEstimate[f] = receivedSymbols[t][f] / pilotSymbols[t][f];
                }
                else
                {
                    channelEstimate[f] = std::complex<double>(1, 0);  // Default to 1 if pilot is 0
                }
                pilotPositions[f] = true;
            }
        }
    }

    // Perform interpolation
    for (size_t f = 0; f < NUM_SUBCARRIERS; ++f)
    {
        if (!pilotPositions[f])
        {
            size_t leftPilot = f - (f % FREQ_PILOT_SPACING);
            size_t rightPilot = std::min(leftPilot + FREQ_PILOT_SPACING, static_cast<size_t>(NUM_SUBCARRIERS - 1));

            if (rightPilot == leftPilot)
            {
                channelEstimate[f] = channelEstimate[leftPilot];
            }
            else
            {
                double weight = static_cast<double>(f - leftPilot) / (rightPilot - leftPilot);
                channelEstimate[f] = (1 - weight) * channelEstimate[leftPilot] + weight * channelEstimate[rightPilot];
            }
        }
    }

    return channelEstimate;
}

std::vector<std::complex<double>> ZFEqualize(const std::vector<std::complex<double>>& receivedSymbol, const std::vector<std::complex<double>>& channelEstimate)
{
    std::vector<std::complex<double>> equalizedSymbol(NUM_SUBCARRIERS);
    for (size_t i = 0; i < NUM_SUBCARRIERS; ++i)
    {
        if (std::abs(channelEstimate[i]) > 0)
        {
            equalizedSymbol[i] = receivedSymbol[i] / channelEstimate[i];
        }
        else
        {
            equalizedSymbol[i] = receivedSymbol[i];  // If channel estimate is 0, don't equalize
        }
    }
    
    
    return equalizedSymbol;
}

std::vector<std::complex<double>> MMSEEqualize(const std::vector<std::complex<double>>& receivedSymbol, const std::vector<std::complex<double>>& channelEstimate, double snr)
{
    std::vector<std::complex<double>> equalizedSymbol(NUM_SUBCARRIERS);
    double noisePower = 1.0 / snr;
    for (size_t i = 0; i < NUM_SUBCARRIERS; ++i)
    {
        std::complex<double> h = channelEstimate[i];
        std::complex<double> hConj = std::conj(h);
        double denominator = std::norm(h) + noisePower;
        if (denominator > 0)
        {
            equalizedSymbol[i] = (hConj * receivedSymbol[i]) / denominator;
        }
        else
        {
            equalizedSymbol[i] = receivedSymbol[i];  // If denominator is 0, don't equalize
        }
    }
    
    
    return equalizedSymbol;
}

int main(int argc, char *argv[])
{
    double bandwidth = 10e6; // 10 MHz
    double ueSpeed = 10.0; // UE speed in m/s

    CommandLine cmd;
    cmd.AddValue("bandwidth", "System bandwidth in Hz", bandwidth);
    cmd.AddValue("ueSpeed", "UE speed in m/s", ueSpeed);
    cmd.Parse(argc, argv);

    double dopplerShift = (ueSpeed * CENTER_FREQ) / 3e8; // Calculate Doppler shift

    LogComponentEnable("OfdmaChannelEstimationSimulation", LOG_LEVEL_INFO);

    Ptr<NormalRandomVariable> noiseGenerator = CreateObject<NormalRandomVariable>();
    noiseGenerator->SetAttribute("Mean", DoubleValue(0.0));

    std::vector<std::complex<double>> channelTaps = GenerateChannelTaps(NUM_TAPS, MAX_DELAY_SPREAD);

    for (double snr = -10; snr <= 30; snr += 5)
    {
        double noisePower = 1.0 / std::pow(10, snr / 10.0);
        noiseGenerator->SetAttribute("Variance", DoubleValue(noisePower / 2));

        uint32_t totalBits = 0;
        uint32_t errorBitsZF = 0;
        uint32_t errorBitsMMSE = 0;

        for (int frame = 0; frame < NUM_FRAMES; ++frame)
        {
            std::vector<std::vector<std::complex<double>>> txSymbols(SYMBOLS_PER_FRAME, std::vector<std::complex<double>>(NUM_SUBCARRIERS));
            std::vector<std::vector<std::complex<double>>> rxSymbols(SYMBOLS_PER_FRAME, std::vector<std::complex<double>>(NUM_SUBCARRIERS));
            std::vector<std::vector<std::complex<double>>> pilotSymbols(SYMBOLS_PER_FRAME, std::vector<std::complex<double>>(NUM_SUBCARRIERS));

            // Generate transmit symbols and pilots
            for (int t = 0; t < SYMBOLS_PER_FRAME; ++t)
            {
                for (int f = 0; f < NUM_SUBCARRIERS; ++f)
                {
                    if (t % TIME_PILOT_SPACING == 0 && f % FREQ_PILOT_SPACING == 0)
                    {
                        // Pilot symbol
                        pilotSymbols[t][f] = std::complex<double>(1, 0);
                        txSymbols[t][f] = pilotSymbols[t][f];
                    }
                    else
                    {
                        // Data symbol
                        int dataBits = rand() % 4;
                        txSymbols[t][f] = QpskModulate(dataBits);
                        totalBits += 2;
                    }
                }
            }

            // Apply channel and noise
            for (int t = 0; t < SYMBOLS_PER_FRAME; ++t)
            {

		double time = frame * SYMBOLS_PER_FRAME * TOTAL_SYMBOL_DURATION + t * TOTAL_SYMBOL_DURATION;
                std::vector<std::complex<double>> evolvedTaps = EvolveChannelTaps(channelTaps, dopplerShift, time);
                
                if (frame == 0 && t == 0)
                {
                    std::cout << "Channel magnitude: " << std::abs(evolvedTaps[0]) << std::endl;
                }
                
                std::vector<std::complex<double>> ofdmSymbol = OfdmModulate(txSymbols[t]);
                std::vector<std::complex<double>> channelOutput = ApplyChannel(ofdmSymbol, evolvedTaps);

                // Add noise
                double signalPower = 0;
                for (const auto& sample : channelOutput)
                {
                    signalPower += std::norm(sample);
                }
                signalPower /= channelOutput.size();

                double noisePower = signalPower / std::pow(10, snr / 10.0);
                for (auto& sample : channelOutput)
                {
                    sample += std::complex<double>(noiseGenerator->GetValue(), noiseGenerator->GetValue()) * std::sqrt(noisePower / 2);
                }

                if (frame == 0 && t == 0)
                {
                    std::cout << "Signal power: " << signalPower << ", Noise power: " << noisePower << std::endl;
                }

                rxSymbols[t] = OfdmDemodulate(channelOutput);
            }

            // Perform channel estimation
            std::vector<std::complex<double>> channelEstimate = EstimateChannel(rxSymbols, pilotSymbols);

            // Equalize and demodulate
            for (int t = 0; t < SYMBOLS_PER_FRAME; ++t)
            {
                std::vector<std::complex<double>> equalizedSymbolZF = ZFEqualize(rxSymbols[t], channelEstimate);
                std::vector<std::complex<double>> equalizedSymbolMMSE = MMSEEqualize(rxSymbols[t], channelEstimate, std::pow(10, snr / 10.0));

                for (int f = 0; f < NUM_SUBCARRIERS; ++f)
                {
                    if (!(t % TIME_PILOT_SPACING == 0 && f % FREQ_PILOT_SPACING == 0))
                    {
                        int transmittedBits = QpskDemodulate(txSymbols[t][f]);
                        int receivedBitsZF = QpskDemodulate(equalizedSymbolZF[f]);
                        int receivedBitsMMSE = QpskDemodulate(equalizedSymbolMMSE[f]);

                        if (frame == 0 && t == 0 && f < 5)  // Print first 5 subcarriers of first symbol in first frame
                        {
                            std::cout << "Subcarrier " << f << ": Tx bits: " << transmittedBits
                                      << ", Rx bits ZF: " << receivedBitsZF
                                      << ", Rx bits MMSE: " << receivedBitsMMSE << std::endl;
                        }

                        errorBitsZF += std::bitset<2>(transmittedBits ^ receivedBitsZF).count();
                        errorBitsMMSE += std::bitset<2>(transmittedBits ^ receivedBitsMMSE).count();
                    }
                }
            }
        }

        double berZF = static_cast<double>(errorBitsZF) / totalBits;
        double berMMSE = static_cast<double>(errorBitsMMSE) / totalBits;
	NS_LOG_INFO("SNR: " << snr << " dB"
            << ", BER (ZF): " << berZF
            << ", BER (MMSE): " << berMMSE
            << ", Total Bits: " << totalBits
            << ", Errors ZF: " << errorBitsZF
            << ", Errors MMSE: " << errorBitsMMSE);
    }

    return 0;
}
