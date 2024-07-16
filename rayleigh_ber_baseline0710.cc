#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include <cmath>
#include <complex>
#include <random>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("QpskChannelSimulation");

double CalculateQpskBerAwgn(double snr)
{
    return 0.5 * std::erfc(std::sqrt(snr / 2));
}

double CalculateQpskBerRayleigh(double snr)
{
    double gamma = snr / 2;
    return 0.5 * (1 - std::sqrt(gamma / (1 + gamma)));
}

std::complex<double> QpskModulate(int symbol)
{
    switch(symbol)
    {
        case 0: return std::complex<double>(1/std::sqrt(2), 1/std::sqrt(2));
        case 1: return std::complex<double>(-1/std::sqrt(2), 1/std::sqrt(2));
        case 2: return std::complex<double>(1/std::sqrt(2), -1/std::sqrt(2));
        case 3: return std::complex<double>(-1/std::sqrt(2), -1/std::sqrt(2));
        default: return std::complex<double>(0, 0); // Should never happen
    }
}

int QpskDemodulate(const std::complex<double>& signal)
{
    if (signal.real() >= 0 && signal.imag() >= 0) return 0;
    if (signal.real() < 0 && signal.imag() >= 0) return 1;
    if (signal.real() >= 0 && signal.imag() < 0) return 2;
    return 3;
}

std::complex<double> GenerateRayleighFading()
{
    static std::default_random_engine generator;
    static std::normal_distribution<double> distribution(0.0, 1.0);
    double real = distribution(generator);
    double imag = distribution(generator);
    return std::complex<double>(real, imag) / std::sqrt(2.0);
}

int main(int argc, char *argv[])
{
    bool useRayleigh = true;  // Set to false for AWGN only
    double txPower = 1.0; // 1 Watt transmit power

    CommandLine cmd;
    cmd.AddValue("useRayleigh", "Use Rayleigh fading channel", useRayleigh);
    cmd.Parse(argc, argv);

    // Enable logging
    LogComponentEnable("QpskChannelSimulation", LOG_LEVEL_INFO);

    // Create a random number generator for AWGN
    Ptr<NormalRandomVariable> noiseGenerator = CreateObject<NormalRandomVariable>();
    noiseGenerator->SetAttribute("Mean", DoubleValue(0.0));

    // Simulate transmission for different SNR values
    for (double snr = -10; snr <= 10; snr += 1)
    {
        // Calculate noise power based on set SNR
        double noisePower = txPower / std::pow(10, snr / 10.0);
        noiseGenerator->SetAttribute("Variance", DoubleValue(noisePower / 2));

        // Calculate theoretical BER
        double theoreticalBerAwgn = CalculateQpskBerAwgn(std::pow(10, snr / 10));
        double theoreticalBerRayleigh = CalculateQpskBerRayleigh(std::pow(10, snr / 10));

        // Simulate symbol errors
        uint32_t numSymbols = 1000000;
        uint32_t numBitErrors = 0;

        for (uint32_t i = 0; i < numSymbols; ++i)
        {
            int transmittedSymbol = rand() % 4;
            std::complex<double> modulatedSignal = QpskModulate(transmittedSymbol);
            
            std::complex<double> channelCoefficient(1.0, 0.0);
            if (useRayleigh)
            {
                channelCoefficient = GenerateRayleighFading();
            }
            
            std::complex<double> channelOutput = modulatedSignal * channelCoefficient;
            
            // Add AWGN
            std::complex<double> receivedSignal = channelOutput + 
                std::complex<double>(noiseGenerator->GetValue(), noiseGenerator->GetValue());
            
            std::complex<double> demodulationInput = useRayleigh ? 
                receivedSignal / channelCoefficient : receivedSignal;
            
            int receivedSymbol = QpskDemodulate(demodulationInput);
            
            if (transmittedSymbol != receivedSymbol)
            {
                numBitErrors += (transmittedSymbol ^ receivedSymbol) == 3 ? 2 : 1;
            }
        }

        double simulatedBer = static_cast<double>(numBitErrors) / (2 * numSymbols);

        NS_LOG_INFO("Set SNR: " << snr << " dB, Tx Power: " << txPower 
                    << ", Noise Power: " << noisePower
                    << ", Theoretical BER (AWGN): " << theoreticalBerAwgn 
                    << ", Theoretical BER (Rayleigh): " << theoreticalBerRayleigh
                    << ", Simulated BER: " << simulatedBer);
    }

    return 0;
}
