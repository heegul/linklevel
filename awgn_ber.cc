#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/log.h"
#include "ns3/random-variable-stream.h"
#include <cmath>
#include <complex>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("QpskRayleighAwgnSimulation");

double CalculateQpskBer(double snr)
{
    return 0.5 * std::erfc(std::sqrt(snr / 2));
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
    double u1 = (rand() + 1.0) / (RAND_MAX + 1.0);
    double u2 = (rand() + 1.0) / (RAND_MAX + 1.0);
    return std::complex<double>(std::sqrt(-2 * std::log(u1)) * std::cos(2 * M_PI * u2),
                                std::sqrt(-2 * std::log(u1)) * std::sin(2 * M_PI * u2));
}

int main(int argc, char *argv[])
{
    LogComponentEnable("QpskRayleighAwgnSimulation", LOG_LEVEL_INFO);

    double txPower = 1.0; // 1 Watt transmit power

    // Simulate transmission for different SNR values
    for (double snr = -10; snr <= 20; snr += 1)
    {
        // Calculate noise power based on set SNR
        double noisePower = txPower / std::pow(10, snr / 10.0);

        // Calculate theoretical BER (AWGN only, for reference)
        double theoreticalBerAwgn = CalculateQpskBer(std::pow(10, snr / 10));

        // Simulate symbol errors
        uint32_t numSymbols = 1000000;
        uint32_t numBitErrors = 0;
        Ptr<NormalRandomVariable> noiseGenerator = CreateObject<NormalRandomVariable>();
        noiseGenerator->SetAttribute("Mean", DoubleValue(0.0));
        noiseGenerator->SetAttribute("Variance", DoubleValue(noisePower / 2)); // Divided by 2 for I and Q components

        for (uint32_t i = 0; i < numSymbols; ++i)
        {
            // Generate random symbol (0, 1, 2, or 3)
            int transmittedSymbol = rand() % 4;
            
            // Modulate
            std::complex<double> modulatedSignal = QpskModulate(transmittedSymbol);
            
            // Generate and apply Rayleigh fading
            std::complex<double> channelCoefficient = GenerateRayleighFading();
            std::complex<double> fadedSignal = modulatedSignal * channelCoefficient;
            
            // Add AWGN
            std::complex<double> receivedSignal = fadedSignal + 
                std::complex<double>(noiseGenerator->GetValue(), noiseGenerator->GetValue());
            
            // Channel estimation and compensation (zero-forcing equalizer)
            std::complex<double> equalizedSignal = receivedSignal / channelCoefficient;
            
            // Demodulate
            int receivedSymbol = QpskDemodulate(equalizedSignal);
            
            // Count bit errors (Gray coding assumption)
            if (transmittedSymbol != receivedSymbol)
            {
                numBitErrors += (transmittedSymbol ^ receivedSymbol) == 3 ? 2 : 1;
            }
        }

        double simulatedBer = static_cast<double>(numBitErrors) / (2 * numSymbols); // 2 bits per symbol

        NS_LOG_INFO("Set SNR: " << snr << " dB, Tx Power: " << txPower 
                    << ", Noise Power: " << noisePower
                    << ", Theoretical BER (AWGN): " << theoreticalBerAwgn 
                    << ", Simulated BER (Rayleigh + AWGN): " << simulatedBer);
    }

    return 0;
}
