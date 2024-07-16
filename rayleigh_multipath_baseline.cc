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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("QpskMultipathSimulation");

double CalculateQpskBerAwgn(double snr)
{
    return 0.5 * std::erfc(std::sqrt(snr / 2));
}

double CalculateQpskBerRayleigh(double snr)
{
    double gamma = snr / 2;
    return 0.5 * (1 - std::sqrt(gamma / (1 + gamma)));
}

double CalculateEffectiveSnr(const std::vector<std::complex<double>>& taps, double snr)
{
    double tapPowerSum = 0;
    for (const auto& tap : taps)
    {
        tapPowerSum += std::norm(tap);
    }
    
    double effectiveSnr = snr * tapPowerSum / taps.size();
    return effectiveSnr;
}

double CalculateQpskBerMultipath(const std::vector<std::complex<double>>& taps, double snr)
{
    double effectiveSnr = CalculateEffectiveSnr(taps, snr);
    // Use the AWGN BER formula with the effective SNR as an approximation
    return CalculateQpskBerAwgn(effectiveSnr);
}
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

std::complex<double> GenerateRayleighFading()
{
    static std::default_random_engine generator;
    static std::normal_distribution<double> distribution(0.0, 1.0);
    double real = distribution(generator);
    double imag = distribution(generator);
    return std::complex<double>(real, imag) / std::sqrt(2.0);
}

std::vector<std::complex<double>> GenerateMultipathTaps(int numPaths)
{
    std::vector<std::complex<double>> taps;
    for (int i = 0; i < numPaths; ++i)
    {
        taps.push_back(GenerateRayleighFading());
    }
    return taps;
}

std::complex<double> ApplyMultipath(const std::complex<double>& signal, const std::vector<std::complex<double>>& taps)
{
    std::complex<double> output(0, 0);
    for (const auto& tap : taps)
    {
        output += signal * tap;
    }
    return output;
}

std::vector<std::complex<double>> Equalize(const std::vector<std::complex<double>>& receivedSignal, const std::vector<std::complex<double>>& taps)
{
    std::vector<std::complex<double>> equalizedSignal(receivedSignal.size());
    std::complex<double> tapSum(0, 0);
    for (const auto& tap : taps)
    {
        tapSum += tap;
    }
    for (size_t i = 0; i < receivedSignal.size(); ++i)
    {
        equalizedSignal[i] = receivedSignal[i] / tapSum;
    }
    return equalizedSignal;
}

int main(int argc, char *argv[])
{
    int numPaths = 2;  // Number of multipath components
    double txPower = 1.0; // 1 Watt transmit power

    CommandLine cmd;
    cmd.AddValue("numPaths", "Number of multipath components", numPaths);
    cmd.Parse(argc, argv);

    LogComponentEnable("QpskMultipathSimulation", LOG_LEVEL_INFO);

    Ptr<NormalRandomVariable> noiseGenerator = CreateObject<NormalRandomVariable>();
    noiseGenerator->SetAttribute("Mean", DoubleValue(0.0));

    for (double snr = -10; snr <= 0; snr += 1)
    {
        double noisePower = txPower / std::pow(10, snr / 10.0);
        noiseGenerator->SetAttribute("Variance", DoubleValue(noisePower / 2));

        double theoreticalBerAwgn = CalculateQpskBerAwgn(std::pow(10, snr / 10));
        double theoreticalBerRayleigh = CalculateQpskBerRayleigh(std::pow(10, snr / 10));

        std::vector<std::complex<double>> taps = GenerateMultipathTaps(numPaths);
        double theoreticalBerMultipath = CalculateQpskBerMultipath(taps, std::pow(10, snr / 10));

	double effectiveSnr = CalculateEffectiveSnr(taps, snr);

        uint32_t numSymbols = 50000000;
        uint32_t numBitErrors = 0;

        std::vector<std::complex<double>> transmittedSignal;
        std::vector<std::complex<double>> receivedSignal;

        for (uint32_t i = 0; i < numSymbols; ++i)
        {
            int transmittedSymbol = rand() % 4;
            std::complex<double> modulatedSignal = QpskModulate(transmittedSymbol);
            
            transmittedSignal.push_back(modulatedSignal);
            
            std::complex<double> channelOutput = ApplyMultipath(modulatedSignal, taps);
            
            std::complex<double> receivedSample = channelOutput + 
                std::complex<double>(noiseGenerator->GetValue(), noiseGenerator->GetValue());
            
            receivedSignal.push_back(receivedSample);
        }

        std::vector<std::complex<double>> equalizedSignal = Equalize(receivedSignal, taps);

        for (uint32_t i = 0; i < numSymbols; ++i)
        {
            int transmittedSymbol = QpskDemodulate(transmittedSignal[i]);
            int receivedSymbol = QpskDemodulate(equalizedSignal[i]);
            
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
                    << ", Theoretical BER (Multipath): " << theoreticalBerMultipath
		    << ", Effective SINR(Multipath): " << effectiveSnr
                    << ", Simulated BER (Multipath): " << simulatedBer);
    }

    return 0;
}
