#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/config-store-module.h"
#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/log.h"
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteMimoFadingExample");

std::ofstream logFile;

void CalculateUeDistances(NodeContainer ueNodes, Ptr<Node> enbNode) {
    Ptr<MobilityModel> enbMobility = enbNode->GetObject<MobilityModel>();
    Vector enbPosition = enbMobility->GetPosition();

    for (uint16_t i = 0; i < ueNodes.GetN(); ++i) {
        Ptr<MobilityModel> ueMobility = ueNodes.Get(i)->GetObject<MobilityModel>();
        Vector uePosition = ueMobility->GetPosition();
        double distance = CalculateDistance(enbPosition, uePosition);
        logFile << "UE " << i << " Distance: " << distance << " meters" << std::endl;
        NS_LOG_UNCOND("Distance of UE " << i << " to eNB: " << distance << " meters");
    }
}


// Function to log packet drops
void PacketDropCallback(std::string context, Ptr<const Packet> packet)
{
    std::cout << Simulator::Now().GetSeconds() << "s Packet Drop: " << packet->GetSize() << " bytes" << std::endl;

}

void MonitorErrorPerformance(Ptr<LteHelper> lteHelper, NodeContainer ueNodes) {
    for (uint16_t i = 0; i < ueNodes.GetN(); ++i) {
        Ptr<NetDevice> ueDevice = ueNodes.Get(i)->GetDevice(0);
        Ptr<LteUeNetDevice> lteUeDevice = DynamicCast<LteUeNetDevice>(ueDevice);
        lteUeDevice->GetPhy()->TraceConnect("PhyRxDrop", "", MakeCallback(&PacketDropCallback));
    }
}


// Function to log RSRP measurements
void RsrpSinrCallback(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti, double rsrp, double sinr)
{
    std::cout << Simulator::Now().GetSeconds() << "s IMSI " << imsi << " CellId " << cellId << " RNTI " << rnti
              << " RSRP " << rsrp << " dBm SINR " << sinr << " dB" << std::endl;
}




int main(int argc, char *argv[])
{
    // Set default parameters
    double simTime = 300.0; // Increased simulation time in seconds
    uint16_t numEnb = 2;  // Number of eNBs
    uint16_t numUePerEnb = 30; // Number of UEs per eNB
    double distance = 10000.0; // Adjusted distance between eNB and UE

    // Open log file
    logFile.open("simulation-log.txt");
    if (!logFile.is_open()) {
        NS_LOG_ERROR("Unable to open log file");
        return 1;
    }

    // Set the default LTE parameters
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(2)); // Reduced Tx Power
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(1)); // Reduced Tx Power
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(25)); // 5 MHz
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(25)); // 5 MHz
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(80)); // Increased SRS periodicity

    Config::SetDefault("ns3::Config::Log::NodeList/*/DeviceList/*/LteUePhy/ReportUeMeasurements", BooleanValue(true));

    // Enable RSRP tracing
    Config::Connect("/NodeList/*/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr", MakeCallback(&RsrpSinrCallback));

    // Enable packet drop tracing
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished", MakeCallback(&PacketDropCallback));

    LogComponentEnable("LteHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LteEnbRrc", LOG_LEVEL_ALL);
    LogComponentEnable("LteUeRrc", LOG_LEVEL_ALL);
    LogComponentEnable("LteEnbMac", LOG_LEVEL_ALL);
    LogComponentEnable("LteUeMac", LOG_LEVEL_ALL);

    // Create nodes: eNodeB and UE
    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create(numEnb);
    ueNodes.Create(numUePerEnb);

    // Install mobility model for eNodeB with height
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(enbNodes);
    enbMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                     "MinX", DoubleValue(0.0),
                                     "MinY", DoubleValue(0.0),
                                     "DeltaX", DoubleValue(5.0),
                                     "DeltaY", DoubleValue(5.0),
                                     "GridWidth", UintegerValue(1),
                                     "LayoutType", StringValue("RowFirst"));

    // Set eNodeB height
    enbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 30.0));

    // Install mobility model for UE with height
    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                    "X", DoubleValue(0.0),
                                    "Y", DoubleValue(0.0),
                                    "rho", DoubleValue(distance));
    ueMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                "Bounds", RectangleValue(Rectangle(-distance, distance, -distance, distance)),
                                "Distance", DoubleValue(100), // Adjusted movement distance
                                "Speed", StringValue("ns3::UniformRandomVariable[Min=0|Max=5]")); // Adjusted speed
    ueMobility.Install(ueNodes);

    // Set UE height
    for (uint16_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mobility = ueNodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        mobility->SetPosition(Vector(pos.x, pos.y, 1.5));
    }

    // Ensure all nodes have height greater than 0
    for (uint16_t i = 0; i < enbNodes.GetN(); ++i) {
        Ptr<MobilityModel> mobility = enbNodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        if (pos.z <= 0) {
            mobility->SetPosition(Vector(pos.x, pos.y, 30.0));
        }
    }

    for (uint16_t i = 0; i < ueNodes.GetN(); ++i) {
        Ptr<MobilityModel> mobility = ueNodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        if (pos.z <= 0) {
            mobility->SetPosition(Vector(pos.x, pos.y, 1.5));
        }
    }

    // Install LTE devices to the nodes
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    // Install the IP stack on the UEs
    InternetStackHelper internet;
    internet.Install(ueNodes);

    // Assign IP addresses to UEs
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

    // Attach a UE to an eNodeB
    for (uint16_t i = 0; i < numUePerEnb; i++)
    {
        lteHelper->AttachToClosestEnb(ueLteDevs.Get(i), enbLteDevs);
    }

    // Activate a data radio bearer
    enum EpsBearer::Qci qci = EpsBearer::NGBR_VIDEO_TCP_DEFAULT;
    EpsBearer bearer(qci);
    lteHelper->ActivateDedicatedEpsBearer(ueLteDevs, bearer, EpcTft::Default());

    // Set up the propagation loss model
    Ptr<OkumuraHataPropagationLossModel> propagationLossModel = CreateObject<OkumuraHataPropagationLossModel>();
    propagationLossModel->SetAttribute("Frequency", DoubleValue(2100000000)); // 2.1 GHz
    propagationLossModel->SetAttribute("Environment", EnumValue(UrbanEnvironment));
    propagationLossModel->SetAttribute("CitySize", EnumValue(LargeCity));    
    
    Ptr<SpectrumChannel> dlChannel = lteHelper->GetDownlinkSpectrumChannel();
    dlChannel->AddPropagationLossModel(propagationLossModel);

    // Add interference
    Ptr<PropagationLossModel> interferenceModel = CreateObject<RandomPropagationLossModel>();
    dlChannel->AddPropagationLossModel(interferenceModel);

    // Enable RSRP tracing
    Config::Connect("/NodeList/*/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr", MakeCallback(&RsrpSinrCallback));

    // Enable packet drop tracing
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished", MakeCallback(&PacketDropCallback));




    // Calculate distances between UEs and eNB
    CalculateUeDistances(ueNodes, enbNodes.Get(0));


    // Monitor error performance per UE
    MonitorErrorPerformance(lteHelper, ueNodes);

    // Enable tracing
    lteHelper->EnablePhyTraces();
    lteHelper->EnableMacTraces();
    lteHelper->EnableRlcTraces();
    lteHelper->EnablePdcpTraces();

    // Run the simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    // Close log file
    logFile.close();

    return 0;
}

