#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store.h"
#include "ns3/netanim-module.h" // Include for NetAnim

using namespace ns3;

int main (int argc, char *argv[])
{
  // Enable logging
  LogComponentEnable ("LteHelper", LOG_LEVEL_INFO);

  // Create the LTE Helper
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();

  // Create nodes: eNodeB and UEs
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create (1);
  ueNodes.Create (2);

  // Install Mobility Model
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (enbNodes);
  mobility.Install (ueNodes);

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  // Attach all UEs to the eNodeB
  lteHelper->Attach (ueLteDevs, enbLteDevs.Get (0));

  // Install the IP stack on the UEs
  InternetStackHelper internet;
  internet.Install (ueNodes);

  // Assign IP addresses to UEs
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("7.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = ipv4.Assign (ueLteDevs);

  // Set the default gateway for the UEs
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting> (ueNode->GetObject<Ipv4> ()->GetRoutingProtocol ());
      ueStaticRouting->SetDefaultRoute ("7.0.0.1", 1);
    }

  // Schedule a test application
  uint16_t dlPort = 1234;
  ApplicationContainer clientApps;
  UdpClientHelper dlClient (ueIpIface.GetAddress (1), dlPort); // Corrected IP address index
  dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (10)));
  dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
  clientApps.Add (dlClient.Install (ueNodes.Get (0))); // Install client on UE 0

  clientApps.Start (Seconds (0.01));
  clientApps.Stop (Seconds (1.0));

  UdpServerHelper dlServer (dlPort);
  ApplicationContainer serverApps;
  serverApps.Add (dlServer.Install (ueNodes.Get (1))); // Install server on UE 1

  serverApps.Start (Seconds (0.01));
  serverApps.Stop (Seconds (1.0));

  // Enable tracing
  lteHelper->EnableTraces ();

  // Configure NetAnim
  AnimationInterface anim ("lte-animation.xml");

  Simulator::Stop (Seconds (1.0));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

