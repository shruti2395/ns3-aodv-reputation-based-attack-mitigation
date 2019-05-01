#include <iostream>
#include <cmath>
#include "ns3/aodv-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/yans-wifi-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("hospital-simulation");

class HospitalScenarioSimulation
{
public:
  HospitalScenarioSimulation ();

  bool Configure (int argc, char **argv);

  void Run ();

private:
  uint32_t port;

  // number of nodes in the grid layout
  uint32_t n_width;
  
  double simulationTime;

  bool enableBlackhole;

  bool enableTrustedRouting;

  // Number of malicious nodes
  uint32_t num_malicious_nodes;

  // Distance between nodes in meters
  double step;

  // Nodes in the system
  NodeContainer allNodes;

  // Containers for managing nodes
  NodeContainer maliciousNodes;
  NodeContainer goodNodes;

  // Devices used in the example
  NetDeviceContainer devices;
  // Interfaces used in the example
  Ipv4InterfaceContainer interfaces;

  std::string outputName;

private:
  // Create the nodes
  void CreateNodes ();
  // Create the devices
  void CreateDevices ();
  // Create the network
  void InstallInternetStack ();
  // Create the simulation applications
  void InstallApplications ();

  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  void ReceivePacket (Ptr<Socket> socket);
  void CheckThroughput ();
};

int main (int argc, char **argv)
{
  HospitalScenarioSimulation simulation;
  if (!simulation.Configure (argc, argv))
  {
    NS_FATAL_ERROR ("Configuration failed. Aborted.");
  }

  simulation.Run ();
  return 0;
}

HospitalScenarioSimulation::HospitalScenarioSimulation () :
  port (9),
  n_width (5),
  simulationTime (200),
  enableBlackhole (false),
  enableTrustedRouting (false),
  step (50),
  outputName ("hospital_scenario_simulation")
{
}

bool
HospitalScenarioSimulation::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy.
  // LogComponentEnable ("AodvRoutingProtocol", LOG_LEVEL_WARN);

  SeedManager::SetSeed (12345);
  CommandLine cmd;

  cmd.AddValue ("output", "Output name", outputName);
  cmd.AddValue ("width", "Width", n_width);
  cmd.AddValue ("enable-blackhole", "Enable blackhole attach node", enableBlackhole);
  cmd.AddValue ("enable-trust", "Enable trust routing model", enableTrustedRouting);

  cmd.Parse (argc, argv);
  return true;
}

void
HospitalScenarioSimulation::CreateNodes ()
{
  std::cout << "Creating nodes in a grid with " << n_width * n_width << " nodes" << std::endl;
  allNodes.Create (n_width * n_width);

  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (step),
                                 "DeltaY", DoubleValue (step),
                                 "GridWidth", UintegerValue (n_width),
                                 "LayoutType", StringValue ("RowFirst"));
  
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  if (enableBlackhole)
  {
    // Set up one malicious node.
    maliciousNodes.Add (allNodes.Get (n_width * (n_width + 1 - (n_width % 2)) / 2));
  }
}

void
HospitalScenarioSimulation::CreateDevices ()
{
  std::cout << "Creating Devices" << std::endl;
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  devices = wifi.Install (wifiPhy, wifiMac, allNodes);
}

void
HospitalScenarioSimulation::InstallInternetStack ()
{
  std::cout << "Installing Internet Stack" << std::endl;
  AodvHelper aodv;
  aodv.Set ("EnableHello", BooleanValue (true));
  aodv.Set ("EnableTrustRouting", BooleanValue (enableTrustedRouting));

  InternetStackHelper stack;
  stack.SetRoutingHelper (aodv);
  stack.Install (allNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);
}

void
HospitalScenarioSimulation::InstallApplications ()
{
  std::cout << "Installing Applications" << std::endl;
  OnOffHelper onOff ("ns3::UdpSocketFactory", Address ());
  onOff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onOff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  onOff.SetAttribute ("DataRate", DataRateValue (DataRate ("1Mbps")));

  for (uint32_t i = 0; i < n_width; i++)
  {
    uint32_t sourceId = n_width * i;
    uint32_t sinkId = sourceId + (n_width - 1);
    std::cout << "Source = " << sourceId << " and " << sinkId << std::endl;

    V4PingHelper ping (interfaces.GetAddress (sinkId));
    ping.SetAttribute ("Verbose", BooleanValue (true));

    ApplicationContainer p = ping.Install (allNodes.Get (sourceId));
    p.Start (Seconds (1));
    p.Stop (Seconds (simulationTime) - Seconds (0.001));

/**
    Ptr<Socket> sink = SetupPacketReceive (interfaces.GetAddress (sinkId), allNodes.Get (sinkId));
    AddressValue remoteAddress (InetSocketAddress (interfaces.GetAddress (sinkId), port));
    onOff.SetAttribute ("Remote", remoteAddress);

    ApplicationContainer temp = onOff.Install (allNodes.Get (sourceId));
    temp.Start (Seconds (0));
    temp.Stop (Seconds (simulationTime));
*/
  }

  for (uint32_t i = 0; i < maliciousNodes.GetN (); i++)
  {
    Ptr<Node> maliciousNode = maliciousNodes.Get (i);
    Ptr<aodv::RoutingProtocol> routingProtocol = maliciousNode->GetObject<aodv::RoutingProtocol> ();
    routingProtocol->SetBlackholeAttackEnable(true);
    routingProtocol->SetBlackholeAttackPacketDropPercentage(100);
  }
}

static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
    }
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}

void
HospitalScenarioSimulation::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      //bytesTotal += packet->GetSize ();
      //packetsReceived += 1;
      NS_LOG_WARN (PrintReceivedPacket (socket, packet, senderAddress));
      std::cout << PrintReceivedPacket (socket, packet, senderAddress) << std::endl;
    }
}

Ptr<Socket>
HospitalScenarioSimulation::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&HospitalScenarioSimulation::ReceivePacket, this));

  return sink;
}

void
HospitalScenarioSimulation::CheckThroughput ()
{
  /*
  double kbs = (bytesTotal * 8.0) / 1000;
  bytesTotal = 0;

  std::ofstream out (m_CSVfileName.c_str (), std::ios::app);

  out << (Simulator::Now ()).GetSeconds () << ","
      << kbs << ","
      << packetsReceived << ","
      << m_nSinks << ","
      << m_protocolName << ","
      << m_txp << ""
      << std::endl;

  out.close ();
  packetsReceived = 0;
  */
  std::cout << "Current time - " << (Simulator::Now ()).GetSeconds () << std::endl;
  Simulator::Schedule (Seconds (1.0), &HospitalScenarioSimulation::CheckThroughput, this);
}

void
HospitalScenarioSimulation::Run ()
{
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  CheckThroughput ();

  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll ();

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  AnimationInterface anim (outputName + "-animation.xml");

  anim.EnablePacketMetadata ();
  anim.EnableIpv4RouteTracking (outputName + "-routing-table.xml", Seconds (0), Seconds (simulationTime), Seconds (5));
  // anim.SetBackgroundImage("/home/shruti/Desktop/hospital.png", 0, 0, 1.0, 1.0, 0.75);

  // Set up node colors for NetAnim
  for (uint32_t i = 0; i < allNodes.GetN (); i++)
  {
    Ptr<Node> n = allNodes.Get (i);
    anim.UpdateNodeColor (n->GetId (), 0, 0, 255);
  }

  for (uint32_t i = 0; i < maliciousNodes.GetN (); i++)
  {
    Ptr<Node> m = maliciousNodes.Get (i);
    anim.UpdateNodeColor (m->GetId (), 255, 0, 0);
  }

  std::cout << "Starting simulation" << std::endl;
  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();

  std::cout << "Ending simulation" << std::endl;

  Simulator::Destroy ();

  flowmon->SerializeToXmlFile (outputName + ".flowmon", false, false);
}
