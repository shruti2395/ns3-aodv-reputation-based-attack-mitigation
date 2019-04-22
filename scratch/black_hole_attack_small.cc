#include <iostream>
#include <cmath>
#include "ns3/aodv-module.h"
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

/**
 * \ingroup aodv-examples
 * \ingroup examples
 * \brief Test script.
 * 
 * This script creates 2-dimensional grid topology and then ping last node in row 1 from the first one:
 * 
 * [10.0.0.1] <-- step --> [10.0.0.2] <-- step --> [10.0.0.3] <-- step --> [10.0.0.4] <-- step --> [10.0.0.5]
 *
 *                                     [10.0.0.6] <-- step --> [10.0.0.7]
 *
 * ping 10.0.0.5 from 10.0.0.1
 * 
 * 10.0.0.3 is set as blackhole for 0-40secs of simulation
 * Total Simulation time = 100sec
 */
class BlackholeAttackSmallExample 
{
public:
  BlackholeAttackSmallExample ();
  /**
   * \brief Configure script parameters
   * \param argc is the command line argument count
   * \param argv is the command line arguments
   * \return true on successful configuration
  */
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /**
   * Report results
   * \param os the output stream
   */
  void Report (std::ostream & os);

private:

  // parameters
  /// Number of nodes
  uint32_t size;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;

  uint32_t port;
  uint32_t bytesTotal;
  uint32_t packetsReceived;

  std::string m_CSVfileName;

  // network
  /// nodes used in the example
  NodeContainer nodes;

  /// devices used in the example
  NetDeviceContainer devices;
  /// interfaces used in the example
  Ipv4InterfaceContainer interfaces;

private:
  /// Create the nodes
  void CreateNodes ();
  /// Create the devices
  void CreateDevices ();
  /// Create the network
  void InstallInternetStack ();
  /// Create the simulation applications
  void InstallApplications ();

  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);

  void ReceivePacket (Ptr<Socket> socket);
  
  void CheckThroughput ();
};

int main (int argc, char **argv)
{
  BlackholeAttackSmallExample test;
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();
  test.Report (std::cout);
  return 0;
}

//-----------------------------------------------------------------------------
BlackholeAttackSmallExample::BlackholeAttackSmallExample () :
  size (7),
  step (100),
  totalTime (100),
  pcap (true),
  printRoutes (true),
  port (9),
  m_CSVfileName ("blackhole_attack_small.output.csv")
{
}

bool
BlackholeAttackSmallExample::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy
  LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_WARN);

  SeedManager::SetSeed (12345);
  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);
  cmd.AddValue ("CSVfileName", "The name of the CSV output file name", m_CSVfileName);

  cmd.Parse (argc, argv);
  return true;
}

void
BlackholeAttackSmallExample::Report (std::ostream & os)
{
}

void
BlackholeAttackSmallExample::Run ()
{
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  AnimationInterface anim ("blackhole-small-routing-animation.xml");

  anim.EnablePacketMetadata();
  anim.EnableIpv4RouteTracking("blackhole-small-routing-table.xml", Seconds (0), Seconds (totalTime), Seconds (5));
  anim.SetBackgroundImage("/home/shruti/Desktop/project-ns3-aodv/hos.png", 0, 0, 1.0, 1.0, 0.75);

  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll ();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  // CheckThroughput();

  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();
  Simulator::Destroy ();

  flowmon->SerializeToXmlFile ("blackhole-small.flowmon", false, false);
}

void
BlackholeAttackSmallExample::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      // NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}

void
BlackholeAttackSmallExample::CheckThroughput ()
{
  double kbs = (bytesTotal * 8.0) / 1000;
  bytesTotal = 0;

  std::ofstream out (m_CSVfileName.c_str (), std::ios::app);

  out << (Simulator::Now ()).GetSeconds () << ","
      << kbs << ","
      << packetsReceived << ","
      << std::endl;

  out.close ();
  packetsReceived = 0;
  Simulator::Schedule (Seconds (1.0), &BlackholeAttackSmallExample::CheckThroughput, this);
}

Ptr<Socket>
BlackholeAttackSmallExample::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&BlackholeAttackSmallExample::ReceivePacket, this));

  return sink;
}

void
BlackholeAttackSmallExample::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  NodeContainer nodesInFirstRow;
  NodeContainer nodesInSecondRow;

  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
    {
      std::ostringstream os;
      os << "node-" << i;
      Names::Add (os.str (), nodes.Get (i));

      if (i < size - 2)
      {
        nodesInFirstRow.Add(nodes.Get(i));
      }
      else
      {
        nodesInSecondRow.Add(nodes.Get(i));
      }
    }
  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (step),
                                 "DeltaY", DoubleValue (0),
                                 "GridWidth", UintegerValue (size),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodesInFirstRow);

  // assign fixed separate position for the last node
  Ptr<ListPositionAllocator> positionAllocator = CreateObject<ListPositionAllocator> ();
  positionAllocator->Add(Vector(step * (size - 4) / 2, step / 2, 0));
  positionAllocator->Add(Vector(step * (size - 2) / 2, step / 2, 0));

  mobility.SetPositionAllocator(positionAllocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodesInSecondRow);
}

void
BlackholeAttackSmallExample::CreateDevices ()
{
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  devices = wifi.Install (wifiPhy, wifiMac, nodes); 

  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("aodv"));
    }
}

void
BlackholeAttackSmallExample::InstallInternetStack ()
{
  AodvHelper aodv;
  aodv.Set("EnableHello", BooleanValue (false));
  // aodv.Set("GratuitousReply", BooleanValue (false));

  // you can configure AODV attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (aodv); // has effect on the next Install ()
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("blackhole_attach_small_aodv.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (10), routingStream);
      aodv.PrintRoutingTableAllAt (Seconds (50), routingStream);
      aodv.PrintRoutingTableAllAt (Seconds (80), routingStream);
      // aodv.PrintRoutingTableEvery(Seconds (1), nodes.Get (1), routingStream, Time::Unit::S);
    }
}

void
BlackholeAttackSmallExample::InstallApplications ()
{
  V4PingHelper ping (interfaces.GetAddress (size - 3));
  ping.SetAttribute ("Verbose", BooleanValue (true));

  ApplicationContainer p = ping.Install (nodes.Get (0));
  p.Start (Seconds (2));
  p.Stop (Seconds (totalTime) - Seconds (0.001));

/*
  Ptr<Socket> sink = SetupPacketReceive (interfaces.GetAddress (size - 3), nodes.Get (size - 3));
  AddressValue sinkRemoteAddress (InetSocketAddress (interfaces.GetAddress (size - 3), port));

  OnOffHelper onoff1 ("ns3::UdpSocketFactory", Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  onoff1.SetAttribute ("Remote", sinkRemoteAddress);

  ApplicationContainer onOffApp = onoff1.Install (nodes.Get (0));
  onOffApp.Start (Seconds (2));
  onOffApp.Stop (Seconds (totalTime) - Seconds (0.001));
*/

  Ptr<Node> maliciousNode = nodes.Get ((size - 2) / 2);
  Ptr<aodv::RoutingProtocol> routingProtocol = maliciousNode->GetObject<aodv::RoutingProtocol> ();
  std::cout << routingProtocol->GetInstanceTypeId () << std::endl;
  routingProtocol->SetBlackholeAttackEnable(true);
  routingProtocol->SetBlackholeAttackPacketDropPercentage(100);

  Simulator::Schedule (Seconds (40), &aodv::RoutingProtocol::SetBlackholeAttackEnable, routingProtocol, false);
  Simulator::Schedule (Seconds (40), &aodv::RoutingProtocol::SetBlackholeAttackPacketDropPercentage, routingProtocol, 0);

  // move node away - disabled for now
  // Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
  //Simulator::Schedule (Seconds (totalTime/3), &MobilityModel::SetPosition, mob, Vector (1e5, 1e5, 1e5));
}
