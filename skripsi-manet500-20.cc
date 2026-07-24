#include <fstream>
#include <iostream>
#include <iomanip>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"

#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("manet-routing-compare-skripsi");

class RoutingExperiment
{
public:
  RoutingExperiment ();
  void Run (int nSinks, double txp);
  void SaveMetrics (Ptr<FlowMonitor> flowmon, FlowMonitorHelper *flowmonHelper);
  std::string CommandSetup (int argc, char **argv);

private:
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  void ReceivePacket (Ptr<Socket> socket);

  uint32_t port;
  int m_nSinks;
  std::string m_protocolName;
  double m_txp;
  bool m_traceMobility;
  uint32_t m_protocol;
  uint32_t m_seed;
};

RoutingExperiment::RoutingExperiment ()
  : port (9),
    m_traceMobility (false),
    m_protocol (2), // Default routing protocol AODV
    m_seed (1423)      // Default seed
{
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
RoutingExperiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));

  return sink;
}

std::string
RoutingExperiment::CommandSetup (int argc, char **argv)
{
  CommandLine cmd (__FILE__);

  cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV", m_protocol);
  cmd.AddValue ("seed", "Random seed ", m_seed);
  cmd.Parse (argc, argv);
  return "";
}

int
main (int argc, char *argv[])
{
  RoutingExperiment experiment;
  experiment.CommandSetup (argc,argv);

  int nSinks = 1;
  double txp = 10;

  experiment.Run (nSinks, txp);
}

void
RoutingExperiment::Run (int nSinks, double txp)
{
  Packet::EnablePrinting ();
  m_nSinks = nSinks;
  m_txp = txp;

  // Set random seed untuk mengubah topo
  RngSeedManager::SetSeed (m_seed);
  RngSeedManager::SetRun (23351);

  int nWifis = 20;

  double TotalTime = 100.0;
  std::string rate ("250Kbps");
  std::string phyMode ("DsssRate11Mbps");
  std::string tr_name ("skripsi-manet-routing-compare");
  int nodeSpeed = 1; //in m/s
  int nodePause = 5; //in s
  m_protocolName = "protocol";

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("1024"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));

  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

  NodeContainer adhocNodes;
  adhocNodes.Create (nWifis);

  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel",
                                  "SystemLoss",    DoubleValue (2.0),
                                  "HeightAboveZ",  DoubleValue (1.0));
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));
  wifiPhy.EnablePcapAll (tr_name);

  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

  MobilityHelper mobilityAdhoc;
  int64_t streamIndex = 1; // used to get consistent mobility across scenarios

  // 1. Setup Mobility
  // Common Random Rectangle Allocator for Destination Picking (Movement)
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  Ptr<PositionAllocator> randomRectAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += randomRectAlloc->AssignStreams (streamIndex);

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";

  // A. Sources (Nodes 1-4): Start at Corners, then Random Waypoint
  MobilityHelper mobilitySources;
  mobilitySources.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                    "Speed", StringValue (ssSpeed.str ()),
                                    "Pause", StringValue (ssPause.str ()),
                                    "PositionAllocator", PointerValue (randomRectAlloc));

  Ptr<ListPositionAllocator> cornerAlloc = CreateObject<ListPositionAllocator> ();
  cornerAlloc->Add (Vector (140.0, 145.0, 0.0));  // Top-Left
  cornerAlloc->Add (Vector (374.0, 132.0, 0.0));  // Top-Right
  cornerAlloc->Add (Vector ( 148.0, 341.0, 0.0));  // Bottom-Left
  cornerAlloc->Add (Vector (345.0, 365.0, 0.0));  // Bottom-Right
  cornerAlloc->Add (Vector (250.0, 280.0, 0.0));  // Center (Source ke-5)
  mobilitySources.SetPositionAllocator (cornerAlloc);

  NodeContainer sourceNodes;
  for (int i = 1; i <= 5; i++) {
      sourceNodes.Add(adhocNodes.Get(i));
  }
  mobilitySources.Install (sourceNodes);
  streamIndex += mobilitySources.AssignStreams (sourceNodes, streamIndex);

  // B1. Relay pertama (Node 5): Start di (500,500) sebagai anchor viewport NetAnim
  Ptr<ListPositionAllocator> anchorAlloc = CreateObject<ListPositionAllocator> ();
  anchorAlloc->Add (Vector (500.0, 500.0, 0.0));
  MobilityHelper mobilityAnchor;
  mobilityAnchor.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                   "Speed", StringValue (ssSpeed.str ()),
                                   "Pause", StringValue (ssPause.str ()),
                                   "PositionAllocator", PointerValue (randomRectAlloc));
  mobilityAnchor.SetPositionAllocator (anchorAlloc);
  NodeContainer anchorNode;
  anchorNode.Add (adhocNodes.Get (6));
  mobilityAnchor.Install (anchorNode);
  streamIndex += mobilityAnchor.AssignStreams (anchorNode, streamIndex);

  // B2. Relay lainnya (Nodes 6+): Random Start, Random Waypoint
  MobilityHelper mobilityRelays;
  mobilityRelays.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                   "Speed", StringValue (ssSpeed.str ()),
                                   "Pause", StringValue (ssPause.str ()),
                                   "PositionAllocator", PointerValue (randomRectAlloc));
  mobilityRelays.SetPositionAllocator (randomRectAlloc);

  NodeContainer relayNodes;
  for (uint32_t i = 7; i < adhocNodes.GetN (); ++i) {
      relayNodes.Add(adhocNodes.Get(i));
  }
  mobilityRelays.Install (relayNodes);
  streamIndex += mobilityRelays.AssignStreams (relayNodes, streamIndex);

  // 2. Setup Static Mobility for Node 0 (Destination) at Center
  MobilityHelper mobilityStatic;
  mobilityStatic.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityStatic.Install (adhocNodes.Get (0));
  adhocNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (250.0, 410.0, 0.0));
  NS_UNUSED (streamIndex); // From this point, streamIndex is unused

  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;

  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  switch (m_protocol)
    {
    case 1:
      list.Add (olsr, 100);
      m_protocolName = "OLSR";
      break;
    case 2:
      list.Add (aodv, 100);
      m_protocolName = "AODV";
      break;
    case 3:
      list.Add (dsdv, 100);
      m_protocolName = "DSDV";
      break;

    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

    {
      internet.SetRoutingHelper (list);
      internet.Install (adhocNodes);
    }

  NS_LOG_INFO ("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);

  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  onoff1.SetAttribute ("MaxBytes", UintegerValue (2094850)); // 2MB limit per flow

  // 1. Setup Sink (Receiver) at Node 0
  Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (0), adhocNodes.Get (0));
  
  // 2. Setup  Sources (Sendefrs) sending to Node 0
  AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (0), port));
  onoff1.SetAttribute ("Remote", remoteAddress);

  for (int i = 1; i <= 5; i++)
    {
      Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      ApplicationContainer temp = onoff1.Install (adhocNodes.Get (i));
      temp.Start (Seconds (var->GetValue (1.0, 10.0)));
      temp.Stop (Seconds (TotalTime));
    }

  std::stringstream ss;
  ss << nWifis;
  std::string nodes = ss.str ();

  std::stringstream ss2;
  ss2 << nodeSpeed;
  std::string sNodeSpeed = ss2.str ();

  std::stringstream ss3;
  ss3 << nodePause;
  std::string sNodePause = ss3.str ();

  std::stringstream ss4;
  ss4 << rate;
  std::string sRate = ss4.str ();

  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll ();
  
  AnimationInterface anim ("skripsi-manet-routing-" + m_protocolName + "500x500-20node.xml");
  anim.EnableIpv4RouteTracking (tr_name + "_" + m_protocolName + "_500x500-20node_routes.xml", Seconds (0), Seconds (TotalTime), Seconds (5.0));
  // Visualization settings
  anim.EnablePacketMetadata (true); // Shows packet flow arrows and types (AODV, UDP, etc)
  anim.SetMaxPktsPerTraceFile (0x7fffffff); // Naikkan batas max paket agar simulasi bisa dilihat sampai selesai
  
  // Coloring Nodes for easier identification
  for (uint32_t i = 0; i < adhocNodes.GetN (); ++i)
    {
       if (i == 0) // Node 0 is Destination
         {
            anim.UpdateNodeColor (adhocNodes.Get (i), 0, 255, 0); // Green
            anim.UpdateNodeDescription (adhocNodes.Get (i), "Posko"); 
            anim.UpdateNodeSize (i, 6.0, 6.0); 
         }
       else if (i >= 1 && i <= 5) // Nodes are Sources
         {
            anim.UpdateNodeColor (adhocNodes.Get (i), 255, 0, 0); // Red
            anim.UpdateNodeDescription (adhocNodes.Get (i), "Relawan Source");
            anim.UpdateNodeSize (i, 5.0, 5.0);
         }
       else
         {
            anim.UpdateNodeColor (adhocNodes.Get (i), 0, 0, 255); // Blue for others
            anim.UpdateNodeDescription (adhocNodes.Get (i), "Relawan Relay");
         }
    }

  NS_LOG_INFO ("Run Simulation.");

  Simulator::Stop (Seconds (TotalTime));
  Simulator::Run ();

  SaveMetrics (flowmon, &flowmonHelper);

  flowmon->SerializeToXmlFile ((tr_name + "-" + m_protocolName + "500x500-20node.flowmon").c_str(), false, false);

  Simulator::Destroy ();
}

void
RoutingExperiment::SaveMetrics (Ptr<FlowMonitor> flowmon, FlowMonitorHelper *flowmonHelper)
{
  flowmon->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmonHelper->GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = flowmon->GetFlowStats ();

  double totalThroughput = 0.0;
  double totalDelayMs = 0.0;      // sum of per-flow mean delays (sesuai FlowMonitor)
  uint32_t totalPacketsTx = 0;
  uint32_t totalPacketsRx = 0;
  uint32_t totalLostPackets = 0;
  uint32_t flowCountThroughput = 0;  // flow yg punya simTime > 0
  uint32_t flowCountDelay = 0;       // flow yg punya rxPackets > 0

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      if (i->second.txPackets == 0)
        continue;
      //filter out flows not going to the sink
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      if (t.destinationPort != port)
        {
          continue; 
        }

      totalPacketsTx += i->second.txPackets;
      totalPacketsRx += i->second.rxPackets;
      totalLostPackets += (i->second.txPackets - i->second.rxPackets);

      if (i->second.rxPackets > 0)
        {
          // Per-flow mean delay = delaySum / rxPackets  (sama persis dengan "Mean delay" di FlowMonitor viewer)
          double perFlowMeanDelayMs = i->second.delaySum.GetSeconds () * 1000.0 / i->second.rxPackets;
          totalDelayMs += perFlowMeanDelayMs;
          flowCountDelay++;

          // Throughput per flow = rxBytes * 8 / simTime (sesuai "Rx bitrate" FlowMonitor viewer)
          double simTime = i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstRxPacket.GetSeconds ();
          if (simTime > 0)
            {
              double throughput = i->second.rxBytes * 8.0 / 1000.0 / simTime; // kbps
              totalThroughput += throughput;
              flowCountThroughput++;
            }
        }
    }

  double avgThroughput = flowCountThroughput > 0 ? totalThroughput / flowCountThroughput : 0.0;
  double avgDelayMs = flowCountDelay > 0 ? totalDelayMs / flowCountDelay : 0.0; // simple avg per-flow mean
  double pdr = totalPacketsTx > 0 ? (double) totalPacketsRx / totalPacketsTx * 100.0 : 0.0;
  double packetLoss = 100.0 - pdr; // Packet Loss dalam %

  // Print to Console
  std::cout << std::fixed << std::setprecision (2);
  std::cout << "\n+------------------------------+" << std::endl;
  std::cout << "|     SIMULATION RESULTS       |" << std::endl;
  std::cout << "+------------------------------+" << std::endl;
  std::cout << "  Protocol   : " << m_protocolName      << std::endl;
  std::cout << "  Nodes      : 20" << std::endl;
  std::cout << "  Area       : 500 x 500 m"             << std::endl;
  std::cout << "+------------------------------+" << std::endl;
  std::cout << "  Throughput : " << avgThroughput << " kbps" << std::endl;
  std::cout << "  PDR        : " << pdr           << " %"    << std::endl;
  std::cout << "  Packet Loss: " << packetLoss    << " %"    << std::endl;
  std::cout << "  E2E Delay  : " << avgDelayMs    << " ms"   << std::endl;
  std::cout << "+------------------------------+" << std::endl;
  std::cout << "  Tx Packets : " << totalPacketsTx  << std::endl;
  std::cout << "  Rx Packets : " << totalPacketsRx  << std::endl;
  std::cout << "  Lost       : " << totalLostPackets << std::endl;
  std::cout << "+------------------------------+\n" << std::endl;

  // Save to CSV
  std::ofstream out ("manet-routing-results-500x500-20node.csv", std::ios::app);
  // Write header if file is empty
  out.seekp (0, std::ios::end);
  if (out.tellp () == 0)
    {
      out << "Protocol,Throughput_kbps,PDR_percent,PacketLoss_percent,E2E_Delay_ms,TotalTx,TotalRx" << std::endl;
    }

  out << m_protocolName << "," 
      << avgThroughput << "," 
      << pdr << "," 
      << packetLoss << ","
      << avgDelayMs << ","
      << totalPacketsTx << ","
      << totalPacketsRx 
      << std::endl;
  out.close ();
}
