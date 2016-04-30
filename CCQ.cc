/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
 
 // Ruiling Zhang, April 2016
 // Use for loop to achieve test different node amount, distance and RSS power
 // save the test data to .dat file 
 // Then use GNUplot to plot figures
 // Based on last year student Hedi Krishna's code
 
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/applications-module.h"

#include "ns3/oh-buildings-propagation-loss-model.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <limits>
#include <iomanip> 

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleInfra");

int main (int argc, char *argv[])
{
  double simulationTime = 10; //seconds
  //==========================
  time_t  timev;
  time(&timev);
  RngSeedManager::SetSeed(timev);
  RngSeedManager::SetRun (7);
  //==========================

  std::string phyMode ("DsssRate11Mbps");
  std::string rtsCts ("150");
  double ApStaDistance;
for(ApStaDistance=40.0;ApStaDistance<90.0;ApStaDistance++)
{
  //std::string DistanceString = std::to_string(ApStaDistance);
  std::ostringstream sstream;
  sstream << ApStaDistance;
  std::string DistanceString = sstream.str(); 
  //std::string DistanceTail ("]");
  
  std::string Distance ="ns3::ConstantRandomVariable[Constant=" + DistanceString + "]";
  //std::cout << Distance << '\n';
  bool verbose = false;
  //double rss = -80; //dbm use for fixdss model

  uint32_t n=40; 
//for(n=1;n<30;n++)
//{
  uint32_t payloadSize = 1024;

  CommandLine cmd;

  cmd.AddValue ("n", "number of nodes", n);
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("rtsCts", "RTS/CTS threshold", rtsCts);
  cmd.AddValue ("Distance", "APSta distance", Distance);
  cmd.Parse (argc, argv);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 'rtsCts' bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue (rtsCts));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", 
                      StringValue (phyMode));

  NodeContainer apContainer;
  NodeContainer staContainer;
  apContainer.Create (1);
  staContainer.Create (n);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (0) ); 
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO); 

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // The below FixedRssLossModel will cause the rss to be fixed regardless
  // of the distance between the two stations, and the transmit power
  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");  //change the different channel path loss function
  wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue(rss));  //change the different channel path loss function

  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  // Setup the rest of the upper mac
  Ssid ssid = Ssid ("wifi-default");
  // setup sta.
  wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDevice = wifi.Install (wifiPhy, wifiMac, staContainer);
  //NetDeviceContainer devices = staDevice;

  // setup ap.
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));
  NetDeviceContainer apDevice = wifi.Install (wifiPhy, wifiMac, apContainer.Get (0));
  //devices.Add (apDevice);  


  // Note that with FixedRssLossModel, the positions below are not 
  // used for received signal strength. 
  MobilityHelper mobility, mobilityAp;

  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomDiscPositionAllocator");
  pos.Set ("Rho",StringValue (Distance));// setting radius is easier to control distance between AP and Sta
  Ptr<PositionAllocator> positionAlloc = pos.Create ()->GetObject<PositionAllocator> ();

  Ptr<ListPositionAllocator> positionAllocAp = CreateObject<ListPositionAllocator> ();
  positionAllocAp->Add (Vector (0.0, 0.0, 0.0));
  mobilityAp.SetPositionAllocator (positionAllocAp);
  mobilityAp.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityAp.Install (apContainer);

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (staContainer);

  InternetStackHelper internet;
  internet.Install (apContainer);
  internet.Install (staContainer);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer iap = ipv4.Assign (apDevice);
  Ipv4InterfaceContainer i = ipv4.Assign (staDevice);

  //port number, given in array
  uint16_t port[n];

  ApplicationContainer apps[n], sinkApp[n];

	for( uint16_t  a = 0; a < n; a = a + 1 )
   	{
	  port[a]=8000+a;
          Address apLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), port[a]));
          PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", apLocalAddress);
          sinkApp[a] = packetSinkHelper.Install (apContainer.Get (0));

          sinkApp[a].Start (Seconds (0.0));
          sinkApp[a].Stop (Seconds (simulationTime+1));
	

          OnOffHelper onoff ("ns3::TcpSocketFactory",Ipv4Address::GetAny ());

          onoff.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=10]"));
          onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
          onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          onoff.SetAttribute ("DataRate", StringValue ("0.5Mbps")); //bit/s set a small datarate for sta in order to observe the relationship between channel capacity and receiver's rate capacity

          AddressValue remoteAddress (InetSocketAddress (iap.GetAddress (0), port[a]));
          onoff.SetAttribute ("Remote", remoteAddress);

          apps[a].Add (onoff.Install (staContainer.Get (a)));

          apps[a].Start (Seconds (1.0));
          apps[a].Stop (Seconds (simulationTime+1));
	}


  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  wifiPhy.EnablePcap ("wifitcp", apDevice);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  Simulator::Stop (Seconds (simulationTime+1));
  Simulator::Run ();

  monitor->CheckForLostPackets ();
  double tot=0;
  double totsq=0;
  //double variance=0;
  std::cout << std::fixed;
  
  double throughput[2*n]; //for every node, there are 2 flows in TCP
  int psent=0;
  int preceived=0;

  Vector pos2;

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
	      throughput[i->first] = i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds())/1024/1024;
		  
		if (t.destinationAddress=="10.1.1.1")
		 {
		  int ncount=(i->first)-1;
		  Ptr<MobilityModel> mob = staContainer.Get(ncount)->GetObject<MobilityModel>();
		  pos2 = mob->GetPosition ();

		  
		  tot=tot+throughput[i->first];
		  totsq=totsq+pow(throughput[i->first],2);
		  psent=psent+i->second.txBytes;
		  preceived=preceived+i->second.rxBytes;
		 }
     }

  std::cout << ApStaDistance << "\t" << tot/n <<"\n";
  Simulator::Destroy ();
}
  return 0;
}
