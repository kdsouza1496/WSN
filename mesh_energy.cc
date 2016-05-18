  /* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- 

    3  * Copyright (c) 2008,2009 IITP RAS
    4  *
    5  * This program is free software; you can redistribute it and/or modify
    6  * it under the terms of the GNU General Public License version 2 as
    7  * published by the Free Software Foundation;
    8  *
    9  * This program is distributed in the hope that it will be useful,
   10  * but WITHOUT ANY WARRANTY; without even the implied warranty of
   11  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   12  * GNU General Public License for more details.
   13  *
   14  * You should have received a copy of the GNU General Public License
   15  * along with this program; if not, write to the Free Software
   16  * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   17  *
   18  * Author: Kirill Andreev <andreev@iitp.ru>
   19  *
   20  *
   21  * By default this script creates m_xSize * m_ySize square grid topology with
   22  * IEEE802.11s stack installed at each node with peering management
   23  * and HWMP protocol.
   24  * The side of the square cell is defined by m_step parameter.
   25  * When topology is created, UDP ping is installed to opposite corners
   26  * by diagonals. packet size of the UDP ping and interval between two
   27  * successive packets is configurable.
   28  * 
   29  *  m_xSize * step
   30  *  |<--------->|
   31  *   step
   32  *  |<--->|
   33  *  * --- * --- * <---Ping sink  _
   34  *  | \   |   / |                ^
   35  *  |   \ | /   |                |
   36  *  * --- * --- * m_ySize * step |
   37  *  |   / | \   |                |
   38  *  | /   |   \ |                |
   39  *  * --- * --- *                _
   40  *  ^ Ping source
   41  *
   42  *  See also MeshTest::Configure to read more about configurable
   43  *  parameters.
   44  
   45 
   46 
   */

    #include "ns3/core-module.h"
    #include "ns3/internet-module.h"
    #include "ns3/network-module.h"
    #include "ns3/applications-module.h"
    #include "ns3/config-store-module.h"
    #include "ns3/energy-module.h"
    #include "ns3/internet-module.h"
    #include "ns3/mesh-helper.h"
    #include "ns3/wifi-module.h"
    #include "ns3/mesh-module.h"
    #include "ns3/mobility-module.h"
    #include "ns3/mesh-helper.h"
    #include "ns3/netanim-module.h"
    

    #include <iostream>
    #include <sstream>
    #include <fstream>
    #include <vector>
    #include <string>

    
    using namespace ns3;
    
    NS_LOG_COMPONENT_DEFINE ("MeshEnergyScript");
    
    class MeshTest
    {
    public:
      MeshTest ();
      void Configure (int argc, char ** argv);
      int Run ();
      
     private:
      int       m_xSize;
      int       m_ySize;
      double    m_step;
      double    m_randomStart;
      double    m_totalTime;
      Time    m_packetInterval;
      uint16_t  m_packetSize;
      uint32_t  m_nIfaces;
      bool      m_chan;
      bool      m_pcap;
      int       m_npackets;
      std::string m_stack;
      std::string m_root;
      NodeContainer nodes;
      NetDeviceContainer meshDevices;
      //Addresses of interfaces:
      Ipv4InterfaceContainer interfaces;
      // MeshHelper. Report is not static methods
      MeshHelper mesh;
    private:
     void CreateNodes ();
     //void EnergyModelSpecs();
     Ptr<Socket> SetupEnergyNodes();
     void InstallInternetStack ();
     void InstallApplication ();
     void Report ();
   };
   MeshTest::MeshTest () :

     m_xSize (3),
     m_ySize (3),
     m_step (100.0),
     m_randomStart (0.1),
     m_totalTime (100.0),
     m_packetInterval (0.1),
     m_packetSize (1024),
     m_nIfaces (1),
     m_chan (true),
     m_pcap (true),
     m_npackets (200),
     m_stack ("ns3::Dot11sStack"),
     m_root ("ff:ff:ff:ff:ff:ff")
     
        {
   }   
   
  static inline std::string
  PrintReceivedPacket (Address& from)
  {
     InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (from);

     std::ostringstream oss;
     oss << "--\nReceived one packet! Socket: " << iaddr.GetIpv4 ()
      << " port: " << iaddr.GetPort ()
      << " at time = " << Simulator::Now ().GetSeconds ()
      << "\n--";

     return oss.str ();
  }

  void
  ReceivePacket (Ptr<Socket> socket)
  {
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      if (packet->GetSize () > 0)
        {
          NS_LOG_UNCOND (PrintReceivedPacket (from));
        }
    }
  }

  static void
  GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, Ptr<Node> n,
                 uint32_t pktCount, Time pktInterval)
  {
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, n,
                           pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
  }

/// Trace function for remaining energy at node.
  void
  RemainingEnergy (double oldValue, double remainingEnergy)
  {
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Current remaining energy = " << remainingEnergy << "J");
  }

/// Trace function for total energy consumption at node.
  void
  TotalEnergy (double oldValue, double totalEnergy)
  {
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Total energy consumed by radio = " << totalEnergy << "J");
  }

   void
   MeshTest::Configure (int argc, char *argv[])
   {
     
     CommandLine cmd;
     cmd.AddValue ("x-size", "Number of nodes in a row grid. [6]", m_xSize);
     cmd.AddValue ("y-size", "Number of rows in a grid. [6]", m_ySize);
     cmd.AddValue ("step",   "Size of edge in our grid, meters. [100 m]", m_step);
     /*
      * As soon as starting node means that it sends a beacon,
      * simultaneous start is not good.
      */
     cmd.AddValue ("start",  "Maximum random start delay, seconds. [0.1 s]", m_randomStart);
     cmd.AddValue ("time",  "Simulation time, seconds [100 s]", m_totalTime);
     cmd.AddValue ("packet-interval",  "Interval between packets in UDP ping, seconds [0.001 s]", m_packetInterval);
     cmd.AddValue ("packet-size",  "Size of packets in UDP ping", m_packetSize);
     cmd.AddValue ("interfaces", "Number of radio interfaces used by each mesh point. [1]", m_nIfaces);
     cmd.AddValue ("channels",   "Use different frequency channels for different interfaces. [0]", m_chan);
     cmd.AddValue ("pcap",   "Enable PCAP traces on interfaces. [0]", m_pcap);
     cmd.AddValue ("numPackets", "Total number of packets to send", m_npackets);
     cmd.AddValue ("stack",  "Type of protocol stack. ns3::Dot11sStack by default", m_stack);
     cmd.AddValue ("root", "Mac address of root mesh point in HWMP", m_root);
     
   
     cmd.Parse (argc, argv);
     NS_LOG_DEBUG ("Grid:" << m_xSize << "*" << m_ySize);
     NS_LOG_DEBUG ("Simulation time: " << m_totalTime << " s");

     if (true)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

   }

   void
   MeshTest::CreateNodes ()
   { 
     /*
      * Create m_ySize*m_xSize stations to form a grid topology
      */
     double offset = 50;
     double Prss = -70;
     nodes.Create (m_ySize*m_xSize);
     // Configure YansWifiChannel
     YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
     YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
     
     wifiPhy.Set ("RxGain", DoubleValue (-5));
     wifiPhy.Set ("TxGain", DoubleValue (offset + Prss));
     wifiPhy.Set ("CcaMode1Threshold", DoubleValue (0.0));

     wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
     wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");

     wifiPhy.SetChannel (wifiChannel.Create ());
     wifiPhy.EnablePcapAll(std::string("cl-")); 
     /*
      * Create mesh helper and set stack installer to it
      * Stack installer creates all needed protocols and install them to
      * mesh point device
      */
     mesh = MeshHelper::Default ();
     if (!Mac48Address (m_root.c_str ()).IsBroadcast ())
       {
         mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue (Mac48Address (m_root.c_str ())));
       }
     else
       {
         //If root is not set, we do not use "Root" attribute, because it
         //is specified only for 11s
         mesh.SetStackInstaller (m_stack);
       }
     if (m_chan)
       {
         mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
       }
     else
       {
         mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
       }
     mesh.SetMacType ("RandomStart", TimeValue (Seconds (m_randomStart)));
     // Set number of interfaces - default is single-interface mesh point
     mesh.SetNumberOfInterfaces (m_nIfaces);
     // Install protocols and return container if MeshPointDevices
     meshDevices = mesh.Install (wifiPhy, nodes);
     // Setup mobility - static grid topology
     MobilityHelper mobility;
     mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                    "MinX", DoubleValue (0.0),
                                    "MinY", DoubleValue (0.0),
                                    "DeltaX", DoubleValue (m_step),
                                    "DeltaY", DoubleValue (m_step),
                                    "GridWidth", UintegerValue (m_xSize),
                                    "LayoutType", StringValue ("RowFirst"));
     mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
     mobility.Install (nodes);

     if (m_pcap)
       wifiPhy.EnablePcapAll (std::string ("mp-"));
   }

   

    
   Ptr<Socket> 
   MeshTest::SetupEnergyNodes()
   {
 

    BasicEnergySourceHelper basicSourceHelper;
    // configure energy source
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (0.4));
    // install source
    EnergySourceContainer sources = basicSourceHelper.Install (nodes);
    /* device energy model */
    WifiRadioEnergyModelHelper radioEnergyHelper;
    // configure radio energy model
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.35));
    // install device model
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (meshDevices, sources);
       

    /*
    for(uint i=0;i<meshDevices.GetN(); i++)
    { 

      Ptr<NetDevice> netdevicepointer = meshDevices.Get(i); 
      DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (netdevicepointer, sources);
       

    }
    */
   

    

   TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
   Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get (0), tid);  // node 1, receiver
   InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
   recvSink->Bind (local);
   recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

   Ptr<Socket> source = Socket::CreateSocket (nodes.Get (1), tid);    // node 0, sender
   InetSocketAddress remote = InetSocketAddress (Ipv4Address::GetBroadcast (), 80);
   source->SetAllowBroadcast (true);
   source->Connect (remote);

  /** connect trace sources **/
  /***************************************************************************/
  // all sources are connected to node 1
  // energy source
   Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (0));
   basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy));
   // device energy model
   Ptr<DeviceEnergyModel> basicRadioModelPtr =
    basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (1);
   NS_ASSERT (basicRadioModelPtr != NULL);
   basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption", MakeCallback (&TotalEnergy));
   return source;
  
   }

   void
   MeshTest::InstallInternetStack ()
   {

     InternetStackHelper internetStack;
     internetStack.Install (nodes);
     Ipv4AddressHelper address;
     address.SetBase ("10.1.1.0", "255.255.255.0");
     interfaces = address.Assign (meshDevices);
   }


   /*void
   MeshTest::InstallApplication ()
   {
     UdpEchoServerHelper echoServer (9);
     ApplicationContainer serverApps = echoServer.Install (nodes.Get (0));
     serverApps.Start (Seconds (0.0));
     serverApps.Stop (Seconds (m_totalTime));
     UdpEchoClientHelper echoClient (interfaces.GetAddress (0), 9);
     echoClient.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)(m_totalTime*(1/m_packetInterval))));
     echoClient.SetAttribute ("Interval", TimeValue (Seconds (m_packetInterval)));
     echoClient.SetAttribute ("PacketSize", UintegerValue (m_packetSize));
     ApplicationContainer clientApps = echoClient.Install (nodes.Get (m_xSize*m_ySize-1));
     clientApps.Start (Seconds (0.0));
     clientApps.Stop (Seconds (m_totalTime));
   }*/

   int
   MeshTest::Run ()
   {
     CreateNodes ();
     //EnergyModelSpecs();
     InstallInternetStack ();
     Ptr<Socket> source = SetupEnergyNodes();
     // InstallApplication ();
     /*Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this);
     Simulator::Stop (Seconds (m_totalTime));*/
      
     Simulator::Schedule (Seconds (0.0), &GenerateTraffic, source, m_packetSize,
                       nodes.Get (1), m_npackets, m_packetInterval);

     Simulator::Stop (Seconds (10.0));

     
     AnimationInterface anim ("cl.xml");
     anim.EnablePacketMetadata (true);

     Simulator::Run ();
     //Report();
     Simulator::Destroy ();
     return 0;
   }

   void
   MeshTest::Report ()
   {
     unsigned n (0);
     for (NetDeviceContainer::Iterator i = meshDevices.Begin (); i != meshDevices.End (); ++i, ++n)
       {
         std::ostringstream os;
         os << "mp-report-" << n << ".xml";
         std::cerr << "Printing mesh point device #" << n << " diagnostics to " << os.str () << "\n";
         std::ofstream of;
         of.open (os.str ().c_str ());
         if (!of.is_open ())
          {
           std::cerr << "Error: Can't open file " << os.str () << "\n";
           return;
          }
         mesh.Report (*i, of);
         of.close ();
       }
   }

   int
   main (int argc, char *argv[])
  {
   MeshTest t; 
   t.Configure (argc, argv);
   return t.Run ();
 
  }