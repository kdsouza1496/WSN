/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include <iostream>
#include <fstream>

//#include "ns3/simulator-module.h"
//#include "ns3/node-module.h"
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
//#include "ns3/helper-module.h"
#include "ns3/rectangle.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include <stdlib.h>
#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/energy-module.h"
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TapWifiVirtualMachineExample");

// Code given below is for ns-2 trace integration.  
static void CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity
  // Prints position and velocities
  *os << foo<<Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}
/// Trace function for remaining energy at node.
void
RemainingEnergy (double oldValue, double remainingEnergy)  // Anirudh: where is this sort of function signature defined exactly ?
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
/* ANIRUDH: Tried an alternate method of tracing here which did not work. 
static void TotalEnergyConsumption (std::ostream *os, std::string context, Ptr<const DeviceEnergyModel> radioEnergyModel)

{
  double energyConsumed = radioEnergyModel->GetTotalEnergyConsumption();
  *os << Simulator::Now ()<<context<<":"<<" Energy consumed is " << energyConsumed << std::endl;
}
static void RemainingEnergy (std::ostream *os, std::string context, Ptr<BasicEnergySource> source)
{
 double remainingEnergy = source->GetRemainingEnergy();  
 *os << Simulator::Now () << context<<":"<<" Remaining Energy is " <<remainingEnergy<< std::endl;
}

*/


// global variables relevant to the simulation to set up the nodes

#define MAX_NODES 60

WifiHelper wifi;
NqosWifiMacHelper wifiMac;
YansWifiChannelHelper wifiChannel;
YansWifiPhyHelper wifiPhy;
NetDeviceContainer device;		// ns-3's representation of the device running on this node
TapBridgeHelper tapBridge;		// ns-3's representation of the tap bridge that this node connects to. 
MobilityHelper mobility;		// ns-3's representation of the node's mobility
BasicEnergySourceHelper basicSourceHelper; // energy source model
WifiRadioEnergyModelHelper radioEnergyHelper; // radio device energy consumption model 


int mobilityMode =-1; 
int terrainDimension=0;
int numberOfNodes=0;
NodeContainer nodes;
int enableDisconnectedClusters=-1;



void InitializeNetworkComponents() {
  
  // We're going to use 802.11p data channel ,so set up a wifi helper to reflect that.
  wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211p_SCH); // 802.11p radios , tweaking this to 802.11a to see if it works out 
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate3MbpsBW10MHz")); // want to see if the range improves now 

  // No reason for pesky access points, so we'll use an ad-hoc network.
  wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // Configure the physcial layer.
  wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy = YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());

  wifiPhy.Set ("TxPowerStart", DoubleValue(33));
  wifiPhy.Set ("TxPowerEnd", DoubleValue(33)); // changing the transmit power to refelct higher allowed transmit powers for 802.11p 

  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (0.1));

  // Configure radio device energy model, for now everything defaults to the default values, except the transmission current that is set here.
  // Later you can add new values for the reception current etc.
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));


}


static void InitializeNetworkedMobileNode(int nodeID) {
	
	std::cout<<"In Initialize Node on node ID  "<<nodeID<<std::endl;
	Ptr<Node> smartNodePointer;
	smartNodePointer=nodes.Get(nodeID);

	device = wifi.Install (wifiPhy, wifiMac, smartNodePointer); // connect node with phy and mac to create device
	if(mobilityMode==1) { // constant position mobility fixed nodes
	   // std::cout<<"Terrain side is  " << terrainDimension<<std::endl;  
   	     Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

	     float randomxPosition=((float)rand()/RAND_MAX)*(terrainDimension) ; // All nodes are randomly distributed in a square of side terrainDimension
	     float randomyPosition=((float)rand()/RAND_MAX)*(terrainDimension) ; // All nodes are randomly distributed in a square of side terrainDimension
	 
	     if(enableDisconnectedClusters)	 {				 // Allow one set of nodes to be totally partitioned from another for scale tests.
		if((float)nodeID >= (float)numberOfNodes /2 ) {
			std::cout<<"Node ID: "<<nodeID<<" is in a disconnected cluster"<<std::endl<<std::endl;
		        randomyPosition=randomyPosition+10000;			// make sure the two clusters can't interfere even by mistake by moving it to 1000,1000 
			randomxPosition=randomxPosition+10000;		
		}
	     }

	     positionAlloc->Add (Vector (randomxPosition,randomyPosition, 0.0));
	     //std::cout<<"Node : "<<i<<": Node position is x : "<< randomxPosition<<" y : "<<randomyPosition<<endl;
	     mobility.SetPositionAllocator (positionAlloc);
	     mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	     mobility.Install(smartNodePointer);
	     mobility.EnableAscii(std::cout,nodeID);
        }
	
	else if (mobilityMode ==2) {

             //mobility.Install(smartNodePointer); 
             //mobility.EnableAscii(std::cout,nodeID);
	}


	char tapString[10];
	sprintf(tapString,"tap%d",nodeID);
	tapBridge.SetAttribute ("Mode", StringValue ("UseLocal"));
	tapBridge.SetAttribute ("DeviceName", StringValue (tapString));
	tapBridge.Install (smartNodePointer, device.Get (0));   // devices is a container but it contains only one object in this case, this is a local variable, is it deleted and recreated every single time ? 


	// create energy source and device energy consumption models here.
        EnergySourceContainer energySource= basicSourceHelper.Install(smartNodePointer);	 
	// install device energy consumption model next
	DeviceEnergyModelContainer deviceModel = radioEnergyHelper.Install (device.Get(0),energySource); // connect the freshly created device to the freshly created energySource.

	 /** connect trace sources to log energy models. **/
	// energy source
       	Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (energySource.Get(0)); // singleton energy source 

       // basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy));
	// device energy model
        Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0); // This seems a convoluted way of getting the Device Energy Model 
	NS_ASSERT (basicRadioModelPtr != NULL);
       // basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption", MakeCallback (&TotalEnergy)); // no debug message is node is connected wrongly, another patch to fix. 
	
}



int main (int argc, char *argv[])
{
//  LogComponentEnable("TapBridge",LOG_ALL);
//  LogComponentEnable("RealtimeSimulatorImpl",LOG_ALL);
//  LogComponentEnable("YansWifisdfsdfsMACh",LOG_ALL);
// std::cout<<"Trying to get log components"<<std::endl;
  srand(time(NULL));
  CommandLine cmd;
  cmd.Parse (argc, argv);
  std::string traceFile; 
 
  int simulationTime=0;
  if(argc < 6) {
	std::cout<<"Usage: ./waf --run \"staggered Simulation <numberOfNodes> <terrainDimension> <mobilityMode : 1 for static, 2 for random way point, 3 for trace driven> <simualtionTime> <enableDisconnectedClusters> <traceFile for mobilityMode =3> "<<std::endl;
	exit(0);
  }
  else {
 	numberOfNodes=atoi(argv[1]);
	terrainDimension=atoi(argv[2]);
	mobilityMode=atoi(argv[3]);
	simulationTime=atoi(argv[4]);
	enableDisconnectedClusters=atoi(argv[5]);

	if(mobilityMode==3) {
	    if(argc < 7) {
		std::cout<<"Usage: ./waf --run \"staggered Simulation <numberOfNodes> <terrainDimension> <mobilityMode : 1 for static, 2 for random way point, 3 for trace driven> <simualtionTime> <enableDisconnectedClusters> <traceFile for mobilityMode =3> "<<std::endl;
		exit(0);
 	     }
	   else {
		traceFile=argv[6];		// traceFile for ns-2 traces. 
	    } 
	}
  }

  // log output from ns-2 mobility trace course changes 
  std::string logFile("/home/anirudh/Desktop/ns-3-dev/examples/tap/courseChange.log");
  std::ofstream os;
  os.open(logFile.c_str());
  // We are interacting with the outside, real, world.  This means we have to 
  // interact in real-time and therefore means we have to use the real-time
  // simulator and take the time to calculate checksums.
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  // Install the wireless devices onto our ghost nodes.
  // TODO: Fix the UseLocal for br
  nodes.Create (numberOfNodes);
  InitializeNetworkComponents();
  if (mobilityMode ==2) {
             std::cout<<"Terrain dimension is "<<terrainDimension<<std::endl;
             mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
              "MinX", DoubleValue (0.0),
              "MinY", DoubleValue (0.0),
              "DeltaX", DoubleValue (0.2),
              "DeltaY", DoubleValue (0.2),
              "GridWidth", UintegerValue (1),
              "LayoutType", StringValue ("RowFirst"));
             mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel","Bounds", RectangleValue (Rectangle (0, terrainDimension, 0, terrainDimension)));
	     mobility.Install(nodes);
//	     mobility.EnableAsciiAll(std::cout);// God knows what is happening here? Now suddenly it all seems to work even without enabling the ASCII all. God, help me out here. 
             std::cout<<"Initiated mobility \n";
  }
  else  if (mobilityMode ==3) { // feed in ns-2 traces. 
             std::cout<<"Terrain dimension is "<<terrainDimension<<std::endl;
             Ns2MobilityHelper ns2mobility=Ns2MobilityHelper(traceFile);
	     ns2mobility.Install(); // TODO: configure call back if required. 
	     Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
             MakeBoundCallback (&CourseChange, &os));
             std::cout<<"Initiated mobility from ns-2 trace file \n";
  }
  // enable tracing for energy consumption and remaining energy 
  //Config::Connect ("/NodeList/*/$ns3::WifiRadioEnergyModel/TotalEnergyConsumption",MakeBoundCallback(&TotalEnergyConsumption,&os));
//  Config::Connect ("/NodeList/*/$ns3::BasicEnergySource/RemainingEnergy",MakeBoundCallback(&RemainingEnergy,&os));
  int i=0;
  for(i=0;i<numberOfNodes;i++) {
 //       std::cout<<"Now inserting node"<<i<<std::endl;
         Simulator::Schedule(Seconds((i+1)*0.1),&InitializeNetworkedMobileNode,i);
  }
//  LogComponentEnable("TapBridge",LOG_ALL);
//  LogComponentEnable("FdReader",LOG_ALL);
//  LogComponentEnable("AdhocWifiMac",LOG_ALL);
  std::cout<<"Simulating for "<<simulationTime<<" seconds "<<std::endl;
  system("killall -s 10 python");	      // send a syncing signal to all processes. Not sure if this will work from within the VMs since there might be resource isolation, in other words do the processes running inside lxcs show up
					      // on the real system too ? Yeah, this works, that's great news. 
  Simulator::Stop (Seconds (simulationTime)); // enough time after start of simulation for simulation to run. So much time to just boot up 
  Simulator::Run();
  Simulator::Destroy ();
}
