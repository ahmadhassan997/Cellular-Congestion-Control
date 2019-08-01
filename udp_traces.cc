/* 
    -*- Collect UDP traces by saturating the uplink and downlink of UE 
        in Handover situations.
*/

#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/lte-global-pathloss-database.h"
#include <math.h>

using namespace ns3;

double simTime = 100; /* Simulation time for each aplication */
static double schedulerTimer = 0; /* timer to schedule position tracking */


int isPedestrian = 1; /* -1 : no mobility and no fading traces , 0 : vehicular trace , 1 = pedestrain trace */
uint16_t isFading = 1;
uint16_t traceTime = 100; /* trace file period, in seconds */
std::string P_TRACE_FILE = "/home/ah/Desktop/projects/fading_traces/EPA_3kmh_100_dl.fad";
std::string V_TRACE_FILE = "/home/ah/Desktop/projects/fading_traces/fading_trace_EVA_60kmph.fad";
std::string traceFile = P_TRACE_FILE; /* location of trace file */

/*
  -*-  Simulation Parameters 
*/

uint16_t numberOfUes = 1;
uint16_t numberOfEnbs = 5;
uint16_t numBearersPerUe = 1;
double distanceBetweenEnbs = 300.0;
uint16_t radioUlBandwidth = 25;
uint16_t radioDlBandwidth = 25;
double s1UplinkLinkDelay = 0.015; /* S1u between eNB and SGW in s */
std::string s1UplinkLinkRate = "1Gb/s";
// double movingBound = 50000;
// static uint16_t ueAllocationType = 0; /* 0 : Fixed Position , 1 : Random Position , 2 : Grid Position */

/*
    -*- Application Parameters
*/

uint16_t ulPort = 10000;
uint16_t dlPort = 20000;
uint32_t packetSize = 1450;
// double samplingInterval = 0.005; /* Invoke  getUdpStats() every x seconds*/
uint16_t PUT_SAMPLING_INTERVAL = 50; /*sample a UDP throughput for each x pkts*/
double startTime = 0.0;
std::string dataRate = "150Mb/s";
LogLevel logLevel = (LogLevel) (LOG_PREFIX_TIME | LOG_PREFIX_NODE| LOG_PREFIX_FUNC | LOG_LEVEL_DEBUG);
bool serverClientApplication = true;

/*
    -*- Handover Parameters
 */

bool isAutoHandover = true;
uint8_t a2ServingCellThreshold = 34;  /* if current cell signal strength smaller than this, consider HO (default 30) [0-34] as in Section 9.1.7 of [TS36133] */
uint8_t a4NeighbourCellOffset = 1;  /* if neighbour cell signal strength is larger than the source cell by this amount, allow HO. (default 1). */
uint32_t hoType = 1;  /* 1. a2a4 HO, 2. a3 HO. */
double x2PathDelay = 20; /* X2 forwarding path delay in ms */
std::string X2PathRate = "1Gb/s" ; //X2 path data rate.
double ueMovingSpeed = 10; // 3Km/h for Pedestrian and 60Km/h for vehicular

/*  
    -*- Flow Monitor Parameters 
*/

Ptr<ns3::FlowMonitor> monitor;
FlowMonitorHelper flowHelper;
Ptr<ns3::Ipv4FlowClassifier> classifier;

// std::map<Ipv4Address, double> last_tx_time;
// std::map<Ipv4Address, double> last_rx_time ;
// std::map<Ipv4Address, double> last_tx_bytes ;
// std::map<Ipv4Address, double> last_rx_bytes ;
// std::map<Ipv4Address, double> tcp_delay ;
// std::map<Ipv4Address, double> last_delay_sum ;
// std::map<Ipv4Address, double> last_rx_pkts ;
// std::map<Ipv4Address, double> last_tx_pkts ;
// std::map<Ipv4Address, uint16_t> init_map ;
// std::map<Ipv4Address, double> last_put_sampling_time;
// /**sending flows stats***/
// std::map<Ipv4Address, double> meanTxRate_send;
// std::map<Ipv4Address, double> meanRxRate_send;
// std::map<Ipv4Address, double> meanTcpDelay_send;
// std::map<Ipv4Address, uint64_t> numOfLostPackets_send;
// std::map<Ipv4Address, uint64_t> numOfTxPacket_send;

/* 
    -*- ASCII output files configuration
*/

static std::string DIR = "/home/ah/Desktop/projects/tracedata/";
static std::string macro = DIR+"macro_output.dat";
static std::string putSend;
static std::string debugger = DIR+"debugger.dat";
static std::string courseChange = DIR+"courseChange.dat";
static std::string overall = "overall.out";
static std::string positionTracking = DIR+"positionTracking.dat";

/*
    -*- Wrappers
*/

static AsciiTraceHelper asciiTraceHelper;
Ptr<OutputStreamWrapper> put_send_wp;
Ptr<OutputStreamWrapper> macro_wp;
Ptr<OutputStreamWrapper> debugger_wp;
Ptr<OutputStreamWrapper> ue_positions_wp;
Ptr<OutputStreamWrapper> overall_wp;
Ptr<OutputStreamWrapper> position_tracking_wp;

/*
    -*- Initialize all functions to be used later on
*/

// static void getUdpPut();
void EnableLogComponents();
void SetDefaultConfigs();
// void CommandLineParameters(int argc , char* argv[]);
void InstallMobility(NodeContainer ueNodes , NodeContainer enbNodes);
void InstallFading(Ptr<LteHelper> lteHelper);
void EnablePositionTracking(NetDeviceContainer enbLteDevs, NodeContainer ueNodes);
static void PosTracking(Ptr<OutputStreamWrapper> positionTrackingWp , Ptr<const MobilityModel> model);
static void CourseChange (Ptr<OutputStreamWrapper> ue_positions_wp, Ptr<const MobilityModel> model);
static void InitWrappers();

void ConfigStoreOutput(std::string);
void ConfigStoreInput(std::string inFile);

/*
    -*- Handover Monitoring
*/

Ptr<OutputStreamWrapper> handoverWrapper = asciiTraceHelper.CreateFileStream(DIR + "handover.dat");

void NotifyConnectionEstablishedUe (std::string context, 
                               uint64_t imsi, 
                               uint16_t cellid, 
                               uint16_t rnti)
{
  *handoverWrapper->GetStream() << Simulator::Now().GetSeconds() 
	    << " : " << context 
            << " UE IMSI " << imsi 
            << ": connected to CellId " << cellid 
            << " with RNTI " << rnti 
            << std::endl;
}

void NotifyHandoverStartUe (std::string context, 
                       uint64_t imsi, 
                       uint16_t cellid, 
                       uint16_t rnti, 
                       uint16_t targetCellId)
{
   *handoverWrapper->GetStream() << Simulator::Now().GetSeconds() 
	    << " : " << context 
            << " UE IMSI " << imsi 
            << ": previously connected to CellId " << cellid 
            << " with RNTI " << rnti 
            << ", doing handover to CellId " << targetCellId 
            << std::endl;
}

void NotifyHandoverEndOkUe (std::string context, 
                       uint64_t imsi, 
                       uint16_t cellid, 
                       uint16_t rnti)
{
  *handoverWrapper->GetStream() << Simulator::Now().GetSeconds() 
	    << " : " << context 
            << " UE IMSI " << imsi 
            << ": successful handover to CellId " << cellid 
            << " with RNTI " << rnti 
            << std::endl;
}

void NotifyConnectionEstablishedEnb (std::string context, 
                                uint64_t imsi, 
                       uint16_t cellid,
                       uint16_t rnti)
{
   *handoverWrapper->GetStream() << Simulator::Now().GetSeconds()
            << " : " << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  *handoverWrapper->GetStream() << Simulator::Now().GetSeconds()
            << " : " << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  *handoverWrapper->GetStream() << Simulator::Now().GetSeconds()
            << " : " << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}


/** 
 * Simulation Script for a X2-Based Handover.
 */

NS_LOG_COMPONENT_DEFINE("X2HandoverUdp");

int main(int argc, char *argv[])
{
    /**
     * Changing Default Attributes to make them reasonable for handover Scenario.
     * Always configure default attributes before processing the command line arguments.
     */
    SetDefaultConfigs();
    ConfigStoreInput("LTE.in");
    InitWrappers();
    EnableLogComponents();

    /* Initialize LTE and EPC Helpers for whole simulation */
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
    if(hoType == 1)
    {
        lteHelper->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm");
		lteHelper->SetHandoverAlgorithmAttribute ("ServingCellThreshold",UintegerValue (a2ServingCellThreshold));
		lteHelper->SetHandoverAlgorithmAttribute ("NeighbourCellOffset",UintegerValue (a4NeighbourCellOffset));
    }
    else if(hoType == 2)
    {
        lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
		lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis", DoubleValue (3.0));
		lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (256)));
    }
    else
    {
           *debugger_wp->GetStream() << "Something wrong with HO type setup\n";
    }

    /* Set Packet Gateway Default Configuration */
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    epcHelper->SetAttribute("S1uLinkDataRate", DataRateValue (DataRate (s1UplinkLinkRate)));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue (Seconds (s1UplinkLinkDelay)));
    epcHelper->SetAttribute("S1uLinkMtu", UintegerValue (1500));

    
    /* Create one udpServer applications and Install it on a Remote Host */
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHost);

    /* Create the Internet */
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate (s1UplinkLinkRate)));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (s1UplinkLinkDelay)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    /* Install Routing of remote host towards the LTE Network */
    // Interface 0 is localhost, 1 is p2p device.
    Ipv4StaticRoutingHelper Ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = Ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address("7.0.0.0") , Ipv4Mask("255.0.0.0") , 1);
    
    NodeContainer ueNodes;
    NodeContainer enbNodes;
    if(isAutoHandover)
    {
        numberOfEnbs = 5;
        simTime = (numberOfEnbs - 1) * distanceBetweenEnbs/ueMovingSpeed + 100;
    }
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    /* Install Mobility and Fading Model */
    InstallMobility(ueNodes , enbNodes);
    InstallFading(lteHelper);

    /* Install LTE devices in eNB and UEs */
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    /* Enable Position Tracking */
    EnablePositionTracking(enbLteDevs , ueNodes);

    /* Install IP Stack on UEs */
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    
    /* Assign IP address to UEs */
    for(uint16_t i = 0; i < ueNodes.GetN(); i++)
    {
        Ptr<Node> ueNode = ueNodes.Get(i);
        /* Set default gateway for the UE */
        Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress() , 1);
    }

    /* Attach all UEs to the first eNodeB */
    for(uint16_t i = 0; i < numberOfUes ; i++)
    {
        lteHelper->Attach(ueLteDevs.Get(i) , enbLteDevs.Get(0));
    }

    NS_LOG_INFO("Setting Up Applications");
    // Randomize start times to avoid simulations artifacts (e.g. buffer overflows due to packet transmissions)
    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
    startTimeSeconds->SetAttribute("Min" , DoubleValue(0.7));
    startTimeSeconds->SetAttribute("Max" , DoubleValue(0.9));

    /* Install and Start applications on UEs and Remote Host */
    for(uint16_t u = 0; u < numberOfUes; u++)
    {
        Ptr<Node> ue = ueNodes.Get(u);
        for(uint16_t b = 0 ; b < numBearersPerUe ; b++)
        {
            dlPort++;
            ulPort++;
            ApplicationContainer clientApps;
            ApplicationContainer serverApps;
            LogComponentEnable("Queue",logLevel);
            PUT_SAMPLING_INTERVAL = PUT_SAMPLING_INTERVAL*5;
            if(serverClientApplication)
            {
                /* Server Sink Application on Uplink  and Downlink*/
                NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
                UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
                clientApps.Add (dlClientHelper.Install (remoteHost));
                PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", 
                                                InetSocketAddress (Ipv4Address::GetAny (), dlPort));
                serverApps.Add (dlPacketSinkHelper.Install (ue));
                
                NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
                UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
                clientApps.Add (ulClientHelper.Install (ue));
                PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", 
                                                InetSocketAddress (Ipv4Address::GetAny (), ulPort));
                serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
            }
            else
            {
                /* OnOff Sink Application on Uplink and Downlink */
                PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), dlPort));
                serverApps.Add(sink.Install(ueNodes.Get(u)));

                OnOffHelper onOffHelper("ns3::UdpSocketFactory", Address ( InetSocketAddress(ueIpIfaces.GetAddress(u), dlPort) ));
                onOffHelper.SetConstantRate( DataRate(dataRate), packetSize );
                clientApps.Add(onOffHelper.Install(remoteHost));

                PacketSinkHelper ul_sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), ulPort));
                serverApps.Add(ul_sink.Install(remoteHost));

                OnOffHelper ul_onOffHelper("ns3::UdpSocketFactory", Address ( InetSocketAddress(remoteHostAddr, ulPort) ));
                ul_onOffHelper.SetConstantRate( DataRate(dataRate), packetSize );
                clientApps.Add(ul_onOffHelper.Install(ueNodes.Get(u)));   
            }           

            /* Set the EPC Traffic Flow Template  */
            Ptr<EpcTft> tft = Create<EpcTft> ();
            EpcTft::PacketFilter dlpf;
            dlpf.localPortStart = dlPort;
            dlpf.localPortEnd = dlPort;
            tft->Add (dlpf); 
            EpcTft::PacketFilter ulpf;
            ulpf.remotePortStart = ulPort;
            ulpf.remotePortEnd = ulPort;
            tft->Add (ulpf);

            /* Active the dedicated bearer */
            EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
            lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

            /* Start the applications */
            Time startTime = Seconds (startTimeSeconds->GetValue ());
            serverApps.Start (startTime);
            clientApps.Start (startTime);
        }
    }

    /* Add X2 Interface for UE Handovers */
    lteHelper->AddX2Interface(enbNodes);

    /* Configure Flow Monitor Helper and enable Traces*/
    monitor = flowHelper.Install(ueNodes);
    monitor = flowHelper.Install(remoteHost);
    monitor = flowHelper.GetMonitor();
    p2ph.EnablePcapAll(DIR+"udptraces");
    lteHelper->EnableMacTraces ();
    lteHelper->EnableRlcTraces ();
    lteHelper->EnablePdcpTraces ();
    Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
    rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (0.05)));
    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (0.05)));

    /* Connect custom trace sinks for RRC connection establishment and handover notification*/
    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback (&NotifyConnectionEstablishedEnb));
    Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                    MakeCallback (&NotifyConnectionEstablishedUe));
    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback (&NotifyHandoverStartEnb));
    Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                    MakeCallback (&NotifyHandoverStartUe));
    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback (&NotifyHandoverEndOkEnb));
    Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                    MakeCallback (&NotifyHandoverEndOkUe));

    /* Configure Output Store */
    ConfigStoreOutput("lte.out");

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    /* Check for lost packets */
    monitor->CheckForLostPackets();
    Ptr<ns3::Ipv4FlowClassifier> classifier = DynamicCast<ns3::Ipv4FlowClassifier> (flowHelper.GetClassifier());
    std::map <FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin(); iter != stats.end(); ++iter){
        ns3::Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);

        *macro_wp->GetStream()  << "***Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << ":" << t.sourcePort 
            << " Dst Addr " << t.destinationAddress << ":" << t.destinationPort  << std::endl
        << "Tx Packets " << iter->second.txPackets << std::endl
        << "Rx Packets " << iter->second.rxPackets << std::endl
        << "Lost packets " << iter->second.lostPackets << std::endl
        << "Lost ratio " << double (iter->second.lostPackets)/(iter->second.lostPackets+iter->second.rxPackets) << std::endl;
        double ONEBIL=1000000000;
        if (iter->second.rxPackets > 1){
            *macro_wp->GetStream()   << "Average delay received " 
        << iter->second.delaySum/iter->second.rxPackets/1000000 << std::endl
            << "Mean received bitrate " 
        << 8*iter->second.rxBytes/(iter->second.timeLastRxPacket-iter->second.timeFirstRxPacket)*ONEBIL/(1024) 
        << std::endl
            << "Mean transmitted bitrate " 
        << 8*iter->second.txBytes/(iter->second.timeLastTxPacket-iter->second.timeFirstTxPacket)*ONEBIL/(1024) 
        << std::endl;
        }	
    }


    Simulator::Destroy();
    return 0;
}


/*
    -*- Get UDP stats on Downlink and Uplink
 */


/*
    -*- Configure files for input and output
 */

void ConfigStoreOutput(std::string out_f)
{
    /****ConfigStore setting****/
    Config::SetDefault("ns3::ConfigStore::Filename", StringValue(out_f));
    Config::SetDefault("ns3::ConfigStore::FileFormat", StringValue("RawText"));
    Config::SetDefault("ns3::ConfigStore::Mode", StringValue("Save"));
    ConfigStore outputConfig;
    outputConfig.ConfigureDefaults();
    outputConfig.ConfigureAttributes();
}

void ConfigStoreInput(std::string in_f)
{
    Config::SetDefault("ns3::ConfigStore::Filename", StringValue(in_f));
    Config::SetDefault("ns3::ConfigStore::FileFormat", StringValue("RawText"));
    Config::SetDefault("ns3::ConfigStore::Mode", StringValue("Load"));
    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();
    inputConfig.ConfigureAttributes();
}

/*
    -*- Configure default parameters for the simulation.
 */


void SetDefaultConfigs()
{
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MicroSeconds(10000)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue(1000000));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue(false));
//   Config::SetDefault ("ns3::DropTailQueue::MaxPackets", UintegerValue(10000));
//   Config::SetDefault ("ns3::DropTailQueue::MaxBytes", UintegerValue(999999));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(150000));
  Config::SetDefault ("ns3::PointToPointEpcHelper::X2LinkDelay", TimeValue (MilliSeconds(x2PathDelay)));
  Config::SetDefault ("ns3::PointToPointEpcHelper::S1uLinkDelay", TimeValue (MilliSeconds(s1UplinkLinkDelay)));

  if (isFading==1)
  	Config::SetDefault("ns3::LteHelper::FadingModel", StringValue("ns3::TraceFadingLossModel"));
}

/*
    -*- Install mobility on UE Nodes and ENBs
 */

void InstallMobility(NodeContainer ueNodes , NodeContainer enbNodes)
{
    /* Install Mobility Model for ENBs */
    MobilityHelper enbMobility;
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject <ListPositionAllocator> ();
    for (uint16_t i = 0; i < numberOfEnbs ; i++)
    {
        Vector enbPosition(distanceBetweenEnbs *i , 0 ,0);
        enbPositionAlloc->Add(enbPosition);
    }
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    /* Install Mobility Model for UE Nodes */
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 100, 0));
}

/*
    -*- Install fading traces on radio link
 */

void InstallFading(Ptr<LteHelper> lteHelper)
{
    if( isPedestrian == 1 && isFading == 1) traceFile = P_TRACE_FILE;
    else if(isPedestrian == 0 && isFading == 1) traceFile = V_TRACE_FILE;
    
    /* Enable trace fading if pedestrian or vehicular */
    if(isPedestrian >= 0 && isFading == 1)
    {
        *debugger_wp->GetStream() << "Trace Fading Enabled ....\n";
        Config::SetDefault("ns3::LteHelper::FadingModel" , StringValue("ns3::TraceFadingLossModel"));
        lteHelper->SetFadingModel("ns3::TraceFadingLossModel");
        lteHelper->SetFadingModelAttribute("TraceLength" , TimeValue(Seconds(traceTime)));
        lteHelper->SetFadingModelAttribute("SamplesNum",UintegerValue(traceTime*1000));  /*1sample/1ms*/
		lteHelper->SetFadingModelAttribute("WindowSize",TimeValue(Seconds(0.5)));
		lteHelper->SetFadingModelAttribute("RbNum",UintegerValue(radioDlBandwidth));
		lteHelper->SetFadingModelAttribute("TraceFilename", StringValue(traceFile));
		      NS_LOG_UNCOND("Trace fading:\n"
				  << "========================"
				  << "\nisPedestrian= " << isPedestrian
				  << "\ntraceTime= " << traceTime
				  << "\nradioDlBandwidth= " << radioDlBandwidth
				  << "\ntraceFile= " << traceFile);
    }
    else 
    {
        *debugger_wp->GetStream() << "Trace fading disabled....\n";
    }
}

/*
    -*- Enable Positon Tracking for the UE nodes
 */

void EnablePositionTracking(NetDeviceContainer enbLteDevs, NodeContainer ueNodes)
{
    NetDeviceContainer::Iterator enbLteDevIt = enbLteDevs.Begin ();
    Vector enbPosition = (*enbLteDevIt)->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
    Ptr<MobilityModel> ue_mobility_model = ueNodes.Get(0)->GetObject<MobilityModel>();
    double x = ue_mobility_model->GetPosition().x;
    double y = ue_mobility_model->GetPosition().y;
    *debugger_wp->GetStream() << "eNB(x,y)= " << enbPosition.x << ", " << enbPosition.y << std::endl;
    *debugger_wp->GetStream() << "UE (x,y)= " << x << ", " << y << " d= " << sqrt(x*x+y*y) << std::endl;

    ue_mobility_model->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&CourseChange, ue_positions_wp));

    /* Schedule Tracking */
    Simulator::Schedule(Seconds(1), &PosTracking, position_tracking_wp, ue_mobility_model);
}

static void CourseChange (Ptr<OutputStreamWrapper> ue_positions_wp, Ptr<const MobilityModel> model)
{
        Vector position = model->GetPosition();
        Vector vel = model->GetVelocity();
        *ue_positions_wp->GetStream() << Simulator::Now().GetSeconds() << " (x,y)= " << position.x << " , " 
                                   << position.y
                                   << " d= " << sqrt(position.x*position.x+position.y*position.y) 
                                   << " v= "
                                   << sqrt (vel.x*vel.x + vel.y*vel.y)
                                   << std::endl;
}

static void PosTracking (Ptr<OutputStreamWrapper> position_tracking_wp, Ptr<const MobilityModel> model)
{
        Vector position = model->GetPosition();
        Vector vel = model->GetVelocity();
        *position_tracking_wp->GetStream() << Simulator::Now().GetSeconds() << " (x,y)= " << position.x << " , "
                                   << position.y
                                   << " d= " << sqrt(position.x*position.x+position.y*position.y)
                                   << " v= "
                                   << sqrt (vel.x*vel.x + vel.y*vel.y)
                                   << std::endl;
        while (schedulerTimer <= simTime)
        {
                schedulerTimer += 3;
                Simulator::Schedule(Seconds(schedulerTimer), &PosTracking, position_tracking_wp, model);
        }
}

/*
    -*- Initialize Wrappers for the simulation
 */

static void InitWrappers()
{
    /* Initialize Wrappers */
    putSend = DIR + "udp-put.dat";
    macro = DIR + "macro_udp.dat";

    /* Create Files For Wrappers */
    debugger_wp = asciiTraceHelper.CreateFileStream(debugger);
    overall_wp = asciiTraceHelper.CreateFileStream(overall, std::ios::app);
    ue_positions_wp = asciiTraceHelper.CreateFileStream(courseChange);
    position_tracking_wp = asciiTraceHelper.CreateFileStream(positionTracking);
    macro_wp = asciiTraceHelper.CreateFileStream(macro);
    put_send_wp = asciiTraceHelper.CreateFileStream(putSend);

    *ue_positions_wp->GetStream() << "========================\n";
    *position_tracking_wp->GetStream() << "========================\n";
    *put_send_wp->GetStream() << "#DestinationIp\t"
                  << "Time\t"
                  << "Send Tcp throughput\t"
                  << "Send Tcp delay\t"
                  << "Number of Lost Pkts\t"
                  << "Number of Tx Pkts\t"
                  << "ErrorUlTx\t"
                  << "ErrorDlTx\t"
                  << "HarqUlTx\t"
                  << "HarqDlTx\n";

}

/*
    -*- Enable Log Components
 */


void EnableLogComponents()
{
    LogComponentEnable ("LteHelper", logLevel);
    LogComponentEnable ("EpcHelper", logLevel);
  
    // LogComponentEnable ("EpcEnbApplication", logLevel);
    // LogComponentEnable ("EpcX2", logLevel);
    // LogComponentEnable ("EpcSgwPgwApplication", logLevel);

    // LogComponentEnable ("LteRlcUm", logLevel);
    // LogComponentEnable ("LteRlcAm", logLevel);
    // LogComponentEnable ("NscTcpSocketImpl",LOG_LEVEL_DEBUG);

    // LogComponentEnable ("LteEnbRrc", logLevel);
    // LogComponentEnable ("LteEnbNetDevice", logLevel);
    LogComponentEnable ("LteUeRrc", logLevel);
}