#include "network_sim.hpp"
#include <ns3/internet-stack-helper.h>

NS_LOG_COMPONENT_DEFINE("NetworkSim");

NetworkSim::NetworkSim() {
    // We always want simulations to run in real-time
    ns3::GlobalValue::Bind(
        "SimulatorImplementationType", 
        ns3::StringValue("ns3::RealtimeSimulatorImpl")
    );

    ns3::YansWifiChannelHelper channel = ns3::YansWifiChannelHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // The below FixedRssLossModel will cause the rss to be fixed regardless
    // of the distance between the two stations, and the transmit power
    channel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", ns3::DoubleValue(-70.0));


    wifiPhy.SetChannel(channel.Create());
    wifiMac.SetType("ns3::AdhocWifiMac");
    

    // Mobility setup
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", ns3::DoubleValue(0.0),
                                  "MinY", ns3::DoubleValue(0.0),
                                  "DeltaX", ns3::DoubleValue(5.0),
                                  "DeltaY", ns3::DoubleValue(10.0),
                                  "GridWidth", ns3::UintegerValue(3),
                                  "LayoutType", ns3::StringValue("RowFirst"));

    mobility.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel", "Bounds",
        ns3::RectangleValue(ns3::Rectangle(-50, 50, -50, 50))
    );

    // IPv4 setup
    address.SetBase("10.0.1.0", "255.255.255.0");

    wifi.SetStandard(ns3::WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 ns3::StringValue("DsssRate1Mbps"),
                                 "ControlMode",
                                 ns3::StringValue("DsssRate1Mbps"));
}

void NetworkSim::add_node(Entity entity) {
    std::cout << "Adding new node" << std::endl;

    // Add a node to the network
    wifiNodes.Create(1);

    auto thisNodeRef = wifiNodes.Get(wifiNodes.GetN() - 1);

    auto apDevices = wifi.Install(wifiPhy, wifiMac, thisNodeRef);
    mobility.Install(thisNodeRef);

    ns3::InternetStackHelper internet;
    internet.Install(thisNodeRef);

    // Assign IP addresses
    auto thisInterface = address.Assign(apDevices);

    // For now, install an echo server on each node
    ns3::UdpEchoServerHelper echoServer(8080);
    ns3::ApplicationContainer serverApps = echoServer.Install(thisNodeRef);

    ns3::UdpEchoClientHelper echoClient(ns3::Ipv4Address("10.0.1.1"), 8080);
    echoClient.SetAttribute("MaxPackets", ns3::UintegerValue(10));
    echoClient.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", ns3::UintegerValue(1024));
    ns3::ApplicationContainer clientApps = echoClient.Install(thisNodeRef);

    serverApps.Start(ns3::Seconds(1.0));
    clientApps.Start(ns3::Seconds(2.0));

    serverApps.Stop(ns3::Seconds(2.0));
    clientApps.Stop(ns3::Seconds(2.0));
}

void NetworkSim::start_sim() {
    wifiPhy.EnablePcapAll("realtime-udp-echo", false);

    std::cout << "Starting simulation" << std::endl;

    ns3::Simulator::Stop(ns3::Seconds(2.0));
    ns3::Simulator::Run();

    std::cout << "Simulation complete. See pcap files in Wireshark." << std::endl;

    ns3::Simulator::Destroy();
}

double NetworkSim::get_metric(Metric metric, Entity& a, Entity& b) {
    return 0.0;
}