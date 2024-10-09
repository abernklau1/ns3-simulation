#pragma once

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"

#include "entity.hpp"
#include "networking/network_metrics.hpp"

class NetworkSim {
    ns3::NodeContainer wifiNodes;

    ns3::YansWifiPhyHelper wifiPhy;
    ns3::WifiMacHelper wifiMac;
    ns3::WifiHelper wifi;

    ns3::MobilityHelper mobility;

    ns3::Ipv4AddressHelper address;

public:
    NetworkSim();

    void add_node(Entity entity);

    void start_sim();

    double get_metric(Metric metric, Entity& a, Entity& b);
};