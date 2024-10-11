#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/wifi-module.h"

#include "../EtxMatrix/EtxMatrix.h"

using namespace ns3;

#ifndef ADHOC_NETWORK_H
  #define ADHOC_NETWORK_H

class AdhocNetwork
{
  public:
    AdhocNetwork( uint32_t numNodes, WifiStandard wifiStandard, std::string macType, std::string ipBase, std::string positionAllocator, double communicationRange );

    void setup( );

    void findNeighbors( );

    void setupApplications( );

    NodeContainer getNodes( ) const { return m_nodes; }

    NetDeviceContainer getDevices( ) const { return m_devices; }

    Ipv4InterfaceContainer getInterfaces( ) const { return m_interfaces; }

    std::vector<std::vector<Node>> getNeighbors( ) const { return m_neighbors; }

    void scheduleFindNeighbors( double interval );

    void initializeRandomPositions( double xMin, double xMax, double yMin, double yMax );

    double CalculateETX( uint32_t nodeId, uint32_t neighborId );

  private:
    uint32_t m_numNodes;
    WifiStandard m_wifiStandard;
    std::string m_macType;
    std::string m_ipBase;
    std::vector<std::vector<Node>> m_neighbors;
    std::vector<Vector> m_positions;

    NodeContainer m_nodes;
    NetDeviceContainer m_devices;
    Ipv4InterfaceContainer m_interfaces;

    std::string m_positionAllocator;
    double m_communicationRange;

    std::vector<std::vector<ns3::ApplicationContainer>> m_onOffApps;
    std::vector<std::vector<ns3::ApplicationContainer>> m_packetSinks;

    void m_trackPacketsSent( Ptr<const Packet> packet );
    void m_trackPacketsReceived( Ptr<const Packet> packet, Address receiver );
    std::vector<std::vector<uint32_t>> m_packetsSent;
    std::vector<std::vector<uint32_t>> m_packetsReceived;

    void m_findNeighborsCallback( double interval );
};

#endif // ADHOC_NETWORK_H
