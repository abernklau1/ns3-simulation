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

    NodeContainer getNodes( ) const { return m_nodes; }

    NetDeviceContainer getDevices( ) const { return m_devices; }

    Ipv4InterfaceContainer getInterfaces( ) const { return m_interfaces; }

    std::vector<std::vector<Ptr<Node>>> getNeighbors( ) const { return m_neighbors; }

    EtxMatrix getEtxMatrix( ) const { return m_etxMatrix; }

    void scheduleFindNeighbors( double interval );

    void initializeRandomPositions( double xMin, double xMax, double yMin, double yMax );

    void CalculateETX( uint32_t nodeId, uint32_t neighborId );

    void CalculateETXHelper( uint32_t nodeId, std::vector<Ptr<Node>> neighbors );

    void SendPackets( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors );

    void SendPacketsHelper( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors );

    void SetupDataReceiver( Ptr<Node> node, uint32_t nodeId );

    uint32_t GetNodeIdFromIpAddress( Ipv4Address address );

    void ReceivePacket( Ptr<Socket> socket );

    void SetupAckReceiver( Ptr<Node> node, uint32_t nodeId );

    void ReceiveAck( Ptr<Socket> socket );

  private:

      struct LinkStats
    {
        uint32_t dataPacketsSent     = 0;
        uint32_t dataPacketsReceived = 0;
        uint32_t ackPacketsSent      = 0;
        uint32_t ackPacketsReceived  = 0;
    };

    uint32_t m_numNodes;
    WifiStandard m_wifiStandard;
    std::string m_macType;
    std::string m_ipBase;
    std::vector<std::vector<Ptr<Node>>> m_neighbors;
    std::vector<Vector> m_positions;

    NodeContainer m_nodes;
    NetDeviceContainer m_devices;
    Ipv4InterfaceContainer m_interfaces;

    std::string m_positionAllocator;
    double m_communicationRange;

    std::map<std::pair<uint32_t, uint32_t>, LinkStats> m_linkStats;

    void m_findNeighborsCallback( double interval );

    EtxMatrix m_etxMatrix;

};

#endif // ADHOC_NETWORK_H
