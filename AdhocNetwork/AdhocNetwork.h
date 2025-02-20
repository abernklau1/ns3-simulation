#ifndef ADHOC_NETWORK_H
#define ADHOC_NETWORK_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "../GossipHeader.h"

// Custom hash for std::pair<uint32_t, uint32_t>
namespace std
{
    template <> struct hash<std::pair<uint32_t, uint32_t>>
    {
            size_t operator( )( const std::pair<uint32_t, uint32_t>& p ) const { return std::hash<uint32_t>( )( p.first ) ^ ( std::hash<uint32_t>( )( p.second ) << 1 ); }
    };
} // namespace std

class AdhocNetwork
{
    public:
        AdhocNetwork( uint32_t numNodes,
                      std::set<uint32_t> sensorTypes,
                      std::set<uint32_t> areaTypes,
                      WifiStandard wifiStandard,
                      std::string macType,
                      std::string ipBase,
                      std::string positionAllocator,
                      double communicationRange,
                      double gridX,
                      double gridY,
                      std::vector<std::vector<uint32_t>> nodeSensors,
                      std::vector<uint32_t> nodeAreas );
        ~AdhocNetwork( );

        // Setup and initialization
        void setup( );
        void initializeRandomPositions( double xMin, double xMax, double yMin, double yMax );
        void InitializeNodeCoverageSets( );

        // Neighbor discovery and selection
        void findNeighbors( uint32_t nodeId );
        void findNeighborsSubset( uint32_t nodeId );
        void scheduleFindNeighbors( double interval );
        void m_findNeighborsCallback( double interval );

        // Packet sending/receiving
        Ptr<Socket> GetSenderSocket( uint32_t senderId, uint32_t receiverId );
        void SendPackets( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors );
        void SendPacketsHelper( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors );
        void SetupDataReceiver( Ptr<Node> node, uint32_t nodeId );
        uint32_t GetNodeIdFromIpAddress( Ipv4Address address );
        void ReceivePacket( Ptr<Socket> socket );

        // Coverage and utility
        std::pair<uint32_t, uint32_t> SetCoverage( uint32_t nodeId );
        bool m_isCovered( uint32_t receiverId );
        double m_calculateUtility( uint32_t senderId, uint32_t receiverId );
        void updateAggregatedSensors( uint32_t receiverId );
        void updateAggregatedAreas( uint32_t receiverId );
        int getMaxCoverage( );
        uint32_t calculateNumSubNeighbors( uint32_t nodeId );
        void PrintFinalCoverage( uint32_t nodeId ) const;
        bool IsCoverageReached( );

        // Getters
        NodeContainer getNodes( ) const { return m_nodes; }

        uint32_t getNumNodes( ) const { return m_numNodes; }

        NetDeviceContainer getDevices( ) const { return m_devices; }

        Ipv4InterfaceContainer getInterfaces( ) const { return m_interfaces; }

        std::vector<std::vector<Ptr<Node>>> getNeighbors( ) const { return m_neighbors; }

        std::vector<std::vector<Ptr<Node>>> getNeighborsSubset( ) const { return m_neighborsSubset; }

    private:
        // Simulation parameters
        uint32_t m_numNodes;
        uint32_t m_numSensors;
        uint32_t m_numAreas;
        WifiStandard m_wifiStandard;
        std::string m_macType;
        std::string m_ipBase;
        std::string m_positionAllocator;
        double m_communicationRange;
        double m_gridX;
        double m_gridY;

        // Node containers and network devices
        NodeContainer m_nodes;
        NetDeviceContainer m_devices;
        Ipv4InterfaceContainer m_interfaces;

        // Mobility and positions
        std::vector<Vector> m_positions;

        // Neighbors and subsets
        std::vector<std::vector<Ptr<Node>>> m_neighbors;
        std::vector<std::vector<Ptr<Node>>> m_neighborsSubset;
        std::unordered_map<std::pair<uint32_t, uint32_t>, Ptr<Socket>> m_senderSockets;

        // Gossip protocol parameters
        uint32_t m_gossipGroupSize;
        double m_alpha;
        double m_beta;
        double m_lambda;
        uint32_t m_maximumDataSize;
        std::vector<uint32_t> m_dataSizesScaled;

        // Coverage information
        std::vector<std::set<std::pair<uint32_t, uint32_t>>> m_nodeCoverageSets;
        std::vector<uint32_t> m_coverageSteps;
        std::vector<std::set<uint32_t>> m_receivedPackets;
        bool m_isCoverageReached;

        // Sensor and area assignments
        std::set<uint32_t> m_sensorTypes;
        std::vector<std::vector<uint32_t>> m_sensorCoverage;
        std::set<uint32_t> m_areas;
        std::vector<std::vector<uint32_t>> m_areaCoverage;

        // Bitset representations
        static constexpr size_t MAX_SENSOR_TYPES = 3;
        static constexpr size_t MAX_AREA_TYPES   = 3;
        std::vector<std::bitset<MAX_SENSOR_TYPES>> m_sensorCoverageBitset;
        std::vector<std::bitset<MAX_AREA_TYPES>> m_areaCoverageBitset;
        std::vector<std::bitset<MAX_SENSOR_TYPES>> m_aggregatedSensorsBitset;
        std::vector<std::bitset<MAX_AREA_TYPES>> m_aggregatedAreasBitset;
        std::vector<std::unordered_map<uint32_t, std::bitset<MAX_SENSOR_TYPES>>> m_neighborSensorBitset;
        std::vector<std::unordered_map<uint32_t, std::bitset<MAX_AREA_TYPES>>> m_neighborAreaBitset;

        // Pre-assigned sensor and area assignments per node
        std::vector<std::vector<uint32_t>> m_assignedSensors;
        std::vector<uint32_t> m_assignedAreas;
};

#endif // ADHOC_NETWORK_H