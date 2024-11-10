#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"

#include <map>
#include <unordered_set>
#include <vector>

using namespace ns3;

#ifndef EXT_NODE_H
    #define EXT_NODE_H

class ExtNode
{
    public:
        ExtNode( uint32_t nodeId, Ptr<Node> node );

        // Node identification
        uint32_t GetNodeId( ) const;
        Ptr<Node> GetNode( ) const;

        // Neighbor management
        void AddNeighbor( Ptr<ExtNode> neighbor );
        const std::vector<Ptr<ExtNode>>& GetNeighbors( ) const;

        void SelectNeighborSubset( );
        const std::vector<Ptr<ExtNode>>& GetNeighborSubset( ) const;

        // Communication management
        void SetupSockets( );
        void SendPackets( );
        void ReceivePacket( Ptr<Socket> socket );
        void ReceiveAck( Ptr<Socket> socket );

        // ETX calculation
        void CalculateETX( );

        // Link statistics management
        void UpdateLinkStats( uint32_t neighborId, bool isAck, bool isSent );

    private:
        uint32_t m_nodeId;
        Ptr<Node> m_node;

        // Neighbors and neighbor subsets
        std::vector<Ptr<ExtNode>> m_neighbors;
        std::vector<Ptr<ExtNode>> m_neighborSubset;

        // Sockets for each neighbor
        std::map<uint32_t, Ptr<Socket>> m_senderSockets; // Keyed by neighbor ID
        std::map<uint32_t, Ptr<Socket>> m_ackSockets;

        // Link statistics per neighbor
        struct LinkStats
        {
                uint32_t dataPacketsSent     = 0;
                uint32_t dataPacketsReceived = 0;
                uint32_t ackPacketsSent      = 0;
                uint32_t ackPacketsReceived  = 0;
        };

        std::map<uint32_t, LinkStats> m_linkStats;

        // ETX values per neighbor
        std::map<uint32_t, double> m_etxValues;
};

#endif // EXT_NODE_H
