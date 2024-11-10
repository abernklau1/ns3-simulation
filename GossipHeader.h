#include "ns3/address-utils.h"
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/packet.h"

#include <set>

#ifndef GOSSIP_HEADER_H
    #define GOSSIP_HEADER_H

using namespace ns3;

class GossipHeader : public Header
{
    public:
        GossipHeader( );
        GossipHeader( uint32_t originNodeId, std::set<std::pair<uint32_t, uint32_t>> coverageSet, uint32_t dataSize );
        ~GossipHeader( );

        void SetOriginNodeId( uint32_t originNodeId );
        uint32_t GetOriginNodeId( ) const;

        void SetCoverageSet( const std::set<std::pair<uint32_t, uint32_t>>& coverageSet );
        std::set<std::pair<uint32_t, uint32_t>> GetCoverageSet( ) const;

        void SetDataSize( uint32_t dataSize );
        uint32_t GetDataSize( ) const;

        // Inherited from Header
        static TypeId GetTypeId( );
        virtual TypeId GetInstanceTypeId( ) const;
        virtual void Print( std::ostream& os ) const;
        virtual uint32_t GetSerializedSize( ) const;
        virtual void Serialize( Buffer::Iterator start ) const;
        virtual uint32_t Deserialize( Buffer::Iterator start );

    private:
        uint32_t m_originNodeId;
        std::set<std::pair<uint32_t, uint32_t>> m_coverageSet;
        uint32_t m_dataSize;
};

#endif // GOSSIP_HEADER_H