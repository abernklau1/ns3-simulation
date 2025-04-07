#ifndef GOSSIP_HEADER_H
#define GOSSIP_HEADER_H

#include "ns3/address-utils.h"
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/packet.h"

#include "../CoverageQuad.h"

#include <set>

using namespace ns3;

/**
 * @brief The GossipHeader class defines the header used in the UAV gossip protocol.
 *
 * This header contains the originating node's ID, a set of sensor-area pairs (representing
 * the node's coverage), and a data size value. The header is serialized into packets and
 * exchanged between nodes.
 */
class GossipHeader : public Header
{
    public:
        /**
         * @brief Default constructor.
         *
         * Initializes the origin node ID and data size to zero.
         */
        GossipHeader( );

        /**
         * @brief Constructs a GossipHeader with specified values.
         *
         * @param originNodeId The ID of the originating node.
         * @param coverageVersion The ID of the gossip round.
         * @param coverageSet A set of sensor-area pairs representing the node's coverage.
         * @param dataSize The data size associated with the header.
         */
        GossipHeader( uint32_t originNodeId, uint32_t coverageVersion, std::set<CoverageQuad, CoverageQuadLess> coverageSet, uint32_t dataSize );

        /**
         * @brief Destructor.
         */
        ~GossipHeader( );

        /**
         * @brief Sets the originating node ID.
         *
         * @param originNodeId The ID of the node that created this header.
         */
        void SetOriginNodeId( uint32_t originNodeId );

        /**
         * @brief Gets the originating node ID.
         *
         * @return The ID of the node that originated the header.
         */
        uint32_t GetOriginNodeId( ) const;

        void SetCoverageVersion( uint32_t coverageVersion );
        uint32_t GetCoverageVersion( ) const;

        /**
         * @brief Sets the coverage set.
         *
         * @param coverageSet A set of sensor-area pairs representing coverage.
         */
        void SetCoverageSet( const std::set<CoverageQuad, CoverageQuadLess>& coverageSet );

        /**
         * @brief Gets the coverage set.
         *
         * @return A set of sensor-area pairs.
         */
        std::set<CoverageQuad, CoverageQuadLess> GetCoverageSet( ) const;

        /**
         * @brief Sets the data size.
         *
         * @param dataSize The data size value.
         */
        void SetDataSize( uint32_t dataSize );

        /**
         * @brief Gets the data size.
         *
         * @return The data size.
         */
        uint32_t GetDataSize( ) const;

        //===========================================================================
        // Header interface implementation
        //===========================================================================

        /**
         * @brief Gets the TypeId of the GossipHeader.
         *
         * @return The TypeId for the GossipHeader class.
         */
        static TypeId GetTypeId( );

        /**
         * @brief Gets the instance TypeId.
         *
         * @return The TypeId of this instance.
         */
        virtual TypeId GetInstanceTypeId( ) const;

        /**
         * @brief Prints the header contents to an output stream.
         *
         * @param os The output stream.
         */
        virtual void Print( std::ostream& os ) const;

        /**
         * @brief Gets the size (in bytes) required to serialize the header.
         *
         * @return The serialized size in bytes.
         */
        virtual uint32_t GetSerializedSize( ) const;

        /**
         * @brief Serializes the header into a buffer.
         *
         * @param start An iterator pointing to the start of the buffer.
         */
        virtual void Serialize( Buffer::Iterator start ) const;

        /**
         * @brief Deserializes the header from a buffer.
         *
         * @param start An iterator pointing to the start of the buffer.
         * @return The number of bytes read during deserialization.
         */
        virtual uint32_t Deserialize( Buffer::Iterator start );

    private:
        uint32_t _originNodeId;                                // The originating node's ID.
        uint32_t _coverageVersion;                             // Gossip coverage round
        std::set<CoverageQuad, CoverageQuadLess> _coverageSet; // Set of sensor-area pairs representing coverage.
        uint32_t _dataSize;                                    // The data size value.
};

#endif // GOSSIP_HEADER_H