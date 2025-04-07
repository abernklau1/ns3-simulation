#include "GossipHeader.h"

NS_LOG_COMPONENT_DEFINE( "GossipHeader" );

GossipHeader::GossipHeader( )
    : _originNodeId( 0 ),
      _coverageVersion( 0 ),
      _dataSize( 0 )
{
}

GossipHeader::GossipHeader( uint32_t originNodeId, uint32_t coverageVersion, std::set<CoverageQuad, CoverageQuadLess> coverageSet, uint32_t dataSize )
    : _originNodeId( originNodeId ),
      _coverageVersion( coverageVersion ),
      _coverageSet( coverageSet ),
      _dataSize( dataSize )
{
}

GossipHeader::~GossipHeader( ) { }

void GossipHeader::SetOriginNodeId( uint32_t originNodeId ) { _originNodeId = originNodeId; }

uint32_t GossipHeader::GetOriginNodeId( ) const { return _originNodeId; }

void GossipHeader::SetCoverageVersion( uint32_t coverageVersion ) { _coverageVersion = coverageVersion; }

uint32_t GossipHeader::GetCoverageVersion( ) const { return _coverageVersion; }

void GossipHeader::SetCoverageSet( const std::set<CoverageQuad, CoverageQuadLess>& coverageSet ) { _coverageSet = coverageSet; }

std::set<CoverageQuad, CoverageQuadLess> GossipHeader::GetCoverageSet( ) const { return _coverageSet; }

void GossipHeader::SetDataSize( uint32_t dataSize ) { _dataSize = dataSize; }

uint32_t GossipHeader::GetDataSize( ) const { return _dataSize; }

// =============================================================================
// Header interface implementation
// =============================================================================

TypeId GossipHeader::GetTypeId( )
{
    static TypeId tid = TypeId( "ns3::GossipHeader" ).SetParent<Header>( ).AddConstructor<GossipHeader>( );
    return tid;
}

TypeId GossipHeader::GetInstanceTypeId( ) const { return GetTypeId( ); }

void GossipHeader::Print( std::ostream& os ) const
{
    os << "OriginNodeId=" << _originNodeId << ", CoverageVersion=" << _coverageVersion << ", DataSize=" << _dataSize << ", CoverageSet={";

    bool first = true;
    for ( auto& quad : _coverageSet )
    {
        if ( !first )
            os << ", ";
        first = false;
        os << "["
           << "sensor=" << quad.sensor << " area=" << quad.area << " origin=" << quad.originId << " dataScaled=" << quad.dataScaled << "]";
    }
    os << "}";
}

uint32_t GossipHeader::GetSerializedSize( ) const
{
    // Each CoverageQuad is 16 bytes:
    //   sensor (4) + area (4) + originId (4) + dataScaled (float:4).
    // Plus 4 bytes each for 4 fields at the start: originNodeId, coverageVersion, dataSize, setSize
    // => 16 * coverageSet.size() + 16 = 16 + 16 * N
    uint32_t baseFields     = 4 * 4;                     // 4 fields x 4 bytes each => 16
    uint32_t coverageFields = _coverageSet.size( ) * 16; // each coverageQuad => 16 bytes

    return baseFields + coverageFields;
}

void GossipHeader::Serialize( Buffer::Iterator start ) const
{
    // 1) Write originNodeId, coverageVersion, dataSize, setSize
    start.WriteHtonU32( _originNodeId );
    start.WriteHtonU32( _coverageVersion );
    start.WriteHtonU32( _dataSize );
    start.WriteHtonU32( static_cast<uint32_t>( _coverageSet.size( ) ) );

    // 2) For each coverageQuad => write 4 fields:
    for ( auto& quad : _coverageSet )
    {
        start.WriteHtonU32( quad.sensor );
        start.WriteHtonU32( quad.area );
        start.WriteHtonU32( quad.originId );

        // Convert double -> float bits or store as 32-bit float
        float f = static_cast<float>( quad.dataScaled );
        // reinterpret the float bits as uint32_t
        uint32_t fBits;
        std::memcpy( &fBits, &f, sizeof( uint32_t ) );
        start.WriteHtonU32( fBits );
    }
}

uint32_t GossipHeader::Deserialize( Buffer::Iterator start )
{
    // 1) read first 4 fields
    _originNodeId    = start.ReadNtohU32( );
    _coverageVersion = start.ReadNtohU32( );
    _dataSize        = start.ReadNtohU32( );
    uint32_t setSize = start.ReadNtohU32( );

    _coverageSet.clear( );
    // 2) read 'setSize' coverageQuad items
    for ( uint32_t i = 0; i < setSize; ++i )
    {
        CoverageQuad quad;
        quad.sensor   = start.ReadNtohU32( );
        quad.area     = start.ReadNtohU32( );
        quad.originId = start.ReadNtohU32( );

        // read float bits from buffer
        uint32_t fBits = start.ReadNtohU32( );
        float f;
        std::memcpy( &f, &fBits, sizeof( uint32_t ) );
        quad.dataScaled = static_cast<double>( f );

        _coverageSet.insert( quad );
    }

    // Return total size consumed
    return GetSerializedSize( );
}