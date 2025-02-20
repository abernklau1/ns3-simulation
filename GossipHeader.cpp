#include "GossipHeader.h"

NS_LOG_COMPONENT_DEFINE( "GossipHeader" );

GossipHeader::GossipHeader( )
    : _originNodeId( 0 ),
      _dataSize( 0 )
{
}

GossipHeader::GossipHeader( uint32_t originNodeId, std::set<std::pair<uint32_t, uint32_t>> coverageSet, uint32_t dataSize )
    : _originNodeId( originNodeId ),
      _coverageSet( coverageSet ),
      _dataSize( dataSize )
{
}

GossipHeader::~GossipHeader( ) { }

void GossipHeader::SetOriginNodeId( uint32_t originNodeId ) { _originNodeId = originNodeId; }

uint32_t GossipHeader::GetOriginNodeId( ) const { return _originNodeId; }

void GossipHeader::SetCoverageSet( const std::set<std::pair<uint32_t, uint32_t>>& coverageSet ) { _coverageSet = coverageSet; }

std::set<std::pair<uint32_t, uint32_t>> GossipHeader::GetCoverageSet( ) const { return _coverageSet; }

void GossipHeader::SetDataSize( uint32_t dataSize ) { _dataSize = dataSize; }

uint32_t GossipHeader::GetDataSize( ) const { return _dataSize; }

//============================================================================
// Header interface implementation
//============================================================================

TypeId GossipHeader::GetTypeId( )
{
    static TypeId tid = TypeId( "ns3::GossipHeader" ).SetParent<Header>( ).AddConstructor<GossipHeader>( );
    return tid;
}

TypeId GossipHeader::GetInstanceTypeId( ) const { return GetTypeId( ); }

void GossipHeader::Print( std::ostream& os ) const
{
    os << "OriginNodeId=" << _originNodeId << ", DataSize=" << _dataSize << ", CoverageSet={";
    for ( auto it = _coverageSet.begin( ); it != _coverageSet.end( ); ++it )
    {
        os << "(" << it->first << "," << it->second << ")";
        if ( std::next( it ) != _coverageSet.end( ) )
            os << ",";
    }
    os << "}";
}

uint32_t GossipHeader::GetSerializedSize( ) const
{
    uint32_t size = 4 + 4 + 4;                // _originNodeId, _dataSize, coverage set size
    size += _coverageSet.size( ) * ( 4 + 4 ); // Each pair of uint32_t
    return size;
}

void GossipHeader::Serialize( Buffer::Iterator start ) const
{
    start.WriteHtonU32( _originNodeId );
    start.WriteHtonU32( _dataSize );
    start.WriteHtonU32( _coverageSet.size( ) );
    for ( auto& pair : _coverageSet )
    {
        start.WriteHtonU32( pair.first );  // Sensor type
        start.WriteHtonU32( pair.second ); // Area coverage
    }
}

uint32_t GossipHeader::Deserialize( Buffer::Iterator start )
{
    _originNodeId    = start.ReadNtohU32( );
    _dataSize        = start.ReadNtohU32( );
    uint32_t setSize = start.ReadNtohU32( );
    _coverageSet.clear( );
    for ( uint32_t i = 0; i < setSize; ++i )
    {
        uint32_t sensorType   = start.ReadNtohU32( );
        uint32_t areaCoverage = start.ReadNtohU32( );
        _coverageSet.insert( std::make_pair( sensorType, areaCoverage ) );
    }
    return GetSerializedSize( );
}