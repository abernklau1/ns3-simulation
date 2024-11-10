#include "GossipHeader.h"

NS_LOG_COMPONENT_DEFINE( "GossipHeader" );

GossipHeader::GossipHeader( )
    : m_originNodeId( 0 ),
      m_dataSize( 0 )
{
}

GossipHeader::GossipHeader( uint32_t originNodeId, std::set<std::pair<uint32_t, uint32_t>> coverageSet, uint32_t dataSize )
    : m_originNodeId( originNodeId ),
      m_coverageSet( coverageSet ),
      m_dataSize( dataSize )
{
}

GossipHeader::~GossipHeader( ) { }

void GossipHeader::SetOriginNodeId( uint32_t originNodeId ) { m_originNodeId = originNodeId; }

uint32_t GossipHeader::GetOriginNodeId( ) const { return m_originNodeId; }

void GossipHeader::SetCoverageSet( const std::set<std::pair<uint32_t, uint32_t>>& coverageSet ) { m_coverageSet = coverageSet; }

std::set<std::pair<uint32_t, uint32_t>> GossipHeader::GetCoverageSet( ) const { return m_coverageSet; }

void GossipHeader::SetDataSize( uint32_t dataSize ) { m_dataSize = dataSize; }

uint32_t GossipHeader::GetDataSize( ) const { return m_dataSize; }

TypeId GossipHeader::GetTypeId( )
{
    static TypeId tid = TypeId( "ns3::GossipHeader" ).SetParent<Header>( ).AddConstructor<GossipHeader>( );
    return tid;
}

TypeId GossipHeader::GetInstanceTypeId( ) const { return GetTypeId( ); }

void GossipHeader::Print( std::ostream& os ) const
{
    os << "OriginNodeId=" << m_originNodeId << ", DataSize=" << m_dataSize << ", CoverageSet={";
    for ( auto it = m_coverageSet.begin( ); it != m_coverageSet.end( ); ++it )
    {
        os << "(" << it->first << "," << it->second << ")";
        if ( std::next( it ) != m_coverageSet.end( ) )
            os << ",";
    }
    os << "}";
}

uint32_t GossipHeader::GetSerializedSize( ) const
{
    uint32_t size = 4 + 4 + 4;                 // m_originNodeId, m_dataSize, coverage set size
    size += m_coverageSet.size( ) * ( 4 + 4 ); // Each pair of uint32_t
    return size;
}

void GossipHeader::Serialize( Buffer::Iterator start ) const
{
    start.WriteHtonU32( m_originNodeId );
    start.WriteHtonU32( m_dataSize );
    start.WriteHtonU32( m_coverageSet.size( ) );
    for ( auto& pair : m_coverageSet )
    {
        start.WriteHtonU32( pair.first );  // Sensor type
        start.WriteHtonU32( pair.second ); // Area coverage
    }
}

uint32_t GossipHeader::Deserialize( Buffer::Iterator start )
{
    m_originNodeId   = start.ReadNtohU32( );
    m_dataSize       = start.ReadNtohU32( );
    uint32_t setSize = start.ReadNtohU32( );
    m_coverageSet.clear( );
    for ( uint32_t i = 0; i < setSize; ++i )
    {
        uint32_t sensorType   = start.ReadNtohU32( );
        uint32_t areaCoverage = start.ReadNtohU32( );
        m_coverageSet.insert( std::make_pair( sensorType, areaCoverage ) );
    }
    return GetSerializedSize( );
}