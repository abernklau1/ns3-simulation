#include "ExtNode.h"

using namespace ns3;

ExtNode::ExtNode( uint32_t nodeId, Ptr<Node> node )
    : m_nodeId( nodeId ),
      m_node( node )
{
}

uint32_t ExtNode::GetNodeId( ) const { return m_nodeId; }

Ptr<Node> ExtNode::GetNode( ) const { return m_node; }

void ExtNode::AddNeighbor( Ptr<ExtNode> neighbor ) { m_neighbors.push_back( neighbor ); }

const std::vector<Ptr<ExtNode>>& ExtNode::GetNeighbors( ) const { return m_neighbors; }

void ExtNode::SelectNeighborSubset( )
{
    NS_LOG_INFO( "Starting subset neighbor selection for node " << m_nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );

    // TODO: Change to conduct search for a single node instead of all nodes
    // Clear the previous subset of neighbors
    m_neighborSubset.clear( );

    Ptr<UniformRandomVariable> randomVar = CreateObject<UniformRandomVariable>( );

    if ( m_neighbors.empty( ) )
    {
        NS_LOG_WARN( "Node " << m_nodeId << " has no neighbors." );
        return;
    }

    uint32_t numNeighbors = m_neighbors.size( );

    // Calculate subset size using ceil to ensure at least 1 neighbor is selected
    uint32_t subsetSize = static_cast<uint32_t>( std::ceil( std::log( static_cast<double>( numNeighbors ) ) ) );
    subsetSize          = ( subsetSize > 0 ) ? subsetSize : 1;
    subsetSize          = std::min( subsetSize, numNeighbors ); // Prevent subsetSize > numNeighbors

    std::unordered_set<uint32_t> selectedIndices;
    while ( selectedIndices.size( ) < subsetSize )
    {
        uint32_t randomIndex = randomVar->GetInteger( 0, numNeighbors - 1 );
        selectedIndices.insert( randomIndex );
    }

    for ( uint32_t index : selectedIndices )
    {
        m_neighborSubset.emplace_back( m_neighbors.at( index ) );
    }
    NS_LOG_DEBUG( "Node " << m_nodeId << " selected " << subsetSize << " neighbors." );

    NS_LOG_INFO( "Subset neighbor selection completed for node " << m_nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
}
