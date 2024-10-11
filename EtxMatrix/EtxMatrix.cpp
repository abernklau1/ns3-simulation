#include "EtxMatrix.h"

NS_LOG_COMPONENT_DEFINE( "EtxMatrix" );

EtxMatrix::EtxMatrix( uint32_t numNodes, NodeContainer nodes, std::vector<std::vector<Node>> neighbors )
    : m_numNodes( numNodes ),
      m_nodes( nodes ),
      m_neighbors( neighbors )
{
  m_etxMatrix.resize( numNodes, std::vector<double>( numNodes, 0.0 ) );
}

EtxMatrix::~EtxMatrix( ) { }

void EtxMatrix::calculateEtxMatrix( )
{
  for ( uint32_t i = 0; i < m_numNodes; ++i )
  {
    for ( uint32_t j = 0; j < m_neighbors.at( i ).size( ); ++j )
    {
      //
      uint32_t packetsSent     = 100; // Simulate sending 100 packets
      uint32_t packetsReceived = simulatePacketExchange( i, j, packetsSent );

      // Calculate forward probability (P_f)
      double forwardProbability = packetsSent > 0 ? static_cast<double>( packetsReceived ) / packetsSent : 0.0;

      // Simulate sending a packet from node j to node i (ACK)
      uint32_t ackPacketsSent     = 100; // Simulate sending 100 ACK packets
      uint32_t ackPacketsReceived = simulatePacketExchange( j, i, ackPacketsSent );

      // Calculate reverse probability (P_r)
      double reverseProbability = ackPacketsSent > 0 ? static_cast<double>( ackPacketsReceived ) / ackPacketsSent : 0.0;

      // Calculate ETX
      double etx = ( forwardProbability > 0 && reverseProbability > 0 ) ? 1.0 / ( forwardProbability * reverseProbability ) : std::numeric_limits<double>::infinity( );

      // Store ETX value in the matrix
      m_etxMatrix.at( i ).at( j ) = etx;
      m_etxMatrix.at( j ).at( i ) = etx;
    }
  }
}

void EtxMatrix::printEtxMatrix( )
{
  for ( uint32_t i = 0; i < m_numNodes; ++i )
  {
    for ( uint32_t j = 0; j < m_numNodes; ++j )
    {
      std::cout << m_etxMatrix.at( i ).at( j ) << " ";
    }
    std::cout << std::endl;
  }
}