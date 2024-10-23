#include "EtxMatrix.h"

NS_LOG_COMPONENT_DEFINE( "EtxMatrix" );

EtxMatrix::EtxMatrix( uint32_t numNodes ) : m_numNodes( numNodes )
{
  m_etxMatrix.resize( numNodes, std::vector<double>( numNodes, 0.0 ) );
}

void EtxMatrix::initializeMatrix( uint32_t numNodes )
{
  m_numNodes = numNodes;
  m_etxMatrix.resize( numNodes, std::vector<double>( numNodes, 0.0 ) );
}

void EtxMatrix::printEtxMatrix( )
{
  for ( uint32_t i = 0; i < m_numNodes; ++i )
  {
    for ( uint32_t j = 0; j < m_numNodes; ++j )
    {
      std::cout << "Node " << i << " to Node " << j << ": " << m_etxMatrix.at( i ).at( j ) << std::endl;
    }
    std::cout << std::endl;
  }
}

void EtxMatrix::setEtx( size_t node, size_t neighbor, double etx ) { m_etxMatrix.at( node ).at( neighbor ) = etx; }