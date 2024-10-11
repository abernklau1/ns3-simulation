#ifndef ETXMATRIX_H
#define ETXMATRIX_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/node.h"
#include "ns3/on-off-helper.h"
#include "ns3/onoff-application.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/simulator.h"
#include "ns3/wifi-module.h"

using namespace ns3;

class EtxMatrix
{
  public:
    EtxMatrix( uint32_t numNodes, NodeContainer nodes, std::vector<std::vector<Node>> neighbors );
    ~EtxMatrix( );

    void calculateEtxMatrix( );
    void printEtxMatrix( );

    std::vector<std::vector<double>> getEtxMatrix( ) const { return m_etxMatrix; }

  private:
    uint32_t m_numNodes;
    std::vector<std::vector<double>> m_etxMatrix;
    NodeContainer m_nodes;
    std::vector<std::vector<Node>> m_neighbors;
};

#endif // ETXMATRIX_H