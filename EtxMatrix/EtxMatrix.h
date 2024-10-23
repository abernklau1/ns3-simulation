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
#include <iostream>

using namespace ns3;

class EtxMatrix
{
  public:
    EtxMatrix( ) = default;
    EtxMatrix( uint32_t numNodes );

    void initializeMatrix( uint32_t numNodes );

    void printEtxMatrix( );

    void setEtx( size_t node, size_t neighbor, double etx );

    std::vector<std::vector<double>> getEtxMatrix( ) const { return m_etxMatrix; }

  private:
    uint32_t m_numNodes;
    std::vector<std::vector<double>> m_etxMatrix;
};

#endif // ETXMATRIX_H