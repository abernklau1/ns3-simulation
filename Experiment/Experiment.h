#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/gnuplot.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE( "Wifi-Adhoc" );

/**
 * WiFi adhoc experiment class.
 *
 * It handles the creation and run of an experiment.
 */
class Experiment
{
  public:
    Experiment( );
    /**
     * Constructor.
     * \param name The name of the experiment.
     */
    Experiment( std::string name );

    /**
     * Run an experiment.
     * \param wifi      //!< The WifiHelper class.
     * \param wifiPhy   //!< The YansWifiPhyHelper class.
     * \param wifiMac   //!< The WifiMacHelper class.
     * \param wifiChannel //!< The YansWifiChannelHelper class.
     * \return a 2D dataset of the experiment data.
     */
    Gnuplot2dDataset Run( const WifiHelper& wifi, const YansWifiPhyHelper& wifiPhy, const WifiMacHelper& wifiMac, const YansWifiChannelHelper& wifiChannel );

  private:
    /**
     * Receive a packet.
     * \param socket The receiving socket.
     */
    void ReceivePacket( Ptr<Socket> socket );
    /**
     * Set the position of a node.
     * \param node The node.
     * \param position The position of the node.
     */
    void SetPosition( Ptr<Node> node, Vector position );
    /**
     * Get the position of a node.
     * \param node The node.
     * \return the position of the node.
     */
    Vector GetPosition( Ptr<Node> node );
    /**
     * Move a node by 1m on the x axis, stops at 210m.
     * \param node The node.
     */
    void AdvancePosition( Ptr<Node> node );
    /**
     * Setup the receiving socket.
     * \param node The receiving node.
     * \return the socket.
     */
    Ptr<Socket> SetupPacketReceive( Ptr<Node> node );

    uint32_t m_bytesTotal;     //!< The number of received bytes.
    Gnuplot2dDataset m_output; //!< The output dataset.
};