#include "AdhocNetwork/AdhocNetwork.h"
#include "ns3/command-line.h"

using namespace ns3;

/*
 * This program sets up an ad-hoc network with a specified number of nodes, WiFi standard, MAC type, and IP base.
 *
 * The network is configured with constant position mobility and the neighbors within a specified communication range are identified.
 *
 * You can set the following parameters using command line arguments:
 *
 * - numNodes: Specifies the number of nodes in the network.
 * - wifiStandard: Specifies the WiFi standard to use.
 * - macType: Specifies the MAC type to use.
 * - ipBase: Specifies the IP base for the network.
 * - communicationRange: Specifies the communication range to find neighbors.
 *
 * Default values for numNodes are: 10.
 * Default values for wifiStandard are: WIFI_STANDARD_80211a.
 * Default values for macType are: "ns3::AdhocWifiMac".
 * Default values for ipBase are: "10.1.1.0".
 * Default values for communicationRange are: 25.0.
 */
int main( int argc, char* argv[] )
{
    CommandLine cmd( __FILE__ );
    cmd.Parse( argc, argv );

    uint32_t numNodes = 10;

    AdhocNetwork adhocNetwork( numNodes, WIFI_STANDARD_80211a, "ns3::AdhocWifiMac", "10.1.1.0", "ns3::RandomRectanglePositionAllocator", 25.0 );
    adhocNetwork.setup( );

    // Schedule every node to find their neighbors 1 second into the simulation
    for ( int i = 0; i < numNodes; i++ )
    {
        Simulator::Schedule( Seconds( 0.0 ), MakeEvent( &AdhocNetwork::findNeighbors, &adhocNetwork, i ) );
    }

    // NOTE: This may still need to occur later when the mobility model is randomized instead of constant
    // adhocNetwork.scheduleFindNeighbors( 5.0 );

    // Set the simulation to stop after 100 seconds
    Simulator::Stop( Seconds( 100.0 ) );

    Simulator::Run( );
    Simulator::Destroy( );

    // Retrieve neighbors after the simulation has run
    std::vector<std::vector<Ptr<Node>>> neighbors = adhocNetwork.getNeighbors( );

    // Print the neighbors
    for ( int i = 0; i < neighbors.size( ); i++ )
    {
        std::cout << "Node " << i << " neighbors: ";
        for ( Ptr<Node> node : neighbors.at( i ) )
        {
            std::cout << node->GetId( ) << " ";
        }
        std::cout << std::endl;
    }

    // Print ETX matrix
    adhocNetwork.getEtxMatrix( ).printEtxMatrix( );

    return 0;
}
