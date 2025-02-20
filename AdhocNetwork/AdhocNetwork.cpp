// AdhocNetwork.cpp

#include "AdhocNetwork.h"

NS_LOG_COMPONENT_DEFINE( "AdhocNetwork" );

//
// Constructor
//
AdhocNetwork::AdhocNetwork( uint32_t numNodes,
                            std::set<uint32_t> sensorTypes,
                            std::set<uint32_t> areaTypes,
                            WifiStandard wifiStandard,
                            std::string macType,
                            std::string ipBase,
                            std::string positionAllocator,
                            double communicationRange,
                            double gridX,
                            double gridY,
                            std::vector<std::vector<uint32_t>> nodeSensors,
                            std::vector<uint32_t> nodeAreas )
    : m_numNodes( numNodes ),
      m_sensorTypes( sensorTypes ),
      m_areas( areaTypes ),
      m_wifiStandard( wifiStandard ),
      m_macType( macType ),
      m_ipBase( ipBase ),
      m_positionAllocator( positionAllocator ),
      m_communicationRange( communicationRange ),
      m_gridX( gridX ),
      m_gridY( gridY ),
      m_assignedSensors( nodeSensors ),
      m_assignedAreas( nodeAreas )
{
    NS_LOG_INFO( "Creating AdhocNetwork with " << numNodes << " nodes" );
    // Resize existing containers
    m_neighbors.resize( numNodes );
    m_positions.resize( numNodes );
    m_neighborsSubset.resize( numNodes );
    m_nodeCoverageSets.resize( numNodes );
    m_receivedPackets.resize( numNodes );
    m_dataSizesScaled.resize( numNodes );
    m_sensorCoverage.resize( numNodes );
    m_areaCoverage.resize( numNodes );
    m_coverageSteps.resize( numNodes );

    // Default parameters
    m_gossipGroupSize   = 3;
    m_alpha             = 2;
    m_beta              = 2;
    m_lambda            = 0.5;
    m_maximumDataSize   = 32;
    m_isCoverageReached = false;

    // Resize new bitset containers
    m_sensorCoverageBitset.resize( numNodes );
    m_areaCoverageBitset.resize( numNodes );
    m_aggregatedSensorsBitset.resize( numNodes );
    m_aggregatedAreasBitset.resize( numNodes );
    m_neighborSensorBitset.resize( numNodes );
    m_neighborAreaBitset.resize( numNodes );
    NS_LOG_INFO( "AdhocNetwork created" );
}

//
// Destructor
//
AdhocNetwork::~AdhocNetwork( )
{
    NS_LOG_INFO( "Destroying AdhocNetwork" );

    // Close and clear all sender sockets
    for ( auto& [link, socket] : m_senderSockets )
    {
        if ( socket )
        {
            socket->Close( );
        }
    }
    m_senderSockets.clear( );

    // Drop any netdevices, interfaces, and nodes
    //    (they are smart pointers, but clearing them makes ownership explicit)
    m_devices    = NetDeviceContainer( );     // just replace with empty container
    m_interfaces = Ipv4InterfaceContainer( ); // likewise
    m_nodes      = NodeContainer( );          // likewise

    // Clear out all std::vectors, sets, etc.
    m_neighbors.clear( );
    m_positions.clear( );
    m_neighborsSubset.clear( );
    m_nodeCoverageSets.clear( );
    m_receivedPackets.clear( );
    m_dataSizesScaled.clear( );
    m_sensorCoverage.clear( );
    m_areaCoverage.clear( );
    m_coverageSteps.clear( );

    m_sensorCoverageBitset.clear( );
    m_areaCoverageBitset.clear( );
    m_aggregatedSensorsBitset.clear( );
    m_aggregatedAreasBitset.clear( );
    m_neighborSensorBitset.clear( );
    m_neighborAreaBitset.clear( );

    // Clear out sets of sensor types and areas
    m_sensorTypes.clear( );
    m_areas.clear( );

    NS_LOG_INFO( "AdhocNetwork destroyed" );
}

//
// setup: create nodes, install WiFi, IP stack, and receivers, then initialize node coverage.
//
void AdhocNetwork::setup( )
{
    NS_LOG_INFO( "Setting up AdhocNetwork" );
    NS_LOG_INFO( "Creating " << m_numNodes << " nodes" );
    m_nodes.Create( m_numNodes );

    NS_LOG_INFO( "Setting up WiFi" );
    WifiHelper wifi;
    wifi.SetStandard( m_wifiStandard );
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default( );
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel( wifiChannel.Create( ) );
    WifiMacHelper wifiMac;
    wifiMac.SetType( m_macType );
    m_devices = wifi.Install( wifiPhy, wifiMac, m_nodes );
    NS_LOG_INFO( "WiFi installed" );

    initializeRandomPositions( 0.0, m_gridX, 0.0, m_gridY );

    NS_LOG_INFO( "Installing Internet stack" );
    InternetStackHelper internet;
    internet.Install( m_nodes );
    NS_LOG_INFO( "Internet stack installed" );

    NS_LOG_INFO( "Assigning IPv4 addresses" );
    Ipv4AddressHelper ipv4;
    ipv4.SetBase( m_ipBase.c_str( ), "255.255.255.0" );
    m_interfaces = ipv4.Assign( m_devices );
    NS_LOG_INFO( "IPv4 addresses assigned" );

    // NS_LOG_INFO( "Enabling pcap" );
    // wifiPhy.EnablePcapAll( "server-debug", false );
    // NS_LOG_INFO( "Pcap enabled" );

    NS_LOG_INFO( "Setting up data receivers" );
    for ( uint32_t i = 0; i < m_nodes.GetN( ); ++i )
    {
        NS_LOG_INFO( "Setting up data receiver for node " << i );
        Ptr<Node> node  = m_nodes.Get( i );
        uint32_t nodeId = node->GetId( );
        SetupDataReceiver( node, nodeId );
    }
    NS_LOG_INFO( "Data receivers set up" );

    InitializeNodeCoverageSets( );
}

//
// findNeighbors: determine which nodes are within communication range.
//
void AdhocNetwork::findNeighbors( uint32_t nodeId )
{
    NS_LOG_INFO( "Starting neighbor discovery for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
    m_neighbors.at( nodeId ).clear( );

    // Get positions for all nodes
    for ( uint32_t i = 0; i < m_numNodes; ++i )
    {
        Ptr<MobilityModel> mobility = m_nodes.Get( i )->GetObject<MobilityModel>( );
        if ( mobility )
        {
            m_positions.at( i ) = mobility->GetPosition( );
        }
        else
        {
            NS_LOG_WARN( "Node " << i << " does not have a MobilityModel." );
        }
    }
    // Check distance and add neighbors
    for ( uint32_t j = 0; j < m_numNodes; ++j )
    {
        if ( nodeId != j )
        {
            double distance = CalculateDistance( m_positions.at( nodeId ), m_positions.at( j ) );
            if ( distance <= m_communicationRange )
            {
                m_neighbors.at( nodeId ).emplace_back( m_nodes.Get( j ) );
                NS_LOG_DEBUG( "Node " << nodeId << " is a neighbor of Node " << j << " (Distance: " << distance << ")" );
            }
        }
    }
    NS_LOG_INFO( "Neighbor discovery completed for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
}

//
// findNeighborsSubset: choose a random subset of neighbors.
//
void AdhocNetwork::findNeighborsSubset( uint32_t nodeId )
{
    NS_LOG_INFO( "Starting subset neighbor selection for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
    m_neighborsSubset.at( nodeId ).clear( );

    Ptr<UniformRandomVariable> randomVar = CreateObject<UniformRandomVariable>( );
    std::vector<Ptr<Node>> neighbors     = m_neighbors.at( nodeId );
    if ( neighbors.empty( ) )
    {
        NS_LOG_WARN( "Node " << nodeId << " has no neighbors." );
        return;
    }
    uint32_t numNeighbors = neighbors.size( );
    uint32_t subsetSize   = m_gossipGroupSize;
    subsetSize            = ( subsetSize > 0 ) ? subsetSize : 1;
    subsetSize            = std::min( subsetSize, numNeighbors );
    std::unordered_set<uint32_t> selectedIndices;
    while ( selectedIndices.size( ) < subsetSize )
    {
        uint32_t randomIndex = randomVar->GetInteger( 0, numNeighbors - 1 );
        selectedIndices.insert( randomIndex );
    }
    for ( uint32_t index : selectedIndices )
    {
        m_neighborsSubset.at( nodeId ).emplace_back( neighbors.at( index ) );
    }
    NS_LOG_DEBUG( "Node " << nodeId << " selected " << subsetSize << " neighbors." );
    NS_LOG_INFO( "Subset neighbor selection completed for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
}

//
// scheduleFindNeighbors: schedule periodic neighbor discovery.
//
void AdhocNetwork::scheduleFindNeighbors( double interval ) { Simulator::Schedule( Seconds( interval ), &AdhocNetwork::m_findNeighborsCallback, this, interval ); }

//
// m_findNeighborsCallback: a callback to update neighbors and schedule packet sending.
//
void AdhocNetwork::m_findNeighborsCallback( double interval )
{
    std::cout << "Finding neighbors" << std::endl;
    findNeighbors( 0 );
    std::cout << "Neighbors found" << std::endl;
    for ( uint32_t i = 0; i < m_nodes.GetN( ); ++i )
    {
        Ptr<Node> node                   = m_nodes.Get( i );
        std::vector<Ptr<Node>> neighbors = m_neighbors.at( i );
        Simulator::Schedule( Seconds( 1.0 ), &AdhocNetwork::SendPacketsHelper, this, node, i, neighbors );
    }
}

//
// initializeRandomPositions: assign random positions to nodes.
//
void AdhocNetwork::initializeRandomPositions( double xMin, double xMax, double yMin, double yMax )
{
    NS_LOG_INFO( "Initializing random positions for nodes." );
    RngSeedManager::SetSeed( time( NULL ) );
    Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>( );
    x->SetAttribute( "Min", DoubleValue( xMin ) );
    x->SetAttribute( "Max", DoubleValue( xMax ) );
    Ptr<UniformRandomVariable> y = CreateObject<UniformRandomVariable>( );
    y->SetAttribute( "Min", DoubleValue( yMin ) );
    y->SetAttribute( "Max", DoubleValue( yMax ) );
    MobilityHelper mobility;
    mobility.SetPositionAllocator( m_positionAllocator, "X", PointerValue( x ), "Y", PointerValue( y ) );
    mobility.SetMobilityModel( "ns3::ConstantPositionMobilityModel" );
    mobility.Install( m_nodes );
    NS_LOG_INFO( "Initialized random positions for nodes." );
}

//
// GetSenderSocket: return or create a sender socket between two nodes.
//
Ptr<Socket> AdhocNetwork::GetSenderSocket( uint32_t senderId, uint32_t receiverId )
{
    std::pair<uint32_t, uint32_t> link = { senderId, receiverId };
    Ptr<Node> senderNode               = m_nodes.Get( senderId );
    Ptr<Node> receiverNode             = m_nodes.Get( receiverId );
    auto it                            = m_senderSockets.find( link );
    if ( it != m_senderSockets.end( ) )
    {
        return it->second;
    }
    else
    {
        Ptr<Socket> senderSocket     = Socket::CreateSocket( senderNode, UdpSocketFactory::GetTypeId( ) );
        Ipv4Address receiverAddress  = receiverNode->GetObject<Ipv4>( )->GetAddress( 1, 0 ).GetLocal( );
        InetSocketAddress remoteAddr = InetSocketAddress( receiverAddress, 8000 + receiverId );
        if ( senderSocket->Connect( remoteAddr ) == -1 )
        {
            NS_LOG_ERROR( "Failed to connect Socket from Node " << senderId << " to Node " << receiverId );
            return nullptr;
        }
        m_senderSockets[link] = senderSocket;
        return senderSocket;
    }
}

//
// SendPackets: schedule packet sending from one node to a set of neighbors.
//
void AdhocNetwork::SendPackets( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors )
{
    std::set<std::pair<uint32_t, uint32_t>> coverageSet = m_nodeCoverageSets.at( senderId );
    uint32_t dataSize                                   = 1024;
    for ( Ptr<Node> receiverNode : neighbors )
    {
        uint32_t receiverId      = receiverNode->GetId( );
        Ptr<Socket> senderSocket = GetSenderSocket( senderId, receiverId );
        if ( !senderSocket )
        {
            continue;
        }
        Simulator::Schedule( MilliSeconds( 1000 ), [this, senderSocket, senderId, receiverId, coverageSet, dataSize]( ) {
            if ( !senderSocket )
            {
                NS_LOG_ERROR( "senderSocket is null in scheduled lambda for Node " << senderId );
                return;
            }
            Ptr<Packet> packet = Create<Packet>( );
            GossipHeader gossipHeader( senderId, coverageSet, dataSize );
            packet->AddHeader( gossipHeader );
            NS_LOG_INFO( "Packet size: " << packet->GetSize( ) );
            m_dataSizesScaled[senderId] = ( static_cast<double>( packet->GetSize( ) ) / m_maximumDataSize ) * ( m_sensorTypes.size( ) + m_areas.size( ) );
            if ( senderSocket->Send( packet ) == -1 )
            {
                NS_LOG_ERROR( "Failed to send packet from Node " << senderId << " to Node " << receiverId );
            }
            else
            {
                NS_LOG_INFO( "Node " << senderId << " sent 1 packet to Node " << receiverId );
            }
        } );
    }
}

//
// SendPacketsHelper: a simple wrapper for SendPackets.
//
void AdhocNetwork::SendPacketsHelper( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors ) { SendPackets( senderNode, senderId, neighbors ); }

//
// SetupDataReceiver: create and bind the data receiver socket for a node.
//
void AdhocNetwork::SetupDataReceiver( Ptr<Node> node, uint32_t nodeId )
{
    Ptr<Socket> receiverSocket  = Socket::CreateSocket( node, UdpSocketFactory::GetTypeId( ) );
    uint16_t receiverPort       = 8000 + nodeId;
    InetSocketAddress localAddr = InetSocketAddress( Ipv4Address::GetAny( ), receiverPort );
    if ( receiverSocket->Bind( localAddr ) == -1 )
    {
        NS_LOG_ERROR( "Failed to bind Data Receiver Socket on Node " << nodeId << " port " << receiverPort );
        return;
    }
    receiverSocket->SetRecvCallback( MakeCallback( &AdhocNetwork::ReceivePacket, this ) );
    NS_LOG_INFO( "Data Receiver Socket bound on Node " << nodeId << " port " << receiverPort );
}

//
// GetNodeIdFromIpAddress: return the node ID associated with a given IP address.
//
uint32_t AdhocNetwork::GetNodeIdFromIpAddress( Ipv4Address address )
{
    for ( uint32_t i = 0; i < m_nodes.GetN( ); ++i )
    {
        Ipv4Address nodeAddress = m_nodes.Get( i )->GetObject<Ipv4>( )->GetAddress( 1, 0 ).GetLocal( );
        if ( nodeAddress == address )
        {
            return i;
        }
    }
    NS_LOG_WARN( "Node ID not found for IP address " << address );
    return UINT32_MAX;
}

//
// ReceivePacket: process incoming data packets.
//
void AdhocNetwork::ReceivePacket( Ptr<Socket> socket )
{
    Ptr<Packet> packet;
    Address from;
    while ( ( packet = socket->RecvFrom( from ) ) )
    {
        InetSocketAddress addr    = InetSocketAddress::ConvertFrom( from );
        Ipv4Address senderAddress = addr.GetIpv4( );
        uint32_t senderId         = GetNodeIdFromIpAddress( senderAddress );
        uint32_t receiverId       = socket->GetNode( )->GetId( );
        if ( senderId == UINT32_MAX )
        {
            NS_LOG_WARN( "Received packet from unknown sender address: " << senderAddress );
            continue;
        }
        // Remove and process the GossipHeader.
        GossipHeader gossipHeader;
        packet->RemoveHeader( gossipHeader );

        // Check if the node has received all packets from its neighbors to reset Gossip propagation.
        if ( m_receivedPackets[receiverId].size( ) == calculateNumSubNeighbors( receiverId ) )
        {
            NS_LOG_INFO( "Node " << receiverId << " has received all packets, resetting received packets" );
            m_receivedPackets[receiverId].clear( );
        }

        // Avoid processing duplicate packets.
        if ( m_receivedPackets[receiverId].find( gossipHeader.GetOriginNodeId( ) ) == m_receivedPackets[receiverId].end( ) )
        {
            m_receivedPackets[receiverId].insert( gossipHeader.GetOriginNodeId( ) );

            // Calculate Utility using the bitset–based method.
            double utility = m_calculateUtility( senderId, receiverId );
            NS_LOG_INFO( "Sender Utility of Node " << senderId << ": " << utility );

            if ( utility > 0 )
            {
                // Update the aggregated bitset.
                m_aggregatedSensorsBitset[receiverId] |= m_sensorCoverageBitset[senderId];
                m_aggregatedAreasBitset[receiverId] |= m_areaCoverageBitset[senderId];

                // Record the neighbor's bitset for future recomputation.
                std::bitset<MAX_SENSOR_TYPES> neighborSensorBitset;
                neighborSensorBitset.reset( );
                for ( uint32_t sensor : m_sensorCoverage[senderId] )
                {
                    neighborSensorBitset.set( sensor, true );
                }
                m_neighborSensorBitset[receiverId][senderId] = neighborSensorBitset;

                std::bitset<MAX_AREA_TYPES> neighborAreaBitset;
                neighborAreaBitset.reset( );
                for ( uint32_t area : m_areaCoverage[senderId] )
                {
                    neighborAreaBitset.set( area, true );
                }
                m_neighborAreaBitset[receiverId][senderId] = neighborAreaBitset;
            }
            else
            {
                // If utility is not positive, remove this neighbor's contribution from the aggregated bitset.
                // Erase the neighbor's bitset entry and recompute the aggregated bitset.
                m_neighborSensorBitset[receiverId].erase( senderId );
                m_neighborAreaBitset[receiverId].erase( senderId );
                updateAggregatedSensors( receiverId );
                updateAggregatedAreas( receiverId );
            }

            double ownUtility = m_calculateUtility( receiverId, receiverId );
            NS_LOG_INFO( "Receiver Utility of Node " << receiverId << ": " << ownUtility );
            if ( ownUtility < 0 )
            {
                m_neighborSensorBitset[receiverId].erase( receiverId );
                m_neighborAreaBitset[receiverId].erase( receiverId );
                updateAggregatedSensors( receiverId );
                updateAggregatedAreas( receiverId );
            }

            m_coverageSteps[receiverId]++;

            if ( !m_isCovered( receiverId ) || Simulator::IsFinished( ) )
            {
                NS_LOG_INFO( "Node " << receiverId << " does not cover, finding neighbors subset and sending packets" );
                Simulator::Schedule( Simulator::Now( ) + Seconds( 1.0 ), &AdhocNetwork::findNeighborsSubset, this, receiverId );
                Simulator::Schedule( Simulator::Now( ) + Seconds( 2.0 ), &AdhocNetwork::SendPackets, this, m_nodes.Get( receiverId ), receiverId, m_neighborsSubset[receiverId] );
            }
            else
            {
                NS_LOG_INFO( "Node " << receiverId << " covers" );
                NS_LOG_INFO( "Coverage Steps: " << m_coverageSteps[receiverId] );

                // Now print the final coverage:
                PrintFinalCoverage( receiverId );
                m_isCoverageReached = true;

                Simulator::Stop( Seconds( Simulator::Now( ).GetSeconds( ) ) );
            }
        }
        else
        {
            NS_LOG_INFO( "Node " << receiverId << " received duplicate packet from Node " << senderId );
        }
    }
}

//
// InitializeNodeCoverageSets: assign intrinsic sensor and area coverage to each node and update bitset data members.
//
void AdhocNetwork::InitializeNodeCoverageSets( )
{
    // For safety if you do multiple runs or re-setup, clear old coverage first
    for ( uint32_t i = 0; i < m_numNodes; ++i )
    {
        m_sensorCoverage[i].clear( );
        m_areaCoverage[i].clear( );
        m_sensorCoverageBitset[i].reset( );
        m_areaCoverageBitset[i].reset( );
        m_aggregatedSensorsBitset[i].reset( );
        m_aggregatedAreasBitset[i].reset( );
    }

    // Now fill them from the user-provided assignments
    for ( uint32_t i = 0; i < m_numNodes; ++i )
    {
        // 1) Sensors
        for ( uint32_t sensorType : m_assignedSensors[i] )
        {
            m_sensorCoverage[i].push_back( sensorType );
            m_sensorCoverageBitset[i].set( sensorType, true );
            m_aggregatedSensorsBitset[i].set( sensorType, true );
        }

        // 2) Exactly one area for each node in round-robin
        // (the user already assigned it in nodeAreas)
        uint32_t area = m_assignedAreas[i];
        m_areaCoverage[i].push_back( area );
        m_areaCoverageBitset[i].set( area, true );
        m_aggregatedAreasBitset[i].set( area, true );

        // Build the intrinsic coverage set
        std::set<std::pair<uint32_t, uint32_t>> coverageSet;
        for ( uint32_t sensor : m_sensorCoverage[i] )
        {
            // node has exactly 1 area in the assigned array
            coverageSet.insert( { sensor, area } );
        }
        m_nodeCoverageSets[i] = coverageSet;

        NS_LOG_INFO( "Node " << i << " coverage set:" );
        for ( auto& pair : coverageSet )
        {
            NS_LOG_INFO( "  (" << pair.first << ", " << pair.second << ")" );
        }
    }
}

//
// SetCoverage: return the total number of sensor types and areas.
//
std::pair<uint32_t, uint32_t> AdhocNetwork::SetCoverage( uint32_t nodeId ) { return std::make_pair( m_sensorTypes.size( ), m_areas.size( ) ); }

//
// m_isCovered: check if the aggregated local coverage covers all sensor types and areas.
//
bool AdhocNetwork::m_isCovered( uint32_t receiverId )
{
    // Check that the number of sensor types in the aggregated sensor bitset equals the total number of sensor types.
    bool sensorsCovered = ( m_aggregatedSensorsBitset[receiverId].count( ) == m_sensorTypes.size( ) );
    // Check that the number of area types in the aggregated area bitset equals the total number of area types.
    bool areasCovered = ( m_aggregatedAreasBitset[receiverId].count( ) == m_areas.size( ) );
    return sensorsCovered && areasCovered;
}

//
// m_calculateUtility: compute utility using bitset differences.
//
double AdhocNetwork::m_calculateUtility( uint32_t senderId, uint32_t receiverId )
{
    std::bitset<MAX_SENSOR_TYPES> newSensorBits = m_sensorCoverageBitset[senderId] & ~( m_aggregatedSensorsBitset[receiverId] );
    int newSensors                              = newSensorBits.count( );
    NS_LOG_INFO( "New sensors: " << newSensors );
    std::bitset<MAX_AREA_TYPES> newAreaBits = m_areaCoverageBitset[senderId] & ~( m_aggregatedAreasBitset[receiverId] );
    int newAreas                            = newAreaBits.count( );
    NS_LOG_INFO( "New areas: " << newAreas );

    return m_alpha * newSensors + m_beta * newAreas - m_lambda * m_dataSizesScaled[senderId];
}

//
// getMinCoverage: return the minimum number of coverage steps among nodes.
//
int AdhocNetwork::getMaxCoverage( )
{
    int maxSteps          = std::numeric_limits<int>::min( );
    bool foundAnyPositive = false;
    for ( uint32_t i = 0; i < m_numNodes; i++ )
    {
        int steps = m_coverageSteps[i];
        if ( steps > 0 )
        {
            foundAnyPositive = true;
            if ( steps > maxSteps )
                maxSteps = steps;
        }
    }
    if ( !foundAnyPositive )
    {
        NS_LOG_WARN( "No node had a positive coverageSteps! All zero or never covered?" );
        return 0;
    }
    return maxSteps;
}

//
// updateAggregatedSensors: helper function to recalculate the aggregated sensor bitset for receiverId.
//
void AdhocNetwork::updateAggregatedSensors( uint32_t receiverId )
{
    std::bitset<MAX_SENSOR_TYPES> aggregated;
    aggregated = m_sensorCoverageBitset[receiverId]; // Start with intrinsic coverage.
    for ( const auto& pair : m_neighborSensorBitset[receiverId] )
    {
        aggregated |= pair.second;
    }
    m_aggregatedSensorsBitset[receiverId] = aggregated;
}

//
// updateAggregatedAreas: helper function to recalculate the aggregated area bitset for receiverId.
//
void AdhocNetwork::updateAggregatedAreas( uint32_t receiverId )
{
    std::bitset<MAX_AREA_TYPES> aggregated;
    aggregated = m_areaCoverageBitset[receiverId]; // Start with intrinsic coverage.
    for ( const auto& pair : m_neighborAreaBitset[receiverId] )
    {
        aggregated |= pair.second;
    }
    m_aggregatedAreasBitset[receiverId] = aggregated;
}

void AdhocNetwork::PrintFinalCoverage( uint32_t nodeId ) const
{
    // Convert the aggregatedSensorsBitset[nodeId] to a list of sensor IDs:
    std::vector<uint32_t> coveredSensors;
    for ( size_t s = 0; s < MAX_SENSOR_TYPES; s++ )
    {
        if ( m_aggregatedSensorsBitset[nodeId].test( s ) )
        {
            coveredSensors.push_back( static_cast<uint32_t>( s ) );
        }
    }

    // Convert the aggregatedAreasBitset[nodeId] to a list of area IDs:
    std::vector<uint32_t> coveredAreas;
    for ( size_t a = 0; a < MAX_AREA_TYPES; a++ )
    {
        if ( m_aggregatedAreasBitset[nodeId].test( a ) )
        {
            coveredAreas.push_back( static_cast<uint32_t>( a ) );
        }
    }

    NS_LOG_INFO( "=== Final Coverage for Node " << nodeId << " ===" );
    NS_LOG_INFO( "Sensors covered: " );
    for ( auto s : coveredSensors )
    {
        NS_LOG_INFO( "  Sensor ID: " << s );
    }
    NS_LOG_INFO( "Areas covered: " );
    for ( auto a : coveredAreas )
    {
        NS_LOG_INFO( "  Area ID: " << a );
    }
}

uint32_t AdhocNetwork::calculateNumSubNeighbors( uint32_t nodeId )
{
    uint32_t numSubNeighbors = 0;
    for ( uint32_t i = 0; i < m_nodes.GetN( ); ++i )
    {
        for ( int j = 0; j < m_neighborsSubset[i].size( ); ++j )
        {
            if ( m_neighborsSubset[i][j]->GetId( ) == nodeId )
                numSubNeighbors++;
        }
    }
    return numSubNeighbors;
}

bool AdhocNetwork::IsCoverageReached( ) { return m_isCoverageReached; }