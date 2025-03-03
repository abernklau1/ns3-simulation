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
    : _numNodes( numNodes ),
      _sensorTypes( sensorTypes ),
      _areas( areaTypes ),
      _wifiStandard( wifiStandard ),
      _macType( macType ),
      _ipBase( ipBase ),
      _positionAllocator( positionAllocator ),
      _communicationRange( communicationRange ),
      _gridX( gridX ),
      _gridY( gridY ),
      _assignedSensors( nodeSensors ),
      _assignedAreas( nodeAreas )
{
    NS_LOG_INFO( "Creating AdhocNetwork with " << numNodes << " nodes" );

    // Resize existing containers
    _neighbors.resize( numNodes );
    _positions.resize( numNodes );
    _neighborsSubset.resize( numNodes );
    _nodeCoverageSets.resize( numNodes );
    _receivedPackets.resize( numNodes );
    _dataSizesScaled.resize( numNodes );
    _sensorCoverage.resize( numNodes );
    _areaCoverage.resize( numNodes );
    _coverageSteps.resize( numNodes );

    // Default parameters
    _gossipGroupSize   = 3;
    _alpha             = 0.5;
    _beta              = 0.5;
    _lambda            = 1;
    _maximumDataSize   = 2048;
    _isCoverageReached = false;
    _coveredSteps      = 0;

    // Resize new bitset containers
    _sensorCoverageBitset.resize( numNodes );
    _areaCoverageBitset.resize( numNodes );
    _aggregatedSensorsBitset.resize( numNodes );
    _aggregatedAreasBitset.resize( numNodes );
    _neighborSensorBitset.resize( numNodes );
    _neighborAreaBitset.resize( numNodes );
    NS_LOG_INFO( "AdhocNetwork created" );
}

//
// Destructor
//
AdhocNetwork::~AdhocNetwork( )
{
    NS_LOG_INFO( "Destroying AdhocNetwork" );

    // Close and clear all sender sockets
    for ( auto& [link, socket] : _senderSockets )
    {
        if ( socket )
        {
            socket->Close( );
        }
    }
    _senderSockets.clear( );

    // Drop any netdevices, interfaces, and nodes
    //    (they are smart pointers, but clearing them makes ownership explicit)
    _devices    = NetDeviceContainer( );     // just replace with empty container
    _interfaces = Ipv4InterfaceContainer( ); // likewise
    _nodes      = NodeContainer( );          // likewise

    // Clear out all std::vectors, sets, etc.
    _neighbors.clear( );
    _positions.clear( );
    _neighborsSubset.clear( );
    _nodeCoverageSets.clear( );
    _receivedPackets.clear( );
    _dataSizesScaled.clear( );
    _sensorCoverage.clear( );
    _areaCoverage.clear( );
    _coverageSteps.clear( );

    _sensorCoverageBitset.clear( );
    _areaCoverageBitset.clear( );
    _aggregatedSensorsBitset.clear( );
    _aggregatedAreasBitset.clear( );
    _neighborSensorBitset.clear( );
    _neighborAreaBitset.clear( );

    // Clear out sets of sensor types and areas
    _sensorTypes.clear( );
    _areas.clear( );

    NS_LOG_INFO( "AdhocNetwork destroyed" );
}

// ------------------------------------------------------------------------------------------------
// Setup and Initialization
// ------------------------------------------------------------------------------------------------

//
// setup: create nodes, install WiFi, IP stack, and receivers, then initialize node coverage.
//
void AdhocNetwork::setup( )
{
    NS_LOG_INFO( "Setting up AdhocNetwork" );
    NS_LOG_INFO( "Creating " << _numNodes << " nodes" );
    _nodes.Create( _numNodes );

    NS_LOG_INFO( "Setting up WiFi" );
    WifiHelper wifi;
    wifi.SetStandard( _wifiStandard );
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default( );
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel( wifiChannel.Create( ) );
    WifiMacHelper wifiMac;
    wifiMac.SetType( _macType );
    _devices = wifi.Install( wifiPhy, wifiMac, _nodes );
    NS_LOG_INFO( "WiFi installed" );

    initializeRandomPositions( 0.0, _gridX, 0.0, _gridY );

    NS_LOG_INFO( "Installing Internet stack" );
    InternetStackHelper internet;
    internet.Install( _nodes );
    NS_LOG_INFO( "Internet stack installed" );

    NS_LOG_INFO( "Assigning IPv4 addresses" );
    Ipv4AddressHelper ipv4;
    ipv4.SetBase( _ipBase.c_str( ), "255.255.255.0" );
    _interfaces = ipv4.Assign( _devices );
    NS_LOG_INFO( "IPv4 addresses assigned" );

    // NS_LOG_INFO( "Enabling pcap" );
    // wifiPhy.EnablePcapAll( "server-debug", false );
    // NS_LOG_INFO( "Pcap enabled" );

    NS_LOG_INFO( "Setting up data receivers" );
    for ( uint32_t i = 0; i < _nodes.GetN( ); ++i )
    {
        NS_LOG_INFO( "Setting up data receiver for node " << i );
        Ptr<Node> node  = _nodes.Get( i );
        uint32_t nodeId = node->GetId( );
        setupDataReceiver( node, nodeId );
    }
    NS_LOG_INFO( "Data receivers set up" );

    initializeNodeCoverageSets( );
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
    mobility.SetPositionAllocator( _positionAllocator, "X", PointerValue( x ), "Y", PointerValue( y ) );
    mobility.SetMobilityModel( "ns3::ConstantPositionMobilityModel" );
    mobility.Install( _nodes );
    NS_LOG_INFO( "Initialized random positions for nodes." );
}

//
// InitializeNodeCoverageSets: assign intrinsic sensor and area coverage to each node and update bitset data members.
//
void AdhocNetwork::initializeNodeCoverageSets( )
{
    // Clear old sensor and area coverage data for all nodes.
    for ( uint32_t i = 0; i < _numNodes; ++i )
    {
        _sensorCoverage[i].clear( );
        _areaCoverage[i].clear( );
        _sensorCoverageBitset[i].reset( );
        _areaCoverageBitset[i].reset( );
        _aggregatedSensorsBitset[i].reset( );
        _aggregatedAreasBitset[i].reset( );
    }

    // For each node, use the pre-assigned sensors and area as provided.
    for ( uint32_t i = 0; i < _numNodes; ++i )
    {
        // Set up the intrinsic sensor coverage for node i using _assignedSensors.
        for ( uint32_t sensorType : _assignedSensors[i] )
        {
            _sensorCoverage[i].push_back( sensorType );
            _sensorCoverageBitset[i].set( sensorType, true );
            _aggregatedSensorsBitset[i].set( sensorType, true );
        }

        // For the area assignment, use the pre-assigned area (exactly one per node).
        uint32_t area = _assignedAreas[i];
        _areaCoverage[i].push_back( area );
        _areaCoverageBitset[i].set( area, true );
        _aggregatedAreasBitset[i].set( area, true );

        // Build the intrinsic coverage set as sensor-area pairs.
        std::set<std::pair<uint32_t, uint32_t>> coverageSet;
        for ( uint32_t sensor : _sensorCoverage[i] )
        {
            coverageSet.insert( { sensor, area } );
        }
        _nodeCoverageSets[i] = coverageSet;

        NS_LOG_INFO( "Node " << i << " coverage set:" );
        for ( auto& pair : coverageSet )
        {
            NS_LOG_INFO( "  (" << pair.first << ", " << pair.second << ")" );
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Neighbor Discovery and Selection
// ------------------------------------------------------------------------------------------------

//
// findNeighbors: determine which nodes are within communication range.
//
void AdhocNetwork::findNeighbors( uint32_t nodeId )
{
    NS_LOG_INFO( "Starting neighbor discovery for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
    _neighbors.at( nodeId ).clear( );

    // TODO: Find a better place for this
    // Get positions for all nodes
    for ( uint32_t i = 0; i < _numNodes; ++i )
    {
        Ptr<MobilityModel> mobility = _nodes.Get( i )->GetObject<MobilityModel>( );
        if ( mobility )
        {
            _positions.at( i ) = mobility->GetPosition( );
        }
        else
        {
            NS_LOG_WARN( "Node " << i << " does not have a MobilityModel." );
        }
    }

    // Check distance and add neighbors
    for ( uint32_t j = 0; j < _numNodes; ++j )
    {
        if ( nodeId != j )
        {
            double distance = CalculateDistance( _positions.at( nodeId ), _positions.at( j ) );
            if ( distance <= _communicationRange )
            {
                _neighbors.at( nodeId ).emplace_back( _nodes.Get( j ) );
                NS_LOG_DEBUG( "Node " << nodeId << " is a neighbor of Node " << j << " (Distance: " << distance << ")" );
            }
        }
    }
    NS_LOG_INFO( "Neighbor discovery completed for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );

    /* TODO: Find a better place for this
     * Currently, this needs to occur here because the positions vector isn't populated until this function is called in the simulation.
     * One solution could be to grab the positions straight from the node's mobility model, but that would still require this to be called after initializing the positions,
     * either randomly -- on the first subrun of a run -- or through the node_positions file.
     * Another solution could be to just call this after the simulation runs completely, but I can't remember if I have a vector that holds only the initial coverage or not.
     */
    printNodeInfoToFile( "stats/node_info.txt" );
}

//
// findNeighborsSubset: choose a random subset of neighbors.
//
void AdhocNetwork::findNeighborsSubset( uint32_t nodeId )
{
    NS_LOG_INFO( "Starting subset neighbor selection for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
    _neighborsSubset.at( nodeId ).clear( );

    Ptr<UniformRandomVariable> randomVar = CreateObject<UniformRandomVariable>( );
    std::vector<Ptr<Node>> neighbors     = _neighbors.at( nodeId );
    if ( neighbors.empty( ) )
    {
        NS_LOG_WARN( "Node " << nodeId << " has no neighbors." );
        return;
    }
    uint32_t numNeighbors = neighbors.size( );
    uint32_t subsetSize   = _gossipGroupSize;
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
        _neighborsSubset.at( nodeId ).emplace_back( neighbors.at( index ) );
    }
    NS_LOG_DEBUG( "Node " << nodeId << " selected " << subsetSize << " neighbors." );
    NS_LOG_INFO( "Subset neighbor selection completed for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
}

//
// scheduleFindNeighbors: schedule periodic neighbor discovery.
//
void AdhocNetwork::scheduleFindNeighbors( double interval ) { Simulator::Schedule( Seconds( interval ), &AdhocNetwork::findNeighborsCallback, this, interval ); }

//
// findNeighborsCallback: a callback to update neighbors and schedule packet sending.
//
void AdhocNetwork::findNeighborsCallback( double interval )
{
    std::cout << "Finding neighbors" << std::endl;
    findNeighbors( 0 );
    std::cout << "Neighbors found" << std::endl;
    for ( uint32_t i = 0; i < _nodes.GetN( ); ++i )
    {
        Ptr<Node> node                   = _nodes.Get( i );
        std::vector<Ptr<Node>> neighbors = _neighbors.at( i );
        Simulator::Schedule( Seconds( 1.0 ), &AdhocNetwork::sendPacketsHelper, this, node, i, neighbors );
    }
}

// ------------------------------------------------------------------------------------------------
// Packet Sending/Receiving (Gossip Communication)
// ------------------------------------------------------------------------------------------------

//
// GetSenderSocket: return or create a sender socket between two nodes.
//
Ptr<Socket> AdhocNetwork::getSenderSocket( uint32_t senderId, uint32_t receiverId )
{
    std::pair<uint32_t, uint32_t> link = { senderId, receiverId };
    Ptr<Node> senderNode               = _nodes.Get( senderId );
    Ptr<Node> receiverNode             = _nodes.Get( receiverId );
    auto it                            = _senderSockets.find( link );
    if ( it != _senderSockets.end( ) )
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
        _senderSockets[link] = senderSocket;
        return senderSocket;
    }
}

//
// SendPackets: schedule packet sending from one node to a set of neighbors.
//
void AdhocNetwork::sendPackets( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors )
{
    std::set<std::pair<uint32_t, uint32_t>> coverageSet = _nodeCoverageSets.at( senderId );
    uint32_t dataSize                                   = 1024;
    for ( Ptr<Node> receiverNode : neighbors )
    {
        uint32_t receiverId      = receiverNode->GetId( );
        Ptr<Socket> senderSocket = getSenderSocket( senderId, receiverId );
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
            _dataSizesScaled[senderId] = ( static_cast<double>( packet->GetSize( ) ) / _maximumDataSize ) * ( _assignedSensors[senderId].size( ) + 1 /* Initial area covered by the node */ );
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
// sendPacketsHelper: a simple wrapper for sendPackets.
//
void AdhocNetwork::sendPacketsHelper( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors ) { sendPackets( senderNode, senderId, neighbors ); }

//
// setupDataReceiver: create and bind the data receiver socket for a node.
//
void AdhocNetwork::setupDataReceiver( Ptr<Node> node, uint32_t nodeId )
{
    Ptr<Socket> receiverSocket  = Socket::CreateSocket( node, UdpSocketFactory::GetTypeId( ) );
    uint16_t receiverPort       = 8000 + nodeId;
    InetSocketAddress localAddr = InetSocketAddress( Ipv4Address::GetAny( ), receiverPort );
    if ( receiverSocket->Bind( localAddr ) == -1 )
    {
        NS_LOG_ERROR( "Failed to bind Data Receiver Socket on Node " << nodeId << " port " << receiverPort );
        return;
    }
    receiverSocket->SetRecvCallback( MakeCallback( &AdhocNetwork::receivePacket, this ) );
    NS_LOG_INFO( "Data Receiver Socket bound on Node " << nodeId << " port " << receiverPort );
}

//
// getNodeIdFromIpAddress: return the node ID associated with a given IP address.
//
uint32_t AdhocNetwork::getNodeIdFromIpAddress( Ipv4Address address )
{
    for ( uint32_t i = 0; i < _nodes.GetN( ); ++i )
    {
        Ipv4Address nodeAddress = _nodes.Get( i )->GetObject<Ipv4>( )->GetAddress( 1, 0 ).GetLocal( );
        if ( nodeAddress == address )
        {
            return i;
        }
    }
    NS_LOG_WARN( "Node ID not found for IP address " << address );
    return UINT32_MAX;
}

//
// receivePacket: process incoming data packets.
//
void AdhocNetwork::receivePacket( Ptr<Socket> socket )
{
    Ptr<Packet> packet;
    Address from;
    while ( ( packet = socket->RecvFrom( from ) ) )
    {
        InetSocketAddress addr    = InetSocketAddress::ConvertFrom( from );
        Ipv4Address senderAddress = addr.GetIpv4( );
        uint32_t senderId         = getNodeIdFromIpAddress( senderAddress );
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
        if ( _receivedPackets[receiverId].size( ) == calculateNumSubNeighbors( receiverId ) )
        {
            NS_LOG_INFO( "Node " << receiverId << " has received all packets, resetting received packets" );
            _receivedPackets[receiverId].clear( );
        }

        // Avoid processing duplicate packets.
        if ( _receivedPackets[receiverId].find( gossipHeader.GetOriginNodeId( ) ) == _receivedPackets[receiverId].end( ) )
        {
            _receivedPackets[receiverId].insert( gossipHeader.GetOriginNodeId( ) );

            // Calculate Utility using the bitset–based method.
            double utility = calculateUtility( senderId, receiverId );
            NS_LOG_INFO( "Sender Utility of Node " << senderId << ": " << utility );

            if ( utility > 0 )
            {
                // Update the aggregated bitset.
                _aggregatedSensorsBitset[receiverId] |= _sensorCoverageBitset[senderId];
                _aggregatedAreasBitset[receiverId] |= _areaCoverageBitset[senderId];

                // Record the neighbor's bitset for future recomputation.
                std::bitset<MAX_SENSOR_TYPES> neighborSensorBitset;
                neighborSensorBitset.reset( );
                for ( uint32_t sensor : _sensorCoverage[senderId] )
                {
                    neighborSensorBitset.set( sensor, true );
                }
                _neighborSensorBitset[receiverId][senderId] = neighborSensorBitset;

                std::bitset<MAX_AREA_TYPES> neighborAreaBitset;
                neighborAreaBitset.reset( );
                for ( uint32_t area : _areaCoverage[senderId] )
                {
                    neighborAreaBitset.set( area, true );
                }
                _neighborAreaBitset[receiverId][senderId] = neighborAreaBitset;
            }
            else
            {
                // If utility is not positive, remove this neighbor's contribution from the aggregated bitset.
                // Erase the neighbor's bitset entry and recompute the aggregated bitset.
                _neighborSensorBitset[receiverId].erase( senderId );
                _neighborAreaBitset[receiverId].erase( senderId );
                updateAggregatedSensors( receiverId );
                updateAggregatedAreas( receiverId );
            }

            double ownUtility = calculateUtility( receiverId, receiverId );
            NS_LOG_INFO( "Receiver Utility of Node " << receiverId << ": " << ownUtility );
            if ( ownUtility < 0 )
            {
                _neighborSensorBitset[receiverId].erase( receiverId );
                _neighborAreaBitset[receiverId].erase( receiverId );
                updateAggregatedSensors( receiverId );
                updateAggregatedAreas( receiverId );
            }

            _coverageSteps[receiverId]++;

            if ( !isCovered( receiverId ) || Simulator::IsFinished( ) )
            {
                NS_LOG_INFO( "Node " << receiverId << " does not cover, finding neighbors subset and sending packets" );
                Simulator::Schedule( Simulator::Now( ) + Seconds( 1.0 ), &AdhocNetwork::findNeighborsSubset, this, receiverId );
                Simulator::Schedule( Simulator::Now( ) + Seconds( 2.0 ), &AdhocNetwork::sendPackets, this, _nodes.Get( receiverId ), receiverId, _neighborsSubset[receiverId] );
            }
            else
            {
                NS_LOG_INFO( "Node " << receiverId << " covers" );
                NS_LOG_INFO( "Coverage Steps: " << _coverageSteps[receiverId] );
                _coveredSteps = _coverageSteps[receiverId];

                // Now print the final coverage:
                printFinalCoverage( receiverId );
                _isCoverageReached = true;

                Simulator::Stop( Seconds( Simulator::Now( ).GetSeconds( ) ) );
            }
        }
        else
        {
            NS_LOG_INFO( "Node " << receiverId << " received duplicate packet from Node " << senderId );
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Coverage and Utility Functions
// ------------------------------------------------------------------------------------------------

//
// SetCoverage: return the total number of sensor types and areas.
//
std::pair<uint32_t, uint32_t> AdhocNetwork::setCoverage( uint32_t nodeId ) { return std::make_pair( _sensorTypes.size( ), _areas.size( ) ); }

//
// isCovered: check if the aggregated local coverage covers all sensor types and areas.
//
bool AdhocNetwork::isCovered( uint32_t receiverId )
{
    // Check that the number of sensor types in the aggregated sensor bitset equals the total number of sensor types.
    bool sensorsCovered = ( _aggregatedSensorsBitset[receiverId].count( ) == _sensorTypes.size( ) );
    // Check that the number of area types in the aggregated area bitset equals the total number of area types.
    bool areasCovered = ( _aggregatedAreasBitset[receiverId].count( ) == _areas.size( ) );
    return sensorsCovered && areasCovered;
}

//
// calculateUtility: compute utility using bitset differences.
//
double AdhocNetwork::calculateUtility( uint32_t senderId, uint32_t receiverId )
{
    std::bitset<MAX_SENSOR_TYPES> newSensorBits = _sensorCoverageBitset[senderId] & ~( _aggregatedSensorsBitset[receiverId] );
    int newSensors                              = newSensorBits.count( );
    NS_LOG_INFO( "New sensors: " << newSensors );
    std::bitset<MAX_AREA_TYPES> newAreaBits = _areaCoverageBitset[senderId] & ~( _aggregatedAreasBitset[receiverId] );
    int newAreas                            = newAreaBits.count( );
    NS_LOG_INFO( "New areas: " << newAreas );

    return _alpha * newSensors + _beta * newAreas - _lambda * _dataSizesScaled[senderId];
}

//
// updateAggregatedSensors: helper function to recalculate the aggregated sensor bitset for receiverId.
//
void AdhocNetwork::updateAggregatedSensors( uint32_t receiverId )
{
    std::bitset<MAX_SENSOR_TYPES> aggregated;
    aggregated = _sensorCoverageBitset[receiverId]; // Start with intrinsic coverage.
    for ( const auto& pair : _neighborSensorBitset[receiverId] )
    {
        aggregated |= pair.second;
    }
    _aggregatedSensorsBitset[receiverId] = aggregated;
}

//
// updateAggregatedAreas: helper function to recalculate the aggregated area bitset for receiverId.
//
void AdhocNetwork::updateAggregatedAreas( uint32_t receiverId )
{
    std::bitset<MAX_AREA_TYPES> aggregated;
    aggregated = _areaCoverageBitset[receiverId]; // Start with intrinsic coverage.
    for ( const auto& pair : _neighborAreaBitset[receiverId] )
    {
        aggregated |= pair.second;
    }
    _aggregatedAreasBitset[receiverId] = aggregated;
}

uint32_t AdhocNetwork::calculateNumSubNeighbors( uint32_t nodeId )
{
    uint32_t numSubNeighbors = 0;
    for ( uint32_t i = 0; i < _nodes.GetN( ); ++i )
    {
        for ( int j = 0; j < _neighborsSubset[i].size( ); ++j )
        {
            if ( _neighborsSubset[i][j]->GetId( ) == nodeId )
                numSubNeighbors++;
        }
    }
    return numSubNeighbors;
}

void AdhocNetwork::printFinalCoverage( uint32_t nodeId ) const
{
    // Convert the aggregatedSensorsBitset[nodeId] to a list of sensor IDs:
    std::vector<uint32_t> coveredSensors;
    for ( size_t s = 0; s < MAX_SENSOR_TYPES; s++ )
    {
        if ( _aggregatedSensorsBitset[nodeId].test( s ) )
        {
            coveredSensors.push_back( static_cast<uint32_t>( s ) );
        }
    }

    // Convert the aggregatedAreasBitset[nodeId] to a list of area IDs:
    std::vector<uint32_t> coveredAreas;
    for ( size_t a = 0; a < MAX_AREA_TYPES; a++ )
    {
        if ( _aggregatedAreasBitset[nodeId].test( a ) )
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

bool AdhocNetwork::isCoverageReached( ) { return _isCoverageReached; }

void AdhocNetwork::printNodeInfoToFile( const std::string& filename ) const
{
    std::ofstream outFile( filename );
    if ( !outFile.is_open( ) )
    {
        NS_LOG_ERROR( "Failed to open file " << filename << " for writing node information." );
        return;
    }

    outFile << "========== Node Information ==========\n";
    for ( uint32_t i = 0; i < _numNodes; ++i )
    {
        outFile << "----------------------------------------\n";
        outFile << "Node ID: " << i << "\n";

        // Print node position
        if ( i < _positions.size( ) )
        {
            outFile << "Position: (" << _positions[i].x << ", " << _positions[i].y << ")\n";
        }

        // Print intrinsic coverage set (sensor–area pairs)
        outFile << "Intrinsic Coverage Set: ";
        for ( auto pair : _nodeCoverageSets[i] )
        {
            outFile << "(" << pair.first << ", " << pair.second << ") ";
        }
        outFile << "\n";

        // Print neighbor information
        outFile << "Discovered Neighbors: ";
        for ( auto neighbor : _neighbors[i] )
        {
            outFile << neighbor->GetId( ) << " ";
        }
        outFile << "\n";
    }
    outFile << "========== End of Node Information ==========\n";
    outFile.close( );
}