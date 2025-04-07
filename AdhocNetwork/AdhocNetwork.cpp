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
    _intrinsicCoverageSets.resize( numNodes );
    _receivedPackets.resize( numNodes );
    _dataSizesScaled.resize( numNodes );
    _sensorCoverage.resize( numNodes );
    _areaCoverage.resize( numNodes );
    _localView.resize( numNodes );
    _intrinsicUtility.resize( numNodes );

    // Default parameters
    _gossipGroupSize   = 3;
    _alpha             = 0.3;
    _beta              = 0.7;
    _lambda            = 1.0;
    _maximumDataSize   = 2048; // Size of a packet that holds a covered set: 4 * _numSensors + 4 * _numAreas + 16 + 4 * pairs;
    _isCoverageReached = false;
    _coveredSteps      = 0;
    _coveredUtility    = 0.0;
    _coveringNode      = UINT32_MAX;
    _coveringSetString.clear( );
    _roundId     = 0;
    _lastRoundId = -1;

    _coverageVersion.resize( _numNodes, 0 );
    _lastSeenVersion.resize( _numNodes );
    for ( int i = 0; i < _numNodes; i++ )
    {
        for ( int j = 0; j < _numNodes; j++ )
        {
            _lastSeenVersion[i].emplace( j, -1 );
        }
    }

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
    _intrinsicCoverageSets.clear( );
    _receivedPackets.clear( );
    _dataSizesScaled.clear( );
    _sensorCoverage.clear( );
    _areaCoverage.clear( );

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

    for ( uint32_t i = 0; i < _numNodes; ++i )
    {
        findNeighbors( i );
    }
}

//
// initializeRandomPositions: assign random positions to nodes.
//
void AdhocNetwork::initializeRandomPositions( double xMin, double xMax, double yMin, double yMax )
{
    NS_LOG_INFO( "Initializing random positions for nodes." );
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
    for ( uint32_t i = 0; i < _numNodes; ++i )
    {
        _intrinsicUtility[i] = ( static_cast<double>( 8 +
                  _assignedSensors[i].size( ) *
                  16 /* The number of initial pairs in each node's intrinsic coverage set is equivalent to the number of assigned sensors because they are initially assigned a single area, multiplied by 8 because each pair contains 8 bytes*/ ) /
              _maximumDataSize ) *
            ( _assignedSensors[i].size( ) + 1 /* Initial area covered by the node */ );
        std::set<CoverageQuad, CoverageQuadLess> coverageSet;
        for ( auto sensor : _assignedSensors[i] )
        {
            CoverageQuad c;
            c.sensor     = sensor;
            c.area       = _assignedAreas[i];
            c.originId   = i;
            c.dataScaled = _intrinsicUtility[i];
            coverageSet.insert( c );
        }
        _intrinsicCoverageSets[i] = coverageSet;
        _localView[i]             = coverageSet;
    }
}

// Start the gossip rounds (call once in main or after setup).
void AdhocNetwork::startRounds( )
{
    NS_LOG_INFO( "Starting round-based gossip with no maximum round count." );
    doRound( );
}

void AdhocNetwork::doRound( )
{
    NS_LOG_INFO( "===== Starting Round " << _roundId << " at time " << Simulator::Now( ).GetSeconds( ) << "s =====" );

    // For each node, find neighbors and pick a subset
    for ( uint32_t i = 0; i < _numNodes; ++i )
    {
        findNeighborsSubset( i );
    }

    // For each node, send its gossip messages
    for ( uint32_t i = 0; i < _numNodes; ++i )
    {
        sendPackets( _nodes.Get( i ), i, _neighborsSubset[i] );
    }

    // Wait some fixed 'roundDuration' so that packets can be delivered
    double roundDuration = 2.0;
    Simulator::Schedule( Seconds( roundDuration ), [this]( ) {
        // Check coverage after this round has “settled”
        bool anyCovered = false;
        for ( uint32_t i = 0; i < _numNodes; i++ )
        {
            if ( isCovered( i ) )
            {
                anyCovered      = true;
                _coveringNode   = i;
                _coveredUtility = computeSummedUtility( i );
                break;
            }
        }

        if ( anyCovered )
        {
            NS_LOG_INFO( "Coverage reached by the end of Round " << _roundId << " at time " << Simulator::Now( ).GetSeconds( ) << "s." );
            _coveredSteps      = _roundId;
            _isCoverageReached = true;
            std::ostringstream coveredSetStream;
            for ( const auto& quad : _localView[_coveringNode] )
            {
                coveredSetStream << "(" << quad.sensor << "," << quad.area << "," << quad.originId << ") ";
            }
            _coveringSetString = coveredSetStream.str( );
            Simulator::Stop( );
            return;
        }

        // If coverage not reached, do the next round
        _lastRoundId = _roundId;
        _roundId++;
        for ( int i = 0; i < _numNodes; i++ )
        {
            _coverageVersion[i]++;
        }
        doRound( );
    } );
}

// ------------------------------------------------------------------------------------------------
// Neighbor Discovery and Selection
// ------------------------------------------------------------------------------------------------

//
// findNeighbors: determine which nodes are within communication range.
//
void AdhocNetwork::findNeighbors( uint32_t nodeId )
{
    // NS_LOG_INFO( "Starting neighbor discovery for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
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
                // NS_LOG_DEBUG( "Node " << nodeId << " is a neighbor of Node " << j << " (Distance: " << distance << ")" );
            }
        }
    }
    // NS_LOG_INFO( "Neighbor discovery completed for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );

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
    // NS_LOG_INFO( "Starting subset neighbor selection for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
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
    // NS_LOG_DEBUG( "Node " << nodeId << " selected " << subsetSize << " neighbors." );
    // NS_LOG_INFO( "Subset neighbor selection completed for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
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
    // Send the node's local view
    std::set<CoverageQuad, CoverageQuadLess> coverageSet = _localView[senderId];
    uint32_t coverageVersion                             = _coverageVersion[senderId];
    uint32_t dataSize                                    = 1024;
    for ( Ptr<Node> receiverNode : neighbors )
    {
        uint32_t receiverId      = receiverNode->GetId( );
        Ptr<Socket> senderSocket = getSenderSocket( senderId, receiverId );
        if ( !senderSocket )
        {
            continue;
        }
        Simulator::Schedule( MilliSeconds( 1000 ), [this, senderSocket, senderId, receiverId, coverageVersion, coverageSet, dataSize]( ) {
            if ( !senderSocket )
            {
                NS_LOG_ERROR( "senderSocket is null in scheduled lambda for Node " << senderId );
                return;
            }
            Ptr<Packet> packet = Create<Packet>( );
            GossipHeader gossipHeader( senderId, coverageVersion, coverageSet, dataSize );
            packet->AddHeader( gossipHeader );
            // NS_LOG_INFO( "Packet size: " << packet->GetSerializedSize( ) );
            _dataSizesScaled[senderId] = ( static_cast<double>( packet->GetSerializedSize( ) ) / _maximumDataSize ) * ( _assignedSensors[senderId].size( ) + 1 /* Initial area covered by the node */ );
            if ( senderSocket->Send( packet ) == -1 )
            {
                NS_LOG_ERROR( "Failed to send packet from Node " << senderId << " to Node " << receiverId );
            }
            else
            {
                // NS_LOG_INFO( "Node " << senderId << " sent 1 packet to Node " << receiverId );
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
    // NS_LOG_INFO( "Data Receiver Socket bound on Node " << nodeId << " port " << receiverPort );
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
/*
 * If one neighbor is providing useful information, that info will be added. Then check if there is duplicate, check for duplicate pairs and remove the one with a higher cost.
 * Calculate utility is greater than 0. Add everything, merge to check cost of each pair. Check duplicate pairs, do not check utility of own node. The cost is the data.
 *
 */
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

        int incomingVersion         = gossipHeader.GetCoverageVersion( );
        int lastSeenForThisNeighbor = _lastSeenVersion[receiverId][senderId];

        // Avoid processing duplicate packets.
        if ( incomingVersion > lastSeenForThisNeighbor )
        {

            _lastSeenVersion[receiverId][senderId] = incomingVersion;
            // Save old coverage for version-change detection
            auto oldCoverage = _localView[receiverId];

            auto coverageFromSender = gossipHeader.GetCoverageSet( );

            // Calculate Utility using the bitset–based method.
            double neighborUtility = calculateUtility( senderId, receiverId );
            NS_LOG_INFO( "Sender Utility of Node " << senderId << ": " << neighborUtility );

            if ( neighborUtility > 0 )
            {
                // Add (M_j, A_j) to local view
                for ( auto& p : coverageFromSender )
                {
                    _localView[receiverId].insert( p );
                }
            }
            else
            {
                checkDuplicates( receiverId, coverageFromSender );
            }

            // 4) Check if local coverage changed
            if ( _localView[receiverId] != oldCoverage )
            {
                // The node learned something new
                // increment coverageVersion[receiverId]
                _coverageVersion[receiverId]++;
                NS_LOG_INFO( "Node " << receiverId << " learned new coverage => coverageVersion=" << _coverageVersion[receiverId] );
            }

            double ownUtility = calculateUtility( receiverId, receiverId );
            NS_LOG_INFO( "Receiver Utility of Node " << receiverId << ": " << ownUtility );
            if ( ownUtility < 0 )
            {
                checkDuplicates( receiverId, _localView[receiverId] );
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

    auto& unionCoverage = _localView[receiverId];
    for ( uint32_t s : _sensorTypes )
    {
        for ( uint32_t a : _areas )
        {
            bool found = false;
            for ( auto& quad : unionCoverage )
            {
                if ( quad.sensor == s && quad.area == a )
                {
                    found = true;
                    break;
                }
            }
            if ( !found )
                return false;
        }
    }
    return true;
}

//
// calculateUtility: compute utility using bitset differences.
// This calculation needs to take into account when a sensor and area are already in a set but not in a pair together
//
double AdhocNetwork::calculateUtility( uint32_t senderId, uint32_t receiverId )
{
    // --------------------------------------------------------
    // (A) Coverage sets for node i (the "sender") and set S_k (the "receiver")
    // --------------------------------------------------------
    // If you only want the intrinsic coverage of sender, use:
    //   auto & senderCoverage = _intrinsicCoverageSets[senderId];
    // If you want everything node i currently "knows", use:
    //   auto & senderCoverage = _localView[senderId];
    const auto& senderCoverage = _intrinsicCoverageSets[senderId];
    const auto& skCoverage     = _localView[receiverId];

    // --------------------------------------------------------
    // (B) Double-loop over sensors × areas
    // --------------------------------------------------------
    double incrementalCoverage = 0.0;

    for ( auto s : _sensorTypes ) // s in S_total
    {
        for ( auto a : _areas ) // a in A_total
        {
            // A_i(s,a) = 1 if 'senderCoverage' has (s,a)
            double Ai_s_a = coversSensorArea( senderCoverage, s, a ) ? 1.0 : 0.0;

            // max_{j in S_k} A_j(s,a) = 1 if 'skCoverage' has (s,a)
            double Sk_s_a = coversSensorArea( skCoverage, s, a ) ? 1.0 : 0.0;

            // incremental coverage for (s,a) from i w.r.t. S_k
            // is Ai_s_a * [1 - Sk_s_a]
            incrementalCoverage += Ai_s_a * ( 1.0 - Sk_s_a );
        }
    }

    // --------------------------------------------------------
    // (C) If you also have a cost term, subtract it
    // --------------------------------------------------------
    double dataCost   = _dataSizesScaled[senderId];
    double totalValue = incrementalCoverage - _lambda * dataCost;

    return totalValue;
}

bool AdhocNetwork::coversSensorArea( const std::set<CoverageQuad, CoverageQuadLess>& coverageSet, uint32_t sensor, uint32_t area )
{
    // We can do a quick 'find_if' for a quad that matches (sensor, area).
    auto it = std::find_if( coverageSet.begin( ), coverageSet.end( ), [sensor, area]( const CoverageQuad& q ) { return q.sensor == sensor && q.area == area; } );
    return ( it != coverageSet.end( ) );
}

double AdhocNetwork::computeIntrinsicUtilityOfNode( uint32_t nodeId ) const
{
    // If each node has exactly 1 area
    uint32_t numSensors = _assignedSensors[nodeId].size( );
    uint32_t numAreas   = 1;
    double dataCost     = _intrinsicUtility[nodeId];

    // Utility = alpha * (#sensors) + beta * (#areas) - lambda * dataCost
    return _alpha * numSensors + _beta * numAreas - _lambda * dataCost;
}

double AdhocNetwork::computeSummedUtility( uint32_t nodeId )
{
    // Use an unordered_set to store unique contributor node IDs.
    std::unordered_set<uint32_t> contributors;

    // Iterate over the final coverage set.
    for ( const auto& quad : _localView[nodeId] )
    {
        contributors.insert( quad.originId );
    }

    double totalUtility = 0.0;
    // Sum up the intrinsic utility of each contributor.
    for ( auto contributorId : contributors )
    {
        totalUtility += computeIntrinsicUtilityOfNode( contributorId );
    }

    return totalUtility;
}

std::set<CoverageQuad, CoverageQuadLess> AdhocNetwork::buildUnionCoverage( uint32_t nodeId ) const
{
    std::set<CoverageQuad, CoverageQuadLess> unionSet = _localView[nodeId];
    return unionSet;
}

void AdhocNetwork::printFinalCoverage( uint32_t nodeId ) const { NS_LOG_INFO( "Final coverage set: " + _coveringSetString ); }

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

        // Print node coverage as pairs
        outFile << "Coverage Pairs: ";
        for ( const auto& sensor : _assignedSensors[i] )
        {
            outFile << "(" << sensor << "," << _assignedAreas[i] << ") ";
        }
        outFile << "\n";

        // Print data size scaled
        // TODO: this will end up being the final dataSizesScaled, we want initial
        outFile << "Data Size Scaled: " << _intrinsicUtility[i] << "\n";
    }
    outFile << "========== End of Node Information ==========\n";
    outFile.close( );
}

void AdhocNetwork::checkDuplicates( const uint32_t& receiverId, const std::set<CoverageQuad, CoverageQuadLess>& senderCoverage )
{
    // For each pair in the sender's coverage set:
    for ( const auto& incomingQuad : senderCoverage )
    {
        // Look for a pair in the receiver's local view that has the same sensor and area.
        auto it = std::find_if( _localView[receiverId].begin( ), _localView[receiverId].end( ), [&incomingQuad]( const CoverageQuad& existingQuad ) {
            return existingQuad.sensor == incomingQuad.sensor && existingQuad.area == incomingQuad.area;
        } );
        if ( it != _localView[receiverId].end( ) )
        {
            // There is a duplicate. Remove the one with the higher cost.
            if ( it->dataScaled > incomingQuad.dataScaled )
            {
                _localView[receiverId].erase( it );
                if ( _localView[receiverId] != senderCoverage )
                {
                    _localView[receiverId].insert( incomingQuad );
                }
            }

            // Otherwise, keep the existing one (the one with lower cost).

            // Deliverable 1:
            //      every node's initial information: sensor, area, data size, node i
            //      utility score of final solution for every sim run: Based on the contributing nodes' local coverage; the summed utility of each contributing node's iniitial utility
            // Deliverable 2:
            //      For each sim run, information (s, a, d) for the selected nodes.
            //      Between each round, every node move to a random adjacent cell
            //      Run for 100-150 sim runs
        }
        else
        {
            if ( _localView[receiverId] != senderCoverage )
            {
                _localView[receiverId].insert( incomingQuad );
            }
        }
    }
}