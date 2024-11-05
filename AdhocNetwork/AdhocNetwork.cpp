#include "AdhocNetwork.h"

AdhocNetwork::AdhocNetwork( uint32_t numNodes, WifiStandard wifiStandard, std::string macType, std::string ipBase, std::string positionAllocator, double communicationRange )
    : m_numNodes( numNodes ),
      m_wifiStandard( wifiStandard ),
      m_macType( macType ),
      m_ipBase( ipBase ),
      m_positionAllocator( positionAllocator ),
      m_communicationRange( communicationRange )
{
    m_neighbors.resize( numNodes );
    m_positions.resize( numNodes );
}

AdhocNetwork::~AdhocNetwork( )
{
    // Close all sender sockets
    for ( auto& [link, socket] : m_senderSockets )
    {
        if ( socket )
        {
            socket->Close( );
            NS_LOG_INFO( "Closed Sender Socket from Node " << link.first << " to Node " << link.second );
        }
    }
    m_senderSockets.clear( );

    // Close all ACK sockets
    for ( auto& [link, socket] : m_ackSockets )
    {
        if ( socket )
        {
            socket->Close( );
            NS_LOG_INFO( "Closed ACK Socket from Node " << link.first << " to Node " << link.second );
        }
    }
    m_ackSockets.clear( );
}

void AdhocNetwork::setup( )
{
    // Create nodes
    m_nodes.Create( m_numNodes );

    // Set up WiFi
    WifiHelper wifi;
    wifi.SetStandard( m_wifiStandard );

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default( );

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel( wifiChannel.Create( ) );

    WifiMacHelper wifiMac;
    wifiMac.SetType( m_macType );

    m_devices = wifi.Install( wifiPhy, wifiMac, m_nodes );

    initializeRandomPositions( 0.0, 100.0, 0.0, 100.0 );

    // Install Internet stack
    InternetStackHelper internet;
    internet.Install( m_nodes );

    Ipv4AddressHelper ipv4;
    ipv4.SetBase( m_ipBase.c_str( ), "255.255.255.0" );
    m_interfaces = ipv4.Assign( m_devices );

    wifiPhy.EnablePcapAll( "server-debug", false );

    m_etxMatrix.initializeMatrix( m_numNodes );

    for ( uint32_t i = 0; i < m_nodes.GetN( ); ++i )
    {
        Ptr<Node> node  = m_nodes.Get( i );
        uint32_t nodeId = node->GetId( );

        // Set up data receiver for the node
        SetupDataReceiver( node, nodeId );

        // Set up ACK receiver for the node
        SetupAckReceiver( node, nodeId );
    }
}

void AdhocNetwork::findNeighbors( uint32_t nodeId )
{
    NS_LOG_INFO( "Starting neighbor discovery for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );

    m_neighbors.at( nodeId ).clear( );

    // Get positions of nodes
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

void AdhocNetwork::findNeighborsSubset( uint32_t nodeId )
{
    NS_LOG_INFO( "Starting subset neighbor selection for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );

    // TODO: Change to conduct search for a single node instead of all nodes
    // Clear the previous subset of neighbors
    m_neighborsSubset.clear( );
    m_neighborsSubset.resize( m_numNodes );

    Ptr<UniformRandomVariable> randomVar = CreateObject<UniformRandomVariable>( );

    for ( uint32_t i = 0; i < m_numNodes; ++i )
    {
        std::vector<Ptr<Node>> neighbors = m_neighbors.at( i );

        if ( neighbors.empty( ) )
        {
            NS_LOG_WARN( "Node " << i << " has no neighbors." );
            continue;
        }

        uint32_t numNeighbors = neighbors.size( );

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
            m_neighborsSubset.at( i ).emplace_back( neighbors.at( index ) );
        }
        NS_LOG_DEBUG( "Node " << i << " selected " << subsetSize << " neighbors." );
    }
    NS_LOG_INFO( "Subset neighbor selection completed for node " << nodeId << " at " << Simulator::Now( ).GetSeconds( ) << " seconds." );
}

void AdhocNetwork::scheduleFindNeighbors( double interval ) { Simulator::Schedule( Seconds( interval ), &AdhocNetwork::m_findNeighborsCallback, this, interval ); }

// TODO: Handle this in a better way. Not sure if we should just completely remove this function
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

        // Schedule ETX calculation
        Simulator::Schedule( Seconds( 3.0 ), &AdhocNetwork::CalculateETXHelper, this, i, neighbors );
    }

    // NOTE: This may still need to occur later when the mobility model is randomized instead of constant
    // scheduleFindNeighbors( interval );
}

void AdhocNetwork::initializeRandomPositions( double xMin, double xMax, double yMin, double yMax )
{
    NS_LOG_INFO( "Initializing random positions for nodes." );
    // Set a different seed for each simulation run
    RngSeedManager::SetSeed( time( NULL ) ); // Use current time as seed

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

// Function to calculate ETX
void AdhocNetwork::CalculateETX( uint32_t nodeId, uint32_t neighborId )
{
    auto it = m_linkStats.find( { nodeId, neighborId } );
    if ( it != m_linkStats.end( ) )
    {
        LinkStats stats = it->second;

        double df  = ( stats.dataPacketsSent > 0 ) ? static_cast<double>( stats.dataPacketsReceived ) / stats.dataPacketsSent : 0.0;
        double dr  = ( stats.ackPacketsSent > 0 ) ? static_cast<double>( stats.ackPacketsReceived ) / stats.ackPacketsSent : 0.0;
        double etx = ( df > 0 && dr > 0 ) ? 1.0 / ( df * dr ) : std::numeric_limits<double>::infinity( );

        m_etxMatrix.setEtx( nodeId, neighborId, etx );

        NS_LOG_INFO( "ETX from Node " << nodeId << " to Node " << neighborId << " is " << etx );
    }
    else
    {
        NS_LOG_WARN( "No link stats available for Node " << nodeId << " to Node " << neighborId );
        m_etxMatrix.setEtx( nodeId, neighborId, std::numeric_limits<double>::infinity( ) );
    }
}

void AdhocNetwork::CalculateETXHelper( uint32_t nodeId, const std::vector<Ptr<Node>>& neighbors )
{
    for ( const Ptr<Node>& neighbor : neighbors )
    {
        CalculateETX( nodeId, neighbor->GetId( ) );
    }
}

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
        Ptr<Socket> senderSocket = Socket::CreateSocket( senderNode, UdpSocketFactory::GetTypeId( ) );

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

void AdhocNetwork::SendPackets( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors )
{
    for ( Ptr<Node> receiverNode : neighbors )
    {
        uint32_t receiverId = receiverNode->GetId( );

        Ptr<Socket> senderSocket = GetSenderSocket( senderId, receiverId );
        if ( !senderSocket )
        {
            continue; // Skip if socket creation failed
        }

        // Send packets
        for ( uint32_t k = 0; k < 100; ++k )
        {
            Ptr<Packet> packet = Create<Packet>( 1024 );
            if ( senderSocket->Send( packet ) == -1 )
            {
                NS_LOG_ERROR( "Failed to send packet from Node " << senderId << " to Node " << receiverId );
            }
            else
            {
                m_linkStats[{ senderId, receiverId }].dataPacketsSent++;
            }
        }

        NS_LOG_INFO( "Node " << senderId << " sent 100 packets to Node " << receiverId );
    }
}

void AdhocNetwork::SendPacketsHelper( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors ) { SendPackets( senderNode, senderId, neighbors ); }

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

uint32_t AdhocNetwork::GetNodeIdFromIpAddress( Ipv4Address address )
{
    for ( uint32_t i = 0; i < m_nodes.GetN( ); ++i )
    {
        Ipv4Address nodeAddress = m_nodes.Get( i )->GetObject<Ipv4>( )->GetAddress( 1, 0 ).GetLocal( );
        if ( nodeAddress == address )
        {
            return i; // Node ID
        }
    }
    NS_LOG_WARN( "Node ID not found for IP address " << address );
    return UINT32_MAX; // Should never reach here
}

Ptr<Socket> AdhocNetwork::GetAckSocket( uint32_t receiverId, uint32_t senderId )
{
    std::pair<uint32_t, uint32_t> link = { receiverId, senderId };
    Ptr<Node> receiverNode             = m_nodes.Get( receiverId );
    Ptr<Node> senderNode               = m_nodes.Get( senderId );
    auto it                            = m_ackSockets.find( link );
    if ( it != m_ackSockets.end( ) )
    {
        return it->second;
    }
    else
    {
        Ptr<Socket> ackSocket = Socket::CreateSocket( receiverNode, UdpSocketFactory::GetTypeId( ) );

        Ipv4Address senderAddress = senderNode->GetObject<Ipv4>( )->GetAddress( 1, 0 ).GetLocal( );
        InetSocketAddress ackAddr = InetSocketAddress( senderAddress, 9000 + senderId );
        if ( ackSocket->Connect( ackAddr ) == -1 )
        {
            NS_LOG_ERROR( "Failed to connect ACK Socket from Node " << receiverId << " to Node " << senderId );
            return nullptr;
        }

        m_ackSockets[link] = ackSocket;
        return ackSocket;
    }
}

void AdhocNetwork::ReceivePacket( Ptr<Socket> socket )
{
    Ptr<Packet> packet;
    Address from;
    while ( ( packet = socket->RecvFrom( from ) ) )
    {
        InetSocketAddress addr    = InetSocketAddress::ConvertFrom( from );
        Ipv4Address senderAddress = addr.GetIpv4( );

        uint32_t senderId   = GetNodeIdFromIpAddress( senderAddress );
        uint32_t receiverId = socket->GetNode( )->GetId( );

        if ( senderId == UINT32_MAX )
        {
            NS_LOG_WARN( "Received packet from unknown sender address: " << senderAddress );
            continue;
        }

        m_linkStats[{ senderId, receiverId }].dataPacketsReceived++;

        // Send ACK back to sender's ACK port using persistent socket
        Ptr<Socket> ackSocket = GetAckSocket( receiverId, senderId );
        if ( !ackSocket )
        {
            continue; // Skip if socket creation failed
        }

        Ptr<Packet> ackPacket = Create<Packet>( 0 ); // Empty ACK packet
        if ( ackSocket->Send( ackPacket ) == -1 )
        {
            NS_LOG_ERROR( "Failed to send ACK from Node " << receiverId << " to Node " << senderId );
        }
        else
        {
            m_linkStats[{ receiverId, senderId }].ackPacketsSent++;
            NS_LOG_INFO( "Node " << receiverId << " sent ACK to Node " << senderId );
        }
    }
}

void AdhocNetwork::SetupAckReceiver( Ptr<Node> node, uint32_t nodeId )
{
    Ptr<Socket> ackReceiverSocket = Socket::CreateSocket( node, UdpSocketFactory::GetTypeId( ) );
    uint16_t ackPort              = 9000 + nodeId;
    InetSocketAddress localAddr   = InetSocketAddress( Ipv4Address::GetAny( ), ackPort );
    if ( ackReceiverSocket->Bind( localAddr ) == -1 )
    {
        NS_LOG_ERROR( "Failed to bind ACK Receiver Socket on Node " << nodeId << " port " << ackPort );
        return;
    }
    ackReceiverSocket->SetRecvCallback( MakeCallback( &AdhocNetwork::ReceiveAck, this ) );

    NS_LOG_INFO( "ACK Receiver Socket bound on Node " << nodeId << " port " << ackPort );
}

void AdhocNetwork::ReceiveAck( Ptr<Socket> socket )
{
    Ptr<Packet> packet;
    Address from;
    while ( ( packet = socket->RecvFrom( from ) ) )
    {
        InetSocketAddress addr      = InetSocketAddress::ConvertFrom( from );
        Ipv4Address neighborAddress = addr.GetIpv4( );

        uint32_t neighborId = GetNodeIdFromIpAddress( neighborAddress );
        uint32_t nodeId     = socket->GetNode( )->GetId( );

        if ( neighborId == UINT32_MAX )
        {
            NS_LOG_WARN( "Received ACK from unknown neighbor address: " << neighborAddress );
            continue;
        }

        m_linkStats[{ nodeId, neighborId }].ackPacketsReceived++;

        NS_LOG_INFO( "Node " << nodeId << " received ACK from Node " << neighborId );
    }
}