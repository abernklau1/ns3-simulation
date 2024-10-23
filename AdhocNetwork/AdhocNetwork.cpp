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
    Ptr<Node> node                   = m_nodes.Get( i );
    uint32_t nodeId                  = node->GetId( );
    std::vector<Ptr<Node>> neighbors = m_neighbors.at( i );

    // Set up data receiver for the node
    SetupDataReceiver( node, nodeId );

    // Set up ACK receiver for the node
    SetupAckReceiver( node, nodeId );
  }
}

void AdhocNetwork::findNeighbors( )
{
  // Reset neighbors
  for ( auto& neighbor : m_neighbors )
  {
    neighbor.clear( );
  }

  // Get positions of nodes
  for ( uint32_t i = 0; i < m_numNodes; ++i )
  {
    Ptr<MobilityModel> mobility = m_nodes.Get( i )->GetObject<MobilityModel>( );
    m_positions.at( i )         = mobility->GetPosition( );
  }

  // Find neighbors within communication range
  for ( uint32_t i = 0; i < m_numNodes; ++i )
  {
    for ( uint32_t j = 0; j < m_numNodes; ++j )
    {
      if ( i != j )
      {
        Vector pos_i    = m_positions.at( i );
        Vector pos_j    = m_positions.at( j );
        double distance = CalculateDistance( pos_i, pos_j );
        if ( distance <= m_communicationRange )
        {
          m_neighbors.at( i ).emplace_back( m_nodes.Get( j ) );
        }
      }
    }
  }

}

void AdhocNetwork::scheduleFindNeighbors( double interval ) { Simulator::Schedule( Seconds( interval ), &AdhocNetwork::m_findNeighborsCallback, this, interval ); }

void AdhocNetwork::m_findNeighborsCallback( double interval )
{
  std::cout << "Finding neighbors" << std::endl;
  findNeighbors( );
  std::cout << "Neighbors found" << std::endl;
  
  for ( uint32_t i = 0; i < m_nodes.GetN( ); ++i )
  {
    Ptr<Node> node                  = m_nodes.Get( i );
    std::vector<Ptr<Node>> neighbors = m_neighbors.at( i );
    Simulator::Schedule( Seconds( 1.0 ), &AdhocNetwork::SendPacketsHelper, this, node, i, neighbors );

    // Schedule ETX calculation
    Simulator::Schedule( Seconds( 3.0 ), &AdhocNetwork::CalculateETXHelper, this, i, neighbors );
  }

  scheduleFindNeighbors( interval );
}

void AdhocNetwork::initializeRandomPositions( double xMin, double xMax, double yMin, double yMax )
{
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
  mobility.SetMobilityModel( "ns3::RandomWalk2dMobilityModel",
                             "Bounds",
                             RectangleValue( Rectangle( 0, 100, 0, 100 ) ),
                             "Distance",
                             DoubleValue( 20.0 ),
                             "Speed",
                             StringValue( "ns3::ConstantRandomVariable[Constant=1.0]" ),
                             "Direction",
                             StringValue( "ns3::UniformRandomVariable[Min=0.0|Max=6.283185307]" ) );
  mobility.Install( m_nodes );
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

    // Optional: Print ETX value for verification
    std::cout << "ETX from Node " << nodeId << " to Node " << neighborId << " is " << etx << std::endl;
  }
  else
  {
    std::cout << "No link stats available for Node " << nodeId << " to Node " << neighborId << std::endl;
  }
}

void AdhocNetwork::CalculateETXHelper( uint32_t nodeId, std::vector<Ptr<Node>> neighbors )
{
  for ( Ptr<Node> neighbor : neighbors )
  {
    CalculateETX( nodeId, neighbor->GetId( ) );
  }
}

void AdhocNetwork::SendPackets( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors )
{
  for ( Ptr<Node> receiverNode : neighbors )
  {
    uint32_t receiverId = receiverNode->GetId( );
    uint16_t receiverPort = 8000 + receiverId;

    // Create sockets
    Ptr<Socket> senderSocket   = Socket::CreateSocket( senderNode, UdpSocketFactory::GetTypeId( ) );

    // Connect sender socket to receiver
    Ipv4Address receiverAddress  = receiverNode->GetObject<Ipv4>( )->GetAddress( 1, 0 ).GetLocal( );
    InetSocketAddress remoteAddr = InetSocketAddress( receiverAddress, receiverPort );
    senderSocket->Connect( remoteAddr );

    // Send packets
    for ( uint32_t k = 0; k < 100; ++k )
    {
      Ptr<Packet> packet = Create<Packet>( 1024 );
      senderSocket->Send( packet );
      m_linkStats[{ senderId, receiverId }].dataPacketsSent++;
    }
  }
}

void AdhocNetwork::SendPacketsHelper( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors )
{
  SendPackets( senderNode, senderId, neighbors );
}

void AdhocNetwork::SetupDataReceiver( Ptr<Node> node, uint32_t nodeId )
{
  Ptr<Socket> receiverSocket  = Socket::CreateSocket( node, UdpSocketFactory::GetTypeId( ) );
  uint16_t receiverPort       = 8000 + nodeId;
  InetSocketAddress localAddr = InetSocketAddress( Ipv4Address::GetAny( ), receiverPort );
  receiverSocket->Bind( localAddr );
  receiverSocket->SetRecvCallback( MakeCallback( &AdhocNetwork::ReceivePacket, this ) );
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
  NS_ASSERT_MSG( false, "Node ID not found for IP address " << address );
  return UINT32_MAX; // Should never reach here
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

    m_linkStats[{ senderId, receiverId }].dataPacketsReceived++;

    // Send ACK back to sender's ACK port
    Ptr<Socket> ackSocket     = Socket::CreateSocket( socket->GetNode( ), UdpSocketFactory::GetTypeId( ) );
    uint16_t senderAckPort    = 9000 + senderId;
    InetSocketAddress ackAddr = InetSocketAddress( senderAddress, senderAckPort );
    ackSocket->Connect( ackAddr );
    Ptr<Packet> ackPacket = Create<Packet>( 0 ); // Empty ACK packet
    ackSocket->Send( ackPacket );
    m_linkStats[{ receiverId, senderId }].ackPacketsSent++;
  }
}

void AdhocNetwork::SetupAckReceiver( Ptr<Node> node, uint32_t nodeId )
{
  Ptr<Socket> ackReceiverSocket = Socket::CreateSocket( node, UdpSocketFactory::GetTypeId( ) );
  uint16_t ackPort              = 9000 + nodeId;
  InetSocketAddress localAddr   = InetSocketAddress( Ipv4Address::GetAny( ), ackPort );
  ackReceiverSocket->Bind( localAddr );
  ackReceiverSocket->SetRecvCallback( MakeCallback( &AdhocNetwork::ReceiveAck, this ) );
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

    m_linkStats[{ nodeId, neighborId }].ackPacketsReceived++;
  }
}