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
  m_packetsSent.resize( numNodes );
  m_packetsReceived.resize( numNodes );
  m_onOffApps.resize( numNodes );
  m_packetSinks.resize( numNodes );
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

  setupApplications( );
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
          m_neighbors.at( i ).emplace_back( *m_nodes.Get( j ) );
        }
      }
    }
  }

  // Setup applications after finding neighbors
  setupApplications( );
}

void AdhocNetwork::scheduleFindNeighbors( double interval ) { Simulator::Schedule( Seconds( interval ), &AdhocNetwork::m_findNeighborsCallback, this, interval ); }

void AdhocNetwork::m_findNeighborsCallback( double interval )
{
  std::cout << "Finding neighbors" << std::endl;
  findNeighbors( );
  std::cout << "Neighbors found" << std::endl;

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

void AdhocNetwork::setupApplications( )
{
  uint16_t basePort = 1024;
  for ( size_t i = 0; i < m_nodes.GetN( ); ++i )
  {
    for ( size_t j = 0; j < m_neighbors.at( i ).size( ); ++j )
    {
      // Ensure port number is unique
      uint16_t port = basePort + i * m_neighbors.at( i ).size( ) + j;

      Node neighbor         = m_neighbors.at( i ).at( j );
      Ptr<Node> neighborPtr = CreateObject<Node>( neighbor );
      // Create OnOffApplication on node i
      if ( !neighborPtr )
      {
        OnOffHelper onOff( "ns3::UdpSocketFactory", InetSocketAddress( m_interfaces.GetAddress( neighborPtr->GetId( ), 0 ), port ) );
        m_onOffApps.at( i ).emplace_back( onOff.Install( m_nodes.Get( i ) ) );
        onOff.SetConstantRate( DataRate( "500kbps" ) );
        onOff.SetAttribute( "PacketSize", UintegerValue( 1024 ) );
        onOff.SetAttribute( "MaxBytes", UintegerValue( 100 * 1024 ) ); // Simulate sending 100 packets

        ApplicationContainer senderApp = onOff.Install( m_nodes.Get( i ) );
        senderApp.Start( Simulator::Now( ) + Seconds( 1.0 ) );
        senderApp.Stop( Simulator::Now( ) + Seconds( 3.0 ) );
      }

      // Create PacketSink on neighbor node
      PacketSinkHelper packetSink( "ns3::UdpSocketFactory", InetSocketAddress( m_interfaces.GetAddress( neighborPtr->GetId( ), 0 ), port ) );
      ApplicationContainer receiverApp = packetSink.Install( neighborPtr );
      m_packetSinks.at( i ).emplace_back( receiverApp );
      receiverApp.Start( Simulator::Now( ) );
      receiverApp.Stop( Simulator::Now( ) + Seconds( 3.0 ) );
      port++;
    }
  }

  // Set up OnOffApplications
  for ( int i = 0; i < m_onOffApps.size( ); ++i )
  {
    for ( int j = 0; j < m_onOffApps.at( i ).size( ); ++j )
    {
      uint32_t packedValue      = ( static_cast<uint32_t>( i ) << 16 ) | j;
      Ptr<OnOffApplication> app = DynamicCast<OnOffApplication>( m_onOffApps.at( i ).at( j ).Get( 0 ) );
      app->TraceConnectWithoutContext( "Tx", MakeCallback( &AdhocNetwork::m_trackPacketsSent, this ) );
    }
  }

  // Set up PacketSinks
  for ( int i = 0; i < m_packetSinks.size( ); ++i )
  {
    for ( int j = 0; j < m_packetSinks.at( i ).size( ); ++j )
    {
      uint32_t packedValue = ( static_cast<uint32_t>( i ) << 16 ) | j;

      Ptr<PacketSink> packetSink = DynamicCast<PacketSink>( m_packetSinks.at( i ).at( j ).Get( 0 ) );
      packetSink->TraceConnectWithoutContext( "Rx", MakeCallback( &AdhocNetwork::m_trackPacketsReceived, this ) );
    }
  }
}

void AdhocNetwork::m_trackPacketsSent( Ptr<const Packet> packet ) { std::cout << "Packet sent" << std::endl; }

void AdhocNetwork::m_trackPacketsReceived( Ptr<const Packet> packet, Address receiver ) { std::cout << "Packet received" << std::endl; }

// Function to calculate ETX
double AdhocNetwork::CalculateETX( uint32_t nodeId, uint32_t neighborId )
{
  // Ensure indices are within bounds
  if ( neighborId >= m_packetsReceived.size( ) )
  {
    std::cerr << "Invalid node or neighbor ID" << std::endl;
    return std::numeric_limits<double>::infinity( ); // Return a large value to indicate error
  }

  std::cout << m_packetsSent.at( nodeId ).size( ) << std::endl;

  // // Get the number of packets sent and received
  // uint32_t sent     = m_packetsSent.at( nodeId ).at( neighborId );
  // uint32_t received = m_packetsReceived.at( nodeId ).at( neighborId );

  // // Avoid division by zero
  // if ( sent == 0 || received == 0 )
  // {
  //   std::cerr << "No packets sent or received" << std::endl;
  //   return std::numeric_limits<double>::infinity( ); // Return a large value to indicate error
  // }

  // // Calculate delivery ratios
  // double df = static_cast<double>( received ) / sent;
  // double dr = static_cast<double>( sent ) / received;

  // // Calculate ETX
  // return 1.0 / ( df * dr );

  return 1.0;
}
