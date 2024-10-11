#include "Experiment.h"

Experiment::Experiment( ) { }

Experiment::Experiment( std::string name )
    : m_output( name )
{
  m_output.SetStyle( Gnuplot2dDataset::LINES );
}

void Experiment::SetPosition( Ptr<Node> node, Vector position )
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>( );
  mobility->SetPosition( position );
}

Vector Experiment::GetPosition( Ptr<Node> node )
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>( );
  return mobility->GetPosition( );
}

void Experiment::AdvancePosition( Ptr<Node> node )
{
  Vector pos   = GetPosition( node );
  double mbs   = ( ( m_bytesTotal * 8.0 ) / 1000000 );
  m_bytesTotal = 0;
  m_output.Add( pos.x, mbs );
  pos.x += 1.0;
  if ( pos.x >= 210.0 )
  {
    return;
  }
  SetPosition( node, pos );
  Simulator::Schedule( Seconds( 1.0 ), &Experiment::AdvancePosition, this, node );
}

void Experiment::ReceivePacket( Ptr<Socket> socket )
{
  Ptr<Packet> packet;
  while ( ( packet = socket->Recv( ) ) )
  {
    m_bytesTotal += packet->GetSize( );
  }
}

Ptr<Socket> Experiment::SetupPacketReceive( Ptr<Node> node )
{
  TypeId tid       = TypeId::LookupByName( "ns3::PacketSocketFactory" );
  Ptr<Socket> sink = Socket::CreateSocket( node, tid );
  sink->Bind( );
  sink->SetRecvCallback( MakeCallback( &Experiment::ReceivePacket, this ) );
  return sink;
}

Gnuplot2dDataset Experiment::Run( const WifiHelper& wifi, const YansWifiPhyHelper& wifiPhy, const WifiMacHelper& wifiMac, const YansWifiChannelHelper& wifiChannel )
{
  m_bytesTotal = 0;

  NodeContainer c;
  c.Create( 2 );

  PacketSocketHelper packetSocket;
  packetSocket.Install( c );

  YansWifiPhyHelper phy = wifiPhy;
  phy.SetChannel( wifiChannel.Create( ) );

  NetDeviceContainer devices = wifi.Install( phy, wifiMac, c );

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>( );
  positionAlloc->Add( Vector( 0.0, 0.0, 0.0 ) );
  positionAlloc->Add( Vector( 5.0, 0.0, 0.0 ) );
  mobility.SetPositionAllocator( positionAlloc );
  mobility.SetMobilityModel( "ns3::ConstantPositionMobilityModel" );

  mobility.Install( c );

  PacketSocketAddress socket;
  socket.SetSingleDevice( devices.Get( 0 )->GetIfIndex( ) );
  socket.SetPhysicalAddress( devices.Get( 1 )->GetAddress( ) );
  socket.SetProtocol( 1 );

  OnOffHelper onoff( "ns3::PacketSocketFactory", Address( socket ) );
  onoff.SetConstantRate( DataRate( 60000000 ) );
  onoff.SetAttribute( "PacketSize", UintegerValue( 2000 ) );

  ApplicationContainer apps = onoff.Install( c.Get( 0 ) );
  apps.Start( Seconds( 0.5 ) );
  apps.Stop( Seconds( 250.0 ) );

  Simulator::Schedule( Seconds( 1.5 ), &Experiment::AdvancePosition, this, c.Get( 1 ) );
  Ptr<Socket> recvSink = SetupPacketReceive( c.Get( 1 ) );

  Simulator::Run( );

  Simulator::Destroy( );

  return m_output;
}