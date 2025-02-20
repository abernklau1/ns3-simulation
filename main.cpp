#include "AdhocNetwork/AdhocNetwork.h"
#include "ns3/command-line.h"
#include "ns3/log.h"

#ifndef NS3_LOG_ENABLE
    #define NS3_LOG_ENABLE 1
#endif

using namespace ns3;

#define NUM_NODES           4
#define NUM_SENSORS         3
#define NUM_AREAS           3
#define NUM_RUNS            15
#define COMMUNICATION_RANGE 70.0
#define MAX_RUN_TIME        100000.0
#define GRID_X              100.0
#define GRID_Y              100.0

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

NS_LOG_COMPONENT_DEFINE( "MainSimulation" );

void ScheduleStep( AdhocNetwork& adhocNetwork );
void ShiftNodePositions( AdhocNetwork& adhocNetwork, uint32_t runIndex );

int main( int argc, char* argv[] )
{

    LogComponentEnable( "MainSimulation", LOG_LEVEL_INFO );
    LogComponentEnable( "AdhocNetwork", LOG_LEVEL_INFO );
    LogComponentEnable( "GossipHeader", LOG_LEVEL_INFO );

    CommandLine cmd( __FILE__ );
    cmd.Parse( argc, argv );

    std::vector<int> convergenceStepsVec;
    convergenceStepsVec.reserve( NUM_RUNS );

    std::vector<Vector> nodePositions;
    nodePositions.resize( NUM_NODES );

    std::set<uint32_t> sensorTypes;
    std::set<uint32_t> areaTypes;

    // Initialize sensor types and area IDs
    for ( uint32_t i = 0; i < NUM_SENSORS; ++i )
    {
        sensorTypes.insert( i );
    }
    for ( uint32_t i = 0; i < NUM_AREAS; ++i )
    {
        areaTypes.insert( i );
    }

    Ptr<UniformRandomVariable> rngSensors = CreateObject<UniformRandomVariable>( );

    std::vector<std::vector<uint32_t>> nodeToSensors( NUM_NODES );
    for ( uint32_t nodeIdx = 0; nodeIdx < NUM_NODES; ++nodeIdx )
    {
        // pick how many sensors for this node:
        uint32_t numPossible = ( NUM_SENSORS > 1 ) ? ( NUM_SENSORS - 1 ) : 1;
        // if NUM_SENSORS=3, then numPossible=2
        uint32_t howMany = rngSensors->GetInteger( 1, numPossible );

        // We want 'howMany' distinct sensor types from [0..NUM_SENSORS-1]
        // Let's sample them randomly without replacement:
        std::vector<uint32_t> allSensors;
        for ( uint32_t s = 0; s < NUM_SENSORS; s++ )
        {
            allSensors.push_back( s );
        }

        // We'll pick 'howMany' out of the 'allSensors'
        std::vector<uint32_t> chosenSensors;
        for ( uint32_t c = 0; c < howMany && !allSensors.empty( ); c++ )
        {
            uint32_t idx = rngSensors->GetInteger( 0, allSensors.size( ) - 1 );
            chosenSensors.push_back( allSensors[idx] );
            // remove it so we don't pick duplicates
            allSensors.erase( allSensors.begin( ) + idx );
        }
        nodeToSensors[nodeIdx] = chosenSensors;
    }

    // 2) Round-robin area assignment
    // This ensures each node has exactly 1 area, reusing areas if #nodes > #areas
    std::vector<uint32_t> nodeToArea( NUM_NODES );
    for ( uint32_t nodeIdx = 0; nodeIdx < NUM_NODES; ++nodeIdx )
    {
        // pick area in [0..(NUM_AREAS - 1)] but cycle so node0->area0, node1->area1, node2->area2, node3->area0, ...
        nodeToArea[nodeIdx] = nodeIdx % NUM_AREAS;
    }

    // For each run:
    for ( uint32_t runIndex = 0; runIndex < NUM_RUNS; ++runIndex )
    {
        NS_LOG_INFO( "===== Starting Run #" << runIndex << " =====" );
        AdhocNetwork adhoc( NUM_NODES,
                            sensorTypes,
                            areaTypes,
                            WIFI_STANDARD_80211a,
                            "ns3::AdhocWifiMac",
                            "10.1.1.0",
                            "ns3::RandomRectanglePositionAllocator",
                            COMMUNICATION_RANGE,
                            GRID_X,
                            GRID_Y,
                            nodeToSensors,
                            nodeToArea );
        adhoc.setup( );

        // if ( runIndex > 0 )
        // {
        //     for ( uint32_t i = 0; i < NUM_NODES; ++i )
        //     {
        //         Ptr<Node> node         = adhocNetwork.getNodes( ).Get( i );
        //         Ptr<MobilityModel> mob = node->GetObject<MobilityModel>( );
        //         mob->SetPosition( nodePositions[i] );
        //     }
        // }

        // for ( uint32_t i = 0; i < adhocNetwork.getNodes( ).GetN( ); ++i )
        // {
        //     Ptr<Node> node         = adhocNetwork.getNodes( ).Get( i );
        //     Ptr<MobilityModel> mob = node->GetObject<MobilityModel>( );
        //     Vector pos             = mob->GetPosition( );
        //     NS_LOG_INFO( "After setup/shift: Node " << i << " position: " << pos );
        // }

        // Schedule events for this step
        ScheduleStep( adhoc );

        if ( Simulator::IsFinished( ) && !adhoc.IsCoverageReached( ) )
        {
            NS_LOG_INFO( "IsFinished" );

            // For each node, schedule neighbor discovery, subset selection, etc.
            for ( uint32_t i = 0; i < NUM_NODES; ++i )
            {
                // Subset selection at startTime + 1.0
                Simulator::Schedule( Seconds( 1.0 ), &AdhocNetwork::findNeighborsSubset, &adhoc, i );

                // Packet sending at startTime + 2.0
                Simulator::Schedule( Seconds( 2.0 ), [&, i]( ) {
                    auto neighborsSubset = adhoc.getNeighborsSubset( ).at( i );
                    if ( !neighborsSubset.empty( ) )
                    {
                        adhoc.SendPacketsHelper( adhoc.getNodes( ).Get( i ), i, neighborsSubset );
                    }
                } );
            }
        }

        // Now let the sim run from currentEndTime to currentEndTime+phaseDuration
        // Simulator::Stop( Seconds( currentEndTime + RUN_DURATION ) );
        // Simulator::Stop( Seconds( MAX_RUN_TIME ) );
        Simulator::Run( );

        // after returning from Run():
        double finalTime = Simulator::Now( ).GetSeconds( );
        if ( finalTime >= MAX_RUN_TIME )
        {
            NS_LOG_INFO( "Coverage not reached by " << MAX_RUN_TIME << " seconds; run timed out." );
        }

        // End of this step
        // Retrieve coverage steps from adhocNetwork, store them, etc.
        int coverageSteps = adhoc.getMaxCoverage( );
        if ( adhoc.IsCoverageReached( ) )
        {
            convergenceStepsVec.push_back( coverageSteps );
            NS_LOG_INFO( "Run " << runIndex << " coverage steps: " << coverageSteps );
        }

        // Shift node positions for this step
        // ShiftNodePositions( adhocNetwork, runIndex );

        // for ( uint32_t i = 0; i < NUM_NODES; ++i )
        // {
        //     Ptr<Node> node         = adhocNetwork.getNodes( ).Get( i );
        //     Ptr<MobilityModel> mob = node->GetObject<MobilityModel>( );
        //     nodePositions[i]       = mob->GetPosition( ); // capture final positions
        // }

        // Reset the simulator (this resets the event queue and simulation time, but not the node state)
        Simulator::Destroy( );

        NS_LOG_INFO( "===== Completed Run #" << runIndex << " =====" );
    }

    NS_LOG_INFO( "Number of runs counted in convergenceStepsVec: " << convergenceStepsVec.size( ) );

    double meanConvergenceSteps = std::accumulate( convergenceStepsVec.begin( ), convergenceStepsVec.end( ), 0.0 ) / convergenceStepsVec.size( );
    NS_LOG_INFO( "Mean convergence steps: " << meanConvergenceSteps );
    double stdDevConvergenceSteps = std::sqrt( std::accumulate( convergenceStepsVec.begin( ),
                                                                convergenceStepsVec.end( ),
                                                                0.0,
                                                                [meanConvergenceSteps]( double sum, int convergenceSteps ) { return sum + std::pow( convergenceSteps - meanConvergenceSteps, 2 ); } ) /
                                               convergenceStepsVec.size( ) );
    NS_LOG_INFO( "Standard deviation of convergence steps: " << stdDevConvergenceSteps );

    return 0;
}

void ScheduleStep( AdhocNetwork& adhocNetwork )
{
    // For each node, schedule neighbor discovery, subset selection, etc.
    for ( uint32_t i = 0; i < NUM_NODES; ++i )
    {
        // Suppose we do neighbor discovery at startTime + 0 seconds
        Simulator::Schedule( Seconds( 0.0 ), &AdhocNetwork::findNeighbors, &adhocNetwork, i );

        // Subset selection at startTime + 1.0
        Simulator::Schedule( Seconds( 1.0 ), &AdhocNetwork::findNeighborsSubset, &adhocNetwork, i );

        // Packet sending at startTime + 2.0
        Simulator::Schedule( Seconds( 2.0 ), [&, i]( ) {
            auto neighborsSubset = adhocNetwork.getNeighborsSubset( ).at( i );
            if ( !neighborsSubset.empty( ) )
            {
                adhocNetwork.SendPacketsHelper( adhocNetwork.getNodes( ).Get( i ), i, neighborsSubset );
            }
        } );
    }
}

void ShiftNodePositions( AdhocNetwork& adhocNetwork, uint32_t runIndex )
{
    Ptr<UniformRandomVariable> randVar = CreateObject<UniformRandomVariable>( );

    double stepSize   = 1.0; // shift magnitude
    uint32_t numNodes = NUM_NODES;

    for ( uint32_t i = 0; i < numNodes; i++ )
    {
        Ptr<Node> node              = adhocNetwork.getNodes( ).Get( i );
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>( );
        if ( !mobility )
        {
            continue;
        }

        Vector pos = mobility->GetPosition( );

        // We'll keep retrying until we get a valid move.
        bool validMove = false;
        while ( !validMove )
        {
            // Decide which axis to move on: x or y
            //   0 => shift x
            //   1 => shift y
            uint32_t axisDecision = randVar->GetInteger( 0, 1 );

            // Decide sign (+/-)
            //   0 => negative
            //   1 => positive
            uint32_t signDecision = randVar->GetInteger( 0, 1 );

            double offset = ( signDecision == 0 ) ? -stepSize : stepSize;

            // Tentatively compute new position
            Vector newPos = pos;
            if ( axisDecision == 0 )
            {
                newPos.x += offset;
            }
            else
            {
                newPos.y += offset;
            }

            // Check if newPos is within [0,100] range
            if ( newPos.x >= 0.0 && newPos.x <= GRID_X && newPos.y >= 0.0 && newPos.y <= GRID_Y )
            {
                // Accept it
                pos       = newPos;
                validMove = true;
            }
            // else: loop again, pick new axis + sign
        }

        // Apply that valid position
        mobility->SetPosition( pos );
    }
}