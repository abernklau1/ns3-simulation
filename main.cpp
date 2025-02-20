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

NS_LOG_COMPONENT_DEFINE( "MainSimulation" );

/**
 * @brief Schedule the initial events for each node.
 *
 * This function schedules the following events for every node:
 * - Neighbor discovery at 0.0 seconds.
 * - Selection of a random subset of discovered neighbors at 1.0 seconds.
 * - Sending packets to the selected neighbors at 2.0 seconds.
 *
 * @param adhoc The instance of AdhocNetwork on which to schedule these events.
 */
void ScheduleStep( AdhocNetwork& adhoc );

/**
 * @brief Randomly shift the positions of nodes between simulation runs.
 *
 * For each node, this function generates a new position by:
 * - Randomly selecting an axis (x or y).
 * - Randomly selecting a direction (positive or negative).
 * - Adjusting the position by a fixed step size.
 * The new position is accepted only if it lies within the grid boundaries [0, GRID_X] and [0, GRID_Y].
 *
 * @param adhoc The instance of AdhocNetwork whose node positions will be updated.
 * @param runIndex The current simulation run index (unused in this implementation).
 */
void ShiftNodePositions( AdhocNetwork& adhoc, uint32_t runIndex );

int main( int argc, char* argv[] )
{
    // Enable logging for components
    LogComponentEnable( "MainSimulation", LOG_LEVEL_INFO );
    LogComponentEnable( "AdhocNetwork", LOG_LEVEL_INFO );
    LogComponentEnable( "GossipHeader", LOG_LEVEL_INFO );

    CommandLine cmd( __FILE__ );
    cmd.Parse( argc, argv );

    std::vector<int> convergenceStepsVec;
    convergenceStepsVec.reserve( NUM_RUNS );

    // Store final node positions between runs if needed.
    std::vector<Vector> nodePositions( NUM_NODES );

    // Initialize sensor types and area IDs.
    std::set<uint32_t> sensorTypes;
    std::set<uint32_t> areaTypes;
    for ( uint32_t i = 0; i < NUM_SENSORS; ++i )
    {
        sensorTypes.insert( i );
    }
    for ( uint32_t i = 0; i < NUM_AREAS; ++i )
    {
        areaTypes.insert( i );
    }

    // Randomly assign sensors to nodes.
    Ptr<UniformRandomVariable> rngSensors = CreateObject<UniformRandomVariable>( );
    std::vector<std::vector<uint32_t>> nodeToSensors( NUM_NODES );
    for ( uint32_t nodeIdx = 0; nodeIdx < NUM_NODES; ++nodeIdx )
    {
        // Choose between 1 and (NUM_SENSORS-1) sensors.
        uint32_t numPossible = ( NUM_SENSORS > 1 ) ? ( NUM_SENSORS - 1 ) : 1;
        uint32_t howMany     = rngSensors->GetInteger( 1, numPossible );

        std::vector<uint32_t> allSensors;
        for ( uint32_t s = 0; s < NUM_SENSORS; ++s )
        {
            allSensors.push_back( s );
        }
        std::vector<uint32_t> chosenSensors;
        for ( uint32_t c = 0; c < howMany && !allSensors.empty( ); ++c )
        {
            uint32_t idx = rngSensors->GetInteger( 0, allSensors.size( ) - 1 );
            chosenSensors.push_back( allSensors[idx] );
            allSensors.erase( allSensors.begin( ) + idx );
        }
        nodeToSensors[nodeIdx] = chosenSensors;
    }

    // Round-robin area assignment: each node gets one area (cycling through available areas)
    std::vector<uint32_t> nodeToArea( NUM_NODES );
    for ( uint32_t nodeIdx = 0; nodeIdx < NUM_NODES; ++nodeIdx )
    {
        nodeToArea[nodeIdx] = nodeIdx % NUM_AREAS;
    }

    // Run the simulation for each run.
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

        // (Optional: reset positions between runs)
        // for (uint32_t i = 0; i < NUM_NODES; ++i)
        // {
        //     Ptr<Node> node = adhoc.getNodes().Get(i);
        //     Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        //     mob->SetPosition(nodePositions[i]);
        // }

        // Schedule initial events (neighbor discovery, subset selection, packet sending)
        ScheduleStep( adhoc );

        // Run the simulation.
        Simulator::Run( );

        double finalTime = Simulator::Now( ).GetSeconds( );
        if ( finalTime >= MAX_RUN_TIME )
        {
            NS_LOG_INFO( "Coverage not reached by " << MAX_RUN_TIME << " seconds; run timed out." );
        }

        // Record coverage steps if coverage was reached.
        int coverageSteps = adhoc.getMaxCoverage( );
        if ( adhoc.isCoverageReached( ) )
        {
            convergenceStepsVec.push_back( coverageSteps );
            NS_LOG_INFO( "Run " << runIndex << " coverage steps: " << coverageSteps );
        }

        // Optionally store final positions.
        // for (uint32_t i = 0; i < NUM_NODES; ++i)
        // {
        //     Ptr<Node> node = adhoc.getNodes().Get(i);
        //     Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        //     nodePositions[i] = mob->GetPosition();
        // }

        Simulator::Destroy( );
        NS_LOG_INFO( "===== Completed Run #" << runIndex << " =====" );
    }

    NS_LOG_INFO( "Number of runs counted in convergenceStepsVec: " << convergenceStepsVec.size( ) );
    double meanConvergenceSteps = std::accumulate( convergenceStepsVec.begin( ), convergenceStepsVec.end( ), 0.0 ) / convergenceStepsVec.size( );
    NS_LOG_INFO( "Mean convergence steps: " << meanConvergenceSteps );
    double stdDevConvergenceSteps = std::sqrt( std::accumulate( convergenceStepsVec.begin( ),
                                                                convergenceStepsVec.end( ),
                                                                0.0,
                                                                [meanConvergenceSteps]( double sum, int steps ) { return sum + std::pow( steps - meanConvergenceSteps, 2 ); } ) /
                                               convergenceStepsVec.size( ) );
    NS_LOG_INFO( "Standard deviation of convergence steps: " << stdDevConvergenceSteps );

    return 0;
}

void ScheduleStep( AdhocNetwork& adhoc )
{
    for ( uint32_t i = 0; i < NUM_NODES; ++i )
    {
        // Schedule neighbor discovery for node i at time 0.0 seconds.
        Simulator::Schedule( Seconds( 0.0 ), &AdhocNetwork::findNeighbors, &adhoc, i );

        // Schedule the selection of a random subset of neighbors for node i at time 1.0 seconds.
        Simulator::Schedule( Seconds( 1.0 ), &AdhocNetwork::findNeighborsSubset, &adhoc, i );

        // Schedule packet sending for node i at time 2.0 seconds.
        Simulator::Schedule( Seconds( 2.0 ), [&, i]( ) {
            auto subset = adhoc.getNeighborsSubset( ).at( i );
            if ( !subset.empty( ) )
            {
                adhoc.sendPacketsHelper( adhoc.getNodes( ).Get( i ), i, subset );
            }
        } );
    }
}

void ShiftNodePositions( AdhocNetwork& adhoc, uint32_t runIndex )
{
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>( );
    double stepSize                = 1.0; // Maximum step magnitude for the position change

    for ( uint32_t i = 0; i < NUM_NODES; ++i )
    {
        Ptr<Node> node         = adhoc.getNodes( ).Get( i );
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>( );
        if ( !mob )
            continue;

        // Get the current position of the node.
        Vector pos = mob->GetPosition( );
        bool valid = false;

        // Generate a new position until a valid position within the grid is found.
        while ( !valid )
        {
            // Randomly choose an axis: 0 for x-axis, 1 for y-axis.
            uint32_t axis = rng->GetInteger( 0, 1 );
            // Randomly choose a direction: 0 for negative, 1 for positive.
            uint32_t sign = rng->GetInteger( 0, 1 );
            double offset = ( sign == 0 ) ? -stepSize : stepSize;

            // Tentatively update the position on the chosen axis.
            Vector newPos = pos;
            if ( axis == 0 )
                newPos.x += offset;
            else
                newPos.y += offset;

            // Accept the new position if it is within the grid boundaries.
            if ( newPos.x >= 0.0 && newPos.x <= GRID_X && newPos.y >= 0.0 && newPos.y <= GRID_Y )
            {
                pos   = newPos;
                valid = true;
            }
        }
        // Update the node's position.
        mob->SetPosition( pos );
    }
}