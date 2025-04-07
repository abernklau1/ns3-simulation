#include "AdhocNetwork/AdhocNetwork.h"
#include "ns3/command-line.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <random>
#include <set>
#include <unordered_set>
#include <vector>

namespace fs = std::filesystem;

#ifndef NS3_LOG_ENABLE
    #define NS3_LOG_ENABLE 1
#endif

using namespace ns3;

#define NUM_NODES           150
#define NUM_SENSORS         8
#define NUM_AREAS           19
#define NUM_RUNS            150
#define COMMUNICATION_RANGE 100
#define MAX_RUN_TIME        100000000.0
#define GRID_X              150.0
#define GRID_Y              150.0

NS_LOG_COMPONENT_DEFINE( "MainSimulation" );

// Function prototypes
bool IsFileEmptyOrNotExist( const std::string& filename );
void ScheduleStep( AdhocNetwork& adhoc );
void ShiftNodePositions( AdhocNetwork& adhoc, uint32_t runIndex );

int main( int argc, char* argv[] )
{
    // Create stats and logs directories if they don't exist
    if ( !fs::exists( "stats" ) )
    {
        fs::create_directory( "stats" );
    }
    if ( !fs::exists( "logs" ) )
    {
        fs::create_directory( "logs" );
    }

    // Redirect NS_LOG output to a file.
    std::freopen( "logs/ns3.log", "w", stderr );

    // Enable logging for our components.
    LogComponentEnable( "MainSimulation", LOG_LEVEL_INFO );
    LogComponentEnable( "AdhocNetwork", LOG_LEVEL_INFO );
    LogComponentEnable( "GossipHeader", LOG_LEVEL_INFO );

    CommandLine cmd( __FILE__ );
    cmd.Parse( argc, argv );

    std::vector<int> convergenceStepsVec;
    convergenceStepsVec.reserve( NUM_RUNS );

    // Load node positions if available.
    std::vector<Vector> nodePositions( NUM_NODES );
    bool positionsFileExists = false;
    std::ifstream posFileIn( "stats/node_positions.txt" );
    if ( posFileIn.good( ) )
    {
        NS_LOG_INFO( "Reading node positions from file." );
        for ( uint32_t i = 0; i < NUM_NODES; ++i )
        {
            double x, y;
            if ( posFileIn >> x >> y )
            {
                nodePositions[i] = Vector( x, y, 0.0 );
            }
            else
            {
                NS_LOG_WARN( "Not enough positions in file; using random positions." );
                positionsFileExists = false;
                break;
            }
        }
        if ( posFileIn.eof( ) || posFileIn.good( ) )
        {
            positionsFileExists = true;
        }
        posFileIn.close( );
    }
    else
    {
        NS_LOG_INFO( "Node positions file not found; will use random positions." );
    }

    // Prepare containers for intrinsic coverage assignment.
    std::vector<std::vector<uint32_t>> nodeToSensors( NUM_NODES );
    std::vector<uint32_t> nodeToArea( NUM_NODES );
    bool coverageFileExists = false;
    std::ifstream coverageFileIn( "stats/node_coverage.txt" );
    if ( coverageFileIn.good( ) )
    {
        NS_LOG_INFO( "Reading node coverage from file." );
        bool fullyReadOk = true;
        // Here we assume that the file contains for each node:
        // <number_of_pairs> followed by that many pairs in the format (sensor,area)
        // and then the assigned area.
        for ( uint32_t i = 0; i < NUM_NODES; ++i )
        {
            if ( !coverageFileIn.good( ) )
            {
                fullyReadOk = false;
                break;
            }
            uint32_t pairCount;
            coverageFileIn >> pairCount;
            if ( !coverageFileIn.good( ) )
            {
                fullyReadOk = false;
                break;
            }
            std::vector<uint32_t> sensors;
            for ( uint32_t j = 0; j < pairCount; ++j )
            {
                char ignore;
                uint32_t sensor, area;
                // Read a pair formatted as: (s,a)
                coverageFileIn >> ignore >> sensor >> ignore >> area >> ignore;
                sensors.push_back( sensor );
            }
            nodeToSensors[i] = sensors;
            uint32_t assignedArea;
            coverageFileIn >> assignedArea;
            nodeToArea[i] = assignedArea;
        }
        if ( fullyReadOk )
        {
            coverageFileExists = true;
        }
        else
        {
            NS_LOG_WARN( "Coverage file incomplete. Will do random coverage assignment." );
        }
    }
    coverageFileIn.close( );

    // If no coverage file exists, then build the universe and randomly assign intrinsic coverage.
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

    if ( !coverageFileExists )
    {
        NS_LOG_INFO( "Generating intrinsic coverage sets from universe." );
        // Create the universe of all sensor–area pairs.
        std::vector<std::pair<uint32_t, uint32_t>> universe;
        for ( uint32_t s = 0; s < NUM_SENSORS; ++s )
        {
            for ( uint32_t a = 0; a < NUM_AREAS; ++a )
            {
                universe.push_back( std::make_pair( s, a ) );
            }
        }

        std::vector<std::vector<uint32_t>> nodesForArea( NUM_AREAS );
        // Assign an intrinsic area to each node using round robin.
        for ( uint32_t nodeIdx = 0; nodeIdx < NUM_NODES; ++nodeIdx )
        {
            uint32_t a          = nodeIdx % NUM_AREAS;
            nodeToArea[nodeIdx] = a;
            nodesForArea[a].push_back( nodeIdx );
        }

        // For each node, select a random subset of pairs (only those with the node's assigned area)
        std::vector<std::set<std::pair<uint32_t, uint32_t>>> nodeIntrinsicCoverage( NUM_NODES );
        Ptr<UniformRandomVariable> rngPairs = CreateObject<UniformRandomVariable>( );
        std::random_device rd;
        std::mt19937 gen( rd( ) );

        Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>( );

        for ( uint32_t nodeIdx = 0; nodeIdx < NUM_NODES; ++nodeIdx )
        {
            uint32_t assignedArea = nodeToArea[nodeIdx];
            // Collect candidate pairs whose area equals the assigned area.
            std::vector<std::pair<uint32_t, uint32_t>> candidatePairs;
            for ( auto& p : universe )
            {
                if ( p.second == assignedArea )
                {
                    candidatePairs.push_back( p );
                }
            }
            uint32_t numCandidates = candidatePairs.size( );
            // Choose a random number (between 1 and numCandidates) of pairs.
            uint32_t howMany = rngPairs->GetInteger( 1, numCandidates - 1 );
            std::shuffle( candidatePairs.begin( ), candidatePairs.end( ), gen );
            std::set<std::pair<uint32_t, uint32_t>> intrinsicSet;
            for ( uint32_t j = 0; j < howMany; ++j )
            {
                intrinsicSet.insert( candidatePairs[j] );
            }
            nodeIntrinsicCoverage[nodeIdx] = intrinsicSet;

            // For the AdhocNetwork constructor, we pass nodeToSensors as a vector of sensor IDs.
            // Since each pair is (sensor, area) and the area is the same for every pair in this node’s intrinsic coverage,
            // we simply extract the sensor IDs.
            std::vector<uint32_t> sensors;
            for ( auto& pair : nodeIntrinsicCoverage[nodeIdx] )
            {
                sensors.push_back( pair.first );
            }
            nodeToSensors[nodeIdx] = sensors;
        }

        // Check if all pairs have been covered, if not add pairs to some nodes
        std::unordered_set<std::pair<uint32_t, uint32_t>> assigned;
        for ( uint32_t nodeIdx = 0; nodeIdx < NUM_NODES; nodeIdx++ )
        {
            for ( auto& p : nodeIntrinsicCoverage[nodeIdx] )
            {
                assigned.insert( p );
            }
        }

        for ( auto& pair : universe )
        {
            if ( assigned.find( pair ) == assigned.end( ) )
            {
                uint32_t area = pair.second;

                auto& candidateNodes = nodesForArea[area];
                for ( auto id : candidateNodes )
                {
                    if ( nodeIntrinsicCoverage[id].size( ) < NUM_SENSORS - 1 )
                    {
                        nodeIntrinsicCoverage[id].insert( pair );
                        assigned.insert( pair );

                        uint32_t sensor = pair.first;
                        auto& sensorVec = nodeToSensors[id];
                        if ( std::find( sensorVec.begin( ), sensorVec.end( ), sensor ) == sensorVec.end( ) )
                        {
                            sensorVec.push_back( sensor );
                        }

                        break;
                    }
                }
            }
        }

        // Write the intrinsic coverage sets to a file.
        std::ofstream coverageFileOut( "stats/node_coverage.txt", std::ios::out );
        if ( coverageFileOut.is_open( ) )
        {
            coverageFileOut << "NUM_PAIRS (sensor,area) ... ASSIGNED_AREA\n";
            for ( uint32_t i = 0; i < NUM_NODES; ++i )
            {
                coverageFileOut << nodeIntrinsicCoverage[i].size( ) << " ";
                for ( auto& p : nodeIntrinsicCoverage[i] )
                {
                    coverageFileOut << "(" << p.first << "," << p.second << ") ";
                }
                coverageFileOut << nodeToArea[i] << "\n";
            }
            coverageFileOut.close( );
        }
        else
        {
            NS_LOG_ERROR( "Failed to open node_coverage.txt for writing intrinsic coverage sets." );
        }
    }
    else
    {
        NS_LOG_INFO( "Re-using coverage sets from node_coverage.txt" );
    }

    // Run the simulation for each run.
    for ( uint32_t runIndex = 0; runIndex < NUM_RUNS; ++runIndex )
    {
        NS_LOG_INFO( "===== Starting Run #" << runIndex << " =====" );

        RngSeedManager::SetSeed( time( NULL ) + runIndex );
        // RngSeedManager::SetRun( runIndex );

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
        adhoc.startRounds( );

        if ( positionsFileExists )
        {
            NS_LOG_INFO( "Assigning stored node positions to nodes." );
            for ( uint32_t i = 0; i < NUM_NODES; ++i )
            {
                Ptr<Node> node         = adhoc.getNodes( ).Get( i );
                Ptr<MobilityModel> mob = node->GetObject<MobilityModel>( );
                mob->SetPosition( nodePositions[i] );
            }
        }

        Simulator::Run( );

        double finalTime = Simulator::Now( ).GetSeconds( );
        if ( finalTime >= MAX_RUN_TIME )
        {
            NS_LOG_INFO( "Coverage not reached by " << MAX_RUN_TIME << " seconds; run timed out." );
        }

        Simulator::Destroy( );
        NS_LOG_INFO( "===== Completed Run #" << runIndex << " =====" );

        int coverageSteps = adhoc.getCoverageSteps( );
        if ( adhoc.isCoverageReached( ) )
        {
            convergenceStepsVec.push_back( coverageSteps );
            NS_LOG_INFO( "Run " << runIndex << " coverage steps: " << coverageSteps );
        }

        if ( !positionsFileExists )
        {
            NS_LOG_INFO( "Storing node positions to file." );
            for ( uint32_t i = 0; i < NUM_NODES; ++i )
            {
                Ptr<Node> node         = adhoc.getNodes( ).Get( i );
                Ptr<MobilityModel> mob = node->GetObject<MobilityModel>( );
                if ( mob )
                {
                    nodePositions[i] = mob->GetPosition( );
                }
            }
            bool fileIsEmpty = IsFileEmptyOrNotExist( "stats/node_positions.txt" );
            std::ofstream posFileOut( "stats/node_positions.txt", std::ios::app );
            if ( posFileOut.is_open( ) )
            {
                if ( fileIsEmpty )
                {
                    posFileOut << "X Y\n";
                }
                for ( uint32_t i = 0; i < NUM_NODES; ++i )
                {
                    posFileOut << nodePositions[i].x << " " << nodePositions[i].y << "\n";
                }
                posFileOut.close( );
                positionsFileExists = true;
            }
            else
            {
                NS_LOG_ERROR( "Failed to open node_positions.txt for writing." );
            }
        }

        {
            if ( adhoc.isCoverageReached( ) )
            {
                bool fileIsEmpty = false;
                {
                    std::ifstream inCheck( "stats/converged_run_details.txt" );
                    fileIsEmpty = ( !inCheck.good( ) || ( inCheck.peek( ) == std::ifstream::traits_type::eof( ) ) );
                    inCheck.close( );
                }
                std::ofstream detailFile( "stats/converged_run_details.txt", std::ios::app );
                if ( !detailFile.is_open( ) )
                {
                    NS_LOG_ERROR( "Could not open stats/converged_run_details.txt" );
                }
                else
                {
                    if ( fileIsEmpty )
                    {
                        detailFile << "RUN NODE STEPS UTILITY SET (sensor, area, originNode, dataScaled)\n";
                    }
                    uint32_t convergedNode = adhoc.getConvergedNode( );
                    uint32_t coveredSteps  = adhoc.getCoverageSteps( );
                    double summedUtility   = adhoc.getSummedUtility( );
                    std::string coveredSet = adhoc.getCoveredSetString( );
                    detailFile << runIndex << " " << convergedNode << " " << coveredSteps << " " << summedUtility << " " << coveredSet << "\n";
                }
                detailFile.close( );
            }
        }
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

    bool fileEmpty = IsFileEmptyOrNotExist( "stats/convergence_results.txt" );
    std::ofstream outFile( "stats/convergence_results.txt", std::ios::app );
    if ( !outFile.is_open( ) )
    {
        NS_LOG_ERROR( "Could not open stats/convergence_results.txt" );
    }
    else
    {
        if ( fileEmpty )
        {
            outFile << "MEAN STD\n";
        }
        outFile << meanConvergenceSteps << " " << stdDevConvergenceSteps << "\n";
    }
    outFile.close( );

    {
        std::ifstream inFile( "stats/convergence_results.txt" );
        std::vector<double> means;
        std::vector<double> stds;
        if ( inFile.is_open( ) )
        {
            double m, s;
            while ( inFile >> m >> s )
            {
                means.push_back( m );
                stds.push_back( s );
            }
            inFile.close( );
        }
        if ( means.size( ) == 10 )
        {
            double sumM        = std::accumulate( means.begin( ), means.end( ), 0.0 );
            double meanOfMeans = sumM / 10.0;
            double sumS        = std::accumulate( stds.begin( ), stds.end( ), 0.0 );
            double meanOfStds  = sumS / 10.0;
            std::cout << "\nAfter collecting 10 lines total:\n";
            std::cout << "Mean of the 10 means = " << meanOfMeans << "\n";
            std::cout << "Mean of the 10 stds  = " << meanOfStds << "\n\n";
            fs::remove_all( "results" );
            NS_LOG_INFO( "Deleted directory 'results'." );
        }
    }

    return 0;
}

bool IsFileEmptyOrNotExist( const std::string& filename )
{
    std::ifstream inCheck( filename, std::ios::ate | std::ios::binary );
    if ( !inCheck.is_open( ) )
    {
        return true;
    }
    return ( inCheck.tellg( ) == 0 );
}

void ScheduleStep( AdhocNetwork& adhoc )
{
    for ( uint32_t i = 0; i < NUM_NODES; ++i )
    {
        Simulator::Schedule( Seconds( 0.0 ), &AdhocNetwork::findNeighbors, &adhoc, i );
        Simulator::Schedule( Seconds( 1.0 ), &AdhocNetwork::findNeighborsSubset, &adhoc, i );
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
    double stepSize                = 1.0;
    for ( uint32_t i = 0; i < NUM_NODES; ++i )
    {
        Ptr<Node> node         = adhoc.getNodes( ).Get( i );
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>( );
        if ( !mob )
            continue;
        Vector pos = mob->GetPosition( );
        bool valid = false;
        while ( !valid )
        {
            uint32_t axis = rng->GetInteger( 0, 1 );
            uint32_t sign = rng->GetInteger( 0, 1 );
            double offset = ( sign == 0 ) ? -stepSize : stepSize;
            Vector newPos = pos;
            if ( axis == 0 )
                newPos.x += offset;
            else
                newPos.y += offset;
            if ( newPos.x >= 0.0 && newPos.x <= GRID_X && newPos.y >= 0.0 && newPos.y <= GRID_Y )
            {
                pos   = newPos;
                valid = true;
            }
        }
        mob->SetPosition( pos );
    }
}