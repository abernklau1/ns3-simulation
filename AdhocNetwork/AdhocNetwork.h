#ifndef ADHOC_NETWORK_H
#define ADHOC_NETWORK_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "../GossipHeader/GossipHeader.h"

/**
 * @brief Custom hash for std::pair<uint32_t, uint32_t> used in unordered_map.
 */
namespace std
{
    template <> struct hash<std::pair<uint32_t, uint32_t>>
    {
            size_t operator( )( const std::pair<uint32_t, uint32_t>& p ) const { return std::hash<uint32_t>( )( p.first ) ^ ( std::hash<uint32_t>( )( p.second ) << 1 ); }
    };
} // namespace std

/**
 * @brief The AdhocNetwork class implements a distributed UAV gossip protocol simulation.
 *
 * Nodes (UAVs) are randomly placed in a grid, and each is assigned a set of sensor types and an area.
 * The nodes discover their neighbors, select a subset for gossip-based communication,
 * and exchange packets that carry coverage information until full coverage is reached.
 */
class AdhocNetwork
{
    public:
        /**
         * @brief Constructs an AdhocNetwork instance.
         *
         * @param numNodes The total number of nodes in the simulation.
         * @param sensorTypes The set of all sensor types.
         * @param areaTypes The set of all area types.
         * @param wifiStandard The WiFi standard to use.
         * @param macType The MAC type.
         * @param ipBase The base IP address.
         * @param positionAllocator The type of position allocator (e.g., "ns3::RandomRectanglePositionAllocator").
         * @param communicationRange The maximum communication range for neighbor discovery.
         * @param gridX The X-dimension of the simulation grid.
         * @param gridY The Y-dimension of the simulation grid.
         * @param nodeSensors A vector of pre-assigned sensor sets for each node.
         * @param nodeAreas A vector of pre-assigned areas (one per node).
         */
        AdhocNetwork( uint32_t numNodes,
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
                      std::vector<uint32_t> nodeAreas );

        /**
         * @brief Destructor. Cleans up resources.
         */
        ~AdhocNetwork( );

        //============================================================================
        // Setup and Initialization
        //============================================================================

        /**
         * @brief Performs the initial setup for the network.
         *
         * This function creates nodes, installs the WiFi and IP stack, assigns IP addresses,
         * sets up data receiver sockets on each node, and initializes the intrinsic coverage
         * (based on pre-assigned sensor and area values).
         */
        void setup( );

        /**
         * @brief Randomly assigns positions to all nodes.
         *
         * @param xMin Minimum x-coordinate.
         * @param xMax Maximum x-coordinate.
         * @param yMin Minimum y-coordinate.
         * @param yMax Maximum y-coordinate.
         */
        void initializeRandomPositions( double xMin, double xMax, double yMin, double yMax );

        /**
         * @brief Initializes each node's intrinsic coverage set.
         *
         * Based on pre-assigned sensors and areas, this function builds each node's local
         * coverage set and initializes bitset representations.
         */
        void initializeNodeCoverageSets( );

        //============================================================================
        // Neighbor Discovery and Selection
        //============================================================================

        /**
         * @brief Discovers neighbors for a given node.
         *
         * Determines which nodes are within the communication range of the given node.
         *
         * @param nodeId The ID of the node performing neighbor discovery.
         */
        void findNeighbors( uint32_t nodeId );

        /**
         * @brief Selects a random subset of discovered neighbors.
         *
         * Chooses a subset (of size m_gossipGroupSize) of the neighbors for gossip communication.
         *
         * @param nodeId The ID of the node selecting a subset.
         */
        void findNeighborsSubset( uint32_t nodeId );

        /**
         * @brief Schedules periodic neighbor discovery events.
         *
         * @param interval The time interval between successive neighbor discovery callbacks.
         */
        void scheduleFindNeighbors( double interval );

        /**
         * @brief Callback function for periodic neighbor discovery.
         *
         * Invokes neighbor discovery for node 0 and then schedules packet sending for all nodes.
         *
         * @param interval The time interval used for scheduling.
         */
        void findNeighborsCallback( double interval );

        //============================================================================
        // Packet Sending/Receiving (Gossip Communication)
        //============================================================================

        /**
         * @brief Retrieves or creates a sender socket for communication between two nodes.
         *
         * @param senderId The sender node's ID.
         * @param receiverId The receiver node's ID.
         * @return A pointer to the sender socket.
         */
        Ptr<Socket> getSenderSocket( uint32_t senderId, uint32_t receiverId );

        /**
         * @brief Schedules packet sending from a node to a set of neighbor nodes.
         *
         * Constructs a packet with the node's coverage information and schedules its transmission.
         *
         * @param senderNode Pointer to the sender node.
         * @param senderId The sender node's ID.
         * @param neighbors A vector of pointers to neighbor nodes.
         */
        void sendPackets( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors );

        /**
         * @brief A wrapper function for SendPackets.
         *
         * @param senderNode Pointer to the sender node.
         * @param senderId The sender node's ID.
         * @param neighbors A vector of pointers to neighbor nodes.
         */
        void sendPacketsHelper( Ptr<Node> senderNode, uint32_t senderId, std::vector<Ptr<Node>> neighbors );

        /**
         * @brief Sets up a data receiver socket on a node.
         *
         * Binds a UDP socket to a specific port (based on the node's ID) and sets a callback for incoming packets.
         *
         * @param node Pointer to the node.
         * @param nodeId The node's ID.
         */
        void setupDataReceiver( Ptr<Node> node, uint32_t nodeId );

        /**
         * @brief Retrieves the node ID corresponding to a given IP address.
         *
         * @param address The IP address.
         * @return The node ID, or UINT32_MAX if not found.
         */
        uint32_t getNodeIdFromIpAddress( Ipv4Address address );

        /**
         * @brief Processes incoming packets on a node's receiver socket.
         *
         * Implements the gossip protocol logic: checks for duplicate packets, calculates utility,
         * updates aggregated coverage, and schedules further message dissemination.
         *
         * @param socket Pointer to the socket that received the packet.
         */
        void receivePacket( Ptr<Socket> socket );

        //============================================================================
        // Coverage and Utility Functions
        //============================================================================

        /**
         * @brief Returns the total number of sensor types and area types in the network.
         *
         * @param nodeId The node's ID (unused in the current implementation).
         * @return A pair (|Stotal|, |Atotal|).
         */
        std::pair<uint32_t, uint32_t> setCoverage( uint32_t nodeId );

        /**
         * @brief Checks if a node's aggregated coverage includes all sensor and area types.
         *
         * @param receiverId The node's ID.
         * @return true if the node covers all sensor and area types; false otherwise.
         */
        bool isCovered( uint32_t receiverId );

        /**
         * @brief Loops over the final coverage set to identify which node's contributed to the final set
         *
         * @param nodeId covering node's ID
         * @return A vector of the IDs of contributing nodes
         */
        std::vector<uint32_t> getFinalContributors( uint32_t nodeId ) const;

        /**
         * @brief Computes the utility of incorporating a sender's coverage information into a receiver's view.
         *
         * The utility is computed based on the number of "new" sensor types and areas that would be added.
         *
         * @param senderId The sender node's ID.
         * @param receiverId The receiver node's ID.
         * @return The utility value.
         */
        double calculateUtility( uint32_t senderId, uint32_t receiverId );

        /**
         * @brief Computes the intrinsic utility of a node
         *
         * @param nodeId Node from which to calculate
         * @return utility
         */
        double computeIntrinsicUtilityOfNode( uint32_t nodeId ) const;

        /**
         * @brief Calculates the summed utility of the covering nodes
         *
         * @param nodeId Covering node ID
         * @return summed utility
         */
        double computeSummedUtility( uint32_t nodeId ) const;

        /**
         * @brief Builds the union coverage set of a given node
         *
         * @param nodeId Id of node from which to build union set
         * @return The union coverage set
         */
        std::set<std::pair<uint32_t, uint32_t>> buildUnionCoverage( uint32_t nodeId ) const;

        // /**
        //  * @brief Updates the aggregated sensor bitset for a node.
        //  *
        //  * Combines the node's intrinsic sensor coverage with the sensor coverage of its neighbors.
        //  *
        //  * @param receiverId The node's ID.
        //  */
        // void updateAggregatedSensors( uint32_t receiverId );

        // /**
        //  * @brief Updates the aggregated area bitset for a node.
        //  *
        //  * Combines the node's intrinsic area coverage with the area coverage of its neighbors.
        //  *
        //  * @param receiverId The node's ID.
        //  */
        // void updateAggregatedAreas( uint32_t receiverId );

        /**
         * @brief Calculates how many neighbor subsets include the given node.
         *
         * This is used for tracking how many distinct packets a node should receive.
         *
         * @param nodeId The node's ID.
         * @return The number of neighbor subsets that include this node.
         */
        uint32_t calculateNumSubNeighbors( uint32_t nodeId );

        /**
         * @brief Prints the final aggregated coverage (sensor and area) of a node.
         *
         * The function logs the list of sensor and area IDs that the node covers.
         *
         * @param nodeId The node's ID.
         */
        void printFinalCoverage( uint32_t nodeId ) const;

        /**
         * @brief Checks whether overall network coverage has been reached.
         *
         * @return true if at least one node has achieved full coverage; false otherwise.
         */
        bool isCoverageReached( );

        //============================================================================
        // Print Node Stats
        //============================================================================

        /**
         * @brief Prints each node's complete initial state to a file.
         *
         * This function writes out, for each node, the following information:
         * - Node ID
         * - Position (x, y, z)
         * - The intrinsic coverage set (sensor–area pairs)
         * - The list of discovered neighbor IDs
         *
         * @param filename The path and name of the file where the information will be written.
         */
        void printNodeInfoToFile( const std::string& filename ) const;

        //============================================================================
        // Getters for Network Objects
        //============================================================================

        /**
         * @brief Returns the container of all nodes.
         * @return The NodeContainer.
         */
        NodeContainer getNodes( ) const { return _nodes; }

        /**
         * @brief Returns the total number of nodes.
         * @return The number of nodes.
         */
        uint32_t getNumNodes( ) const { return _numNodes; }

        /**
         * @brief Returns the container of network devices.
         * @return The NetDeviceContainer.
         */
        NetDeviceContainer getDevices( ) const { return _devices; }

        /**
         * @brief Returns the container of IP interfaces.
         * @return The Ipv4InterfaceContainer.
         */
        Ipv4InterfaceContainer getInterfaces( ) const { return _interfaces; }

        /**
         * @brief Returns the vector of neighbor lists (one per node).
         * @return A vector containing a neighbor list for each node.
         */
        std::vector<std::vector<Ptr<Node>>> getNeighbors( ) const { return _neighbors; }

        /**
         * @brief Returns the vector of neighbor subset lists (one per node).
         * @return A vector containing a neighbor subset for each node.
         */
        std::vector<std::vector<Ptr<Node>>> getNeighborsSubset( ) const { return _neighborsSubset; }

        /**
         * @brief Returns the number of steps it took to discover a covering set
         * @return An unsigned 32 bit integer containing the number of steps to coverage
         */
        uint32_t getCoverageSteps( ) const { return _coveredSteps; }

        /**
         * @brief Returns the summed utilty of the covering node's original coverage.
         * @return a double containing the summed utility
         */
        double getSummedUtility( ) const { return _coveredUtility; }

        /**
         * @brief Returns the ID of the node that holds the covering set
         * @return a uint32_t holding the node ID
         */
        uint32_t getConvergedNode( ) const { return _coveringNode; }

        /**
         * @brief Returns a string representation of the covering set
         * @return an std::string of the covering set
         */
        std::string getCoveredSetString( ) const { return _coveringSetString; }

    private:
        //===========================================================================
        // Simulation Parameters
        //===========================================================================

        uint32_t _numNodes;             // Total number of nodes (UAVs)
        uint32_t _numSensors;           // Total number of sensor types (simulation-wide)
        uint32_t _numAreas;             // Total number of area types (simulation-wide)
        WifiStandard _wifiStandard;     // WiFi standard used in the simulation
        std::string _macType;           // MAC type used
        std::string _ipBase;            // Base IP address for nodes
        std::string _positionAllocator; // Type of position allocator used
        double _communicationRange;     // Maximum communication range for neighbor discovery
        double _gridX;                  // Width of the simulation grid
        double _gridY;                  // Height of the simulation grid

        //===========================================================================
        // Network and Mobility Containers
        //===========================================================================

        NodeContainer _nodes;               // Container holding all nodes
        NetDeviceContainer _devices;        // Container for network devices
        Ipv4InterfaceContainer _interfaces; // Container for IP interfaces
        std::vector<Vector> _positions;     // Current positions of nodes

        //===========================================================================
        // Neighbor Information
        //===========================================================================

        std::vector<std::vector<Ptr<Node>>> _neighbors;                                // List of neighbors for each node
        std::vector<std::vector<Ptr<Node>>> _neighborsSubset;                          // Selected neighbor subsets for gossip exchange
        std::unordered_map<std::pair<uint32_t, uint32_t>, Ptr<Socket>> _senderSockets; // Map of sender sockets keyed by (senderId, receiverId)

        //===========================================================================
        // Gossip Protocol Parameters
        //===========================================================================

        uint32_t _gossipGroupSize;              // Number of neighbors selected for gossip communication
        double _alpha;                          // Weight factor for sensor utility
        double _beta;                           // Weight factor for area utility
        double _lambda;                         // Weight factor for data size penalty
        uint32_t _maximumDataSize;              // Maximum data size used for scaling
        std::vector<uint32_t> _dataSizesScaled; // Scaled data size per node

        //===========================================================================
        // Coverage and Utility Data
        //===========================================================================

        std::vector<std::set<std::pair<uint32_t, uint32_t>>> _intrinsicCoverageSets; // Intrinsic coverage set per node (sensor-area pairs)
        // For each node i, store localView[i], which maps neighborID -> coverageSet
        // coverageSet is e.g. std::set<std::pair<uint32_t, uint32_t>>
        std::vector<std::unordered_map<uint32_t, std::set<std::pair<uint32_t, uint32_t>>>> _localView;
        std::vector<uint32_t> _coverageSteps;             // Number of times each node has updated its coverage view
        std::vector<std::set<uint32_t>> _receivedPackets; // Tracker for received (unique) packet IDs per node
        bool _isCoverageReached;                          // Flag indicating if full coverage has been reached by any node
        uint32_t _coveredSteps;                           // Number of steps it took to converge for this simulation
        double _coveredUtility;                           // Summed utility of the converged node's original coverage
        uint32_t _coveringNode;                           // The node that covers
        std::string _coveringSetString;                   // String representation of the covering set

        //===========================================================================
        // Sensor and Area Assignments
        //===========================================================================

        std::set<uint32_t> _sensorTypes;                    // Set of all sensor types available
        std::vector<std::vector<uint32_t>> _sensorCoverage; // Intrinsic sensor types per node
        std::set<uint32_t> _areas;                          // Set of all area types available
        std::vector<std::vector<uint32_t>> _areaCoverage;   // Intrinsic area (one per node) assignments

        //===========================================================================
        // Bitset Representations for Coverage
        //===========================================================================

        // static constexpr size_t MAX_SENSOR_TYPES = 3;                                                   // Maximum number of sensor types (used for bitset sizes)
        // static constexpr size_t MAX_AREA_TYPES   = 3;                                                   // Maximum number of area types (used for bitset sizes)
        // std::vector<std::bitset<MAX_SENSOR_TYPES>> _sensorCoverageBitset;                               // Bitset representation of intrinsic sensor coverage per node
        // std::vector<std::bitset<MAX_AREA_TYPES>> _areaCoverageBitset;                                   // Bitset representation of intrinsic area coverage per node
        // std::vector<std::bitset<MAX_SENSOR_TYPES>> _aggregatedSensorsBitset;                            // Aggregated sensor coverage from self and neighbors
        // std::vector<std::bitset<MAX_AREA_TYPES>> _aggregatedAreasBitset;                                // Aggregated area coverage from self and neighbors
        // std::vector<std::unordered_map<uint32_t, std::bitset<MAX_SENSOR_TYPES>>> _neighborSensorBitset; // Stores each neighbor's sensor bitset per node
        // std::vector<std::unordered_map<uint32_t, std::bitset<MAX_AREA_TYPES>>> _neighborAreaBitset;     // Stores each neighbor's area bitset per node

        //===========================================================================
        // Pre-assigned Sensor and Area Assignments
        //===========================================================================

        std::vector<std::vector<uint32_t>> _assignedSensors; // Pre-assigned sensor types for each node
        std::vector<uint32_t> _assignedAreas;                // Pre-assigned area for each node
};

#endif // ADHOC_NETWORK_H