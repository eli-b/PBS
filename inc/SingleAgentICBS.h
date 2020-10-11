#pragma once
//#include "single_agent_ecbs.h"
//#include "MDD.h"
#include <stdlib.h>

#include <vector>
#include <list>
#include <utility>
#include <tuple>
#include <unordered_map>
#include <memory>  // For unique_ptr

#include "LLNode.h"
#include "egraph_reader.h"
#include "map_loader.h"
// #include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/pairing_heap.hpp>
#include "conflict_avoidance_table.h"


class SingleAgentICBS
    //public SingleAgentECBS
{
public:
    int agent_id;

    // define typedefs (will also be used in ecbs_search)
    typedef boost::heap::pairing_heap<LLNode*, boost::heap::compare<LLNode::compare_node>> heap_open_t;
    typedef boost::heap::pairing_heap<LLNode*, boost::heap::compare<LLNode::secondary_compare_node>> heap_focal_t;
    /* typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t; */
    /* typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::secondary_compare_node> > heap_focal_t; */

    typedef std::unordered_map<LLNode*, std::unique_ptr<LLNode>, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
    // note -- hash_map (key is a node pointer, data is a node handler,
    //                   NodeHasher is the hash function to be used,
    //                   eqnode is used to break ties when hash values are equal)

    //std::shared_ptr<vector<PathEntry>> path;  // a path that takes the agent from initial to goal location satisying all constraints
    // consider changing path from vector to deque (efficient front insertion)
    //double path_cost;
    int start_location;
    int goal_location;
    //MapLoader::orientation_t start_orientation;
    const bool* my_map;
    int map_size;
    const int* moves_offset;
    //const int* actions_offset;
    uint64_t num_expanded;
    uint64_t num_generated;
    //const EgraphReader* egr;
    //bool tweak_g_val;
    //double e_weight;  // EGRAPH's inflation factor
    double lower_bound;  // FOCAL's lower bound ( = e_weight * min_f_val)
    double min_f_val;  // min f-val seen so far
    int num_of_conf;
    //int num_non_hwy_edges;
    int num_col;

    vector<int> my_heuristic;  // this is the precomputed heuristic for this agent
    //bool findPathByMDD(bool* res_table, MDD &mdd);

    // note -- handle typedefs is defined inside the class (hence, include node.h is not enough).
    //  Node::open_handle_t open_handle;
    heap_open_t open_list;

    //  Node::focal_handle_t focal_handle;
    heap_focal_t focal_list;

    hashtable_t allNodes_table;

    // used in hash table and would be deleted from the d'tor
    LLNode* empty_node;
    LLNode* deleted_node;


    /* return a pointer to the path found.
    */
    //std::shared_ptr<vector<PathEntry>> getPath() { return path; }  // return a pointer to the path found;


    /* Updates the path datamember (vector<int>).
    After update it will contain the sequence of locations found from the goal to the start.
    */
    void updatePath(const LLNode* goal, vector<PathEntry>& path);  // $$$ make inline?

    /* Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
    */
    std::tuple<ConflictAvoidanceTable, ConflictAvoidanceTable, ConflictAvoidanceTable>
    buildConflictAvoidanceTables(const vector<vector<bool>>& priorities, const vector<vector<PathEntry>*>& current_paths);

    /* Iterate over OPEN and adds to FOCAL all nodes with: 1) f-val > old_min_f_val ; and 2) f-val * f_weight < new_lower_bound.
    */
    void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);

    bool findPath(vector<PathEntry>& path, double f_weight, const vector<vector<bool>>& priorities,
                  const vector<vector<PathEntry>*>& current_paths, size_t max_plan_len, double lowerbound);

    SingleAgentICBS(int id, int start_location, int goal_location, const bool* my_map, int map_size,
                    const int* moves_offset, int num_col);

    ~SingleAgentICBS();

};

