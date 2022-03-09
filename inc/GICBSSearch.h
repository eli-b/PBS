#pragma once
#include <cstdlib>
#include "GICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"


class GICBSSearch
{
public:
    constraint_strategy cons_strategy;
    bool fixed_prior;
    double runtime = 0;
    double pre_runtime = 0;
    double runtime_lowlevel;
    double runtime_conflictdetection;
    double runtime_computeh;
    double runtime_listoperation;
    double runtime_updatepaths;
    double runtime_updatecons;
    uint64_t agent_itself_failed = 0;
    uint64_t lower_priority_agent_failed = 0;
    list<tuple<int, int, int, int, int, int, int>> node_stat;
    //double upper_bound;
    typedef boost::heap::pairing_heap<GICBSNode*, boost::heap::compare<GICBSNode::compare_node>> heap_open_t;  // Note the heap only gives preference to higher depth, making it not a heap but a kind of stack
    //typedef boost::heap::fibonacci_heap< GICBSNode*, boost::heap::compare<GICBSNode::secondary_compare_node> > heap_focal_t;
    //typedef boost::heap::fibonacci_heap< MDDNode*, boost::heap::compare<MDDNode::compare_node> > mdd_open_t;
    //typedef dense_hash_map<GICBSNode*, GICBSNode*, GICBSNode::GICBSNodeHasher, GICBSNode::ecbs_eqnode> hashtable_t;

    heap_open_t open_list;
    //heap_focal_t focal_list;
    //hashtable_t allNodes_table;
    list<std::unique_ptr<GICBSNode>> allNodes_table;

    bool solution_found;
    int solution_cost;

    double focal_w = 1.0;
    double min_f_val;
    double focal_list_threshold;

    const bool* my_map;
    int map_size;
    int num_of_agents;
    const int* actions_offset;
    const int* moves_offset;
    int num_col;
    AgentsLoader al;
    set<int> all_agents;


    uint64_t num_single_pathfinding = 0;

    uint64_t HL_num_expanded = 0;
    uint64_t HL_num_generated = 0;
    uint64_t LL_num_expanded = 0;
    uint64_t LL_num_generated = 0;

    GICBSNode* dummy_start;

    vector<vector<PathEntry>*> paths;
    vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found

    vector<SingleAgentICBS*> search_engines;  // used to find (single) agents' paths and mdd
    bool runGICBSSearch();

    bool findPathForSingleAgent(GICBSNode* node, int ag, double lowerbound = 0);

    bool generateChild(GICBSNode* child, GICBSNode* curr);

    inline void updatePaths(GICBSNode* curr);

    void copyConflicts(const AgentsConflicts& conflicts, AgentsConflicts& copy, 
        const set<int>& excluded_agents);

    bool findAgentsConflicts(GICBSNode& curr, int a1, int a2, uint64_t num=1, size_t start_t=0);
    shared_ptr<Conflict> findEarliestConflict(GICBSNode& curr, int a1, int a2, size_t start_t=0);

    void findConflicts(GICBSNode& curr, uint64_t num);
    void findConflicts(GICBSNode& curr);
    bool isCollide(const GICBSNode& curr, int a1, int a2);
    void selectConflict(GICBSNode& curr);
    void printConflicts(GICBSNode& curr);

    set<int> findMetaAgent(const GICBSNode& curr, int ag, size_t size_th=SIZE_MAX);

    int countCollidingPairs();

    //inline bool updateGICBSNode(GICBSNode* leaf_node, GICBSNode* root_node);
    //inline void updatePaths(GICBSNode* curr, GICBSNode* root_node);
    //void generateChildwithCurrentCost(GICBSNode* n1, const GICBSNode* curr);
    inline int compute_g_val();

    inline int getAgentLocation(int agent_id, size_t timestep);

    void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);

    //void updateReservationTable(bool* res_table, bool* res_table_low_prio, int exclude_agent, const GICBSNode &node);
    inline void releaseClosedListNodes();

    inline void releaseOpenListNodes();

    void printPaths() const;
    void printAgentPath(int ag) const;

    void printConflicts(const GICBSNode& n) const;

    void printConstraints(const GICBSNode* n) const;

    GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr, constraint_strategy c,
                bool fixed_prior = false);

    ~GICBSSearch();
};