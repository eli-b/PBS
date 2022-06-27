#pragma once
#include <cstdlib>
#include "GICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"

#define DEBUG_LOG_BRANCH_ANALYSIS 0
#define DEBUG_LOG_EXPANSION 1
#define DEBUG_LOG_DETAILED 2

class GICBSSearch
{
public:
    constraint_strategy cons_strategy;
    bool fixed_prior;
    double runtime = 0;
    double pre_runtime = 0;
    double runtime_lowlevel;
    double runtime_conflictdetection;
    double runtime_gen_child;
    double runtime_computeh;
    double runtime_listoperation;
    double runtime_updatepaths;
    double runtime_updatecons;
    uint64_t agent_itself_failed = 0;
    uint64_t lower_priority_agent_failed = 0;
    list<tuple<int, int, int, int, int, int, int>> node_stat;
    // Note the heap only gives preference to higher depth, making it not a heap but a kind of stack
    typedef boost::heap::pairing_heap<GICBSNode*, boost::heap::compare<GICBSNode::compare_node>> heap_open_t;

    heap_open_t open_list;
    list<std::unique_ptr<GICBSNode>> allNodes_table;

    bool solution_found;
    int solution_cost;

    double focal_w = 1.0;
    double min_f_val;
    size_t max_ma_size;

    const bool* my_map;
    int map_size;
    int num_of_agents;
    const int* actions_offset;
    const int* moves_offset;
    int num_col;
    AgentsLoader al;
    set<int> all_agents;
    int screen;
    int conf_select_mode;

    uint64_t num_single_pathfinding = 0;

    uint64_t HL_num_expanded = 0;
    uint64_t HL_num_generated = 0;
    uint64_t LL_num_expanded = 0;
    uint64_t LL_num_generated = 0;
    uint64_t num_total_conf = 0;
    uint64_t num_ex_conf = 0;
    uint64_t num_in_conf = 0;
    uint64_t num_0child = 0;
    uint64_t num_1child = 0;
    uint64_t num_2child = 0;

    std::shared_ptr<vector<uint64_t>> br_node_idx;
    std::shared_ptr<vector<int>> br_node_soc;
    std::shared_ptr<vector<size_t>> br_max_ma_size;

    GICBSNode* dummy_start;

    vector<vector<PathEntry>*> paths;
    vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found

    vector<SingleAgentICBS*> search_engines;  // used to find (single) agents' paths and mdd

    GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr, constraint_strategy c,
                bool fixed_prior=false, int scr=0, int mode=0);
    ~GICBSSearch();

    bool runGICBSSearch();
    bool findPathForSingleAgent(GICBSNode* node, int ag, double lowerbound = 0);
    bool generateChild(GICBSNode* child, GICBSNode* curr);
    inline void updatePaths(GICBSNode* curr);
    vector<vector<PathEntry>> copyPaths(const GICBSNode* curr);

    // bool findAgentsConflicts(GICBSNode& curr, int a1, int a2, uint64_t num=1, size_t start_t=0);
    shared_ptr<Conflict> findEarliestConflict(GICBSNode& curr, int a1, int a2, 
        size_t start_t=0, size_t end_t=SIZE_MAX);
    void findConflicts(GICBSNode& curr);
    void findConflictsOri(GICBSNode& curr);
    void findConflictsRandom(GICBSNode& curr);
    void findConflictsminTimestep(GICBSNode& curr);
    void findConflictswithMinMA(GICBSNode& curr);
    void findConflictswithMaxMA(GICBSNode& curr);
    void findConflictsBFS(GICBSNode& curr);
    void findConflictsDFS(GICBSNode& curr);
    void findConflictswithMinConstraints(GICBSNode& curr);
    void findConflictswithMaxConstraints(GICBSNode& curr);
    // void findConflictswithMaxEarliestConf(GICBSNode& curr);

    bool isCollide(int a1, int a2);
    bool isPathsValid(GICBSNode* curr);

    set<int> findMetaAgent(const GICBSNode& curr, int ag, size_t size_th=SIZE_MAX);

    int countCollidingPairs();
    inline int compute_g_val();

    inline int getAgentLocation(int agent_id, size_t timestep);
    int getNumConstraints(const GICBSNode& curr, int a1, int a2);

    inline void releaseClosedListNodes();
    inline void releaseOpenListNodes();

    void printPaths() const;
    void printAgentPath(int ag) const;
    void printAgentPath(int ag, const vector<PathEntry>& in_path) const;
    void printConflicts(const GICBSNode& n) const
    {
        cout << "<a1:" << get<0>(*n.conflict) << ", a2:" << get<1>(*n.conflict) << 
        ", v1:" << get<2>(*n.conflict) << ", v2:" << get<3>(*n.conflict) << 
        ", t:" << get<4>(*n.conflict) << ">" << endl;
    }

    void getBranchEval(GICBSNode* n);
    void saveEval(void);
};