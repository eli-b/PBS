#pragma once

#include "SingleAgentICBS.h"

struct pair_hash
{
    template <class T1, class T2>
    size_t operator() (const pair<T1, T2> &pair) const {
        auto h1 = std::hash<T1>{}(pair.first);
        auto h2 = std::hash<T2>{}(pair.second);
        size_t seed = 0;
        boost::hash_combine(seed, h1);
        boost::hash_combine(seed, h2);
        return seed;
    }
};

typedef tuple<int, int, int, int, int> Conflict;
typedef unordered_map<pair<int,int>, list<shared_ptr<Conflict>>, pair_hash> AgentsConflicts;
class GICBSNode
{
public:
    // the following is used to comapre nodes in the OPEN list
    struct compare_node
    {
        bool operator()(const GICBSNode* n1, const GICBSNode* n2) const
        {
            if (n1->depth == n2->depth)
            {
                if (n1->f_val == n2->f_val)
                {
                    if (n1->num_of_colliding_pairs == n2->num_of_colliding_pairs)
                    {
                        if (n1->g_val == n2->g_val)
                        {
                            return n1->time_generated >
                                   n2->time_generated; // break ties towards earilier generated nodes (FIFO manner)
                        }
                        return n1->g_val <= n2->g_val;  // break ties towards larger g_val
                    }
                    return n1->num_of_colliding_pairs >= n2->num_of_colliding_pairs;
                }
                return n1->f_val >= n2->f_val;
            }
            return n1->depth <= n2->depth;  // This is a max heap, so this means we prefer HIGHER depth nodes come out of the heap FIRST.
        }
    };  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

    typedef boost::heap::pairing_heap<GICBSNode*, compare<GICBSNode::compare_node>>::handle_type open_handle_t;

    open_handle_t open_handle;

    // The following is used by googledensehash for generating the hash value of a nodes
    // this is needed because otherwise we'll have to define the specilized template inside std namespace
    struct GICBSNodeHasher
    {
        std::size_t operator()(const GICBSNode* n) const
        {
            size_t agent_id_hash = boost::hash<int>()(n->agent_id);
            size_t time_generated_hash = boost::hash<int>()(n->time_generated);
            return (agent_id_hash ^ (time_generated_hash << 1));
        }
    };

    GICBSNode* parent;
    std::shared_ptr<Conflict> conflict;
    int agent_id;
    list<pair<int, vector<PathEntry>>> new_paths;

    int g_val;
    int h_val;
    int f_val;
    size_t depth;
    size_t makespan;
    int num_of_colliding_pairs;

    uint64_t time_expanded;
    uint64_t time_generated;

    vector<vector<bool>> priorities;
    vector<vector<bool>> trans_priorities;

    // This is for BFS conflict selection
    list<int> ag_open_list;
    inline bool isInOpen(int ag)
    {
        return find(ag_open_list.begin(), ag_open_list.end(), ag) != ag_open_list.end();
    }

    void clear();
};
