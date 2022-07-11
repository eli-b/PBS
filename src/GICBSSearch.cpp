#include <random>  // For random_device
#include <algorithm>  // For shuffle

#include "GICBSSearch.h"

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void GICBSSearch::updatePaths(GICBSNode* curr)
{
    for (int i = 0; i < num_of_agents; i++)
        paths[i] = &paths_found_initially[i];
    vector<bool> updated(num_of_agents, false);  // initialized for false
    /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
    * because younger nodes take into account ancesstors' nodes constraints. */
    while (curr->parent != NULL)
    {
        for (list<pair<int, vector<PathEntry>>>::iterator it = curr->new_paths.begin();
             it != curr->new_paths.end(); it++)
        {
            if (!updated[it->first])
            {
                paths[it->first] = &(it->second);
                updated[get<0>(*it)] = true;
            }
        }
        curr = curr->parent;
    }
}

vector<vector<PathEntry>> GICBSSearch::copyPaths(const GICBSNode* curr)
{
    vector<vector<PathEntry>> out_paths = paths_found_initially;
    vector<bool> updated(num_of_agents, false);
    const GICBSNode* tmp_node = curr;
    while (tmp_node->parent != nullptr)
    {
        for (const auto& path_pair : tmp_node->new_paths)
        {
            if (!updated[path_pair.first])
            {
                out_paths[path_pair.first] = vector<PathEntry>(path_pair.second);
                updated[path_pair.first] = true;
            }
        }
        tmp_node = tmp_node->parent;
    }
    return out_paths;
}

// bool GICBSSearch::findAgentsConflicts(GICBSNode& curr, int a1, int a2, uint64_t num, size_t start_t)
// {
//     // #ifdef DEBUG
//     // if (a1 == 0 && a2 == 38)
//     //     cout << endl;
//     // shared_ptr<Conflict> __conf__ = findEarliestConflict(curr, a1, a2, start_t);
//     // endif

//     pair<int, int> conf_ags = make_pair(min(a1,a2), max(a1,a2));
//     if (curr.conflicts.count(conf_ags) && curr.conflicts[conf_ags].back() == nullptr)
//         return true;  // All the conflicts are found

//     shared_ptr<Conflict> conf;
//     uint64_t count = 0;
//     size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
//     for (size_t timestep = start_t; timestep < min_path_length; timestep++)
//     {
//         int loc1 = paths[a1]->at(timestep).location;
//         int loc2 = paths[a2]->at(timestep).location;
//         if (loc1 == loc2)
//         {
//             // This is a vertex conflict
//             conf = make_shared<Conflict>(a1, a2, loc1, -1, timestep);
//             if (!curr.conflicts.count(conf_ags))
//                 curr.conflicts[conf_ags] = list<shared_ptr<Conflict>>({conf});
//             else
//                 curr.conflicts[conf_ags].push_back(conf);
//             count ++;
//             if (count == num)
//                 return true;
//         }
//         else if (timestep < min_path_length - 1
//                     && loc1 == paths[a2]->at(timestep + 1).location
//                     && loc2 == paths[a1]->at(timestep + 1).location)
//         {
//             // This is an edge conflict
//             conf = make_shared<Conflict>(a1, a2, loc1, loc2, timestep + 1);
//             if (!curr.conflicts.count(conf_ags))
//                 curr.conflicts[conf_ags] = list<shared_ptr<Conflict>>({conf});
//             else
//                 curr.conflicts[conf_ags].push_back(conf);
//             count ++;
//             if (count == num)
//                 return true;
//         }
//     }
//     if (paths[a1]->size() != paths[a2]->size())
//     {
//         int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
//         int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
//         int loc1 = paths[a1_]->back().location;
//         for (size_t timestep = max(start_t, min_path_length); timestep < paths[a2_]->size(); timestep++)
//         {
//             int loc2 = paths[a2_]->at(timestep).location;
//             if (loc1 == loc2)
//             {
//                 conf = make_shared<Conflict>(a1_, a2_, loc1, -1, timestep);
//                 if (!curr.conflicts.count(conf_ags))
//                     curr.conflicts[conf_ags] = list<shared_ptr<Conflict>>({conf});
//                 else
//                     curr.conflicts[conf_ags].push_back(conf);
//                 count ++;
//                 if (count == num)
//                     return true;
//             }
//         }
//     }
//     // Add a pseudo conflict at the end if all the conflicts are founded
//     // Check if conf_ags is in the key
//     // Check if there are conflicts
//     if (curr.conflicts.count(conf_ags) && curr.conflicts[conf_ags].size() > 0)
//     {
//         curr.conflicts[conf_ags].push_back(nullptr);
//         return true;
//     }
//     assert(!curr.conflicts.count(conf_ags));
//     return false;
// }

shared_ptr<Conflict> GICBSSearch::findEarliestConflict(GICBSNode& curr, int a1, int a2, 
    size_t start_t, size_t end_t)
{
    if (a1 == a2) return nullptr;
    size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
    for (size_t timestep = start_t; timestep < min_path_length; timestep++)
    {
        if (timestep > end_t)
            return nullptr;

        int loc1 = paths[a1]->at(timestep).location;
        int loc2 = paths[a2]->at(timestep).location;
        if (loc1 == loc2)
        {
            // This is a vertex conflict
            return make_shared<Conflict>(a1, a2, loc1, -1, timestep);
        }
        else if (timestep < min_path_length - 1
            && loc1 == paths[a2]->at(timestep + 1).location
            && loc2 == paths[a1]->at(timestep + 1).location)
        {
            // This is an edge conflict
            return make_shared<Conflict>(a1, a2, loc1, loc2, timestep + 1);
        }
    }
    if (paths[a1]->size() != paths[a2]->size())
    {
        int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
        int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
        int loc1 = paths[a1_]->back().location;
        for (size_t timestep = max(start_t, min_path_length); timestep < paths[a2_]->size(); timestep++)
        {
            if (timestep > end_t)
                return nullptr;

            int loc2 = paths[a2_]->at(timestep).location;
            if (loc1 == loc2)
            {
                return make_shared<Conflict>(a1_, a2_, loc1, -1, timestep);
            }
        }
    }
    return nullptr;
}

bool GICBSSearch::isCollide(int a1, int a2)
{
    size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
    for (size_t timestep = 0; timestep < min_path_length; timestep++)
    {
        int loc1 = paths[a1]->at(timestep).location;
        int loc2 = paths[a2]->at(timestep).location;
        if (loc1 == loc2)
        {
            // This is a vertex conflict
            cout << endl;
            cout << "*********************************" << endl;
            cout << a1 << " and " << a2 << " collide at " << loc1 << 
                " at time " << timestep << endl;
            printAgentPath(a1);
            printAgentPath(a2);
            cout << "*********************************" << endl;
            return true;
        }
        else if (timestep < min_path_length - 1
                    && loc1 == paths[a2]->at(timestep + 1).location
                    && loc2 == paths[a1]->at(timestep + 1).location)
        {
            // This is an edge conflict
            cout << endl;
            cout << "*********************************" << endl;
            cout << a1 << " and " << a2 << " collide at " << loc1 << " and " << loc2 <<
                " at time " << timestep << endl;
            printAgentPath(a1);
            printAgentPath(a2);
            cout << "*********************************" << endl;
            return true;
        }
    }
    if (paths[a1]->size() != paths[a2]->size())
    {
        int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
        int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
        int loc1 = paths[a1_]->back().location;
        for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
        {
            int loc2 = paths[a2_]->at(timestep).location;
            if (loc1 == loc2)
            {
                cout << endl;
                cout << "*********************************" << endl;
                cout << a1_ << " and " << a2_ << " collide at goal " << loc1 << " at time " << 
                    timestep << endl;
                printAgentPath(a1);
                printAgentPath(a2);
                cout << "*********************************" << endl;
                return true;
            }
        }
    }
    return false;
}

bool GICBSSearch::isPathsValid(GICBSNode* curr)
{
    bool is_collide = false;
    vector<vector<PathEntry>*> copy_paths(paths);
    updatePaths(curr);

    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1+1; a2 < num_of_agents; a2++)
        {
            if (curr->priorities[a1][a2] || curr->priorities[a2][a1])
            {
                is_collide = isCollide(a1, a2);
                if (is_collide)
                {
                    if (curr->priorities[a1][a2])
                        cout << "priorities: " << right << setw(2) << a1 << "->" << right << setw(2) << a2 << endl;
                    else
                        cout << "priorities: " << right << setw(2) << a2 << "->" << right << setw(2) << a1 << endl;
                }
                assert(!is_collide);
            }
        }
    }
    paths = copy_paths;
    return !is_collide;
}

void GICBSSearch::findConflictsRandom(GICBSNode& curr)
{
    curr.conflict = nullptr;  // Initialize the conflict pointer to nullptr

    vector<int> new_ag(num_of_agents);
    iota(new_ag.begin(), new_ag.end(), 0);
    random_shuffle(new_ag.begin(), new_ag.end());

    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, new_ag[a1], new_ag[a2]);
            if (tmp_conf != nullptr) 
            {
                curr.conflict = tmp_conf;
                num_total_conf++;
                // Evaluate the number of the external and internal conflicts
                set<int> ma1 = findMetaAgent(curr, new_ag[a1]);
                if (find(ma1.begin(), ma1.end(), new_ag[a2]) != ma1.end()) num_in_conf++;
                else num_ex_conf++;
                return;
            }
        }
    }
    return;
}

void GICBSSearch::findConflictswithMinMA(GICBSNode& curr)
{
    if (curr.parent == nullptr)
    {
        shared_ptr<Conflict> tmp_conf = nullptr;
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            for (int a2 = a1+1; a2 < num_of_agents; a2++)
            {
                tmp_conf = findEarliestConflict(curr, a1, a2);
                if (tmp_conf != nullptr)
                {
                    curr.conflict = tmp_conf;
                    num_total_conf++;
                    num_ex_conf++;
                    return;
                }
            }
        }
        assert(tmp_conf == nullptr);
        curr.conflict = tmp_conf;  // There is no conflict in the root node!
    }
    else
    {
        // Find the internal conflict of the previously-merged agent
        // We must find the internal conflicts first in order to make the new PBS works
        assert(curr.agent_id != -1);
        set<int> curr_ma = findMetaAgent(curr, curr.agent_id);
        for (auto it1=curr_ma.begin(); it1!=curr_ma.end(); it1++)
        {
            for (auto it2=next(it1); it2!=curr_ma.end(); it2++)
            {
                shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, *it1, *it2);
                if (tmp_conf != nullptr)
                {
                    curr.conflict = tmp_conf;
                    num_total_conf++;
                    num_in_conf++;
                    return;
                }
            }
        }

        size_t min_total_size = SIZE_MAX;
        list<shared_ptr<Conflict>> min_size_conf;
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            set<int> ma1 = findMetaAgent(curr, a1, min_total_size+1);
            if (ma1.size() > min_total_size) continue;

            for (int a2 = a1+1; a2 < num_of_agents; a2++)
            {
                if (find(ma1.begin(), ma1.end(), a2) != ma1.end())  continue;  // a2 is in ma1
                set<int> ma2 = findMetaAgent(curr, a2, min_total_size+1);
                if (ma2.size() > min_total_size || ma1.size()+ma2.size() > min_total_size) continue;

                shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, a1, a2);
                if (tmp_conf == nullptr) continue;  // skip if there is no conflict
                if (ma1.size() + ma2.size() < min_total_size)
                {
                    min_total_size = ma1.size() + ma2.size();
                    min_size_conf.clear();
                }
                min_size_conf.emplace_back(tmp_conf);
            }
        }

        if (min_size_conf.empty())
        {
            curr.conflict = nullptr;  // It is a goal node (conflict-free)
            return;
        }

        // list<shared_ptr<Conflict>>::iterator cit = max_size_conf.begin();  // Choose randomly
        // int random = rand() % max_size_conf.size();
        // advance(cit, random);
        // curr.conflict = *cit;

        int min_conf_t = INT_MAX;  // Choose the earliest conflict
        for (const auto& conf : min_size_conf)
        {
            if (get<4>(*conf) < min_conf_t)
            {
                curr.conflict = conf;
                min_conf_t = get<4>(*conf);
            }
        }
        num_ex_conf++;
        num_total_conf++;
    }

    return;
}

void GICBSSearch::findConflictswithMaxMA(GICBSNode& curr)
{
    if (curr.parent == nullptr)
    {
        shared_ptr<Conflict> tmp_conf = nullptr;
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            for (int a2 = a1+1; a2 < num_of_agents; a2++)
            {
                tmp_conf = findEarliestConflict(curr, a1, a2);
                if (tmp_conf != nullptr)
                {
                    curr.conflict = tmp_conf;
                    num_ex_conf++;
                    num_total_conf++;
                    return;
                }
            }
        }
        assert(tmp_conf == nullptr);
        curr.conflict = tmp_conf;  // There is no conflict in the root node!
    }
    else
    {
        // Find the internal conflict of the previously-merged agent
        // We must find the internal conflicts first in order to make the new PBS works
        assert(curr.agent_id != -1);
        set<int> curr_ma = findMetaAgent(curr, curr.agent_id);
        for (auto it1=curr_ma.begin(); it1!=curr_ma.end(); it1++)
        {
            for (auto it2=next(it1); it2!=curr_ma.end(); it2++)
            {
                shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, *it1, *it2);
                if (tmp_conf != nullptr)
                {
                    curr.conflict = tmp_conf;
                    num_in_conf++;
                    num_total_conf++;
                    return;
                }
            }
        }

        size_t max_total_size = 0;
        list<shared_ptr<Conflict>> max_size_conf;
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            set<int> ma1 = findMetaAgent(curr, a1, SIZE_MAX);
            for (int a2 = a1+1; a2 < num_of_agents; a2++)
            {
                if (find(ma1.begin(), ma1.end(), a2) != ma1.end()) continue;  // a2 is in ma1
                set<int> ma2 = findMetaAgent(curr, a2, SIZE_MAX);
                if (ma1.size() + ma2.size() >= max_total_size)
                {
                    shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, a1, a2);
                    if (tmp_conf == nullptr)  continue;  // skip there is no conflict
                    if (ma1.size() + ma2.size() > max_total_size)
                    {
                        max_total_size = ma1.size() + ma2.size();
                        max_size_conf.clear();
                    }
                    max_size_conf.emplace_back(tmp_conf);
                }
            }
        }

        if (max_size_conf.empty())
        {
            curr.conflict = nullptr;  // It is a goal node (conflict-free)
            return;
        }

        // list<shared_ptr<Conflict>>::iterator cit = max_size_conf.begin();  // Choose randomly
        // int random = rand() % max_size_conf.size();
        // advance(cit, random);
        // curr.conflict = *cit;

        int min_conf_t = INT_MAX;  // Choose the earliest
        for (const auto& conf : max_size_conf)
        {
            if (get<4>(*conf) < min_conf_t)
            {
                curr.conflict = conf;
                min_conf_t = get<4>(*conf);
            }
        }
        num_ex_conf++;
        num_total_conf++;
    }

    return;
}


// void GICBSSearch::findConflictswithMaxEarliestConf(GICBSNode& curr)
// {
//     unordered_map<int, set<shared_ptr<Conflict>>> ag_conf;
//     for (int a1 = 0; a1 < num_of_agents; a1++)
//     {
//         for (int a2 = a1+1; a2 < num_of_agents; a2++)
//         {
//             shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, a1, a2);
//             if (tmp_conf != nullptr)
//             {
//                 if (ag_conf.count(a1) == 0)
//                 {
//                     ag_conf[a1] = set<shared_ptr<Conflict>>({tmp_conf});
//                 }
//                 else
//                 {
//                     assert(ag_conf[a1].size() > 0);
//                     ag_conf[a1].insert(tmp_conf);
//                 }
//             }
//         }
//     }

//     if (ag_conf.size() == 0)  // Conflict-free
//         curr.conflict = nullptr;

//     set<int> max_conf_ags;
//     size_t max_num_conf = 0;
//     for (const auto& conf : ag_conf)
//     {
//         if (conf.second.size() >= max_num_conf)
//         {
//             if (conf.second.size() > max_num_conf)
//             {
//                 max_conf_ags.clear();
//                 max_num_conf = conf.second.size();
//             }
//             max_conf_ags.insert(conf.first);
//         }
//     }

//     set<shared_ptr<Conflict>> tmp_conf_set;
//     for (const int& ag : max_conf_ags)
//     {
//         for (const auto& conf : ag_conf[ag])
//         {
//             tmp_conf_set.insert(conf);
//         }
//     }

//     // Debug: check duplication
//     for (auto it=tmp_conf_set.begin(); it!=tmp_conf_set.end(); it++)
//     {
//         for (auto it2=next(it); it2!=tmp_conf_set.end(); it2++)
//         {
//             bool is_same = (get<0>(**it) == get<0>(**it2)) && (get<1>(**it) == get<1>(**it2)) &&
//                 (get<2>(**it) == get<2>(**it2)) && (get<3>(**it) == get<3>(**it2)) && 
//                 (get<4>(**it) == get<4>(**it2));
//             if (is_same)
//             {
//                 cout << "Should not be the same!!!" << endl;
//                 assert(!is_same);
//             }
//         }
//     }

//     int min_timestep = INT_MAX;
//     assert(curr.conflict == nullptr);
//     for (const auto& conf : tmp_conf_set)
//     {
//         int cur_timestep = get<4>(*conf);
//         if (cur_timestep < min_timestep)
//         {
//             min_timestep = cur_timestep;
//             curr.conflict = conf;
//         }  
//     }
// }

void GICBSSearch::findConflictsBFS(GICBSNode& curr)
{
    if (curr.parent == nullptr)
    {
        shared_ptr<Conflict> tmp_conf = nullptr;
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            for (int a2 = a1+1; a2 < num_of_agents; a2++)
            {
                tmp_conf = findEarliestConflict(curr, a1, a2);
                if (tmp_conf != nullptr)
                {
                    curr.conflict = tmp_conf;
                    num_ex_conf++;
                    num_total_conf++;
                    return;
                }
            }
        }
        assert(tmp_conf == nullptr);
        curr.conflict = tmp_conf;  // There is no conflict in the root node!
    }
    else
    {
        // Find the conflict in the top agent in the open list of curr
        // The purpose of the agent open list is to guarantee the conflicting agent is connected
        set<int> remain_ag = all_agents;
        shared_ptr<Conflict> tmp_conf = nullptr;
        while (!curr.ag_open_list.empty())
        {
            int top_ag = curr.ag_open_list.front();
            for (int a2 = 0; a2 < num_of_agents; a2++)
            {
                // Ignore this agent if it is the top agent itself or connected agent
                if (a2 == top_ag || curr.priorities[top_ag][a2] || curr.priorities[a2][top_ag])
                {
                    assert(findEarliestConflict(curr, top_ag, a2) == nullptr);
                    continue;
                }

                tmp_conf = findEarliestConflict(curr, top_ag, a2);
                if (tmp_conf != nullptr)  // Put the other conflicting agent into the open list
                {
                    curr.conflict = tmp_conf;
                    num_in_conf++;
                    num_total_conf++;
                    return;  // break the for loop
                }
            }
            if (tmp_conf == nullptr)  // the top agent has no conflicts
            {
                // If there is no conflict for the top agent, then pop the top agent to the close list
                curr.ag_open_list.pop_front();
                remain_ag.erase(top_ag);
            }
        }

        if (tmp_conf == nullptr)  // If the conflict is still not found in the agent open list
        {
            for (set<int>::iterator it1 = remain_ag.begin(); it1 != remain_ag.end(); it1++)
            {
                for (set<int>::iterator it2 = next(it1); it2 != remain_ag.end(); it2++)
                {
                    tmp_conf = findEarliestConflict(curr, *it1, *it2);
                    if (tmp_conf != nullptr)
                    {
                        assert(!curr.isInOpen(*it1));
                        assert(!curr.isInOpen(*it2));
                        curr.conflict = tmp_conf;
                        num_ex_conf++;
                        num_total_conf++;
                        return;
                    }
                }
            }
        }
        curr.conflict = tmp_conf;
    }
}

void GICBSSearch::findConflictsDFS(GICBSNode& curr)
{
    if (curr.parent == nullptr)
    {
        shared_ptr<Conflict> tmp_conf = nullptr;
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            for (int a2 = a1+1; a2 < num_of_agents; a2++)
            {
                tmp_conf = findEarliestConflict(curr, a1, a2);
                if (tmp_conf != nullptr)
                {
                    curr.conflict = tmp_conf;
                    num_ex_conf++;
                    num_total_conf++;
                    return;
                }
            }
        }
        assert(tmp_conf == nullptr);
        curr.conflict = tmp_conf;  // There is no conflict in the root node!
    }
    else
    {
        // Find the conflict in the top agent in the open list of curr
        // The purpose of the agent open list is to guarantee the conflicting agent is connected
        set<int> remain_ag = all_agents;
        shared_ptr<Conflict> tmp_conf = nullptr;
        while (!curr.ag_open_list.empty())
        {
            int top_ag = curr.ag_open_list.back();
            for (int a2 = 0; a2 < num_of_agents; a2++)
            {
                // Ignore this agent if it is the top agent itself or connected agent
                if (a2 == top_ag || curr.priorities[top_ag][a2] || curr.priorities[a2][top_ag])
                {
                    assert(findEarliestConflict(curr, top_ag, a2) == nullptr);
                    continue;
                }

                tmp_conf = findEarliestConflict(curr, top_ag, a2);
                if (tmp_conf != nullptr)  // Put the other conflicting agent into the open list
                {
                    curr.conflict = tmp_conf;
                    num_in_conf++;
                    num_total_conf++;
                    return;  // break the for loop
                }
            }
            if (tmp_conf == nullptr)  // the top agent has no conflicts
            {
                // If there is no conflict for the top agent, then pop the top agent to the close list
                curr.ag_open_list.pop_back();
                remain_ag.erase(top_ag);
            }
        }

        if (tmp_conf == nullptr)  // If the conflict is still not found in the agent open list
        {
            for (set<int>::iterator it1 = remain_ag.begin(); it1 != remain_ag.end(); it1++)
            {
                for (set<int>::iterator it2 = next(it1); it2 != remain_ag.end(); it2++)
                {
                    tmp_conf = findEarliestConflict(curr, *it1, *it2);
                    if (tmp_conf != nullptr)
                    {
                        assert(!curr.isInOpen(*it1));
                        assert(!curr.isInOpen(*it2));
                        curr.conflict = tmp_conf;
                        num_ex_conf++;
                        num_total_conf++;
                        return;
                    }
                }
            }
        }
        curr.conflict = tmp_conf;
    }
}

void GICBSSearch::findConflictswithMinConstraints(GICBSNode& curr)
{
    // Find all conflicting agent pairs
    shared_ptr<Conflict> tmp_conf = nullptr;
    int min_num_constraints = INT_MAX;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1+1; a2 < num_of_agents; a2++)
        {
            tmp_conf = findEarliestConflict(curr, a1, a2);
            if (tmp_conf == nullptr) continue;

            int curr_num_constraints = getNumConstraints(curr, a1, a2);
            if (curr_num_constraints < min_num_constraints)
            {
                // Set the priority order between a1 and a2 (a1->a2)
                get<0>(*tmp_conf) = a1;
                get<1>(*tmp_conf) = a2;
                curr.conflict = tmp_conf;
                min_num_constraints = curr_num_constraints;
            }

            curr_num_constraints = getNumConstraints(curr, a2, a1);
            if (curr_num_constraints < min_num_constraints)
            {
                // Set the priority order between a1 and a2 (a2->a1)
                get<0>(*tmp_conf) = a2;
                get<1>(*tmp_conf) = a1;
                curr.conflict = tmp_conf;
                min_num_constraints = curr_num_constraints;
            }
        }
    }
    if (curr.conflict != nullptr)  // Evaluate the number external & internal conflicts
    {
        num_total_conf++;
        set<int> ma1 = findMetaAgent(curr, get<0>(*curr.conflict));
        if (find(ma1.begin(), ma1.end(), get<1>(*curr.conflict)) != ma1.end()) num_in_conf++;
        else num_ex_conf++;
    }
}


void GICBSSearch::findConflictswithMaxConstraints(GICBSNode& curr)
{
    // Find all conflicting agent pairs
    shared_ptr<Conflict> tmp_conf = nullptr;
    int max_num_constraints = 0;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1+1; a2 < num_of_agents; a2++)
        {
            tmp_conf = findEarliestConflict(curr, a1, a2);
            if (tmp_conf == nullptr) continue;

            int curr_num_constraints = getNumConstraints(curr, a1, a2);
            if (curr_num_constraints > max_num_constraints)
            {
                // Set the priority order between a1 and a2 (a1 -> a2)
                get<0>(*tmp_conf) = a1;
                get<1>(*tmp_conf) = a2;
                curr.conflict = tmp_conf;
                max_num_constraints = curr_num_constraints;
            }

            curr_num_constraints = getNumConstraints(curr, a2, a1);
            if (curr_num_constraints > max_num_constraints)
            {
                // Set the priority order between a1 and a2 (a2 -> a1)
                get<0>(*tmp_conf) = a2;
                get<1>(*tmp_conf) = a1;
                curr.conflict = tmp_conf;
                max_num_constraints = curr_num_constraints;
            }
        }
    }
    if (curr.conflict != nullptr)  // Evaluate the number external & internal conflicts
    {
        num_total_conf++;
        set<int> ma1 = findMetaAgent(curr, get<0>(*curr.conflict));
        if (find(ma1.begin(), ma1.end(), get<1>(*curr.conflict)) != ma1.end()) num_in_conf++;
        else num_ex_conf++;
    }
}

set<int> GICBSSearch::findMetaAgent(const GICBSNode& curr, int ag, size_t size_th)
{
    // Use DFS to find the connected component
    // TODO: memorize the meta-agent in the curr node
    set<int> ma;
    vector<int> is_visited(num_of_agents, false);
    list<int> ma_open_list = list<int>({ag});
    is_visited[ag] = true;
    while(!ma_open_list.empty())
    {
        int cur_ag = ma_open_list.front();
        ma.insert(cur_ag);
        if (ma.size() == size_th)
            break;
        ma_open_list.pop_front();
        for (int a2 = 0; a2 < num_of_agents; a2++)
        {
            if(((curr.priorities[cur_ag][a2] && !curr.priorities[a2][cur_ag]) || 
                (curr.priorities[a2][cur_ag] && !curr.priorities[cur_ag][a2])) && 
                !is_visited[a2])
            {
                assert(find(ma.begin(), ma.end(), a2) == ma.end());
                is_visited[a2] = true;
                ma_open_list.push_front(a2);
            }
        }
    }
    // ma.sort();
    return ma;
}

void GICBSSearch::findConflictsOri(GICBSNode& curr)
{
    curr.conflict = nullptr;  // Initialize the conflict pointer to nullptr
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, a1, a2);
            if (tmp_conf != nullptr)
            {
                curr.conflict = tmp_conf;
                num_total_conf++;
                set<int> ma1 = findMetaAgent(curr, a1);
                if (find(ma1.begin(), ma1.end(), a2) != ma1.end()) num_in_conf++;
                else num_ex_conf++;
                return;
            }
        }
    }
    return;
}

void GICBSSearch::findConflictsminTimestep(GICBSNode& curr)
{
    vector<int> new_ag(num_of_agents);
    iota(new_ag.begin(), new_ag.end(), 0);
    random_shuffle(new_ag.begin(), new_ag.end());

    curr.conflict = nullptr;
    for (size_t t_step = 0; t_step < curr.makespan+1; t_step++)
    {
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            for (int a2 = a1+1; a2 < num_of_agents; a2++)
            {
                shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, new_ag[a1], new_ag[a2], t_step, t_step);
                if (tmp_conf != nullptr)  // Guarding 
                {
                    curr.conflict = tmp_conf;
                    num_total_conf ++;
                    set<int> ma1 = findMetaAgent(curr, new_ag[a1]);
                    if (find(ma1.begin(), ma1.end(), new_ag[a2]) != ma1.end()) num_in_conf++;
                    else num_ex_conf++;
                    return;
                }
            }
        }
    }
    return;
}

void GICBSSearch::findConflicts(GICBSNode& curr)
{
    if (screen > DEBUG_LOG_EXPANSION) 
        cout << "node " << right << setw(3) << curr.time_generated << " conflict: ";
    switch (conf_select_mode)
    {
    case 0:
        findConflictsOri(curr);
        break;
    case 1:
        findConflictsRandom(curr);
        break;
    case 2:
        findConflictsminTimestep(curr);
        break;
    case 3:
        findConflictswithMinMA(curr);
        break;
    case 4:
        findConflictswithMaxMA(curr);
        break;
    case 5:
        findConflictsBFS(curr);
        break;
    case 6:
        findConflictsDFS(curr);
        break;
    case 7:
        findConflictswithMinConstraints(curr);
        break;
    case 8:
        findConflictswithMaxConstraints(curr);
        break;
    default:
        exit(-1);
        break;
    }
}

// Based on the <paths> member. Consider making it take a node parameter.
int GICBSSearch::countCollidingPairs()
{
    int result = 0;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            bool isColliding = false;
            size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
            for (size_t timestep = 0; timestep < min_path_length; timestep++)
            {
                int loc1 = paths[a1]->at(timestep).location;
                int loc2 = paths[a2]->at(timestep).location;
                if (loc1 == loc2)
                {
                    result++;
                    isColliding = true;
                    break;
                }
                else if (timestep < min_path_length - 1 &&
                         loc1 == paths[a2]->at(timestep + 1).location &&
                         loc2 == paths[a1]->at(timestep + 1).location)
                {
                    result++;
                    isColliding = true;
                    break;
                }
            }

            // Check for a vertex collision after one of the agents has finished its path
            if (!isColliding && paths[a1]->size() != paths[a2]->size())
            {
                int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
                int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
                int loc1 = paths[a1_]->back().location;
                for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
                {
                    int loc2 = paths[a2_]->at(timestep).location;
                    if (loc1 == loc2)
                    {
                        result++;
                        break;
                    }
                }
            }
        }
    }
    return result;
}


/*
return agent_id's location for the given timestep
Note -- if timestep is longer than its plan length,
then the location remains the same as its last cell)
*/
inline int GICBSSearch::getAgentLocation(int agent_id, size_t timestep)
{
    // if last timestep > plan length, agent remains in its last location
    if (timestep >= paths[agent_id]->size())
        return paths[agent_id]->at(paths[agent_id]->size() - 1).location;
    // otherwise, return its location for that timestep
    return paths[agent_id]->at(timestep).location;
}


int GICBSSearch::getNumConstraints(const GICBSNode& curr, int a1, int a2)
{
    int num_high_pri_ag = 0;
    int num_low_pri_ag = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        if (i == a1) assert(!curr.priorities[i][a1]);
        if (curr.priorities[i][a1]) num_high_pri_ag ++;
    }
    for (int i = 0; i < num_of_agents; i++)
    {
        if (i == a2) assert(!curr.priorities[a2][i]);
        if (curr.priorities[a2][i]) num_low_pri_ag ++;
    }

    return (num_high_pri_ag+1) * (num_low_pri_ag+1);
}


// May absolutely find new paths for multiple agents!
bool GICBSSearch::findPathForSingleAgent(GICBSNode* node, int ag, double lowerbound)
{
    bool foundSol = true;
    vector<vector<PathEntry>> new_paths(num_of_agents, vector<PathEntry>());

    // Prepare a topological sort of the agents from ag and below it.
    vector<bool> visited(num_of_agents, true);
    for (int i = 0; i < num_of_agents; i++)
    {
        if (node->priorities[ag][i])
        {
            visited[i] = false;
        }
    }
    stack<pair<bool, int>> dfs;
    list<int> topSort;
    dfs.push(make_pair(false, ag));  // <Whether we've already opened it, The agent>
    while (!dfs.empty())
    {
        pair<bool, int> parent = dfs.top();
        dfs.pop();
        if (parent.first)
        {
            topSort.push_front(parent.second);
            continue;
        }
        visited[parent.second] = true;
        dfs.push(make_pair(true, parent.second));
        for (int i = 0; i < num_of_agents; i++)
        {
            if (node->priorities[parent.second][i] && !visited[i])
            {
                dfs.push(make_pair(false, i));
            }
        }
    }

    if (screen > DEBUG_LOG_DETAILED)
    {
        cout << "\tTopo: [";
        for (const auto& a : topSort)
        {
            cout << right << setw(2) << a;
            if (a != topSort.back())
                cout << ",";
        }
        cout << "] | ";
    }

    // Find a new path for ag and for every lower-priority agent that has a collision with a new path.
    for (auto iter = topSort.begin(); iter != topSort.end(); iter++)
    {
        int curr_agent = *iter;
        bool isColliding = false;
        for (int a2 = 0; a2 < num_of_agents; a2++)
        {
            if (node->priorities[a2][curr_agent])
            {
                size_t min_path_length =
                        paths[curr_agent]->size() < paths[a2]->size() ? paths[curr_agent]->size() : paths[a2]->size();
                for (size_t timestep = 0; timestep < min_path_length; timestep++)
                {
                    int loc1 = paths[curr_agent]->at(timestep).location;
                    int loc2 = paths[a2]->at(timestep).location;
                    if (loc1 == loc2)
                    {
                        isColliding = true;
                        break;
                    }
                    else if (timestep < min_path_length - 1
                             && loc1 == paths[a2]->at(timestep + 1).location
                             && loc2 == paths[curr_agent]->at(timestep + 1).location)
                    {
                        isColliding = true;
                        break;
                    }
                }
                if (!isColliding && paths[curr_agent]->size() != paths[a2]->size())
                {
                    int a1_ = paths[curr_agent]->size() < paths[a2]->size() ? curr_agent : a2;
                    int a2_ = paths[curr_agent]->size() < paths[a2]->size() ? a2 : curr_agent;
                    int loc1 = paths[a1_]->back().location;
                    for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
                    {
                        int loc2 = paths[a2_]->at(timestep).location;
                        if (loc1 == loc2)
                        {
                            isColliding = true;
                            break;
                        }
                    }
                }
            }
            if (isColliding)
            {
                break;
            }
        }
        if (!isColliding && curr_agent != ag)  // Current agent isn't colliding with any higher-priority agent in the
                                               // topological sort and isn't the original agent we need to find a path
                                               // for - no need to find a new path for it
        {
            if (screen > DEBUG_LOG_DETAILED) cout << "ag " << right << setw(2) << curr_agent << " no-conf, ";
            continue;
        }
        size_t max_plan_len = node->makespan + 1; //getPathsMaxLength();
        num_single_pathfinding++;

        foundSol = search_engines[curr_agent]->findPath(new_paths[curr_agent], focal_w, node->priorities, paths,
                                                        max_plan_len, lowerbound);        
        LL_num_expanded += search_engines[curr_agent]->num_expanded;
        LL_num_generated += search_engines[curr_agent]->num_generated;
        if (foundSol)
        {
            node->g_val = node->g_val - paths[curr_agent]->size() + new_paths[curr_agent].size();
            paths[curr_agent] = &new_paths[curr_agent]; // might be used by the next findPath() call
            node->makespan = max(node->makespan, new_paths[curr_agent].size() - 1);
        }
        else
        {
            if (curr_agent == node->agent_id) agent_itself_failed ++;
            else lower_priority_agent_failed ++;
            return false;
        }
    }
    if (foundSol)
    {
        if (screen > DEBUG_LOG_EXPANSION) cout << "replan: [";
        for (int i = 0; i < num_of_agents; i++)
        {
            if (!new_paths[i].empty())
            {
                node->new_paths.push_back(make_pair(i, new_paths[i]));
                paths[i] = &node->new_paths.back().second; // make sure i gets the correct pointer
                if (screen > DEBUG_LOG_EXPANSION) cout << right << setw(2) << i << ",";
            }
        }
        if (screen > DEBUG_LOG_EXPANSION) cout << "] -> ";
    }
    return true;
}

bool GICBSSearch::generateChild(GICBSNode* node, GICBSNode* curr)
{
    clock_t t0 = clock();
    node->parent = curr;
    node->g_val = curr->g_val;
    node->makespan = curr->makespan;
    node->depth = curr->depth + 1;

    clock_t t1 = clock();
    bool path_found = findPathForSingleAgent(node, node->agent_id);
    runtime_lowlevel += clock() - t1;
    if (!path_found) return false;

    node->f_val = node->g_val;
    runtime_gen_child += clock() - t0;
    return true;
}


void GICBSSearch::printPaths() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        cout << "Agent " << right << setw(2) << i << " (" << paths_found_initially[i].size() - 1 << 
            " -->" << paths[i]->size() - 1 << "): ";
        for (size_t t = 0; t < paths[i]->size(); t++)
            cout << right << setw(3) << paths[i]->at(t).location << ",";
        cout << endl;
    }
}


void GICBSSearch::printAgentPath(int ag) const
{
    cout << "Agent " << right << setw(2) << ag << " (" << paths_found_initially[ag].size() - 1 << 
        " ->" << paths[ag]->size() - 1 << "): ";
    for (size_t t = 0; t < paths[ag]->size(); t++)
        cout << right << setw(3) << paths[ag]->at(t).location << ",";
    cout << endl;
}


void GICBSSearch::printAgentPath(int ag, const vector<PathEntry>& in_path) const
{
    cout << "Agent " << right << setw(2) << ag << " (" << paths_found_initially[ag].size() - 1 << 
        " ->" << in_path.size() - 1 << "): ";
    for (size_t t = 0; t < in_path.size(); t++)
        cout << right << setw(3) << in_path[t].location << ",";
    cout << endl;
}

// computes g_val based on current paths
inline int GICBSSearch::compute_g_val()
{
    int retVal = 0;
    for (int i = 0; i < num_of_agents; i++)
        retVal += paths[i]->size() - 1;
    return retVal;
}

bool GICBSSearch::runGICBSSearch()
{
    if (screen > DEBUG_LOG_BRANCH_ANALYSIS) cout << "PBS: " << endl;
    if (solution_cost == -2)
    {
        runtime = pre_runtime;
        return false;
    }
    // set timer
    clock_t start;
    start = clock();
    clock_t t1;
    runtime_computeh = 0;
    runtime_lowlevel = 0;
    runtime_listoperation = 0;
    runtime_conflictdetection = 0;
    runtime_gen_child = 0;
    runtime_updatepaths = 0;
    runtime_updatecons = 0;
    // start is already in the open_list
    GICBSNode* curr = nullptr;
    while (!open_list.empty() && !solution_found)
    {
        // break after 1 minute
        runtime = (clock() - start) + pre_runtime;
        if (runtime > TIME_LIMIT || HL_num_expanded > 1000000)
        {
            // timeout after 1 minutes or 1000000 expanded nodes
            size_t max_size = 0;
            for (int tmp_a = 0; tmp_a < num_of_agents; tmp_a++)
            {
                set<int> tmp_ma = findMetaAgent(*curr, tmp_a);
                if (tmp_ma.size() > max_size)
                    max_size = tmp_ma.size();
            }
            max_ma_size = max_size;

            cout << "Timeout, runtime=" << runtime / CLOCKS_PER_SEC <<
                ", solution delta=" << min_f_val - dummy_start->g_val <<
                ", HL exp(gen)=" << HL_num_expanded << "(" << HL_num_generated << ")" <<
                ", LL exp(gen)=" << LL_num_expanded << "(" << LL_num_generated << ")" <<
                ", max_ma_size=" << max_ma_size << endl;

            if (screen > DEBUG_LOG_DETAILED)
            {
                cout << "\tRuntime summary: lowlevel = " << runtime_lowlevel / CLOCKS_PER_SEC << 
                    ", listoperation = " << runtime_listoperation / CLOCKS_PER_SEC <<
                    ", conflictdetection = " << runtime_conflictdetection / CLOCKS_PER_SEC << 
                    ", computeh = " << runtime_computeh / CLOCKS_PER_SEC <<
                    ", updatepaths = " << runtime_updatepaths / CLOCKS_PER_SEC << 
                    ", collectcons = " << runtime_updatecons / CLOCKS_PER_SEC << endl;

                if (curr != nullptr)
                    cout << "\tDepth of last node PBS worked on: " << curr->depth << endl;
                if (!open_list.empty())
                {
                    // Print the state of OPEN
                    double countSameF = 0;
                    double currF = open_list.top()->f_val;
                    double maxDepth = 0;

                    cout << "\tF-value counts in OPEN: " << endl;
                    while (!open_list.empty())
                    {
                        curr = open_list.top();
                        open_list.pop();
                        if (curr->depth > maxDepth)
                            maxDepth = curr->depth;
                        if (curr->f_val > currF + 0.001)
                        {
                            cout << "\t\t#(f=" << currF << ") = " << countSameF << endl;
                            countSameF = 1;
                            currF = curr->f_val;
                        }
                        else
                            countSameF++;
                    }
                    cout << "\t\t#(f=" << currF << ") = " << countSameF << endl;
                    cout << "\tMax depth in OPEN: " << maxDepth << endl;
                }
                else
                {
                    cout << "\tOpen List Empty!!!" << endl;
                }
            }
            solution_found = false;
            break;
        }
        t1 = clock();
        curr = open_list.top();
        open_list.pop();
        runtime_listoperation += clock() - t1;
        // takes the paths_found_initially and UPDATE all constrained paths 
        // found for agents from curr to dummy_start (and lower-bounds)
        t1 = clock();
        updatePaths(curr);
        runtime_updatepaths += clock() - t1;

        t1 = clock();
        findConflicts(*curr);  // Find one conflict for expansion
        runtime_conflictdetection += clock() - t1;
        if (screen > DEBUG_LOG_EXPANSION) printConflicts(*curr);

        if (curr->conflict == nullptr) // Fail to find a conflict => no conflicts
        {   // found a solution (and finish the while loop)
            runtime = (clock() - start) + pre_runtime;
            solution_found = true;
            solution_cost = curr->g_val;

            size_t max_size = 0;
            for (int tmp_a = 0; tmp_a < num_of_agents; tmp_a++)
            {
                set<int> tmp_ma = findMetaAgent(*curr, tmp_a);
                if (tmp_ma.size() > max_size)
                    max_size = tmp_ma.size();
            }
            max_ma_size = max_size;

            cout << "Succeed:) runtime=" << runtime / CLOCKS_PER_SEC << 
                ", cost=" << solution_cost << 
                ", solution delta=" << solution_cost - dummy_start->g_val <<
                ", #pathfinding=" << num_single_pathfinding <<
                ", HL exp(gen)=" << HL_num_expanded << "(" << HL_num_generated << ")" <<
                ", LL exp(gen)=" << LL_num_expanded << "(" << LL_num_generated << ")" <<
                ", max_size=" << max_ma_size << endl;

            // Check the paths are valid
            for (int a1 = 0; a1 < num_of_agents; a1++)
            {
                for (int a2=a1+1; a2 < num_of_agents; a2++)
                {
                    if (isCollide(a1, a2))
                    {
                        cout << "=======================" << endl;
                        cout << "Solution is invalid!!!" << endl;
                        cout << "=======================" << endl;
                        solution_cost = -2;
                        solution_found = false;
                        return solution_found;
                    }
                }
            }

            // Get branch evaluation
            if (screen > DEBUG_LOG_BRANCH_ANALYSIS)
            {
                getBranchEval(curr);
                saveEval();
            }

            break;
        }

        HL_num_expanded++;
        curr->time_expanded = HL_num_expanded;

        int conf_a1 = get<0>(*curr->conflict);
        int conf_a2 = get<1>(*curr->conflict);
        vector<vector<PathEntry>*> copy_paths(paths);

        // if a1->a2 (a2->a1) then do not generate n1 with a2->a1 (n2 with a1->a2)
        bool gen_n1 = !curr->priorities[conf_a1][conf_a2];
        bool gen_n2 = !curr->priorities[conf_a2][conf_a1];

        if (gen_n1)  // Generate node with a2->a1 (a2 has higher priotiry than a1) TODO: lazy gen
        {
            GICBSNode* n1 = new GICBSNode();
            n1->agent_id = conf_a1;
            if (conf_select_mode == 5 || conf_select_mode == 6)
            {
                n1->ag_open_list = curr->ag_open_list;
                if (!n1->isInOpen(conf_a2)) n1->ag_open_list.push_back(conf_a2);
                if (!n1->isInOpen(conf_a1)) n1->ag_open_list.push_back(conf_a1);
            }

            n1->priorities = vector<vector<bool>>(curr->priorities);
            list<int> high_pri = list<int>({conf_a2});
            assert(!n1->priorities[conf_a2][conf_a2]);
            for (int i = 0; i < num_of_agents; i++)
                if (n1->priorities[i][conf_a2]) high_pri.push_back(i);

            list<int> low_pri = list<int>({conf_a1});
            assert(!n1->priorities[conf_a1][conf_a1]);
            for (int i = 0; i < num_of_agents; i++)
                if (n1->priorities[conf_a1][i]) low_pri.push_back(i);

            for (const auto& h_ag : high_pri)
                for (const auto& l_ag : low_pri)
                    n1->priorities[h_ag][l_ag] = true;
            assert(n1->priorities[conf_a2][conf_a1]);

            gen_n1 = generateChild(n1, curr);

            if (gen_n1)
            {
                HL_num_generated++;
                n1->time_generated = HL_num_generated;
                n1->open_handle = open_list.push(n1);  // update handles
                allNodes_table.emplace_back(n1);
                if (screen > DEBUG_LOG_EXPANSION)
                {
                    isPathsValid(n1);  // Check if priority-connected agents are collision-free
                    cout << " generate node " << right << setw(3) << n1->time_generated;
                    cout << " Success!" << endl;
                }
            }
            else
            {
                n1->time_generated = HL_num_generated + 1;
                if (screen > DEBUG_LOG_EXPANSION)
                {
                    cout << " generate node " << right << setw(3) << n1->time_generated;
                    cout << " Failed!" << endl;
                }
                delete (n1);
                n1 = nullptr;
            }
        }
        paths = copy_paths;
        if (gen_n2)  // Generate node with a1->a2 (a1 has higher priotiry than a2)
        {
            GICBSNode* n2 = new GICBSNode();
            n2->agent_id = conf_a2;

            if (conf_select_mode == 5 || conf_select_mode == 6)
            {
                n2->ag_open_list = curr->ag_open_list;
                if (!n2->isInOpen(conf_a1)) n2->ag_open_list.push_back(conf_a1);
                if (!n2->isInOpen(conf_a2)) n2->ag_open_list.push_back(conf_a2);
            }

            n2->priorities = vector<vector<bool>>(curr->priorities);
            list<int> high_pri = list<int>({conf_a1});
            assert(!n2->priorities[conf_a1][conf_a1]);
            for (int i = 0; i < num_of_agents; i++)
                if (n2->priorities[i][conf_a1]) high_pri.push_back(i);

            list<int> low_pri = list<int>({conf_a2});
            assert(!n2->priorities[conf_a2][conf_a2]);
            for (int i = 0; i < num_of_agents; i++)
                if (n2->priorities[conf_a2][i]) low_pri.push_back(i);

            for (const auto& h_ag : high_pri)
                for (const auto& l_ag : low_pri)
                    n2->priorities[h_ag][l_ag] = true;
            assert(n2->priorities[conf_a1][conf_a2]);

            gen_n2 = generateChild(n2, curr);
            if (gen_n2)
            {
                HL_num_generated++;
                n2->time_generated = HL_num_generated;
                if (conf_select_mode == 7 || conf_select_mode == 8) 
                    n2->depth ++;  // n2 gets expanded earlier than n1
                n2->open_handle = open_list.push(n2);  // update handles
                allNodes_table.emplace_back(n2);
                if (screen > DEBUG_LOG_EXPANSION)
                {
                    isPathsValid(n2);  // Check if priority-connected agents are collision-free
                    cout << " generate node " << right << setw(3) << n2->time_generated;
                    cout << " Success!" << endl;
                }
            }
            else
            {
                n2->time_generated = HL_num_generated + 1;
                if (screen > DEBUG_LOG_EXPANSION)
                {
                    cout << " generate node " << right << setw(3) << n2->time_generated;
                    cout << " Failed!" << endl;
                }
                delete (n2);
                n2 = nullptr;
            }
        }

        // Analyze the number of generated nodes
        if (gen_n1 && gen_n2) num_2child++;
        else if (!gen_n1 && !gen_n2) num_0child++;
        else num_1child++;

        curr->clear();
        t1 = clock();
        if (open_list.empty())
        {
            solution_found = false;
            break;
        }
        runtime_listoperation += clock() - t1;
    }  // end of while loop

    if (open_list.empty() && solution_cost < 0 && !(runtime > TIME_LIMIT))
    {
        solution_cost = -2;
        cout << "No solutions  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val <<
            " ; HL exp(gen)=" << HL_num_expanded << "(" << HL_num_generated << ") ; " << 
            " ; LL exp(gen)=" << LL_num_expanded << "(" << LL_num_generated << ") ; " << 
            "Runtime=" << runtime / CLOCKS_PER_SEC << " ; " <<
            "|Open|=" << open_list.size() << endl;
        solution_found = false;
    }
    return solution_found;
}


GICBSSearch::GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr,
                         bool fixed_prior, int scr, int mode) : 
                         fixed_prior(fixed_prior), focal_w(f_w), screen(scr), conf_select_mode(mode)
{
    clock_t start_t = clock();

    if (screen > DEBUG_LOG_BRANCH_ANALYSIS)
    {
        br_max_ma_size = make_shared<vector<size_t>>();
        br_node_soc = make_shared<vector<int>>();
        br_node_idx = make_shared<vector<uint64_t>>();
    }

    HL_num_expanded = 0;
    HL_num_generated = 0;
    LL_num_expanded = 0;
    LL_num_generated = 0;
    this->num_col = ml.cols;
    this->al = al;
    num_of_agents = al.num_of_agents;
    map_size = ml.rows * ml.cols;
    solution_found = false;
    solution_cost = -1;
    search_engines = vector<SingleAgentICBS*>(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
        int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
        ComputeHeuristic ch(init_loc, goal_loc, ml.get_map(), ml.rows, ml.cols, ml.moves_offset, 1.0,
                            &egr);
        search_engines[i] = new SingleAgentICBS(i, init_loc, goal_loc, ml.get_map(), ml.rows * ml.cols,
                                                ml.moves_offset, ml.cols);
        ch.getHVals(search_engines[i]->my_heuristic);
        all_agents.insert(i);
    }

    dummy_start = new GICBSNode();
    dummy_start->agent_id = -1;

    // initialize paths_found_initially
    paths.resize(num_of_agents, NULL);
    paths_found_initially.resize(num_of_agents);

    if (fixed_prior)
    {
        solver_name = "PP";
        int iteration = 0;
        while (true)
        {
            runtime = (clock() - start_t);
            if (runtime > TIME_LIMIT)
            {
                cout << "NO SOLUTION EXISTS AFTER " << iteration << " ITERATIONS";
                solution_cost = -2;
                break;
            }
            iteration++;
            bool found = true;
            dummy_start->makespan = 0;
            paths.clear();
            paths_found_initially.clear();
            paths.resize(num_of_agents, NULL);
            paths_found_initially.resize(num_of_agents);
            dummy_start->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));

            vector<int> ordering(num_of_agents);
            iota(ordering.begin(), ordering.end(), 0);
            random_shuffle(ordering.begin(), ordering.end());

            // Go over agents in decreasing order of priority
            for (int i = 0; i < num_of_agents; i++)
            {
                int a = ordering[i];
                if (!search_engines[a]->findPath(paths_found_initially[a], f_w, dummy_start->priorities, paths,
                                                 dummy_start->makespan + 1, 0))
                {
                    iteration++;
                    found = false;
                    agent_itself_failed = iteration;
                    break;
                }
                for (int j = i + 1; j < num_of_agents; j++)
                {
                    int b = ordering[j];
                    dummy_start->priorities[a][b] = true;
                }
                paths[a] = &paths_found_initially[a];
                dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[a].size() - 1);
                LL_num_expanded += search_engines[a]->num_expanded;
                LL_num_generated += search_engines[a]->num_generated;
            }
            if (found)
            {
                cout << iteration << " iterations" << endl;
                break;
            }
        }
        
        // Find the max size of the priority-connected agent
        max_ma_size = 0;
        for (size_t i = 0; i < num_of_agents; i++)
        {
            size_t tmp_size = findMetaAgent(*dummy_start, i).size();
        }
    }
    else
    {
        solver_name = "PBS";
        dummy_start->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
        for (int i = 0; i < num_of_agents; i++)
        {
            if (!search_engines[i]->findPath(paths_found_initially[i], f_w, dummy_start->priorities, paths,
                                             dummy_start->makespan + 1, 0))
            {
                cout << "NO SOLUTION EXISTS";
                solution_cost = -2;
                break;
            }
            paths[i] = &paths_found_initially[i];
            dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
            LL_num_expanded += search_engines[i]->num_expanded;
            LL_num_generated += search_engines[i]->num_generated;
        }
    }

    if (solution_cost != -2)
    {
        dummy_start->g_val = 0;
        for (int i = 0; i < num_of_agents; i++)
            dummy_start->g_val += paths[i]->size() - 1;
        dummy_start->h_val = 0;
        dummy_start->f_val = dummy_start->g_val;
        dummy_start->depth = 0;
        dummy_start->open_handle = open_list.push(dummy_start);
        HL_num_generated++;
        dummy_start->time_generated = HL_num_generated;
        allNodes_table.emplace_back(dummy_start);
        min_f_val = dummy_start->f_val;
    }

    pre_runtime = clock() - start_t;
}

inline void GICBSSearch::releaseOpenListNodes()
{
    while (!open_list.empty())
    {
        GICBSNode* curr = open_list.top();
        open_list.pop();
        delete curr;
    }
}

GICBSSearch::~GICBSSearch()
{
    for (size_t i = 0; i < search_engines.size(); i++)
        delete (search_engines[i]);
}

void GICBSSearch::getBranchEval(GICBSNode* n)
{
    uint node_cnt = 0;
    while (n != nullptr)
    {
        size_t br_max_size=0;
        for (int ag=0; ag<num_of_agents; ag++)
        {
            size_t tmp_size = findMetaAgent(*n, ag).size();
            if (tmp_size > br_max_size)
                br_max_size = tmp_size;
        }
        br_max_ma_size->push_back(br_max_size);
        br_node_idx->push_back(n->time_generated);
        br_node_soc->push_back(n->g_val);

        n = n->parent;
        node_cnt ++;
    }

    // Reverse
    std::reverse(br_max_ma_size->begin(), br_max_ma_size->end());
    std::reverse(br_node_idx->begin(), br_node_idx->end());
    std::reverse(br_node_soc->begin(), br_node_soc->end());
    return;
}

void GICBSSearch::saveEval(void)
{
    ofstream stats;
	stats.open("iteration_data.txt", ios::out);
	if (!stats.is_open())
	{
		cout << "Failed to open file." << endl;
        return;
	}
    stats << "br_node_idx,";
    copy(br_node_idx->begin(), br_node_idx->end(), ostream_iterator<double>(stats, ","));
    stats << endl;
    stats << "br_node_soc,";
    copy(br_node_soc->begin(), br_node_soc->end(), ostream_iterator<int>(stats, ","));
    stats << endl;
    stats << "br_max_ma_size,";
    copy(br_max_ma_size->begin(), br_max_ma_size->end(), ostream_iterator<double>(stats, ","));
    stats << endl;
}