#include <random>  // For random_device
#include <algorithm>  // For shuffle

#include "GICBSSearch.h"
//#define ROOT
//#define DEBUG
//#define STAT


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

bool GICBSSearch::findAgentsConflicts(GICBSNode& curr, int a1, int a2, uint64_t num, size_t start_t)
{
    // #ifdef DEBUG
    // if (a1 == 0 && a2 == 38)
    //     cout << endl;
    // shared_ptr<Conflict> __conf__ = findEarliestConflict(curr, a1, a2, start_t);
    // endif

    pair<int, int> conf_ags = make_pair(min(a1,a2), max(a1,a2));
    if (curr.conflicts.count(conf_ags) && curr.conflicts[conf_ags].back() == nullptr)
        return true;  // All the conflicts are found

    shared_ptr<Conflict> conf;
    uint64_t count = 0;
    size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
    for (size_t timestep = start_t; timestep < min_path_length; timestep++)
    {
        int loc1 = paths[a1]->at(timestep).location;
        int loc2 = paths[a2]->at(timestep).location;
        if (loc1 == loc2)
        {
            // This is a vertex conflict
            conf = make_shared<Conflict>(a1, a2, loc1, -1, timestep);
            if (!curr.conflicts.count(conf_ags))
                curr.conflicts[conf_ags] = list<shared_ptr<Conflict>>({conf});
            else
                curr.conflicts[conf_ags].push_back(conf);
            count ++;
            if (count == num)
                return true;
        }
        else if (timestep < min_path_length - 1
                    && loc1 == paths[a2]->at(timestep + 1).location
                    && loc2 == paths[a1]->at(timestep + 1).location)
        {
            // This is an edge conflict
            conf = make_shared<Conflict>(a1, a2, loc1, loc2, timestep + 1);
            if (!curr.conflicts.count(conf_ags))
                curr.conflicts[conf_ags] = list<shared_ptr<Conflict>>({conf});
            else
                curr.conflicts[conf_ags].push_back(conf);
            count ++;
            if (count == num)
                return true;
        }
    }
    if (paths[a1]->size() != paths[a2]->size())
    {
        int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
        int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
        int loc1 = paths[a1_]->back().location;
        for (size_t timestep = max(start_t, min_path_length); timestep < paths[a2_]->size(); timestep++)
        {
            int loc2 = paths[a2_]->at(timestep).location;
            if (loc1 == loc2)
            {
                conf = make_shared<Conflict>(a1_, a2_, loc1, -1, timestep);
                if (!curr.conflicts.count(conf_ags))
                    curr.conflicts[conf_ags] = list<shared_ptr<Conflict>>({conf});
                else
                    curr.conflicts[conf_ags].push_back(conf);
                count ++;
                if (count == num)
                    return true;
            }
        }
    }
    // Add a pseudo conflict at the end if all the conflicts are founded
    // Check if conf_ags is in the key
    // Check if there are conflicts
    if (curr.conflicts.count(conf_ags) && curr.conflicts[conf_ags].size() > 0)
    {
        curr.conflicts[conf_ags].push_back(nullptr);
        return true;
    }
    assert(!curr.conflicts.count(conf_ags));
    return false;
}

shared_ptr<Conflict> GICBSSearch::findEarliestConflict(GICBSNode& curr, int a1, int a2, size_t start_t)
{
    assert(a1 != a2);
    int _a1_ = min(a1, a2);
    int _a2_ = max(a1, a2);
    pair<int, int> conf_ags = make_pair(_a1_, _a2_);
    if (curr.conflicts.count(conf_ags) && curr.conflicts[conf_ags].back() == nullptr)
        return curr.conflicts[conf_ags].front();  // All the conflicts are found

    size_t min_path_length = paths[_a1_]->size() < paths[_a2_]->size() ? paths[_a1_]->size() : paths[_a2_]->size();
    for (size_t timestep = start_t; timestep < min_path_length; timestep++)
    {
        int loc1 = paths[_a1_]->at(timestep).location;
        int loc2 = paths[_a2_]->at(timestep).location;
        if (loc1 == loc2)
        {
            // This is a vertex conflict
            return make_shared<Conflict>(_a1_, _a2_, loc1, -1, timestep);
        }
        else if (timestep < min_path_length - 1
                    && loc1 == paths[_a2_]->at(timestep + 1).location
                    && loc2 == paths[_a1_]->at(timestep + 1).location)
        {
            // This is an edge conflict
            return make_shared<Conflict>(_a1_, _a2_, loc1, loc2, timestep + 1);
        }
    }
    if (paths[_a1_]->size() != paths[_a2_]->size())
    {
        int a1_ = paths[_a1_]->size() < paths[_a2_]->size() ? _a1_ : _a2_;
        int a2_ = paths[_a1_]->size() < paths[_a2_]->size() ? _a2_ : _a1_;
        int loc1 = paths[a1_]->back().location;
        for (size_t timestep = max(start_t, min_path_length); timestep < paths[a2_]->size(); timestep++)
        {
            int loc2 = paths[a2_]->at(timestep).location;
            if (loc1 == loc2)
            {
                return make_shared<Conflict>(a1_, a2_, loc1, -1, timestep);
            }
        }
    }
    return nullptr;
}

bool GICBSSearch::isCollide(const GICBSNode& curr, int a1, int a2)
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
            cout << a1 << " and " << a2 << " collide at (";
            cout << loc1 / num_col << "," << loc1 % num_col << ") at time " << timestep << endl;
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
            cout << a1 << " and " << a2 << " collide at (" << loc1/num_col << "," << loc1%num_col;
            cout << ") and (" << loc2/num_col << "," << loc2%num_col <<") at time " << timestep << endl;
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
                cout << a1_ << " and " << a2_ << " collide at (" << loc1 / num_col << "," << loc1 % num_col << ") at time " << timestep << endl;
                cout << "*********************************" << endl;
                return true;
            }
        }
    }
    return false;
}

void GICBSSearch::copyConflicts(const AgentsConflicts& conflicts, AgentsConflicts& copy,
    const set<int>& excluded_agents)
{
    for (const auto& ag_conf : conflicts)
    {
        bool found = false;
        for (const int& a : excluded_agents)
        {
            if (ag_conf.first.first == a || ag_conf.first.second == a)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            copy[ag_conf.first] = ag_conf.second;
        }
    }
    return;
}

void GICBSSearch::findConflictswithMA(GICBSNode& curr)
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
        // cout << "------------------------------------" << endl;
        // cout << "At node->time_generated " << curr.time_generated << ", ";
        // cout << "Replan agent " << curr.agent_id << endl;
        // cout << "Find internal conflict: ";
        assert(curr.agent_id != -1);
        set<int> curr_ma = findMetaAgent(curr, curr.agent_id);
        for (auto it1=curr_ma.begin(); it1!=curr_ma.end(); it1++)
        {
            for (auto it2=next(it1); it2!=curr_ma.end(); it2++)
            {
                shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, *it1, *it2);
                if (tmp_conf != nullptr)
                {
                    // cout << "<" << get<0>(tmp_conf) << ", " << get<1>(tmp_conf) 
                    //     << ", " << get<2>(tmp_conf) << ", " << get<3>(tmp_conf) 
                    //     << ", " << get<4>(tmp_conf) << ">" << endl;
                    curr.conflict = tmp_conf;
                    return;
                }
                // if (curr.trans_priorities[*it1][*it2] || curr.trans_priorities[*it2][*it1])
                //     continue;
                // else
                // {
                //     shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, *it1, *it2);
                //     if (tmp_conf != nullptr)
                //     {
                //         curr.conflict = tmp_conf;
                //         cout << "<" << get<0>(*curr.conflict) << ", " << get<1>(*curr.conflict) 
                //             << ", " << get<2>(*curr.conflict) << ", " << get<3>(*curr.conflict) 
                //             << ", " << get<4>(*curr.conflict) << ">" << endl;
                //         return;
                //     }
                // }
            }
        }
        // cout << "None" << endl;
        // // Debug
        // for (auto it1=curr_ma.begin(); it1!=curr_ma.end(); it1++)
        // {
        //     for (auto it2=next(it1); it2!=curr_ma.end(); it2++)
        //     {
        //         if (isCollide(curr, *it1, *it2))
        //         {
        //             cout << "Missing internal conflict between agents " << *it1 << " and " << *it2 << endl;
        //             cout << "transPriorities[" << *it1 << "][" << *it2 << "]" << curr.trans_priorities[*it1][*it2] << endl;
        //             cout << "transPriorities[" << *it2 << "][" << *it1 << "]" << curr.trans_priorities[*it2][*it1] << endl;
        //         }
        //         assert(!isCollide(curr, *it1, *it2));
        //     }
        // }
        // // end debug

        // cout << "Find min size of agent";
        size_t min_total_size = SIZE_MAX;
        list<shared_ptr<Conflict>> min_size_conf;
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            set<int> ma1 = findMetaAgent(curr, a1, min_total_size+1);
            // cout << "--- agent a1: " << a1 << endl;
            // cout << "ma1: ";
            // for (const int& da1 : ma1)
            // {
            //     cout << da1 << ", ";
            // }
            // cout << endl;

            if (ma1.size() > min_total_size)
            {
                assert(ma1.size() == min_total_size+1);
                // cout << "\tma1 size > min_total_size" << min_total_size << endl;
                continue;
            }

            for (int a2 = a1+1; a2 < num_of_agents; a2++)
            {
                if (find(ma1.begin(), ma1.end(), a2) != ma1.end())  // a2 is in ma1
                {
                    // cout << "--- agent a2: " << a2 << " is in ma1 ---" << endl;
                    continue;
                }
                else
                {
                    set<int> ma2 = findMetaAgent(curr, a2, min_total_size+1);
                    // cout << "ma2: ";
                    // for (const int& da1 : ma2)
                    // {
                    //     cout << da1 << ", ";
                    // }
                    // cout << endl;

                    if (ma2.size() > min_total_size)
                    {
                        // cout << "\tma2 size > min_total_size" << min_total_size << endl;
                        assert(ma2.size() == min_total_size+1);
                        continue;
                    }

                    if (ma1.size() + ma2.size() > min_total_size)
                    {
                        // cout << "\tma1+ma2 size > min_total_size" << min_total_size << endl;
                        continue;
                    }
                    else
                    {
                        shared_ptr<Conflict> tmp_conf = findEarliestConflict(curr, a1, a2);
                        if (tmp_conf == nullptr)
                        {
                            continue;
                        }
                        else if (ma1.size() + ma2.size() < min_total_size)
                        {
                            min_total_size = ma1.size() + ma2.size();
                            min_size_conf.clear();
                        }
                        min_size_conf.emplace_back(tmp_conf);
                    }
                }
            }
        }

        if (!min_size_conf.empty())
        {
            // for (const auto& conf : min_size_conf)
            // {
            //     cout << "<" << get<0>(*conf) << ", " << get<1>(*conf) << ", " << get<2>(*conf) 
            //         << ", " << get<3>(*conf) << ", " << get<4>(*conf) << "> | ";
            // }
            // cout << endl;
            list<shared_ptr<Conflict>>::iterator cit = min_size_conf.begin();
            int random = rand() % min_size_conf.size();
            std::advance(cit, random);
            curr.conflict = *cit;
        }
        else
        {
            curr.conflict = nullptr;  // It is a goal node (conflict-free)
            // Debug
            for (int da1 = 0; da1 < num_of_agents; da1++)
            {
                for (int da2 = da1+1; da2 < num_of_agents; da2++)
                {
                    assert(!isCollide(curr, da1, da2));
                }
            }
            // end debug
        }
    }

    return;
}

void GICBSSearch::selectConflict(GICBSNode& curr)
{   
    size_t min_size = SIZE_MAX;
    unordered_map<pair<int,int>, set<int>, pair_hash> min_size_ma;
    for (const auto& conf : curr.conflicts)
    {
        set<int> ma = findMetaAgent(curr, conf.first.first);  // meta-agent of a1
        if (find(ma.begin(), ma.end(), conf.first.second) != ma.end()) // Internal conflict
        {
            curr.conflict = conf.second.front();
            break;
        }
        
        else  // External conflict
        {
            set<int> ma2 = findMetaAgent(curr, conf.first.second);
            ma.merge(ma2);  // merge the list of meta-agents of a2
            if (ma.size() < min_size)  // Reset the dict if a smaller meta-agent is found
            {
                min_size_ma.clear();
                min_size_ma[conf.first] = ma;
                min_size = ma.size();
            }
            else if (ma.size() == min_size)
            {
                min_size_ma[conf.first] = ma;
            }
        }
        
        // // Debug: compare if there is common term in ma and ma2
        // cout << "[" << conf.first.first << "]: {";
        // for(const int& a : ma)
        //     cout << a << ", ";
        // cout << "} | [" << conf.first.second << "]: {";
        // for(const int& a : ma2)
        //     cout << a << ", ";
        // cout << "}" << endl;

        // bool is_same = false;
        // for (const int& tmp_a1 : ma)
        // {
        //     if (find(ma2.begin(), ma2.end(), tmp_a1) != ma2.end())
        //     {
        //         is_same = true;
        //         break;
        //     }
        // }
        // assert(!is_same);
    }

    // Tie-breaking with the maximum number of conflicts with agents outside the meta-agent
    size_t max_num_conf = 0;
    pair<int,int> target_ma;
    for (const auto& ma : min_size_ma)
    {
        size_t num_conf = 0;
        // Find the external conflicts for (a1, a2)
        for (const auto& conf : curr.conflicts)
        {
            if (conf.first == ma.first)
                continue;

            int a1 = conf.first.first;
            int a2 = conf.first.second;
            bool is_a1_ma = find(ma.second.begin(), ma.second.end(), a1) != ma.second.end();
            bool is_a2_ma = find(ma.second.begin(), ma.second.end(), a2) != ma.second.end();
            if ((is_a1_ma && !is_a2_ma) || (!is_a1_ma && is_a2_ma))
            {
                // Find all the external conflicts
                auto rit = conf.second.rbegin();
                if (*rit != nullptr)  // Not all the conflicts are found
                {
                    assert(conf.second.size() == 1);
                    int last_timestep = get<4>(**rit);
                    findAgentsConflicts(curr, a1, a2, INT64_MAX, last_timestep+1);

                    for (auto it1=curr.conflicts[make_pair(a1,a2)].begin(); it1!=curr.conflicts[make_pair(a1,a2)].end(); it1++)
                    {
                        for (auto it2=next(it1,1); it2!=curr.conflicts[make_pair(a1,a2)].end(); it2++)
                        {
                            if (it1 == it2)
                            {
                                // printConflicts(curr);
                                assert(false);
                                exit(-1);
                            }
                        }
                    }
                    // printConflicts(curr);
                }
                assert(conf.second.back() == nullptr);
                num_conf += conf.second.size() - 1;  // Ignore the nullptr 
            }
        }

        if (num_conf > max_num_conf)  // Assume there is only one target_ma
        {
            target_ma = ma.first;
            max_num_conf = num_conf;
        }
        else if (num_conf == max_num_conf)
        {
            target_ma = ma.first;
        }
    }
    curr.conflict = curr.conflicts[target_ma].front();
    assert(curr.conflict != nullptr);
    // cout << "select conflict: " << "<" << get<0>(*curr.conflict) << ", " << get<1>(*curr.conflict);
    // cout << ", " << get<2>(*curr.conflict) << ", " << get<3>(*curr.conflict) << ", ";
    // cout << get<4>(*curr.conflict) << ">" << endl;
    // cout << "Select conflict DONE!!" << endl;
    return;
}

set<int> GICBSSearch::findMetaAgent(const GICBSNode& curr, int ag, size_t size_th)
{
    // Use DFS to find the connected component
    // TODO: memorize the meta-agent in the curr node
    set<int> ma;
    vector<int> is_visited(num_of_agents, false);
    list<int> open_list = list<int>({ag});
    is_visited[ag] = true;
    while(!open_list.empty())
    {
        int cur_ag = open_list.front();
        ma.insert(cur_ag);
        if (ma.size() == size_th)
            break;
        open_list.pop_front();
        for (int a2 = 0; a2 < num_of_agents; a2++)
        {
            if(((curr.trans_priorities[cur_ag][a2] && !curr.trans_priorities[a2][cur_ag]) || 
                (curr.trans_priorities[a2][cur_ag] && !curr.trans_priorities[cur_ag][a2])) && 
                !is_visited[a2])
            {
                assert(find(ma.begin(), ma.end(), a2) == ma.end());
                is_visited[a2] = true;
                open_list.push_front(a2);
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
                return;
            }
        }
    }
    return;
}

void GICBSSearch::findConflicts(GICBSNode& curr)
{
    switch (conf_select_mode)
    {
    case 0:
        findConflictsOri(curr);
        break;
    case 1:
        findConflictswithMA(curr);
        break;
    default:
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
                else if (timestep < min_path_length - 1
                         && loc1 == paths[a2]->at(timestep + 1).location
                         && loc2 == paths[a1]->at(timestep + 1).location)
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


// May absolutely find new paths for multiple agents!
bool GICBSSearch::findPathForSingleAgent(GICBSNode* node, int ag, double lowerbound)
{
    // Debug
    // cout << "\n\n########## Replan agent " << ag << " ##########" << endl;
    // cout << "node: " << node->time_expanded << ", " << node->time_generated << endl;
    // printAgentPath(ag);
    // cout << "##########################" << endl;

    bool foundSol = true;
    vector<vector<PathEntry>> new_paths(num_of_agents, vector<PathEntry>());

    // Prepare a topological sort of the agents from ag and below it.
    vector<bool> visited(num_of_agents, true);
    for (int i = 0; i < num_of_agents; i++)
    {
        if (node->trans_priorities[ag][i])
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

    // Find a new path for ag and for every lower-priority agent that has a collision with a new path.
    // Initially, we assume all paths are consistent with the current priority ordering, so consistent[a][b] = true
    // for all a and b.
    // If a has higher priority than b (a≺b), then consistent[a][b] = true.
    // After we find a new path for agent b, we set consistent[a][b] = true for all agents a with higher priority,
    // and consistent[b][c] = false for every agent c with lower priority, meaning we can no longer assume without
    // checking that c's path has no collisions with b's.
    vector<vector<bool>> consistent(num_of_agents, vector<bool>(num_of_agents, true));
    for (auto iter = topSort.begin(); iter != topSort.end(); iter++)
    {
        int curr_agent = *iter;
        bool isColliding = false;
        for (int a2 = 0; a2 < num_of_agents; a2++)
        {
            // if (!consistent[a2][curr_agent])
            // {
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
                    int a1_ = paths[curr_agent]->size() < paths[a2]->size() ? ag : a2;
                    int a2_ = paths[curr_agent]->size() < paths[a2]->size() ? a2 : ag;
                    int loc1 = paths[a1_]->back().location;
                    for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
                    {
                        int loc2 = paths[a2_]->at(timestep).location;
                        if (loc1 == loc2)
                        {
                            break;
                        }
                    }
                }
            // }
            if (isColliding)
            {
                break;
            }
        }
        if (!isColliding && curr_agent != ag)  // Current agent isn't colliding with any higher-priority agent in the
                                               // topological sort and isn't the original agent we need to find a path
                                               // for - no need to find a new path for it
        {
            continue;
        }
        size_t max_plan_len = node->makespan + 1; //getPathsMaxLength();
        //bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
        //bool* res_table_low_prio = new bool[map_size * max_plan_len]();  // initialized to false
        //updateReservationTable(res_table, res_table_low_prio, curr_agent, *node);
        // find a path w.r.t cons_vec (and prioretize by res_table).
        //pair<int, vector<PathEntry>> newPath;
        //vector<PathEntry> newPath;
        //newPath.first = curr_agent;
        //foundSol = search_engines[curr_agent]->findPath(newPath.second, focal_w, node->trans_priorities, paths, max_plan_len, lowerbound);
        num_single_pathfinding++;

        // auto t1 = std::clock();
        //findConflicts(*node);
        foundSol = search_engines[curr_agent]->findPath(new_paths[curr_agent], focal_w, node->trans_priorities, paths,
                                                        max_plan_len, lowerbound);
        // cout << "ll solver: " << std::clock() - t1 << endl;
        LL_num_expanded += search_engines[curr_agent]->num_expanded;
        LL_num_generated += search_engines[curr_agent]->num_generated;
        //delete (cons_vec);
        //delete[] res_table;
        if (foundSol)
        {
            //node->new_paths.push_back(newPath);
            //new_paths[curr_agent] = newPath;
            node->g_val = node->g_val - paths[curr_agent]->size() + new_paths[curr_agent].size();
            //paths[curr_agent] = &node->new_paths.back().second;
            paths[curr_agent] = &new_paths[curr_agent]; // might be used by the next findPath() call
            //node->paths[ag] = search_engines[ag]->getPath();
            node->makespan = max(node->makespan, new_paths[curr_agent].size() - 1);
            for (int i = 0; i < num_of_agents; i++)
            {
                if (node->trans_priorities[curr_agent][i])
                {
                    consistent[curr_agent][i] = false;
                }
                if (node->trans_priorities[i][curr_agent])
                {
                    consistent[i][curr_agent] = true;
                }
            }
        }
        else
        {
            if (curr_agent == node->agent_id)
                ++agent_itself_failed;
            else
                ++lower_priority_agent_failed;
            return false;
        }
    }
    if (foundSol)
    {
        for (int i = 0; i < num_of_agents; i++)
        {
            if (!new_paths[i].empty())
            {
                node->new_paths.push_back(make_pair(i, new_paths[i]));
                paths[i] = &node->new_paths.back().second; // make sure paths[i] gets the correct pointer
            }
        }
    }
    return true;
}

bool GICBSSearch::generateChild(GICBSNode* node, GICBSNode* curr)
{
    node->parent = curr;
    node->g_val = curr->g_val;
    node->makespan = curr->makespan;
    //node->f_val = curr->f_val - curr->h_val;
    node->depth = curr->depth + 1;
    //node->paths = curr->paths;
    //node->paths.resize(num_of_agents);
    //node->paths.assign(curr->paths.begin(), curr->paths.end());
    //node->single.resize(num_of_agents, NULL);
    std::clock_t t1;

    t1 = std::clock();

    if (get<3>(node->constraint)) //positive constraint
    {
        for (int ag = 0; ag < num_of_agents; ag++)
        {
            if (ag == node->agent_id)
                continue;
            else if (get<1>(node->constraint) < 0 && // vertex constraint
                     getAgentLocation(ag, get<2>(node->constraint)) == get<0>(node->constraint))
            {
                if (!findPathForSingleAgent(node, ag))
                    return false;
                //else
                //	node->paths[ag] = &(get<1>(node->paths_updated.back()));
            }
            else if (get<1>(node->constraint) >= 0 && //edge constraint
                     getAgentLocation(ag, get<2>(node->constraint) - 1) == get<1>(node->constraint) &&
                     getAgentLocation(ag, get<2>(node->constraint)) == get<0>(node->constraint))
            {
                if (!findPathForSingleAgent(node, ag))
                    return false;
                else
                    paths[ag] = &(node->new_paths.back().second);
            }
        }
    }
    else // negative constraint
    {
        // double lowerbound;
        // if (get<2>(*curr->conflict) < 0) // rectangle conflict
        //     lowerbound = (int) paths[node->agent_id]->size() - 1;
        // else if (get<4>(*curr->conflict) >=
        //          (int) paths[node->agent_id]->size()) //conflict happens after agent reaches its goal
        //     lowerbound = get<4>(*curr->conflict) + 1;
        // else if (!paths[node->agent_id]->at(get<4>(*curr->conflict)).single) // not cardinal
        //     lowerbound = (int) paths[node->agent_id]->size() - 1;
        // else if (get<2>(*curr->conflict) >= 0 && get<3>(*curr->conflict) < 0) // Cardinal vertex
        //     lowerbound = (int) paths[node->agent_id]->size();
        // else if (paths[node->agent_id]->at(get<4>(*curr->conflict) - 1).single) // Cardinal edge
        //     lowerbound = (int) paths[node->agent_id]->size();
        // else // Not cardinal edge
        //     lowerbound = (int) paths[node->agent_id]->size() - 1;

        if (!findPathForSingleAgent(node, node->agent_id))
            return false;
        //else
        //	paths[node->agent_id] = &(get<1>(node->paths_updated.back()));
    }

    runtime_lowlevel += std::clock() - t1;

    node->f_val = node->g_val;

    t1 = std::clock();
    runtime_conflictdetection += std::clock() - t1;

    // update handles
    node->open_handle = open_list.push(node);
    HL_num_generated++;
    node->time_generated = HL_num_generated;
    allNodes_table.emplace_back(node);

    return true;
}


void GICBSSearch::printPaths() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
                  paths[i]->size() - 1 << "): ";
        for (size_t t = 0; t < paths[i]->size(); t++)
            std::cout << "(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col
                      << ")->";
        std::cout << std::endl;
    }
}


void GICBSSearch::printAgentPath(int ag) const
{
    std::cout << "Agent " << ag << " (" << paths_found_initially[ag].size() - 1 << " -->" <<
                  paths[ag]->size() - 1 << "): ";
    for (size_t t = 0; t < paths[ag]->size(); t++)
        std::cout << "(" << paths[ag]->at(t).location / num_col << "," << 
            paths[ag]->at(t).location % num_col << ")->";
    std::cout << std::endl;
}


void GICBSSearch::printConstraints(const GICBSNode* n) const
{
    const GICBSNode* curr = n;
    while (curr != dummy_start)
    {
        std::cout << "<" << curr->agent_id
                  << ", " << get<0>(curr->constraint)
                  << ", " << get<1>(curr->constraint)
                  << ", " << get<2>(curr->constraint);
        if (get<3>(curr->constraint))
            std::cout << ", positive>" << std::endl;
        else
            std::cout << ", negative>" << std::endl;
        curr = curr->parent;
    }
}

void GICBSSearch::printConflicts(GICBSNode& curr)
{
    for (const auto& conf : curr.conflicts)
    {
        cout << "[" << conf.first.first << ", " << conf.first.second << "]: ";
        for (const auto c : conf.second)
        {
            if (c == nullptr)
                cout << " NULL";
            else
                cout << "<" << get<2>(*c) << "," << get<3>(*c) << "," << get<4>(*c) << ">" << ",";
        }
        cout << endl;
    }
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
    node_stat.clear();
    cout << "       GICBS: ";
    if (solution_cost == -2)
    {
        runtime = pre_runtime;
        return false;
    }
    // set timer
    std::clock_t start;
    start = std::clock();
    std::clock_t t1;
    runtime_computeh = 0;
    runtime_lowlevel = 0;
    runtime_listoperation = 0;
    runtime_conflictdetection = 0;
    runtime_updatepaths = 0;
    runtime_updatecons = 0;
    // start is already in the open_list
    GICBSNode* curr = nullptr;
    while (!open_list.empty() && !solution_found)
    {
        // break after 1 minute
        runtime = (std::clock() - start) + pre_runtime; // / (double) CLOCKS_PER_SEC;
        if (runtime > TIME_LIMIT || HL_num_expanded > 1000000)
        {  // timeout after 1 minutes or 1000000 expanded nodes

            size_t max_size = 0;
            for (int tmp_a = 0; tmp_a < num_of_agents; tmp_a++)
            {
                set<int> tmp_ma = findMetaAgent(*curr, tmp_a);
                if (tmp_ma.size() > max_size)
                    max_size = tmp_ma.size();
            }
            max_ma_size = max_size;

            cout << "TIMEOUT; cost=" << solution_cost << "; solution delta=" << min_f_val - dummy_start->g_val <<
                 "; {HL,LL}x{expanded,generated}=" << HL_num_expanded << ", " << HL_num_generated <<
                 ", " << LL_num_expanded << ", " << LL_num_generated <<
                 "; runtime=" << runtime / CLOCKS_PER_SEC << " ; max_ma_size=" << max_ma_size << endl;

            std::cout << "	Runtime summary: lowlevel = " << runtime_lowlevel / CLOCKS_PER_SEC << " ; listoperation = "
                      << runtime_listoperation / CLOCKS_PER_SEC <<
                      " ; conflictdetection = " << runtime_conflictdetection / CLOCKS_PER_SEC << " ; computeh = " << runtime_computeh / CLOCKS_PER_SEC <<
                      " ; updatepaths = " << runtime_updatepaths / CLOCKS_PER_SEC << " ; collectcons = " << runtime_updatecons / CLOCKS_PER_SEC
                      << std::endl;

            if (curr != nullptr)
                std::cout << "Depth of last node PBS worked on: " << curr->depth << std::endl;
            if (!open_list.empty())
            {
                // Print the state of OPEN
                double countSameF = 0;
                double currF = open_list.top()->f_val;
                double maxDepth = 0;

                std::cout << "F-value counts in OPEN: " << std::endl;
                while (!open_list.empty())
                {
                    curr = open_list.top();
                    open_list.pop();
                    if (curr->depth > maxDepth)
                        maxDepth = curr->depth;
                    if (curr->f_val > currF + 0.001)
                    {
                        cout << "				#(f=" << currF << ") = " << countSameF << endl;
                        countSameF = 1;
                        currF = curr->f_val;
                    }
                    else
                        countSameF++;
                }
                cout << "				#(f=" << currF << ") = " << countSameF << endl;
                std::cout << "Max depth in OPEN: " << maxDepth << std::endl;
            }
            else
            {
                std::cout << "Open List Empty!!!" << endl;
            }
            solution_found = false;
            break;
        }
        t1 = std::clock();
        curr = open_list.top();
        open_list.pop();
        // cout << "Expand node id:" << curr->agent_id << ", expand:" << curr->time_expanded
        //     << ", generate:" << curr->time_generated << endl;
        runtime_listoperation += std::clock() - t1;
        // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
        t1 = std::clock();
        updatePaths(curr);
        runtime_updatepaths += std::clock() - t1;

        #ifdef DEBUG
        //printPaths();
        #endif

        t1 = std::clock();
        findConflicts(*curr);  // Find one conflict for each conflicting pair of agents in node curr
        runtime_conflictdetection += std::clock() - t1;

        if (curr->conflict == nullptr) // Fail to find a conflict => no conflicts
        {   // found a solution (and finish the while loop)
            runtime = (std::clock() - start) + pre_runtime; // / (double) CLOCKS_PER_SEC;
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

            cout << "cost=" << solution_cost << "; solution delta=" << solution_cost - dummy_start->g_val <<
                 "; #pathfinding=" << num_single_pathfinding <<
                 "; {HL,LL}x{expanded,generated}=" << HL_num_expanded << ", " << HL_num_generated <<
                 ", " << LL_num_expanded << ", " << LL_num_generated << "; runtime=" << runtime / CLOCKS_PER_SEC <<
                 "; max_size=" << max_ma_size << endl;
//#ifdef DEBUG
//			int numCollidingPairs = countCollidingPairs();
//			if (numCollidingPairs > 0)
//				std::cout << "ERROR!" << std::endl;
//			std::cout << std::endl << "****** Solution: " << std::endl;
//			printPaths();
//#endif
            // Check the paths are valid
            for (int a1 = 0; a1 < num_of_agents; a1++)
            {
                for (int a2=a1+1; a2 < num_of_agents; a2++)
                {
                    assert(!isCollide(*curr, a1, a2));
                }
            }

            // Get branch evaluation
            if (screen > 0)
            {
                getBranchEval(curr);
                saveEval();
            }

            break;
        }

        // Select a conflict based on the size of the meta-agent
        // We assume only merge operation is used to resolve each conflict
        // 1. Check the size of meta-agents after merging
        // 2. Find the minimum size of the meta-agent
        // 3. Tie breaking with the highest number of conflicts
        // selectConflict(*curr);

        //Expand the node
        // cout << "\n\n================ find conflicts ================" << endl;
        // cout << get<0>(*curr->conflict) << "," << get<1>(*curr->conflict) << "," << 
        //     get<2>(*curr->conflict) << "," << get<3>(*curr->conflict) << "," << 
        //     get<4>(*curr->conflict) << ",";
        // cout << "\n============== end find conflicts =================" << endl;

        HL_num_expanded++;
        curr->time_expanded = HL_num_expanded;

        #ifdef DEBUG
        std::cout << std::endl << "****** Expanded #" << curr->time_generated << " with f= " << curr->g_val <<
            "+" << curr->h_val << " (";
        for (int i = 0; i < num_of_agents; i++)
            std::cout << paths[i]->size() - 1 << ", ";
        std::cout << ")" << std::endl;
        std::cout << "Choose conflict <";
        std::cout << "A1=" << get<0>(*curr->conflict) << ",A2=" << get<1>(*curr->conflict)
            << ",loc1=(" << get<2>(*curr->conflict) / num_col << "," << get<2>(*curr->conflict) % num_col
            << "),loc2=(" << get<3>(*curr->conflict) / num_col << "," << get<3>(*curr->conflict) % num_col
            << "),t=" << get<4>(*curr->conflict) << ">" << std::endl;
        #endif

        GICBSNode* n1 = new GICBSNode();
        GICBSNode* n2 = new GICBSNode();

        n1->agent_id = get<0>(*curr->conflict);
        n2->agent_id = get<1>(*curr->conflict);
        if (get<3>(*curr->conflict) < 0) // vertex conflict
        {
            n1->constraint = make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict), false);
            n2->constraint = make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict), false);
        }
        else // edge conflict
        {
            n1->constraint = make_tuple(get<2>(*curr->conflict), get<3>(*curr->conflict), get<4>(*curr->conflict),
                                        false);
            n2->constraint = make_tuple(get<3>(*curr->conflict), get<2>(*curr->conflict), get<4>(*curr->conflict),
                                        false);
        }

        bool Sol1 = false, Sol2 = false;
        vector<vector<PathEntry>*> copy(paths);
        //int lowerbound1 = max(get<4>(curr->conflict) + 1, (int)paths[n1->agent_id]->size() - 1);
        //int lowerbound2 = max(get<4>(curr->conflict) + 1, (int)paths[n2->agent_id]->size() - 1);
        //lowerbound2 = ; // The cost of path should be at least the confliting time + 1
        //if (!curr->cardinalConf.empty() || !curr->rectCardinalConf.empty()) // Resolve a cardinal conflict
        //{
        //
        //}
        //else if (!curr->semiConf.empty() || !curr->rectSemiConf.empty()) // Resolve a semi
        //{
        //	if (Sol1 && Sol2 && abs(n1->g_val + n2->g_val - 2 * curr->g_val) < 0.001)
        //	{
        //		std::cout << "***********ERROR**************" << std::endl;
        //		system("pause");
        //	}
        //}

        bool gen_n1 = true, gen_n2 = true;

        if (curr->trans_priorities[n1->agent_id][n2->agent_id])
        { // a1->a2 do not generate n1
            gen_n1 = false;
        }
        if (curr->trans_priorities[n2->agent_id][n1->agent_id])
        { // a2->a1 do not generate n2
            gen_n2 = false;
        }

        if (gen_n1)  // Generate node with a2->a1 (a2 has higher priotiry than a1)
        {
            n1->priorities = vector<vector<bool>>(curr->priorities);
            n1->trans_priorities = vector<vector<bool>>(curr->trans_priorities);
            n1->priorities[n2->agent_id][n1->agent_id] = true; // a2->a1
            n1->trans_priorities[n2->agent_id][n1->agent_id] = true;
            for (int i = 0; i < num_of_agents; i++)
            { // transitivity
                if (n1->trans_priorities[i][n2->agent_id] && !n1->trans_priorities[i][n1->agent_id])
                {
                    for (int j = 0; j < num_of_agents; j++)
                    {
                        if (n1->trans_priorities[n1->agent_id][j])
                        {
                            n1->trans_priorities[i][j] = true;
                        }
                    }
                }
            }

            Sol1 = generateChild(n1, curr);
            if (!gen_n2)
            {
                n1->depth--;
            }
        }
        paths = copy;
        //updatePaths(curr);
        if (gen_n2)  // Generate node with a1->a2 (a1 has higher priotiry than a2)
        {
            n2->priorities = vector<vector<bool>>(curr->priorities);
            n2->trans_priorities = vector<vector<bool>>(curr->trans_priorities);
            n2->priorities[n1->agent_id][n2->agent_id] = true; // a1->a2
            n2->trans_priorities[n1->agent_id][n2->agent_id] = true;
            for (int i = 0; i < num_of_agents; i++)
            { // transitivity
                if (n2->trans_priorities[i][n1->agent_id] && !n2->trans_priorities[i][n2->agent_id])
                {
                    for (int j = 0; j < num_of_agents; j++)
                    {
                        if (n2->trans_priorities[n2->agent_id][j])
                        {
                            n2->trans_priorities[i][j] = true;
                        }
                    }
                }
            }

            Sol2 = generateChild(n2, curr);
            if (!gen_n2)
            {
                n2->depth--;
            }
        }




        /*if (!Sol1 ){
          std::cout << "Not feasible child for " << n1->agent_id << ", " << n2->agent_id << endl;
        }
        if (!Sol2 ){
          std::cout << "Not feasible child for " << n2->agent_id << ", " << n1->agent_id << endl;
        }

        if (!Sol1 && !Sol2){
          std::cout << "Not able to resolve between" << n1->agent_id << ", " << n2->agent_id << endl;
        }*/


#ifdef DEBUG
        if(Sol1)
        {
            std::cout	<< "Generate #" << n1->time_generated
                            << " with cost " << n1->g_val
                            << " and " << n1->num_of_colliding_pairs << " conflicts " <<  std::endl;
        }
        else
        {
            std::cout << "No feasible solution for left child! " << std::endl;
        }
        if (Sol2)
        {
            std::cout	<< "Generate #" << n2->time_generated
                            << " with cost " << n2->g_val
                            << " and " << n2->num_of_colliding_pairs << " conflicts " << std::endl;
        }
        else
        {
            std::cout << "No feasible solution for right child! " << std::endl;
        }

        if (!curr->cardinalConf.empty() || !curr->rectCardinalConf.empty()) // Resolve a cardinal conflict
        {
            if (Sol1 && abs(n1->g_val - curr->g_val) < 0.001)
            {
                std::cout << "***********ERROR**************" << std::endl;
                system("pause");
            }
            if (Sol2 && abs(n2->g_val - curr->g_val) < 0.001)
            {
                std::cout << "***********ERROR**************" << std::endl;
                system("pause");
            }
        }
        else if (!curr->semiConf.empty() || !curr->rectSemiConf.empty()) // Resolve a semi
        {
            if (Sol1 && Sol2 && abs(n1->g_val + n2->g_val - 2 * curr->g_val) < 0.001)
            {
                std::cout << "***********ERROR**************" << std::endl;
                system("pause");
            }
        }
#endif
        if (!Sol1)
        {
            delete (n1);
            n1 = NULL;
        }
        if (!Sol2)
        {
            delete (n2);
            n2 = NULL;
        }
        //if(curr != dummy_start) // We save dummy_start for statistics analysis later

        curr->clear();
        t1 = std::clock();
        if (open_list.empty())
        {
            solution_found = false;
            break;
        }
#ifdef DEBUG
        cout << " ; (after) " << focal_list_threshold << endl << endl;
#endif
        runtime_listoperation += std::clock() - t1;

    }  // end of while loop


    //    printPaths();
    if (open_list.empty() && solution_cost < 0 && !(runtime > TIME_LIMIT))
    {
        solution_cost = -2;
        cout << "No solutions  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
             HL_num_expanded << " ; " << HL_num_generated << " ; " <<
             LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; " <<
             "|Open|=" << open_list.size() << endl;
        solution_found = false;
    }
    return solution_found;
}


GICBSSearch::GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr,
                         constraint_strategy c, bool fixed_prior, int scr, int mode) : 
                         fixed_prior(fixed_prior), focal_w(f_w), screen(scr), conf_select_mode(mode)
{
    clock_t start_t = std::clock();

    if (screen > 0)
    {
        br_max_ma_size = make_shared<vector<size_t>>();
        br_node_soc = make_shared<vector<int>>();
        br_node_idx = make_shared<vector<uint64_t>>();
    }

    cons_strategy = c;
    //focal_w = f_w;
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
    //ll_min_f_vals = vector <double>(num_of_agents);
    //paths_costs = vector <double>(num_of_agents);
    //ll_min_f_vals_found_initially = vector <double>(num_of_agents);
    //paths_costs_found_initially = vector <double>(num_of_agents);
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

    //std::random_device rd();
    std::mt19937 g(123 /*rd()*/);  // constant seed TODO: Feed it the seed parameter.

    if (fixed_prior)
    {
        int iteration = 0;
        while (true)
        {
            runtime = (std::clock() - start_t);
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
            dummy_start->trans_priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));

            vector<int> ordering(num_of_agents);
            for (int i = 0; i < num_of_agents; i++)
            {
                ordering[i] = i;
            }
            std::shuffle(ordering.begin(), ordering.end(), g);

            // Go over agents in decreasing order of priority
            for (int i = 0; i < num_of_agents; i++)
            {
                int a = ordering[i];
                num_single_pathfinding++;
                if (!search_engines[a]->findPath(paths_found_initially[a], f_w, dummy_start->trans_priorities, paths,
                                                 dummy_start->makespan + 1, 0))
                {
                    iteration++;
                    found = false;
                    break;
                }
                for (int j = i + 1; j < num_of_agents; j++)
                {
                    int b = ordering[j];
                    dummy_start->trans_priorities[a][b] = true;
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
    }
    else
    {
        dummy_start->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
        dummy_start->trans_priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
        for (int i = 0; i < num_of_agents; i++)
        {
            //    cout << "Computing initial path for agent " << i << endl; fflush(stdout);
            //bool* res_table = new bool[map_size * (dummy_start->makespan + 1)]();  // initialized to false
            //bool* res_table_low_prio = new bool[map_size * (dummy_start->makespan + 1)]();  // initialized to false
            //updateReservationTable(res_table, res_table_low_prio, i, *dummy_start);
            //cout << "*** CALCULATING INIT PATH FOR AGENT " << i << ". Reservation Table[MAP_SIZE x MAX_PLAN_LEN]: " << endl;
            //printResTable(res_table, max_plan_len);
            num_single_pathfinding++;
            if (!search_engines[i]->findPath(paths_found_initially[i], f_w, dummy_start->trans_priorities, paths,
                                             dummy_start->makespan + 1, 0))
            {
                cout << "NO SOLUTION EXISTS";
                solution_cost = -2;
                break;
            }
            //dummy_start->paths[i] = search_engines[i]->getPath();
            paths[i] = &paths_found_initially[i];
            dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
            //search_engines[i]->path.reset();
            //ll_min_f_vals_found_initially[i] = search_engines[i]->min_f_val;
            //paths_costs_found_initially[i] = search_engines[i]->path_cost;
            LL_num_expanded += search_engines[i]->num_expanded;
            LL_num_generated += search_engines[i]->num_generated;
            //delete[] res_table;
            //    cout << endl;
        }
    }

    //ll_min_f_vals = ll_min_f_vals_found_initially;
    //paths_costs = paths_costs_found_initially;

    if (solution_cost != -2)
    {
        dummy_start->g_val = 0;
        for (int i = 0; i < num_of_agents; i++)
            dummy_start->g_val += paths[i]->size() - 1;
        dummy_start->h_val = 0;
        dummy_start->f_val = dummy_start->g_val;
        //dummy_start->ll_min_f_val = 0;
        dummy_start->depth = 0;

        dummy_start->open_handle = open_list.push(dummy_start);
        //dummy_start->focal_handle = focal_list.push(dummy_start);
        //dummy_start->single.resize(num_of_agents);
        //dummy_start->constraints.resize(num_of_agents);
        HL_num_generated++;
        dummy_start->time_generated = HL_num_generated;
        allNodes_table.emplace_back(dummy_start);
        // findConflicts(*dummy_start);  // TODO: Should we add the time this takes to runtime_conflictdetection?
                                         //       It would mean it would slightly overlap with pre_runtime.
        //initial_g_val = dummy_start->g_val;
        min_f_val = dummy_start->f_val;
        focal_list_threshold = min_f_val * focal_w;

        //  cout << "Paths in START (high-level) node:" << endl;
        //  printPaths();
        // cout << "SUM-MIN-F-VALS: " << dummy_start->sum_min_f_vals << endl;
    }

    pre_runtime = std::clock() - start_t;
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
    //for (size_t i = 0; i < paths_found_initially.size(); i++)
    //	delete (paths_found_initially[i]);
    //  for (size_t i=0; i<paths.size(); i++)
    //    delete (paths[i]);
    // releaseOpenListNodes();
    //delete (empty_node);
    //delete (deleted_node);
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
	stats.open("iteration_data.txt", std::ios::out);
	if (!stats.is_open())
	{
		cout << "Failed to open file." << endl;
	}
	else
	{
        stats << "br_node_idx,";
		std::copy(br_node_idx->begin(), br_node_idx->end(), std::ostream_iterator<double>(stats, ","));
		stats << endl;
		stats << "br_node_soc,";
		std::copy(br_node_soc->begin(), br_node_soc->end(), std::ostream_iterator<int>(stats, ","));
		stats << endl;
		stats << "br_max_ma_size,";
		std::copy(br_max_ma_size->begin(), br_max_ma_size->end(), std::ostream_iterator<double>(stats, ","));
		stats << endl;
    }
}