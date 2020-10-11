#include "SingleAgentICBS.h"

inline int offset2action(int offset, const int* move_offsets)
{
    for (int i = 0; i < 5; i++)
    {
        if (move_offsets[i] == offset)
        {
            return i;
        }
    }
}

inline int action2offset(int action, const int* move_offsets)
{
    return move_offsets[action];
}


void SingleAgentICBS::updatePath(const LLNode* goal, vector<PathEntry>& path)
{
    //path = std::shared_ptr<vector<PathEntry>>(new vector<PathEntry>(goal->timestep + 1));
    path.resize(goal->g_val + 1);
    const LLNode* curr = goal;
    // cout << "   UPDATING Path for one agent to: ";
    num_of_conf = goal->num_internal_conf;
    for (int t = goal->g_val; t >= 0; t--)
    {
        path[t].location = curr->loc;
        curr = curr->parent;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// priorities are the transitive priorities here!
// Returns three CATs in descending order of avoidance priority. The first contains moves of higher priority agents -
// must never collide with them! The second is for agents whose priority ordering with this->agent_id is not set,
// and the third is for agents with lower priority than this->agent_id.
std::tuple<ConflictAvoidanceTable, ConflictAvoidanceTable, ConflictAvoidanceTable>
SingleAgentICBS::buildConflictAvoidanceTables(const vector<vector<bool>>& priorities, const vector<vector<PathEntry>*>& current_paths)
{
    ConflictAvoidanceTable cat_higher_priority(this->moves_offset, this->map_size);
    ConflictAvoidanceTable cat_unset_priority(this->moves_offset, this->map_size);
    ConflictAvoidanceTable cat_lower_priority(this->moves_offset, this->map_size);

    for (int ag = 0; ag < (int) current_paths.size(); ag++)
    {
        if (ag != agent_id && current_paths[ag] != nullptr)
        {
            // Sometimes agents are forced to find a longer path than necessary, find the first time step from which all
            // actions are WAIT
            int first_wait_at_goal_timestep = current_paths[ag]->size();
            for (int j = current_paths[ag]->size() - 2; j >= 0 ; --j) {
                if (current_paths[ag]->at(j).location == current_paths[ag]->at(j + 1).location)
                    --first_wait_at_goal_timestep;
                else
                    break;
            }
            ConflictAvoidanceTable* cat;
            if (priorities[ag][agent_id])
                cat = &cat_higher_priority;
            else {
                if (priorities[agent_id][ag])  // The other agent is of lower priority
                    cat = &cat_lower_priority;
                else
                    cat = &cat_unset_priority;
            }

            for (int i = 1; i < first_wait_at_goal_timestep; i++)
            {
                cat->add_action(i, current_paths[ag]->at(i - 1).location, current_paths[ag]->at(i).location);
            }
            cat->add_wait_at_goal(current_paths[ag]->size(), current_paths[ag]->back().location);
        }
    }
    return std::make_tuple(std::move(cat_higher_priority), std::move(cat_unset_priority), std::move(cat_lower_priority));
}

// $$$ -- is there a more efficient way to do that?
void SingleAgentICBS::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
bool SingleAgentICBS::findPath(vector<PathEntry>& path, double f_weight, const vector<vector<bool>>& priorities,
                               const vector<vector<PathEntry>*>& current_paths, size_t max_plan_len, double lowerbound)
{
    // clear data structures if they had been used before
    // (note -- nodes are deleted before findPath returns)

    // vector<bool> next_colliding_agents = vector<bool>(curr->colliding_agents);
    auto [cat_higher_priority, cat_unset_priority, cat_lower_priority] = buildConflictAvoidanceTables(priorities, current_paths);

    num_expanded = 0;
    num_generated = 0;

    hashtable_t::iterator it;  // will be used for find()

    // generate start and add it to the OPEN list
    LLNode* start = new LLNode(start_location, 0, my_heuristic[start_location], NULL, 0, 0, false);
    start->colliding_agents = vector<bool>(current_paths.size(), false);
    num_generated++;
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist = true;
    allNodes_table[start] = std::unique_ptr<LLNode>(start);
    min_f_val = start->getFVal();
    lower_bound = max(lowerbound, f_weight * min_f_val);

    int lastGoalConsTime = cat_higher_priority.latest_vertex_entry(goal_location);
    int timeLastConstraintAvoidableWithWait = cat_higher_priority.latest_entry() - 1;  // Note locations _can_ be blocked after this time if they
                                                                                       // are blocked forever, but waiting won't help avoid them

#ifdef  _DEBUG
    if (agent_id == 0 || agent_id == 7) {
        cout << endl;
        for (int ag = 0; ag < current_paths.size(); ag++) {
            if (priorities[ag][agent_id]) {
                cout << ag << "->" << agent_id << endl;
                if (ag == 0 || ag == 7) {
                    cout << "!!!";
                }
            }
        }
    }
#endif //  _DEBUG

    while (!focal_list.empty())
    {
        // DO NOT CHECK FOR A TIMEOUT HERE!!
        // THIS LOW-LEVEL SEARCH IS SUPPOSED TO BE ABLE TO TERMINATE IN (relatively short) FINITE TIME BECAUSE THE MAP
        // IS FINITE. IF YOUR RUNS GET STUCK RUNNING FOREVER HERE, CHECK THAT THE "SUPER IMPORTANT" TRICK BELOW WAS NOT
        // TEMPERED WITH.

        //    cout << "|F|=" << focal_list.size() << " ; |O|=" << open_list.size() << endl;
        LLNode* curr = focal_list.top();
        focal_list.pop();
        //    cout << "Current FOCAL bound is " << lower_bound << endl;
        //    cout << "POPPED FOCAL's HEAD: (" << curr << ") " << (*curr) << endl;
        open_list.erase(curr->open_handle);
        //    cout << "DELETED" << endl; fflush(stdout);
        curr->in_openlist = false;
        num_expanded++;

        // check if the popped node is a goal
        if (curr->loc == goal_location)
        {
            bool is_goal = true;
            if (curr->g_val < lastGoalConsTime)
                is_goal = false;
            if (is_goal)
            {
                updatePath(curr, path);
                open_list.clear();
                focal_list.clear();
                allNodes_table.clear();
                return true;
            }
        }

        int next_id;
        for (int i = 0; i < MapLoader::MOVE_COUNT; i++)
        {
            if (curr->timestep >= timeLastConstraintAvoidableWithWait && i == MapLoader::WAIT_MOVE)
            { // no reason to wait
                continue;
            }

            next_id = curr->loc + moves_offset[i];

            int next_timestep = curr->timestep + 1;;
            if (curr->timestep == timeLastConstraintAvoidableWithWait)
                next_timestep = curr->timestep;  // Super important! A node's identity is derived from its location and
                                                 // its time step, but NOT from its g-value. This allows us to exhaust
                                                 // the open list once we pass the last avoidable constraint. Otherwise
                                                 // we would be able to move back and forth between two locations forever
                                                 // if the agent is blocked by a higher priority agent waiting at its goal.
            int next_g_val = curr->g_val + 1;

            if (0 <= next_id && next_id < map_size && abs(next_id % moves_offset[MapLoader::valid_moves_t::SOUTH] -
                                                          curr->loc % moves_offset[MapLoader::valid_moves_t::SOUTH]) <
                                                      2)
            {
                //bool free = true;
                //int num_row = map_size / num_col;
                //cout << "NUMBER of rows and cols: " <<num_row << " " << num_col << endl;;
                //int row = next_id / num_col;
                //int col = next_id % num_col;

                // bool is_cons_0 = isConstrained(curr->loc, next_id, next_g_val, priorities, current_paths);
                int next_h_val = my_heuristic[next_id];  // Can be deferred to inside the if below, but helps debugging
                bool is_blocked = my_map[next_id];
                bool is_constrained = is_blocked || (cat_higher_priority.num_conflicts_for_step(curr->loc, next_id, next_g_val) > 0);
                if (!is_blocked && !is_constrained)
                {
                    // generate (maybe temporary) node
                    int next_internal_conflicts = curr->num_internal_conf + cat_unset_priority.num_conflicts_for_step(curr->loc, next_id, next_g_val);
                    int next_internal_conflicts_lp = curr->num_internal_conf_lp + cat_lower_priority.num_conflicts_for_step(curr->loc, next_id, next_g_val);
                    auto next = new LLNode(next_id, next_g_val, next_h_val, curr, next_timestep,
                                           next_internal_conflicts, next_internal_conflicts_lp, false);

                    // next->colliding_agents = next_colliding_agents;
                    // cout << "   NEXT(" << next << ")=" << *next << endl;
                    // try to retrieve it from the hash table
                    it = allNodes_table.find(next);

                    if (it == allNodes_table.end())
                    {
                        //          cout << "   ADDING it as new." << endl;
                        next->open_handle = open_list.push(next);
                        next->in_openlist = true;
                        num_generated++;
                        if (next->getFVal() <= lower_bound)
                            next->focal_handle = focal_list.push(next);
                        allNodes_table[next] = std::unique_ptr<LLNode>(next);
                    }
                    else
                    {  // update existing node's g, h, and other details if needed (only if it's still in the open_list)
                        delete (next);  // not needed anymore -- we already generated it before
                        LLNode* existing_next = (*it).second.get();
                        //          cout << "Actually next exists. Its address is " << existing_next << endl;
                        if (existing_next->in_openlist == true)
                        {  // if it's in the open list
                            if (existing_next->getFVal() > next_g_val + next_h_val ||
                                (existing_next->getFVal() == next_g_val + next_h_val &&
                                 existing_next->num_internal_conf > next_internal_conflicts) ||
                                (existing_next->getFVal() == next_g_val + next_h_val &&
                                 existing_next->num_internal_conf == next_internal_conflicts &&
                                 existing_next->num_internal_conf_lp > next_internal_conflicts_lp))
                            {
                                // if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
                                //              cout << "   UPDATE its f-val in OPEN (decreased or less #conflicts)" << endl;
                                //              cout << "   Node state before update: " << *existing_next;
                                bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
                                bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
                                bool update_open = false;
                                if ((next_g_val + next_h_val) <= lower_bound)
                                {  // if the new f-val qualify to be in FOCAL
                                    if (existing_next->getFVal() > lower_bound)
                                        add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                                    else
                                        update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
                                }
                                if (existing_next->getFVal() > next_g_val + next_h_val)
                                    update_open = true;
                                // update existing node
                                existing_next->g_val = next_g_val;
                                existing_next->h_val = next_h_val;
                                existing_next->parent = curr;
                                existing_next->num_internal_conf = next_internal_conflicts;
                                existing_next->num_internal_conf_lp = next_internal_conflicts_lp;
                                //              cout << "   Node state after update: " << *existing_next;
                                if (update_open)
                                {
                                    open_list.increase(existing_next->open_handle);  // increase because f-val improved
                                    //                cout << "     Increased in OPEN" << endl;
                                }
                                if (add_to_focal)
                                {
                                    existing_next->focal_handle = focal_list.push(existing_next);
                                    //                cout << "     Inserted to FOCAL" << endl;
                                }
                                if (update_in_focal)
                                {
                                    focal_list.update(
                                            existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
                                    //                cout << "     Updated in FOCAL" << endl;
                                }
                            }
                            //            cout << "   Do NOT update in OPEN (f-val for this node increased or stayed the same and has more conflicts)" << endl;
                        }
                    }  // end update an existing node
                }  // end if case for grid not blocked
            }
        }  // end for loop that generates successors
        // update FOCAL if min f-val increased
        if (open_list.empty())  // in case OPEN is empty, no path found...
            break;
        LLNode* open_head = open_list.top();
        if (open_head->getFVal() > min_f_val)
        {
            double new_min_f_val = open_head->getFVal();
            double new_lower_bound = max(lowerbound, f_weight * new_min_f_val);
            /*
            cout << "LL FOCAL UPDATE! Old-f-min=" << min_f_val << " ; Old-LB=" << lower_bound << endl;
            cout << "OPEN: ";
            for (Node* n : open_list)
            cout << n << " , ";
            cout << endl;
            cout << "FOCAL: ";
            for (Node* n : focal_list)
            cout << n << " , ";
            cout << endl;
            */
            //  cout << "Update Focal: (old_LB=" << lower_bound << " ; new_LB=" << new_lower_bound << endl;;
            for (LLNode* n : open_list)
            {
                //    cout << "   Considering " << n << " , " << *n << endl;
                if (n->getFVal() > lower_bound &&
                    n->getFVal() <= new_lower_bound)
                {
                    //      cout << "      Added (n->f-val=" << n->getFVal() << ")" << endl;
                    n->focal_handle = focal_list.push(n);
                }
            }
            //updateFocalList(lower_bound, new_lower_bound, f_weight);
            min_f_val = new_min_f_val;
            lower_bound = new_lower_bound;
            /*
            cout << "   New-f-min=" << min_f_val << " ; New-LB=" << lower_bound << endl;
            cout << "FOCAL: ";
            for (Node* n : focal_list)
            cout << n << " , ";
            cout << endl;
            */
        }
        if (focal_list.empty())
            std::cout << "ERROR!" << std::endl;
    }  // end while loop
    // no path found
    //path.clear();
    open_list.clear();
    focal_list.clear();
    allNodes_table.clear();
    return false;
}

SingleAgentICBS::SingleAgentICBS(int id, int start_location, int goal_location,
                                 const bool* my_map, int map_size, const int* moves_offset, int num_col)
{
    this->agent_id = id;
    this->my_map = my_map;
    this->moves_offset = moves_offset;
    //this->actions_offset = actions_offset;
    this->start_location = start_location;
    this->goal_location = goal_location;
    //this->start_orientation = start_orientation;
    this->map_size = map_size;
    //this->e_weight = e_weight;
    this->num_expanded = 0;
    this->num_generated = 0;
    //this->path_cost = 0;
    this->lower_bound = 0;
    this->min_f_val = 0;
    //this->num_non_hwy_edges = 0;
    this->num_col = num_col;

    // initialize allNodes_table (hash table)
    empty_node = new LLNode();
    empty_node->loc = -1;
    deleted_node = new LLNode();
    deleted_node->loc = -2;
    //allNodes_table.set_empty_key(empty_node);
    //allNodes_table.set_deleted_key(deleted_node);

}


SingleAgentICBS::~SingleAgentICBS()
{
    delete[] my_map;
    delete (empty_node);
    delete (deleted_node);
}
