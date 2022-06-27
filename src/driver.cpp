#include "map_loader.h"
#include "agents_loader.h"
#include "egraph_reader.h"
#include "GICBSSearch.h"

#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>

#include "boost/program_options.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include<boost/tokenizer.hpp>


namespace pt = boost::property_tree;
using namespace std;

int main(int argc, char** argv)
{

    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("map,m", po::value<std::string>()->required(), "input file for map")
            ("agents,a", po::value<std::string>()->required(), "input file for agents")
            ("output,o", po::value<std::string>()->required(), "output file for schedule")
            ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
            ("seed,s", po::value<int>()->default_value(123), "random seed")
            ("debug,d", po::value<int>()->default_value(0), 
                "debug mode (see README.md for more explenation)")
            ("conf,c", po::value<int>()->default_value(0), 
                "conflict select mode (see README.md for more explenation)")
            ("fixedOrder,f", po::value<bool>()->default_value(true),
             "fixed order. true->Prioritized Search, false->PBS");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    srand(vm["seed"].as<int>());

    // read the map file and construct its two-dim array
    MapLoader ml(vm["map"].as<string>());

    // read agents' start and goal locations
    AgentsLoader al(vm["agents"].as<string>(), ml, vm["agentNum"].as<int>());

    // read the egraph --- we don't use highway here
    EgraphReader egr;

    bool fixed_prior = vm["fixedOrder"].as<bool>();
    int scr = vm["debug"].as<int>();
    int conf_mode = vm["conf"].as<int>();
    GICBSSearch icbs(ml, al, 1.0, egr, constraint_strategy::ICBS, fixed_prior, scr, conf_mode);
    bool res;
    res = icbs.runGICBSSearch();

    if (!icbs.node_stat.empty())
    {
        ofstream stats;
        stats.open(vm["output"].as<string>(), ios::app);
        stats << get<0>(icbs.node_stat.front()) << "," <<
              get<1>(icbs.node_stat.front()) << "," <<
              get<2>(icbs.node_stat.front()) << "," <<
              get<3>(icbs.node_stat.front()) << "," <<
              get<4>(icbs.node_stat.front()) << "," <<
              get<5>(icbs.node_stat.front()) << "," <<
              get<6>(icbs.node_stat.front()) << endl;
        stats.close();
        return 0;
    }
    std::ifstream infile(vm["output"].as<string>());
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(vm["output"].as<string>());
        addHeads << "runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
                 "solution cost,root g value,#pathfinding," <<
                 "preprocessing runtime,computeh runtime,conflictdetection runtime,listoperation runtime,lowlevel runtime,gen child runtime," <<
                 "updatecons runtime,updatepaths runtime," <<
                 "orig agent failed,another agent failed,solver name,instance name," <<
                 "#agents,max_ma_size,num_ex_conf,num_in_conf,num_total_conf," <<
                 "num_0child,num_1child,num_2child" << endl;
        addHeads.close();
    }
    ofstream stats;
    stats.open(vm["output"].as<string>(), ios::app);
    stats << icbs.runtime / CLOCKS_PER_SEC << "," <<
          icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
          icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
          icbs.solution_cost << "," << icbs.dummy_start->g_val << "," <<
          icbs.num_single_pathfinding << "," <<
          icbs.pre_runtime / CLOCKS_PER_SEC << "," << icbs.runtime_computeh / CLOCKS_PER_SEC << "," <<
          icbs.runtime_conflictdetection / CLOCKS_PER_SEC << "," << icbs.runtime_listoperation / CLOCKS_PER_SEC << "," <<
          icbs.runtime_lowlevel / CLOCKS_PER_SEC << "," << icbs.runtime_gen_child / CLOCKS_PER_SEC  << "," << 
          icbs.runtime_updatecons / CLOCKS_PER_SEC << "," << icbs.runtime_updatepaths / CLOCKS_PER_SEC << "," <<
          icbs.agent_itself_failed << "," << icbs.lower_priority_agent_failed <<
          ",PBS," << vm["agents"].as<string>() << "," << vm["agentNum"].as<int>() << "," << 
          icbs.max_ma_size << "," << icbs.num_ex_conf << "," << icbs.num_in_conf << "," << icbs.num_total_conf << "," <<
          icbs.num_0child << "," << icbs.num_1child << "," << icbs.num_2child << endl;
    stats.close();

    if (res)
        return 0;
    else
        return 1;

}
