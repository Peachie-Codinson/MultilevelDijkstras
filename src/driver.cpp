#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>
#include <random>

#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"
#include "Utils/IOUtils.h"
#include "Utils/Logger.h"
#include "BOAStar.h"
#include "PPA.h"
#include "SingleCriteria.h"
#include "ApexSearch.h"
#include "NAMOA.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

#include "RApexSearch.h"
#include "MultiLevelDijkstra.h"

#include "RulebookPlanning/Rulebook.h"
#include "RulebookPlanning/RulebookCost.h"
#include "RulebookPlanning/WeightedGraph.h"

using namespace std;
using WEdge = WeightedEdge<RulebookCost>;
using VertexPtr = std::shared_ptr<Vertex>;
using WEdgePtr = std::shared_ptr<WEdge>;

const std::string resource_path = "resources/";
const std::string output_path = "output/";
const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::RANDOM;
std::string alg_variant = "";


// Simple example to demonstarte the usage of the algorithm

SolutionSet single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, EPS &epsV, unsigned int time_limit, RulebookGraph& rgraph) {
    // Compute heuristic
    std::cout << "Start Computing Heuristic" << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    // sp_heuristic.set_all_to_zero();
    std::cout << "Finish Computing Heuristic\n" << std::endl;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

    SolutionSet solutions;
    int num_exp, num_gen;
    auto runtime = std::clock();

    std::unique_ptr<AbstractSolver> solver;
    if (algorithm == "PPA"){
        Pair<double> eps_pair({epsV[0], epsV[1]});
        solver = std::make_unique<PPA>(graph, eps_pair, logger);
    }else if (algorithm == "BOA"){
        Pair<double> eps_pair({epsV[0], epsV[1]});
        solver = std::make_unique<BOAStar>(graph, eps_pair, logger);
    }else if (algorithm == "NAMOAdr"){
        solver = std::make_unique<NAMOAdr>(graph, epsV, logger);
        // ((ApexSearch*)solver.get())->set_merge_strategy(ms);
    }else if (algorithm == "Apex"){
        solver = std::make_unique<ApexSearch>(graph, epsV, logger);
        ((ApexSearch*)solver.get())->set_merge_strategy(ms);
    }else if (algorithm == "RApex") {
        solver = std::make_unique<RApexSearch>(graph, epsV, logger);
        ((RApexSearch*)solver.get())->set_noDr(false);
        ((RApexSearch*)solver.get())->set_merge_strategy(ms);
        ((RApexSearch*)solver.get())->set_rulebook_graph(rgraph);
    }else if (algorithm == "RApexNoDr") {
        solver = std::make_unique<RApexSearch>(graph, epsV, logger);
        ((RApexSearch*)solver.get())->set_noDr(true);
        ((RApexSearch*)solver.get())->set_merge_strategy(ms);
        ((RApexSearch*)solver.get())->set_rulebook_graph(rgraph);
    }else{
        std::cerr << "unknown solver name" << std::endl;
        exit(-1);
    }
    auto start =std::clock();
    (*solver)(source, target, heuristic, solutions, time_limit);
    runtime = std::clock() - start;

    std::cout << "Node expansion: " << solver->get_num_expansion() << std::endl;
    std::cout << "Runtime: " <<  ((double) runtime) / CLOCKS_PER_SEC<< std::endl;
    num_exp = solver->get_num_expansion();
    num_gen = solver->get_num_generation();
    for (auto sol: solutions){
        std::cout << *sol << std::endl;
    }


    output << algorithm << "-" << alg_variant << "eps(";
    for (size_t i = 0; i < epsV.size(); i++) {
        output << epsV[i] << (i < (epsV.size()-1) ? ", " : "");
    }
    output << ")" << "\t"
           << source << "\t" << target << "\t"
           << num_gen << "\t"
           << num_exp << "\t"
           << solutions.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;

    std::cout << "-----End Single Example-----" << std::endl;

    return solutions;
}

OptimalSet<std::vector<WEdgePtr>, RulebookCost> single_run_map_planning(size_t graph_size, WeightedGraph<RulebookCost> &graph, size_t source, size_t target, std::ostream& output, LoggerPtr logger, size_t time_limit, EPS epsV, bool approx = false) {
    auto start =std::clock();
    const auto optimal_set = graph.getOptimalPaths(source, target, time_limit, approx);
    auto runtime = std::clock() - start;

    std::cout << "Graph size: " << graph_size << std::endl;
    std::cout << "Runtime: " <<  ((double) runtime) / CLOCKS_PER_SEC<< std::endl;

    std::cout << optimal_set << std::endl;

    vector<vector<size_t>> planningSolutionsVector;
    for (size_t eid : optimal_set.getAllElementIDs()) {
        const auto ele = optimal_set.getElement(eid);
        vector<size_t> sol;
        for (const auto& it : ele.cost.getCosts()) {
            sol.push_back(it->getValue());
        }
        planningSolutionsVector.push_back(sol);
    }
    std::sort(planningSolutionsVector.begin(), planningSolutionsVector.end());
    planningSolutionsVector.erase(std::unique(planningSolutionsVector.begin(), planningSolutionsVector.end()), planningSolutionsVector.end());

    output << "Planning" << "-" << "eps(";
    for (size_t i = 0; i < epsV.size(); i++) {
        output << epsV[i] << (i < (epsV.size()-1) ? ", " : "");
    }
    output << ")" << "\t"
           << source << "\t" << target << "\t"
           << planningSolutionsVector.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;

    std::cout << "-----End Single Example-----" << std::endl;

    return optimal_set;
}

void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string rules_file, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, int time_limit) {
    std::ofstream stats;
    stats.open(output_file);

    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(query_file, queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    Rulebook planning_rulebook;
    RulebookGraph rapex_rb_graph;
    EPS epsV;
    if (load_rules(rules_file, planning_rulebook, rapex_rb_graph, epsV) == false) {
        std::cout << "Failed to load rules file" << std::endl;
        return;
    }

    planning_rulebook.build();
    rapex_rb_graph.calculate_quotient_graph();

    std::vector<size_t> multi_level_order;
    bool multi_level_fork = false;
    if (algorithm == "MultiLevel") {
        multi_level_fork = planning_rulebook.getNumRules() == 3 &&
            planning_rulebook.isAllSingletons() &&
            planning_rulebook.getSuccessors(0) == std::unordered_set<size_t>{1, 2} &&
            planning_rulebook.getSuccessors(1).empty() &&
            planning_rulebook.getSuccessors(2).empty();
        if (!multi_level_fork && !planning_rulebook.isTotallyOrdered())
            throw std::invalid_argument("MultiLevel supports a strict chain or r0 > {r1,r2}; use RBExact for other rulebooks");
        if (!multi_level_fork)
            for (const auto& rule_class : planning_rulebook)
                multi_level_order.push_back(*rule_class.begin());
    }

    if (algorithm == "MultiLevel") {
        size_t query_count = 0;
        for (const auto& [source, target] : queries) {
            ++query_count;
            auto result = multi_level_fork
                ? multi_level::solve_fork(edges, graph_size, source, target, time_limit)
                : multi_level::solve(edges, graph_size, source, target,
                                     multi_level_order, time_limit);
            const std::string path = output_file + ".q" + std::to_string(query_count) + ".edges";
            if (result.reachable) multi_level::write_subgraph(path, edges, result);
            const std::string solutions_path = output_file + ".q" + std::to_string(query_count) + ".solutions.txt";
            std::ofstream solutions_out(solutions_path);
            if (!solutions_out) throw std::runtime_error("cannot write " + solutions_path);
            for (const auto& cost : result.solutions)
                solutions_out << cost[0] << ' ' << cost[1] << ' ' << cost[2] << '\n';
            std::cout << "Query " << query_count << ": "
                      << (result.timed_out ? "timed out" : result.reachable ? "optimal" : "unreachable")
                      << ", edges=" << result.optimal_edges.size() << ", cost=[";
            for (size_t i = 0; i < result.cost.size(); ++i)
                std::cout << (i ? ", " : "") << result.cost[i];
            std::cout << "]\n";
            stats << "MultiLevel\t" << source << '\t' << target << '\t'
                  << result.solutions.size() << '\t' << result.seconds << '\t'
                  << result.optimal_edges.size() << '\t'
                  << (result.reachable ? path : "-") << '\t' << solutions_path
                  << '\t' << (result.timed_out ? "TIMEOUT" : "COMPLETE") << '\n';
        }
        return;
    }

    // Build the graph for Rulebook Planning
    RulebookCost::setRulebook(planning_rulebook);
    WeightedGraph<RulebookCost> planning_graph;
    unordered_map<size_t, bool> Vs;
    for (auto &e : edges) {
        if (Vs.find(e.source) == Vs.end()) {
            planning_graph.addVertex(e.source);
            Vs.insert({e.source, true});
        }
        if (Vs.find(e.target) == Vs.end()) {
            planning_graph.addVertex(e.target);
            Vs.insert({e.target, true});
        }
        RulebookCost c;
        for (int i=0; i<planning_rulebook.getNumRules(); i++) {
            c.setRuleCost(i, e.cost[i]);
            c.setRuleEps(i, epsV[i]);
        }
        planning_graph.addEdge(e.source, e.target, c);
    }

    // Build the graph for all other algorithms
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

        query_count++;
        std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
        size_t source = iter->first;
        size_t target = iter->second;
        if (algorithm == "RBExact" || algorithm == "RBApprox") {
            // if (query_count != 4 && query_count != 8 && query_count != 11) continue;
            single_run_map_planning(graph_size, planning_graph, source, target, stats, logger, time_limit, epsV, algorithm == "RBApprox");
        } else {
            single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, epsV, time_limit, rapex_rb_graph);
        }
    }
    stats.close();
}

int main(int argc, char** argv){
    namespace po = boost::program_options;

    std::vector<string> objective_files;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("query,q", po::value<std::string>()->default_value(""), "file for queries")
        ("rules,r", po::value<std::string>()->default_value(""), "rules file")
        ("map,m",po::value< std::vector<std::string> >(&objective_files)->multitoken(), "files for edge weight")
        ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
        ("algorithm,a", po::value<std::string>()->default_value("RApex"), "solvers (BOA, PPA, Apex, RBExact, RBApprox, RApex, RApexNoDr, MultiLevel)")
        ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
        ("output,o", po::value<std::string>()->required(), "Name of the output file")
        ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
        // ("start,s", po::value<int>()->default_value(-1), "start location")
        // ("goal,g", po::value<int>()->default_value(-1), "goal location")
        // ("eps,e", po::value<double>()->default_value(0), "approximation factor")
        // ("noDr", "do not use dimensionality reduction")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    srand((int)time(0));

    // if (vm["query"].as<std::string>() != ""){
    //     if (vm["start"].as<int>() != -1 || vm["goal"].as<int>() != -1){
    //         std::cerr << "query file and start/goal cannot be given at the same time !" << std::endl;
    //         return -1;
    //     }
    // }

    LoggerPtr logger = nullptr;

    if (vm["logging_file"].as<std::string>() != ""){
        logger = new Logger(vm["logging_file"].as<std::string>());
    }

    // Load files
    size_t graph_size;
    std::vector<Edge> edges;

    for (auto file:objective_files){
        std::cout << file << std::endl;
    }


    if (load_gr_files(objective_files, edges, graph_size) == false) {
        std::cout << "Failed to load gr files" << std::endl;
        return -1;
    }

    std::cout << "Graph Size: " << graph_size << std::endl;

    // Build graphs
    MergeStrategy ms = DEFAULT_MERGE_STRATEGY;
    alg_variant = vm["merge"].as<std::string>();

    if (vm["merge"].as<std::string>() != "" && vm["algorithm"].as<std::string>() != "Apex" &&
        vm["algorithm"].as<std::string>() != "RApex" && vm["algorithm"].as<std::string>() != "RApexNoDr"){
        alg_variant = "";
        std::cout << "WARNING: merge strategy with non-apex search" << std::endl;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2"){
        ms = MergeStrategy::SMALLER_G2;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2_FIRST"){
        ms = MergeStrategy::SMALLER_G2_FIRST;
    }else if(vm["merge"].as<std::string>() == "RANDOM"){
        ms = MergeStrategy::RANDOM;
    }else if(vm["merge"].as<std::string>() == "MORE_SLACK"){
        ms = MergeStrategy::MORE_SLACK;
    }else if(vm["merge"].as<std::string>() == "REVERSE_LEX"){
        ms = MergeStrategy::REVERSE_LEX;
    }else if(vm["merge"].as<std::string>() != ""){
        std::cerr << "unknown merge strategy" << std::endl;
    }

    if (vm["query"].as<std::string>() != ""){
        run_query(graph_size, edges, vm["query"].as<std::string>(), vm["rules"].as<std::string>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["cutoffTime"].as<int>());
    }

    delete(logger);

    return 0;
}
