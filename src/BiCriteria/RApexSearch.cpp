#include <memory>
#include <vector>

#include <iostream>

#include "RApexSearch.h"

RApexSearch::RApexSearch(const AdjacencyMatrix &adj_matrix, EPS eps, const LoggerPtr logger) : AbstractSolver(
        adj_matrix, eps, logger),
    num_of_objectives(adj_matrix.get_num_of_objectives()), rulebook_graph(0) {
    expanded.resize(this->adj_matrix.size() + 1);
}

void RApexSearch::insert(RealizationPairPtr &ap, RPQueue &queue) {
    std::list<RealizationPairPtr> &relevant_aps = queue.get_open(ap->id);
    for (auto existing_ap = relevant_aps.begin(); existing_ap != relevant_aps.end(); ++existing_ap) {
        if ((*existing_ap)->is_active == false) {
            continue;
        }
        if (ap->update_nodes_by_merge_if_bounded(*existing_ap, this->eps, ms) == true) {
            // pp and existing_pp were merged successfuly into pp
            // std::cout << "merge!" << std::endl;
            if ((ap-> rule_apex!= (*existing_ap)->rule_apex) ||
                (ap-> realization!= (*existing_ap)->realization)) {
                // If merged_pp == existing_pp we avoid inserting it to keep the queue as small as possible.
                // existing_pp is deactivated and not removed to avoid searching through the heap
                // (it will be removed on pop and ignored)
                (*existing_ap)->is_active = false;
                queue.insert(ap);
            }
            // both rule_apex and realization are equal -> ap is dominated
            return;
        }
    }
    queue.insert(ap);
}


void RApexSearch::merge_to_solutions(const RealizationPairPtr &ap, RealizationSolutionSet &solutions) {
    for (auto existing_solution = solutions.begin(); existing_solution != solutions.end(); ++existing_solution) {
        if ((*existing_solution)->update_nodes_by_merge_if_bounded(ap, this->eps, ms) == true) {
            return;
        }
    }
    solutions.push_back(ap);
    // std::cout << "update solution checker" << std::endl;
    solution_dom_checker->add_node(ap);
}


bool RApexSearch::is_dominated(RealizationPairPtr ap, bool transferFlag = false){
    if (local_dom_checker->is_dominated(ap, transferFlag)){
        return true;
    }
    return solution_dom_checker->is_dominated(ap, transferFlag);
}


void RApexSearch::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit) {

    init_search();

    auto start_time = std::clock();

    if (this->noDr) {
        local_dom_checker = std::make_unique<LocalCheckRulebookBasic>(eps, this->adj_matrix.size());
        solution_dom_checker = std::make_unique<SolutionCheckRulebookBasic>(eps);
    } else {
        local_dom_checker = std::make_unique<LocalCheckRulebook>(eps, this->adj_matrix.size());
        solution_dom_checker = std::make_unique<SolutionCheckRulebook>(eps);
    }


    this->start_logging(source, target);

    RealizationSolutionSet rp_solutions;
    RealizationPairPtr   rp;
    RealizationPairPtr   next_rp;

    // Saving all the unused PathPairPtrs in a vector improves performace for some reason
    // std::vector<RealizationPairPtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    // std::vector<size_t> min_g2(this->adj_matrix.size()+1, MAX_COST);
    
    // Init open heap
    RPQueue open(this->adj_matrix.size()+1);

    NodePtr source_node = std::make_shared<Node>(source, std::vector<size_t>(num_of_objectives, 0), heuristic(source), nullptr, this->rulebook_graph);
    rp = std::make_shared<RealizationPair>(source_node, source_node, heuristic, rulebook_graph);
    open.insert(rp);

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            for (auto solution = rp_solutions.begin(); solution != rp_solutions.end(); ++solution) {
                solutions.push_back((*solution)->realization);

            }

            this->end_logging(solutions, false);
            return;
        }
        // Pop min from queue and process
        rp = open.pop();
        num_generation +=1;

        // Optimization: PathPairs are being deactivated instead of being removed so we skip them.
        if (rp->is_active == false) {
            continue;
        }

        // Dominance check
        if (is_dominated(rp, true)){
            continue;
        }

        //  min_g2[ap->id] = ap->bottom_right->g[1];
        local_dom_checker->add_node(rp);

        num_expansion += 1;
        if (trace_expanded) trace_expanded->push_back(rp->id);

        expanded[rp->id].push_back(rp);

        if (rp->id == target) {
            this->merge_to_solutions(rp, rp_solutions);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[rp->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            // Prepare extension of path pair

            next_rp = std::make_shared<RealizationPair>(rp, *p_edge);

            // Dominance check
            // if ((((1+this->eps[1])*(bottom_right_next_g[1]+next_h[1])) >= min_g2[target]) ||
            //     (bottom_right_next_g[1] >= min_g2[next_id])) {
            if (is_dominated(next_rp, false)){
                continue;
            }

            // If not dominated extend path pair and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            // std::cout <<"generate node on " << next_ap->id << std::endl;
            this->insert(next_rp, open);
            // closed.push_back(pp);
        }
    }

    // Pair solutions is used only for logging, as we need both the solutions for testing reasons
    for (auto solution = rp_solutions.begin(); solution != rp_solutions.end(); ++solution) {
        solutions.push_back((*solution)->realization);

    }

    this->end_logging(solutions);
}

std::string RApexSearch::get_solver_name() {
    std::string alg_variant;
    if (ms == MergeStrategy::SMALLER_G2){
        alg_variant ="-s2";
    } else if ( ms == MergeStrategy::SMALLER_G2_FIRST){
        alg_variant ="-s2f";
    } else if (ms == MergeStrategy::RANDOM){
        alg_variant ="-r";
    } else if (ms == MergeStrategy::MORE_SLACK){
        alg_variant ="-ms";
    } else if (ms == MergeStrategy::REVERSE_LEX){
        alg_variant ="-rl";
    }
    return "Rulebook" + alg_variant;
}

void RApexSearch::init_search(){
    AbstractSolver::init_search();
    expanded.clear();
    expanded.resize(this->adj_matrix.size()+1);
}
