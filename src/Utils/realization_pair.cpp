
#include "Utils/Definitions.h"

bool RealizationPair::keep_paths = false;

// return true if node dom ape x
// bool is_dominated_dr(NodePtr rule_apex, NodePtr node){
//     for (int i = 1; i < rule_apex->f.size(); i ++ ){
//       if (node->f[i] > rule_apex->f[i]){
//         return false;
//       }
//     }
//   // TODO MORE
//   return true;
// }
//
// // (dominatee ,dominator)
// bool is_dominated_dr(NodePtr rule_apex, NodePtr node, const EPS eps){
//     for (int i = 1; i < rule_apex->f.size(); i ++ ){
//         if (node->f[i] > (1 + eps[i]) * rule_apex->f[i]){
//             return false;
//         }
//     }
//     // TODO MORE
//     return true;
// }
//
//
// bool is_bounded(NodePtr rule_apex, NodePtr node,  const EPS eps){
//     for (int i = 0; i < rule_apex->f.size(); i ++ ){
//         if (node->f[i] > (1 + eps[i]) * rule_apex->f[i]){
//             return false;
//         }
//     }
//     return true;
// }

RealizationPair::RealizationPair(const RealizationPairPtr parent, const Edge&  edge): h(parent->h), rulebook_graph(parent->rulebook_graph){
    size_t next_id = edge.target;
    id = next_id;

    std::vector<size_t> new_rule_apex_g(parent->rule_apex->g);
    std::vector<size_t> new_g(parent->realization->g);

    for (int i = 0; i < new_rule_apex_g.size(); i ++){
        new_rule_apex_g[i] += edge.cost[i];
        new_g[i] += edge.cost[i];
    }
    auto new_h = h(next_id);
    this->rule_apex = std::make_shared<Node>(next_id, new_rule_apex_g, new_h, nullptr, parent->rule_apex->rulebook_graph);
    this->realization = std::make_shared<Node>(next_id, new_g, new_h, keep_paths ? parent->realization : nullptr,
                                               parent->realization->rulebook_graph);
}

bool RealizationPair::update_nodes_by_merge_if_bounded(const RealizationPairPtr &other, const std::vector<double> eps, MergeStrategy s){
  // Returns true on successful merge and false if it failure
  if (this->id != other->id) {
    return false;
  }

  NodePtr new_rule_apex = std::make_shared<Node>(this->rule_apex->id, this->rule_apex->g, this->rule_apex->h);
  NodePtr new_realization = nullptr;

  // Merge rule_apex
  for (int i = 0; i < other->rule_apex->g.size(); i ++){
    if (other->rule_apex->g[i] < new_rule_apex->g[i]){
      new_rule_apex->g[i] = other->rule_apex->g[i];
      new_rule_apex->f[i] = other->rule_apex->f[i];
    }
  }

  // choose a path node
  if (s == MergeStrategy::RANDOM){
    if (this->rulebook_graph.dominates(this->realization->f,new_rule_apex->f, eps)){
      new_realization = this->realization;
    }
    if (this->rulebook_graph.dominates(other->realization->f, new_rule_apex->f, eps)){
      if (new_realization == nullptr){
        new_realization = other->realization;
      }else{
        if (rand() % 2 == 1){
          new_realization = other->realization;
        }
      }
    }
    if (new_realization == nullptr){
      return false;
    }
  } else {
    std::cerr << "merge strategy not known" << std::endl;
    exit(-1);
  }

    // Check if path pair is bounded after merge - if not the merge is illegal
  // if ((((1+eps[0])*new_top_left->g[0]) < new_bottom_right->g[0]) ||
  //     (((1+eps[1])*new_bottom_right->g[1]) < new_top_left->g[1])) {
  //   return false;
  // }

  this->rule_apex = new_rule_apex;
  this->realization = new_realization;
  return true;
}

bool RealizationPair::update_rule_apex_by_merge_if_bounded(const NodePtr &other_rule_apex, const std::vector<double> eps){
  // if (!is_bounded(other_pseudoR, path_node, eps)){
  //   return false;
  // }
  NodePtr new_rule_apex = std::make_shared<Node>(this->rule_apex->id, this->rule_apex->g, this->rule_apex->h);
  bool update_flag = false;
  // Merge rule_apex
  for (int i = 0; i < other_rule_apex->g.size(); i ++){
    if (other_rule_apex->f[i] < new_rule_apex->f[i]){
      new_rule_apex->g[i] = other_rule_apex->f[i];
      new_rule_apex->f[i] = other_rule_apex->f[i];
      update_flag = true;
    }
  }
  if (!this->rulebook_graph.dominates(realization->f, new_rule_apex->f, eps)) {
    return false;
  }
  if (update_flag){
    rule_apex = new_rule_apex;
  }
  return true;
}



bool RealizationPair::more_than_full_cost::operator()(const RealizationPairPtr &a, const RealizationPairPtr &b) const {
  std::vector<size_t> rules = a->rulebook_graph.get_ordered_rules();
  for (size_t i = 0; i + 1 < rules.size(); i ++) {
    if (a->rule_apex->f[rules[i]] != b->rule_apex->f[rules[i]]) {
      return (a->rule_apex->f[rules[i]] > b->rule_apex->f[rules[i]]);
    }
  }
  return (a->rule_apex->f[rules.back()] > b->rule_apex->f[rules.back()]);
}


std::ostream& operator<<(std::ostream &stream, const RealizationPair &ap) {
  // Printed in JSON format
  stream << "{" << *(ap.rule_apex) << ", " << *(ap.realization) << "}";
  return stream;
}