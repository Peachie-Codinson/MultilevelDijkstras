#include "Utils/RulebookGraph.h"
#include <algorithm>
#include <cassert>

// runs depth first search starting at vertex v.
// each visited vertex is appended to the output vector when dfs leaves it.
void RulebookGraph::dfs(size_t v, std::vector<std::vector<size_t>> const& adj, std::vector<size_t> &output, std::vector<bool> &visited) {
    visited[v] = true;
    for (auto u : adj[v])
        if (!visited[u])
            dfs(u, adj, output, visited);
    output.push_back(v);
}

// input: adj -- adjacency list of G
// output: components -- the strongy connected components in G
// output: adj_cond -- adjacency list of G^SCC (by root vertices)
void RulebookGraph::strongly_connected_components() {
    size_t n = adj.size();
    components.clear(), qAdj.clear();

    std::vector<size_t> order; // will be a sorted list of G's vertices by exit time

    std::vector<bool> visited(n, false); // keeps track of which vertices are already visited

    // first series of depth first searches
    for (int i = 0; i < n; i++)
        if (!visited[i])
            dfs(i, adj, order, visited);

    // create adjacency list of G^T
    std::vector<std::vector<size_t>> adj_rev(n);
    for (int v = 0; v < n; v++)
        for (int u : adj[v])
            adj_rev[u].push_back(v);

    visited.assign(n, false);
    std::reverse(order.begin(), order.end());

    comp_roots.assign(n, 0); // gives the root vertex of a vertex's SCC

    size_t rootCnt = 0;
    // second series of depth first searches
    for (auto v : order)
        if (!visited[v]) {
            std::vector<size_t> component;
            dfs(v, adj_rev, component, visited);
            components.push_back(component);
            int root = rootCnt++;
            for (auto u : component)
                comp_roots[u] = root;
        }

    // add edges to condensation graph
    qAdj.assign(rootCnt, {});
    for (int v = 0; v < n; v++)
        for (auto u : adj[v])
            if (comp_roots[v] != comp_roots[u])
                qAdj[comp_roots[v]].push_back(comp_roots[u]);
}

void RulebookGraph::dfs(size_t v, std::vector<bool> &visited, std::vector<size_t> &output) {
    visited[v] = true;
    for (int u : qAdj[v]) {
        if (!visited[u]) {
            dfs(u, visited, output);
        }
    }
    output.push_back(v);
}

void RulebookGraph::topological_sort() {
    std::vector<bool> visited(qAdj.size(), false);
    ordered_components.clear();
    for (int i = 0; i < static_cast<int>(qAdj.size()); ++i) {
        if (!visited[i]) {
            dfs(i, visited, ordered_components);
        }
    }
    std::reverse(ordered_components.begin(), ordered_components.end());
}

void RulebookGraph::add_relationship(size_t fromR, size_t toR) {
    const size_t maxRule = std::max(fromR, toR);
    if (maxRule >= numRules) {
        numRules = maxRule + 1;
        adj.resize(numRules);
    }
    adj[fromR].push_back(toR);
}

void RulebookGraph::calculate_quotient_graph() {
    strongly_connected_components();
    ordered_components.clear();
    topological_sort();
    numQRules = ordered_components.size();

    ordered_rules.clear();
    for (const auto &v : ordered_components) {
        for (const auto c : components[v]) {
            ordered_rules.push_back(c);
        }
    }
}

const std::vector<size_t>& RulebookGraph::get_ordered_rules() const {
    return ordered_rules;
}

const std::vector<size_t>& RulebookGraph::get_Q_neighbors(size_t rule) const {
    return qAdj[comp_roots[rule]];
}

size_t RulebookGraph::get_Q_root(size_t rule) const {
    return comp_roots[rule];
}

const std::vector<size_t>& RulebookGraph::get_component_rules(size_t rootId) const {
    return components[rootId];
}

void RulebookGraph::recursively_exclude(size_t comp, std::vector<bool>& excluded) const {
    excluded[comp] = true;
    for (const size_t c : qAdj[comp]) {
        if (excluded[c]) continue;
        recursively_exclude(c, excluded);
    }
}

// returns true if a is dr-weakly-rule-dominated by b
bool RulebookGraph::is_dominated_equal(const std::vector<size_t> &a, const std::vector<size_t> &b) const {
    std::vector<bool> excluded(numQRules, false);
    assert(comp_roots[ordered_rules[0]] == ordered_components[0]);
    for (int i=0; i < numQRules; i++) {
        size_t c = ordered_components[i];
        if (excluded[c]) continue;
        bool better = false;
        for (const size_t rule : components[c]) {
            if (!i && rule == ordered_rules[0]) continue;
            if (a[rule] < b[rule]) {
                return false;
            }
            if (a[rule] > b[rule]) {
                better = true;
            }
        }
        if (better) {
            recursively_exclude(c, excluded);
        }
    }
    return true;
}

bool RulebookGraph::is_dominated_less_than(const std::vector<size_t> &a, const std::vector<size_t> &b) const {
    std::vector<bool> excluded(numQRules, false);
    assert(comp_roots[ordered_rules[0]] == ordered_components[0]);
    for (int i=0; i < numQRules; i++) {
        size_t c = ordered_components[i];
        if (excluded[c]) continue;
        bool better = false;
        for (const size_t rule : components[c]) {
            if (!i && rule == ordered_rules[0]) {
                better = true;
                continue;
            }
            if (a[rule] < b[rule]) {
                return false;
            }
            if (a[rule] > b[rule]) {
                better = true;
            }
        }
        if (better) {
            recursively_exclude(c, excluded);
        }
    }
    return true;
}

// Returns true if Tr(a) is weakly rule-dominated by Tr(b) (in the truncated space)
bool RulebookGraph::is_dominated_equal(const std::vector<size_t> &a, const std::vector<size_t> &b, const std::vector<double> eps) const {
    std::vector<bool> excluded(numQRules, false);
    assert(comp_roots[ordered_rules[0]] == ordered_components[0]);
    for (int i=0; i < numQRules; i++) {
        size_t c = ordered_components[i];
        if (excluded[c]) continue;
        bool better = false;
        for (const size_t rule : components[c]) {
            if (!i && rule == ordered_rules[0]) {
                better = (eps[rule] > 0.0000000001);
                continue;
            }
            if (a[rule] * (1.0 + eps[rule]) < b[rule]) {
                return false;
            }
            if (a[rule] * (1 + eps[rule]) > b[rule]) {
                better = true;
            }
        }
        if (better) {
            recursively_exclude(c, excluded);
        }
    }
    return true;
}


// Returns true if alpha(a) is weakly rule-dominated by alpha(b) (in the residual space)
bool RulebookGraph::is_dominated_less_than(const std::vector<size_t> &a, const std::vector<size_t> &b, const std::vector<double> eps) const {
    std::vector<bool> excluded(numQRules, false);
    assert(comp_roots[ordered_rules[0]] == ordered_components[0]);
    for (int i=0; i < numQRules; i++) {
        size_t c = ordered_components[i];
        if (excluded[c]) continue;
        bool better = false;
        for (const size_t rule : components[c]) {
            if (!i && rule == ordered_rules[0]) {
                better = true;
                continue;
            }
            if (a[rule] * (1.0 + eps[rule]) < b[rule]) {
                return false;
            }
            if (a[rule] * (1 + eps[rule]) > b[rule]) {
                better = true;
            }
        }
        if (better) {
            recursively_exclude(c, excluded);
        }
    }
    return true;
}

// Returns true if a weakly-rule-dominates b
bool RulebookGraph::dominates(const std::vector<size_t> &a, const std::vector<size_t> &b) const {
    std::vector<bool> excluded(numQRules, false);
    assert(comp_roots[ordered_rules[0]] == ordered_components[0]);
    for (int i=0; i < numQRules; i++) {
        size_t c = ordered_components[i];
        if (excluded[c]) continue;
        bool better = false;
        for (const size_t rule : components[c]) {
            if (a[rule] > b[rule]) {
                return false;
            }
            if (a[rule] < b[rule]) {
                better = true;
            }
        }
        if (better) {
            recursively_exclude(c, excluded);
        }
    }
    return true;
}

// Returns true if a eps-weakly-rule-dominates b
bool RulebookGraph::dominates(const std::vector<size_t> &a, const std::vector<size_t> &b, const std::vector<double> eps) const {
    std::vector<bool> excluded(numQRules, false);
    assert(comp_roots[ordered_rules[0]] == ordered_components[0]);
    for (int i=0; i < numQRules; i++) {
        size_t c = ordered_components[i];
        if (excluded[c]) continue;
        bool better = false;
        for (const size_t rule : components[c]) {
            if (a[rule] > (1.0 + eps[rule]) * b[rule]) {
                return false;
            }
            if (a[rule] < (1.0 + eps[rule]) * b[rule]) {
                better = true;
            }
        }
        if (better) {
            recursively_exclude(c, excluded);
        }
    }
    return true;
}

