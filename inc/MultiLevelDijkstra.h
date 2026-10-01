#pragma once

// Slutsky et al., Iterated Dijkstra Propagation (Algorithms 1 and 2).
// Exact for a strict total order of additive, nonnegative objectives.
// The surviving edge set compactly represents every optimal s-t path.
#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Utils/Definitions.h>

namespace multi_level {

struct Result {
    std::vector<size_t> cost;
    std::vector<std::vector<size_t>> solutions; // unique nondominated cost vectors
    std::vector<size_t> optimal_edges; // indices into the input edge vector
    bool reachable = false;
    bool timed_out = false;
    double seconds = 0;
};

inline Result solve(const std::vector<Edge>& edges, size_t vertex_count,
                    size_t source, size_t target,
                    const std::vector<size_t>& priority_order,
                    double timeout_seconds) {
    using Clock = std::chrono::steady_clock;
    const auto begin = Clock::now();
    const size_t inf = std::numeric_limits<size_t>::max();
    if (source > vertex_count || target > vertex_count || priority_order.empty())
        throw std::invalid_argument("invalid query or empty priority order");
    const size_t n = vertex_count + 1; // DIMACS vertices are 1-based
    const size_t objectives = edges.empty() ? 0 : edges.front().cost.size();
    std::vector<std::vector<size_t>> outgoing(n), incoming(n);
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto& e = edges[i];
        if (e.source >= n || e.target >= n || e.cost.size() != objectives)
            throw std::invalid_argument("edge vertex or objective count mismatch");
        outgoing[e.source].push_back(i);
        incoming[e.target].push_back(i);
    }
    std::vector<char> active(edges.size(), 1);
    auto distance = [&](size_t start, size_t rule, bool reverse) {
        std::vector<size_t> dist(n, inf);
        using Item = std::pair<size_t, size_t>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
        dist[start] = 0;
        queue.push({0, start});
        while (!queue.empty()) {
            const auto [d, v] = queue.top();
            queue.pop();
            if (d != dist[v]) continue;
            for (size_t i : reverse ? incoming[v] : outgoing[v]) {
                if (!active[i]) continue;
                const Edge& e = edges[i];
                const size_t next = reverse ? e.source : e.target;
                if (e.cost[rule] > inf - d) throw std::overflow_error("path cost overflow");
                const size_t candidate = d + e.cost[rule];
                if (candidate < dist[next]) {
                    dist[next] = candidate;
                    queue.push({candidate, next});
                }
            }
        }
        return dist;
    };

    Result result;
    result.cost.assign(objectives, 0);
    result.reachable = true;
    for (size_t rule : priority_order) {
        if (rule >= objectives)
            throw std::invalid_argument("rule id outside the objective vector");
        if (timeout_seconds > 0 &&
            std::chrono::duration<double>(Clock::now() - begin).count() >= timeout_seconds) {
            result.timed_out = true;
            result.reachable = false;
            break;
        }
        auto from_source = distance(source, rule, false);
        auto to_target = distance(target, rule, true);
        const size_t optimum = from_source[target];
        if (optimum == inf) { result.reachable = false; break; }
        result.cost[rule] = optimum;
        for (size_t i = 0; i < edges.size(); ++i) {
            if (!active[i]) continue;
            const auto& e = edges[i];
            if (from_source[e.source] == inf || to_target[e.target] == inf ||
                from_source[e.source] > optimum ||
                e.cost[rule] > optimum - from_source[e.source] ||
                to_target[e.target] > optimum - from_source[e.source] - e.cost[rule] ||
                from_source[e.source] + e.cost[rule] + to_target[e.target] != optimum)
                active[i] = 0;
        }
    }
    if (result.reachable)
        for (size_t i = 0; i < edges.size(); ++i)
            if (active[i]) result.optimal_edges.push_back(i);
    if (result.reachable && priority_order.size() == objectives)
        result.solutions.push_back(result.cost);
    result.seconds = std::chrono::duration<double>(Clock::now() - begin).count();
    return result;
}

// r0 strictly above incomparable r1 and r2: peel r0 with Dijkstra, then
// enumerate the exact (r1,r2) Pareto cost set on the surviving subgraph.
// Equal cost labels at a vertex are collapsed, so paths with identical costs
// are represented by one vector. A finite frontier can still be exponential.
inline Result solve_fork(const std::vector<Edge>& edges, size_t vertex_count,
                         size_t source, size_t target, double timeout_seconds) {
    using Clock = std::chrono::steady_clock;
    const auto begin = Clock::now();
    Result result = solve(edges, vertex_count, source, target, {0}, timeout_seconds);
    if (!result.reachable || result.timed_out) return result;
    if (result.cost.size() != 3)
        throw std::invalid_argument("fork solver requires three objectives");
    std::vector<std::vector<size_t>> outgoing(vertex_count + 1);
    for (size_t i : result.optimal_edges) outgoing[edges[i].source].push_back(i);

    struct Label { size_t vertex, second, third; bool active; };
    std::vector<Label> labels;
    std::vector<std::vector<size_t>> at(vertex_count + 1);
    using QueueItem = std::pair<std::pair<size_t, size_t>, size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    labels.push_back({source, 0, 0, true});
    at[source].push_back(0);
    queue.push({{0, 0}, 0});

    while (!queue.empty()) {
        if (timeout_seconds > 0 &&
            std::chrono::duration<double>(Clock::now() - begin).count() >= timeout_seconds) {
            result.timed_out = true;
            break;
        }
        size_t id = queue.top().second;
        queue.pop();
        if (!labels[id].active) continue;
        const Label current = labels[id];
        if (current.vertex == target) continue;
        for (size_t edge_id : outgoing[current.vertex]) {
            const Edge& e = edges[edge_id];
            if (e.cost[1] > std::numeric_limits<size_t>::max() - current.second ||
                e.cost[2] > std::numeric_limits<size_t>::max() - current.third)
                throw std::overflow_error("Pareto path cost overflow");
            size_t a = current.second + e.cost[1];
            size_t b = current.third + e.cost[2];
            bool dominated = false;
            for (size_t old : at[e.target]) {
                const auto& label = labels[old];
                if (label.active && label.second <= a && label.third <= b) {
                    dominated = true;
                    break;
                }
            }
            if (dominated) continue;
            for (size_t old : at[e.target]) {
                auto& label = labels[old];
                if (label.active && a <= label.second && b <= label.third)
                    label.active = false;
            }
            size_t next = labels.size();
            labels.push_back({e.target, a, b, true});
            at[e.target].push_back(next);
            queue.push({{a, b}, next});
        }
    }
    for (size_t id : at[target]) {
        const auto& label = labels[id];
        if (label.active)
            result.solutions.push_back({result.cost[0], label.second, label.third});
    }
    std::sort(result.solutions.begin(), result.solutions.end());
    result.reachable = !result.solutions.empty();
    result.seconds = std::chrono::duration<double>(Clock::now() - begin).count();
    return result;
}

inline void write_subgraph(const std::string& path, const std::vector<Edge>& edges,
                           const Result& result) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write optimal subgraph: " + path);
    out << "c Edges on all optimal paths; costs follow input objective order\n";
    for (size_t i : result.optimal_edges) {
        const auto& e = edges[i];
        out << "a " << e.source << ' ' << e.target;
        for (size_t cost : e.cost) out << ' ' << cost;
        out << '\n';
    }
}
} // namespace multi_level
