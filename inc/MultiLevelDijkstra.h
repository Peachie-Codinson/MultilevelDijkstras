#pragma once

// Slutsky et al., Iterated Dijkstra Propagation (Algorithms 1 and 2).
// Exact for a strict total order of additive, nonnegative objectives.
// The surviving edge set compactly represents every optimal s-t path.
//
// Each pass computes the optimal subgraph for one rule (Algorithm 1) without
// scanning the whole graph: a forward Dijkstra that stops once distances pass
// the optimum c, then a backward A* guided by those forward distances, which
// only settles vertices with come + go = c and collects the kept edges as it
// goes. Later passes run on the kept edges only.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Utils/Definitions.h>

namespace multi_level {

constexpr size_t INF = std::numeric_limits<size_t>::max();

struct Result {
    std::vector<size_t> cost;
    std::vector<std::vector<size_t>> solutions; // unique nondominated cost vectors
    std::vector<size_t> optimal_edges; // indices into the input edge vector
    // Size of the surviving subgraph after each Dijkstra pass (one entry per rule).
    std::vector<size_t> pass_edges, pass_vertices;
    size_t labels_created = 0, labels_rejected = 0; // fork Pareto search only
    bool reachable = false;
    bool timed_out = false;
    double seconds = 0;
};

// Compressed adjacency over a set of edges. gid maps a local edge id back to
// the input edge, so costs are looked up in the graph's per-rule arrays.
struct Csr {
    size_t n = 0;
    std::vector<size_t> src, dst, gid;
    std::vector<size_t> vid; // local vertex -> input vertex id; empty means identity
    std::vector<size_t> out_start, out_edge, in_start, in_edge;

    size_t global(size_t v) const { return vid.empty() ? v : vid[v]; }

    void build(size_t vertices) {
        n = vertices;
        out_start.assign(n + 1, 0);
        in_start.assign(n + 1, 0);
        for (size_t e = 0; e < src.size(); ++e) { ++out_start[src[e] + 1]; ++in_start[dst[e] + 1]; }
        for (size_t v = 0; v < n; ++v) { out_start[v + 1] += out_start[v]; in_start[v + 1] += in_start[v]; }
        out_edge.resize(src.size());
        in_edge.resize(src.size());
        std::vector<size_t> o(out_start.begin(), out_start.end() - 1), i(in_start.begin(), in_start.end() - 1);
        for (size_t e = 0; e < src.size(); ++e) { out_edge[o[src[e]]++] = e; in_edge[i[dst[e]]++] = e; }
    }
};

// Built once per input graph and shared by every query.
struct Graph {
    size_t vertex_count = 0, objectives = 0;
    std::vector<std::vector<size_t>> cost; // cost[rule][input edge id], contiguous per rule
    Csr csr; // vertex ids are the DIMACS ids (slot 0 unused)

    Graph(const std::vector<Edge>& edges, size_t vertex_count) : vertex_count(vertex_count) {
        objectives = edges.empty() ? 0 : edges.front().cost.size();
        cost.assign(objectives, std::vector<size_t>(edges.size()));
        csr.src.resize(edges.size());
        csr.dst.resize(edges.size());
        csr.gid.resize(edges.size());
        for (size_t i = 0; i < edges.size(); ++i) {
            const auto& e = edges[i];
            if (e.source > vertex_count || e.target > vertex_count || e.cost.size() != objectives)
                throw std::invalid_argument("edge vertex or objective count mismatch");
            csr.src[i] = e.source;
            csr.dst[i] = e.target;
            csr.gid[i] = i;
            for (size_t r = 0; r < objectives; ++r) cost[r][i] = e.cost[r];
        }
        csr.build(vertex_count + 1);
    }
};

// Per-vertex values with O(1) reset, so a query never clears whole-graph arrays.
struct Marks {
    std::vector<size_t> value;
    std::vector<uint32_t> stamp;
    uint32_t epoch = 0;
    void fit(size_t n) {
        if (value.size() < n) { value.resize(n); stamp.resize(n, 0); }
    }
    void reset() {
        if (++epoch == 0) { std::fill(stamp.begin(), stamp.end(), 0); epoch = 1; }
    }
    size_t get(size_t v) const { return stamp[v] == epoch ? value[v] : INF; }
    void set(size_t v, size_t x) { stamp[v] = epoch; value[v] = x; }
};

// Optional record of the search order, for visualisation. All ids are input ids.
struct Trace {
    std::vector<std::vector<size_t>> forward, backward; // per pass: vertices in the order they were settled
    std::vector<std::vector<size_t>> kept;              // per pass: kept edges in the order they were found
    std::vector<size_t> pareto;                         // BOA* expansions, one vertex per expanded label
    std::vector<std::vector<size_t>> paths;             // vertex sequence of each solution found
    std::vector<std::vector<size_t>> path_costs;        // cost vector of each of those paths
    void clear() { forward.clear(); backward.clear(); kept.clear(); pareto.clear(); paths.clear(); path_costs.clear(); }
};

class Solver {
public:
    explicit Solver(const Graph& graph) : g_(graph) {}

    // Record the search order of the following queries into `trace` (nullptr to stop).
    void set_trace(Trace* trace) { trace_ = trace; }

    Result solve(size_t source, size_t target, const std::vector<size_t>& priority_order,
                 double timeout_seconds) {
        const auto begin = Clock::now();
        Csr sub;
        size_t s = source, t = target;
        Result result = run_passes(source, target, priority_order, timeout_seconds, begin, sub, s, t);
        if (result.reachable && priority_order.size() == g_.objectives)
            result.solutions.push_back(result.cost);
        result.seconds = elapsed(begin);
        return result;
    }

    // r0 strictly above incomparable r1 and r2: peel r0 with one pass, then find
    // the exact (r1,r2) Pareto cost set on the surviving subgraph with BOA*
    // (Hernandez et al.): lexicographic best-first search guided by exact r1/r2
    // distances to the goal, pruning a label when its r2 cost is no better than
    // the best r2 already expanded at its vertex or found at the goal. Paths with
    // identical costs are represented by one vector.
    Result solve_fork(size_t source, size_t target, double timeout_seconds) {
        const auto begin = Clock::now();
        Csr sub;
        size_t s = source, t = target;
        Result result = run_passes(source, target, {0}, timeout_seconds, begin, sub, s, t);
        if (!result.reachable || result.timed_out) {
            result.seconds = elapsed(begin);
            return result;
        }
        if (g_.objectives != 3) throw std::invalid_argument("fork solver requires three objectives");
        const auto& w1 = g_.cost[1];
        const auto& w2 = g_.cost[2];
        const auto h1 = distances_to(sub, w1, t), h2 = distances_to(sub, w2, t);

        struct Label { size_t vertex, a, b; };
        std::vector<Label> labels;
        using Item = std::tuple<size_t, size_t, size_t>; // f1, f2, label id
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
        std::vector<size_t> best_b(sub.n, INF); // smallest r2 cost expanded per vertex
        std::vector<size_t> parent; // label -> parent label, kept only while tracing
        labels.push_back({s, 0, 0});
        if (trace_) parent.push_back(INF);
        queue.push({h1[s], h2[s], 0});
        size_t pops = 0;
        while (!queue.empty()) {
            if ((++pops & 4095) == 0 && over(begin, timeout_seconds)) { result.timed_out = true; break; }
            const size_t id = std::get<2>(queue.top());
            queue.pop();
            const Label cur = labels[id];
            if (cur.b >= best_b[cur.vertex] || cur.b + h2[cur.vertex] >= best_b[t]) {
                ++result.labels_rejected;
                continue;
            }
            best_b[cur.vertex] = cur.b;
            if (trace_) trace_->pareto.push_back(sub.global(cur.vertex));
            if (cur.vertex == t) {
                result.solutions.push_back({result.cost[0], cur.a, cur.b});
                if (trace_) {
                    std::vector<size_t> path;
                    for (size_t x = id; x != INF; x = parent[x]) path.push_back(sub.global(labels[x].vertex));
                    trace_->paths.emplace_back(path.rbegin(), path.rend());
                    trace_->path_costs.push_back({result.cost[0], cur.a, cur.b});
                }
                continue;
            }
            for (size_t k = sub.out_start[cur.vertex]; k < sub.out_start[cur.vertex + 1]; ++k) {
                const size_t e = sub.out_edge[k], v = sub.dst[e], ge = sub.gid[e];
                if (h1[v] == INF) continue;
                if (w1[ge] > INF - cur.a - h1[v] || w2[ge] > INF - cur.b - h2[v])
                    throw std::overflow_error("Pareto path cost overflow");
                const size_t a = cur.a + w1[ge], b = cur.b + w2[ge];
                if (b >= best_b[v] || b + h2[v] >= best_b[t]) { ++result.labels_rejected; continue; }
                labels.push_back({v, a, b});
                if (trace_) parent.push_back(id);
                queue.push({a + h1[v], b + h2[v], labels.size() - 1});
            }
        }
        result.labels_created = labels.size();
        std::sort(result.solutions.begin(), result.solutions.end());
        result.reachable = !result.solutions.empty();
        result.seconds = elapsed(begin);
        return result;
    }

private:
    using Clock = std::chrono::steady_clock;
    const Graph& g_;
    Marks come_, go_, done_, remap_;
    Trace* trace_ = nullptr;

    static double elapsed(Clock::time_point begin) {
        return std::chrono::duration<double>(Clock::now() - begin).count();
    }
    static bool over(Clock::time_point begin, double timeout) {
        return timeout > 0 && elapsed(begin) >= timeout;
    }

    // One pass per rule. Leaves the final subgraph in `sub` with s and t
    // renumbered into it.
    Result run_passes(size_t source, size_t target, const std::vector<size_t>& order, double timeout,
                      Clock::time_point begin, Csr& sub, size_t& s, size_t& t) {
        if (source > g_.vertex_count || target > g_.vertex_count || order.empty())
            throw std::invalid_argument("invalid query or empty priority order");
        Result result;
        result.cost.assign(g_.objectives, 0);
        result.reachable = true;
        if (trace_) trace_->clear();
        const Csr* cur = &g_.csr;
        for (size_t rule : order) {
            if (rule >= g_.objectives) throw std::invalid_argument("rule id outside the objective vector");
            if (over(begin, timeout)) { result.timed_out = true; result.reachable = false; break; }
            std::vector<size_t> kept;
            const size_t optimum = optimal_edges(*cur, g_.cost[rule], s, t, kept);
            if (optimum == INF) { result.reachable = false; break; }
            result.cost[rule] = optimum;
            size_t vertices = 0;
            Csr next = induced(*cur, kept, s, t, vertices);
            result.pass_edges.push_back(kept.size());
            result.pass_vertices.push_back(vertices);
            sub = std::move(next);
            cur = &sub;
        }
        if (result.reachable) {
            result.optimal_edges = sub.gid;
            std::sort(result.optimal_edges.begin(), result.optimal_edges.end());
        }
        return result;
    }

    // Algorithm 1 for one rule on graph g: returns the optimum (INF if t is
    // unreachable) and appends the local ids of edges with come(u)+w+go(v) = c.
    size_t optimal_edges(const Csr& g, const std::vector<size_t>& w, size_t s, size_t t,
                         std::vector<size_t>& kept) {
        come_.fit(g.n); go_.fit(g.n); done_.fit(g.n);
        come_.reset(); go_.reset(); done_.reset();
        using Item = std::pair<size_t, size_t>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
        if (trace_) { trace_->forward.emplace_back(); trace_->backward.emplace_back(); trace_->kept.emplace_back(); }

        // Forward Dijkstra, stopped once popped distances exceed c. Afterwards
        // come_ holds exact values for every vertex with come <= c; anything
        // larger is either unset or a tentative value that is also > c.
        size_t c = INF;
        come_.set(s, 0);
        queue.push({0, s});
        while (!queue.empty()) {
            const auto [d, v] = queue.top();
            queue.pop();
            if (d != come_.get(v)) continue;
            if (d > c) break;
            if (trace_) trace_->forward.back().push_back(g.global(v));
            if (v == t) c = d;
            for (size_t k = g.out_start[v]; k < g.out_start[v + 1]; ++k) {
                const size_t e = g.out_edge[k], u = g.dst[e], cost = w[g.gid[e]];
                if (cost > c - d) continue; // cannot lie on a path of cost <= c
                const size_t nd = d + cost;
                if (nd < come_.get(u)) { come_.set(u, nd); queue.push({nd, u}); }
            }
        }
        if (c == INF) return INF;

        // Backward A* from t with h = come (consistent: come(v) <= come(u) + w(u,v)).
        // Every f-value is >= c, so it only settles vertices with come + go = c,
        // i.e. vertices on optimal paths, each with an exact go value.
        queue = {};
        go_.set(t, 0);
        queue.push({c, t});
        while (!queue.empty()) {
            const auto [f, v] = queue.top();
            queue.pop();
            const size_t gv = go_.get(v);
            if (done_.get(v) != INF || gv + come_.get(v) != f) continue;
            done_.set(v, 1);
            if (trace_) trace_->backward.back().push_back(g.global(v));
            for (size_t k = g.in_start[v]; k < g.in_start[v + 1]; ++k) {
                const size_t e = g.in_edge[k], u = g.src[e], cost = w[g.gid[e]];
                const size_t cu = come_.get(u);
                if (cu > c || cost > c - cu || gv > c - cu - cost) continue;
                const size_t ng = gv + cost;
                if (cu + ng == c) {
                    kept.push_back(e);
                    if (trace_) trace_->kept.back().push_back(g.gid[e]);
                }
                if (ng < go_.get(u)) { go_.set(u, ng); queue.push({ng + cu, u}); }
            }
        }
        return c;
    }

    // Subgraph made of the kept edges, with vertices renumbered 0..k-1.
    // `vertices` counts the endpoints of kept edges; s and t always get an id.
    Csr induced(const Csr& g, const std::vector<size_t>& kept, size_t& s, size_t& t, size_t& vertices) {
        remap_.fit(g.n);
        remap_.reset();
        Csr out;
        size_t next = 0;
        auto id = [&](size_t v) {
            size_t x = remap_.get(v);
            if (x == INF) { x = next++; remap_.set(v, x); out.vid.push_back(g.global(v)); }
            return x;
        };
        out.src.reserve(kept.size());
        out.dst.reserve(kept.size());
        out.gid.reserve(kept.size());
        for (size_t e : kept) {
            out.src.push_back(id(g.src[e]));
            out.dst.push_back(id(g.dst[e]));
            out.gid.push_back(g.gid[e]);
        }
        vertices = next;
        s = id(s);
        t = id(t);
        out.build(next);
        return out;
    }

    // Exact distances to t over g (plain backward Dijkstra on a small subgraph).
    static std::vector<size_t> distances_to(const Csr& g, const std::vector<size_t>& w, size_t t) {
        std::vector<size_t> d(g.n, INF);
        using Item = std::pair<size_t, size_t>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
        d[t] = 0;
        queue.push({0, t});
        while (!queue.empty()) {
            const auto [dv, v] = queue.top();
            queue.pop();
            if (dv != d[v]) continue;
            for (size_t k = g.in_start[v]; k < g.in_start[v + 1]; ++k) {
                const size_t e = g.in_edge[k], u = g.src[e], cost = w[g.gid[e]];
                if (cost > INF - dv) throw std::overflow_error("path cost overflow");
                if (dv + cost < d[u]) { d[u] = dv + cost; queue.push({d[u], u}); }
            }
        }
        return d;
    }
};

// One-off helpers that build the graph for a single query (tests, scripts).
inline Result solve(const std::vector<Edge>& edges, size_t vertex_count, size_t source, size_t target,
                    const std::vector<size_t>& priority_order, double timeout_seconds) {
    Graph graph(edges, vertex_count);
    return Solver(graph).solve(source, target, priority_order, timeout_seconds);
}

inline Result solve_fork(const std::vector<Edge>& edges, size_t vertex_count, size_t source,
                         size_t target, double timeout_seconds) {
    Graph graph(edges, vertex_count);
    return Solver(graph).solve_fork(source, target, timeout_seconds);
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
