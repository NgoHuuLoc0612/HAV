/**
 * HAV CLI - Command-line interface for the C++ engine
 * Run maze generation, pathfinding, sorting, graph benchmarks
 */

#include "hav_engine.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <sstream>
#include <numeric>
#include <algorithm>
#include <random>

using namespace hav;
using Clock = std::chrono::high_resolution_clock;

// ─── Utilities ────────────────────────────────────────────────────────────────

static double elapsed_ms(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

static void printHeader(const std::string& title) {
    std::cout << "\n+--------------------------------------+\n";
    std::cout << "+  " << std::left << std::setw(36) << title << "+\n";
    std::cout << "+--------------------------------------+\n";
}

static void printSep() { std::cout << "  " << std::string(40, '-') << '\n'; }

// ─── Maze benchmark ───────────────────────────────────────────────────────────

void benchMaze(int rows, int cols) {
    printHeader("MAZE GENERATION BENCHMARK");

    struct Config { GrowingTreeStrategy s; const char* name; };
    std::vector<Config> configs = {
        {GrowingTreeStrategy::NEWEST, "Growing-Tree NEWEST (DFS-like)"},
        {GrowingTreeStrategy::RANDOM, "Growing-Tree RANDOM (Prim-like)"},
        {GrowingTreeStrategy::MIX,    "Growing-Tree MIX (hybrid 0.5)"},
        {GrowingTreeStrategy::MIDDLE, "Growing-Tree MIDDLE"},
        {GrowingTreeStrategy::OLDEST, "Growing-Tree OLDEST (BFS-like)"},
    };

    for (auto& cfg : configs) {
        MazeEngine eng(rows, cols);
        auto t = Clock::now();
        eng.generateGrowingTree(cfg.s, 0.5);
        double ms = elapsed_ms(t);
        std::cout << "  " << std::left << std::setw(38) << cfg.name
                  << std::right << std::setw(8) << std::fixed << std::setprecision(2)
                  << ms << " ms  |  " << eng.steps().size() << " steps\n";
    }

    printSep();
    { // Recursive Division
        MazeEngine eng(rows, cols);
        auto t = Clock::now();
        eng.generateRecursiveDivision();
        double ms = elapsed_ms(t);
        std::cout << "  " << std::left << std::setw(38) << "Recursive Division"
                  << std::right << std::setw(8) << ms << " ms  |  " << eng.steps().size() << " steps\n";
    }
    { // Wilson's
        MazeEngine eng(rows, cols);
        auto t = Clock::now();
        eng.generateWilson();
        double ms = elapsed_ms(t);
        std::cout << "  " << std::left << std::setw(38) << "Wilson's Algorithm"
                  << std::right << std::setw(8) << ms << " ms  |  " << eng.steps().size() << " steps\n";
    }
}

// ─── Path benchmark ───────────────────────────────────────────────────────────

void benchPath(int size) {
    printHeader("PATHFINDING BENCHMARK");

    // Build simple open grid
    std::vector<std::vector<int>> grid(size, std::vector<int>(size, CELL_EMPTY));
    // Add some walls
    for (int r = 5; r < size-5; r += 4)
        for (int c = 0; c < size-3; ++c)
            if (c != size/3 && c != 2*size/3) grid[r][c] = CELL_WALL;

    PathEngine eng(grid);
    int er = size-2, ec = size-2;

    struct Config { const char* name; std::function<PathResult()> fn; };
    std::vector<Config> configs = {
        {"A* Manhattan",     [&](){ return eng.aStar(1,1,er,ec,HeuristicType::MANHATTAN,false); }},
        {"A* Euclidean",     [&](){ return eng.aStar(1,1,er,ec,HeuristicType::EUCLIDEAN,false); }},
        {"A* Octile +diag",  [&](){ return eng.aStar(1,1,er,ec,HeuristicType::OCTILE,true); }},
        {"Dijkstra",         [&](){ return eng.dijkstraImpl(1,1,er,ec); }},
        {"BFS",              [&](){ return eng.bfs(1,1,er,ec); }},
        {"DFS",              [&](){ return eng.dfs(1,1,er,ec); }},
        {"Bidirectional A*", [&](){ return eng.bidirectionalAStar(1,1,er,ec); }},
    };

    for (auto& cfg : configs) {
        auto t = Clock::now();
        auto res = cfg.fn();
        double ms = elapsed_ms(t);
        std::cout << "  " << std::left << std::setw(22) << cfg.name
                  << "  found=" << (res.found?"YES":"NO ")
                  << "  len=" << std::setw(4) << res.path.size()
                  << "  steps=" << std::setw(6) << res.steps.size()
                  << "  " << std::right << std::setw(8) << ms << " ms\n";
    }
}

// ─── Sort benchmark ───────────────────────────────────────────────────────────

void benchSort(int n) {
    printHeader("SORTING BENCHMARK");

    // random data
    std::vector<int> base(n);
    std::iota(base.begin(), base.end(), 1);
    std::shuffle(base.begin(), base.end(), std::mt19937(42));

    struct Config { const char* name; std::function<void(SortEngine&)> fn; };
    std::vector<Config> configs = {
        {"Bubble Sort",    [](SortEngine& e){ e.runBubble(); }},
        {"Insertion Sort", [](SortEngine& e){ e.runInsertion(); }},
        {"Selection Sort", [](SortEngine& e){ e.runSelection(); }},
        {"Shell Sort",     [](SortEngine& e){ e.runShell(); }},
        {"Merge Sort",     [](SortEngine& e){ e.runMerge(); }},
        {"Heap Sort",      [](SortEngine& e){ e.runHeap(); }},
        {"Quick Sort",     [](SortEngine& e){ e.runQuick(); }},
        {"Tim Sort",       [](SortEngine& e){ e.runTimSort(); }},
        {"Radix Sort",     [](SortEngine& e){ e.runRadix(); }},
        {"Counting Sort",  [](SortEngine& e){ e.runCounting(); }},
    };

    for (auto& cfg : configs) {
        SortEngine eng(base);
        auto t = Clock::now();
        cfg.fn(eng);
        double ms = elapsed_ms(t);
        std::cout << "  " << std::left << std::setw(16) << cfg.name
                  << "  cmp=" << std::setw(9) << eng.comparisons()
                  << "  swp=" << std::setw(7) << eng.swaps()
                  << "  steps=" << std::setw(8) << eng.steps().size()
                  << "  " << std::right << std::setw(8) << ms << " ms\n";
    }
}

// ─── Graph benchmark ──────────────────────────────────────────────────────────

void benchGraph(int n) {
    printHeader("GRAPH ALGORITHMS BENCHMARK");

    // Build random connected graph
    GraphEngine eng(n);
    for (int u = 1; u < n; ++u) {
        int v = std::uniform_int_distribution<int>(0, u-1)(std::mt19937(u));
        eng.addEdge(u, v, std::uniform_real_distribution<>(1,10)(std::mt19937(u*17)));
    }
    // Extra edges
    for (int i = 0; i < n; ++i) {
        int u = i, v = (i + n/3) % n;
        if (u != v) eng.addEdge(u, v, 5.0);
    }

    auto bench = [&](const char* name, auto fn) {
        auto t = Clock::now();
        auto res = fn();
        double ms = elapsed_ms(t);
        std::cout << "  " << std::left << std::setw(20) << name
                  << "  steps=" << std::setw(6) << res.steps.size()
                  << "  " << std::right << std::setw(8) << ms << " ms\n";
    };

    bench("Kruskal MST",        [&](){ return eng.kruskal(); });
    bench("Prim MST",           [&](){ return eng.prim(0); });
    bench("Bellman-Ford",       [&](){ return eng.bellmanFord(0); });
    bench("Floyd-Warshall",     [&](){ return eng.floydWarshall(); });
    bench("Tarjan SCC",         [&](){ return eng.tarjanSCC(); });
    bench("Topological Sort",   [&](){ return eng.topologicalSort(); });
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::cout << "\n  HAV Engine v2.0 — C++ Core Benchmark\n";
    std::cout << "  ======================================\n";

    bool all = argc < 2;
    std::string cmd = argc >= 2 ? argv[1] : "all";

    int mazeR=25, mazeC=41, pathSz=60, sortN=500, graphN=20;
    if (all || cmd=="maze")  benchMaze(mazeR, mazeC);
    if (all || cmd=="path")  benchPath(pathSz);
    if (all || cmd=="sort")  benchSort(sortN);
    if (all || cmd=="graph") benchGraph(graphN);

    std::cout << "\n  ✓ All benchmarks complete.\n\n";
    return 0;
}