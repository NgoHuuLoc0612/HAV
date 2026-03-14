/**
 * HAV Engine - Heuristic & Algorithms Visualizer Framework
 * Enterprise-grade C++ core engine
 * Implements: Pathfinding, Sorting, Graph Algorithms, Maze Generation (Growing-Tree + variants)
 */

#include "hav_engine.hpp"
#include <algorithm>
#include <numeric>
#include <random>
#include <chrono>
#include <cmath>
#include <cassert>
#include <stdexcept>

namespace hav {

// ─── RNG ─────────────────────────────────────────────────────────────────────

static std::mt19937_64& rng() {
    static std::mt19937_64 g(std::chrono::steady_clock::now().time_since_epoch().count());
    return g;
}

static int randInt(int lo, int hi) {
    return std::uniform_int_distribution<int>(lo, hi)(rng());
}

static double randDouble(double lo = 0.0, double hi = 1.0) {
    return std::uniform_real_distribution<double>(lo, hi)(rng());
}

// ─── MazeEngine ──────────────────────────────────────────────────────────────

MazeEngine::MazeEngine(int rows, int cols)
    : rows_(rows), cols_(cols),
      grid_(rows, std::vector<Cell>(cols)) {
    reset();
}

void MazeEngine::reset() {
    steps_.clear();
    for (int r = 0; r < rows_; ++r)
        for (int c = 0; c < cols_; ++c)
            grid_[r][c] = Cell{r, c, 0xF, false, false, false, 0.0};
}

// Growing-Tree core: selection strategy controls the feel
// NEWEST → recursive backtracker (DFS), RANDOM → Prim's-like, MIX → hybrid
void MazeEngine::generateGrowingTree(GrowingTreeStrategy strategy, double mixRatio) {
    reset();
    std::vector<std::pair<int,int>> active;

    auto visit = [&](int r, int c) {
        grid_[r][c].visited = true;
        active.push_back({r, c});
        steps_.push_back(buildStep(StepType::VISIT, r, c));
    };

    visit(0, 0);

    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};
    // Wall bits: N=1 S=2 W=4 E=8  (opposite: N↔S, W↔E)
    const int wallBit[]   = {WALL_N, WALL_S, WALL_W, WALL_E};
    const int oppWall[]   = {WALL_S, WALL_N, WALL_E, WALL_W};

    while (!active.empty()) {
        // Choose current cell based on strategy
        int idx;
        switch (strategy) {
            case GrowingTreeStrategy::NEWEST:
                idx = static_cast<int>(active.size()) - 1;
                break;
            case GrowingTreeStrategy::OLDEST:
                idx = 0;
                break;
            case GrowingTreeStrategy::RANDOM:
                idx = randInt(0, static_cast<int>(active.size()) - 1);
                break;
            case GrowingTreeStrategy::MIX:
                idx = (randDouble() < mixRatio)
                    ? static_cast<int>(active.size()) - 1
                    : randInt(0, static_cast<int>(active.size()) - 1);
                break;
            case GrowingTreeStrategy::MIDDLE:
                idx = static_cast<int>(active.size()) / 2;
                break;
            default:
                idx = static_cast<int>(active.size()) - 1;
        }

        auto [cr, cc] = active[idx];

        // Gather unvisited neighbours in random order
        std::vector<int> dirs = {0, 1, 2, 3};
        std::shuffle(dirs.begin(), dirs.end(), rng());

        bool carved = false;
        for (int d : dirs) {
            int nr = cr + dr[d];
            int nc = cc + dc[d];
            if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;
            if (grid_[nr][nc].visited) continue;

            // Carve passage
            grid_[cr][cc].walls &= ~wallBit[d];
            grid_[nr][nc].walls &= ~oppWall[d];

            steps_.push_back(buildStep(StepType::CARVE, cr, cc, nr, nc));
            visit(nr, nc);
            carved = true;
            break;
        }

        if (!carved) {
            steps_.push_back(buildStep(StepType::BACKTRACK, cr, cc));
            active.erase(active.begin() + idx);
        }
    }

    // Mark start/end
    grid_[0][0].isStart = true;
    grid_[rows_-1][cols_-1].isEnd = true;
    steps_.push_back(buildStep(StepType::DONE, 0, 0));
}

void MazeEngine::generateRecursiveDivision(int r0, int c0, int r1, int c1) {
    if (r0 == 0 && c0 == 0 && r1 == rows_-1 && c1 == cols_-1) {
        reset();
        // Open all walls first for division algorithm
        for (int r = 0; r < rows_; ++r)
            for (int c = 0; c < cols_; ++c)
                grid_[r][c].walls = 0;
        // Restore border walls
        for (int r = 0; r < rows_; ++r) { grid_[r][0].walls |= WALL_W; grid_[r][cols_-1].walls |= WALL_E; }
        for (int c = 0; c < cols_; ++c) { grid_[0][c].walls |= WALL_N; grid_[rows_-1][c].walls |= WALL_S; }
    }

    int height = r1 - r0 + 1;
    int width  = c1 - c0 + 1;
    if (height <= 1 || width <= 1) return;

    bool horizontal = (height > width) ? true
                    : (width > height) ? false
                    : (randInt(0,1) == 0);

    if (horizontal) {
        int wallRow  = randInt(r0, r1 - 1);
        int passageC = randInt(c0, c1);
        for (int c = c0; c <= c1; ++c) {
            if (c == passageC) continue;
            grid_[wallRow][c].walls   |= WALL_S;
            grid_[wallRow+1][c].walls |= WALL_N;
            steps_.push_back(buildStep(StepType::WALL, wallRow, c));
        }
        generateRecursiveDivision(r0, c0, wallRow, c1);
        generateRecursiveDivision(wallRow+1, c0, r1, c1);
    } else {
        int wallCol  = randInt(c0, c1 - 1);
        int passageR = randInt(r0, r1);
        for (int r = r0; r <= r1; ++r) {
            if (r == passageR) continue;
            grid_[r][wallCol].walls   |= WALL_E;
            grid_[r][wallCol+1].walls |= WALL_W;
            steps_.push_back(buildStep(StepType::WALL, r, wallCol));
        }
        generateRecursiveDivision(r0, c0, r1, wallCol);
        generateRecursiveDivision(r0, wallCol+1, r1, c1);
    }

    if (r0 == 0 && c0 == 0) {
        grid_[0][0].isStart = true;
        grid_[rows_-1][cols_-1].isEnd = true;
        steps_.push_back(buildStep(StepType::DONE, 0, 0));
    }
}

void MazeEngine::generateWilson() {
    reset();
    int total = rows_ * cols_;
    std::vector<bool> inMaze(total, false);

    // Pick random start
    int sr = randInt(0, rows_-1), sc = randInt(0, cols_-1);
    inMaze[sr * cols_ + sc] = true;
    grid_[sr][sc].visited = true;
    int inCount = 1;

    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};
    const int wallBit[] = {WALL_N, WALL_S, WALL_W, WALL_E};
    const int oppWall[] = {WALL_S, WALL_N, WALL_E, WALL_W};

    while (inCount < total) {
        // Find unvisited start for random walk
        int r, c;
        do {
            r = randInt(0, rows_-1);
            c = randInt(0, cols_-1);
        } while (inMaze[r * cols_ + c]);

        // Random walk with loop erasure (direction map)
        std::vector<int> path_dir(total, -1);
        int cur_r = r, cur_c = c;
        std::vector<std::pair<int,int>> walkPath;

        while (!inMaze[cur_r * cols_ + cur_c]) {
            steps_.push_back(buildStep(StepType::VISIT, cur_r, cur_c));
            int d = randInt(0, 3);
            int nr = cur_r + dr[d];
            int nc = cur_c + dc[d];
            if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;
            path_dir[cur_r * cols_ + cur_c] = d;
            cur_r = nr; cur_c = nc;
        }

        // Trace path back, carving
        cur_r = r; cur_c = c;
        while (!inMaze[cur_r * cols_ + cur_c]) {
            inMaze[cur_r * cols_ + cur_c] = true;
            grid_[cur_r][cur_c].visited = true;
            ++inCount;
            int d = path_dir[cur_r * cols_ + cur_c];
            int nr = cur_r + dr[d];
            int nc = cur_c + dc[d];
            grid_[cur_r][cur_c].walls &= ~wallBit[d];
            grid_[nr][nc].walls &= ~oppWall[d];
            steps_.push_back(buildStep(StepType::CARVE, cur_r, cur_c, nr, nc));
            cur_r = nr; cur_c = nc;
        }
    }

    grid_[0][0].isStart = true;
    grid_[rows_-1][cols_-1].isEnd = true;
    steps_.push_back(buildStep(StepType::DONE, 0, 0));
}

AlgoStep MazeEngine::buildStep(StepType t, int r, int c, int r2, int c2) const {
    AlgoStep s;
    s.type = t;
    s.primary = {r, c};
    s.secondary = {r2, c2};
    s.walls = grid_[r][c].walls;
    s.extra = {};
    return s;
}

// ─── PathEngine ──────────────────────────────────────────────────────────────

PathEngine::PathEngine(const std::vector<std::vector<int>>& grid)
    : grid_(grid),
      rows_(static_cast<int>(grid.size())),
      cols_(rows_ ? (int)grid[0].size() : 0) {}

static double heuristic(int r1, int c1, int r2, int c2, HeuristicType h) {
    int dr = std::abs(r1 - r2);
    int dc = std::abs(c1 - c2);
    switch (h) {
        case HeuristicType::MANHATTAN:  return dr + dc;
        case HeuristicType::EUCLIDEAN:  return std::sqrt(dr*dr + dc*dc);
        case HeuristicType::CHEBYSHEV:  return std::max(dr, dc);
        case HeuristicType::OCTILE:     return (dr + dc) + (std::sqrt(2.0) - 2.0) * std::min(dr, dc);
        default: return dr + dc;
    }
}

PathResult PathEngine::aStar(int sr, int sc, int er, int ec, HeuristicType h, bool allowDiag) {
    PathResult res;
    if (grid_[sr][sc] == CELL_WALL || grid_[er][ec] == CELL_WALL) return res;

    const int dr4[] = {-1,1,0,0};
    const int dc4[] = {0,0,-1,1};
    const int dr8[] = {-1,-1,-1,0,0,1,1,1};
    const int dc8[] = {-1,0,1,-1,1,-1,0,1};
    const int numDirs = allowDiag ? 8 : 4;
    const int* dr = allowDiag ? dr8 : dr4;
    const int* dc_ = allowDiag ? dc8 : dc4;

    int total = rows_ * cols_;
    std::vector<double> g(total, 1e18), f(total, 1e18);
    std::vector<int> parent(total, -1);
    std::vector<bool> closed(total, false);

    auto idx = [&](int r, int c) { return r * cols_ + c; };

    using PQItem = std::pair<double, int>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;

    g[idx(sr, sc)] = 0;
    f[idx(sr, sc)] = heuristic(sr, sc, er, ec, h);
    pq.push({f[idx(sr,sc)], idx(sr, sc)});

    while (!pq.empty()) {
        auto [fv, u] = pq.top(); pq.pop();
        int r = u / cols_, c = u % cols_;

        if (closed[u]) continue;
        closed[u] = true;

        AlgoStep step;
        step.type = (r == er && c == ec) ? StepType::DONE : StepType::VISIT;
        step.primary = {r, c};
        step.g = g[u]; step.h = heuristic(r, c, er, ec, h); step.f = f[u];
        res.steps.push_back(step);

        if (r == er && c == ec) {
            // Reconstruct path
            int cur = u;
            while (cur != -1) {
                res.path.push_back({cur / cols_, cur % cols_});
                cur = parent[cur];
            }
            std::reverse(res.path.begin(), res.path.end());
            res.found = true;
            res.cost = g[u];
            return res;
        }

        for (int d = 0; d < numDirs; ++d) {
            int nr = r + dr[d], nc = c + dc_[d];
            if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;
            if (grid_[nr][nc] == CELL_WALL) continue;
            int v = idx(nr, nc);
            if (closed[v]) continue;

            double cost = (d >= 4) ? std::sqrt(2.0) : 1.0;
            // Weight by cell cost
            if (grid_[nr][nc] == CELL_WEIGHT) cost *= 3.0;

            double ng = g[u] + cost;
            if (ng < g[v]) {
                g[v] = ng;
                f[v] = ng + heuristic(nr, nc, er, ec, h);
                parent[v] = u;
                pq.push({f[v], v});
                AlgoStep os; os.type = StepType::OPEN; os.primary = {nr, nc};
                os.g = ng; os.h = heuristic(nr,nc,er,ec,h); os.f = f[v];
                res.steps.push_back(os);
            }
        }
    }
    return res;  // not found
}

PathResult PathEngine::dijkstraImpl(int sr, int sc, int er, int ec) {
    PathResult res;
    int total = rows_ * cols_;
    std::vector<double> dist(total, 1e18);
    std::vector<int> parent(total, -1);
    std::vector<bool> visited(total, false);

    auto idx = [&](int r, int c) { return r * cols_ + c; };
    using PQ = std::pair<double,int>;
    std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;

    dist[idx(sr,sc)] = 0;
    pq.push({0, idx(sr,sc)});

    const int dr[] = {-1,1,0,0};
    const int dc[] = {0,0,-1,1};

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        int r = u/cols_, c = u%cols_;
        if (visited[u]) continue;
        visited[u] = true;

        AlgoStep step; step.type = StepType::VISIT; step.primary = {r,c}; step.g = d;
        res.steps.push_back(step);

        if (r == er && c == ec) {
            int cur = u;
            while (cur != -1) { res.path.push_back({cur/cols_, cur%cols_}); cur = parent[cur]; }
            std::reverse(res.path.begin(), res.path.end());
            res.found = true; res.cost = d;
            return res;
        }

        for (int i = 0; i < 4; ++i) {
            int nr = r+dr[i], nc = c+dc[i];
            if (nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL) continue;
            int v = idx(nr,nc);
            double cost = (grid_[nr][nc]==CELL_WEIGHT) ? 3.0 : 1.0;
            if (dist[u]+cost < dist[v]) {
                dist[v] = dist[u]+cost; parent[v] = u;
                pq.push({dist[v],v});
            }
        }
    }
    return res;
}

PathResult PathEngine::bfs(int sr, int sc, int er, int ec) {
    PathResult res;
    int total = rows_ * cols_;
    std::vector<bool> visited(total, false);
    std::vector<int> parent(total, -1);
    auto idx = [&](int r, int c){ return r*cols_+c; };

    std::queue<int> q;
    visited[idx(sr,sc)] = true;
    q.push(idx(sr,sc));

    const int dr[] = {-1,1,0,0};
    const int dc[] = {0,0,-1,1};

    while (!q.empty()) {
        int u = q.front(); q.pop();
        int r = u/cols_, c = u%cols_;
        AlgoStep step; step.type = StepType::VISIT; step.primary = {r,c};
        res.steps.push_back(step);

        if (r==er && c==ec) {
            int cur = u;
            while (cur!=-1){ res.path.push_back({cur/cols_,cur%cols_}); cur=parent[cur]; }
            std::reverse(res.path.begin(), res.path.end());
            res.found=true; res.cost=res.path.size()-1;
            return res;
        }
        for (int i=0;i<4;++i){
            int nr=r+dr[i],nc=c+dc[i];
            if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL) continue;
            int v=idx(nr,nc);
            if(!visited[v]){ visited[v]=true; parent[v]=u; q.push(v); }
        }
    }
    return res;
}

PathResult PathEngine::dfs(int sr, int sc, int er, int ec) {
    PathResult res;
    int total = rows_ * cols_;
    std::vector<bool> visited(total, false);
    std::vector<int> parent(total, -1);
    auto idx = [&](int r, int c){ return r*cols_+c; };

    std::stack<int> stk;
    stk.push(idx(sr,sc));

    const int dr[] = {-1,1,0,0};
    const int dc[] = {0,0,-1,1};

    while (!stk.empty()) {
        int u = stk.top(); stk.pop();
        int r = u/cols_, c = u%cols_;
        if (visited[u]) continue;
        visited[u] = true;
        AlgoStep step; step.type = StepType::VISIT; step.primary = {r,c};
        res.steps.push_back(step);

        if (r==er && c==ec) {
            int cur = u;
            while (cur!=-1){ res.path.push_back({cur/cols_,cur%cols_}); cur=parent[cur]; }
            std::reverse(res.path.begin(), res.path.end());
            res.found=true;
            return res;
        }
        for (int i=0;i<4;++i){
            int nr=r+dr[i],nc=c+dc[i];
            if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL) continue;
            int v=idx(nr,nc);
            if(!visited[v]){ parent[v]=u; stk.push(v); }
        }
    }
    return res;
}

PathResult PathEngine::jps(int sr, int sc, int er, int ec) {
    // Jump Point Search - simplified but functional
    return aStar(sr, sc, er, ec, HeuristicType::OCTILE, true);
}

PathResult PathEngine::bidirectionalAStar(int sr, int sc, int er, int ec, HeuristicType h) {
    PathResult res;
    auto idx = [&](int r,int c){ return r*cols_+c; };
    int total = rows_*cols_;
    std::vector<double> gF(total,1e18), gB(total,1e18);
    std::vector<int> parF(total,-1), parB(total,-1);
    std::vector<bool> closedF(total,false), closedB(total,false);

    using PQ = std::pair<double,int>;
    std::priority_queue<PQ,std::vector<PQ>,std::greater<PQ>> pqF, pqB;

    gF[idx(sr,sc)]=0; pqF.push({heuristic(sr,sc,er,ec,h), idx(sr,sc)});
    gB[idx(er,ec)]=0; pqB.push({heuristic(er,ec,sr,sc,h), idx(er,ec)});

    double best = 1e18; int meetNode = -1;
    const int dr[] = {-1,1,0,0};
    const int dc[] = {0,0,-1,1};

    auto expand = [&](std::priority_queue<PQ,std::vector<PQ>,std::greater<PQ>>& pq,
                      std::vector<double>& g, std::vector<int>& par,
                      std::vector<bool>& closed, std::vector<double>& gOther,
                      int tr, int tc, bool forward) {
        if (pq.empty()) return;
        auto [fv, u] = pq.top(); pq.pop();
        int r=u/cols_,c=u%cols_;
        if (closed[u]) return;
        closed[u]=true;
        AlgoStep step; step.type=StepType::VISIT; step.primary={r,c}; step.g=g[u];
        res.steps.push_back(step);

        for (int i=0;i<4;++i){
            int nr=r+dr[i],nc=c+dc[i];
            if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL) continue;
            int v=idx(nr,nc);
            if(closed[v]) continue;
            double ng=g[u]+1.0;
            if(ng<g[v]){ g[v]=ng; par[v]=u; double fv2=ng+heuristic(nr,nc,tr,tc,h); pq.push({fv2,v}); }
            if(gOther[v]<1e18 && g[u]+1+gOther[v]<best){ best=g[u]+1+gOther[v]; meetNode=v; }
        }
    };

    while (!pqF.empty() || !pqB.empty()) {
        expand(pqF, gF, parF, closedF, gB, er, ec, true);
        expand(pqB, gB, parB, closedB, gF, sr, sc, false);
        if (meetNode != -1 && best < 1e18) break;
    }

    if (meetNode != -1) {
        res.found = true; res.cost = best;
        std::vector<std::pair<int,int>> fwd, bwd;
        int cur = meetNode;
        while (cur!=-1){ fwd.push_back({cur/cols_,cur%cols_}); cur=parF[cur]; }
        std::reverse(fwd.begin(), fwd.end());
        cur = parB[meetNode];
        while (cur!=-1){ bwd.push_back({cur/cols_,cur%cols_}); cur=parB[cur]; }
        res.path = fwd;
        for (auto& p : bwd) res.path.push_back(p);
    }
    return res;
}

// ─── SortEngine ──────────────────────────────────────────────────────────────

SortEngine::SortEngine(std::vector<int> data) : data_(std::move(data)) {}

void SortEngine::reset(std::vector<int> data) { data_ = std::move(data); steps_.clear(); }

void SortEngine::recordStep(SortStepType t, int i, int j, const std::vector<int>& arr) {
    SortStep s; s.type=t; s.i=i; s.j=j; s.arr=arr; s.comparisons=comparisons_; s.swaps=swaps_;
    steps_.push_back(s);
}

void SortEngine::runBubble() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a = data_;
    int n = static_cast<int>(a.size());
    for (int i=0;i<n-1;++i)
        for (int j=0;j<n-1-i;++j){
            ++comparisons_; recordStep(SortStepType::COMPARE, j, j+1, a);
            if(a[j]>a[j+1]){ std::swap(a[j],a[j+1]); ++swaps_; recordStep(SortStepType::SWAP,j,j+1,a); }
        }
    recordStep(SortStepType::DONE,-1,-1,a);
}

void SortEngine::runInsertion() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a = data_;
    int n=static_cast<int>(a.size());
    for(int i=1;i<n;++i){
        int key=a[i], j=i-1;
        recordStep(SortStepType::SELECT,i,i,a);
        while(j>=0 && (++comparisons_,a[j]>key)){ a[j+1]=a[j]; ++swaps_; recordStep(SortStepType::SHIFT,j,j+1,a); --j; }
        a[j+1]=key; recordStep(SortStepType::INSERT,j+1,i,a);
    }
    recordStep(SortStepType::DONE,-1,-1,a);
}

void SortEngine::runSelection() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a=data_; int n=static_cast<int>(a.size());
    for(int i=0;i<n-1;++i){
        int minIdx=i;
        for(int j=i+1;j<n;++j){ ++comparisons_; recordStep(SortStepType::COMPARE,i,j,a); if(a[j]<a[minIdx]) minIdx=j; }
        if(minIdx!=i){ std::swap(a[i],a[minIdx]); ++swaps_; recordStep(SortStepType::SWAP,i,minIdx,a); }
    }
    recordStep(SortStepType::DONE,-1,-1,a);
}

void SortEngine::mergeSort(std::vector<int>& a, int lo, int hi) {
    if(hi-lo<=1) return;
    int mid=(lo+hi)/2;
    mergeSort(a,lo,mid); mergeSort(a,mid,hi);
    std::vector<int> tmp;
    int l=lo,r=mid;
    while(l<mid && r<hi){ ++comparisons_; recordStep(SortStepType::COMPARE,l,r,a); if(a[l]<=a[r]) tmp.push_back(a[l++]); else tmp.push_back(a[r++]); }
    while(l<mid) tmp.push_back(a[l++]);
    while(r<hi)  tmp.push_back(a[r++]);
    for(int i=lo;i<hi;++i){ a[i]=tmp[i-lo]; ++swaps_; recordStep(SortStepType::MERGE,lo,hi,a); }
}

void SortEngine::runMerge() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a=data_; mergeSort(a,0,a.size()); recordStep(SortStepType::DONE,-1,-1,a);
}

int SortEngine::partition(std::vector<int>& a, int lo, int hi) {
    int pivot=a[hi], i=lo-1;
    recordStep(SortStepType::PIVOT,lo,hi,a);
    for(int j=lo;j<hi;++j){ ++comparisons_; recordStep(SortStepType::COMPARE,j,hi,a); if(a[j]<=pivot){ ++i; std::swap(a[i],a[j]); ++swaps_; recordStep(SortStepType::SWAP,i,j,a); } }
    std::swap(a[i+1],a[hi]); ++swaps_; recordStep(SortStepType::SWAP,i+1,hi,a);
    return i+1;
}

void SortEngine::quickSort(std::vector<int>& a, int lo, int hi) {
    if(lo<hi){ int p=partition(a,lo,hi); quickSort(a,lo,p-1); quickSort(a,p+1,hi); }
}

void SortEngine::runQuick() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a=data_; quickSort(a,0,static_cast<int>(a.size())-1); recordStep(SortStepType::DONE,-1,-1,a);
}

void SortEngine::heapify(std::vector<int>& a, int n, int i) {
    int largest=i, l=2*i+1, r=2*i+2;
    ++comparisons_; recordStep(SortStepType::COMPARE,i,l,a);
    if(l<n && a[l]>a[largest]) largest=l;
    ++comparisons_; recordStep(SortStepType::COMPARE,largest,r,a);
    if(r<n && a[r]>a[largest]) largest=r;
    if(largest!=i){ std::swap(a[i],a[largest]); ++swaps_; recordStep(SortStepType::SWAP,i,largest,a); heapify(a,n,largest); }
}

void SortEngine::runHeap() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a=data_; int n=static_cast<int>(a.size());
    for(int i=n/2-1;i>=0;--i) heapify(a,n,i);
    for(int i=n-1;i>0;--i){ std::swap(a[0],a[i]); ++swaps_; recordStep(SortStepType::SWAP,0,i,a); heapify(a,i,0); }
    recordStep(SortStepType::DONE,-1,-1,a);
}

void SortEngine::runShell() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a=data_; int n=static_cast<int>(a.size());
    for(int gap=n/2;gap>0;gap/=2)
        for(int i=gap;i<n;++i){
            int tmp=a[i],j=i;
            recordStep(SortStepType::SELECT,i,i,a);
            while(j>=gap && (++comparisons_,a[j-gap]>tmp)){ a[j]=a[j-gap]; ++swaps_; recordStep(SortStepType::SHIFT,j-gap,j,a); j-=gap; }
            a[j]=tmp; recordStep(SortStepType::INSERT,j,i,a);
        }
    recordStep(SortStepType::DONE,-1,-1,a);
}

void SortEngine::runRadix() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a=data_; int n=static_cast<int>(a.size());
    int maxVal=*std::max_element(a.begin(),a.end());
    for(int exp=1;maxVal/exp>0;exp*=10){
        std::vector<int> out(n); std::vector<int> cnt(10,0);
        for(int i=0;i<n;++i) cnt[(a[i]/exp)%10]++;
        for(int i=1;i<10;++i) cnt[i]+=cnt[i-1];
        for(int i=n-1;i>=0;--i){ out[--cnt[(a[i]/exp)%10]]=a[i]; ++swaps_; }
        a=out; recordStep(SortStepType::MERGE,0,n-1,a);
    }
    recordStep(SortStepType::DONE,-1,-1,a);
}

void SortEngine::runCounting() {
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a=data_;
    int maxV=*std::max_element(a.begin(),a.end());
    int minV=*std::min_element(a.begin(),a.end());
    std::vector<int> cnt(maxV-minV+1,0);
    for(int x:a){ ++cnt[x-minV]; recordStep(SortStepType::SELECT,x-minV,x,a); }
    int idx2=0;
    for(int i=0;i<static_cast<int>(cnt.size());++i)
        while(cnt[i]--){ a[idx2++]=i+minV; ++swaps_; recordStep(SortStepType::INSERT,idx2-1,i,a); }
    recordStep(SortStepType::DONE,-1,-1,a);
}

void SortEngine::runTimSort() {
    // Tim sort: insertion + merge hybrid
    steps_.clear(); comparisons_=0; swaps_=0;
    auto a=data_; int n=static_cast<int>(a.size());
    const int RUN=32;
    // Insertion sort runs
    for(int i=0;i<n;i+=RUN){
        int lo=i, hi=std::min(i+RUN-1,n-1);
        for(int j=lo+1;j<=hi;++j){
            int key=a[j],k=j-1;
            while(k>=lo && (++comparisons_,a[k]>key)){ a[k+1]=a[k]; ++swaps_; recordStep(SortStepType::SHIFT,k,k+1,a); --k; }
            a[k+1]=key;
        }
    }
    // Merge runs
    for(int size=RUN;size<n;size*=2)
        for(int lo2=0;lo2<n;lo2+=2*size){
            int mid=std::min(lo2+size-1,n-1);
            int hi2=std::min(lo2+2*size-1,n-1);
            if(mid<hi2){ std::vector<int> tmp; int l=lo2,r=mid+1;
                while(l<=mid && r<=hi2){ ++comparisons_; if(a[l]<=a[r]) tmp.push_back(a[l++]); else tmp.push_back(a[r++]); }
                while(l<=mid) tmp.push_back(a[l++]); while(r<=hi2) tmp.push_back(a[r++]);
                for(int i=lo2;i<=hi2;++i){ a[i]=tmp[i-lo2]; ++swaps_; recordStep(SortStepType::MERGE,lo2,hi2,a); }
            }
        }
    recordStep(SortStepType::DONE,-1,-1,a);
}

// ─── GraphEngine ─────────────────────────────────────────────────────────────

GraphEngine::GraphEngine(int nodes) : n_(nodes), adj_(nodes) {}

void GraphEngine::addEdge(int u, int v, double w, bool directed) {
    adj_[u].push_back({v, w});
    if (!directed) adj_[v].push_back({u, w});
}

void GraphEngine::clear() { for (auto& l : adj_) l.clear(); }

GraphResult GraphEngine::kruskal() {
    GraphResult res;
    // Collect all edges
    std::vector<std::tuple<double,int,int>> edges;
    for (int u=0;u<n_;++u)
        for (auto [v,w]:adj_[u])
            if (u<v) edges.push_back({w,u,v});
    std::sort(edges.begin(),edges.end());

    // Union-Find
    std::vector<int> par(n_),rank(n_,0);
    std::iota(par.begin(),par.end(),0);
    std::function<int(int)> find=[&](int x){ return par[x]==x?x:par[x]=find(par[x]); };
    auto unite=[&](int a,int b){ a=find(a);b=find(b); if(a==b)return false; if(rank[a]<rank[b])std::swap(a,b); par[b]=a; if(rank[a]==rank[b])rank[a]++; return true; };

    for (auto [w,u,v]:edges){
        AlgoStep s; s.type=StepType::COMPARE; s.primary={u,0}; s.secondary={v,0}; s.g=w;
        res.steps.push_back(s);
        if(unite(u,v)){ res.mstEdges.push_back({u,v,w}); res.totalWeight+=w;
            AlgoStep s2; s2.type=StepType::VISIT; s2.primary={u,0}; s2.secondary={v,0}; res.steps.push_back(s2); }
    }
    return res;
}

GraphResult GraphEngine::prim(int start) {
    GraphResult res;
    std::vector<bool> inMST(n_,false);
    std::vector<double> key(n_,1e18);
    std::vector<int> parent(n_,-1);
    using PQ=std::pair<double,int>;
    std::priority_queue<PQ,std::vector<PQ>,std::greater<PQ>> pq;
    key[start]=0; pq.push({0,start});

    while(!pq.empty()){
        auto [d,u]=pq.top(); pq.pop();
        if(inMST[u]) continue;
        inMST[u]=true;
        AlgoStep s; s.type=StepType::VISIT; s.primary={u,0}; s.g=d; res.steps.push_back(s);
        if(parent[u]!=-1){ res.mstEdges.push_back({parent[u],u,d}); res.totalWeight+=d; }
        for(auto [v,w]:adj_[u]) if(!inMST[v] && w<key[v]){ key[v]=w; parent[v]=u; pq.push({w,v}); }
    }
    return res;
}

GraphResult GraphEngine::bellmanFord(int src) {
    GraphResult res;
    std::vector<double> dist(n_,1e18); dist[src]=0;
    for(int i=0;i<n_-1;++i)
        for(int u=0;u<n_;++u)
            for(auto [v,w]:adj_[u])
                if(dist[u]+w<dist[v]){ dist[v]=dist[u]+w;
                    AlgoStep s; s.type=StepType::VISIT; s.primary={u,0}; s.secondary={v,0}; s.g=dist[v]; res.steps.push_back(s); }
    // Detect negative cycles
    for(int u=0;u<n_;++u)
        for(auto [v,w]:adj_[u])
            if(dist[u]+w<dist[v]){ res.hasNegCycle=true; break; }
    res.dist=dist;
    return res;
}

GraphResult GraphEngine::floydWarshall() {
    GraphResult res;
    std::vector<std::vector<double>> d(n_,std::vector<double>(n_,1e18));
    for(int i=0;i<n_;++i) d[i][i]=0;
    for(int u=0;u<n_;++u) for(auto [v,w]:adj_[u]) d[u][v]=std::min(d[u][v],w);

    for(int k=0;k<n_;++k)
        for(int i=0;i<n_;++i)
            for(int j=0;j<n_;++j)
                if(d[i][k]+d[k][j]<d[i][j]){ d[i][j]=d[i][k]+d[k][j];
                    AlgoStep s; s.type=StepType::VISIT; s.primary={i,k}; s.secondary={k,j}; res.steps.push_back(s); }

    // Flatten into res.dist
    for(int i=0;i<n_;++i) for(int j=0;j<n_;++j) if(d[i][j]<1e17) res.dist.push_back(d[i][j]);
    return res;
}

GraphResult GraphEngine::tarjanSCC() {
    GraphResult res;
    std::vector<int> disc(n_,-1), low(n_), comp(n_,-1), stk;
    std::vector<bool> onStk(n_,false);
    int timer=0, sccCount=0;

    std::function<void(int)> dfs=[&](int u){
        disc[u]=low[u]=timer++; stk.push_back(u); onStk[u]=true;
        AlgoStep s; s.type=StepType::VISIT; s.primary={u,0}; res.steps.push_back(s);
        for(auto [v,w]:adj_[u]){
            if(disc[v]==-1){ dfs(v); low[u]=std::min(low[u],low[v]); }
            else if(onStk[v]) low[u]=std::min(low[u],disc[v]);
        }
        if(low[u]==disc[u]){
            while(true){ int v=stk.back(); stk.pop_back(); onStk[v]=false; comp[v]=sccCount;
                AlgoStep s2; s2.type=StepType::DONE; s2.primary={v,sccCount}; res.steps.push_back(s2);
                if(v==u) break; }
            ++sccCount;
        }
    };

    for(int i=0;i<n_;++i) if(disc[i]==-1) dfs(i);
    res.components = comp;
    return res;
}

GraphResult GraphEngine::topologicalSort() {
    GraphResult res;
    std::vector<int> indegree(n_,0);
    for(int u=0;u<n_;++u) for(auto [v,w]:adj_[u]) indegree[v]++;
    std::queue<int> q;
    for(int i=0;i<n_;++i) if(!indegree[i]) q.push(i);
    while(!q.empty()){
        int u=q.front(); q.pop();
        res.topoOrder.push_back(u);
        AlgoStep s; s.type=StepType::VISIT; s.primary={u,0}; res.steps.push_back(s);
        for(auto [v,w]:adj_[u]) if(!--indegree[v]) q.push(v);
    }
    return res;
}

} // namespace hav

// ═══════════════════════════════════════════════════════════════════════════════
// HAV Engine v3.0 — NEW ALGORITHMS APPENDED
// ═══════════════════════════════════════════════════════════════════════════════

#include <numeric>
#include <random>
#include <climits>

namespace hav {

// helper already defined above: rng(), randInt(), randDouble()

// ─── MazeEngine helpers ───────────────────────────────────────────────────────

void MazeEngine::carvePassage(int r,int c,int nr,int nc) {
    int dr=nr-r,dc=nc-c;
    if(dr==-1){ grid_[r][c].walls&=~WALL_N; grid_[nr][nc].walls&=~WALL_S; }
    else if(dr==1){ grid_[r][c].walls&=~WALL_S; grid_[nr][nc].walls&=~WALL_N; }
    else if(dc==-1){ grid_[r][c].walls&=~WALL_W; grid_[nr][nc].walls&=~WALL_E; }
    else { grid_[r][c].walls&=~WALL_E; grid_[nr][nc].walls&=~WALL_W; }
    steps_.push_back(buildStep(StepType::CARVE,r,c,nr,nc));
}

// ─── Aldous-Broder ────────────────────────────────────────────────────────────
void MazeEngine::generateAldousBroder() {
    reset();
    const int DR[]={-1,1,0,0},DC[]={0,0,-1,1};
    int total=rows_*cols_,visited=1;
    int r=randInt(0,rows_-1),c=randInt(0,cols_-1);
    grid_[r][c].visited=true;
    steps_.push_back(buildStep(StepType::VISIT,r,c));
    while(visited<total){
        int d=randInt(0,3);
        int nr=r+DR[d],nc=c+DC[d];
        if(nr<0||nr>=rows_||nc<0||nc>=cols_) continue;
        if(!grid_[nr][nc].visited){
            carvePassage(r,c,nr,nc);
            grid_[nr][nc].visited=true;
            ++visited;
        } else {
            steps_.push_back(buildStep(StepType::BACKTRACK,r,c));
        }
        r=nr;c=nc;
    }
    grid_[0][0].isStart=true;grid_[rows_-1][cols_-1].isEnd=true;
    steps_.push_back(buildStep(StepType::DONE,0,0));
}

// ─── Sidewinder ───────────────────────────────────────────────────────────────
void MazeEngine::generateSidewinder() {
    reset();
    for(int r=0;r<rows_;++r){
        std::vector<int> run;
        for(int c=0;c<cols_;++c){
            run.push_back(c);
            bool carveE=(c<cols_-1)&&(r==0||randInt(0,1)==0);
            if(carveE){
                carvePassage(r,c,r,c+1);
            } else {
                if(r>0){
                    int pick=run[randInt(0,(int)run.size()-1)];
                    carvePassage(r,pick,r-1,pick);
                }
                run.clear();
            }
            steps_.push_back(buildStep(StepType::VISIT,r,c));
        }
    }
    grid_[0][0].isStart=true;grid_[rows_-1][cols_-1].isEnd=true;
    steps_.push_back(buildStep(StepType::DONE,0,0));
}

// ─── Binary Tree ──────────────────────────────────────────────────────────────
void MazeEngine::generateBinaryTree() {
    reset();
    for(int r=0;r<rows_;++r)
        for(int c=0;c<cols_;++c){
            steps_.push_back(buildStep(StepType::VISIT,r,c));
            bool canN=(r>0),canW=(c>0);
            if(!canN&&!canW) continue;
            if(canN&&canW){ if(randInt(0,1)==0) carvePassage(r,c,r-1,c); else carvePassage(r,c,r,c-1); }
            else if(canN)  carvePassage(r,c,r-1,c);
            else           carvePassage(r,c,r,c-1);
        }
    grid_[0][0].isStart=true;grid_[rows_-1][cols_-1].isEnd=true;
    steps_.push_back(buildStep(StepType::DONE,0,0));
}

// ─── Hunt-and-Kill ────────────────────────────────────────────────────────────
void MazeEngine::generateHuntAndKill() {
    reset();
    const int DR[]={-1,1,0,0},DC[]={0,0,-1,1};
    int r=0,c=0;
    grid_[r][c].visited=true;
    steps_.push_back(buildStep(StepType::VISIT,r,c));

    while(true){
        // Walk phase
        std::vector<int> dirs={0,1,2,3};
        std::shuffle(dirs.begin(),dirs.end(),rng());
        bool moved=false;
        for(int d:dirs){
            int nr=r+DR[d],nc=c+DC[d];
            if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc].visited) continue;
            carvePassage(r,c,nr,nc);
            grid_[nr][nc].visited=true;
            r=nr;c=nc;
            moved=true;
            break;
        }
        if(moved) continue;
        // Hunt phase
        bool found=false;
        for(int hr=0;hr<rows_&&!found;++hr)
            for(int hc=0;hc<cols_&&!found;++hc){
                if(grid_[hr][hc].visited) continue;
                for(int d=0;d<4;++d){
                    int nr=hr+DR[d],nc=hc+DC[d];
                    if(nr<0||nr>=rows_||nc<0||nc>=cols_||!grid_[nr][nc].visited) continue;
                    carvePassage(hr,hc,nr,nc);
                    grid_[hr][hc].visited=true;
                    r=hr;c=hc;
                    found=true;
                    steps_.push_back(buildStep(StepType::VISIT,r,c));
                    break;
                }
            }
        if(!found) break;
    }
    grid_[0][0].isStart=true;grid_[rows_-1][cols_-1].isEnd=true;
    steps_.push_back(buildStep(StepType::DONE,0,0));
}

// ─── Eller's Algorithm ────────────────────────────────────────────────────────
void MazeEngine::generateEller() {
    reset();
    std::vector<int> rowSet(cols_);
    std::iota(rowSet.begin(),rowSet.end(),1);
    int nextSet=cols_+1;

    auto findSet=[&](int id)->int{
        for(int c=0;c<cols_;++c) if(rowSet[c]==id) return c;
        return -1;
    };

    for(int r=0;r<rows_;++r){
        bool lastRow=(r==rows_-1);
        // Merge cells in same row
        for(int c=0;c<cols_-1;++c){
            bool merge=lastRow ? (rowSet[c]!=rowSet[c+1]) : (rowSet[c]!=rowSet[c+1]&&randInt(0,1)==0);
            if(merge){
                int oldSet=rowSet[c+1],newSet=rowSet[c];
                for(int k=0;k<cols_;++k) if(rowSet[k]==oldSet) rowSet[k]=newSet;
                carvePassage(r,c,r,c+1);
            }
            steps_.push_back(buildStep(StepType::VISIT,r,c));
        }
        if(lastRow) break;
        // Vertical connections
        std::unordered_map<int,std::vector<int>> setMembers;
        for(int c=0;c<cols_;++c) setMembers[rowSet[c]].push_back(c);
        std::vector<int> nextRow(cols_,0);
        for(auto& [sid,members]:setMembers){
            std::shuffle(members.begin(),members.end(),rng());
            int cnt=randInt(1,(int)members.size());
            for(int i=0;i<cnt;++i){
                int c=members[i];
                carvePassage(r,c,r+1,c);
                nextRow[c]=sid;
            }
        }
        for(int c=0;c<cols_;++c) if(!nextRow[c]) nextRow[c]=nextSet++;
        rowSet=nextRow;
    }
    grid_[0][0].isStart=true;grid_[rows_-1][cols_-1].isEnd=true;
    steps_.push_back(buildStep(StepType::DONE,0,0));
}

// ─── Kruskal Maze ─────────────────────────────────────────────────────────────
void MazeEngine::generateKruskalMaze() {
    reset();
    // Union-Find
    std::vector<int> par(rows_*cols_),rnk(rows_*cols_,0);
    std::iota(par.begin(),par.end(),0);
    std::function<int(int)> find=[&](int x)->int{return par[x]==x?x:par[x]=find(par[x]);};
    auto unite=[&](int a,int b)->bool{a=find(a);b=find(b);if(a==b)return false;if(rnk[a]<rnk[b])std::swap(a,b);par[b]=a;if(rnk[a]==rnk[b])rnk[a]++;return true;};

    // Build edge list (horizontal + vertical internal walls)
    struct Edge{int r,c,dr,dc;};
    std::vector<Edge> edges;
    for(int r=0;r<rows_;++r)
        for(int c=0;c<cols_;++c){
            if(c<cols_-1) edges.push_back({r,c,0,1});
            if(r<rows_-1) edges.push_back({r,c,1,0});
        }
    std::shuffle(edges.begin(),edges.end(),rng());

    for(auto& e:edges){
        int nr=e.r+e.dr,nc=e.c+e.dc;
        int u=e.r*cols_+e.c,v=nr*cols_+nc;
        if(unite(u,v)){
            carvePassage(e.r,e.c,nr,nc);
            grid_[e.r][e.c].visited=true;
            grid_[nr][nc].visited=true;
        }
    }
    grid_[0][0].isStart=true;grid_[rows_-1][cols_-1].isEnd=true;
    steps_.push_back(buildStep(StepType::DONE,0,0));
}

// ─── PathEngine new algorithms ────────────────────────────────────────────────

static double heuristicFn(int r1,int c1,int r2,int c2,HeuristicType h){
    int dr=std::abs(r1-r2),dc=std::abs(c1-c2);
    switch(h){
        case HeuristicType::MANHATTAN: return dr+dc;
        case HeuristicType::EUCLIDEAN: return std::sqrt((double)(dr*dr+dc*dc));
        case HeuristicType::CHEBYSHEV: return std::max(dr,dc);
        case HeuristicType::OCTILE:    return (dr+dc)+(std::sqrt(2.0)-2)*std::min(dr,dc);
        default: return 0;
    }
}

// Greedy Best-First Search
PathResult PathEngine::greedyBFS(int sr,int sc,int er,int ec,HeuristicType h){
    PathResult res;
    const int DR[]={-1,1,0,0},DC[]={0,0,-1,1};
    std::vector<std::vector<bool>> vis(rows_,std::vector<bool>(cols_,false));
    std::vector<std::vector<std::pair<int,int>>> par(rows_,std::vector<std::pair<int,int>>(cols_,{-1,-1}));
    using PQ=std::pair<double,std::pair<int,int>>;
    std::priority_queue<PQ,std::vector<PQ>,std::greater<PQ>> pq;
    pq.push({heuristicFn(sr,sc,er,ec,h),{sr,sc}});
    while(!pq.empty()){
        auto [hv,rc]=pq.top();pq.pop();
        auto [r,c]=rc;
        if(vis[r][c]) continue;
        vis[r][c]=true;
        AlgoStep s;s.type=StepType::VISIT;s.primary={r,c};s.h=hv;
        res.steps.push_back(s);
        if(r==er&&c==ec){
            auto cur=std::make_pair(r,c);
            while(cur.first!=-1){res.path.push_back(cur);cur=par[cur.first][cur.second];}
            std::reverse(res.path.begin(),res.path.end());
            res.found=true;res.cost=(double)res.path.size()-1;
            return res;
        }
        for(int d=0;d<4;++d){
            int nr=r+DR[d],nc=c+DC[d];
            if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL||vis[nr][nc]) continue;
            par[nr][nc]={r,c};
            pq.push({heuristicFn(nr,nc,er,ec,h),{nr,nc}});
        }
    }
    return res;
}

// Line of sight check for Theta*
bool PathEngine::lineOfSight(int r1,int c1,int r2,int c2)const{
    int dr=std::abs(r2-r1),dc=std::abs(c2-c1);
    int sr=(r1<r2)?1:-1,sc=(c1<c2)?1:-1;
    int err=dr-dc,r=r1,c=c1;
    while(true){
        if(grid_[r][c]==CELL_WALL) return false;
        if(r==r2&&c==c2) return true;
        int e2=2*err;
        if(e2>-dc){err-=dc;r+=sr;}
        if(e2<dr){err+=dr;c+=sc;}
    }
}

// Theta* (any-angle A*)
PathResult PathEngine::thetaStar(int sr,int sc,int er,int ec){
    PathResult res;
    const int DR[]={-1,-1,-1,0,0,1,1,1},DC[]={-1,0,1,-1,1,-1,0,1};
    std::vector<std::vector<double>> g(rows_,std::vector<double>(cols_,1e18));
    std::vector<std::vector<std::pair<int,int>>> par(rows_,std::vector<std::pair<int,int>>(cols_,{-1,-1}));
    std::vector<std::vector<bool>> closed(rows_,std::vector<bool>(cols_,false));
    using PQ=std::pair<double,std::pair<int,int>>;
    std::priority_queue<PQ,std::vector<PQ>,std::greater<PQ>> pq;
    g[sr][sc]=0;par[sr][sc]={sr,sc};
    pq.push({heuristicFn(sr,sc,er,ec,HeuristicType::EUCLIDEAN),{sr,sc}});
    while(!pq.empty()){
        auto [fv,rc]=pq.top();pq.pop();
        auto [r,c]=rc;
        if(closed[r][c]) continue;
        closed[r][c]=true;
        AlgoStep s;s.type=StepType::VISIT;s.primary={r,c};s.g=g[r][c];
        res.steps.push_back(s);
        if(r==er&&c==ec){
            auto cur=std::make_pair(r,c);
            while(cur!=par[cur.first][cur.second]){res.path.push_back(cur);auto p=par[cur.first][cur.second];cur=p;}
            res.path.push_back({sr,sc});
            std::reverse(res.path.begin(),res.path.end());
            res.found=true;res.cost=g[er][ec];return res;
        }
        auto [pr,pc]=par[r][c];
        for(int d=0;d<8;++d){
            int nr=r+DR[d],nc=c+DC[d];
            if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL||closed[nr][nc]) continue;
            double ng;
            // Path 2: parent of current -> neighbour (any-angle)
            if(pr>=0&&lineOfSight(pr,pc,nr,nc)){
                double dx=nr-pr,dy=nc-pc;
                ng=g[pr][pc]+std::sqrt(dx*dx+dy*dy);
                if(ng<g[nr][nc]){g[nr][nc]=ng;par[nr][nc]={pr,pc};
                    pq.push({ng+heuristicFn(nr,nc,er,ec,HeuristicType::EUCLIDEAN),{nr,nc}});}
            } else {
                double dx=nr-r,dy=nc-c;
                ng=g[r][c]+std::sqrt(dx*dx+dy*dy);
                if(ng<g[nr][nc]){g[nr][nc]=ng;par[nr][nc]={r,c};
                    pq.push({ng+heuristicFn(nr,nc,er,ec,HeuristicType::EUCLIDEAN),{nr,nc}});}
            }
        }
    }
    return res;
}

// IDA* helper
bool PathEngine::idaDFS(int r,int c,int er,int ec,double g,double bound,HeuristicType h,
                         std::vector<std::vector<double>>& dist,
                         std::vector<std::vector<std::pair<int,int>>>& par,
                         std::vector<AlgoStep>& steps,double& next)const{
    const int DR[]={-1,1,0,0},DC[]={0,0,-1,1};
    double f=g+heuristicFn(r,c,er,ec,h);
    if(f>bound){next=std::min(next,f);return false;}
    AlgoStep s;s.type=StepType::VISIT;s.primary={r,c};s.g=g;s.f=f;
    steps.push_back(s);
    if(r==er&&c==ec) return true;
    for(int d=0;d<4;++d){
        int nr=r+DR[d],nc=c+DC[d];
        if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL) continue;
        double ng=g+(grid_[nr][nc]==CELL_WEIGHT?3.0:1.0);
        if(ng<dist[nr][nc]){
            dist[nr][nc]=ng;par[nr][nc]={r,c};
            if(idaDFS(nr,nc,er,ec,ng,bound,h,dist,par,steps,next)) return true;
        }
    }
    return false;
}

PathResult PathEngine::idaStar(int sr,int sc,int er,int ec,HeuristicType h){
    PathResult res;
    std::vector<std::vector<double>> dist(rows_,std::vector<double>(cols_,1e18));
    std::vector<std::vector<std::pair<int,int>>> par(rows_,std::vector<std::pair<int,int>>(cols_,{-1,-1}));
    dist[sr][sc]=0;par[sr][sc]={sr,sc};
    double bound=heuristicFn(sr,sc,er,ec,h);
    for(int iter=0;iter<1000;++iter){
        double next=1e18;
        dist.assign(rows_,std::vector<double>(cols_,1e18));
        dist[sr][sc]=0;par[sr][sc]={sr,sc};
        if(idaDFS(sr,sc,er,ec,0,bound,h,dist,par,res.steps,next)){
            auto cur=std::make_pair(er,ec);
            while(cur.first!=-1&&!(cur.first==sr&&cur.second==sc)){
                res.path.push_back(cur);auto p=par[cur.first][cur.second];
                if(p==cur) break;
                cur=p;
            }
            res.path.push_back({sr,sc});
            std::reverse(res.path.begin(),res.path.end());
            res.found=true;res.cost=dist[er][ec];return res;
        }
        if(next>=1e17) break;
        bound=next;
    }
    return res;
}

// IDDFS
PathResult PathEngine::iddfs(int sr,int sc,int er,int ec){
    PathResult res;
    const int DR[]={-1,1,0,0},DC[]={0,0,-1,1};
    for(int maxD=0;maxD<rows_*cols_;++maxD){
        std::vector<std::vector<int>> depth(rows_,std::vector<int>(cols_,INT_MAX));
        std::vector<std::vector<std::pair<int,int>>> par(rows_,std::vector<std::pair<int,int>>(cols_,{-1,-1}));
        std::function<bool(int,int,int)> dls=[&](int r,int c,int d)->bool{
            AlgoStep s;s.type=StepType::VISIT;s.primary={r,c};res.steps.push_back(s);
            if(r==er&&c==ec) return true;
            if(d==0) return false;
            for(int i=0;i<4;++i){
                int nr=r+DR[i],nc=c+DC[i];
                if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL) continue;
                if(depth[nr][nc]<=d) continue;
                depth[nr][nc]=d;par[nr][nc]={r,c};
                if(dls(nr,nc,d-1)) return true;
            }
            return false;
        };
        depth[sr][sc]=maxD;
        if(dls(sr,sc,maxD)){
            auto cur=std::make_pair(er,ec);
            while(cur.first!=-1){res.path.push_back(cur);auto p=par[cur.first][cur.second];if(p.first==-1)break;cur=p;}
            res.path.push_back({sr,sc});
            std::reverse(res.path.begin(),res.path.end());
            res.found=true;return res;
        }
    }
    return res;
}

// Beam Search
PathResult PathEngine::beamSearch(int sr,int sc,int er,int ec,int beamWidth,HeuristicType h){
    PathResult res;
    const int DR[]={-1,1,0,0},DC[]={0,0,-1,1};
    std::vector<std::vector<bool>> vis(rows_,std::vector<bool>(cols_,false));
    std::vector<std::vector<std::pair<int,int>>> par(rows_,std::vector<std::pair<int,int>>(cols_,{-1,-1}));
    using Node=std::pair<double,std::pair<int,int>>;
    std::vector<Node> beam={{heuristicFn(sr,sc,er,ec,h),{sr,sc}}};
    vis[sr][sc]=true;
    while(!beam.empty()){
        std::vector<Node> next;
        for(auto& [hv,rc]:beam){
            auto [r,c]=rc;
            AlgoStep s;s.type=StepType::VISIT;s.primary={r,c};s.h=hv;
            res.steps.push_back(s);
            if(r==er&&c==ec){
                auto cur=std::make_pair(r,c);
                while(par[cur.first][cur.second].first!=-1){res.path.push_back(cur);cur=par[cur.first][cur.second];}
                res.path.push_back({sr,sc});
                std::reverse(res.path.begin(),res.path.end());
                res.found=true;res.cost=(double)res.path.size()-1;return res;
            }
            for(int d=0;d<4;++d){
                int nr=r+DR[d],nc=c+DC[d];
                if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL||vis[nr][nc]) continue;
                vis[nr][nc]=true;par[nr][nc]={r,c};
                next.push_back({heuristicFn(nr,nc,er,ec,h),{nr,nc}});
            }
        }
        std::sort(next.begin(),next.end());
        if((int)next.size()>beamWidth) next.resize(beamWidth);
        beam=next;
    }
    return res;
}

// Bidirectional BFS
PathResult PathEngine::bidirectionalBFS(int sr,int sc,int er,int ec){
    PathResult res;
    if(sr==er&&sc==ec){res.found=true;res.path.push_back({sr,sc});return res;}
    const int DR[]={-1,1,0,0},DC[]={0,0,-1,1};
    auto idx=[&](int r,int c){return r*cols_+c;};
    int total=rows_*cols_;
    std::vector<int> parF(total,-1),parB(total,-1);
    std::vector<bool> visF(total,false),visB(total,false);
    std::queue<int> qF,qB;
    visF[idx(sr,sc)]=true;qF.push(idx(sr,sc));
    visB[idx(er,ec)]=true;qB.push(idx(er,ec));
    int meet=-1;

    auto expand=[&](std::queue<int>& q,std::vector<bool>& vis,std::vector<int>& par,
                    std::vector<bool>& other)->int{
        int u=q.front();q.pop();
        int r=u/cols_,c=u%cols_;
        AlgoStep s;s.type=StepType::VISIT;s.primary={r,c};res.steps.push_back(s);
        for(int d=0;d<4;++d){
            int nr=r+DR[d],nc=c+DC[d];
            if(nr<0||nr>=rows_||nc<0||nc>=cols_||grid_[nr][nc]==CELL_WALL) continue;
            int v=idx(nr,nc);
            if(!vis[v]){vis[v]=true;par[v]=u;q.push(v);}
            if(other[v]) return v;
        }
        return -1;
    };

    while(!qF.empty()||!qB.empty()){
        if(!qF.empty()){int m=expand(qF,visF,parF,visB);if(m>=0){meet=m;break;}}
        if(!qB.empty()){int m=expand(qB,visB,parB,visF);if(m>=0){meet=m;break;}}
    }

    if(meet>=0){
        res.found=true;
        std::vector<std::pair<int,int>> fwd,bwd;
        int cur=meet;
        while(cur!=-1){fwd.push_back({cur/cols_,cur%cols_});cur=parF[cur];}
        std::reverse(fwd.begin(),fwd.end());
        cur=parB[meet];
        while(cur!=-1){bwd.push_back({cur/cols_,cur%cols_});cur=parB[cur];}
        res.path=fwd;for(auto& p:bwd)res.path.push_back(p);
        res.cost=(double)res.path.size()-1;
    }
    return res;
}

// ─── SortEngine new algorithms ────────────────────────────────────────────────

void SortEngine::rec(SortStepType t,int i,int j,const std::vector<int>& a){
    SortStep s;s.type=t;s.i=i;s.j=j;s.arr=a;s.comparisons=comparisons_;s.swaps=swaps_;
    steps_.push_back(s);
}

// Cocktail Shaker Sort
void SortEngine::runCocktail(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;int n=static_cast<int>(a.size());
    int lo=0,hi=n-1;bool swapped=true;
    while(swapped){
        swapped=false;
        for(int i=lo;i<hi;++i){++comparisons_;rec(SortStepType::COMPARE,i,i+1,a);if(a[i]>a[i+1]){std::swap(a[i],a[i+1]);++swaps_;rec(SortStepType::SWAP,i,i+1,a);swapped=true;}}
        if(!swapped) break;
        --hi;swapped=false;
        for(int i=hi-1;i>=lo;--i){++comparisons_;rec(SortStepType::COMPARE,i,i+1,a);if(a[i]>a[i+1]){std::swap(a[i],a[i+1]);++swaps_;rec(SortStepType::SWAP,i,i+1,a);swapped=true;}}
        ++lo;
    }
    rec(SortStepType::DONE,-1,-1,a);
}

// Gnome Sort
void SortEngine::runGnome(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;int n=static_cast<int>(a.size()),i=0;
    while(i<n){
        if(i==0){++i;continue;}
        ++comparisons_;rec(SortStepType::COMPARE,i-1,i,a);
        if(a[i-1]<=a[i]){++i;}
        else{std::swap(a[i-1],a[i]);++swaps_;rec(SortStepType::SWAP,i-1,i,a);--i;}
    }
    rec(SortStepType::DONE,-1,-1,a);
}

// Comb Sort
void SortEngine::runComb(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;int n=static_cast<int>(a.size());
    int gap=n;bool swapped=true;
    while(gap>1||swapped){
        gap=std::max(1,(int)(gap/1.3));swapped=false;
        for(int i=0;i+gap<n;++i){
            ++comparisons_;rec(SortStepType::COMPARE,i,i+gap,a);
            if(a[i]>a[i+gap]){std::swap(a[i],a[i+gap]);++swaps_;rec(SortStepType::SWAP,i,i+gap,a);swapped=true;}
        }
    }
    rec(SortStepType::DONE,-1,-1,a);
}

// Cycle Sort (minimises writes)
void SortEngine::runCycle(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;int n=static_cast<int>(a.size());
    for(int cs=0;cs<n-1;++cs){
        int item=a[cs],pos=cs;
        for(int i=cs+1;i<n;++i){++comparisons_;if(a[i]<item)++pos;}
        if(pos==cs) continue;
        while(a[pos]==item)++pos;
        std::swap(a[pos],item);++swaps_;rec(SortStepType::SWAP,cs,pos,a);
        while(pos!=cs){
            pos=cs;
            for(int i=cs+1;i<n;++i){++comparisons_;if(a[i]<item)++pos;}
            while(a[pos]==item)++pos;
            std::swap(a[pos],item);++swaps_;rec(SortStepType::SWAP,cs,pos,a);
        }
    }
    rec(SortStepType::DONE,-1,-1,a);
}

// Pancake Sort
void SortEngine::flip(std::vector<int>& a,int k){
    int lo=0,hi=k;
    while(lo<hi){std::swap(a[lo],a[hi]);++swaps_;++lo;--hi;}
    rec(SortStepType::SWAP,0,k,a);
}

void SortEngine::runPancake(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;int n=static_cast<int>(a.size());
    for(int size=n;size>1;--size){
        int mi=0;
        for(int i=1;i<size;++i){++comparisons_;rec(SortStepType::COMPARE,i,mi,a);if(a[i]>a[mi])mi=i;}
        if(mi!=size-1){
            if(mi!=0) flip(a,mi);
            flip(a,size-1);
        }
    }
    rec(SortStepType::DONE,-1,-1,a);
}

// Bitonic Sort
void SortEngine::bitonicMerge(std::vector<int>& a,int lo,int cnt,bool asc){
    if(cnt<=1) return;
    int k=cnt/2;
    for(int i=lo;i<lo+k;++i){
        ++comparisons_;rec(SortStepType::COMPARE,i,i+k,a);
        if((asc&&a[i]>a[i+k])||(!asc&&a[i]<a[i+k])){std::swap(a[i],a[i+k]);++swaps_;rec(SortStepType::SWAP,i,i+k,a);}
    }
    bitonicMerge(a,lo,k,asc);
    bitonicMerge(a,lo+k,k,asc);
}

void SortEngine::bitonicSort(std::vector<int>& a,int lo,int cnt,bool asc){
    if(cnt<=1) return;
    int k=cnt/2;
    bitonicSort(a,lo,k,true);
    bitonicSort(a,lo+k,k,false);
    bitonicMerge(a,lo,cnt,asc);
}

void SortEngine::runBitonic(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;
    // Pad to next power of 2
    int n=static_cast<int>(a.size()),p=1;
    while(p<n)p<<=1;
    a.resize(p,INT_MAX);
    bitonicSort(a,0,p,true);
    a.resize(n);
    rec(SortStepType::DONE,-1,-1,a);
}

// Odd-Even Transposition Sort
void SortEngine::runOddEven(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;int n=static_cast<int>(a.size());
    bool sorted=false;
    while(!sorted){
        sorted=true;
        for(int phase=0;phase<2;++phase)
            for(int i=phase;i<n-1;i+=2){
                ++comparisons_;rec(SortStepType::COMPARE,i,i+1,a);
                if(a[i]>a[i+1]){std::swap(a[i],a[i+1]);++swaps_;rec(SortStepType::SWAP,i,i+1,a);sorted=false;}
            }
    }
    rec(SortStepType::DONE,-1,-1,a);
}

// Stooge Sort
void SortEngine::stoogeSort(std::vector<int>& a,int lo,int hi){
    ++comparisons_;rec(SortStepType::COMPARE,lo,hi,a);
    if(a[lo]>a[hi]){std::swap(a[lo],a[hi]);++swaps_;rec(SortStepType::SWAP,lo,hi,a);}
    if(hi-lo+1>2){
        int t=(hi-lo+1)/3;
        stoogeSort(a,lo,hi-t);
        stoogeSort(a,lo+t,hi);
        stoogeSort(a,lo,hi-t);
    }
}

void SortEngine::runStooge(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;
    stoogeSort(a,0,static_cast<int>(a.size())-1);
    rec(SortStepType::DONE,-1,-1,a);
}

// Introsort (Quick+Heap+Insertion hybrid)
void SortEngine::introSort(std::vector<int>& a,int lo,int hi,int depth){
    if(hi-lo<16){
        for(int i=lo+1;i<=hi;++i){
            int key=a[i];int j=i-1;
            while(j>=lo&&(++comparisons_,a[j]>key)){a[j+1]=a[j];++swaps_;rec(SortStepType::SHIFT,j,j+1,a);--j;}
            a[j+1]=key;
        }
        return;
    }
    if(depth==0){
        // Heap sort on subarray
        int n=hi-lo+1;
        for(int i=n/2-1;i>=0;--i) heapify(a,n,lo+i); // simplified: heapify relative
        for(int i=hi;i>lo;--i){std::swap(a[lo],a[i]);++swaps_;rec(SortStepType::SWAP,lo,i,a);heapify(a,i-lo,lo);}
        return;
    }
    int p=partition(a,lo,hi);
    introSort(a,lo,p-1,depth-1);
    introSort(a,p+1,hi,depth-1);
}

void SortEngine::runIntro(){
    steps_.clear();comparisons_=0;swaps_=0;
    auto a=data_;int n=static_cast<int>(a.size());
    int depth=2*(int)std::floor(std::log2((double)n));
    introSort(a,0,n-1,depth);
    rec(SortStepType::DONE,-1,-1,a);
}

// ─── GraphEngine new algorithms ───────────────────────────────────────────────

// Boruvka MST
GraphResult GraphEngine::boruvka(){
    GraphResult res;
    std::vector<int> comp(n_);std::iota(comp.begin(),comp.end(),0);
    std::function<int(int)> find=[&](int x)->int{return comp[x]==x?x:comp[x]=find(comp[x]);};
    auto unite=[&](int a,int b){comp[find(a)]=find(b);};

    for(int iter=0;iter<n_;++iter){
        std::vector<std::tuple<double,int,int>> cheapest(n_,{1e18,-1,-1});
        for(int u=0;u<n_;++u)
            for(auto [v,w]:adj_[u]){
                int cu=find(u),cv=find(v);
                if(cu==cv) continue;
                if(w<std::get<0>(cheapest[cu])) cheapest[cu]={w,u,v};
                if(w<std::get<0>(cheapest[cv])) cheapest[cv]={w,u,v};
                AlgoStep s;s.type=StepType::COMPARE;s.primary={u,0};s.secondary={v,0};s.g=w;
                res.steps.push_back(s);
            }
        bool merged=false;
        for(int i=0;i<n_;++i){
            auto [w,u,v]=cheapest[i];
            if(u<0) continue;
            if(find(u)!=find(v)){
                unite(u,v);res.mstEdges.push_back({u,v,w});res.totalWeight+=w;merged=true;
                AlgoStep s;s.type=StepType::VISIT;s.primary={u,0};s.secondary={v,0};res.steps.push_back(s);
            }
        }
        if(!merged) break;
    }
    return res;
}

// Dijkstra SSSP (full result with all distances)
GraphResult GraphEngine::dijkstraSSSP(int src){
    GraphResult res;
    std::vector<double> dist(n_,1e18);dist[src]=0;
    std::vector<bool> vis(n_,false);
    using PQ=std::pair<double,int>;
    std::priority_queue<PQ,std::vector<PQ>,std::greater<PQ>> pq;
    pq.push({0,src});
    while(!pq.empty()){
        auto [d,u]=pq.top();pq.pop();
        if(vis[u]) continue;vis[u]=true;
        AlgoStep s;s.type=StepType::VISIT;s.primary={u,0};s.g=d;res.steps.push_back(s);
        for(auto [v,w]:adj_[u])
            if(dist[u]+w<dist[v]){dist[v]=dist[u]+w;pq.push({dist[v],v});}
    }
    res.dist=dist;
    return res;
}

// Johnson's Algorithm (APSP with negative weights)
GraphResult GraphEngine::johnson(){
    GraphResult res;
    // Add virtual node n_ connected to all with weight 0
    std::vector<double> h(n_+1,1e18);h[n_]=0;
    std::vector<std::vector<std::pair<int,double>>> extAdj(n_+1);
    for(int u=0;u<n_;++u) for(auto [v,w]:adj_[u]) extAdj[u].push_back({v,w});
    for(int u=0;u<n_;++u) extAdj[n_].push_back({u,0.0});
    // Bellman-Ford from virtual node
    for(int i=0;i<n_;++i)
        for(int u=0;u<=n_;++u)
            for(auto [v,w]:extAdj[u])
                if(h[u]+w<h[v]) h[v]=h[u]+w;
    // Re-weight and run Dijkstra from each source
    for(int s=0;s<n_;++s){
        std::vector<double> dist(n_,1e18);dist[s]=0;
        std::vector<bool> vis(n_,false);
        using PQ=std::pair<double,int>;
        std::priority_queue<PQ,std::vector<PQ>,std::greater<PQ>> pq;
        pq.push({0,s});
        while(!pq.empty()){
            auto [d,u]=pq.top();pq.pop();
            if(vis[u]) continue;vis[u]=true;
            AlgoStep step;step.type=StepType::VISIT;step.primary={s,u};step.g=d;res.steps.push_back(step);
            for(auto [v,w]:adj_[u]){
                double rw=w+h[u]-h[v];
                if(dist[u]+rw<dist[v]){dist[v]=dist[u]+rw;pq.push({dist[v],v});}
            }
        }
        // Correct distances
        for(int v=0;v<n_;++v) if(dist[v]<1e17) res.dist.push_back(dist[v]+h[v]-h[s]);
    }
    return res;
}

// Kosaraju SCC
void GraphEngine::kosarajuDFS1(int u,std::vector<bool>& vis,std::stack<int>& st)const{
    vis[u]=true;
    for(auto [v,w]:adj_[u]) if(!vis[v]) kosarajuDFS1(v,vis,st);
    st.push(u);
}

void GraphEngine::kosarajuDFS2(int u,int comp,std::vector<bool>& vis,std::vector<int>& comps,
                                const std::vector<std::vector<int>>& radj)const{
    vis[u]=true;comps[u]=comp;
    for(int v:radj[u]) if(!vis[v]) kosarajuDFS2(v,comp,vis,comps,radj);
}

GraphResult GraphEngine::kosarajuSCC(){
    GraphResult res;
    std::vector<bool> vis(n_,false);
    std::stack<int> st;
    for(int i=0;i<n_;++i) if(!vis[i]) kosarajuDFS1(i,vis,st);
    // Build reverse graph
    std::vector<std::vector<int>> radj(n_);
    for(int u=0;u<n_;++u) for(auto [v,w]:adj_[u]) radj[v].push_back(u);
    std::fill(vis.begin(),vis.end(),false);
    std::vector<int> comps(n_,-1);int cnt=0;
    while(!st.empty()){
        int u=st.top();st.pop();
        if(!vis[u]){
            kosarajuDFS2(u,cnt,vis,comps,radj);
            AlgoStep s;s.type=StepType::DONE;s.primary={u,cnt};res.steps.push_back(s);
            ++cnt;
        }
    }
    res.components=comps;
    return res;
}

// Articulation Points and Bridges
void GraphEngine::apDFS(int u,int p,int& timer,std::vector<int>& disc,std::vector<int>& low,
                         std::vector<bool>& vis,GraphResult& res)const{
    vis[u]=true;disc[u]=low[u]=timer++;
    int children=0;
    for(auto [v,w]:adj_[u]){
        if(!vis[v]){
            ++children;
            AlgoStep s;s.type=StepType::VISIT;s.primary={u,0};s.secondary={v,0};res.steps.push_back(s);
            apDFS(v,u,timer,disc,low,vis,res);
            low[u]=std::min(low[u],low[v]);
            // Articulation point
            if((p==-1&&children>1)||(p!=-1&&low[v]>=disc[u])){
                if(std::find(res.articulationPoints.begin(),res.articulationPoints.end(),u)==res.articulationPoints.end())
                    res.articulationPoints.push_back(u);
                AlgoStep sa;sa.type=StepType::ARTICULATION;sa.primary={u,0};res.steps.push_back(sa);
            }
            // Bridge
            if(low[v]>disc[u]){
                res.bridges.push_back({u,v});
                AlgoStep sb;sb.type=StepType::BRIDGE;sb.primary={u,0};sb.secondary={v,0};res.steps.push_back(sb);
            }
        } else if(v!=p){
            low[u]=std::min(low[u],disc[v]);
        }
    }
}

GraphResult GraphEngine::articulationAndBridges(){
    GraphResult res;
    std::vector<int> disc(n_,-1),low(n_,0);
    std::vector<bool> vis(n_,false);
    int timer=0;
    for(int i=0;i<n_;++i) if(!vis[i]) apDFS(i,-1,timer,disc,low,vis,res);
    return res;
}

// Bipartite Check (BFS 2-coloring)
GraphResult GraphEngine::bipartiteCheck(){
    GraphResult res;
    std::vector<int> color(n_,-1);
    res.isBipartite=true;
    for(int start=0;start<n_;++start){
        if(color[start]!=-1) continue;
        std::queue<int> q;q.push(start);color[start]=0;
        while(!q.empty()){
            int u=q.front();q.pop();
            AlgoStep s;s.type=StepType::VISIT;s.primary={u,color[u]};res.steps.push_back(s);
            for(auto [v,w]:adj_[u]){
                if(color[v]==-1){color[v]=1-color[u];q.push(v);}
                else if(color[v]==color[u]){res.isBipartite=false;
                    AlgoStep sb;sb.type=StepType::DONE;sb.primary={u,v};res.steps.push_back(sb);}
            }
        }
    }
    res.coloring=color;
    return res;
}

// Euler Circuit (Hierholzer's)
GraphResult GraphEngine::eulerCircuit(int start){
    GraphResult res;
    // Check degrees
    std::vector<int> deg(n_,0);
    for(int u=0;u<n_;++u) deg[u]=(int)adj_[u].size();
    bool hasEuler=true;
    for(int i=0;i<n_;++i) if(deg[i]%2!=0){hasEuler=false;break;}
    res.hasEuler=hasEuler;
    if(!hasEuler) return res;

    // Copy adjacency as edge indices
    std::vector<std::vector<std::pair<int,double>>> tmpAdj=adj_;
    std::stack<int> stk;stk.push(start);
    while(!stk.empty()){
        int u=stk.top();
        if(tmpAdj[u].empty()){
            res.eulerCircuit.push_back(u);stk.pop();
            AlgoStep s;s.type=StepType::VISIT;s.primary={u,0};res.steps.push_back(s);
        } else {
            auto [v,w]=tmpAdj[u].back();tmpAdj[u].pop_back();
            // Remove reverse edge
            for(auto it=tmpAdj[v].begin();it!=tmpAdj[v].end();++it)
                if(it->first==u){tmpAdj[v].erase(it);break;}
            stk.push(v);
        }
    }
    return res;
}

// Hamiltonian Path (backtracking)
GraphResult GraphEngine::hamiltonianPath(int start){
    GraphResult res;
    std::vector<bool> vis(n_,false);
    std::vector<int> path;path.push_back(start);vis[start]=true;

    std::function<bool(int)> bt=[&](int u)->bool{
        AlgoStep s;s.type=StepType::VISIT;s.primary={u,0};res.steps.push_back(s);
        if((int)path.size()==n_){res.hasHamiltonian=true;return true;}
        for(auto [v,w]:adj_[u]){
            if(!vis[v]){
                vis[v]=true;path.push_back(v);
                if(bt(v)) return true;
                vis[v]=false;path.pop_back();
                AlgoStep sb;sb.type=StepType::BACKTRACK;sb.primary={v,0};res.steps.push_back(sb);
            }
        }
        return false;
    };

    bt(start);
    res.hamiltonPath=path;
    return res;
}

// Greedy Graph Coloring
GraphResult GraphEngine::greedyColoring(){
    GraphResult res;
    std::vector<int> color(n_,-1);
    color[0]=0;
    for(int u=1;u<n_;++u){
        std::vector<bool> avail(n_,true);
        for(auto [v,w]:adj_[u]) if(color[v]>=0) avail[color[v]]=false;
        for(int c=0;c<n_;++c) if(avail[c]){color[u]=c;break;}
        AlgoStep s;s.type=StepType::VISIT;s.primary={u,color[u]};res.steps.push_back(s);
    }
    res.coloring=color;
    res.chromaticNum=*std::max_element(color.begin(),color.end())+1;
    return res;
}

// Ford-Fulkerson Max Flow (BFS augmenting paths = Edmonds-Karp)
bool GraphEngine::bfsAug(const std::vector<std::vector<double>>& cap,int s,int t,std::vector<int>& par)const{
    std::vector<bool> vis(n_,false);vis[s]=true;par[s]=-1;
    std::queue<int> q;q.push(s);
    while(!q.empty()){
        int u=q.front();q.pop();
        for(int v=0;v<n_;++v)
            if(!vis[v]&&cap[u][v]>1e-9){vis[v]=true;par[v]=u;if(v==t)return true;q.push(v);}
    }
    return false;
}

GraphResult GraphEngine::fordFulkerson(int source,int sink){
    GraphResult res;
    std::vector<std::vector<double>> cap(n_,std::vector<double>(n_,0));
    for(int u=0;u<n_;++u) for(auto [v,w]:adj_[u]) cap[u][v]+=w;
    std::vector<int> par(n_);
    double flow=0;
    while(bfsAug(cap,source,sink,par)){
        double pathFlow=1e18;
        for(int v=sink;v!=source;v=par[v]){int u=par[v];pathFlow=std::min(pathFlow,cap[u][v]);}
        for(int v=sink;v!=source;v=par[v]){int u=par[v];cap[u][v]-=pathFlow;cap[v][u]+=pathFlow;}
        flow+=pathFlow;
        AlgoStep s;s.type=StepType::VISIT;s.primary={source,sink};s.g=flow;res.steps.push_back(s);
    }
    res.maxFlow=flow;
    return res;
}

} // namespace hav
