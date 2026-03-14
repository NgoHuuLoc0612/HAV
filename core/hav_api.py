"""
HAV Python Bindings & API Server
Bridges C++ core via pybind11 → exposes REST/WebSocket API for frontend
"""

import json
import time
import asyncio
import threading
from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from enum import Enum
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

# ─── Attempt pybind11 import, fallback to pure-Python stub ───────────────────

try:
    import hav_native as _cpp  # compiled pybind11 module
    NATIVE_AVAILABLE = True
except ImportError:
    NATIVE_AVAILABLE = False
    _cpp = None

# ─── Python-level enums (mirror C++) ─────────────────────────────────────────

class GrowingTreeStrategy(str, Enum):
    NEWEST = "NEWEST"
    OLDEST = "OLDEST"
    RANDOM = "RANDOM"
    MIX    = "MIX"
    MIDDLE = "MIDDLE"

class HeuristicType(str, Enum):
    MANHATTAN = "MANHATTAN"
    EUCLIDEAN = "EUCLIDEAN"
    CHEBYSHEV = "CHEBYSHEV"
    OCTILE    = "OCTILE"

class SortAlgorithm(str, Enum):
    BUBBLE    = "bubble"
    INSERTION = "insertion"
    SELECTION = "selection"
    MERGE     = "merge"
    QUICK     = "quick"
    HEAP      = "heap"
    SHELL     = "shell"
    RADIX     = "radix"
    COUNTING  = "counting"
    TIMSORT   = "timsort"

class PathAlgorithm(str, Enum):
    ASTAR         = "astar"
    DIJKSTRA      = "dijkstra"
    BFS           = "bfs"
    DFS           = "dfs"
    JPS           = "jps"
    BIDIR_ASTAR   = "bidir_astar"

class GraphAlgorithm(str, Enum):
    KRUSKAL       = "kruskal"
    PRIM          = "prim"
    BELLMAN_FORD  = "bellman_ford"
    FLOYD         = "floyd_warshall"
    TARJAN        = "tarjan"
    TOPO          = "topological"

class MazeAlgorithm(str, Enum):
    GROWING_TREE       = "growing_tree"
    RECURSIVE_DIVISION = "recursive_division"
    WILSON             = "wilson"

# ─── Pure-Python fallback implementations ────────────────────────────────────

class _PythonMazeFallback:
    """Pure-Python Growing-Tree maze generator (used when C++ not compiled)"""

    def __init__(self, rows: int, cols: int):
        self.rows = rows
        self.cols = cols
        self.WALL_N, self.WALL_S, self.WALL_W, self.WALL_E = 1, 2, 4, 8

    def generate(self, strategy: GrowingTreeStrategy, mix_ratio: float = 0.5):
        import random
        rows, cols = self.rows, self.cols
        grid = [[0xF] * cols for _ in range(rows)]
        visited = [[False] * cols for _ in range(rows)]
        steps = []
        active: list = []

        DR = [-1, 1, 0, 0]
        DC = [0, 0, -1, 1]
        WALL_BIT = [self.WALL_N, self.WALL_S, self.WALL_W, self.WALL_E]
        OPP_WALL = [self.WALL_S, self.WALL_N, self.WALL_E, self.WALL_W]

        def visit(r, c):
            visited[r][c] = True
            active.append((r, c))
            steps.append({"type": "VISIT", "r": r, "c": c, "walls": grid[r][c]})

        visit(0, 0)

        while active:
            if strategy == GrowingTreeStrategy.NEWEST:
                idx = len(active) - 1
            elif strategy == GrowingTreeStrategy.OLDEST:
                idx = 0
            elif strategy == GrowingTreeStrategy.RANDOM:
                idx = random.randint(0, len(active) - 1)
            elif strategy == GrowingTreeStrategy.MIX:
                idx = len(active) - 1 if random.random() < mix_ratio else random.randint(0, len(active) - 1)
            elif strategy == GrowingTreeStrategy.MIDDLE:
                idx = len(active) // 2
            else:
                idx = len(active) - 1

            cr, cc = active[idx]
            dirs = list(range(4))
            random.shuffle(dirs)
            carved = False

            for d in dirs:
                nr, nc = cr + DR[d], cc + DC[d]
                if not (0 <= nr < rows and 0 <= nc < cols): continue
                if visited[nr][nc]: continue
                grid[cr][cc] &= ~WALL_BIT[d]
                grid[nr][nc] &= ~OPP_WALL[d]
                steps.append({"type": "CARVE", "r": cr, "c": cc,
                               "r2": nr, "c2": nc, "walls": grid[cr][cc]})
                visit(nr, nc)
                carved = True
                break

            if not carved:
                steps.append({"type": "BACKTRACK", "r": cr, "c": cc, "walls": grid[cr][cc]})
                active.pop(idx)

        steps.append({"type": "DONE", "r": 0, "c": 0, "walls": 0})
        return {"grid": grid, "steps": steps, "rows": rows, "cols": cols}


class _PythonSortFallback:
    """Pure-Python sort engine fallback"""

    def run(self, algorithm: SortAlgorithm, data: List[int]) -> Dict:
        arr = list(data)
        steps = []
        comparisons = [0]
        swaps = [0]

        def record(typ, i, j):
            steps.append({"type": typ, "i": i, "j": j, "arr": arr[:],
                          "comparisons": comparisons[0], "swaps": swaps[0]})

        if algorithm == SortAlgorithm.BUBBLE:
            n = len(arr)
            for i in range(n - 1):
                for j in range(n - 1 - i):
                    comparisons[0] += 1; record("COMPARE", j, j+1)
                    if arr[j] > arr[j+1]:
                        arr[j], arr[j+1] = arr[j+1], arr[j]
                        swaps[0] += 1; record("SWAP", j, j+1)

        elif algorithm == SortAlgorithm.QUICK:
            def qs(lo, hi):
                if lo >= hi: return
                pivot = arr[hi]; i = lo - 1
                record("PIVOT", lo, hi)
                for j in range(lo, hi):
                    comparisons[0] += 1; record("COMPARE", j, hi)
                    if arr[j] <= pivot:
                        i += 1; arr[i], arr[j] = arr[j], arr[i]
                        swaps[0] += 1; record("SWAP", i, j)
                arr[i+1], arr[hi] = arr[hi], arr[i+1]
                swaps[0] += 1; record("SWAP", i+1, hi)
                qs(lo, i); qs(i+2, hi)
            qs(0, len(arr) - 1)

        elif algorithm == SortAlgorithm.MERGE:
            def ms(a, lo, hi):
                if hi - lo <= 1: return
                mid = (lo + hi) // 2
                ms(a, lo, mid); ms(a, mid, hi)
                tmp = []
                l, r = lo, mid
                while l < mid and r < hi:
                    comparisons[0] += 1
                    if a[l] <= a[r]: tmp.append(a[l]); l += 1
                    else: tmp.append(a[r]); r += 1
                tmp += a[l:mid] + a[r:hi]
                a[lo:hi] = tmp
                swaps[0] += (hi - lo); record("MERGE", lo, hi)
            ms(arr, 0, len(arr))

        elif algorithm == SortAlgorithm.HEAP:
            def heapify(n, i):
                largest = i; l = 2*i+1; r = 2*i+2
                comparisons[0] += 1
                if l < n and arr[l] > arr[largest]: largest = l
                comparisons[0] += 1
                if r < n and arr[r] > arr[largest]: largest = r
                if largest != i:
                    arr[i], arr[largest] = arr[largest], arr[i]
                    swaps[0] += 1; record("SWAP", i, largest)
                    heapify(n, largest)
            n = len(arr)
            for i in range(n//2-1, -1, -1): heapify(n, i)
            for i in range(n-1, 0, -1):
                arr[0], arr[i] = arr[i], arr[0]; swaps[0] += 1; record("SWAP", 0, i)
                heapify(i, 0)

        elif algorithm == SortAlgorithm.INSERTION:
            for i in range(1, len(arr)):
                key = arr[i]; j = i - 1; record("SELECT", i, i)
                while j >= 0 and (comparisons.__setitem__(0, comparisons[0]+1) or True) and arr[j] > key:
                    arr[j+1] = arr[j]; swaps[0] += 1; record("SHIFT", j, j+1); j -= 1
                arr[j+1] = key; record("INSERT", j+1, i)

        elif algorithm == SortAlgorithm.SELECTION:
            n = len(arr)
            for i in range(n-1):
                min_idx = i
                for j in range(i+1, n):
                    comparisons[0] += 1; record("COMPARE", i, j)
                    if arr[j] < arr[min_idx]: min_idx = j
                if min_idx != i:
                    arr[i], arr[min_idx] = arr[min_idx], arr[i]
                    swaps[0] += 1; record("SWAP", i, min_idx)

        record("DONE", -1, -1)
        return {"steps": steps, "comparisons": comparisons[0], "swaps": swaps[0], "sorted": arr}


class _PythonPathFallback:
    """Pure-Python pathfinding fallback"""

    def run(self, algorithm: PathAlgorithm, grid: List[List[int]],
            sr: int, sc: int, er: int, ec: int,
            heuristic: HeuristicType = HeuristicType.MANHATTAN) -> Dict:
        import heapq, math
        rows, cols = len(grid), len(grid[0])
        steps = []

        def h(r, c):
            dr, dc = abs(r-er), abs(c-ec)
            if heuristic == HeuristicType.MANHATTAN: return dr + dc
            if heuristic == HeuristicType.EUCLIDEAN: return math.sqrt(dr*dr+dc*dc)
            if heuristic == HeuristicType.CHEBYSHEV: return max(dr, dc)
            if heuristic == HeuristicType.OCTILE: return (dr+dc)+(math.sqrt(2)-2)*min(dr,dc)
            return dr + dc

        if algorithm in (PathAlgorithm.ASTAR, PathAlgorithm.DIJKSTRA):
            g = [[float('inf')]*cols for _ in range(rows)]
            parent = [[None]*cols for _ in range(rows)]
            g[sr][sc] = 0
            pq = [(h(sr,sc), 0, sr, sc)]
            while pq:
                fv, gv, r, c = heapq.heappop(pq)
                if gv > g[r][c]: continue
                steps.append({"type": "VISIT", "r": r, "c": c, "g": gv, "h": h(r,c), "f": fv})
                if r == er and c == ec:
                    path = []
                    cur = (r, c)
                    while cur:
                        path.append({"r": cur[0], "c": cur[1]})
                        cur = parent[cur[0]][cur[1]]
                    path.reverse()
                    return {"found": True, "cost": g[er][ec], "path": path, "steps": steps}
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nr, nc = r+dr, c+dc
                    if 0<=nr<rows and 0<=nc<cols and grid[nr][nc] != 1:
                        ng = g[r][c] + (3.0 if grid[nr][nc]==2 else 1.0)
                        if ng < g[nr][nc]:
                            g[nr][nc] = ng; parent[nr][nc] = (r,c)
                            heapq.heappush(pq, (ng + (0 if algorithm==PathAlgorithm.DIJKSTRA else h(nr,nc)), ng, nr, nc))

        elif algorithm == PathAlgorithm.BFS:
            from collections import deque
            visited = [[False]*cols for _ in range(rows)]
            parent = [[None]*cols for _ in range(rows)]
            q = deque([(sr, sc)])
            visited[sr][sc] = True
            while q:
                r, c = q.popleft()
                steps.append({"type": "VISIT", "r": r, "c": c})
                if r == er and c == ec:
                    path = []
                    cur = (r, c)
                    while cur:
                        path.append({"r": cur[0], "c": cur[1]})
                        cur = parent[cur[0]][cur[1]]
                    path.reverse()
                    return {"found": True, "cost": len(path)-1, "path": path, "steps": steps}
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nr, nc = r+dr, c+dc
                    if 0<=nr<rows and 0<=nc<cols and not visited[nr][nc] and grid[nr][nc]!=1:
                        visited[nr][nc]=True; parent[nr][nc]=(r,c); q.append((nr,nc))

        elif algorithm == PathAlgorithm.DFS:
            visited = [[False]*cols for _ in range(rows)]
            parent = [[None]*cols for _ in range(rows)]
            stk = [(sr, sc)]
            while stk:
                r, c = stk.pop()
                if visited[r][c]: continue
                visited[r][c] = True
                steps.append({"type": "VISIT", "r": r, "c": c})
                if r == er and c == ec:
                    path = []
                    cur = (r, c)
                    while cur:
                        path.append({"r": cur[0], "c": cur[1]})
                        cur = parent[cur[0]][cur[1]]
                    path.reverse()
                    return {"found": True, "path": path, "steps": steps}
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nr, nc = r+dr, c+dc
                    if 0<=nr<rows and 0<=nc<cols and not visited[nr][nc] and grid[nr][nc]!=1:
                        parent[nr][nc]=(r,c); stk.append((nr,nc))

        return {"found": False, "path": [], "steps": steps}


# ─── HAV API Layer ────────────────────────────────────────────────────────────

class HAVEngine:
    """
    High-level Python orchestration layer.
    Delegates to C++ via pybind11 when available, else pure-Python fallback.
    """

    def __init__(self):
        self._maze_fb    = None
        self._sort_fb    = _PythonSortFallback()
        self._path_fb    = _PythonPathFallback()

    # ── Maze ────────────────────────────────────────────────────────────────

    def generate_maze(self, rows: int, cols: int,
                      algorithm: MazeAlgorithm = MazeAlgorithm.GROWING_TREE,
                      strategy: GrowingTreeStrategy = GrowingTreeStrategy.NEWEST,
                      mix_ratio: float = 0.5) -> Dict:
        t0 = time.perf_counter()

        if NATIVE_AVAILABLE:
            engine = _cpp.MazeEngine(rows, cols)
            if algorithm == MazeAlgorithm.GROWING_TREE:
                strat_map = {
                    GrowingTreeStrategy.NEWEST: _cpp.GrowingTreeStrategy.NEWEST,
                    GrowingTreeStrategy.OLDEST: _cpp.GrowingTreeStrategy.OLDEST,
                    GrowingTreeStrategy.RANDOM: _cpp.GrowingTreeStrategy.RANDOM,
                    GrowingTreeStrategy.MIX:    _cpp.GrowingTreeStrategy.MIX,
                    GrowingTreeStrategy.MIDDLE: _cpp.GrowingTreeStrategy.MIDDLE,
                }
                engine.generateGrowingTree(strat_map[strategy], mix_ratio)
            elif algorithm == MazeAlgorithm.RECURSIVE_DIVISION:
                engine.generateRecursiveDivision()
            elif algorithm == MazeAlgorithm.WILSON:
                engine.generateWilson()
            elif algorithm == MazeAlgorithm.ALDOUS_BRODER:
                engine.generateAldousBroder()
            elif algorithm == MazeAlgorithm.SIDEWINDER:
                engine.generateSidewinder()
            elif algorithm == MazeAlgorithm.BINARY_TREE:
                engine.generateBinaryTree()
            elif algorithm == MazeAlgorithm.HUNT_AND_KILL:
                engine.generateHuntAndKill()
            elif algorithm == MazeAlgorithm.ELLER:
                engine.generateEller()
            elif algorithm == MazeAlgorithm.KRUSKAL_MAZE:
                engine.generateKruskalMaze()

            raw_steps = engine.steps()
            raw_grid  = engine.grid()

            steps = [{"type": s.type.name, "r": s.primary[0], "c": s.primary[1],
                      "r2": s.secondary[0], "c2": s.secondary[1], "walls": s.walls}
                     for s in raw_steps]
            grid = [[cell.walls for cell in row] for row in raw_grid]
        else:
            fb = _PythonMazeFallback(rows, cols)
            result = fb.generate(strategy, mix_ratio)
            steps = result["steps"]
            grid  = result["grid"]

        elapsed = (time.perf_counter() - t0) * 1000
        return {
            "ok": True, "rows": rows, "cols": cols,
            "algorithm": algorithm.value, "strategy": strategy.value,
            "grid": grid, "steps": steps,
            "stepCount": len(steps), "elapsedMs": round(elapsed, 3),
            "native": NATIVE_AVAILABLE
        }

    # ── Pathfinding ─────────────────────────────────────────────────────────

    def find_path(self, grid: List[List[int]],
                  sr: int, sc: int, er: int, ec: int,
                  algorithm: PathAlgorithm = PathAlgorithm.ASTAR,
                  heuristic: HeuristicType = HeuristicType.MANHATTAN,
                  allow_diagonal: bool = False) -> Dict:
        t0 = time.perf_counter()

        if NATIVE_AVAILABLE:
            engine = _cpp.PathEngine(grid)
            h_map = {
                HeuristicType.MANHATTAN: _cpp.HeuristicType.MANHATTAN,
                HeuristicType.EUCLIDEAN: _cpp.HeuristicType.EUCLIDEAN,
                HeuristicType.CHEBYSHEV: _cpp.HeuristicType.CHEBYSHEV,
                HeuristicType.OCTILE:    _cpp.HeuristicType.OCTILE,
            }
            h = h_map[heuristic]
            if   algorithm == PathAlgorithm.ASTAR:       res = engine.aStar(sr,sc,er,ec,h,allow_diagonal)
            elif algorithm == PathAlgorithm.DIJKSTRA:    res = engine.dijkstraImpl(sr,sc,er,ec)
            elif algorithm == PathAlgorithm.BFS:         res = engine.bfs(sr,sc,er,ec)
            elif algorithm == PathAlgorithm.DFS:         res = engine.dfs(sr,sc,er,ec)
            elif algorithm == PathAlgorithm.JPS:         res = engine.jps(sr,sc,er,ec)
            elif algorithm == PathAlgorithm.BIDIR_ASTAR: res = engine.bidirectionalAStar(sr,sc,er,ec,h)
            elif algorithm == PathAlgorithm.GREEDY_BFS:   res = engine.greedyBFS(sr,sc,er,ec,h)
            elif algorithm == PathAlgorithm.IDA_STAR:     res = engine.idaStar(sr,sc,er,ec,h)
            elif algorithm == PathAlgorithm.IDDFS:        res = engine.iddfs(sr,sc,er,ec)
            elif algorithm == PathAlgorithm.THETA_STAR:   res = engine.thetaStar(sr,sc,er,ec)
            elif algorithm == PathAlgorithm.BEAM_SEARCH:  res = engine.beamSearch(sr,sc,er,ec,5,h)
            elif algorithm == PathAlgorithm.BIDIR_BFS:    res = engine.bidirectionalBFS(sr,sc,er,ec)
            else: res = engine.aStar(sr,sc,er,ec,h,allow_diagonal)

            steps = [{"type": s.type.name, "r": s.primary[0], "c": s.primary[1],
                      "g": s.g, "h": s.h, "f": s.f} for s in res.steps]
            path  = [{"r": p[0], "c": p[1]} for p in res.path]
            found = res.found; cost = res.cost
        else:
            result = self._path_fb.run(algorithm, grid, sr, sc, er, ec, heuristic)
            steps = result["steps"]; path = result.get("path",[]); found = result["found"]; cost = result.get("cost",0)

        elapsed = (time.perf_counter() - t0) * 1000
        return {
            "ok": True, "found": found, "cost": cost,
            "algorithm": algorithm.value, "heuristic": heuristic.value,
            "path": path, "steps": steps,
            "stepCount": len(steps), "pathLength": len(path),
            "elapsedMs": round(elapsed, 3), "native": NATIVE_AVAILABLE
        }

    # ── Sorting ─────────────────────────────────────────────────────────────

    def sort(self, data: List[int], algorithm: SortAlgorithm = SortAlgorithm.QUICK) -> Dict:
        t0 = time.perf_counter()

        if NATIVE_AVAILABLE:
            engine = _cpp.SortEngine(data)
            dispatch = {
                SortAlgorithm.BUBBLE:    engine.runBubble,
                SortAlgorithm.INSERTION: engine.runInsertion,
                SortAlgorithm.SELECTION: engine.runSelection,
                SortAlgorithm.MERGE:     engine.runMerge,
                SortAlgorithm.QUICK:     engine.runQuick,
                SortAlgorithm.HEAP:      engine.runHeap,
                SortAlgorithm.SHELL:     engine.runShell,
                SortAlgorithm.RADIX:     engine.runRadix,
                SortAlgorithm.COUNTING:  engine.runCounting,
                SortAlgorithm.TIMSORT:   engine.runTimSort,
                SortAlgorithm.COCKTAIL:  engine.runCocktail,
                SortAlgorithm.GNOME:     engine.runGnome,
                SortAlgorithm.COMB:      engine.runComb,
                SortAlgorithm.CYCLE:     engine.runCycle,
                SortAlgorithm.PANCAKE:   engine.runPancake,
                SortAlgorithm.BITONIC:   engine.runBitonic,
                SortAlgorithm.ODDEVEN:   engine.runOddEven,
                SortAlgorithm.STOOGE:    engine.runStooge,
                SortAlgorithm.INTRO:     engine.runIntro,
            }
            dispatch[algorithm]()
            raw = engine.steps()
            steps = [{"type": s.type.name, "i": s.i, "j": s.j, "arr": list(s.arr),
                      "comparisons": s.comparisons, "swaps": s.swaps} for s in raw]
            comparisons = engine.comparisons(); swaps = engine.swaps()
            final = list(raw[-1].arr) if raw else sorted(data)
        else:
            result = self._sort_fb.run(algorithm, data)
            steps = result["steps"]; comparisons = result["comparisons"]
            swaps = result["swaps"]; final = result["sorted"]

        elapsed = (time.perf_counter() - t0) * 1000
        return {
            "ok": True, "algorithm": algorithm.value,
            "steps": steps, "comparisons": comparisons, "swaps": swaps,
            "sorted": final, "stepCount": len(steps),
            "elapsedMs": round(elapsed, 3), "native": NATIVE_AVAILABLE
        }

    # ── Graph ───────────────────────────────────────────────────────────────

    def run_graph(self, n: int, edges: List[Dict],
                  algorithm: GraphAlgorithm = GraphAlgorithm.KRUSKAL,
                  directed: bool = False, source: int = 0) -> Dict:
        t0 = time.perf_counter()

        if NATIVE_AVAILABLE:
            engine = _cpp.GraphEngine(n)
            for e in edges: engine.addEdge(e["u"], e["v"], e.get("w", 1.0), directed)
            dispatch = {
                GraphAlgorithm.KRUSKAL:      lambda: engine.kruskal(),
                GraphAlgorithm.PRIM:         lambda: engine.prim(source),
                GraphAlgorithm.BELLMAN_FORD: lambda: engine.bellmanFord(source),
                GraphAlgorithm.FLOYD:        lambda: engine.floydWarshall(),
                GraphAlgorithm.TARJAN:       lambda: engine.tarjanSCC(),
                GraphAlgorithm.TOPO:         lambda: engine.topologicalSort(),
                GraphAlgorithm.BORUVKA:      lambda: engine.boruvka(),
                GraphAlgorithm.DIJKSTRA_SSSP:lambda: engine.dijkstraSSSP(source),
                GraphAlgorithm.JOHNSON:      lambda: engine.johnson(),
                GraphAlgorithm.KOSARAJU:     lambda: engine.kosarajuSCC(),
                GraphAlgorithm.ARTIC_BRIDGE: lambda: engine.articulationAndBridges(),
                GraphAlgorithm.BIPARTITE:    lambda: engine.bipartiteCheck(),
                GraphAlgorithm.EULER:        lambda: engine.eulerCircuit(source),
                GraphAlgorithm.HAMILTONIAN:  lambda: engine.hamiltonianPath(source),
                GraphAlgorithm.COLORING:     lambda: engine.greedyColoring(),
                GraphAlgorithm.FORD_FULKERSON:lambda: engine.fordFulkerson(source, min(source+1,n-1)),
            }
            res = dispatch[algorithm]()
            steps = [{"type": s.type.name, "u": s.primary[0], "v": s.secondary[0],
                      "g": s.g} for s in res.steps]
            mst = [{"u": e.u, "v": e.v, "w": e.weight} for e in res.mstEdges]
            result = {"steps": steps, "mst": mst, "totalWeight": res.totalWeight,
                      "hasNegCycle": res.hasNegCycle,
                      "components": list(res.components) if res.components else [],
                      "topoOrder": list(res.topoOrder)}
        else:
            result = {"steps": [], "mst": [], "totalWeight": 0, "hasNegCycle": False,
                      "components": [], "topoOrder": []}

        elapsed = (time.perf_counter() - t0) * 1000
        return {
            "ok": True, "algorithm": algorithm.value,
            "elapsedMs": round(elapsed, 3), "native": NATIVE_AVAILABLE,
            **result
        }

    def status(self) -> Dict:
        return {
            "native": NATIVE_AVAILABLE,
            "version": "2.0.0",
            "algorithms": {
                "maze":    [a.value for a in MazeAlgorithm],
                "path":    [a.value for a in PathAlgorithm],
                "sort":    [a.value for a in SortAlgorithm],
                "graph":   [a.value for a in GraphAlgorithm],
            }
        }


# ─── HTTP Request Handler ─────────────────────────────────────────────────────

_engine = HAVEngine()

class HAVHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass  # suppress default logging

    def _send_json(self, data: dict, status: int = 200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/status":
            self._send_json(_engine.status())
        else:
            self._send_json({"error": "not found"}, 404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length)) if length else {}
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        try:
            if path == "/api/maze":
                result = _engine.generate_maze(
                    rows=body.get("rows", 21),
                    cols=body.get("cols", 21),
                    algorithm=MazeAlgorithm(body.get("algorithm", "growing_tree")),
                    strategy=GrowingTreeStrategy(body.get("strategy", "NEWEST")),
                    mix_ratio=float(body.get("mixRatio", 0.5))
                )
            elif path == "/api/path":
                result = _engine.find_path(
                    grid=body["grid"],
                    sr=body["sr"], sc=body["sc"],
                    er=body["er"], ec=body["ec"],
                    algorithm=PathAlgorithm(body.get("algorithm", "astar")),
                    heuristic=HeuristicType(body.get("heuristic", "MANHATTAN")),
                    allow_diagonal=bool(body.get("diagonal", False))
                )
            elif path == "/api/sort":
                result = _engine.sort(
                    data=body["data"],
                    algorithm=SortAlgorithm(body.get("algorithm", "quick"))
                )
            elif path == "/api/graph":
                result = _engine.run_graph(
                    n=body["n"],
                    edges=body.get("edges", []),
                    algorithm=GraphAlgorithm(body.get("algorithm", "kruskal")),
                    directed=bool(body.get("directed", False)),
                    source=int(body.get("source", 0))
                )
            else:
                result = {"error": "unknown endpoint"}

            self._send_json(result)

        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, 500)


def start_server(host: str = "127.0.0.1", port: int = 7432):
    srv = HTTPServer((host, port), HAVHandler)
    print(f"[HAV] API server: http://{host}:{port}  (native C++: {NATIVE_AVAILABLE})")
    srv.serve_forever()


if __name__ == "__main__":
    start_server()
