# HAV Framework

**Heuristic & Algorithms Visualizer** — a high-performance algorithm visualization engine with a C++ core, Python REST API, and browser-based frontend.

---

## Architecture

```
HAV/
├── core/
│   ├── hav_engine.hpp      # C++ engine header
│   ├── hav_engine.cpp      # C++ engine implementation
│   ├── bindings.cpp        # pybind11 Python bindings
│   └── hav_api.py          # Python HTTP API server
├── cli/
│   └── main_cli.cpp        # Standalone CLI benchmark binary
├── frontend/
│   └── index.html          # Browser visualizer
├── CMakeLists.txt
├── build.bat               # Windows build script
└── launch.ps1              # Windows launcher
```

The engine runs in one of two modes:

- **Native mode** — `hav_api.py` imports `hav_native.pyd` (the compiled C++ module). All algorithms run at full C++ speed.
- **Fallback mode** — if the `.pyd` has not been compiled, `hav_api.py` falls back to pure-Python implementations automatically. All endpoints remain functional.

---

## Requirements

| Dependency | Version | Notes |
|---|---|---|
| CMake | ≥ 3.18 | [cmake.org](https://cmake.org/download/) |
| C++ compiler | MSVC 2019/2022/2026 **or** MinGW-w64 GCC | MSVC recommended |
| Python | ≥ 3.8 | [python.org](https://python.org) |
| pybind11 | ≥ 2.12 | installed automatically by `build.bat` if missing |

---

## Building

Run from the project root in a terminal where your compiler is visible (e.g., a Visual Studio Developer Command Prompt, or any shell where `g++` is on PATH):

```bat
build.bat
```

The script auto-detects MSVC or MinGW, configures CMake, and builds two outputs:

| Output | Location | Purpose |
|---|---|---|
| `hav_native*.pyd` | `core/` | Python extension — imported by `hav_api.py` |
| `hav_cli.exe` | project root | Standalone CLI benchmark |

---

## Running

### Option 1 — PowerShell launcher (recommended)

Starts the API server and opens the visualizer in your default browser:

```powershell
.\launch.ps1
```

Options:

```powershell
.\launch.ps1 -Port 8080          # use a custom port (default: 7432)
.\launch.ps1 -NoServer           # open frontend in pure-JS mode, no server
```

### Option 2 — Manual

Start the API server:

```bat
cd core
python hav_api.py
```

Then open `frontend/index.html` in a browser. Set `USE_API = true` at the top of the HTML to connect it to the local server, or leave it `false` to run entirely in-browser JS.

### Option 3 — CLI benchmark

```bat
hav_cli.exe           # run all benchmarks
hav_cli.exe maze      # maze generation only
hav_cli.exe path      # pathfinding only
hav_cli.exe sort      # sorting only
hav_cli.exe graph     # graph algorithms only
```

---

## API Reference

The server listens on `http://127.0.0.1:7432` by default. All endpoints accept and return JSON.

### `GET /api/status`

Returns engine status and available algorithms.

```json
{
  "native": true,
  "version": "2.0.0",
  "algorithms": {
    "maze":  ["growing_tree", "recursive_division", "wilson"],
    "path":  ["astar", "dijkstra", "bfs", "dfs", "jps", "bidir_astar"],
    "sort":  ["bubble", "insertion", "selection", "merge", "quick", "heap", "shell", "radix", "counting", "timsort"],
    "graph": ["kruskal", "prim", "bellman_ford", "floyd_warshall", "tarjan", "topological"]
  }
}
```

---

### `POST /api/maze`

Generate a maze and return step-by-step animation data.

```json
{
  "rows": 21,
  "cols": 21,
  "algorithm": "growing_tree",
  "strategy": "NEWEST",
  "mixRatio": 0.5
}
```

**`algorithm`** — `growing_tree` | `recursive_division` | `wilson`

**`strategy`** (Growing Tree only) — controls cell selection behavior:

| Value | Behavior |
|---|---|
| `NEWEST` | Always pick most recent cell → DFS-like, winding passages |
| `OLDEST` | Always pick oldest cell → BFS-like, wide open mazes |
| `RANDOM` | Pick random cell → Prim's-like, more uniform |
| `MIX` | Blend of NEWEST and RANDOM, controlled by `mixRatio` |
| `MIDDLE` | Pick middle of active list |

Response includes `grid` (wall bitmask per cell) and `steps` (animation sequence with types `VISIT`, `CARVE`, `BACKTRACK`, `WALL`, `DONE`).

---

### `POST /api/path`

Run a pathfinding algorithm on a grid.

```json
{
  "grid": [[0, 0, 1, 0], [0, 0, 0, 0]],
  "sr": 0, "sc": 0,
  "er": 1, "ec": 3,
  "algorithm": "astar",
  "heuristic": "MANHATTAN",
  "diagonal": false
}
```

**`algorithm`** — `astar` | `dijkstra` | `bfs` | `dfs` | `jps` | `bidir_astar`

**`heuristic`** (A* variants) — `MANHATTAN` | `EUCLIDEAN` | `CHEBYSHEV` | `OCTILE`

**Cell values** — `0` empty, `1` wall, `2` weighted (cost ×3), `3` mud

Response includes `found`, `cost`, `path` (list of `[row, col]`), and `steps` for animation.

---

### `POST /api/sort`

Sort an array and return every comparison and swap as animation steps.

```json
{
  "data": [38, 27, 43, 3, 9, 82, 10],
  "algorithm": "merge"
}
```

**`algorithm`** — `bubble` | `insertion` | `selection` | `merge` | `quick` | `heap` | `shell` | `radix` | `counting` | `timsort`

Response includes `sorted`, `comparisons`, `swaps`, `stepCount`, `elapsedMs`, and the full `steps` array.

---

### `POST /api/graph`

Run a graph algorithm and return steps with result metadata.

```json
{
  "n": 6,
  "edges": [
    {"u": 0, "v": 1, "w": 4.0},
    {"u": 0, "v": 2, "w": 2.0},
    {"u": 1, "v": 3, "w": 5.0}
  ],
  "algorithm": "kruskal",
  "directed": false,
  "source": 0
}
```

**`algorithm`** — `kruskal` | `prim` | `bellman_ford` | `floyd_warshall` | `tarjan` | `topological`

Response includes `steps`, `mst` edges (for MST algorithms), `totalWeight`, `hasNegCycle`, `components`, and `topoOrder`.

---

## Algorithms Implemented

### Maze Generation

| Algorithm | Notes |
|---|---|
| Growing Tree | 5 strategies: NEWEST / OLDEST / RANDOM / MIX / MIDDLE |
| Recursive Division | Adds walls recursively with random passages |
| Wilson's | Uniform spanning tree via loop-erased random walk |
| Aldous-Broder | Uniform spanning tree, simpler random walk |
| Sidewinder | Row-by-row carving |
| Binary Tree | Simplest possible maze, strong diagonal bias |
| Hunt-and-Kill | DFS with hunt phase for unvisited cells |
| Eller's | Row-by-row, generates perfect mazes in linear space |
| Kruskal (maze) | Randomized Kruskal on cell graph |

### Pathfinding

| Algorithm | Notes |
|---|---|
| A\* | Configurable heuristic, optional diagonal movement |
| Dijkstra | Uniform-cost search, respects cell weights |
| BFS | Unweighted shortest path |
| DFS | Explores deeply, non-optimal |
| Jump Point Search | A\* acceleration for uniform grids |
| Bidirectional A\* | Simultaneous forward/backward search |
| Greedy BFS | Heuristic-only, fast but non-optimal |
| IDA\* | Iterative deepening A\*, memory-efficient |
| IDDFS | Iterative deepening DFS |
| Theta\* | Any-angle pathfinding with line-of-sight |
| Beam Search | Width-limited BFS |
| Bidirectional BFS | Meets in the middle, unweighted |

### Sorting

| Algorithm |
|---|
| Bubble, Cocktail Shaker |
| Insertion, Shell |
| Selection |
| Merge |
| Quick |
| Heap |
| Tim Sort |
| Radix (LSD) |
| Counting |
| Gnome, Comb, Cycle, Pancake, Odd-Even, Bitonic, Stooge, Introsort |

### Graph Algorithms

| Algorithm | Output |
|---|---|
| Kruskal MST | Minimum spanning tree |
| Prim MST | Minimum spanning tree |
| Borůvka MST | Minimum spanning tree |
| Dijkstra SSSP | Single-source shortest paths |
| Bellman-Ford | SSSP with negative edge support |
| Floyd-Warshall | All-pairs shortest paths |
| Johnson's | All-pairs, sparse graphs |
| Tarjan SCC | Strongly connected components |
| Kosaraju SCC | Strongly connected components |
| Topological Sort | DFS-based ordering |
| Articulation Points & Bridges | Cut vertices and edges |
| Bipartite Check | BFS 2-coloring |
| Euler Circuit | Hierholzer's algorithm |
| Hamiltonian Path | Backtracking |
| Greedy Coloring | Approximate chromatic number |
| Ford-Fulkerson | Maximum flow (Edmonds-Karp BFS) |

---

## Build Troubleshooting

**MSVC version not detected correctly** — the build script reads the compiler banner to pick the VS generator. If it defaults to `Visual Studio 18 2026` when you have an older VS, pass `-G` manually:

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

**pybind11 fetch fails behind a proxy** — install pybind11 manually first:

```bat
pip install pybind11
```

CMake will find it via `find_package(pybind11)` and skip the FetchContent step.

**`.pyd` not found at runtime** — the build copies `hav_native*.pyd` into `core/`. If Python still cannot import it, make sure you are running `hav_api.py` from inside the `core/` directory, or that `core/` is on `sys.path`.

**MinGW build** — ensure `g++`, `cmake`, and `python` are all on PATH. Run from a regular Command Prompt or PowerShell, not a VS Developer Prompt.
