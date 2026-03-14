/**
 * HAV Engine Header v3.0
 */
#pragma once
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <functional>
#include <string>
#include <utility>
#include <limits>

namespace hav {

constexpr int CELL_EMPTY=0,CELL_WALL=1,CELL_WEIGHT=2,CELL_MUD=3;
constexpr int WALL_N=1,WALL_S=2,WALL_W=4,WALL_E=8;

enum class StepType { VISIT,OPEN,CLOSED,PATH,BACKTRACK,CARVE,WALL,PIVOT,DONE,COMPARE,SWAP,SELECT,BRIDGE,ARTICULATION };
enum class HeuristicType { MANHATTAN,EUCLIDEAN,CHEBYSHEV,OCTILE,ZERO };
enum class GrowingTreeStrategy { NEWEST,OLDEST,RANDOM,MIX,MIDDLE };
enum class SortStepType { COMPARE,SWAP,SELECT,INSERT,SHIFT,MERGE,PIVOT,MARK,DONE };

struct Cell { int row,col,walls; bool visited,isStart,isEnd; double weight; };
struct AlgoStep {
    StepType type;
    std::pair<int,int> primary,secondary;
    int walls=0; double g=0,h=0,f=0;
    std::vector<int> extra;
    AlgoStep():primary({-1,-1}),secondary({-1,-1}){}
};
struct PathResult { bool found=false; double cost=0; std::vector<std::pair<int,int>> path; std::vector<AlgoStep> steps; };
struct SortStep { SortStepType type; int i=-1,j=-1; std::vector<int> arr; long long comparisons=0,swaps=0; };
struct MSTEdge { int u,v; double weight; };
struct GraphResult {
    double totalWeight=0,maxFlow=0;
    bool hasNegCycle=false,isBipartite=false,hasEuler=false,hasHamiltonian=false;
    int chromaticNum=0;
    std::vector<MSTEdge> mstEdges;
    std::vector<AlgoStep> steps;
    std::vector<double> dist;
    std::vector<int> components,topoOrder,eulerCircuit,hamiltonPath,coloring,articulationPoints;
    std::vector<std::pair<int,int>> bridges;
};

class MazeEngine {
public:
    MazeEngine(int rows,int cols);
    void reset();
    void generateGrowingTree(GrowingTreeStrategy s=GrowingTreeStrategy::NEWEST,double mix=0.5);
    void generateRecursiveDivision(int r0=0,int c0=0,int r1=-1,int c1=-1);
    void generateWilson();
    void generateAldousBroder();
    void generateSidewinder();
    void generateBinaryTree();
    void generateHuntAndKill();
    void generateEller();
    void generateKruskalMaze();
    const std::vector<std::vector<Cell>>& grid()  const{return grid_;}
    const std::vector<AlgoStep>&          steps() const{return steps_;}
    int rows()const{return rows_;}int cols()const{return cols_;}
private:
    int rows_,cols_;
    std::vector<std::vector<Cell>> grid_;
    std::vector<AlgoStep> steps_;
    AlgoStep buildStep(StepType t,int r,int c,int r2=-1,int c2=-1)const;
    void carvePassage(int r,int c,int nr,int nc);
};

class PathEngine {
public:
    explicit PathEngine(const std::vector<std::vector<int>>& grid);
    PathResult aStar(int sr,int sc,int er,int ec,HeuristicType h=HeuristicType::MANHATTAN,bool diag=false);
    PathResult dijkstraImpl(int sr,int sc,int er,int ec);
    PathResult bfs(int sr,int sc,int er,int ec);
    PathResult dfs(int sr,int sc,int er,int ec);
    PathResult jps(int sr,int sc,int er,int ec);
    PathResult bidirectionalAStar(int sr,int sc,int er,int ec,HeuristicType h=HeuristicType::MANHATTAN);
    PathResult greedyBFS(int sr,int sc,int er,int ec,HeuristicType h=HeuristicType::MANHATTAN);
    PathResult idaStar(int sr,int sc,int er,int ec,HeuristicType h=HeuristicType::MANHATTAN);
    PathResult iddfs(int sr,int sc,int er,int ec);
    PathResult thetaStar(int sr,int sc,int er,int ec);
    PathResult beamSearch(int sr,int sc,int er,int ec,int beamWidth=5,HeuristicType h=HeuristicType::MANHATTAN);
    PathResult bidirectionalBFS(int sr,int sc,int er,int ec);
    void updateGrid(const std::vector<std::vector<int>>& g){grid_=g;}
private:
    std::vector<std::vector<int>> grid_;
    int rows_,cols_;
    bool lineOfSight(int r1,int c1,int r2,int c2)const;
    bool idaDFS(int r,int c,int er,int ec,double g,double bound,HeuristicType h,
                std::vector<std::vector<double>>& dist,
                std::vector<std::vector<std::pair<int,int>>>& par,
                std::vector<AlgoStep>& steps,double& next)const;
};

class SortEngine {
public:
    explicit SortEngine(std::vector<int> data);
    void reset(std::vector<int> data);
    void runBubble();void runInsertion();void runSelection();
    void runMerge();void runQuick();void runHeap();
    void runShell();void runRadix();void runCounting();void runTimSort();
    void runCocktail();void runGnome();void runComb();
    void runCycle();void runPancake();void runBitonic();
    void runOddEven();void runStooge();void runIntro();
    const std::vector<SortStep>& steps()      const{return steps_;}
    long long                    comparisons() const{return comparisons_;}
    long long                    swaps()       const{return swaps_;}
private:
    std::vector<int> data_;
    std::vector<SortStep> steps_;
    long long comparisons_=0,swaps_=0;
    void rec(SortStepType t,int i,int j,const std::vector<int>& a);
    void recordStep(SortStepType t,int i,int j,const std::vector<int>& arr);
    void mergeSort(std::vector<int>& a,int lo,int hi);
    void quickSort(std::vector<int>& a,int lo,int hi);
    int  partition(std::vector<int>& a,int lo,int hi);
    void heapify(std::vector<int>& a,int n,int i);
    void bitonicMerge(std::vector<int>& a,int lo,int cnt,bool asc);
    void bitonicSort(std::vector<int>& a,int lo,int cnt,bool asc);
    void stoogeSort(std::vector<int>& a,int lo,int hi);
    void introSort(std::vector<int>& a,int lo,int hi,int depth);
    void flip(std::vector<int>& a,int k);
};

class GraphEngine {
public:
    explicit GraphEngine(int nodes);
    void addEdge(int u,int v,double w=1.0,bool directed=false);
    void clear();
    GraphResult kruskal();GraphResult prim(int s=0);GraphResult boruvka();
    GraphResult bellmanFord(int src);GraphResult floydWarshall();
    GraphResult johnson();GraphResult dijkstraSSSP(int src);
    GraphResult tarjanSCC();GraphResult kosarajuSCC();
    GraphResult topologicalSort();GraphResult articulationAndBridges();
    GraphResult bipartiteCheck();
    GraphResult eulerCircuit(int start=0);GraphResult hamiltonianPath(int start=0);
    GraphResult greedyColoring();GraphResult fordFulkerson(int source,int sink);
    int nodeCount()const{return n_;}
private:
    int n_;
    std::vector<std::vector<std::pair<int,double>>> adj_;
    bool bfsAug(const std::vector<std::vector<double>>& cap,int s,int t,std::vector<int>& par)const;
    void apDFS(int u,int p,int& timer,std::vector<int>& disc,std::vector<int>& low,
               std::vector<bool>& vis,GraphResult& res)const;
    void kosarajuDFS1(int u,std::vector<bool>& vis,std::stack<int>& st)const;
    void kosarajuDFS2(int u,int comp,std::vector<bool>& vis,std::vector<int>& comps,
                      const std::vector<std::vector<int>>& radj)const;
};

} // namespace hav
