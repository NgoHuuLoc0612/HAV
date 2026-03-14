/**
 * HAV pybind11 bindings v3.0
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "hav_engine.hpp"
namespace py=pybind11;
using namespace hav;

PYBIND11_MODULE(hav_native,m){
    m.doc()="HAV native C++ engine v3.0";

    py::enum_<StepType>(m,"StepType")
        .value("VISIT",StepType::VISIT).value("OPEN",StepType::OPEN)
        .value("CLOSED",StepType::CLOSED).value("PATH",StepType::PATH)
        .value("BACKTRACK",StepType::BACKTRACK).value("CARVE",StepType::CARVE)
        .value("WALL",StepType::WALL).value("PIVOT",StepType::PIVOT)
        .value("DONE",StepType::DONE).value("COMPARE",StepType::COMPARE)
        .value("SWAP",StepType::SWAP).value("SELECT",StepType::SELECT)
        .value("BRIDGE",StepType::BRIDGE).value("ARTICULATION",StepType::ARTICULATION);

    py::enum_<HeuristicType>(m,"HeuristicType")
        .value("MANHATTAN",HeuristicType::MANHATTAN).value("EUCLIDEAN",HeuristicType::EUCLIDEAN)
        .value("CHEBYSHEV",HeuristicType::CHEBYSHEV).value("OCTILE",HeuristicType::OCTILE)
        .value("ZERO",HeuristicType::ZERO);

    py::enum_<GrowingTreeStrategy>(m,"GrowingTreeStrategy")
        .value("NEWEST",GrowingTreeStrategy::NEWEST).value("OLDEST",GrowingTreeStrategy::OLDEST)
        .value("RANDOM",GrowingTreeStrategy::RANDOM).value("MIX",GrowingTreeStrategy::MIX)
        .value("MIDDLE",GrowingTreeStrategy::MIDDLE);

    py::enum_<SortStepType>(m,"SortStepType")
        .value("COMPARE",SortStepType::COMPARE).value("SWAP",SortStepType::SWAP)
        .value("SELECT",SortStepType::SELECT).value("INSERT",SortStepType::INSERT)
        .value("SHIFT",SortStepType::SHIFT).value("MERGE",SortStepType::MERGE)
        .value("PIVOT",SortStepType::PIVOT).value("MARK",SortStepType::MARK)
        .value("DONE",SortStepType::DONE);

    py::class_<Cell>(m,"Cell")
        .def_readonly("row",&Cell::row).def_readonly("col",&Cell::col)
        .def_readonly("walls",&Cell::walls).def_readonly("visited",&Cell::visited)
        .def_readonly("isStart",&Cell::isStart).def_readonly("isEnd",&Cell::isEnd);

    py::class_<AlgoStep>(m,"AlgoStep")
        .def_readonly("type",&AlgoStep::type).def_readonly("primary",&AlgoStep::primary)
        .def_readonly("secondary",&AlgoStep::secondary).def_readonly("walls",&AlgoStep::walls)
        .def_readonly("g",&AlgoStep::g).def_readonly("h",&AlgoStep::h).def_readonly("f",&AlgoStep::f);

    py::class_<PathResult>(m,"PathResult")
        .def_readonly("found",&PathResult::found).def_readonly("cost",&PathResult::cost)
        .def_readonly("path",&PathResult::path).def_readonly("steps",&PathResult::steps);

    py::class_<SortStep>(m,"SortStep")
        .def_readonly("type",&SortStep::type).def_readonly("i",&SortStep::i)
        .def_readonly("j",&SortStep::j).def_readonly("arr",&SortStep::arr)
        .def_readonly("comparisons",&SortStep::comparisons).def_readonly("swaps",&SortStep::swaps);

    py::class_<MSTEdge>(m,"MSTEdge")
        .def_readonly("u",&MSTEdge::u).def_readonly("v",&MSTEdge::v)
        .def_readonly("weight",&MSTEdge::weight);

    py::class_<GraphResult>(m,"GraphResult")
        .def_readonly("totalWeight",&GraphResult::totalWeight)
        .def_readonly("maxFlow",&GraphResult::maxFlow)
        .def_readonly("hasNegCycle",&GraphResult::hasNegCycle)
        .def_readonly("isBipartite",&GraphResult::isBipartite)
        .def_readonly("hasEuler",&GraphResult::hasEuler)
        .def_readonly("hasHamiltonian",&GraphResult::hasHamiltonian)
        .def_readonly("chromaticNum",&GraphResult::chromaticNum)
        .def_readonly("mstEdges",&GraphResult::mstEdges)
        .def_readonly("steps",&GraphResult::steps)
        .def_readonly("dist",&GraphResult::dist)
        .def_readonly("components",&GraphResult::components)
        .def_readonly("topoOrder",&GraphResult::topoOrder)
        .def_readonly("eulerCircuit",&GraphResult::eulerCircuit)
        .def_readonly("hamiltonPath",&GraphResult::hamiltonPath)
        .def_readonly("coloring",&GraphResult::coloring)
        .def_readonly("articulationPoints",&GraphResult::articulationPoints)
        .def_readonly("bridges",&GraphResult::bridges);

    py::class_<MazeEngine>(m,"MazeEngine")
        .def(py::init<int,int>())
        .def("reset",&MazeEngine::reset)
        .def("generateGrowingTree",&MazeEngine::generateGrowingTree,
             py::arg("strategy")=GrowingTreeStrategy::NEWEST,py::arg("mixRatio")=0.5)
        .def("generateRecursiveDivision",&MazeEngine::generateRecursiveDivision,
             py::arg("r0")=0,py::arg("c0")=0,py::arg("r1")=-1,py::arg("c1")=-1)
        .def("generateWilson",&MazeEngine::generateWilson)
        .def("generateAldousBroder",&MazeEngine::generateAldousBroder)
        .def("generateSidewinder",&MazeEngine::generateSidewinder)
        .def("generateBinaryTree",&MazeEngine::generateBinaryTree)
        .def("generateHuntAndKill",&MazeEngine::generateHuntAndKill)
        .def("generateEller",&MazeEngine::generateEller)
        .def("generateKruskalMaze",&MazeEngine::generateKruskalMaze)
        .def("grid",&MazeEngine::grid,py::return_value_policy::reference)
        .def("steps",&MazeEngine::steps,py::return_value_policy::reference)
        .def("rows",&MazeEngine::rows).def("cols",&MazeEngine::cols);

    py::class_<PathEngine>(m,"PathEngine")
        .def(py::init<const std::vector<std::vector<int>>&>())
        .def("aStar",&PathEngine::aStar,py::arg("sr"),py::arg("sc"),py::arg("er"),py::arg("ec"),
             py::arg("h")=HeuristicType::MANHATTAN,py::arg("allowDiag")=false)
        .def("dijkstraImpl",&PathEngine::dijkstraImpl)
        .def("bfs",&PathEngine::bfs)
        .def("dfs",&PathEngine::dfs)
        .def("jps",&PathEngine::jps)
        .def("bidirectionalAStar",&PathEngine::bidirectionalAStar)
        .def("greedyBFS",&PathEngine::greedyBFS,
             py::arg("sr"),py::arg("sc"),py::arg("er"),py::arg("ec"),
             py::arg("h")=HeuristicType::MANHATTAN)
        .def("idaStar",&PathEngine::idaStar,
             py::arg("sr"),py::arg("sc"),py::arg("er"),py::arg("ec"),
             py::arg("h")=HeuristicType::MANHATTAN)
        .def("iddfs",&PathEngine::iddfs)
        .def("thetaStar",&PathEngine::thetaStar)
        .def("beamSearch",&PathEngine::beamSearch,
             py::arg("sr"),py::arg("sc"),py::arg("er"),py::arg("ec"),
             py::arg("beamWidth")=5,py::arg("h")=HeuristicType::MANHATTAN)
        .def("bidirectionalBFS",&PathEngine::bidirectionalBFS)
        .def("updateGrid",&PathEngine::updateGrid);

    py::class_<SortEngine>(m,"SortEngine")
        .def(py::init<std::vector<int>>())
        .def("reset",&SortEngine::reset)
        .def("runBubble",&SortEngine::runBubble).def("runInsertion",&SortEngine::runInsertion)
        .def("runSelection",&SortEngine::runSelection).def("runMerge",&SortEngine::runMerge)
        .def("runQuick",&SortEngine::runQuick).def("runHeap",&SortEngine::runHeap)
        .def("runShell",&SortEngine::runShell).def("runRadix",&SortEngine::runRadix)
        .def("runCounting",&SortEngine::runCounting).def("runTimSort",&SortEngine::runTimSort)
        .def("runCocktail",&SortEngine::runCocktail).def("runGnome",&SortEngine::runGnome)
        .def("runComb",&SortEngine::runComb).def("runCycle",&SortEngine::runCycle)
        .def("runPancake",&SortEngine::runPancake).def("runBitonic",&SortEngine::runBitonic)
        .def("runOddEven",&SortEngine::runOddEven).def("runStooge",&SortEngine::runStooge)
        .def("runIntro",&SortEngine::runIntro)
        .def("steps",&SortEngine::steps,py::return_value_policy::reference)
        .def("comparisons",&SortEngine::comparisons).def("swaps",&SortEngine::swaps);

    py::class_<GraphEngine>(m,"GraphEngine")
        .def(py::init<int>())
        .def("addEdge",&GraphEngine::addEdge,py::arg("u"),py::arg("v"),
             py::arg("w")=1.0,py::arg("directed")=false)
        .def("clear",&GraphEngine::clear)
        .def("kruskal",&GraphEngine::kruskal)
        .def("prim",&GraphEngine::prim,py::arg("start")=0)
        .def("boruvka",&GraphEngine::boruvka)
        .def("bellmanFord",&GraphEngine::bellmanFord)
        .def("floydWarshall",&GraphEngine::floydWarshall)
        .def("johnson",&GraphEngine::johnson)
        .def("dijkstraSSSP",&GraphEngine::dijkstraSSSP)
        .def("tarjanSCC",&GraphEngine::tarjanSCC)
        .def("kosarajuSCC",&GraphEngine::kosarajuSCC)
        .def("topologicalSort",&GraphEngine::topologicalSort)
        .def("articulationAndBridges",&GraphEngine::articulationAndBridges)
        .def("bipartiteCheck",&GraphEngine::bipartiteCheck)
        .def("eulerCircuit",&GraphEngine::eulerCircuit,py::arg("start")=0)
        .def("hamiltonianPath",&GraphEngine::hamiltonianPath,py::arg("start")=0)
        .def("greedyColoring",&GraphEngine::greedyColoring)
        .def("fordFulkerson",&GraphEngine::fordFulkerson)
        .def("nodeCount",&GraphEngine::nodeCount);
}
