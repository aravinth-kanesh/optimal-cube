// Performance benchmarks for the cube solver
#include "../src/cube.h"
#include "../src/moves.h"
#include "../src/solver.h"
#include "../src/pattern_db.h"
#include "../src/heuristic.h"
#include "../src/utils.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <string>
#include <cstring>

struct BenchResult {
    std::string scramble;
    int solution_length;
    uint64_t nodes;
    double seconds;
};

static BenchResult run_benchmark(IDASolver& solver, const std::string& scramble_str) {
    auto moves = parse_move_sequence(scramble_str);
    CubeState state = apply_moves(SOLVED_CUBE, moves);

    auto result = solver.solve(state, 20);

    return BenchResult{
        scramble_str,
        result.found ? (int)result.moves.size() : -1,
        result.nodes_explored,
        result.time_seconds
    };
}

int main(int argc, char* argv[]) {
    std::string data_dir = "data";
    bool use_db = true;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--data-dir") == 0 && i + 1 < argc) {
            data_dir = argv[++i];
        } else if (strcmp(argv[i], "--no-db") == 0) {
            use_db = false;
        }
    }

    init_moves();

    std::function<int(const CubeState&)> hfn;
    PatternDatabases dbs;

    if (use_db) {
        bool loaded = dbs.load_or_build(data_dir);
        if (!loaded) {
            std::cerr << "Pattern databases built (first run)\n";
        }
        hfn = make_heuristic(dbs);
    } else {
        std::cerr << "Using simple misplaced heuristic\n";
        hfn = heuristic_misplaced;
    }

    IDASolver solver(hfn);

    // Benchmark cases: known scrambles at various depths
    struct TestCase {
        std::string name;
        std::string scramble;
    };

    std::vector<TestCase> cases = {
        {"3 moves",  "R U F"},
        {"5 moves",  "R U F D B"},
        {"6 moves",  "R U F D' B' L"},
        {"8 moves",  "R U R' U' F2 D' L B"},
        {"10 moves", "R U R' U' R' F R2 U' R'"},
        {"12 moves", "R U2 R' U' R U' R' L' U2 L U L' U L"},
        // Superflip-adjacent (hard cases)
        {"15 moves (random)", "R U2 R' F2 R U R' U' F' U' F R2 U' R' U"},
    };

    std::cout << "\n=== Rubik's Cube Solver Benchmark ===\n";
    std::cout << "Heuristic: " << (use_db ? "Pattern Databases" : "Misplaced Cubies") << "\n\n";

    std::cout << std::left
              << std::setw(20) << "Case"
              << std::setw(8)  << "Moves"
              << std::setw(14) << "Nodes"
              << std::setw(12) << "Time(s)"
              << "\n"
              << std::string(54, '-') << "\n";

    double total_time = 0;
    for (const auto& tc : cases) {
        auto r = run_benchmark(solver, tc.scramble);
        total_time += r.seconds;

        std::cout << std::left
                  << std::setw(20) << tc.name
                  << std::setw(8)  << (r.solution_length >= 0 ? std::to_string(r.solution_length) : "FAIL")
                  << std::setw(14) << r.nodes
                  << std::setw(12) << std::fixed << std::setprecision(3) << r.seconds
                  << "\n";
    }

    std::cout << std::string(54, '-') << "\n";
    std::cout << "Total time: " << std::fixed << std::setprecision(3) << total_time << "s\n";

    // Test random scrambles at increasing depths
    std::cout << "\n=== Random Scramble Benchmarks ===\n";
    std::cout << std::left
              << std::setw(20) << "Depth"
              << std::setw(8)  << "Moves"
              << std::setw(14) << "Nodes"
              << std::setw(12) << "Time(s)"
              << "\n"
              << std::string(54, '-') << "\n";

    for (int depth : {8, 10, 12}) {
        auto scramble = random_scramble(depth, 12345);
        std::string scramble_str = format_move_sequence(scramble);
        CubeState state = apply_moves(SOLVED_CUBE, scramble);
        auto result = solver.solve(state, 20);

        std::cout << std::left
                  << std::setw(20) << ("depth=" + std::to_string(depth))
                  << std::setw(8)  << (result.found ? std::to_string((int)result.moves.size()) : "FAIL")
                  << std::setw(14) << result.nodes_explored
                  << std::setw(12) << std::fixed << std::setprecision(3) << result.time_seconds
                  << "\n";
    }

    return 0;
}
