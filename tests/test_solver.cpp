// Unit tests for IDA* solver correctness
#include "../src/cube.h"
#include "../src/moves.h"
#include "../src/solver.h"
#include "../src/utils.h"
#include <iostream>
#include <cassert>
#include <sstream>
#include <vector>
#include <string>
#include <tuple>

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
    g_tests_run++; \
    std::cout << "  " #name "... "; \
    try { test_##name(); std::cout << "PASS\n"; g_tests_passed++; } \
    catch (const std::exception& e) { std::cout << "FAIL: " << e.what() << "\n"; g_tests_failed++; } \
    catch (...) { std::cout << "FAIL: unknown exception\n"; g_tests_failed++; } \
} while(0)

#define ASSERT(cond) do { if (!(cond)) throw std::runtime_error("Assertion failed: " #cond " at line " + std::to_string(__LINE__)); } while(0)
#define ASSERT_EQ(a, b) do { if ((a) != (b)) { std::ostringstream ss; ss << "Expected " << (b) << " got " << (a) << " at line " << __LINE__; throw std::runtime_error(ss.str()); } } while(0)

// Test solving with misplaced-cubies heuristic (weak but always works)
IDASolver* g_solver = nullptr;

// Solve and verify: scramble, solve, check result
static void solve_and_verify(const std::string& scramble_str, int expected_max_len = 20) {
    auto moves = parse_move_sequence(scramble_str);
    CubeState state = apply_moves(SOLVED_CUBE, moves);

    if (state.is_solved()) return;  // trivial

    auto result = g_solver->solve(state, expected_max_len + 2);

    if (!result.found) {
        throw std::runtime_error("Solver failed to find solution for: " + scramble_str);
    }

    if ((int)result.moves.size() > expected_max_len) {
        std::ostringstream ss;
        ss << "Solution too long: " << result.moves.size()
           << " > " << expected_max_len << " for " << scramble_str;
        throw std::runtime_error(ss.str());
    }

    // Verify the solution actually solves the cube
    CubeState check = apply_moves(state, result.moves);
    if (!check.is_solved()) {
        throw std::runtime_error("Solution does not solve the cube: " + scramble_str);
    }
}

TEST(solve_already_solved) {
    auto result = g_solver->solve(SOLVED_CUBE);
    ASSERT(result.found);
    ASSERT(result.moves.empty());
}

TEST(solve_one_move) {
    // One move should be solved in 1 move
    for (int m = 0; m < NUM_MOVES; m++) {
        CubeState s = apply_move(SOLVED_CUBE, m);
        auto result = g_solver->solve(s, 1);
        ASSERT(result.found);
        ASSERT_EQ((int)result.moves.size(), 1);

        CubeState check = apply_moves(s, result.moves);
        ASSERT(check.is_solved());
    }
}

TEST(solve_two_moves) {
    // Two non-inverse moves, ensure solution is ≤2
    std::vector<std::string> seqs = {"R U", "F L", "U2 F", "B R'"};
    for (const auto& s : seqs) {
        solve_and_verify(s, 2);
    }
}

TEST(solve_sexy_move) {
    // R U R' U' has a 4-move inverse: U R U' R'
    solve_and_verify("R U R' U'", 4);
}

TEST(solve_5_moves) {
    solve_and_verify("R U F D B", 5);
    solve_and_verify("R2 U F' D2 B", 5);
}

TEST(solve_8_moves) {
    // T-perm: R U R' U' R' F R2 U' R' U' R U R' F' is 14 moves but known optimal
    // Use simpler 8-move sequence
    solve_and_verify("R U2 R' F2 D' L B'", 8);
}

TEST(solution_is_optimal) {
    // For 1-move scrambles, solution must be exactly 1 move
    for (int m = 0; m < NUM_MOVES; m++) {
        CubeState s = apply_move(SOLVED_CUBE, m);
        auto result = g_solver->solve(s, 20);
        ASSERT(result.found);
        ASSERT_EQ((int)result.moves.size(), 1);
    }
}

TEST(validate_cube_valid) {
    std::string err;
    ASSERT(validate_cube(SOLVED_CUBE, err));
    ASSERT(err.empty());

    auto state = apply_moves(SOLVED_CUBE, parse_move_sequence("R U R' U'"));
    ASSERT(validate_cube(state, err));
}

TEST(validate_cube_invalid_perm) {
    CubeState bad = SOLVED_CUBE;
    // Swap two corners (creates invalid perm)
    std::swap(bad.cp[0], bad.cp[1]);

    std::string err;
    bool valid = validate_cube(bad, err);
    // Should fail parity check
    ASSERT(!valid);
    ASSERT(!err.empty());
}

int main() {
    init_moves();

    // Use simple heuristic for tests (pattern DB may not be built)
    IDASolver solver(heuristic_misplaced);
    g_solver = &solver;

    std::cout << "=== Solver Tests ===\n";
    RUN_TEST(solve_already_solved);
    RUN_TEST(solve_one_move);
    RUN_TEST(solve_two_moves);
    RUN_TEST(solve_sexy_move);
    RUN_TEST(solve_5_moves);
    RUN_TEST(solve_8_moves);
    RUN_TEST(solution_is_optimal);
    RUN_TEST(validate_cube_valid);
    RUN_TEST(validate_cube_invalid_perm);

    std::cout << "\n" << g_tests_run << " tests: "
              << g_tests_passed << " passed, "
              << g_tests_failed << " failed\n";

    return g_tests_failed > 0 ? 1 : 0;
}
