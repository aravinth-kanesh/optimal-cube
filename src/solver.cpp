#include "solver.h"
#include <chrono>
#include <algorithm>
#include <cassert>
#include <iostream>

// ============================================================
// IDASolver implementation
// ============================================================

IDASolver::IDASolver(HeuristicFn heuristic)
    : heuristic_(std::move(heuristic)), nodes_explored_(0) {}

SolveResult IDASolver::solve(const CubeState& start, int max_depth) {
    auto t_start = std::chrono::high_resolution_clock::now();
    nodes_explored_ = 0;
    path_.clear();

    if (start.is_solved()) {
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(t_end - t_start).count();
        return SolveResult{true, {}, 0, elapsed};
    }

    int threshold = heuristic_(start);

    while (threshold <= max_depth) {
        path_.clear();
        int result = search(start, 0, threshold);

        if (result == FOUND) {
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(t_end - t_start).count();
            return SolveResult{true, path_, nodes_explored_, elapsed};
        }

        if (result == INF) {
            // No solution exists (shouldn't happen for valid cube)
            break;
        }

        threshold = result;
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    return SolveResult{false, {}, nodes_explored_, elapsed};
}

int IDASolver::search(const CubeState& state, int g, int threshold) {
    int h = heuristic_(state);
    int f = g + h;

    if (f > threshold) return f;

    nodes_explored_++;

    if (state.is_solved()) return FOUND;

    int min_threshold = INF;
    int prev_move = path_.empty() ? -1 : path_.back();

    for (int m = 0; m < NUM_MOVES; m++) {
        // Pruning: skip redundant move sequences
        if (is_redundant_sequence(prev_move, m)) continue;

        CubeState next = apply_move(state, m);
        path_.push_back(m);

        int result = search(next, g + 1, threshold);

        if (result == FOUND) return FOUND;

        if (result < min_threshold) min_threshold = result;

        path_.pop_back();
    }

    return min_threshold;
}

// ============================================================
// Simple heuristic: misplaced cubies (admissible)
// Each move cycles exactly 4 corners and 4 edges.
// Therefore: at most 4 corners can be placed correctly per move,
// and at most 4 edges can be placed correctly per move.
// Taking max of the two separate bounds gives an admissible heuristic.
// ============================================================
int heuristic_misplaced(const CubeState& state) {
    int misplaced_corners = 0;
    for (int i = 0; i < NUM_CORNERS; i++) {
        if (state.cp[i] != i || state.co[i] != 0) misplaced_corners++;
    }
    int misplaced_edges = 0;
    for (int i = 0; i < NUM_EDGES; i++) {
        if (state.ep[i] != i || state.eo[i] != 0) misplaced_edges++;
    }
    // Each move fixes at most 4 corners and at most 4 edges
    int h_corners = (misplaced_corners + 3) / 4;
    int h_edges   = (misplaced_edges   + 3) / 4;
    return std::max(h_corners, h_edges);
}

// ============================================================
// Build pre-computed coordinate move tables for fast IDA*
//
// Instead of computing full CubeState compositions during search,
// we pre-compute how each move transforms each coordinate index.
// This converts the expensive compose() into a simple table lookup.
// ============================================================
void build_move_tables(
    std::array<std::array<uint32_t, 40320>, NUM_MOVES>& corner_perm_table,
    std::array<std::array<uint32_t, 2187>, NUM_MOVES>&  corner_orient_table,
    std::array<std::array<uint32_t, 2048>, NUM_MOVES>&  edge_orient_table)
{
    // Corner permutation table: 8! = 40320 states
    // For each (move, perm_idx) -> new perm_idx
    for (int m = 0; m < NUM_MOVES; m++) {
        const CubeState& mv = MOVE_TABLE[m];
        for (uint32_t cp_idx = 0; cp_idx < 40320; cp_idx++) {
            auto cp = decode_corner_perm(cp_idx);
            // Apply move permutation to cp
            std::array<uint8_t, NUM_CORNERS> new_cp;
            for (int i = 0; i < NUM_CORNERS; i++) {
                new_cp[i] = cp[mv.cp[i]];
            }
            corner_perm_table[m][cp_idx] = encode_corner_perm(new_cp);
        }
    }

    // Corner orientation table: 3^7 = 2187 states
    for (int m = 0; m < NUM_MOVES; m++) {
        const CubeState& mv = MOVE_TABLE[m];
        for (uint32_t co_idx = 0; co_idx < 2187; co_idx++) {
            auto co = decode_corner_orient(co_idx);
            std::array<uint8_t, NUM_CORNERS> new_co;
            for (int i = 0; i < NUM_CORNERS; i++) {
                // In compose(mv, state): new_co[i] = (mv.co[mv.cp[i]] + ... hmm
                // We need to track how the orientation of the cubie landing at position i
                // changes. Using the compose formula:
                // result.co[i] = (a.co[b.cp[i]] + b.co[i]) % 3
                // Here a = mv, b.cp[i] is the identity (we're working on orientation only
                // with the current permutation already absorbed into co). Actually we need
                // to work with the actual perm.
                // For orientation table: assume we apply the move to a state where
                // corners are in solved permutation but with given orientations.
                // Since compose(mv, state):
                //   result.cp[i] = mv.cp[state.cp[i]] = mv.cp[i]  (state.cp = identity)
                //   result.co[i] = (mv.co[state.cp[i]] + state.co[i]) % 3
                //                = (mv.co[i] + co[i]) % 3
                new_co[i] = (mv.co[i] + co[i]) % 3;
            }
            corner_orient_table[m][co_idx] = encode_corner_orient(new_co);
        }
    }

    // Edge orientation table: 2^11 = 2048 states
    // Uses compose(state, mv): new_eo[i] = (eo[mv.ep[i]] + mv.eo[i]) % 2
    // (the edge that arrives at position i came from position mv.ep[i])
    for (int m = 0; m < NUM_MOVES; m++) {
        const CubeState& mv = MOVE_TABLE[m];
        for (uint32_t eo_idx = 0; eo_idx < 2048; eo_idx++) {
            auto eo = decode_edge_orient(eo_idx);
            std::array<uint8_t, NUM_EDGES> new_eo;
            for (int i = 0; i < NUM_EDGES; i++) {
                new_eo[i] = (uint8_t)((eo[mv.ep[i]] + mv.eo[i]) % 2);
            }
            edge_orient_table[m][eo_idx] = encode_edge_orient(new_eo);
        }
    }
}
