#pragma once
#include "cube.h"
#include "moves.h"
#include "pattern_db.h"
#include "solver.h"
#include <array>
#include <vector>

// Precomputed corner permutation move table (left multiplication).
// CP_MOVE_TABLE[move][cp_idx] → new cp_idx after applying move.
// 18 × 40320 × 4 bytes ≈ 2.8 MB. Built once by build_cp_move_table().
using CpMoveTable = std::array<std::array<uint32_t, 40320>, NUM_MOVES>;
extern CpMoveTable CP_MOVE_TABLE;

void build_cp_move_table();  // call once after init_moves()

// IDA* solver that avoids CubeState copies and Lehmer re-encoding on every node.
//
// The bottleneck in the naive solver is:
//   1. compose() — allocates and fills a new CubeState (~80 ops)
//   2. corner_db_index() — Lehmer encoding for cp is O(n²) (~100 ops)
//
// This solver instead:
//   - Keeps the search state as raw arrays, modified in-place with save/restore
//   - Updates cp_idx in O(1) via CP_MOVE_TABLE
//   - Encodes co and eo on-the-fly (O(7) and O(11) — fast)
//
// Requires CornerPatternDB and EdgeOrientDB to be loaded.
class FastIDASolver {
public:
    FastIDASolver(const CornerPatternDB& corner_db,
                  const EdgeOrientDB&    edge_orient_db);

    SolveResult solve(const CubeState& start, int max_depth = 20);

    uint64_t get_nodes_explored() const { return nodes_explored_; }

private:
    const CornerPatternDB& corner_db_;
    const EdgeOrientDB&    edge_orient_db_;

    uint64_t         nodes_explored_;
    std::vector<int> path_;

    // Search state — modified in-place, saved/restored across recursive calls
    uint8_t  cp_[8], co_[8], ep_[12], eo_[12];
    uint32_t cp_idx_;  // Lehmer index of cp_, kept in sync by apply()

    static constexpr int FOUND = -1;
    static constexpr int INF   = 1000;

    int  heuristic() const;
    bool is_solved()  const;
    void apply(int m);

    int search(int g, int threshold, int prev_move);
};
