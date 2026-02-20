#include "fast_solver.h"
#include <chrono>
#include <cstring>
#include <algorithm>

CpMoveTable CP_MOVE_TABLE;

// Build the corner permutation move table for left multiplication.
// For each move m and each permutation index cp_idx:
//   new_cp[i] = mv.cp[cp[i]]   (compose(mv, state).cp[i])
//   CP_MOVE_TABLE[m][cp_idx] = encode_corner_perm(new_cp)
//
// Lets us update cp_idx in O(1) rather than re-running Lehmer encoding.
void build_cp_move_table() {
    for (int m = 0; m < NUM_MOVES; m++) {
        const CubeState& mv = MOVE_TABLE[m];
        for (uint32_t idx = 0; idx < 40320; idx++) {
            auto cp = decode_corner_perm(idx);
            std::array<uint8_t, NUM_CORNERS> new_cp;
            for (int i = 0; i < NUM_CORNERS; i++)
                new_cp[i] = mv.cp[cp[i]];
            CP_MOVE_TABLE[m][idx] = encode_corner_perm(new_cp);
        }
    }
}

FastIDASolver::FastIDASolver(const CornerPatternDB& corner_db,
                             const EdgeOrientDB&    edge_orient_db)
    : corner_db_(corner_db), edge_orient_db_(edge_orient_db),
      nodes_explored_(0), cp_idx_(0)
{
    memset(cp_, 0, sizeof cp_);
    memset(co_, 0, sizeof co_);
    memset(ep_, 0, sizeof ep_);
    memset(eo_, 0, sizeof eo_);
}

SolveResult FastIDASolver::solve(const CubeState& start, int max_depth) {
    auto t0 = std::chrono::high_resolution_clock::now();
    nodes_explored_ = 0;
    path_.clear();

    // Load initial state
    for (int i = 0; i < 8;  i++) { cp_[i] = start.cp[i]; co_[i] = start.co[i]; }
    for (int i = 0; i < 12; i++) { ep_[i] = start.ep[i]; eo_[i] = start.eo[i]; }
    cp_idx_ = encode_corner_perm(start.cp);

    if (is_solved()) {
        auto t1 = std::chrono::high_resolution_clock::now();
        return {true, {}, 0, std::chrono::duration<double>(t1 - t0).count()};
    }

    int threshold = heuristic();

    while (threshold <= max_depth) {
        // Restore start state at each IDA* iteration
        for (int i = 0; i < 8;  i++) { cp_[i] = start.cp[i]; co_[i] = start.co[i]; }
        for (int i = 0; i < 12; i++) { ep_[i] = start.ep[i]; eo_[i] = start.eo[i]; }
        cp_idx_ = encode_corner_perm(start.cp);
        path_.clear();

        int result = search(0, threshold, -1);

        if (result == FOUND) {
            auto t1 = std::chrono::high_resolution_clock::now();
            return {true, path_, nodes_explored_,
                    std::chrono::duration<double>(t1 - t0).count()};
        }
        if (result == INF) break;
        threshold = result;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    return {false, {}, nodes_explored_,
            std::chrono::duration<double>(t1 - t0).count()};
}

int FastIDASolver::heuristic() const {
    // Corner orientation: O(7) encode — no Lehmer needed, just base-3
    std::array<uint8_t, NUM_CORNERS> co_arr;
    memcpy(co_arr.data(), co_, NUM_CORNERS);
    uint32_t co_idx = encode_corner_orient(co_arr);

    // Edge orientation: O(11) encode
    std::array<uint8_t, NUM_EDGES> eo_arr;
    memcpy(eo_arr.data(), eo_, NUM_EDGES);
    uint32_t eo_idx = encode_edge_orient(eo_arr);

    int h = (int)corner_db_.lookup_idx(cp_idx_ * 2187u + co_idx);
    return std::max(h, (int)edge_orient_db_.lookup_idx(eo_idx));
}

bool FastIDASolver::is_solved() const {
    if (cp_idx_ != 0) return false;
    for (int i = 0; i < 8;  i++) if (co_[i] != 0) return false;
    for (int i = 0; i < 12; i++) if (ep_[i] != (uint8_t)i || eo_[i] != 0) return false;
    return true;
}

// Apply move m in-place.  Left multiplication: compose(mv, state).
//   new_cp[i] = mv.cp[cp[i]],   new_co[i] = (mv.co[cp[i]] + co[i]) % 3
//   new_ep[i] = mv.ep[ep[i]],   new_eo[i] = (mv.eo[ep[i]] + eo[i]) % 2
// cp_idx_ is updated via CP_MOVE_TABLE in O(1).
void FastIDASolver::apply(int m) {
    const CubeState& mv = MOVE_TABLE[m];
    uint8_t ncp[8], nco[8], nep[12], neo[12];

    for (int i = 0; i < 8; i++) {
        ncp[i] = mv.cp[cp_[i]];
        nco[i] = (mv.co[cp_[i]] + co_[i]) % 3;
    }
    for (int i = 0; i < 12; i++) {
        nep[i] = mv.ep[ep_[i]];
        neo[i] = (mv.eo[ep_[i]] + eo_[i]) % 2;
    }

    cp_idx_ = CP_MOVE_TABLE[m][cp_idx_];  // O(1) — must use old cp_idx_
    memcpy(cp_, ncp, 8);  memcpy(co_, nco, 8);
    memcpy(ep_, nep, 12); memcpy(eo_, neo, 12);
}

int FastIDASolver::search(int g, int threshold, int prev_move) {
    int h = heuristic();
    int f = g + h;
    if (f > threshold) return f;
    if (is_solved()) return FOUND;

    nodes_explored_++;

    // Save state before branching — 44 bytes, very cache-friendly at depth ≤ 20
    uint8_t  scp[8], sco[8], sep[12], seo[12];
    uint32_t scp_idx = cp_idx_;
    memcpy(scp, cp_, 8);  memcpy(sco, co_, 8);
    memcpy(sep, ep_, 12); memcpy(seo, eo_, 12);

    int min_t = INF;

    for (int m = 0; m < NUM_MOVES; m++) {
        if (is_redundant_sequence(prev_move, m)) continue;

        apply(m);
        path_.push_back(m);

        int result = search(g + 1, threshold, m);
        if (result == FOUND) return FOUND;
        if (result < min_t) min_t = result;

        path_.pop_back();

        // Restore state
        cp_idx_ = scp_idx;
        memcpy(cp_, scp, 8);  memcpy(co_, sco, 8);
        memcpy(ep_, sep, 12); memcpy(eo_, seo, 12);
    }

    return min_t;
}
