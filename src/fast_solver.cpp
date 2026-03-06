#include "fast_solver.h"
#include <chrono>
#include <cstring>
#include <algorithm>
#include <future>

CpMoveTable CP_MOVE_TABLE;
CoMoveTable CO_MOVE_TABLE;
EoMoveTable EO_MOVE_TABLE;
InvEpTable  INV_EP_TABLE;

// Corner permutation move table (right multiplication).
// new_cp[i] = cp[mv.cp[i]]
// CP_MOVE_TABLE[m][cp_idx] = encode_corner_perm(new_cp)
void build_cp_move_table() {
    for (int m = 0; m < NUM_MOVES; m++) {
        const CubeState& mv = MOVE_TABLE[m];
        for (uint32_t idx = 0; idx < 40320; idx++) {
            auto cp = decode_corner_perm(idx);
            std::array<uint8_t, NUM_CORNERS> ncp;
            for (int i = 0; i < NUM_CORNERS; i++)
                ncp[i] = cp[mv.cp[i]];
            CP_MOVE_TABLE[m][idx] = encode_corner_perm(ncp);
        }
    }
}

// Corner orientation move table (right multiplication).
// new_co[i] = (co[mv.cp[i]] + mv.co[i]) % 3
// Independent of the current corner permutation, so a standalone table is valid.
// CO_MOVE_TABLE[m][co_idx] = encode_corner_orient(new_co)
void build_co_move_table() {
    for (int m = 0; m < NUM_MOVES; m++) {
        const CubeState& mv = MOVE_TABLE[m];
        for (uint32_t idx = 0; idx < 2187; idx++) {
            auto co = decode_corner_orient(idx);
            std::array<uint8_t, NUM_CORNERS> nco;
            for (int i = 0; i < NUM_CORNERS; i++)
                nco[i] = (co[mv.cp[i]] + mv.co[i]) % 3;
            CO_MOVE_TABLE[m][idx] = (uint16_t)encode_corner_orient(nco);
        }
    }
}

// Edge orientation move table (right multiplication).
// new_eo[i] = (eo[mv.ep[i]] + mv.eo[i]) % 2
// Independent of the current edge permutation, so a standalone table is valid.
// EO_MOVE_TABLE[m][eo_idx] = encode_edge_orient(new_eo)
void build_eo_move_table() {
    for (int m = 0; m < NUM_MOVES; m++) {
        const CubeState& mv = MOVE_TABLE[m];
        for (uint32_t idx = 0; idx < 2048; idx++) {
            auto eo = decode_edge_orient(idx);
            std::array<uint8_t, NUM_EDGES> neo;
            for (int i = 0; i < NUM_EDGES; i++)
                neo[i] = (eo[mv.ep[i]] + mv.eo[i]) % 2;
            EO_MOVE_TABLE[m][idx] = (uint16_t)encode_edge_orient(neo);
        }
    }
}

void build_inv_ep_table() {
    for (int m = 0; m < NUM_MOVES; m++)
        for (int j = 0; j < 12; j++)
            INV_EP_TABLE[m][MOVE_TABLE[m].ep[j]] = (uint8_t)j;
}

// Partial Lehmer rank factors: P(11,5), P(10,4), P(9,3), P(8,2), P(7,1), P(6,0)
static const uint32_t EPF[6] = {55440u, 5040u, 504u, 56u, 7u, 1u};

FastIDASolver::FastIDASolver(const CornerPatternDB& corner_db,
                             const EdgePatternDB&   edge_db1,
                             const EdgePatternDB&   edge_db2,
                             const EdgePatternDB&   edge_db3)
    : corner_db_(corner_db), edge_db1_(edge_db1), edge_db2_(edge_db2), edge_db3_(edge_db3),
      nodes_explored_(0), cp_idx_(0), co_idx_(0), eo_idx_(0),
      ep1_idx_(0), ep2_idx_(0), ep3_idx_(0)
{
    memset(cp_, 0, sizeof cp_);
    memset(co_, 0, sizeof co_);
    memset(ep_, 0, sizeof ep_);
    memset(eo_, 0, sizeof eo_);
    memset(ep1_pos_, 0, sizeof ep1_pos_);
    memset(ep1_ori_, 0, sizeof ep1_ori_);
    memset(ep2_pos_, 0, sizeof ep2_pos_);
    memset(ep2_ori_, 0, sizeof ep2_ori_);
    memset(ep3_pos_, 0, sizeof ep3_pos_);
    memset(ep3_ori_, 0, sizeof ep3_ori_);
}

uint32_t FastIDASolver::encode_ep(const uint8_t pos[6], const uint8_t ori[6]) const {
    uint32_t perm_rank = 0;
    bool used[12] = {};
    for (int k = 0; k < 6; k++) {
        int count = 0;
        for (int j = 0; j < pos[k]; j++) if (!used[j]) count++;
        perm_rank += (uint32_t)count * EPF[k];
        used[pos[k]] = true;
    }
    uint32_t orient = 0;
    for (int k = 0; k < 6; k++) orient |= ((uint32_t)ori[k] << k);
    return perm_rank * 64u + orient;
}

// Labels for group 2: edges 0,2,4,6,8,10 (UR,UL,DR,DL,FR,BL alternating)
static const uint8_t G2_LABELS[6] = {0,2,4,6,8,10};

void FastIDASolver::init_state(const CubeState& start) {
    for (int i = 0; i < 8;  i++) { cp_[i] = start.cp[i]; co_[i] = start.co[i]; }
    for (int i = 0; i < 12; i++) { ep_[i] = start.ep[i]; eo_[i] = start.eo[i]; }
    cp_idx_ = encode_corner_perm(start.cp);
    co_idx_ = encode_corner_orient(start.co);
    eo_idx_ = encode_edge_orient(start.eo);
    for (int k = 0; k < 6; k++) {
        for (int j = 0; j < 12; j++) {
            if (start.ep[j] == k)            { ep1_pos_[k] = j; ep1_ori_[k] = start.eo[j]; }
            if (start.ep[j] == k+6)          { ep2_pos_[k] = j; ep2_ori_[k] = start.eo[j]; }
            if (start.ep[j] == G2_LABELS[k]) { ep3_pos_[k] = j; ep3_ori_[k] = start.eo[j]; }
        }
    }
    ep1_idx_ = encode_ep(ep1_pos_, ep1_ori_);
    ep2_idx_ = encode_ep(ep2_pos_, ep2_ori_);
    ep3_idx_ = encode_ep(ep3_pos_, ep3_ori_);
}

int FastIDASolver::compute_heuristic(const CubeState& start) {
    init_state(start);
    return heuristic();
}

int FastIDASolver::search_threshold_single(const CubeState& start, int root_move,
                                            int threshold, const std::atomic<bool>& found) {
    nodes_explored_ = 0;
    path_.clear();
    abort_ = &found;
    init_state(start);
    apply(root_move);
    path_.push_back(root_move);
    int result = search(1, threshold, root_move);
    abort_ = nullptr;
    return result;
}

SolveResult FastIDASolver::solve(const CubeState& start, int max_depth) {
    auto t0 = std::chrono::high_resolution_clock::now();
    nodes_explored_ = 0;
    path_.clear();

    // Load initial state; encode indices once, reuse across threshold iterations
    init_state(start);
    const uint32_t start_cp_idx  = cp_idx_,  start_co_idx  = co_idx_,  start_eo_idx  = eo_idx_;
    const uint32_t start_ep1_idx = ep1_idx_, start_ep2_idx = ep2_idx_, start_ep3_idx = ep3_idx_;
    uint8_t s_ep1_pos[6], s_ep1_ori[6], s_ep2_pos[6], s_ep2_ori[6];
    uint8_t s_ep3_pos[6], s_ep3_ori[6];
    memcpy(s_ep1_pos, ep1_pos_, 6); memcpy(s_ep1_ori, ep1_ori_, 6);
    memcpy(s_ep2_pos, ep2_pos_, 6); memcpy(s_ep2_ori, ep2_ori_, 6);
    memcpy(s_ep3_pos, ep3_pos_, 6); memcpy(s_ep3_ori, ep3_ori_, 6);

    if (is_solved()) {
        auto t1 = std::chrono::high_resolution_clock::now();
        return {true, {}, 0, std::chrono::duration<double>(t1 - t0).count()};
    }

    int threshold = heuristic();

    while (threshold <= max_depth) {
        for (int i = 0; i < 8;  i++) { cp_[i] = start.cp[i]; co_[i] = start.co[i]; }
        for (int i = 0; i < 12; i++) { ep_[i] = start.ep[i]; eo_[i] = start.eo[i]; }
        cp_idx_ = start_cp_idx; co_idx_ = start_co_idx; eo_idx_ = start_eo_idx;
        ep1_idx_ = start_ep1_idx; ep2_idx_ = start_ep2_idx; ep3_idx_ = start_ep3_idx;
        memcpy(ep1_pos_, s_ep1_pos, 6); memcpy(ep1_ori_, s_ep1_ori, 6);
        memcpy(ep2_pos_, s_ep2_pos, 6); memcpy(ep2_ori_, s_ep2_ori, 6);
        memcpy(ep3_pos_, s_ep3_pos, 6); memcpy(ep3_ori_, s_ep3_ori, 6);
        path_.clear();

        int result = search(0, threshold, -1);

        if (result == FOUND) {
            // Right-mult search produces path [m_1, ..., m_k] with
            // initial ∘ m_1 ∘ ... ∘ m_k = identity, i.e. m_1...m_k = initial^{-1}.
            // Physical solution (left-mult convention): [m_k, ..., m_1].
            std::reverse(path_.begin(), path_.end());
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

// Four table lookups, no encoding.
int FastIDASolver::heuristic() const {
    int h = (int)corner_db_.lookup_idx(cp_idx_ * 2187u + co_idx_);
    h = std::max(h, (int)edge_db1_.lookup_idx(ep1_idx_));
    h = std::max(h, (int)edge_db2_.lookup_idx(ep2_idx_));
    h = std::max(h, (int)edge_db3_.lookup_idx(ep3_idx_));
    return h;
}

bool FastIDASolver::is_solved() const {
    if (cp_idx_ != 0) return false;
    for (int i = 0; i < 8;  i++) if (co_[i] != 0) return false;
    for (int i = 0; i < 12; i++) if (ep_[i] != (uint8_t)i || eo_[i] != 0) return false;
    return true;
}

// Apply move m in-place.  Right multiplication: compose(state, mv).
//   ncp[i] = cp[mv.cp[i]],   nco[i] = (co[mv.cp[i]] + mv.co[i]) % 3
//   nep[i] = ep[mv.ep[i]],   neo[i] = (eo[mv.ep[i]] + mv.eo[i]) % 2
// Corner/EO indices updated via move tables. Partial edge state updated via INV_EP_TABLE.
void FastIDASolver::apply(int m) {
    const CubeState& mv = MOVE_TABLE[m];
    uint8_t ncp[8], nco[8], nep[12], neo[12];

    for (int i = 0; i < 8; i++) {
        ncp[i] = cp_[mv.cp[i]];
        nco[i] = (co_[mv.cp[i]] + mv.co[i]) % 3;
    }
    for (int i = 0; i < 12; i++) {
        nep[i] = ep_[mv.ep[i]];
        neo[i] = (eo_[mv.ep[i]] + mv.eo[i]) % 2;
    }

    cp_idx_ = CP_MOVE_TABLE[m][cp_idx_];
    co_idx_ = CO_MOVE_TABLE[m][co_idx_];
    eo_idx_ = EO_MOVE_TABLE[m][eo_idx_];
    memcpy(cp_, ncp, 8);  memcpy(co_, nco, 8);
    memcpy(ep_, nep, 12); memcpy(eo_, neo, 12);

    // Update partial edge state for groups 0, 1, and 2
    for (int k = 0; k < 6; k++) {
        uint8_t np = INV_EP_TABLE[m][ep1_pos_[k]];
        ep1_ori_[k] = (ep1_ori_[k] + mv.eo[np]) % 2;
        ep1_pos_[k] = np;
    }
    for (int k = 0; k < 6; k++) {
        uint8_t np = INV_EP_TABLE[m][ep2_pos_[k]];
        ep2_ori_[k] = (ep2_ori_[k] + mv.eo[np]) % 2;
        ep2_pos_[k] = np;
    }
    for (int k = 0; k < 6; k++) {
        uint8_t np = INV_EP_TABLE[m][ep3_pos_[k]];
        ep3_ori_[k] = (ep3_ori_[k] + mv.eo[np]) % 2;
        ep3_pos_[k] = np;
    }
    ep1_idx_ = encode_ep(ep1_pos_, ep1_ori_);
    ep2_idx_ = encode_ep(ep2_pos_, ep2_ori_);
    ep3_idx_ = encode_ep(ep3_pos_, ep3_ori_);
}

int FastIDASolver::search(int g, int threshold, int prev_move) {
    if (abort_ && abort_->load(std::memory_order_relaxed)) return INF;
    int h = heuristic();
    int f = g + h;
    if (f > threshold) return f;
    if (is_solved()) return FOUND;

    nodes_explored_++;

    // Save state (raw arrays + cached indices + partial edge tracking for 3 groups)
    uint8_t  scp[8], sco[8], sep[12], seo[12];
    uint8_t  sep1_pos[6], sep1_ori[6], sep2_pos[6], sep2_ori[6];
    uint8_t  sep3_pos[6], sep3_ori[6];
    uint32_t scp_idx  = cp_idx_,  sco_idx  = co_idx_,  seo_idx  = eo_idx_;
    uint32_t sep1_idx = ep1_idx_, sep2_idx = ep2_idx_,  sep3_idx = ep3_idx_;
    memcpy(scp, cp_, 8);  memcpy(sco, co_, 8);
    memcpy(sep, ep_, 12); memcpy(seo, eo_, 12);
    memcpy(sep1_pos, ep1_pos_, 6); memcpy(sep1_ori, ep1_ori_, 6);
    memcpy(sep2_pos, ep2_pos_, 6); memcpy(sep2_ori, ep2_ori_, 6);
    memcpy(sep3_pos, ep3_pos_, 6); memcpy(sep3_ori, ep3_ori_, 6);

    int min_t = INF;

    for (int m = 0; m < NUM_MOVES; m++) {
        if (is_redundant_sequence(prev_move, m)) continue;

        apply(m);
        path_.push_back(m);

        int result = search(g + 1, threshold, m);
        if (result == FOUND) return FOUND;
        if (result < min_t) min_t = result;

        path_.pop_back();

        cp_idx_  = scp_idx;  co_idx_  = sco_idx;  eo_idx_  = seo_idx;
        ep1_idx_ = sep1_idx; ep2_idx_ = sep2_idx; ep3_idx_ = sep3_idx;
        memcpy(cp_, scp, 8);  memcpy(co_, sco, 8);
        memcpy(ep_, sep, 12); memcpy(eo_, seo, 12);
        memcpy(ep1_pos_, sep1_pos, 6); memcpy(ep1_ori_, sep1_ori, 6);
        memcpy(ep2_pos_, sep2_pos, 6); memcpy(ep2_ori_, sep2_ori, 6);
        memcpy(ep3_pos_, sep3_pos, 6); memcpy(ep3_ori_, sep3_ori, 6);
    }

    return min_t;
}

// -- ParallelFastIDASolver ---------------------------------------------------

ParallelFastIDASolver::ParallelFastIDASolver(const CornerPatternDB& corner_db,
                                             const EdgePatternDB&   edge_db1,
                                             const EdgePatternDB&   edge_db2,
                                             const EdgePatternDB&   edge_db3)
    : corner_db_(corner_db), edge_db1_(edge_db1), edge_db2_(edge_db2), edge_db3_(edge_db3),
      nodes_explored_(0)
{}

SolveResult ParallelFastIDASolver::solve(const CubeState& start, int max_depth) {
    auto t0 = std::chrono::high_resolution_clock::now();
    nodes_explored_ = 0;

    if (start.is_solved()) {
        auto t1 = std::chrono::high_resolution_clock::now();
        return {true, {}, 0, std::chrono::duration<double>(t1 - t0).count()};
    }

    // Compute initial threshold with a temporary probe solver.
    int threshold;
    {
        FastIDASolver probe(corner_db_, edge_db1_, edge_db2_, edge_db3_);
        threshold = probe.compute_heuristic(start);
    }

    using TaskResult = std::tuple<int, std::vector<int>, uint64_t>;

    while (threshold <= max_depth) {
        std::atomic<bool> found{false};

        std::vector<std::future<TaskResult>> futures;
        futures.reserve(NUM_MOVES);

        for (int m = 0; m < NUM_MOVES; m++) {
            futures.push_back(std::async(std::launch::async,
                [&, m]() -> TaskResult {
                    FastIDASolver solver(corner_db_, edge_db1_, edge_db2_, edge_db3_);
                    int result = solver.search_threshold_single(start, m, threshold, found);
                    if (result == FastIDASolver::FOUND)
                        found.store(true, std::memory_order_relaxed);
                    return {result,
                            result == FastIDASolver::FOUND ? solver.path() : std::vector<int>{},
                            solver.get_nodes_explored()};
                }));
        }

        int min_t = FastIDASolver::INF;
        std::vector<int> solution;

        for (auto& f : futures) {
            auto [result, path, nodes] = f.get();
            nodes_explored_ += nodes;
            if (result == FastIDASolver::FOUND && solution.empty()) {
                solution = std::move(path);
                std::reverse(solution.begin(), solution.end());
            } else if (result != FastIDASolver::FOUND && result < min_t) {
                min_t = result;
            }
        }

        if (!solution.empty()) {
            auto t1 = std::chrono::high_resolution_clock::now();
            return {true, solution, nodes_explored_,
                    std::chrono::duration<double>(t1 - t0).count()};
        }
        if (min_t == FastIDASolver::INF) break;
        threshold = min_t;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    return {false, {}, nodes_explored_,
            std::chrono::duration<double>(t1 - t0).count()};
}
