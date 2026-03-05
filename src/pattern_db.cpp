#include "pattern_db.h"
#include <queue>
#include <fstream>
#include <iostream>
#include <cassert>
#include <cstring>
#include <chrono>

CornerPatternDB::CornerPatternDB() {
    // Packed nibbles: CORNER_DB_SIZE entries, 4 bits each
    // Need ceil(CORNER_DB_SIZE / 2) bytes
    data_.assign((CORNER_DB_SIZE + 1) / 2, 0xFF);  // 0xFF = both nibbles unvisited (15)
}

uint8_t CornerPatternDB::get(uint32_t idx) const {
    uint8_t byte = data_[idx / 2];
    return (idx % 2 == 0) ? (byte & 0x0F) : (byte >> 4);
}

void CornerPatternDB::set(uint32_t idx, uint8_t val) {
    uint8_t& byte = data_[idx / 2];
    if (idx % 2 == 0) {
        byte = (byte & 0xF0) | (val & 0x0F);
    } else {
        byte = (byte & 0x0F) | ((val & 0x0F) << 4);
    }
}

uint8_t CornerPatternDB::lookup(const CubeState& state) const {
    return get(corner_db_index(state));
}

uint32_t CornerPatternDB::populated_count() const {
    uint32_t count = 0;
    for (uint32_t i = 0; i < CORNER_DB_SIZE; i++) {
        if (get(i) != 15) count++;
    }
    return count;
}

// BFS from solved state: 8! * 3^7 = 88,179,840 states,
// indexed by (corner_perm_idx * 2187 + corner_orient_idx).
void CornerPatternDB::build() {
    auto t_start = std::chrono::high_resolution_clock::now();

    // Reset: all entries unvisited (stored as 15 in nibbles via 0xFF bytes)
    std::fill(data_.begin(), data_.end(), 0xFF);

    // BFS queue stores corner DB indices
    std::queue<uint32_t> q;

    // Solved state
    uint32_t solved_idx = corner_db_index(SOLVED_CUBE);
    set(solved_idx, 0);
    q.push(solved_idx);

    uint64_t visited = 1;
    int current_depth = 0;

    std::cerr << "Building corner pattern database (" << CORNER_DB_SIZE << " states)...\n";

    while (!q.empty()) {
        uint32_t idx = q.front();
        q.pop();

        uint8_t dist = get(idx);
        if (dist != current_depth) {
            current_depth = dist;
            auto t_now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(t_now - t_start).count();
            std::cerr << "  Depth " << current_depth
                      << ": " << visited << " states visited"
                      << " (" << elapsed << "s)\n";
        }

        // Decode the corner state from the DB index
        uint32_t cp_idx = idx / 2187u;
        uint32_t co_idx = idx % 2187u;
        auto cp = decode_corner_perm(cp_idx);
        auto co = decode_corner_orient(co_idx);

        // Apply all 18 moves
        for (int m = 0; m < NUM_MOVES; m++) {
            const CubeState& mv = MOVE_TABLE[m];

            // Apply move to corner permutation
            std::array<uint8_t, NUM_CORNERS> new_cp;
            std::array<uint8_t, NUM_CORNERS> new_co;
            for (int i = 0; i < NUM_CORNERS; i++) {
                new_cp[i] = cp[mv.cp[i]];
                // compose(state, mv): new_co[i] = (state.co[mv.cp[i]] + mv.co[i]) % 3
                new_co[i] = (uint8_t)((co[mv.cp[i]] + mv.co[i]) % 3);
            }

            uint32_t new_cp_idx = encode_corner_perm(new_cp);
            uint32_t new_co_idx = encode_corner_orient(new_co);
            uint32_t new_idx = new_cp_idx * 2187u + new_co_idx;

            if (get(new_idx) == 15) {  // unvisited
                set(new_idx, dist + 1);
                q.push(new_idx);
                visited++;
            }
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    std::cerr << "Corner DB complete: " << visited << " states in "
              << elapsed << "s\n";

    ready_ = true;
}

bool CornerPatternDB::save(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "Failed to open " << path << " for writing\n";
        return false;
    }
    f.write(reinterpret_cast<const char*>(data_.data()), (long)data_.size());
    return f.good();
}

bool CornerPatternDB::load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    // Check file size
    f.seekg(0, std::ios::end);
    size_t file_size = f.tellg();
    f.seekg(0, std::ios::beg);

    size_t expected = (CORNER_DB_SIZE + 1) / 2;
    if (file_size != expected) {
        std::cerr << "Corner DB file size mismatch: expected " << expected
                  << " bytes, got " << file_size << "\n";
        return false;
    }

    f.read(reinterpret_cast<char*>(data_.data()), (long)data_.size());
    if (!f) return false;

    ready_ = true;
    return true;
}

// EdgeOrientDB

EdgeOrientDB::EdgeOrientDB() {
    data_.fill(255);  // 255 = unvisited
}

uint8_t EdgeOrientDB::lookup(const CubeState& state) const {
    return data_[edge_orient_index(state)];
}

void EdgeOrientDB::build() {
    data_.fill(255);

    std::queue<uint32_t> q;
    uint32_t solved_idx = edge_orient_index(SOLVED_CUBE);
    data_[solved_idx] = 0;
    q.push(solved_idx);

    uint64_t visited = 1;
    std::cerr << "Building edge orientation database (" << EDGE_ORIENT_DB_SIZE << " states)...\n";

    while (!q.empty()) {
        uint32_t idx = q.front();
        q.pop();

        uint8_t dist = data_[idx];
        auto eo = decode_edge_orient(idx);

        for (int m = 0; m < NUM_MOVES; m++) {
            const CubeState& mv = MOVE_TABLE[m];

            std::array<uint8_t, NUM_EDGES> new_eo;
            for (int i = 0; i < NUM_EDGES; i++) {
                // compose(state, mv): new_eo[i] = (eo[mv.ep[i]] + mv.eo[i]) % 2
                // mv.ep[i] is the source position of position i after the move
                new_eo[i] = (uint8_t)((eo[mv.ep[i]] + mv.eo[i]) % 2);
            }

            uint32_t new_idx = encode_edge_orient(new_eo);
            if (data_[new_idx] == 255) {
                data_[new_idx] = dist + 1;
                q.push(new_idx);
                visited++;
            }
        }
    }

    std::cerr << "Edge orient DB complete: " << visited << " states\n";
    ready_ = true;
}

bool EdgeOrientDB::save(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<const char*>(data_.data()), (long)data_.size());
    return f.good();
}

bool EdgeOrientDB::load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.read(reinterpret_cast<char*>(data_.data()), (long)data_.size());
    if (!f) return false;
    ready_ = true;
    return true;
}

// EdgePatternDB

EdgePatternDB::EdgePatternDB() {
    data_.assign((DB_SIZE + 1) / 2, 0xFF);
}

uint8_t EdgePatternDB::get(uint32_t idx) const {
    uint8_t byte = data_[idx / 2];
    return (idx % 2 == 0) ? (byte & 0x0F) : (byte >> 4);
}

void EdgePatternDB::set(uint32_t idx, uint8_t val) {
    uint8_t& byte = data_[idx / 2];
    if (idx % 2 == 0) byte = (byte & 0xF0) | (val & 0x0F);
    else               byte = (byte & 0x0F) | ((val & 0x0F) << 4);
}

uint8_t EdgePatternDB::lookup(const uint8_t ep[12], const uint8_t eo[12], int group) const {
    return lookup_idx(encode_edge_partial(ep, eo, group));
}

// Factors for partial Lehmer rank: P(11,5), P(10,4), ..., P(6,0)
static const uint32_t EPF[6] = {55440u, 5040u, 504u, 56u, 7u, 1u};

// Encode (pos[6], ori[6]) partial state to a DB index
static uint32_t ep_encode(const uint8_t pos[6], const uint8_t ori[6]) {
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

// Decode DB index to (pos[6], ori[6]) partial state
static void ep_decode(uint32_t idx, uint8_t pos[6], uint8_t ori[6]) {
    uint32_t orient = idx % 64;
    uint32_t perm_rank = idx / 64;
    for (int k = 0; k < 6; k++) ori[k] = (orient >> k) & 1;
    bool used[12] = {};
    for (int k = 0; k < 6; k++) {
        uint32_t count = perm_rank / EPF[k];
        perm_rank %= EPF[k];
        int j = 0, n = 0;
        for (; j < 12; j++) {
            if (!used[j]) { if ((uint32_t)n == count) break; n++; }
        }
        pos[k] = (uint8_t)j;
        used[j] = true;
    }
}

// BFS from solved state over partial edge states (pos[6], ori[6]).
// Applies moves using inverse edge permutation: if edge k is at pos[k],
// after move m it goes to inv_ep[m][pos[k]], with orientation updated by mv.eo.
void EdgePatternDB::build(int group) {
    auto t_start = std::chrono::high_resolution_clock::now();
    std::fill(data_.begin(), data_.end(), 0xFF);

    // Precompute inverse edge permutation for each move
    // inv_ep[m][p] = destination of the edge that was at position p after move m
    uint8_t inv_ep[NUM_MOVES][12];
    for (int m = 0; m < NUM_MOVES; m++)
        for (int j = 0; j < 12; j++)
            inv_ep[m][MOVE_TABLE[m].ep[j]] = (uint8_t)j;

    std::queue<uint32_t> q;
    uint32_t solved_idx = encode_edge_partial(SOLVED_CUBE.ep.data(),
                                              SOLVED_CUBE.eo.data(), group);
    set(solved_idx, 0);
    q.push(solved_idx);

    uint64_t visited = 1;
    int current_depth = 0;
    std::cerr << "Building edge pattern DB " << group
              << " (" << DB_SIZE << " states)...\n";

    while (!q.empty()) {
        uint32_t idx = q.front(); q.pop();
        uint8_t dist = get(idx);

        if (dist != current_depth) {
            current_depth = dist;
            auto t_now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(t_now - t_start).count();
            std::cerr << "  Depth " << current_depth
                      << ": " << visited << " states (" << elapsed << "s)\n";
        }

        uint8_t pos[6], ori[6];
        ep_decode(idx, pos, ori);

        for (int m = 0; m < NUM_MOVES; m++) {
            const CubeState& mv = MOVE_TABLE[m];
            uint8_t npos[6], nori[6];
            for (int k = 0; k < 6; k++) {
                npos[k] = inv_ep[m][pos[k]];
                nori[k] = (ori[k] + mv.eo[npos[k]]) % 2;
            }
            uint32_t new_idx = ep_encode(npos, nori);
            if (get(new_idx) == 15) {
                set(new_idx, dist + 1);
                q.push(new_idx);
                visited++;
            }
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    std::cerr << "Edge DB " << group << " complete: " << visited
              << " states in " << elapsed << "s\n";
    ready_ = true;
}

bool EdgePatternDB::save(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f) { std::cerr << "Failed to open " << path << " for writing\n"; return false; }
    f.write(reinterpret_cast<const char*>(data_.data()), (long)data_.size());
    return f.good();
}

bool EdgePatternDB::load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.seekg(0, std::ios::end);
    size_t file_size = f.tellg();
    f.seekg(0, std::ios::beg);
    size_t expected = (DB_SIZE + 1) / 2;
    if (file_size != expected) {
        std::cerr << "Edge DB file size mismatch: expected " << expected
                  << " bytes, got " << file_size << "\n";
        return false;
    }
    f.read(reinterpret_cast<char*>(data_.data()), (long)data_.size());
    if (!f) return false;
    ready_ = true;
    return true;
}

// PatternDatabases

bool PatternDatabases::load_or_build(const std::string& data_dir) {
    std::string corner_path      = data_dir + "/corner_pattern.db";
    std::string edge_orient_path = data_dir + "/edge_orient.db";
    std::string edge1_path       = data_dir + "/edge1_pattern.db";
    std::string edge2_path       = data_dir + "/edge2_pattern.db";

    bool loaded_from_disk = true;

    auto try_load_or_build = [&](auto& db, const std::string& path,
                                 const std::string& name, auto build_fn) {
        if (!db.load(path)) {
            std::cerr << name << " not found, building...\n";
            build_fn();
            if (!db.save(path))
                std::cerr << "Warning: could not save " << name << " to " << path << "\n";
            loaded_from_disk = false;
        } else {
            std::cerr << "Loaded " << name << " from " << path << "\n";
        }
    };

    try_load_or_build(corner_db, corner_path, "corner pattern DB",
                      [&]{ corner_db.build(); });
    try_load_or_build(edge_orient_db, edge_orient_path, "edge orient DB",
                      [&]{ edge_orient_db.build(); });
    try_load_or_build(edge_db1, edge1_path, "edge pattern DB 0",
                      [&]{ edge_db1.build(0); });
    try_load_or_build(edge_db2, edge2_path, "edge pattern DB 1",
                      [&]{ edge_db2.build(1); });

    return loaded_from_disk;
}

int PatternDatabases::heuristic(const CubeState& state) const {
    int h = 0;
    if (corner_db.is_ready())    h = std::max(h, (int)corner_db.lookup(state));
    if (edge_orient_db.is_ready()) h = std::max(h, (int)edge_orient_db.lookup(state));
    if (edge_db1.is_ready())     h = std::max(h, (int)edge_db1.lookup(state.ep.data(), state.eo.data(), 0));
    if (edge_db2.is_ready())     h = std::max(h, (int)edge_db2.lookup(state.ep.data(), state.eo.data(), 1));
    return h;
}
