#include "moves.h"
#include <cassert>
#include <stdexcept>
#include <sstream>
#include <random>
#include <algorithm>

const std::array<std::string, NUM_MOVES> MOVE_NAMES = {
    "U", "U'", "U2",
    "D", "D'", "D2",
    "F", "F'", "F2",
    "B", "B'", "B2",
    "L", "L'", "L2",
    "R", "R'", "R2"
};

const std::array<int, NUM_MOVES> INVERSE_MOVE = {
    Up, U, U2,
    Dp, D, D2,
    Fp, F, F2,
    Bp, B, B2,
    Lp, L, L2,
    Rp, R, R2
};

// Face index: 0=U, 1=D, 2=F, 3=B, 4=L, 5=R
const std::array<int, NUM_MOVES> FACE_OF_MOVE = {
    0, 0, 0,
    1, 1, 1,
    2, 2, 2,
    3, 3, 3,
    4, 4, 4,
    5, 5, 5
};

std::array<CubeState, NUM_MOVES> MOVE_TABLE;

// Build a CubeState representing one 90° clockwise face turn.
// corner_cycle[4]: the 4 corner positions involved (a, b, c, d).
// After the turn: position b gets the cubie from a, c from b, d from c, a from d.
// corner_orients[4]: orientation delta at each destination (b, c, d, a).
// Same pattern for edges.
//
// Orientation notes:
//   U, D: no orientation change (delta = 0).
//   F, B: edges flip (delta = 1); corners alternate +2/+1 deltas.
//   L, R: no edge orientation change; corners alternate +1/+2 deltas.
static CubeState make_face_move(
    const int corner_cycle[4], const int corner_orients[4],
    const int edge_cycle[4],   const int edge_orients[4])
{
    CubeState m = SOLVED_CUBE;

    int a = corner_cycle[0], b = corner_cycle[1],
        c = corner_cycle[2], d = corner_cycle[3];
    m.cp[b] = (uint8_t)a; m.co[b] = (uint8_t)corner_orients[0];
    m.cp[c] = (uint8_t)b; m.co[c] = (uint8_t)corner_orients[1];
    m.cp[d] = (uint8_t)c; m.co[d] = (uint8_t)corner_orients[2];
    m.cp[a] = (uint8_t)d; m.co[a] = (uint8_t)corner_orients[3];

    int ea = edge_cycle[0], eb = edge_cycle[1],
        ec = edge_cycle[2], ed = edge_cycle[3];
    m.ep[eb] = (uint8_t)ea; m.eo[eb] = (uint8_t)edge_orients[0];
    m.ep[ec] = (uint8_t)eb; m.eo[ec] = (uint8_t)edge_orients[1];
    m.ep[ed] = (uint8_t)ec; m.eo[ed] = (uint8_t)edge_orients[2];
    m.ep[ea] = (uint8_t)ed; m.eo[ea] = (uint8_t)edge_orients[3];

    return m;
}

static CubeState make_U() {
    const int cc[4] = {URF, UFL, ULB, UBR}, co[4] = {0,0,0,0};
    const int ec[4] = {UR,  UF,  UL,  UB},  eo[4] = {0,0,0,0};
    return make_face_move(cc, co, ec, eo);
}

static CubeState make_D() {
    const int cc[4] = {DFR, DLF, DBL, DRB}, co[4] = {0,0,0,0};
    const int ec[4] = {DR,  DF,  DL,  DB},  eo[4] = {0,0,0,0};
    return make_face_move(cc, co, ec, eo);
}

// F: corners cycle URF←DFR←DLF←UFL, orientation deltas {2,1,2,1}.
//    Edges cycle UF←FR←DF←FL, all flipped.
static CubeState make_F() {
    const int cc[4] = {URF, UFL, DLF, DFR}, co[4] = {2,1,2,1};
    const int ec[4] = {UF,  FL,  DF,  FR},  eo[4] = {1,1,1,1};
    return make_face_move(cc, co, ec, eo);
}

static CubeState make_B() {
    const int cc[4] = {UBR, ULB, DBL, DRB}, co[4] = {1,2,1,2};
    const int ec[4] = {UB,  BL,  DB,  BR},  eo[4] = {1,1,1,1};
    return make_face_move(cc, co, ec, eo);
}

static CubeState make_L() {
    const int cc[4] = {UFL, ULB, DBL, DLF}, co[4] = {1,2,1,2};
    const int ec[4] = {UL,  BL,  DL,  FL},  eo[4] = {0,0,0,0};
    return make_face_move(cc, co, ec, eo);
}

static CubeState make_R() {
    const int cc[4] = {URF, UBR, DRB, DFR}, co[4] = {2,1,2,1};
    const int ec[4] = {UR,  FR,  DR,  BR},  eo[4] = {0,0,0,0};
    return make_face_move(cc, co, ec, eo);
}

void init_moves() {
    CubeState U_cw = make_U(), D_cw = make_D(), F_cw = make_F(),
              B_cw = make_B(), L_cw = make_L(), R_cw = make_R();

    MOVE_TABLE[MoveIndex::U]  = U_cw;
    MOVE_TABLE[MoveIndex::D]  = D_cw;
    MOVE_TABLE[MoveIndex::F]  = F_cw;
    MOVE_TABLE[MoveIndex::B]  = B_cw;
    MOVE_TABLE[MoveIndex::L]  = L_cw;
    MOVE_TABLE[MoveIndex::R]  = R_cw;

    MOVE_TABLE[MoveIndex::Up] = inverse(U_cw);
    MOVE_TABLE[MoveIndex::Dp] = inverse(D_cw);
    MOVE_TABLE[MoveIndex::Fp] = inverse(F_cw);
    MOVE_TABLE[MoveIndex::Bp] = inverse(B_cw);
    MOVE_TABLE[MoveIndex::Lp] = inverse(L_cw);
    MOVE_TABLE[MoveIndex::Rp] = inverse(R_cw);

    MOVE_TABLE[MoveIndex::U2] = compose(U_cw, U_cw);
    MOVE_TABLE[MoveIndex::D2] = compose(D_cw, D_cw);
    MOVE_TABLE[MoveIndex::F2] = compose(F_cw, F_cw);
    MOVE_TABLE[MoveIndex::B2] = compose(B_cw, B_cw);
    MOVE_TABLE[MoveIndex::L2] = compose(L_cw, L_cw);
    MOVE_TABLE[MoveIndex::R2] = compose(R_cw, R_cw);
}

CubeState apply_move(const CubeState& state, int move_id) {
    return compose(MOVE_TABLE[move_id], state);
}

// Pruning: skip curr_move if it can't possibly improve over a merged or
// reordered sequence. Avoids searching the same sequences multiple times.
bool is_redundant_sequence(int prev_move, int curr_move) {
    if (prev_move < 0) return false;

    int prev_face = FACE_OF_MOVE[prev_move];
    int curr_face = FACE_OF_MOVE[curr_move];

    if (prev_face == curr_face) return true;

    // On opposite faces (same axis), allow only canonical order (lower index first).
    static const int OPPOSITE_FACE[6] = {1, 0, 3, 2, 5, 4};
    if (curr_face == OPPOSITE_FACE[prev_face] && curr_face < prev_face)
        return true;

    return false;
}

std::vector<int> parse_move_sequence(const std::string& seq) {
    std::vector<int> result;
    std::istringstream iss(seq);
    std::string token;
    while (iss >> token) {
        bool found = false;
        for (int i = 0; i < NUM_MOVES; i++) {
            if (MOVE_NAMES[i] == token) { result.push_back(i); found = true; break; }
        }
        if (!found) throw std::invalid_argument("Unknown move: " + token);
    }
    return result;
}

std::string format_move_sequence(const std::vector<int>& moves) {
    std::string result;
    for (size_t i = 0; i < moves.size(); i++) {
        if (i > 0) result += ' ';
        result += MOVE_NAMES[moves[i]];
    }
    return result;
}

CubeState apply_moves(const CubeState& start, const std::vector<int>& moves) {
    CubeState state = start;
    for (int m : moves) state = apply_move(state, m);
    return state;
}

std::vector<int> random_scramble(int depth, unsigned seed) {
    std::mt19937 rng(seed == 0 ? std::random_device{}() : seed);
    std::uniform_int_distribution<int> dist(0, NUM_MOVES - 1);

    std::vector<int> scramble;
    scramble.reserve(depth);

    int prev_move = -1;
    for (int i = 0; i < depth; i++) {
        int m, attempts = 0;
        do { m = dist(rng); attempts++; }
        while (is_redundant_sequence(prev_move, m) && attempts < 100);
        scramble.push_back(m);
        prev_move = m;
    }
    return scramble;
}
