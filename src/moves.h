#pragma once
#include "cube.h"
#include <array>
#include <string>
#include <vector>

// 18 moves in standard WCA notation.
// Indices: U=0 U'=1 U2=2, D=3 D'=4 D2=5, F=6 F'=7 F2=8,
//          B=9 B'=10 B2=11, L=12 L'=13 L2=14, R=15 R'=16 R2=17

static constexpr int NUM_MOVES = 18;

extern const std::array<std::string, NUM_MOVES> MOVE_NAMES;

enum MoveIndex : int {
    U=0, Up=1, U2=2,
    D=3, Dp=4, D2=5,
    F=6, Fp=7, F2=8,
    B=9, Bp=10, B2=11,
    L=12, Lp=13, L2=14,
    R=15, Rp=16, R2=17
};

// MOVE_TABLE[m] = CubeState for a single move m (from solved).
// apply_move(state, m) = compose(MOVE_TABLE[m], state)
extern std::array<CubeState, NUM_MOVES> MOVE_TABLE;

// INVERSE_MOVE[m] = the move that undoes m
extern const std::array<int, NUM_MOVES> INVERSE_MOVE;

// FACE_OF_MOVE[m]: 0=U, 1=D, 2=F, 3=B, 4=L, 5=R
extern const std::array<int, NUM_MOVES> FACE_OF_MOVE;

void init_moves();  // call once at startup

CubeState apply_move (const CubeState& state, int move_id);
CubeState apply_moves(const CubeState& start, const std::vector<int>& moves);

// Returns false if applying curr after prev is redundant (same face,
// or opposite face in non-canonical order).
bool is_redundant_sequence(int prev_move, int curr_move);

std::vector<int> parse_move_sequence (const std::string& seq);
std::string      format_move_sequence(const std::vector<int>& moves);
std::vector<int> random_scramble(int depth, unsigned seed = 0);
