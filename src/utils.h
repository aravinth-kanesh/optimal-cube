#pragma once
#include "cube.h"
#include "moves.h"
#include <string>
#include <vector>

// Verify that the cube state is internally consistent
// (valid permutation, valid orientation sums)
bool validate_cube(const CubeState& state, std::string& error_msg);

// Print a visual ASCII representation of the cube
// (shows face stickers in a cross layout)
void print_cube_ascii(const CubeState& state);

// Format a solve result summary
std::string format_solve_stats(const struct SolveResult& result);
