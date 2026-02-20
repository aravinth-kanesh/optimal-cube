#pragma once
#include "cube.h"
#include "pattern_db.h"
#include <functional>

// ============================================================
// Heuristic functions for IDA* solver
// ============================================================
//
// All heuristics are ADMISSIBLE: they never overestimate the
// true minimum number of moves needed to solve the cube.
//
// Better (larger) heuristics prune more nodes → faster solving.
// ============================================================

// Heuristic using only corner pattern database
// h = corner_db.lookup(state)
// Admissible because solving corners is a lower bound on solving the full cube
int heuristic_corner_db(const CubeState& state, const CornerPatternDB& corner_db);

// Heuristic using max of corner DB + edge orientation DB
// h = max(corner_db, edge_orient_db)
// Better than corner alone because it also considers edge orientations
int heuristic_combined(const CubeState& state, const PatternDatabases& dbs);

// Wrapper that creates a HeuristicFn from PatternDatabases
// (for use with IDASolver)
std::function<int(const CubeState&)> make_heuristic(const PatternDatabases& dbs);
std::function<int(const CubeState&)> make_heuristic_corner_only(const CornerPatternDB& db);
