# Rubik's Cube Solver — Project Notes for Claude

## What This Is

An optimal 3×3 Rubik's cube solver in C++17. Uses IDA* with pattern databases
to find guaranteed minimum-move solutions (≤20 moves, per God's Number). Based
on Korf's 1997 paper.

## Current State (End of Phase 1)

**Done:**
- Cube state representation: `CubeState` with `cp[8]`, `co[8]`, `ep[12]`, `eo[12]`
- All 18 moves (U U' U2 ... R R' R2) as pre-computed `CubeState` transforms
- IDA* solver with admissible misplaced-cubies heuristic
- Corner pattern DB (88M states) — build code written, not yet built
- Edge orientation DB (2048 states) — working and verified
- CLI: `--scramble`, `--random-depth`, `--interactive`, `--no-pattern-db`
- 28 unit tests, all passing (CTest)

**Not yet started (Phase 3+):**
- Actually building and saving the corner pattern DB (~5–10 min one-time build)
- Performance optimisation (coordinate move tables, symmetry reduction)
- Disjoint pattern databases

## Building

```bash
# Debug (for development)
cmake -B build && cmake --build build
ctest --output-on-failure   # run all tests

# Release (for actual solving)
cmake -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release

# Build pattern databases (one-time, ~5-10 min)
mkdir -p data
./build-release/build_pattern_db --data-dir data

# Solve a scramble
./build-release/cube_solver --scramble "R U R' U' F2 D' L B"
./build-release/cube_solver --random-depth 15 --data-dir data
```

## Key Architecture Decisions

### Cube representation
`CubeState` stores position and orientation arrays separately (not bitpacked).
Readable and correct comes first; bitpacking is a later optimisation if needed.

### Composition convention
`compose(a, b)` means **apply b first, then a** — i.e., a∘b in the mathematical
sense. So `compose(a, b).cp[i] = a.cp[b.cp[i]]`.

`apply_move(state, m)` = `compose(MOVE_TABLE[m], state)` — left multiplication.

The pattern DB BFS uses **right multiplication** (`compose(state, mv)`), which
gives the same distances as left multiplication (both are min word length in the
generator set), so it's valid.

### BFS formulas (right-multiplication: compose(state, mv))
```
new_cp[i] = cp[mv.cp[i]]
new_co[i] = (co[mv.cp[i]] + mv.co[i]) % 3
new_ep[i] = ep[mv.ep[i]]
new_eo[i] = (eo[mv.ep[i]] + mv.eo[i]) % 2   ← important: mv.ep[i] not i
```
The edge orient formula uses `mv.ep[i]` because the edge **arriving** at position i
came from position `mv.ep[i]`, and carries its old orientation. The bug of using
`eo[i]` instead would only expand 4 states instead of 2048.

### Move orientation convention
`mv.co[i]` / `mv.eo[i]` is the orientation **delta added at the destination**
(position i), not at the source. Corner orientation twists: U/D moves = 0 delta,
F/B/L/R = alternating 1/2 pattern on the cycled corners. Edge orientation flips:
only F, F', B, B' flip edges (4 edges each); U/D/L/R and all half-turns don't.

### Pattern DB storage
Corner DB: packed 4-bit nibbles → 44,089,920 bytes (~42 MB on disk, 88M entries).
Edge orient DB: 1 byte per entry → 2048 bytes.

## File Layout

```
src/
  cube.h / cube.cpp         — CubeState, compose, inverse, Lehmer encoding
  moves.h / moves.cpp       — MOVE_TABLE, apply_move, parse/format, pruning
  solver.h / solver.cpp     — IDASolver (IDA*), heuristic_misplaced
  pattern_db.h / pattern_db.cpp  — CornerPatternDB, EdgeOrientDB, PatternDatabases
  heuristic.h / heuristic.cpp    — wrapper functions, make_heuristic()
  utils.h / utils.cpp       — validate_cube, print_cube_ascii, format_solve_stats
  main.cpp                  — CLI
  build_db_main.cpp         — standalone DB builder
tests/
  test_moves.cpp   — 14 tests: moves, inverses, encoding, parsing
  test_solver.cpp  — 9 tests: IDA* correctness, optimality, validation
  test_pattern_db.cpp — 5 tests: edge orient DB (2048 states, admissibility)
data/                        — generated DB files (NOT committed to git)
  corner_pattern.db          — 42 MB, built by build_pattern_db
  edge_orient.db             — 2 KB, built by build_pattern_db
```

## Common Gotchas

- `decode_corner_perm` uses `std::vector` (not `std::array`) for `available`
  because `std::array` has no `erase`. Don't change this to array.
- The misplaced heuristic divides corners and edges **separately** then takes max.
  Combining them and dividing by 4 is NOT admissible (one move fixes 4+4=8 cubies).
- Pattern DB files must exist before running `cube_solver` without `--no-pattern-db`.
  First run: either pass `--no-pattern-db` or run `build_pattern_db` first.
- When resuming Phase 3 work: start with building the corner DB and verifying
  that solved state = 0 and 1-move states = 1 before doing anything else.

## Phase 3 Plan (next session)

1. Build corner pattern DB, verify correctness
2. Integrate corner DB into IDA* solver, benchmark vs simple heuristic
3. Benchmark 15-move scrambles — should be < 1s
4. Consider adding edge position DB (6-edge subset) for better heuristic
5. Performance: coordinate move tables (avoid full `compose()` in hot loop)
