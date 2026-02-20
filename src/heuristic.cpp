#include "heuristic.h"
#include <algorithm>

int heuristic_corner_db(const CubeState& state, const CornerPatternDB& corner_db) {
    return (int)corner_db.lookup(state);
}

int heuristic_combined(const CubeState& state, const PatternDatabases& dbs) {
    return dbs.heuristic(state);
}

std::function<int(const CubeState&)> make_heuristic(const PatternDatabases& dbs) {
    return [&dbs](const CubeState& state) -> int {
        return dbs.heuristic(state);
    };
}

std::function<int(const CubeState&)> make_heuristic_corner_only(const CornerPatternDB& db) {
    return [&db](const CubeState& state) -> int {
        return (int)db.lookup(state);
    };
}
