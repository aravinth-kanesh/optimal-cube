#include "cube.h"
#include "moves.h"
#include "pattern_db.h"
#include <iostream>
#include <string>
#include <filesystem>
#include <cstring>

int main(int argc, char* argv[]) {
    std::string data_dir = "data";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--data-dir") == 0 && i + 1 < argc) {
            data_dir = argv[++i];
        } else {
            std::cerr << "Usage: " << argv[0] << " [--data-dir DIR]\n";
            return 1;
        }
    }

    std::filesystem::create_directories(data_dir);

    std::cerr << "Initialising move tables...\n";
    init_moves();

    PatternDatabases dbs;

    std::string corner_path = data_dir + "/corner_pattern.db";
    std::string edge0_path  = data_dir + "/edge_orient.db";
    std::string edge1_path  = data_dir + "/edge1_pattern.db";
    std::string edge2_path  = data_dir + "/edge2_pattern.db";

    auto build_and_save = [](auto& db, const std::string& path, auto build_fn) {
        build_fn();
        if (db.save(path)) std::cerr << "Saved to " << path << "\n";
        else { std::cerr << "Failed to save to " << path << "!\n"; return false; }
        return true;
    };

    std::cerr << "\nBuilding corner pattern database...\n";
    if (!build_and_save(dbs.corner_db, corner_path, [&]{ dbs.corner_db.build(); }))
        return 1;

    std::cerr << "\nBuilding edge orientation database...\n";
    if (!build_and_save(dbs.edge_orient_db, edge0_path, [&]{ dbs.edge_orient_db.build(); }))
        return 1;

    std::cerr << "\nBuilding edge pattern database (group 0: edges 0-5)...\n";
    if (!build_and_save(dbs.edge_db1, edge1_path, [&]{ dbs.edge_db1.build(0); }))
        return 1;

    std::cerr << "\nBuilding edge pattern database (group 1: edges 6-11)...\n";
    if (!build_and_save(dbs.edge_db2, edge2_path, [&]{ dbs.edge_db2.build(1); }))
        return 1;

    std::cerr << "\nAll databases built successfully.\n\nVerification:\n";
    std::cerr << "  Solved corner:  " << (int)dbs.corner_db.lookup(SOLVED_CUBE) << " (expected 0)\n";
    std::cerr << "  Solved edge0:   " << (int)dbs.edge_db1.lookup(SOLVED_CUBE.ep.data(), SOLVED_CUBE.eo.data(), 0) << " (expected 0)\n";
    std::cerr << "  Solved edge1:   " << (int)dbs.edge_db2.lookup(SOLVED_CUBE.ep.data(), SOLVED_CUBE.eo.data(), 1) << " (expected 0)\n";

    auto moves = parse_move_sequence("R U R' U'");
    auto state = apply_moves(SOLVED_CUBE, moves);
    std::cerr << "  After 'R U R' U'': corner=" << (int)dbs.corner_db.lookup(state)
              << " edge0=" << (int)dbs.edge_db1.lookup(state.ep.data(), state.eo.data(), 0)
              << " edge1=" << (int)dbs.edge_db2.lookup(state.ep.data(), state.eo.data(), 1) << "\n";

    return 0;
}
