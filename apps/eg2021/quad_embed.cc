/**
  * Command line interface to our algorithm.
  */

#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/QuadMeshing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <cxxopts.hpp>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080);
const int screenshot_samples = 64;

int main(int argc, char** argv)
{
    using namespace std;

    register_segfault_handler();

    fs::path layout_path;
    fs::path target_path;
    std::string algo = "bnb";
    // bool smooth = false;
    int smooth_iter = -1;
    int max_subdiv = -1;
    double edge_length = 0.02;

    const auto output_dir = fs::current_path();
    // fs::create_directories(output_dir);

    cxxopts::Options opts("embed_quads",
        "Creates a quad mesh from a target mesh using an embedded layout as a base complex.\n"
        "Layout connectivity is provided as a polygon mesh.\n"
        "Layout vertices are projected to target surface to define landmark positions.\n"
        "\n"
        // "Output files are written to <build-folder>/output/embed.\n"
        // "\n"
        "Supported algorithms are:\n"
        "    bnb:       Branch-and-bound algorithm (default)\n"
        "    greedy:    Greedy algorithm, always choosing shortest path\n"
        "    praun:     Greedy algorithm with heuristic based on [Praun2001]\n"
        "    kraevoy:   Greedy algorithm with heuristic based on [Kraevoy2003] / [Kraevoy2004]\n"
        "    schreiner: Greedy algorithm with heuristic based on [Schreiner2004]\n");
        
    opts.add_options()("t,target", "Path to target mesh. Must be a triangle mesh.", cxxopts::value<std::string>());
    opts.add_options()("l,layout", "Path to layout mesh.", cxxopts::value<std::string>());
    
    opts.add_options()("a,algo", "Algorithm, one of: bnb, greedy, praun, kraevoy, schreiner.", cxxopts::value<std::string>()->default_value("bnb"));

    opts.add_options()("e,edge_length", "Quad edge length.", cxxopts::value<double>()->default_value("0.02"));
    opts.add_options()("m,max_subdiv", "Max base complex subdivisions.", cxxopts::value<int>()->default_value("-1"));

    // opts.add_options()("s,smooth", "Apply smoothing post-process based on [Praun2001].", cxxopts::value<bool>());
    opts.add_options()("i,smooth_iter", "Apply smoothing post-process based on [Praun2001] for n iterations", cxxopts::value<int>()->default_value("-1"));

    opts.add_options()("h,help", "Help.");
    opts.parse_positional({"target", "layout"});
    opts.positional_help("[target] [layout]");
    opts.show_positional_help();
    try {
        auto args = opts.parse(argc, argv);

        layout_path = args["layout"].as<std::string>();
        target_path = args["target"].as<std::string>();

        edge_length = args["edge_length"].as<double>();        
        max_subdiv = args["max_subdiv"].as<int>();
        smooth_iter = args["smooth_iter"].as<int>();

        algo = args["algo"].as<std::string>();
        const std::set<std::string> valid_algos = { "bnb", "greedy", "praun", "kraevoy", "schreiner" };
        if (valid_algos.count(algo) == 0) {
            throw cxxopts::OptionException("Invalid algo: " + algo);
        }

        // smooth = args["smooth"].as<bool>();

        if (args.count("help") || args.count("layout") == 0 || args.count("target") == 0) {
            std::cout << opts.help() << std::endl;
            return 0;
        }
    }
    catch (const cxxopts::OptionException& e) {
        std::cout << e.what() << "\n\n";
        std::cout << opts.help() << std::endl;
        return 1;
    }

    std::cout << "Params: ";
    std::cout << "algo: " << algo << " ";
    std::cout << "edge_length: " << to_string(edge_length) << " ";
    std::cout << "max_subdiv: " << to_string(max_subdiv) << " ";
    std::cout << "smooth_iter: " << to_string(smooth_iter) << std::endl;

    // Load input
    EmbeddingInput input;
    input.load(layout_path, target_path);

    // Compute embedding
    Embedding em(input);
    if (algo == "greedy")
        embed_greedy(em);
    else if (algo == "praun")
        embed_praun(em);
    else if (algo == "kraevoy")
        embed_kraevoy(em);
    else if (algo == "schreiner")
        embed_schreiner(em);
    else if (algo == "bnb")
    {
        // TODO: what's this?
        BranchAndBoundSettings settings;
        settings.time_limit = 60;
        settings.optimality_gap = 0.02;
        branch_and_bound(em, settings);

        // branch_and_bound(em);
    }        
    else
        LE_ASSERT(false);
    
    // Smooth embedding
    if (smooth_iter > 0)
        em = smooth_paths(em, smooth_iter);

    // Save embedding files (inp, lem)
    auto embed_path = output_dir / target_path.stem();
    em.save(embed_path);
    std::cout << "saved embedding: " << embed_path.string() << std::endl;

    // Compute integer-grid map
    if (max_subdiv == -1)
        max_subdiv = 12;
    const auto l_subdivisions = choose_loop_subdivisions(em, edge_length, max_subdiv);
    const auto param = parametrize_patches(em, l_subdivisions);

    // Extract quad mesh
    pm::Mesh q;
    pm::face_attribute<pm::face_handle> q_matching_layout_face;
    const auto q_pos = extract_quad_mesh(em, param, q, q_matching_layout_face);

    // Save quad mesh
    const fs::path quad_obj_path = output_dir / (target_path.stem().string() + "_" + "quad.obj");
    std::cout << "quad mesh: " << quad_obj_path.string() << std::endl;
    pm::save(quad_obj_path.string(), q_pos);

}
