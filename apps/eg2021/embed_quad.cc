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
    register_segfault_handler();

    fs::path layout_path;
    fs::path target_path;
    std::string algo = "bnb";
    bool smooth = false;
    int max_subdiv = 12;
    double edge_length = 0.05;

    cxxopts::Options opts("embed_quads",
        "Creates a quad mesh from a target mesh using an embedded layout as a base complex.\n"
        "Layout connectivity is provided as a polygon mesh.\n"
        "Layout vertices are projected to target surface to define landmark positions.\n"
        "\n"
        "Output files are written to <build-folder>/output/embed.\n"
        "\n"
        "Supported algorithms are:\n"
        "    bnb:       Branch-and-bound algorithm (default)\n"
        "    greedy:    Greedy algorithm, always choosing shortest path\n"
        "    praun:     Greedy algorithm with heuristic based on [Praun2001]\n"
        "    kraevoy:   Greedy algorithm with heuristic based on [Kraevoy2003] / [Kraevoy2004]\n"
        "    schreiner: Greedy algorithm with heuristic based on [Schreiner2004]\n");
    opts.add_options()("l,layout", "Path to layout mesh.", cxxopts::value<std::string>());
    opts.add_options()("t,target", "Path to target mesh. Must be a triangle mesh.", cxxopts::value<std::string>());
    opts.add_options()("a,algo", "Algorithm, one of: bnb, greedy, praun, kraevoy, schreiner.", cxxopts::value<std::string>()->default_value("bnb"));
    opts.add_options()("s,smooth", "Apply smoothing post-process based on [Praun2001].", cxxopts::value<bool>());

    opts.add_options()("e,edge_length", "Quad edge length.", cxxopts::value<double>());
    opts.add_options()("m,max_subdiv", "Max base complex subdivisions.", cxxopts::value<int>());

    opts.add_options()("h,help", "Help.");
    opts.parse_positional({"layout", "target"});
    opts.positional_help("[layout] [target]");
    opts.show_positional_help();
    try {
        auto args = opts.parse(argc, argv);

        layout_path = args["layout"].as<std::string>();
        target_path = args["target"].as<std::string>();

        edge_length = args["edge_length"].as<double>();
        max_subdiv = args["max_subdiv"].as<int>();

        algo = args["algo"].as<std::string>();
        const std::set<std::string> valid_algos = { "bnb", "greedy", "praun", "kraevoy", "schreiner" };
        if (valid_algos.count(algo) == 0) {
            throw cxxopts::OptionException("Invalid algo: " + algo);
        }

        smooth = args["smooth"].as<bool>();

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

    // glow::glfw::GlfwContext ctx;

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
        BranchAndBoundSettings settings;
        // settings.time_limit = 60;
        // settings.optimality_gap = 0.02;
        branch_and_bound(em, settings);
    }        
    else
        LE_ASSERT(false);

    // Smooth embedding
    if (smooth)
        em = smooth_paths(em);

    // Save embedding
    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "embed";
    fs::create_directories(output_dir);
    auto embed_path = output_dir / target_path.stem();
    em.save(embed_path);
    std::cout << "saved embedding: " << embed_path.string() << std::endl;

    // Compute integer-grid map
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
