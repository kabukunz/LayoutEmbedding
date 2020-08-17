#pragma once

#include <LayoutEmbedding/Embedding.hh>

namespace LayoutEmbedding {

/// Layout and Embedding side-by-side in a grid view
void view_embedding(const Embedding& _em);

/// Layout mesh with colorized embedded edges
void view_layout(const Embedding& _em);

/// Target mesh with colorized embedded edge paths
void view_target(const Embedding& _em);

pm::vertex_attribute<tg::pos3> make_layout_mesh_positions(const Embedding& _em);

void view_layout_mesh(const Embedding& _em);

void view_target_mesh(const Embedding& _em);

void view_path(const Embedding& _em, const std::vector<pm::vertex_handle>& _path, const tg::color3& _color);
void view_path(const Embedding& _em, const VirtualPath& _path, const tg::color3& _color);

void view_edge(const pm::vertex_attribute<tg::pos3>& _pos, const pm::edge_handle& _e, const tg::color3& _color);
void view_vertex(const pm::vertex_attribute<tg::pos3>& _pos, const pm::vertex_handle& _v, const tg::color3& _color);

}
