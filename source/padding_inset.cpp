/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2026 by Yoshiaki Tazaki.
 * All rights reserved.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "padding_inset.hpp"
#include "util.hpp"

namespace PaddingInset
{
    using namespace InsetMath;

    struct PNode
    {
        V2     co;
        size_t orig_idx = 0;  // associated original vertex index
        PNode* next     = nullptr;
        PNode* prev     = nullptr;
    };

    static void link_pnodes(std::vector<PNode*>& nodes)
    {
        size_t count = nodes.size();
        for (size_t i = 0; i < count; ++i)
        {
            size_t next_i       = (i + 1) % count;
            nodes[i]->next      = nodes[next_i];
            nodes[next_i]->prev = nodes[i];
        }
    }

    static bool is_concave_2d(const V2& p_prev, const V2& p_curr, const V2& p_next)
    {
        V2     m1 = p_next - p_curr;
        V2     m2 = p_prev - p_curr;
        double c  = cross(m1, m2);
        return c <= 0.0;
    }

    static V2 get_mid_2d(const V2& m1, const V2& m2)
    {
        V2 c1{ -m1.y, m1.x };
        V2 c2{ m2.y, -m2.x };
        V2 c3 = c1.normalized() + c2.normalized();
        return c3.normalized();
    }

    static V2 turn_2d(const V2& m1)
    {
        V2 n = m1.normalized();
        return V2{ -n.y, n.x };
    }

    static bool find_intersection(const std::vector<PNode*>& bs, PNode* p1, PNode* p2, V2& pin, PNode*& out_p3, PNode*& out_p4)
    {
        for (PNode* p3 : bs)
        {
            if (!p3 || !p3->next)
                continue;
            PNode* p4 = p3->next;

            if (p1 == p3 || p1 == p4 || p2 == p3 || p2 == p4)
                continue;

            V2 inter;
            if (intersect_segments_2d(p1->co, p2->co, p3->co, p4->co, inter))
            {
                double           d1  = (inter - p1->co).length();
                double           d2  = (inter - p2->co).length();
                double           d3  = (inter - p3->co).length();
                double           d4  = (inter - p4->co).length();
                constexpr double ths = 1e-5;
                if (d1 <= ths || d2 <= ths || d3 <= ths || d4 <= ths)
                    continue;

                pin    = inter;
                out_p3 = p3;
                out_p4 = p4;
                return true;
            }
        }
        return false;
    }

    static std::vector<std::vector<PNode*>> divide_all(std::vector<PNode*>& bs_in, std::vector<PNode*>& allocated_nodes)
    {
        std::vector<PNode*> bs = bs_in;
        std::vector<PNode*> pps;
        pps.reserve(bs.size() * 2);

        while (!bs.empty())
        {
            PNode* p1 = bs.back();
            bs.pop_back();
            PNode* p2 = p1->next;
            if (!p2)
                continue;

            V2     pin;
            PNode* p3 = nullptr;
            PNode* p4 = nullptr;
            if (!find_intersection(bs, p1, p2, pin, p3, p4))
            {
                pps.push_back(p1);
                continue;
            }

            PNode* pk1 = new PNode{ pin, p2->orig_idx, p4, p1 };
            PNode* pk2 = new PNode{ pin, p4->orig_idx, p2, p3 };
            allocated_nodes.push_back(pk1);
            allocated_nodes.push_back(pk2);

            p1->next = pk1;
            p4->prev = pk1;
            p3->next = pk2;
            p2->prev = pk2;

            bs.push_back(pk1);
            bs.push_back(pk2);
            bs.push_back(p1);
        }

        for (PNode* p : bs)
            pps.push_back(p);

        // Extract simple closed loops
        std::set<PNode*>                 visited;
        std::vector<std::vector<PNode*>> polys;

        for (PNode* start : pps)
        {
            if (visited.count(start))
                continue;

            std::vector<PNode*> loop;
            PNode*              curr   = start;
            bool                closed = false;

            while (curr && !visited.count(curr))
            {
                visited.insert(curr);
                loop.push_back(curr);
                curr = curr->next;
                if (curr == start)
                {
                    closed = true;
                    break;
                }
            }

            if (closed && loop.size() >= 3)
            {
                polys.push_back(loop);
            }
        }

        return polys;
    }

    static V2 find_center_point(const std::vector<PNode*>& poly, const V2& direction)
    {
        V2 cen{ 0.0, 0.0 };
        for (PNode* p : poly)
            cen = cen + p->co;
        cen = cen / static_cast<double>(poly.size());

        std::vector<V2> ins;
        for (PNode* p1 : poly)
        {
            PNode* p2 = p1->next;
            if (!p2)
                continue;
            V2 pin;
            if (intersect_segments_2d(cen, cen + direction * 1000.0, p1->co, p2->co, pin))
            {
                ins.push_back(pin);
            }
        }

        if (ins.size() % 2 == 1)
            return cen;
        else if (ins.size() >= 2)
            return (ins[0] + ins[1]) * 0.5;

        return cen;
    }

    static bool is_loop_valid_inset(const std::vector<std::vector<PNode*>>& loops,
                                    const std::vector<PNode*>&                  loop,
                                    const V2&                                   direction)
    {
        if (loop.size() < 3)
            return false;

        const V2 center = find_center_point(loop, direction);
        int      winding = 0;
        for (const auto& boundary : loops)
        {
            for (size_t i = 0; i < boundary.size(); ++i)
            {
                PNode* p1 = boundary[i];
                PNode* p2 = boundary[(i + 1) % boundary.size()];
                V2     intersection;
                if (!intersect_segments_2d(center, center + direction * 1000.0, p1->co, p2->co, intersection))
                    continue;

                if (cross(p1->co - center, direction) > 0.0)
                    ++winding;
                else
                    --winding;
            }
        }

        return winding > 0;
    }

    static void merge_close_points(std::vector<V2>& pts, std::vector<size_t>& orig_indices, double dist_eps = 0.001)
    {
        bool            changed = true;
        while (changed)
        {
            changed = false;
            std::vector<V2> next_pts;
            std::vector<size_t> next_orig_indices;
            size_t          n = pts.size();
            for (size_t i = 0; i < n; ++i)
            {
                size_t next_i = (i + 1) % n;
                if ((pts[next_i] - pts[i]).length() < dist_eps)
                {
                    changed = true;
                    continue;
                }
                next_pts.push_back(pts[i]);
                next_orig_indices.push_back(orig_indices[i]);
            }
            pts = std::move(next_pts);
            orig_indices = std::move(next_orig_indices);
            if (pts.size() < 3)
                break;
        }
    }

    bool ComputePaddingInset(const std::vector<V3>& input_vertices, const V3& normal, const PaddingInsetParams& params, InsetResult& result)
    {
        size_t n = input_vertices.size();
        if (n < 3)
            return false;

        // Basis setup
        Basis3D basis;
        basis.o = input_vertices[0];
        basis.n = normal.normalized();

        V3 x_dir;
        for (size_t i = 0; i < n; ++i)
        {
            size_t next = (i + 1) % n;
            V3     e    = input_vertices[next] - input_vertices[i];
            if (e.length() > 1e-6)
            {
                x_dir = e.normalized();
                break;
            }
        }
        if (x_dir.length() < 1e-6)
            return false;

        basis.x = x_dir;
        basis.y = cross(basis.n, basis.x).normalized();

        // Project input vertices to 2D
        std::vector<V2> ps(n);
        for (size_t i = 0; i < n; ++i)
            ps[i] = basis.project(input_vertices[i]);

        // Ensure CCW orientation
        if (polygon_signed_area_2d(ps) < 0.0)
            std::reverse(ps.begin(), ps.end());

        // Build offset loops bs
        std::vector<PNode*>          bs;
        std::vector<PNode*>          allocated_nodes;
        auto create_node = [&](const V2& co, size_t orig_idx) -> PNode*
        {
            PNode* node = new PNode{ co, orig_idx, nullptr, nullptr };
            allocated_nodes.push_back(node);
            return node;
        };

        for (size_t i = 0; i < n; ++i)
        {
            size_t prev_i = (i + n - 1) % n;
            size_t next_i = (i + 1) % n;

            V2 p1 = ps[i];
            V2 p2 = ps[next_i];

            bool con1 = is_concave_2d(ps[prev_i], ps[i], ps[next_i]);
            bool con2 = is_concave_2d(ps[i], ps[next_i], ps[(next_i + 1) % n]);

            V2 m1_edge = p2 - p1;
            V2 m1_prev = p1 - ps[prev_i];
            V2 m1_next = ps[(next_i + 1) % n] - p2;

            V2 mid_p1 = get_mid_2d(m1_edge, ps[prev_i] - p1);
            V2 mid_p2 = get_mid_2d(m1_next, p1 - p2);

            double theta1 = InsetMath::angle(m1_edge, ps[prev_i] - p1);
            double theta2 = InsetMath::angle(m1_next, p1 - p2);

            double sin1 = std::sin(theta1 * 0.5);
            double sin2 = std::sin(theta2 * 0.5);

            if (!con1)
            {
                bs.push_back(create_node(p1, i));
            }
            else
            {
                if (params.flat)
                {
                    V2 pk0 = p1 + mid_p1 * params.inset_size;
                    bs.push_back(create_node(pk0, i));
                }
            }

            V2 mid1;
            if (con1)
            {
                if (!params.flat && sin1 > 1e-4)
                {
                    mid1 = mid_p1 * (params.inset_size / sin1);
                }
                else
                {
                    mid1 = turn_2d(m1_edge) * params.inset_size;
                }
            }
            else
            {
                mid1 = turn_2d(m1_edge) * params.inset_size;
            }

            V2 mid2;
            if (con2)
            {
                if (!params.flat && sin2 > 1e-4)
                {
                    mid2 = mid_p2 * (params.inset_size / sin2);
                }
                else
                {
                    mid2 = turn_2d(m1_edge) * params.inset_size;
                }
            }
            else
            {
                mid2 = turn_2d(m1_edge) * params.inset_size;
            }

            V2 pk1 = p1 + mid1;
            V2 pk2 = p2 + mid2;

            bs.push_back(create_node(pk1, i));
            bs.push_back(create_node(pk2, next_i));
        }

        link_pnodes(bs);

        // Decompose self-intersecting loops
        std::vector<std::vector<PNode*>> bss = divide_all(bs, allocated_nodes);
        const double                  ray_angle = params.align * 3.0;
        const V2                      direction{ -std::sin(ray_angle), std::cos(ray_angle) };

        // Select interior loops
        std::vector<std::vector<V2>>     accepted_islands;
        std::vector<std::vector<size_t>> accepted_orig_indices;

        for (const auto& b1 : bss)
        {
            if (b1.size() < 3)
                continue;

            if (!is_loop_valid_inset(bss, b1, direction))
                continue;

            std::vector<V2>     pts;
            std::vector<size_t> orig_indices;
            pts.reserve(b1.size());
            orig_indices.reserve(b1.size());
            for (PNode* p : b1)
            {
                pts.push_back(p->co);
                orig_indices.push_back(p->orig_idx);
            }

            merge_close_points(pts, orig_indices);
            if (pts.size() < 3)
                continue;

            accepted_islands.push_back(pts);
            accepted_orig_indices.push_back(orig_indices);
        }

        // Clean up allocated PNodes
        for (PNode* node : allocated_nodes)
            delete node;

        if (accepted_islands.empty())
        {
            // Fallback: if offset shrank to zero or no loops survived, return empty
            return false;
        }

        result.all_new_vertices.clear();
        result.all_faces.clear();
        result.is_top_face.clear();

        // 0..n-1: original input vertices
        std::vector<size_t> outer_indices(n);
        for (size_t i = 0; i < n; ++i)
        {
            result.all_new_vertices.push_back(basis.unproject(ps[i]));
            outer_indices[i] = i;
        }

        // Add top face islands
        std::vector<std::vector<size_t>> inner_islands_idx;
        for (size_t k = 0; k < accepted_islands.size(); ++k)
        {
            const auto&         island = accepted_islands[k];
            std::vector<size_t> idxs;
            for (const auto& p2d : island)
            {
                idxs.push_back(result.all_new_vertices.size());
                result.all_new_vertices.push_back(basis.unproject(p2d));
            }

            result.all_faces.push_back(idxs);
            result.is_top_face.push_back(true);
            inner_islands_idx.push_back(idxs);
        }

        // Connect each inset edge to its corresponding section of the original boundary.
        if (params.connect)
        {
            for (size_t k = 0; k < inner_islands_idx.size(); ++k)
            {
                const auto& inner = inner_islands_idx[k];
                const auto& origins = accepted_orig_indices[k];
                for (size_t i = 0; i < inner.size(); ++i)
                {
                    const size_t next_i = (i + 1) % inner.size();
                    // PNode origins are one vertex ahead of the source boundary anchors.
                    const size_t from = (origins[next_i] + n - 1) % n;
                    const size_t to = (origins[(next_i + 1) % inner.size()] + n - 1) % n;

                    std::vector<size_t> face = { inner[next_i], inner[i] };
                    size_t boundary_index = from;
                    for (size_t step = 0; step <= n; ++step)
                    {
                        face.push_back(outer_indices[boundary_index]);
                        if (boundary_index == to)
                            break;
                        boundary_index = (boundary_index + 1) % n;
                    }

                    if (face.size() < 3 || boundary_index != to)
                        continue;

                    result.all_faces.push_back(std::move(face));
                    result.is_top_face.push_back(false);
                }
            }
        }

        result.success = true;
        return true;
    }

    LxResult ProcessPolygon(CLxUser_Mesh&              mesh,
                            CLxUser_Polygon&           polygon,
                            CLxUser_Point&             point,
                            const PaddingInsetParams&  params,
                            std::vector<LXtPolygonID>& created_polys,
                            std::vector<LXtPolygonID>& top_polys)
    {
        unsigned nvert = 0;
        if (LXx_FAIL(polygon.VertexCount(&nvert)) || nvert < 3)
            return LXe_OK;

        LXtID4 polyType = LXiPTYP_FACE;
        polygon.Type(&polyType);

        LXtVector norm;
        polygon.Normal(norm);
        V3 normal_v{ norm[0], norm[1], norm[2] };

        std::vector<LXtPointID> orig_pt_ids(nvert);
        std::vector<V3>         input_vertices(nvert);

        for (unsigned i = 0; i < nvert; ++i)
        {
            polygon.VertexByIndex(i, &orig_pt_ids[i]);
            point.Select(orig_pt_ids[i]);
            LXtFVector pos;
            point.Pos(pos);
            input_vertices[i] = V3{ pos[0], pos[1], pos[2] };
        }

        // Create a space transform for UV mapping
        CSpaceTransform spaceTransform(mesh, polygon);

        InsetResult result;
        if (!ComputePaddingInset(input_vertices, normal_v, params, result) || !result.success)
            return LXe_FAILED;

        std::vector<LXtPointID> modo_point_ids(result.all_new_vertices.size(), nullptr);
        for (size_t i = 0; i < nvert; ++i)
        {
            modo_point_ids[i] = orig_pt_ids[i];
        }
    
        std::vector<LXtPointID> new_pt_ids;

        for (size_t i = nvert; i < result.all_new_vertices.size(); ++i)
        {
            LXtVector p3;
            p3[0] = result.all_new_vertices[i].x;
            p3[1] = result.all_new_vertices[i].y;
            p3[2] = result.all_new_vertices[i].z;

            // Add the new vertex to the mesh and get its point ID
            LXtPointID new_pt = nullptr;
            LxResult rc = point.New(p3, &new_pt);
            if (LXx_FAIL(rc) || !new_pt)
                return LXe_FAILED;
            modo_point_ids[i] = new_pt;
            new_pt_ids.push_back(new_pt);
        }

        // Set UVs for new points using the space transform
        spaceTransform.SetUVs(new_pt_ids);

        LXtPolygonID protoID = polygon.ID();

        for (size_t f = 0; f < result.all_faces.size(); ++f)
        {
            const auto& face_idx = result.all_faces[f];
            if (face_idx.size() < 3)
                continue;

            std::vector<LXtPointID> poly_verts;
            poly_verts.reserve(face_idx.size());
            for (size_t idx : face_idx)
            {
                if (idx < modo_point_ids.size() && modo_point_ids[idx])
                {
                    poly_verts.push_back(modo_point_ids[idx]);
                }
            }

            if (poly_verts.size() < 3)
                continue;

            unsigned int rev = MeshUtil::VertexListIsReverse(mesh, poly_verts, norm);

            LXtPolygonID new_poly = nullptr;
            polygon.Select(protoID);
            if (LXx_OK(polygon.NewProto(polyType, poly_verts.data(), static_cast<unsigned>(poly_verts.size()), rev, &new_poly)) && new_poly)
            {
                created_polys.push_back(new_poly);
                if (result.is_top_face[f])
                {
                    top_polys.push_back(new_poly);
                }
            }
        }

        polygon.Select(protoID);
        polygon.Remove();

        return LXe_OK;
    }
}  // namespace PaddingInset
