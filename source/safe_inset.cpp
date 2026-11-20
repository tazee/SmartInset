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

#include "safe_inset.hpp"
#include "util.hpp"
#include <iostream>

namespace SafeInset
{
    using namespace InsetMath;

    struct SafeLoopNode
    {
        V2     co;
        V2     mid;
        double deg = 0.0;
        V2     next_co;
        size_t index = 0;
    };

    static V2 get_mid_2d(const V2& m1, const V2& m2)
    {
        // c1 = -(m1 x z) in 2D = (-m1.y, m1.x)
        // c2 = m2 x z in 2D = (m2.y, -m2.x)
        V2 c1{ -m1.y, m1.x };
        V2 c2{ m2.y, -m2.x };
        V2 c1n = c1.normalized();
        V2 c2n = c2.normalized();
        V2 c3  = c1n + c2n;
        return c3.normalized();
    }

    static bool point_line_near_3d(const V3& pin, const V3& pe1, const V3& pe2, double offset)
    {
        V3     m1     = pe2 - pe1;
        V3     m2     = pin - pe1;
        double len_sq = dot(m1, m1);
        if (len_sq < 1e-12)
            return (pin - pe1).length() < offset;
        double pro   = dot(m1, m2) / len_sq;
        V3     pro_v = pe1 + m1 * pro;
        V3     h1    = pin - pro_v;
        return h1.length() < offset;
    }

    static bool line_poly_intersect_3d(const V3& p1, const V3& p2, const std::vector<V3>& poly_in, const V3& normal, double offset, V3& out_pin)
    {
        if (poly_in.size() != 4)
            return false;

        V3 cen = (poly_in[0] + poly_in[1] + poly_in[2] + poly_in[3]) * 0.25;

        V3 pin;
        if (!line_plane_intersection(p1, p2, cen, normal, pin))
            return false;

        std::vector<V3> poly(4);
        for (size_t i = 0; i < 4; ++i)
        {
            V3 m1   = poly_in[i] - cen;
            V3 m2   = m1.normalized();
            poly[i] = cen + m2 * (m1.length() + offset);
        }

        bool n1 = point_line_near_3d(pin, poly[0], poly[3], offset);
        bool n2 = point_line_near_3d(pin, poly[1], poly[2], offset);

        if (n1 || n2)
        {
            out_pin = pin;
            return true;
        }

        // Project 3D quad and pin to 2D for point-in-polygon test
        V3              qx = (poly[1] - poly[0]).normalized();
        V3              qy = cross(normal, qx).normalized();
        std::vector<V2> poly2d(4);
        for (size_t i = 0; i < 4; ++i)
        {
            V3 d      = poly[i] - cen;
            poly2d[i] = { dot(d, qx), dot(d, qy) };
        }
        V3 dp = pin - cen;
        V2 pin2d{ dot(dp, qx), dot(dp, qy) };

        if (is_point_inside_polygon_2d(pin2d, poly2d))
        {
            out_pin = pin;
            return true;
        }

        return false;
    }

    static bool is_cross_region_2d(const V2& p1, const V2& p2, const std::vector<V2>& quad, V2& hit)
    {
        if (quad.size() < 4)
            return false;

        // Check intersection with outer edge quad[0] - quad[1]
        V2 pin;
        if (intersect_segments_2d(p1, p2, quad[0], quad[1], pin))
        {
            hit = pin;
            return true;
        }
        if (is_point_inside_polygon_2d(p1, quad))
        {
            hit = p2;
            return true;
        }
        if (is_point_inside_polygon_2d(p2, quad))
        {
            hit = p2;
            return true;
        }
        return false;
    }

    bool ComputeSafeInset(const std::vector<V3>& input_vertices, const V3& normal, const SafeInsetParams& params, InsetResult& result)
    {
        size_t n = input_vertices.size();
        if (n < 3)
            return false;

        // Setup basis
        Basis3D basis;
        basis.o = input_vertices[0];
        basis.n = normal.normalized();

        // Find first non-degenerate edge for X axis
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
        {
            ps[i] = basis.project(input_vertices[i]);
        }

        // Ensure CCW orientation
        if (polygon_signed_area_2d(ps) < 0.0)
        {
            std::reverse(ps.begin(), ps.end());
        }

        // Bounding box size for ray shooting
        double x_min = std::numeric_limits<double>::infinity();
        double x_max = -std::numeric_limits<double>::infinity();
        double y_min = std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();
        for (const auto& pt : ps)
        {
            x_min = std::min(x_min, pt.x);
            x_max = std::max(x_max, pt.x);
            y_min = std::min(y_min, pt.y);
            y_max = std::max(y_max, pt.y);
        }
        double maxlen = std::max(x_max - x_min, y_max - y_min);
        if (maxlen < 1e-6)
            maxlen = 1.0;

        // Compute bisectors and deg values
        std::vector<V2>     mids(n);
        std::vector<double> degs(n);
        for (size_t i = 0; i < n; ++i)
        {
            size_t prev  = (i + n - 1) % n;
            size_t next  = (i + 1) % n;
            V2     m1    = ps[next] - ps[i];
            V2     m2    = ps[prev] - ps[i];
            mids[i]      = get_mid_2d(m1, m2);
            double theta = InsetMath::angle(m1, m2);
            degs[i]      = std::sin(theta * 0.5);
        }

        // Construct 3D elevated prism points pks
        std::vector<V3> pks(n);
        for (size_t i = 0; i < n; ++i)
        {
            double dmin = (degs[i] < 1e-6) ? maxlen : (maxlen / degs[i]);
            V2     d1   = mids[i] * dmin;
            pks[i]      = V3{ ps[i].x + d1.x, ps[i].y + d1.y, maxlen };
        }

        // Calculate collision limit distance for each vertex
        std::vector<V2> ps2;
        ps2.reserve(n);

        for (size_t i = 0; i < n; ++i)
        {
            size_t next_i = (i + 1) % n;
            size_t prev_i = (i + n - 1) % n;

            V3 p1_3d{ ps[i].x, ps[i].y, 0.0 };
            V3 pk_3d = pks[i];

            double min_dist = std::numeric_limits<double>::infinity();

            // Collision with edge sweep prisms
            for (size_t k = 0; k < n; ++k)
            {
                size_t k2 = (k + 1) % n;
                if (k == i || k2 == i)
                    continue;

                V3 p2_3d{ ps[k].x, ps[k].y, 0.0 };
                V3 p3_3d{ ps[k2].x, ps[k2].y, 0.0 };
                V3 pk2_3d = pks[k];
                V3 pk3_3d = pks[k2];

                std::vector<V3> poly3d = { p2_3d, p3_3d, pk3_3d, pk2_3d };
                V3              s1     = cross(p3_3d - p2_3d, pk2_3d - p2_3d).normalized();

                V3 pin3d;
                if (line_poly_intersect_3d(p1_3d, pk_3d, poly3d, s1, params.threshold, pin3d))
                {
                    V2     pin2d{ pin3d.x, pin3d.y };
                    double d1 = (pin2d - ps[i]).length();
                    min_dist  = std::min(min_dist, d1);
                }
            }

            // 2D Ray intersection with adjacent bisectors
            V2 mid1     = mids[i];
            V2 mid_next = mids[next_i];
            V2 mid_prev = mids[prev_i];

            V2 pin_adj;
            if (intersect_lines_2d(ps[i], ps[i] + mid1 * 1000.0, ps[next_i], ps[next_i] + mid_next * 1000.0, pin_adj))
            {
                // Verify intersection is forward along ray
                if (dot(pin_adj - ps[i], mid1) > 0.0 && dot(pin_adj - ps[next_i], mid_next) > 0.0)
                {
                    double d1 = (pin_adj - ps[i]).length();
                    min_dist  = std::min(min_dist, d1);
                }
            }
            if (intersect_lines_2d(ps[i], ps[i] + mid1 * 1000.0, ps[prev_i], ps[prev_i] + mid_prev * 1000.0, pin_adj))
            {
                if (dot(pin_adj - ps[i], mid1) > 0.0 && dot(pin_adj - ps[prev_i], mid_prev) > 0.0)
                {
                    double d1 = (pin_adj - ps[i]).length();
                    min_dist  = std::min(min_dist, d1);
                }
            }

            double d2      = (degs[i] < 1e-6) ? 0.0 : (params.inset_size / degs[i]);
            double d_final = std::min(min_dist, d2);
            double off2    = std::max(1.0 - params.threshold, 0.0);
            V2     pk_co   = ps[i] + mids[i] * (d_final * off2);
            ps2.push_back(pk_co);
        }

        // Cross region check
        struct AddVert
        {
            size_t insert_idx;
            V2     co;
        };
        std::vector<AddVert> adds;

        for (size_t i = 0; i < ps2.size(); ++i)
        {
            size_t i2 = (i + 1) % ps2.size();
            V2     p3 = ps2[i];
            V2     p4 = ps2[i2];

            for (size_t k = 0; k < ps.size(); ++k)
            {
                if (k == i || k == i2)
                    continue;

                V2 p1_orig = ps[k];
                V2 p2_orig = ps2[k];
                V2 p3b     = ps[i];
                V2 p4b     = ps[i2];

                std::vector<V2> quad = { p3b, p4b, p4, p3 };
                V2              pin_hit;
                if (is_cross_region_2d(p1_orig, p2_orig, quad, pin_hit))
                {
                    V2     arrow   = p2_orig - p1_orig;
                    double arr_len = arrow.length();
                    if (arr_len > 1e-6)
                    {
                        V2 pa = p1_orig + arrow.normalized() * (arr_len + params.threshold);
                        adds.push_back({ i + 1, pa });
                    }
                }
            }
        }

        // Insert additional vertices if needed (in reverse index order)
        std::sort(adds.begin(), adds.end(), [](const AddVert& a, const AddVert& b) { return a.insert_idx > b.insert_idx; });
        for (const auto& item : adds)
        {
            if (item.insert_idx <= ps2.size())
            {
                ps2.insert(ps2.begin() + item.insert_idx, item.co);
            }
        }

        // Unproject 2D vertices back to 3D
        result.all_new_vertices.clear();
        result.all_faces.clear();
        result.is_top_face.clear();

        // 0..n-1: original input vertices (re-unprojected or referenced)
        std::vector<size_t> outer_indices(n);
        for (size_t i = 0; i < n; ++i)
        {
            result.all_new_vertices.push_back(basis.unproject(ps[i]));
            outer_indices[i] = i;
        }

        // n..n+ps2.size()-1: new inner inset vertices
        size_t              inner_start = result.all_new_vertices.size();
        std::vector<size_t> inner_indices;
        for (const auto& p2d : ps2)
        {
            inner_indices.push_back(result.all_new_vertices.size());
            result.all_new_vertices.push_back(basis.unproject(p2d));
        }

        // Optional vertex merging for inner vertices if requested
        if (params.merge && params.merge_dist > 1e-6)
        {
            std::vector<size_t> remap(inner_indices.size());
            for (size_t i = 0; i < inner_indices.size(); ++i)
                remap[i] = inner_indices[i];

            for (size_t i = 0; i < inner_indices.size(); ++i)
            {
                for (size_t j = i + 1; j < inner_indices.size(); ++j)
                {
                    if (dist(result.all_new_vertices[inner_indices[i]], result.all_new_vertices[inner_indices[j]]) < params.merge_dist)
                    {
                        remap[j] = remap[i];
                    }
                }
            }

            // Compact unique vertices
            std::vector<size_t> unique_inner;
            for (size_t i = 0; i < inner_indices.size(); ++i)
            {
                size_t mapped = remap[i];
                if (unique_inner.empty() || unique_inner.back() != mapped)
                {
                    unique_inner.push_back(mapped);
                }
            }
            if (!unique_inner.empty() && unique_inner.front() == unique_inner.back())
            {
                unique_inner.pop_back();
            }
            if (unique_inner.size() >= 3)
            {
                inner_indices = unique_inner;
            }
        }

        // Create top face
        if (inner_indices.size() >= 3)
        {
            result.all_faces.push_back(inner_indices);
            result.is_top_face.push_back(true);
        }

        // Create side skirt faces (fan quads between outer boundary and inner boundary)
        size_t inner_count = ps2.size();
        for (size_t i = 0; i < n; ++i)
        {
            size_t next_i  = (i + 1) % n;
            size_t in_i    = inner_start + (i % inner_count);
            size_t in_next = inner_start + (next_i % inner_count);

            std::vector<size_t> quad = { outer_indices[i], outer_indices[next_i], in_next, in_i };
            result.all_faces.push_back(quad);
            result.is_top_face.push_back(false);
        }

        result.success = true;
        return true;
    }

    LxResult ProcessPolygon(CLxUser_Mesh&              mesh,
                            CLxUser_Polygon&           polygon,
                            CLxUser_Point&             point,
                            const SafeInsetParams&     params,
                            std::vector<LXtPolygonID>& created_polys,
                            std::vector<LXtPolygonID>& top_polys)
    {
        unsigned nvert = 0;
        LxResult rc    = polygon.VertexCount(&nvert);
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
        if (!ComputeSafeInset(input_vertices, normal_v, params, result) || !result.success)
            return LXe_FAILED;

        // Create new vertices in Modo mesh for inner points
        // First n vertices in all_new_vertices correspond to original vertices
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

        // Create polygons using NewProto from prototype
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

            unsigned int rev = 0;
            if (result.is_top_face[f])
            {
                rev = MeshUtil::VertexListIsReverse(mesh, poly_verts, norm);
            }

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

        // Remove the original polygon
        polygon.Select(protoID);
        polygon.Remove();

        return LXe_OK;
    }
}  // namespace SafeInset
