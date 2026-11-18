# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

#pragma once

#include "common.hpp"
#include "inset_math.hpp"

namespace SafeInset
{
    struct SafeInsetParams
    {
        double inset_size = 0.15;
        double threshold = 0.01;
        bool   merge = false;
        double merge_dist = 0.04;
    };

    struct InsetResult
    {
        bool success = false;
        std::vector<InsetMath::V3> top_vertices;
        std::vector<std::vector<size_t>> top_faces; // indices into top_vertices
        std::vector<std::vector<size_t>> side_faces; // indices referencing (outer, inner)
        std::vector<InsetMath::V3> all_new_vertices;
        std::vector<std::vector<size_t>> all_faces; // polygons to create
        std::vector<bool> is_top_face; // true for inset cap, false for side skirt
    };

    // Core algorithmic function operating on 3D polygon vertices and normal
    bool ComputeSafeInset(
        const std::vector<InsetMath::V3>& input_vertices,
        const InsetMath::V3& normal,
        const SafeInsetParams& params,
        InsetResult& result
    );

    // Modo mesh integration: processes a selected polygon in the edit mesh
    LxResult ProcessPolygon(
        CLxUser_Mesh& mesh,
        CLxUser_Polygon& polygon,
        CLxUser_Point& point,
        const SafeInsetParams& params,
        std::vector<LXtPolygonID>& created_polys,
        std::vector<LXtPolygonID>& top_polys
    );
}
