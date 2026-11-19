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

#pragma once

#include "common.hpp"
#include "inset_math.hpp"

namespace PaddingInset
{
    struct PaddingInsetParams
    {
        double inset_size = 0.2;
        double align = 0.0;     // ray alignment in radians (or degrees from UI)
        bool   flat = false;    // flat concave mode
        bool   connect = true;  // connect side skirts
    };

    struct InsetResult
    {
        bool success = false;
        std::vector<InsetMath::V3> all_new_vertices;
        std::vector<std::vector<size_t>> all_faces;
        std::vector<bool> is_top_face;
    };

    // Core algorithmic function operating on 3D polygon vertices and normal
    bool ComputePaddingInset(
        const std::vector<InsetMath::V3>& input_vertices,
        const InsetMath::V3& normal,
        const PaddingInsetParams& params,
        InsetResult& result
    );

    // Modo mesh integration: processes a selected polygon in the edit mesh
    LxResult ProcessPolygon(
        CLxUser_Mesh& mesh,
        CLxUser_Polygon& polygon,
        CLxUser_Point& point,
        const PaddingInsetParams& params,
        std::vector<LXtPolygonID>& created_polys,
        std::vector<LXtPolygonID>& top_polys
    );
}
