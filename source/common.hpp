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

#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxvmath.h>

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <map>

enum InsetMode : int
{
    MODE_SAFE = 0,
    MODE_PADDING = 1
};

#define SRVNAME_COMMAND "poly.smartInset"
#define SRVNAME_TOOL    "tool.smartInset"

#define ATTRs_MODE       "mode"
#define ATTRs_SIZE       "size"
#define ATTRs_THRESHOLD  "threshold"
#define ATTRs_MERGE      "merge"
#define ATTRs_MERGEDIST  "mergeDist"
#define ATTRs_FLAT       "flat"
#define ATTRs_ALIGN      "align"
#define ATTRs_CONNECT    "connect"

#define ATTRi_MODE       0
#define ATTRi_SIZE       1
#define ATTRi_THRESHOLD  2
#define ATTRi_MERGE      3
#define ATTRi_MERGEDIST  4
#define ATTRi_FLAT       5
#define ATTRi_ALIGN      6
#define ATTRi_CONNECT    7
