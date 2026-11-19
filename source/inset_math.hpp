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

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

namespace InsetMath
{
    static constexpr double EPS = 1.0e-9;
    static constexpr double PI = 3.14159265358979323846;

    struct V2
    {
        double x = 0.0;
        double y = 0.0;

        V2() = default;
        V2(double x_, double y_) : x(x_), y(y_) {}

        V2 operator+(const V2& b) const { return {x + b.x, y + b.y}; }
        V2 operator-(const V2& b) const { return {x - b.x, y - b.y}; }
        V2 operator*(double s) const { return {x * s, y * s}; }
        V2 operator/(double s) const { return {x / s, y / s}; }

        double length_sq() const { return x * x + y * y; }
        double length() const { return std::sqrt(length_sq()); }

        V2 normalized() const
        {
            double l = length();
            return (l < EPS) ? V2{0.0, 0.0} : V2{x / l, y / l};
        }
    };

    inline double dot(const V2& a, const V2& b) { return a.x * b.x + a.y * b.y; }
    inline double cross(const V2& a, const V2& b) { return a.x * b.y - a.y * b.x; }
    inline double dist(const V2& a, const V2& b) { return (a - b).length(); }

    inline double angle(const V2& a, const V2& b)
    {
        double la = a.length();
        double lb = b.length();
        if (la < EPS || lb < EPS)
            return 0.0;
        double d = dot(a, b) / (la * lb);
        d = std::max(-1.0, std::min(1.0, d));
        return std::acos(d);
    }

    struct V3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        V3() = default;
        V3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

        V3 operator+(const V3& b) const { return {x + b.x, y + b.y, z + b.z}; }
        V3 operator-(const V3& b) const { return {x - b.x, y - b.y, z - b.z}; }
        V3 operator*(double s) const { return {x * s, y * s, z * s}; }
        V3 operator/(double s) const { return {x / s, y / s, z / s}; }

        double length_sq() const { return x * x + y * y + z * z; }
        double length() const { return std::sqrt(length_sq()); }

        V3 normalized() const
        {
            double l = length();
            return (l < EPS) ? V3{0.0, 0.0, 0.0} : V3{x / l, y / l, z / l};
        }
    };

    inline double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline V3 cross(const V3& a, const V3& b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }
    inline double dist(const V3& a, const V3& b) { return (a - b).length(); }

    // 3D coordinate frame for face projection
    struct Basis3D
    {
        V3 o; // origin
        V3 x; // local X axis
        V3 y; // local Y axis
        V3 n; // normal Z axis

        V2 project(const V3& p) const
        {
            V3 d = p - o;
            return {dot(d, x), dot(d, y)};
        }

        V3 unproject(const V2& p) const
        {
            return o + x * p.x + y * p.y;
        }

        V3 unproject3(const V3& p) const
        {
            return o + x * p.x + y * p.y + n * p.z;
        }
    };

    // 2D line-line intersection (infinite lines)
    inline bool intersect_lines_2d(const V2& p1, const V2& p2, const V2& p3, const V2& p4, V2& out)
    {
        double denom = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
        if (std::abs(denom) < 1e-12)
            return false;

        double t1 = p1.x * p2.y - p1.y * p2.x;
        double t2 = p3.x * p4.y - p3.y * p4.x;
        out.x = (t1 * (p3.x - p4.x) - (p1.x - p2.x) * t2) / denom;
        out.y = (t1 * (p3.y - p4.y) - (p1.y - p2.y) * t2) / denom;
        return true;
    }

    // 2D ray-segment intersection (ray from p1 along dir to ray_len, segment p3-p4)
    inline bool intersect_ray_segment_2d(const V2& p1, const V2& dir, double ray_len, const V2& p3, const V2& p4, V2& out)
    {
        V2 p2 = p1 + dir * ray_len;
        V2 r = p2 - p1;
        V2 s = p4 - p3;
        double den = cross(r, s);
        if (std::abs(den) < 1e-12)
            return false;

        V2 q = p3 - p1;
        double t = cross(q, s) / den;
        double u = cross(q, r) / den;

        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0)
        {
            out = p1 + r * t;
            return true;
        }
        return false;
    }

    // 2D finite segment-segment intersection
    inline bool intersect_segments_2d(const V2& p1, const V2& p2, const V2& p3, const V2& p4, V2& out, double eps = 1e-8)
    {
        V2 r = p2 - p1;
        V2 s = p4 - p3;
        double den = cross(r, s);
        if (std::abs(den) < 1e-12)
            return false;

        V2 q = p3 - p1;
        double t = cross(q, s) / den;
        double u = cross(q, r) / den;

        if (t >= -eps && t <= 1.0 + eps && u >= -eps && u <= 1.0 + eps)
        {
            out = p1 + r * t;
            return true;
        }
        return false;
    }

    // 3D line-plane intersection
    inline bool line_plane_intersection(const V3& p1, const V3& p2, const V3& p0, const V3& normal, V3& out)
    {
        double den = dot(normal, p2 - p1);
        if (std::abs(den) < 1e-6)
            return false;
        double num = dot(normal, p0 - p1);
        double t = num / den;
        if (t < 0.0 || t > 1.0)
            return false;
        out = p1 + (p2 - p1) * t;
        return true;
    }

    // Distance from 2D point to line segment
    inline double dist_point_to_segment_2d(const V2& p, const V2& a, const V2& b)
    {
        V2 v = b - a;
        V2 w = p - a;
        double c1 = dot(w, v);
        if (c1 <= 0.0)
            return (p - a).length();
        double c2 = dot(v, v);
        if (c2 <= c1)
            return (p - b).length();
        double ratio = c1 / c2;
        V2 proj = a + v * ratio;
        return (p - proj).length();
    }

    // Winding test for point inside 2D polygon (from safe_inset.py is_inside)
    inline double is_left(const V2& p0, const V2& p1, const V2& p2)
    {
        return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
    }

    inline bool is_point_inside_polygon_2d(const V2& point, const std::vector<V2>& poly)
    {
        int wn = 0;
        size_t n = poly.size();
        for (size_t i = 0; i < n; ++i)
        {
            size_t next = (i + 1) % n;
            if (poly[i].y <= point.y)
            {
                if (poly[next].y > point.y)
                {
                    if (is_left(poly[i], poly[next], point) > 0.0)
                        wn++;
                }
            }
            else
            {
                if (poly[next].y <= point.y)
                {
                    if (is_left(poly[i], poly[next], point) < 0.0)
                        wn--;
                }
            }
        }
        return wn != 0;
    }

    // Compute polygon signed area (positive if CCW)
    inline double polygon_signed_area_2d(const std::vector<V2>& poly)
    {
        double area = 0.0;
        size_t n = poly.size();
        for (size_t i = 0; i < n; ++i)
        {
            size_t next = (i + 1) % n;
            area += (poly[i].x * poly[next].y - poly[next].x * poly[i].y);
        }
        return area * 0.5;
    }

    // Inward turn of an edge vector in 2D (assuming CCW winding)
    inline V2 turn_inward_2d(const V2& edge_dir)
    {
        // Rotating (dx, dy) by 90 degrees CCW gives (-dy, dx)
        V2 n = edge_dir.normalized();
        return V2{-n.y, n.x};
    }
}
