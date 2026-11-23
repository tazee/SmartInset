#include <iostream>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include "inset_math.hpp"
#include "safe_inset.hpp"
#include "padding_inset.hpp"

using namespace InsetMath;

namespace
{
    void require(bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    uint16_t read_be16(const std::vector<uint8_t>& data, size_t offset)
    {
        require(offset + 2 <= data.size(), "Unexpected end of LXO file");
        return static_cast<uint16_t>((static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1]);
    }

    uint32_t read_be32(const std::vector<uint8_t>& data, size_t offset)
    {
        require(offset + 4 <= data.size(), "Unexpected end of LXO file");
        return (static_cast<uint32_t>(data[offset]) << 24) |
               (static_cast<uint32_t>(data[offset + 1]) << 16) |
               (static_cast<uint32_t>(data[offset + 2]) << 8) |
               static_cast<uint32_t>(data[offset + 3]);
    }

    struct LxoMesh
    {
        std::vector<V3> points;
        std::vector<std::vector<size_t>> faces;
    };

    LxoMesh read_lxo_mesh(const std::string& path)
    {
        std::ifstream file(path, std::ios::binary);
        require(static_cast<bool>(file), "Could not open LXO regression fixture");
        std::vector<uint8_t> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        require(data.size() >= 12, "Invalid LXO regression fixture");
        LxoMesh mesh;
        size_t offset = 12;

        while (offset + 8 <= data.size())
        {
            const std::string tag(data.begin() + offset, data.begin() + offset + 4);
            const uint32_t chunk_size = read_be32(data, offset + 4);
            const size_t start = offset + 8;
            const size_t end = start + chunk_size;
            require(end <= data.size(), "Invalid LXO chunk size");

            if (tag == "PNTS")
            {
                require(chunk_size % 12 == 0, "Invalid LXO point chunk");
                for (size_t p = start; p < end; p += 12)
                {
                    const uint32_t words[] = {
                        read_be32(data, p),
                        read_be32(data, p + 4),
                        read_be32(data, p + 8)
                    };
                    float xyz[3];
                    for (size_t axis = 0; axis < 3; ++axis)
                        std::memcpy(&xyz[axis], &words[axis], sizeof(words[axis]));
                    mesh.points.emplace_back(xyz[0], xyz[1], xyz[2]);
                }
            }
            else if (tag == "POLS" && chunk_size >= 4 &&
                     std::string(data.begin() + start, data.begin() + start + 4) == "FACE")
            {
                size_t p = start + 4;
                while (p < end)
                {
                    const uint16_t count = read_be16(data, p);
                    p += 2;
                    std::vector<size_t> current_face;
                    current_face.reserve(count);
                    for (uint16_t i = 0; i < count; ++i)
                    {
                        const uint16_t short_index = read_be16(data, p);
                        if (short_index >= 0xff00)
                        {
                            current_face.push_back(read_be32(data, p) & 0x00ffffff);
                            p += 4;
                        }
                        else
                        {
                            current_face.push_back(short_index);
                            p += 2;
                        }
                    }
                    mesh.faces.push_back(std::move(current_face));
                }
            }

            offset = end + (chunk_size & 1);
        }

        require(!mesh.points.empty(), "LXO fixture has no points");
        require(!mesh.faces.empty(), "LXO fixture has no faces");
        return mesh;
    }

    bool same_face_cycle(const std::vector<size_t>& a, const std::vector<size_t>& b)
    {
        if (a.empty() || a.size() != b.size())
            return false;

        for (size_t start = 0; start < b.size(); ++start)
        {
            if (a[0] != b[start])
                continue;

            bool forward = true;
            bool reverse = true;
            for (size_t i = 1; i < a.size(); ++i)
            {
                forward &= a[i] == b[(start + i) % b.size()];
                reverse &= a[i] == b[(start + b.size() - i) % b.size()];
            }
            if (forward || reverse)
                return true;
        }

        return false;
    }
}

void test_basic_math()
{
    std::cout << "[Test] Basic vector math..." << std::endl;
    V2 a{1.0, 0.0};
    V2 b{0.0, 1.0};
    assert(std::abs(dot(a, b)) < 1e-9);
    assert(std::abs(cross(a, b) - 1.0) < 1e-9);
    assert(std::abs(angle(a, b) - PI * 0.5) < 1e-9);

    V2 p1{0, 0}, p2{2, 2}, p3{0, 2}, p4{2, 0};
    V2 inter;
    bool hit = intersect_lines_2d(p1, p2, p3, p4, inter);
    assert(hit);
    assert(std::abs(inter.x - 1.0) < 1e-6 && std::abs(inter.y - 1.0) < 1e-6);

    std::vector<V2> square = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
    assert(polygon_signed_area_2d(square) > 0.0);
    assert(is_point_inside_polygon_2d({1, 1}, square));
    assert(!is_point_inside_polygon_2d({3, 3}, square));

    std::cout << "  -> Passed!" << std::endl;
}

void test_safe_inset_quad()
{
    std::cout << "[Test] SafeInset on 1x1 square..." << std::endl;
    std::vector<V3> square = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0},
        {0.0, 1.0, 0.0}
    };
    V3 normal{0.0, 0.0, 1.0};

    SafeInset::SafeInsetParams params;
    params.inset_size = 0.2;
    params.threshold = 0.01;

    SafeInset::InsetResult result;
    bool ok = SafeInset::ComputeSafeInset(square, normal, params, result);
    assert(ok);
    assert(result.success);
    assert(!result.all_faces.empty());

    // Check top face has 4 vertices
    bool found_top = false;
    for (size_t i = 0; i < result.all_faces.size(); ++i)
    {
        if (result.is_top_face[i])
        {
            found_top = true;
            assert(result.all_faces[i].size() == 4);
        }
    }
    assert(found_top);
    std::cout << "  -> Passed! Generated " << result.all_faces.size() << " faces." << std::endl;
}

void test_safe_inset_concave()
{
    std::cout << "[Test] SafeInset on L-shaped concave polygon..." << std::endl;
    // 6-sided L-shape
    std::vector<V3> l_shape = {
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {2.0, 1.0, 0.0},
        {1.0, 1.0, 0.0},
        {1.0, 2.0, 0.0},
        {0.0, 2.0, 0.0}
    };
    V3 normal{0.0, 0.0, 1.0};

    SafeInset::SafeInsetParams params;
    params.inset_size = 0.15;
    params.threshold = 0.01;

    SafeInset::InsetResult result;
    bool ok = SafeInset::ComputeSafeInset(l_shape, normal, params, result);
    assert(ok);
    assert(result.success);
    assert(!result.all_faces.empty());

    std::cout << "  -> Passed! Generated " << result.all_faces.size() << " faces." << std::endl;
}

void test_padding_inset_quad()
{
    std::cout << "[Test] PaddingInset on 1x1 square..." << std::endl;
    std::vector<V3> square = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0},
        {0.0, 1.0, 0.0}
    };
    V3 normal{0.0, 0.0, 1.0};

    PaddingInset::PaddingInsetParams params;
    params.inset_size = 0.2;
    params.connect = true;

    PaddingInset::InsetResult result;
    bool ok = PaddingInset::ComputePaddingInset(square, normal, params, result);
    assert(ok);
    assert(result.success);
    assert(!result.all_faces.empty());

    bool found_top = false;
    for (size_t i = 0; i < result.all_faces.size(); ++i)
    {
        if (result.is_top_face[i])
        {
            found_top = true;
            assert(result.all_faces[i].size() == 4);
        }
    }
    assert(found_top);
    std::cout << "  -> Passed! Generated " << result.all_faces.size() << " faces." << std::endl;
}

void test_padding_inset_concave()
{
    std::cout << "[Test] PaddingInset on L-shaped concave polygon..." << std::endl;
    std::vector<V3> l_shape = {
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {2.0, 1.0, 0.0},
        {1.0, 1.0, 0.0},
        {1.0, 2.0, 0.0},
        {0.0, 2.0, 0.0}
    };
    V3 normal{0.0, 0.0, 1.0};

    PaddingInset::PaddingInsetParams params;
    params.inset_size = 0.15;
    params.connect = true;

    PaddingInset::InsetResult result;
    bool ok = PaddingInset::ComputePaddingInset(l_shape, normal, params, result);
    assert(ok);
    assert(result.success);
    assert(!result.all_faces.empty());

    std::cout << "  -> Passed! Generated " << result.all_faces.size() << " faces." << std::endl;
}

void test_padding_inset_lxo_regression()
{
    const LxoMesh source = read_lxo_mesh(std::string(INSET_TEST_SOURCE_DIR) + "/padding_test.lxo");
    const LxoMesh target = read_lxo_mesh(std::string(INSET_TEST_SOURCE_DIR) + "/padding_result.lxo");
    require(source.faces.size() == 1, "Expected one selected face in padding_test.lxo");
    const auto& face = source.faces[0];

    std::vector<V3> polygon;
    polygon.reserve(face.size());
    for (size_t index : face)
    {
        require(index < source.points.size(), "Invalid point index in padding_test.lxo");
        polygon.push_back(source.points[index]);
    }

    V3 normal{};
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        const V3& a = polygon[i];
        const V3& b = polygon[(i + 1) % polygon.size()];
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
    }

    PaddingInset::PaddingInsetParams params;
    params.inset_size = 0.3;
    params.connect = true;

    PaddingInset::InsetResult result;
    require(ComputePaddingInset(polygon, normal.normalized(), params, result), "Padding inset failed on the regression face");
    require(result.success, "Padding inset did not mark the regression result successful");
    require(result.all_new_vertices.size() == 122, "Unexpected vertex count for 300mm padding inset");
    require(result.all_faces.size() == 42, "Unexpected face count for 300mm padding inset");
    require(target.points.size() == result.all_new_vertices.size(), "Target scene vertex count differs");
    require(target.faces.size() == result.all_faces.size(), "Target scene face count differs");

    std::map<size_t, size_t> face_sizes;
    size_t top_faces = 0;
    for (size_t i = 0; i < result.all_faces.size(); ++i)
    {
        ++face_sizes[result.all_faces[i].size()];
        top_faces += result.is_top_face[i] ? 1 : 0;
    }
    require(top_faces == 1, "Expected one inset cap");
    require(result.all_faces[0].size() == 41, "Unexpected inset cap vertex count");
    require(face_sizes[4] == 37, "Unexpected number of quad skirt faces");
    require(face_sizes[12] == 3, "Unexpected number of 12-vertex skirt faces");
    require(face_sizes[20] == 1, "Unexpected number of 20-vertex skirt faces");

    std::vector<size_t> result_to_target(result.all_new_vertices.size());
    std::vector<bool> target_point_used(target.points.size(), false);
    for (size_t i = 0; i < result.all_new_vertices.size(); ++i)
    {
        size_t best = target.points.size();
        double best_distance = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < target.points.size(); ++j)
        {
            if (target_point_used[j])
                continue;
            const double distance = dist(result.all_new_vertices[i], target.points[j]);
            if (distance < best_distance)
            {
                best = j;
                best_distance = distance;
            }
        }
        require(best < target.points.size(), "No corresponding target vertex found");
        require(best_distance < 1e-5, "Inset vertex differs from target scene");
        result_to_target[i] = best;
        target_point_used[best] = true;
    }

    std::vector<bool> target_face_used(target.faces.size(), false);
    size_t unmatched_faces = 0;
    for (const auto& result_face : result.all_faces)
    {
        std::vector<size_t> mapped_face;
        mapped_face.reserve(result_face.size());
        for (size_t index : result_face)
            mapped_face.push_back(result_to_target[index]);

        bool matched = false;
        for (size_t j = 0; j < target.faces.size(); ++j)
        {
            if (!target_face_used[j] && same_face_cycle(mapped_face, target.faces[j]))
            {
                target_face_used[j] = true;
                matched = true;
                break;
            }
        }
        if (!matched)
            ++unmatched_faces;
    }
    require(unmatched_faces == 0, "Inset face topology differs from target scene");

    std::cout << "  -> Passed! Matches the 300mm padding result scene." << std::endl;
}

void test_safe_inset_lxo_regression()
{
    std::cout << "[Test] SafeInset on safe_test.lxo..." << std::endl;
    const LxoMesh source = read_lxo_mesh(std::string(INSET_TEST_SOURCE_DIR) + "/safe_test.lxo");
    const LxoMesh target = read_lxo_mesh(std::string(INSET_TEST_SOURCE_DIR) + "/safe_result.lxo");
    require(source.faces.size() == 1, "Expected one selected face in safe_test.lxo");
    const auto& face = source.faces[0];

    std::vector<V3> polygon;
    polygon.reserve(face.size());
    for (size_t index : face)
    {
        require(index < source.points.size(), "Invalid point index in safe_test.lxo");
        polygon.push_back(source.points[index]);
    }

    V3 normal{};
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        const V3& a = polygon[i];
        const V3& b = polygon[(i + 1) % polygon.size()];
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
    }

    SafeInset::SafeInsetParams params;
    params.inset_size = 0.5;
    params.threshold = 0.01;

    SafeInset::InsetResult result;
    require(SafeInset::ComputeSafeInset(polygon, normal.normalized(), params, result), "Safe inset failed on regression face");
    require(result.success, "Safe inset did not mark the regression result successful");
    require(result.all_new_vertices.size() == target.points.size(), "Unexpected vertex count");
    require(result.all_faces.size() == target.faces.size(), "Unexpected face count");

    // Match each result vertex to a target vertex
    std::vector<size_t> result_to_target(result.all_new_vertices.size());
    std::vector<bool> target_point_used(target.points.size(), false);
    for (size_t i = 0; i < result.all_new_vertices.size(); ++i)
    {
        size_t best = target.points.size();
        double best_distance = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < target.points.size(); ++j)
        {
            if (target_point_used[j])
                continue;
            const double distance = dist(result.all_new_vertices[i], target.points[j]);
            if (distance < best_distance)
            {
                best = j;
                best_distance = distance;
            }
        }
        require(best < target.points.size(), "No corresponding target vertex found");
        require(best_distance < 1e-4, "Inset vertex differs from target scene");
        result_to_target[i] = best;
        target_point_used[best] = true;
    }

    // Match each result face to a target face
    std::vector<bool> target_face_used(target.faces.size(), false);
    size_t unmatched_faces = 0;
    for (const auto& result_face : result.all_faces)
    {
        std::vector<size_t> mapped_face;
        mapped_face.reserve(result_face.size());
        for (size_t index : result_face)
            mapped_face.push_back(result_to_target[index]);

        bool matched = false;
        for (size_t j = 0; j < target.faces.size(); ++j)
        {
            if (!target_face_used[j] && same_face_cycle(mapped_face, target.faces[j]))
            {
                target_face_used[j] = true;
                matched = true;
                break;
            }
        }
        if (!matched)
            ++unmatched_faces;
    }
    require(unmatched_faces == 0, "Inset face topology differs from target scene");

    size_t top_faces = 0;
    for (size_t i = 0; i < result.all_faces.size(); ++i)
    {
        if (result.is_top_face[i])
            ++top_faces;
    }
    require(top_faces == 1, "Expected exactly one top face");

    std::cout << "  -> Passed! Matches the safe_result.lxo scene." << std::endl;
}

int main()
{
    std::cout << "=== Running Inset Algorithm Unit Tests ===" << std::endl;
    test_basic_math();
    test_safe_inset_quad();
    test_safe_inset_concave();
    test_padding_inset_quad();
    test_padding_inset_concave();
    test_padding_inset_lxo_regression();
    test_safe_inset_lxo_regression();
    std::cout << "=== ALL TESTS PASSED SUCCESSFULLY! ===" << std::endl;
    return 0;
}
