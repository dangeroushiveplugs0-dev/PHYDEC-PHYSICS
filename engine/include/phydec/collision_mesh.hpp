#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>
#include "phydec/collision.hpp"

namespace phydec {

struct Triangle {
    Vec3 a{};
    Vec3 b{};
    Vec3 c{};
};

// Axis-aligned bounds used by the mesh's bounding-volume hierarchy.
struct TriangleAabb {
    Vec3 minimum{};
    Vec3 maximum{};
};

struct TriangleBvhNode {
    TriangleAabb bounds{};
    std::size_t first{0};
    std::size_t count{0};
    int left{-1};
    int right{-1};

    bool is_leaf() const noexcept { return left < 0 && right < 0; }
};

struct TriangleMeshCollider {
    std::vector<Triangle> triangles{};
    // The BVH stores triangle indices, not copies of geometry. Rebuild it after
    // changing triangles; an empty hierarchy automatically uses brute force.
    std::vector<std::size_t> bvh_triangle_indices{};
    std::vector<TriangleBvhNode> bvh_nodes{};
};

struct ClosestTrianglePoint {
    Vec3 position{};
    Vec3 normal{};
    float distance_squared{0.0F};
    bool valid{false};
};

inline Vec3 closest_point_on_triangle(const Vec3& point, const Triangle& triangle) noexcept {
    const Vec3 ab = triangle.b - triangle.a;
    const Vec3 ac = triangle.c - triangle.a;
    const Vec3 ap = point - triangle.a;
    const float d1 = dot(ab, ap);
    const float d2 = dot(ac, ap);
    if (d1 <= 0.0F && d2 <= 0.0F) return triangle.a;

    const Vec3 bp = point - triangle.b;
    const float d3 = dot(ab, bp);
    const float d4 = dot(ac, bp);
    if (d3 >= 0.0F && d4 <= d3) return triangle.b;

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0F && d1 >= 0.0F && d3 <= 0.0F) {
        const float v = d1 / (d1 - d3);
        return triangle.a + ab * v;
    }

    const Vec3 cp = point - triangle.c;
    const float d5 = dot(ab, cp);
    const float d6 = dot(ac, cp);
    if (d6 >= 0.0F && d5 <= d6) return triangle.c;

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0F && d2 >= 0.0F && d6 <= 0.0F) {
        const float w = d2 / (d2 - d6);
        return triangle.a + ac * w;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0F && (d4 - d3) >= 0.0F && (d5 - d6) >= 0.0F) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return triangle.b + (triangle.c - triangle.b) * w;
    }

    const float denominator = va + vb + vc;
    if (std::abs(denominator) <= 1.0e-12F) {
        const auto closest_on_segment = [&point](const Vec3& start, const Vec3& end) {
            const Vec3 edge = end - start;
            const float edge_squared = dot(edge, edge);
            if (edge_squared <= 1.0e-12F) return start;
            const float t = std::clamp(dot(point - start, edge) / edge_squared, 0.0F, 1.0F);
            return start + edge * t;
        };
        const Vec3 p0 = closest_on_segment(triangle.a, triangle.b);
        const Vec3 p1 = closest_on_segment(triangle.b, triangle.c);
        const Vec3 p2 = closest_on_segment(triangle.c, triangle.a);
        const float q0 = dot(point - p0, point - p0);
        const float q1 = dot(point - p1, point - p1);
        const float q2 = dot(point - p2, point - p2);
        return q0 <= q1 && q0 <= q2 ? p0 : (q1 <= q2 ? p1 : p2);
    }

    const float inverse = 1.0F / denominator;
    const float v = vb * inverse;
    const float w = vc * inverse;
    return triangle.a + ab * v + ac * w;
}

inline TriangleAabb triangle_bounds(const Triangle& t) noexcept {
    return {
        {std::min({t.a.x, t.b.x, t.c.x}), std::min({t.a.y, t.b.y, t.c.y}),
         std::min({t.a.z, t.b.z, t.c.z})},
        {std::max({t.a.x, t.b.x, t.c.x}), std::max({t.a.y, t.b.y, t.c.y}),
         std::max({t.a.z, t.b.z, t.c.z})}
    };
}

inline Vec3 triangle_centroid(const Triangle& t) noexcept {
    return (t.a + t.b + t.c) / 3.0F;
}

inline float point_aabb_distance_squared(const Vec3& p, const TriangleAabb& b) noexcept {
    const float dx = p.x < b.minimum.x ? b.minimum.x - p.x :
                     (p.x > b.maximum.x ? p.x - b.maximum.x : 0.0F);
    const float dy = p.y < b.minimum.y ? b.minimum.y - p.y :
                     (p.y > b.maximum.y ? p.y - b.maximum.y : 0.0F);
    const float dz = p.z < b.minimum.z ? b.minimum.z - p.z :
                     (p.z > b.maximum.z ? p.z - b.maximum.z : 0.0F);
    return dx * dx + dy * dy + dz * dz;
}

inline TriangleAabb merge_aabb(const TriangleAabb& a, const TriangleAabb& b) noexcept {
    return {
        {std::min(a.minimum.x, b.minimum.x), std::min(a.minimum.y, b.minimum.y),
         std::min(a.minimum.z, b.minimum.z)},
        {std::max(a.maximum.x, b.maximum.x), std::max(a.maximum.y, b.maximum.y),
         std::max(a.maximum.z, b.maximum.z)}
    };
}

// Build a median-split BVH. Returns false when no finite triangles are available.
inline bool build_triangle_mesh_bvh(TriangleMeshCollider& mesh,
                                    std::size_t leaf_size = 4) {
    mesh.bvh_nodes.clear();
    mesh.bvh_triangle_indices.clear();
    leaf_size = std::max<std::size_t>(leaf_size, 1);

    for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        const Triangle& t = mesh.triangles[i];
        if (is_finite(t.a) && is_finite(t.b) && is_finite(t.c)) {
            mesh.bvh_triangle_indices.push_back(i);
        }
    }
    if (mesh.bvh_triangle_indices.empty()) return false;

    const auto build = [&](auto&& self, std::size_t first, std::size_t count) -> int {
        TriangleAabb bounds = triangle_bounds(mesh.triangles[mesh.bvh_triangle_indices[first]]);
        TriangleAabb centroid_bounds{triangle_centroid(mesh.triangles[mesh.bvh_triangle_indices[first]]),
                                     triangle_centroid(mesh.triangles[mesh.bvh_triangle_indices[first]])};
        for (std::size_t i = first + 1; i < first + count; ++i) {
            const Triangle& t = mesh.triangles[mesh.bvh_triangle_indices[i]];
            bounds = merge_aabb(bounds, triangle_bounds(t));
            const Vec3 c = triangle_centroid(t);
            centroid_bounds = merge_aabb(centroid_bounds, TriangleAabb{c, c});
        }

        const int node_index = static_cast<int>(mesh.bvh_nodes.size());
        mesh.bvh_nodes.push_back(TriangleBvhNode{bounds, first, count, -1, -1});
        if (count <= leaf_size) return node_index;

        const Vec3 extent = centroid_bounds.maximum - centroid_bounds.minimum;
        int axis = extent.y > extent.x ? 1 : 0;
        if ((axis == 0 ? extent.x : extent.y) < extent.z) axis = 2;
        const auto coordinate = [axis](const Vec3& p) {
            return axis == 0 ? p.x : (axis == 1 ? p.y : p.z);
        };
        const std::size_t middle = first + count / 2;
        std::nth_element(mesh.bvh_triangle_indices.begin() + static_cast<std::ptrdiff_t>(first),
                         mesh.bvh_triangle_indices.begin() + static_cast<std::ptrdiff_t>(middle),
                         mesh.bvh_triangle_indices.begin() + static_cast<std::ptrdiff_t>(first + count),
                         [&](std::size_t lhs, std::size_t rhs) {
                             return coordinate(triangle_centroid(mesh.triangles[lhs])) <
                                    coordinate(triangle_centroid(mesh.triangles[rhs]));
                         });
        const int left = self(self, first, middle - first);
        const int right = self(self, middle, first + count - middle);
        mesh.bvh_nodes[static_cast<std::size_t>(node_index)].left = left;
        mesh.bvh_nodes[static_cast<std::size_t>(node_index)].right = right;
        mesh.bvh_nodes[static_cast<std::size_t>(node_index)].count = 0;
        return node_index;
    };
    build(build, 0, mesh.bvh_triangle_indices.size());
    return !mesh.bvh_nodes.empty();
}

inline ClosestTrianglePoint closest_point_on_mesh(const Vec3& point,
                                                   const TriangleMeshCollider& mesh) noexcept {
    ClosestTrianglePoint best;
    best.distance_squared = INFINITY;
    if (!is_finite(point)) return best;

    const auto consider = [&](const Triangle& triangle) {
        if (!is_finite(triangle.a) || !is_finite(triangle.b) || !is_finite(triangle.c)) return;
        const Vec3 closest = closest_point_on_triangle(point, triangle);
        const Vec3 offset = point - closest;
        const float distance_squared = dot(offset, offset);
        if (!std::isfinite(distance_squared) || distance_squared >= best.distance_squared) return;

        const Vec3 face = normalized(Vec3{
            (triangle.b.y - triangle.a.y) * (triangle.c.z - triangle.a.z) -
                (triangle.b.z - triangle.a.z) * (triangle.c.y - triangle.a.y),
            (triangle.b.z - triangle.a.z) * (triangle.c.x - triangle.a.x) -
                (triangle.b.x - triangle.a.x) * (triangle.c.z - triangle.a.z),
            (triangle.b.x - triangle.a.x) * (triangle.c.y - triangle.a.y) -
                (triangle.b.y - triangle.a.y) * (triangle.c.x - triangle.a.x)
        });
        best.position = closest;
        best.normal = length(offset) > 1.0e-7F ? normalized(offset) : face;
        best.distance_squared = distance_squared;
        best.valid = is_finite(best.normal) && length(best.normal) > 1.0e-7F;
    };

    if (mesh.bvh_nodes.empty() || mesh.bvh_triangle_indices.empty()) {
        for (const Triangle& triangle : mesh.triangles) consider(triangle);
        return best;
    }

    const auto visit = [&](auto&& self, int node_index) -> void {
        const TriangleBvhNode& node = mesh.bvh_nodes[static_cast<std::size_t>(node_index)];
        if (point_aabb_distance_squared(point, node.bounds) > best.distance_squared) return;
        if (node.is_leaf()) {
            for (std::size_t i = node.first; i < node.first + node.count; ++i) {
                consider(mesh.triangles[mesh.bvh_triangle_indices[i]]);
            }
            return;
        }
        const float left_distance = point_aabb_distance_squared(
            point, mesh.bvh_nodes[static_cast<std::size_t>(node.left)].bounds);
        const float right_distance = point_aabb_distance_squared(
            point, mesh.bvh_nodes[static_cast<std::size_t>(node.right)].bounds);
        if (left_distance <= right_distance) {
            self(self, node.left);
            self(self, node.right);
        } else {
            self(self, node.right);
            self(self, node.left);
        }
    };
    visit(visit, 0);
    return best;
}

// Surface contact only: watertight solid containment is a separate feature.
inline Contact resolve_mesh_contact(Vec3& point, float particle_radius,
                                    const TriangleMeshCollider& mesh) noexcept {
    Contact contact;
    if (!is_finite(point) || !std::isfinite(particle_radius) || particle_radius < 0.0F) {
        return contact;
    }

    const ClosestTrianglePoint closest = closest_point_on_mesh(point, mesh);
    if (!closest.valid) return contact;

    const Vec3 offset = point - closest.position;
    const float distance = std::sqrt(std::max(closest.distance_squared, 0.0F));
    if (distance >= particle_radius) return contact;

    contact.normal = distance > 1.0e-7F ? offset / distance : closest.normal;
    if (length(contact.normal) <= 1.0e-7F || !is_finite(contact.normal)) return Contact{};
    contact.penetration_depth = particle_radius - distance;
    point += contact.normal * contact.penetration_depth;
    contact.position = closest.position;
    contact.touching = true;
    return contact;
}

} // namespace phydec
