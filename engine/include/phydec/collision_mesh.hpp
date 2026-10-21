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
    // Opt in only for closed, non-self-intersecting meshes. Open surfaces remain
    // surface-only colliders and do not trigger containment projection.
    bool closed_mesh_solid{false};
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

// Refit an existing BVH after vertex positions change while triangle count/order stay
// unchanged. This avoids the cost of rebuilding the split structure each frame.
// Returns false if the hierarchy is absent or geometry is invalid; on failure the
// hierarchy is cleared so queries safely use the brute-force reference path.
inline bool refit_triangle_mesh_bvh(TriangleMeshCollider& mesh) {
    if (mesh.bvh_nodes.empty() || mesh.bvh_triangle_indices.empty()) return false;

    // Refit is valid only while the set of finite triangles is unchanged.
    // The builder intentionally omits non-finite triangles. If one of those
    // triangles becomes finite, it must be inserted by a full rebuild; otherwise
    // the refitted hierarchy would silently miss geometry.
    std::size_t finite_triangle_count = 0;
    for (const Triangle& triangle : mesh.triangles) {
        if (is_finite(triangle.a) && is_finite(triangle.b) && is_finite(triangle.c)) {
            ++finite_triangle_count;
        }
    }
    if (finite_triangle_count != mesh.bvh_triangle_indices.size()) {
        mesh.bvh_nodes.clear();
        mesh.bvh_triangle_indices.clear();
        return false;
    }

    for (const std::size_t index : mesh.bvh_triangle_indices) {
        if (index >= mesh.triangles.size()) {
            mesh.bvh_nodes.clear();
            mesh.bvh_triangle_indices.clear();
            return false;
        }
        const Triangle& triangle = mesh.triangles[index];
        if (!is_finite(triangle.a) || !is_finite(triangle.b) || !is_finite(triangle.c)) {
            mesh.bvh_nodes.clear();
            mesh.bvh_triangle_indices.clear();
            return false;
        }
    }

    // Builder emits parents before children, so reverse order visits children first.
    for (std::size_t i = mesh.bvh_nodes.size(); i-- > 0;) {
        TriangleBvhNode& node = mesh.bvh_nodes[i];
        if (node.is_leaf()) {
            bool initialized = false;
            TriangleAabb bounds{};
            for (std::size_t j = node.first; j < node.first + node.count; ++j) {
                const std::size_t triangle_index = mesh.bvh_triangle_indices[j];
                const TriangleAabb triangle_box = triangle_bounds(mesh.triangles[triangle_index]);
                bounds = initialized ? merge_aabb(bounds, triangle_box) : triangle_box;
                initialized = true;
            }
            if (!initialized) {
                mesh.bvh_nodes.clear();
                mesh.bvh_triangle_indices.clear();
                return false;
            }
            node.bounds = bounds;
        } else {
            if (node.left < 0 || node.right < 0 ||
                static_cast<std::size_t>(node.left) >= mesh.bvh_nodes.size() ||
                static_cast<std::size_t>(node.right) >= mesh.bvh_nodes.size()) {
                mesh.bvh_nodes.clear();
                mesh.bvh_triangle_indices.clear();
                return false;
            }
            node.bounds = merge_aabb(
                mesh.bvh_nodes[static_cast<std::size_t>(node.left)].bounds,
                mesh.bvh_nodes[static_cast<std::size_t>(node.right)].bounds);
        }
    }
    return true;
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

// Continuous sphere-vs-mesh surface query. Conservative advancement follows the
// segment from start to end and advances by the current separation divided by
// travel distance, so it can detect crossing a thin triangle between frames.
struct SweptMeshContact {
    Vec3 position{}; // sphere center at time of impact, offset by radius
    Vec3 normal{};
    float time_of_impact{1.0F};
    bool hit{false};
};

inline SweptMeshContact sweep_sphere_against_mesh(const Vec3& start, const Vec3& end,
                                                  float radius,
                                                  const TriangleMeshCollider& mesh,
                                                  unsigned max_iterations = 32) noexcept {
    SweptMeshContact result;
    if (!is_finite(start) || !is_finite(end) || !std::isfinite(radius) || radius < 0.0F) {
        return result;
    }

    const Vec3 travel = end - start;
    const float travel_length = length(travel);
    const ClosestTrianglePoint initial = closest_point_on_mesh(start, mesh);
    if (!initial.valid) return result;

    const float radius_squared = radius * radius;
    if (initial.distance_squared <= radius_squared) {
        const Vec3 offset = start - initial.position;
        result.normal = length(offset) > 1.0e-7F ? normalized(offset) : initial.normal;
        result.position = start + result.normal *
            std::max(radius - std::sqrt(std::max(initial.distance_squared, 0.0F)), 0.0F);
        result.time_of_impact = 0.0F;
        result.hit = length(result.normal) > 1.0e-7F && is_finite(result.normal);
        return result;
    }
    if (travel_length <= 1.0e-7F || max_iterations == 0) return result;

    float t = 0.0F;
    for (unsigned iteration = 0; iteration < max_iterations; ++iteration) {
        const Vec3 center = start + travel * t;
        const ClosestTrianglePoint closest = closest_point_on_mesh(center, mesh);
        if (!closest.valid) return result;

        const float distance = std::sqrt(std::max(closest.distance_squared, 0.0F));
        if (distance <= radius + 1.0e-5F) {
            const Vec3 offset = center - closest.position;
            Vec3 normal = distance > 1.0e-7F ? normalized(offset) : closest.normal;
            if (length(normal) <= 1.0e-7F || !is_finite(normal)) return result;
            result.normal = normal;
            result.position = closest.position + normal * radius;
            result.time_of_impact = t;
            result.hit = true;
            return result;
        }

        // The distance to a fixed surface is 1-Lipschitz: this step cannot
        // advance farther than the remaining gap along the swept segment.
        const float step = (distance - radius) / travel_length;
        if (!(step > 1.0e-7F) || !std::isfinite(step)) return result;
        t += step;
        if (t > 1.0F) return result;
    }
    return result;
}

// Continuous sphere query against a triangle mesh undergoing rigid translation.
// Triangle coordinates are expressed in the mesh's untranslated reference frame.
// This reuses the mesh BVH by sweeping in relative motion: sphere motion minus
// collider translation. It does not support rotation or per-vertex deformation;
// those require a swept/deforming BVH and a dedicated CCD strategy.
inline SweptMeshContact sweep_sphere_against_translating_mesh(
    const Vec3& sphere_start, const Vec3& sphere_end, float radius,
    const Vec3& mesh_translation_start, const Vec3& mesh_translation_end,
    const TriangleMeshCollider& mesh, unsigned max_iterations = 32) noexcept {
    SweptMeshContact result;
    if (!is_finite(sphere_start) || !is_finite(sphere_end) ||
        !is_finite(mesh_translation_start) || !is_finite(mesh_translation_end)) {
        return result;
    }

    const Vec3 relative_start = sphere_start - mesh_translation_start;
    const Vec3 relative_end = sphere_end - mesh_translation_end;
    result = sweep_sphere_against_mesh(relative_start, relative_end, radius, mesh, max_iterations);
    if (!result.hit) return result;

    const float t = std::clamp(result.time_of_impact, 0.0F, 1.0F);
    const Vec3 mesh_translation = mesh_translation_start +
        (mesh_translation_end - mesh_translation_start) * t;
    result.position += mesh_translation;
    return result;
}

// Continuous sphere query against linearly deforming triangles.
// Previous/current meshes must have identical triangle count and ordering.
// Current-mesh BVH node bounds are expanded by the maximum vertex displacement,
// conservatively retaining candidates that moved into the sphere's swept AABB.
inline SweptMeshContact sweep_sphere_against_deforming_mesh(
    const Vec3& sphere_start, const Vec3& sphere_end, float radius,
    const TriangleMeshCollider& previous_mesh, const TriangleMeshCollider& current_mesh,
    unsigned max_iterations = 32) {
    SweptMeshContact result;
    if (!is_finite(sphere_start) || !is_finite(sphere_end) ||
        !std::isfinite(radius) || radius < 0.0F || max_iterations == 0 ||
        previous_mesh.triangles.size() != current_mesh.triangles.size()) return result;

    // Current-mesh BVH bounds are valid only for the current geometry.
    // The caller must rebuild/refit that BVH after changing current vertices.
    const auto triangle_displacement = [&](std::size_t i) {
        if (i >= previous_mesh.triangles.size() || i >= current_mesh.triangles.size()) return 0.0F;
        const Triangle& a = previous_mesh.triangles[i];
        const Triangle& b = current_mesh.triangles[i];
        if (!is_finite(a.a) || !is_finite(a.b) || !is_finite(a.c) ||
            !is_finite(b.a) || !is_finite(b.b) || !is_finite(b.c)) return 0.0F;
        return std::max({length(b.a-a.a), length(b.b-a.b), length(b.c-a.c)});
    };

    const TriangleAabb query{
        {std::min(sphere_start.x,sphere_end.x)-radius,
         std::min(sphere_start.y,sphere_end.y)-radius,
         std::min(sphere_start.z,sphere_end.z)-radius},
        {std::max(sphere_start.x,sphere_end.x)+radius,
         std::max(sphere_start.y,sphere_end.y)+radius,
         std::max(sphere_start.z,sphere_end.z)+radius}};
    const auto overlaps=[](const TriangleAabb& a,const TriangleAabb& b) {
        return a.minimum.x<=b.maximum.x && a.maximum.x>=b.minimum.x &&
            a.minimum.y<=b.maximum.y && a.maximum.y>=b.minimum.y &&
            a.minimum.z<=b.maximum.z && a.maximum.z>=b.minimum.z;
    };
    std::vector<std::size_t> candidates;
    if (!current_mesh.bvh_nodes.empty() && !current_mesh.bvh_triangle_indices.empty()) {
        // Compute a conservative displacement bound per BVH node. A branch
        // only expands by the largest vertex movement among triangles below it,
        // rather than by the largest movement anywhere in the whole mesh.
        std::vector<float> node_displacement(current_mesh.bvh_nodes.size(), 0.0F);
        const auto measure_node = [&](auto&& self, int index) -> float {
            if (index < 0 || static_cast<std::size_t>(index) >= current_mesh.bvh_nodes.size()) return 0.0F;
            const TriangleBvhNode& node = current_mesh.bvh_nodes[static_cast<std::size_t>(index)];
            float maximum = 0.0F;
            if (node.is_leaf()) {
                const std::size_t end = std::min(node.first + node.count,
                                                 current_mesh.bvh_triangle_indices.size());
                for (std::size_t i = node.first; i < end; ++i) {
                    maximum = std::max(maximum,
                        triangle_displacement(current_mesh.bvh_triangle_indices[i]));
                }
            } else {
                maximum = std::max(self(self, node.left), self(self, node.right));
            }
            node_displacement[static_cast<std::size_t>(index)] = maximum;
            return maximum;
        };
        measure_node(measure_node, 0);

        const auto visit=[&](auto&& self,int index)->void {
            if (index < 0 || static_cast<std::size_t>(index) >= current_mesh.bvh_nodes.size()) return;
            const TriangleBvhNode& node=current_mesh.bvh_nodes[static_cast<std::size_t>(index)];
            const float displacement = node_displacement[static_cast<std::size_t>(index)];
            const Vec3 pad{displacement, displacement, displacement};
            const TriangleAabb bounds{node.bounds.minimum-pad,node.bounds.maximum+pad};
            if (!overlaps(bounds,query)) return;
            if (node.is_leaf()) {
                const std::size_t end = std::min(node.first + node.count,
                                                 current_mesh.bvh_triangle_indices.size());
                for(std::size_t i=node.first;i<end;++i)
                    candidates.push_back(current_mesh.bvh_triangle_indices[i]);
            } else { self(self,node.left); self(self,node.right); }
        };
        visit(visit,0);
    } else {
        candidates.reserve(current_mesh.triangles.size());
        for(std::size_t i=0;i<current_mesh.triangles.size();++i) candidates.push_back(i);
    }

    const Vec3 travel=sphere_end-sphere_start;
    const float sphere_speed=length(travel);
    float earliest=2.0F;
    for(const std::size_t index:candidates) {
        if(index>=previous_mesh.triangles.size() || index>=current_mesh.triangles.size()) continue;
        const Triangle& a=previous_mesh.triangles[index];
        const Triangle& b=current_mesh.triangles[index];
        if(!is_finite(a.a)||!is_finite(a.b)||!is_finite(a.c)||
           !is_finite(b.a)||!is_finite(b.b)||!is_finite(b.c)) continue;
        const float speed=sphere_speed+triangle_displacement(index);
        const auto triangle_at=[&](float t) { return Triangle{
            a.a+(b.a-a.a)*t,a.b+(b.b-a.b)*t,a.c+(b.c-a.c)*t}; };
        float t=0.0F;
        for(unsigned iteration=0;iteration<max_iterations;++iteration) {
            const Vec3 center=sphere_start+travel*t;
            const Triangle tri=triangle_at(t);
            const Vec3 closest=closest_point_on_triangle(center,tri);
            const Vec3 offset=center-closest;
            const float distance=length(offset);
            if(!std::isfinite(distance)) break;
            if(distance<=radius+1.0e-5F) {
                if(t<earliest) {
                    Vec3 normal=distance>1.0e-7F ? normalized(offset) :
                        normalized(cross(tri.b-tri.a,tri.c-tri.a));
                    if(!is_finite(normal)||length(normal)<=1.0e-7F) break;
                    earliest=t; result.hit=true; result.time_of_impact=t;
                    result.normal=normal;
                    result.position=center+normal*std::max(radius-distance,0.0F);
                }
                break;
            }
            if(speed<=1.0e-7F) break;
            const float step=(distance-radius)/speed;
            if(!(step>1.0e-7F)||!std::isfinite(step)) break;
            t+=step;
            if(t>1.0F||t>=earliest) break;
        }
    }
    return result;
}

// Ray/AABB test used by the BVH-accelerated closed-mesh containment query.
inline bool ray_intersects_aabb(const Vec3& origin, const Vec3& direction,
                                const TriangleAabb& bounds) noexcept {
    float near_t = 0.0F;
    float far_t = INFINITY;
    const float origins[3] = {origin.x, origin.y, origin.z};
    const float directions[3] = {direction.x, direction.y, direction.z};
    const float minimums[3] = {bounds.minimum.x, bounds.minimum.y, bounds.minimum.z};
    const float maximums[3] = {bounds.maximum.x, bounds.maximum.y, bounds.maximum.z};
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(directions[axis]) <= 1.0e-8F) {
            if (origins[axis] < minimums[axis] || origins[axis] > maximums[axis]) return false;
            continue;
        }
        float a = (minimums[axis] - origins[axis]) / directions[axis];
        float b = (maximums[axis] - origins[axis]) / directions[axis];
        if (a > b) std::swap(a, b);
        near_t = std::max(near_t, a);
        far_t = std::min(far_t, b);
        if (far_t < near_t) return false;
    }
    return far_t >= 0.0F;
}

inline bool ray_intersects_triangle(const Vec3& origin, const Vec3& direction,
                                    const Triangle& triangle, float& distance) noexcept {
    const Vec3 edge1 = triangle.b - triangle.a;
    const Vec3 edge2 = triangle.c - triangle.a;
    const Vec3 p = cross(direction, edge2);
    const float determinant = dot(edge1, p);
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1.0e-8F) return false;
    const float inverse = 1.0F / determinant;
    const Vec3 t = origin - triangle.a;
    const float u = dot(t, p) * inverse;
    if (u < -1.0e-6F || u > 1.0F + 1.0e-6F) return false;
    const Vec3 q = cross(t, edge1);
    const float v = dot(direction, q) * inverse;
    if (v < -1.0e-6F || u + v > 1.0F + 1.0e-6F) return false;
    distance = dot(edge2, q) * inverse;
    return std::isfinite(distance) && distance > 1.0e-6F;
}

// Classifies a point using odd/even ray crossings. Requires a closed, non-self-
// intersecting triangle surface. Points on the surface are reported separately
// as boundary/unknown so callers can resolve contact without a false inside flag.
// A skew ray avoids common axis-aligned edge/vertex degeneracies. BVH traversal
// skips nodes the ray cannot reach; the no-BVH path remains a reference fallback.
enum class MeshContainment { outside, inside, boundary_or_unknown };

inline MeshContainment classify_point_against_closed_mesh(
    const Vec3& point, const TriangleMeshCollider& mesh,
    float surface_tolerance = 1.0e-5F) noexcept {
    if (!is_finite(point) || !std::isfinite(surface_tolerance) || surface_tolerance < 0.0F ||
        mesh.triangles.empty()) return MeshContainment::boundary_or_unknown;

    const ClosestTrianglePoint nearest = closest_point_on_mesh(point, mesh);
    if (!nearest.valid) return MeshContainment::boundary_or_unknown;
    if (nearest.distance_squared <= surface_tolerance * surface_tolerance) {
        return MeshContainment::boundary_or_unknown;
    }

    const Vec3 direction = normalized(Vec3{1.0F, 0.37139067F, 0.5291031F});
    std::size_t crossings = 0;
    bool ambiguous = false;
    const auto consider_triangle = [&](std::size_t index) {
        if (index >= mesh.triangles.size()) return;
        const Triangle& triangle = mesh.triangles[index];
        if (!is_finite(triangle.a) || !is_finite(triangle.b) || !is_finite(triangle.c)) return;
        float distance = 0.0F;
        if (!ray_intersects_triangle(point, direction, triangle, distance)) return;
        const Vec3 hit = point + direction * distance;
        const Vec3 closest = closest_point_on_triangle(hit, triangle);
        if (length(hit - closest) <= 1.0e-5F) {
            // Mark near-edge hits as ambiguous; a second ray direction would be
            // a future robustness upgrade for pathological triangulations.
            const Vec3 ab = triangle.b - triangle.a;
            const Vec3 ac = triangle.c - triangle.a;
            const Vec3 ap = hit - triangle.a;
            const float d00 = dot(ab, ab), d01 = dot(ab, ac), d11 = dot(ac, ac);
            const float d20 = dot(ap, ab), d21 = dot(ap, ac);
            const float denom = d00 * d11 - d01 * d01;
            if (std::abs(denom) > 1.0e-12F) {
                const float v = (d11 * d20 - d01 * d21) / denom;
                const float w = (d00 * d21 - d01 * d20) / denom;
                if (v < 1.0e-5F || w < 1.0e-5F || v + w > 1.0F - 1.0e-5F) {
                    ambiguous = true;
                }
            }
            ++crossings;
        }
    };

    if (mesh.bvh_nodes.empty() || mesh.bvh_triangle_indices.empty()) {
        for (std::size_t i = 0; i < mesh.triangles.size(); ++i) consider_triangle(i);
    } else {
        const auto visit = [&](auto&& self, int node_index) -> void {
            if (node_index < 0 || static_cast<std::size_t>(node_index) >= mesh.bvh_nodes.size()) return;
            const TriangleBvhNode& node = mesh.bvh_nodes[static_cast<std::size_t>(node_index)];
            if (!ray_intersects_aabb(point, direction, node.bounds)) return;
            if (node.is_leaf()) {
                for (std::size_t i = node.first; i < node.first + node.count; ++i) {
                    consider_triangle(mesh.bvh_triangle_indices[i]);
                }
            } else {
                self(self, node.left);
                self(self, node.right);
            }
        };
        visit(visit, 0);
    }
    if (ambiguous) return MeshContainment::boundary_or_unknown;
    return (crossings % 2U) == 1U ? MeshContainment::inside : MeshContainment::outside;
}

// Odd/even ray test for points against a closed, watertight triangle mesh.
// The mesh must describe a closed surface; this does not validate manifoldness.
// Points within tolerance of the surface are treated as boundary/outside.
inline bool point_inside_closed_mesh(const Vec3& point,
                                     const TriangleMeshCollider& mesh,
                                     float surface_tolerance = 1.0e-5F) noexcept {
    if (!is_finite(point) || !std::isfinite(surface_tolerance) || surface_tolerance < 0.0F ||
        mesh.triangles.empty()) return false;
    const ClosestTrianglePoint nearest = closest_point_on_mesh(point, mesh);
    if (!nearest.valid || nearest.distance_squared <= surface_tolerance * surface_tolerance) return false;
    const Vec3 direction = normalized(Vec3{1.0F, 0.37139067F, 0.52910316F});
    std::size_t intersections = 0;
    constexpr float epsilon = 1.0e-7F;
    const auto ray_hits_triangle = [&](const Triangle& triangle) {
        if (!is_finite(triangle.a) || !is_finite(triangle.b) || !is_finite(triangle.c)) return;
        const Vec3 edge1 = triangle.b - triangle.a;
        const Vec3 edge2 = triangle.c - triangle.a;
        const Vec3 pvec{direction.y * edge2.z - direction.z * edge2.y,
                        direction.z * edge2.x - direction.x * edge2.z,
                        direction.x * edge2.y - direction.y * edge2.x};
        const float determinant = dot(edge1, pvec);
        if (std::abs(determinant) <= epsilon) return;
        const float inverse = 1.0F / determinant;
        const Vec3 tvec = point - triangle.a;
        const float u = dot(tvec, pvec) * inverse;
        if (u < -epsilon || u > 1.0F + epsilon) return;
        const Vec3 qvec{tvec.y * edge1.z - tvec.z * edge1.y,
                        tvec.z * edge1.x - tvec.x * edge1.z,
                        tvec.x * edge1.y - tvec.y * edge1.x};
        const float v = dot(direction, qvec) * inverse;
        if (v < -epsilon || u + v > 1.0F + epsilon) return;
        const float distance = dot(edge2, qvec) * inverse;
        if (distance > surface_tolerance) ++intersections;
    };
    const auto ray_hits_bounds = [&](const TriangleAabb& bounds) {
        float near_t = 0.0F;
        float far_t = INFINITY;
        for (int axis = 0; axis < 3; ++axis) {
            const float origin = axis == 0 ? point.x : (axis == 1 ? point.y : point.z);
            const float dir = axis == 0 ? direction.x : (axis == 1 ? direction.y : direction.z);
            const float low = axis == 0 ? bounds.minimum.x : (axis == 1 ? bounds.minimum.y : bounds.minimum.z);
            const float high = axis == 0 ? bounds.maximum.x : (axis == 1 ? bounds.maximum.y : bounds.maximum.z);
            if (std::abs(dir) <= epsilon) {
                if (origin < low || origin > high) return false;
                continue;
            }
            float a = (low - origin) / dir;
            float b = (high - origin) / dir;
            if (a > b) std::swap(a, b);
            near_t = std::max(near_t, a);
            far_t = std::min(far_t, b);
            if (near_t > far_t) return false;
        }
        return far_t >= 0.0F;
    };
    if (mesh.bvh_nodes.empty() || mesh.bvh_triangle_indices.empty()) {
        for (const Triangle& triangle : mesh.triangles) ray_hits_triangle(triangle);
    } else {
        const auto visit = [&](auto&& self, int node_index) -> void {
            const TriangleBvhNode& node = mesh.bvh_nodes[static_cast<std::size_t>(node_index)];
            if (!ray_hits_bounds(node.bounds)) return;
            if (node.is_leaf()) {
                for (std::size_t i = node.first; i < node.first + node.count; ++i) {
                    ray_hits_triangle(mesh.triangles[mesh.bvh_triangle_indices[i]]);
                }
                return;
            }
            self(self, node.left);
            self(self, node.right);
        };
        visit(visit, 0);
    }
    return (intersections % 2U) == 1U;
}

// Projects a point out of a closed solid mesh when it is fully inside.
// This is intentionally opt-in: open or non-manifold surfaces must remain surface-only.
inline Contact resolve_closed_mesh_contact(Vec3& point, float particle_radius,
                                           const TriangleMeshCollider& mesh,
                                           float surface_tolerance = 1.0e-5F) noexcept {
    Contact contact;
    if (!mesh.closed_mesh_solid || !is_finite(point) || !std::isfinite(particle_radius) ||
        particle_radius < 0.0F) return contact;
    if (classify_point_against_closed_mesh(point, mesh, surface_tolerance) != MeshContainment::inside) {
        return contact;
    }

    const ClosestTrianglePoint closest = closest_point_on_mesh(point, mesh);
    if (!closest.valid) return contact;
    const Vec3 outward = normalized(closest.position - point);
    if (!is_finite(outward) || length(outward) <= 1.0e-7F) return contact;

    const float distance = std::sqrt(std::max(closest.distance_squared, 0.0F));
    point = closest.position + outward * particle_radius;
    contact.position = closest.position;
    contact.normal = outward;
    contact.penetration_depth = distance + particle_radius;
    contact.touching = true;
    return contact;
}

// Surface contact for open meshes; closed-mesh projection is an explicit opt-in above.
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
