#pragma once

#include <cmath>
#include <vector>
#include "phydec/collision.hpp"

namespace phydec {

// A mesh collider stores the actual collision surface as triangles.
// Vertices are supplied in the same coordinate space as simulated points.
// This first implementation is a brute-force narrow phase; BVH acceleration
// and closed-volume containment are deliberately separate follow-up features.
struct Triangle {
    Vec3 a{};
    Vec3 b{};
    Vec3 c{};
};

struct TriangleMeshCollider {
    std::vector<Triangle> triangles{};
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
        // Degenerate triangle: choose the nearest of its three edges.
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

inline ClosestTrianglePoint closest_point_on_mesh(const Vec3& point,
                                                   const TriangleMeshCollider& mesh) noexcept {
    ClosestTrianglePoint best;
    best.distance_squared = INFINITY;
    if (!is_finite(point)) return best;

    for (const Triangle& triangle : mesh.triangles) {
        if (!is_finite(triangle.a) || !is_finite(triangle.b) || !is_finite(triangle.c)) continue;
        const Vec3 closest = closest_point_on_triangle(point, triangle);
        const Vec3 offset = point - closest;
        const float distance_squared = dot(offset, offset);
        if (!std::isfinite(distance_squared) || distance_squared >= best.distance_squared) continue;

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
    }
    return best;
}

// Sphere-vs-triangle-surface contact: projects the point to the nearest actual
// triangle and pushes it away from that surface. This is surface contact, not
// yet a watertight solid/inside-outside test.
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
