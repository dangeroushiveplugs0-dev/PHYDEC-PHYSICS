#include "phydec/collision_mesh.hpp"

#include <cassert>
#include <cmath>

int main() {
    using namespace phydec;

    TriangleMeshCollider mesh;
    mesh.triangles.push_back(Triangle{
        {-2.0F, -2.0F, 0.0F},
        {2.0F, -2.0F, 0.0F},
        {0.0F, 2.0F, 0.0F}
    });

    Vec3 point{0.0F, 0.0F, 0.02F};
    const Contact hit = resolve_mesh_contact(point, 0.1F, mesh);
    assert(hit.touching);
    assert(std::abs(hit.penetration_depth - 0.08F) < 1.0e-5F);
    assert(std::abs(point.z - 0.1F) < 1.0e-5F);
    assert(std::abs(hit.normal.z - 1.0F) < 1.0e-5F);

    Vec3 miss_point{0.0F, 0.0F, 0.2F};
    assert(!resolve_mesh_contact(miss_point, 0.1F, mesh).touching);

    const Vec3 nearest = closest_point_on_triangle(
        Vec3{3.0F, 0.0F, 0.0F}, mesh.triangles.front());
    assert(is_finite(nearest));
    assert(nearest.x <= 2.0F);

    TriangleMeshCollider empty_mesh;
    Vec3 unchanged{1.0F, 2.0F, 3.0F};
    assert(!resolve_mesh_contact(unchanged, 0.5F, empty_mesh).touching);
    assert(!build_triangle_mesh_bvh(empty_mesh));
    assert(std::abs(unchanged.x - 1.0F) < 1.0e-6F);

    TriangleMeshCollider invalid_mesh;
    invalid_mesh.triangles.push_back(Triangle{
        {NAN, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {1.0F, 0.0F, 0.0F}
    });
    Vec3 invalid_mesh_point{};
    assert(!resolve_mesh_contact(invalid_mesh_point, 0.1F, invalid_mesh).touching);
    assert(!build_triangle_mesh_bvh(invalid_mesh));

    // Mesh contact also supports a point near an edge, not just face interiors.
    Vec3 edge_point{2.02F, -2.0F, 0.0F};
    const Contact edge_hit = resolve_mesh_contact(edge_point, 0.1F, mesh);
    assert(edge_hit.touching);
    assert(is_finite(edge_point));

    // Build a larger mesh so the BVH must split into multiple levels.
    TriangleMeshCollider grid;
    for (int x = 0; x < 12; ++x) {
        for (int y = 0; y < 12; ++y) {
            const float x0 = static_cast<float>(x);
            const float y0 = static_cast<float>(y);
            grid.triangles.push_back(Triangle{
                {x0, y0, 0.0F}, {x0 + 1.0F, y0, 0.0F}, {x0, y0 + 1.0F, 0.0F}
            });
            grid.triangles.push_back(Triangle{
                {x0 + 1.0F, y0, 0.0F}, {x0 + 1.0F, y0 + 1.0F, 0.0F},
                {x0, y0 + 1.0F, 0.0F}
            });
        }
    }
    assert(build_triangle_mesh_bvh(grid, 3));
    assert(!grid.bvh_nodes.empty());
    assert(grid.bvh_nodes.front().left >= 0);
    assert(grid.bvh_nodes.front().right >= 0);

    // Compare accelerated results with the brute-force reference for several
    // points, including points outside the mesh bounds and above its surface.
    TriangleMeshCollider brute_force = grid;
    brute_force.bvh_nodes.clear();
    brute_force.bvh_triangle_indices.clear();
    const Vec3 queries[] = {
        {0.25F, 0.25F, 0.2F}, {7.7F, 4.2F, 1.5F},
        {11.8F, 11.8F, -0.4F}, {-2.0F, 3.0F, 0.7F},
        {6.0F, 6.0F, 0.0F}, {15.0F, -5.0F, 2.0F}
    };
    for (const Vec3& query : queries) {
        const ClosestTrianglePoint accelerated = closest_point_on_mesh(query, grid);
        const ClosestTrianglePoint reference = closest_point_on_mesh(query, brute_force);
        assert(accelerated.valid == reference.valid);
        assert(std::abs(accelerated.distance_squared - reference.distance_squared) < 1.0e-4F);
        assert(length(accelerated.position - reference.position) < 1.0e-4F);
    }

    // BVH build excludes invalid triangles while retaining finite triangles.
    grid.triangles.push_back(Triangle{
        {NAN, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}
    });
    assert(build_triangle_mesh_bvh(grid, 4));
    assert(grid.bvh_triangle_indices.size() + 1 == grid.triangles.size());
    const auto after_rebuild = closest_point_on_mesh(Vec3{2.0F, 2.0F, 0.5F}, grid);
    assert(after_rebuild.valid);
    return 0;
}
