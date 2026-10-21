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


    // Continuous collision query catches a fast sphere crossing a thin triangle
    // even when its final position is already on the far side of the surface.
    const SweptMeshContact swept = sweep_sphere_against_mesh(
        Vec3{0.0F, 0.0F, 5.0F}, Vec3{0.0F, 0.0F, -5.0F}, 0.1F, mesh);
    assert(swept.hit);
    assert(swept.time_of_impact > 0.0F && swept.time_of_impact < 1.0F);
    assert(std::abs(swept.position.z - 0.1F) < 2.0e-4F);
    assert(swept.normal.z > 0.9F);

    const SweptMeshContact swept_miss = sweep_sphere_against_mesh(
        Vec3{8.0F, 8.0F, 5.0F}, Vec3{8.0F, 8.0F, -5.0F}, 0.1F, mesh);
    assert(!swept_miss.hit);

    const SweptMeshContact stationary = sweep_sphere_against_mesh(
        Vec3{0.0F, 0.0F, 2.0F}, Vec3{0.0F, 0.0F, 2.0F}, 0.1F, mesh);
    assert(!stationary.hit);

    // BVH build excludes invalid triangles while retaining finite triangles.
    grid.triangles.push_back(Triangle{
        {NAN, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}
    });
    assert(build_triangle_mesh_bvh(grid, 4));
    assert(grid.bvh_triangle_indices.size() + 1 == grid.triangles.size());
    const auto after_rebuild = closest_point_on_mesh(Vec3{2.0F, 2.0F, 0.5F}, grid);
    assert(after_rebuild.valid);
    // Deforming geometry can refit the existing hierarchy without rebuilding
    // its split structure, and results must still match the brute-force reference.
    TriangleMeshCollider moving_grid = grid;
    moving_grid.triangles.pop_back(); // remove the deliberately invalid triangle
    assert(build_triangle_mesh_bvh(moving_grid, 4));
    const std::size_t original_node_count = moving_grid.bvh_nodes.size();
    for (Triangle& triangle : moving_grid.triangles) {
        triangle.a.z += 3.0F;
        triangle.b.z += 3.0F;
        triangle.c.z += 3.0F;
    }
    assert(refit_triangle_mesh_bvh(moving_grid));
    assert(moving_grid.bvh_nodes.size() == original_node_count);

    TriangleMeshCollider moving_reference = moving_grid;
    moving_reference.bvh_nodes.clear();
    moving_reference.bvh_triangle_indices.clear();
    const Vec3 moved_query{2.25F, 3.25F, 3.5F};
    const ClosestTrianglePoint moved_accelerated = closest_point_on_mesh(moved_query, moving_grid);
    const ClosestTrianglePoint moved_reference_hit = closest_point_on_mesh(moved_query, moving_reference);
    assert(moved_accelerated.valid && moved_reference_hit.valid);
    assert(std::abs(moved_accelerated.distance_squared - moved_reference_hit.distance_squared) < 1.0e-4F);
    assert(length(moved_accelerated.position - moved_reference_hit.position) < 1.0e-4F);

    // A failed refit must discard stale bounds rather than silently querying them.
    moving_grid.triangles.front().a.x = NAN;
    assert(!refit_triangle_mesh_bvh(moving_grid));
    assert(moving_grid.bvh_nodes.empty());
    assert(moving_grid.bvh_triangle_indices.empty());

    // Closed cube containment: odd/even ray crossings should classify an
    // interior point and an exterior point, while surface points stay ambiguous.
    TriangleMeshCollider cube;
    const Vec3 v000{-1.0F, -1.0F, -1.0F}, v100{1.0F, -1.0F, -1.0F};
    const Vec3 v010{-1.0F, 1.0F, -1.0F}, v110{1.0F, 1.0F, -1.0F};
    const Vec3 v001{-1.0F, -1.0F, 1.0F}, v101{1.0F, -1.0F, 1.0F};
    const Vec3 v011{-1.0F, 1.0F, 1.0F}, v111{1.0F, 1.0F, 1.0F};
    const auto quad = [&cube](const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        cube.triangles.push_back(Triangle{a, b, c});
        cube.triangles.push_back(Triangle{a, c, d});
    };
    quad(v000, v010, v110, v100); // bottom
    quad(v001, v101, v111, v011); // top
    quad(v000, v100, v101, v001); // front
    quad(v010, v011, v111, v110); // back
    quad(v000, v001, v011, v010); // left
    quad(v100, v110, v111, v101); // right
    assert(build_triangle_mesh_bvh(cube, 2));
    assert(classify_point_against_closed_mesh(Vec3{0.13F, 0.21F, 0.37F}, cube) ==
           MeshContainment::inside);
    assert(classify_point_against_closed_mesh(Vec3{2.5F, 0.2F, -0.1F}, cube) ==
           MeshContainment::outside);
    assert(classify_point_against_closed_mesh(Vec3{1.0F, 0.0F, 0.0F}, cube) ==
           MeshContainment::boundary_or_unknown);

    // The reference path without a BVH must agree for ordinary interior/exterior queries.
    TriangleMeshCollider cube_reference = cube;
    cube_reference.bvh_nodes.clear();
    cube_reference.bvh_triangle_indices.clear();
    assert(classify_point_against_closed_mesh(Vec3{0.13F, 0.21F, 0.37F}, cube_reference) ==
           MeshContainment::inside);
    assert(classify_point_against_closed_mesh(Vec3{2.5F, 0.2F, -0.1F}, cube_reference) ==
           MeshContainment::outside);

    // Closed tetrahedron containment is separate from surface-contact resolution.
    TriangleMeshCollider tetrahedron;
    tetrahedron.triangles = {
        Triangle{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}},
        Triangle{{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F, 0.0F}},
        Triangle{{0.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}},
        Triangle{{1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F, 0.0F}}
    };
    assert(point_inside_closed_mesh(Vec3{0.1F, 0.1F, 0.1F}, tetrahedron));
    assert(!point_inside_closed_mesh(Vec3{1.2F, 1.2F, 1.2F}, tetrahedron));
    assert(!point_inside_closed_mesh(Vec3{0.2F, 0.2F, 0.0F}, tetrahedron));
    assert(!point_inside_closed_mesh(Vec3{NAN, 0.0F, 0.0F}, tetrahedron));
    assert(!point_inside_closed_mesh(Vec3{0.1F, 0.1F, 0.1F}, empty_mesh));
    assert(build_triangle_mesh_bvh(tetrahedron, 1));
    const TriangleMeshCollider tetrahedron_brute_force = [&]() {
        TriangleMeshCollider copy = tetrahedron;
        copy.bvh_nodes.clear();
        copy.bvh_triangle_indices.clear();
        return copy;
    }();
    const Vec3 containment_queries[] = {
        {0.1F, 0.1F, 0.1F}, {1.2F, 1.2F, 1.2F}, {0.2F, 0.2F, 0.0F},
        {-0.5F, 0.2F, 0.2F}, {0.25F, 0.25F, 0.25F}
    };
    for (const Vec3& query : containment_queries) {
        assert(point_inside_closed_mesh(query, tetrahedron) ==
               point_inside_closed_mesh(query, tetrahedron_brute_force));
    }

    return 0;
}
