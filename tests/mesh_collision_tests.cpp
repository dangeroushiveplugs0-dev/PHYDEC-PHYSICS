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
    assert(std::abs(unchanged.x - 1.0F) < 1.0e-6F);

    TriangleMeshCollider invalid_mesh;
    invalid_mesh.triangles.push_back(Triangle{
        {NAN, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {1.0F, 0.0F, 0.0F}
    });
    Vec3 invalid_mesh_point{};
    assert(!resolve_mesh_contact(invalid_mesh_point, 0.1F, invalid_mesh).touching);

    // Mesh contact also supports a point near an edge, not just face interiors.
    Vec3 edge_point{2.02F, -2.0F, 0.0F};
    const Contact edge_hit = resolve_mesh_contact(edge_point, 0.1F, mesh);
    assert(edge_hit.touching);
    assert(is_finite(edge_point));
    return 0;
}
