#include "phydec/secondary_motion.hpp"

#include <cassert>
#include <cmath>
#include <vector>

int main() {
    using phydec::PlaneCollider;
    using phydec::SecondaryMotionChain;
    using phydec::SecondaryMotionSettings;
    using phydec::Triangle;
    using phydec::TriangleMeshCollider;
    using phydec::Vec3;

    SecondaryMotionChain chain({{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}});
    assert(chain.points().size() == 2);
    assert(chain.valid());

    SecondaryMotionSettings settings;
    settings.gravity = {0.0F, 0.0F, -9.81F};
    settings.fixed_delta_seconds = 1.0F / 60.0F;

    for (int i = 0; i < 120; ++i) {
        chain.set_anchors({{0.0F, 0.0F, 0.0F}, {0.1F, 0.0F, -1.0F}});
        chain.step(settings);
        assert(chain.valid());
        assert(std::isfinite(chain.points()[1].position.z));
    }

    const auto root = chain.points().front().position;
    assert(std::abs(root.x) < 1.0e-5F);
    assert(std::abs(root.y) < 1.0e-5F);
    assert(std::abs(root.z) < 1.0e-5F);

    const auto before = chain.points()[1].position;
    settings.fixed_delta_seconds = 0.0F;
    chain.step(settings);
    const auto after = chain.points()[1].position;
    assert(std::abs(before.x - after.x) < 1.0e-6F);
    assert(std::abs(before.y - after.y) < 1.0e-6F);
    assert(std::abs(before.z - after.z) < 1.0e-6F);

    SecondaryMotionChain colliding_chain({{0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}});
    SecondaryMotionSettings collision_settings;
    collision_settings.gravity = {};
    collision_settings.fixed_delta_seconds = 1.0F / 60.0F;
    collision_settings.plane_colliders.push_back(PlaneCollider{{}, {0.0F, 0.0F, 1.0F}});
    colliding_chain.step(collision_settings);
    assert(colliding_chain.valid());
    assert(colliding_chain.points()[1].position.z >=
           colliding_chain.points()[1].collision_radius - 1.0e-4F);
    assert(std::abs(colliding_chain.points()[0].position.z - 1.0F) < 1.0e-5F);

    SecondaryMotionChain mesh_chain({{0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}});
    SecondaryMotionSettings mesh_settings;
    mesh_settings.gravity = {};
    mesh_settings.fixed_delta_seconds = 1.0F / 60.0F;
    TriangleMeshCollider floor_mesh;
    floor_mesh.triangles.push_back(Triangle{
        {-2.0F, -2.0F, 0.0F}, {2.0F, -2.0F, 0.0F}, {0.0F, 2.0F, 0.0F}
    });
    mesh_settings.mesh_colliders.push_back(floor_mesh);
    mesh_chain.step(mesh_settings);
    assert(mesh_chain.valid());
    assert(mesh_chain.points()[1].position.z >=
           mesh_chain.points()[1].collision_radius - 1.0e-4F);

    // Explicit closed-solid mode should eject a secondary-motion point that
    // starts inside a closed collider. Ordinary triangle surfaces stay surface-only.
    SecondaryMotionChain solid_chain({{0.0F, 0.0F, 0.0F}, {0.1F, 0.0F, 0.0F}});
    SecondaryMotionSettings solid_settings;
    solid_settings.gravity = {};
    solid_settings.fixed_delta_seconds = 1.0F / 60.0F;
    TriangleMeshCollider solid_cube;
    const Vec3 v000{-1.0F, -1.0F, -1.0F}, v100{1.0F, -1.0F, -1.0F};
    const Vec3 v010{-1.0F, 1.0F, -1.0F}, v110{1.0F, 1.0F, -1.0F};
    const Vec3 v001{-1.0F, -1.0F, 1.0F}, v101{1.0F, -1.0F, 1.0F};
    const Vec3 v011{-1.0F, 1.0F, 1.0F}, v111{1.0F, 1.0F, 1.0F};
    const auto add_quad = [&solid_cube](const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        solid_cube.triangles.push_back(Triangle{a, b, c});
        solid_cube.triangles.push_back(Triangle{a, c, d});
    };
    add_quad(v000, v010, v110, v100);
    add_quad(v001, v101, v111, v011);
    add_quad(v000, v100, v101, v001);
    add_quad(v010, v011, v111, v110);
    add_quad(v000, v001, v011, v010);
    add_quad(v100, v110, v111, v101);
    solid_cube.closed_mesh_solid = true;
    assert(phydec::build_triangle_mesh_bvh(solid_cube));
    solid_settings.mesh_colliders.push_back(solid_cube);
    solid_chain.step(solid_settings);
    assert(solid_chain.valid());
    assert(!phydec::point_inside_closed_mesh(solid_chain.points()[1].position, solid_cube));
    return 0;
}
