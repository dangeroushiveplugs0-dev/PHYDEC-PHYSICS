#include "phydec/collision_mesh.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <utility>

int main() {
    using namespace phydec;
    using Clock = std::chrono::steady_clock;

    // Deterministic grid gives a repeatable workload without random-number noise.
    TriangleMeshCollider accelerated;
    constexpr int grid_width = 80;
    for (int x = 0; x < grid_width; ++x) {
        for (int y = 0; y < grid_width; ++y) {
            const float x0 = static_cast<float>(x);
            const float y0 = static_cast<float>(y);
            accelerated.triangles.push_back(Triangle{
                {x0, y0, 0.0F}, {x0 + 1.0F, y0, 0.0F}, {x0, y0 + 1.0F, 0.0F}});
            accelerated.triangles.push_back(Triangle{
                {x0 + 1.0F, y0, 0.0F}, {x0 + 1.0F, y0 + 1.0F, 0.0F},
                {x0, y0 + 1.0F, 0.0F}});
        }
    }
    if (!build_triangle_mesh_bvh(accelerated, 4)) {
        std::cerr << "BVH build failed\n";
        return 1;
    }

    TriangleMeshCollider reference = accelerated;
    reference.bvh_nodes.clear();
    reference.bvh_triangle_indices.clear();

    constexpr std::size_t query_count = 1200;
    Vec3 queries[query_count];
    for (std::size_t i = 0; i < query_count; ++i) {
        const float n = static_cast<float>(i);
        queries[i] = Vec3{
            std::fmod(n * 17.13F, 84.0F) - 2.0F,
            std::fmod(n * 29.71F, 84.0F) - 2.0F,
            0.25F + std::fmod(n * 0.37F, 3.0F)};
    }

    volatile double checksum = 0.0;
    const auto run = [&](const TriangleMeshCollider& mesh) {
        const auto start = Clock::now();
        double local_checksum = 0.0;
        for (const Vec3& query : queries) {
            const ClosestTrianglePoint closest = closest_point_on_mesh(query, mesh);
            if (!closest.valid) return std::pair<double, double>{-1.0, -1.0};
            local_checksum += static_cast<double>(closest.distance_squared);
            local_checksum += static_cast<double>(closest.position.x) * 1.0e-6;
        }
        const auto elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        checksum = checksum + local_checksum;
        return std::pair<double, double>{elapsed, local_checksum};
    };

    const auto bvh = run(accelerated);
    const auto brute = run(reference);
    if (bvh.first < 0.0 || brute.first < 0.0 ||
        std::abs(bvh.second - brute.second) > 1.0e-3) {
        std::cerr << "Benchmark correctness check failed\n";
        return 2;
    }

    std::cout << "PHYDEC mesh nearest-point benchmark\n"
              << "Triangles: " << accelerated.triangles.size() << '\n'
              << "Queries: " << query_count << '\n'
              << std::fixed << std::setprecision(3)
              << "BVH: " << bvh.first << " ms\n"
              << "Brute-force reference: " << brute.first << " ms\n"
              << "Speedup: " << (brute.first / bvh.first) << "x\n"
              << "Checksum guard: " << static_cast<double>(checksum) << '\n'
              << "Note: compare on the same machine/build; timing is informational, not a CI pass threshold.\n";
    return 0;
}
