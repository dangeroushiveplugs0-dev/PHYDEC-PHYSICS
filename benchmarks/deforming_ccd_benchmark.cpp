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
    TriangleMeshCollider previous, current;
    previous.triangles.push_back(Triangle{{-1,-1,-2},{1,-1,-2},{0,1,-2}});
    current.triangles.push_back(Triangle{{-1,-1,2},{1,-1,2},{0,1,2}});
    constexpr int side = 64;
    for (int x=0; x<side; ++x) for (int y=0; y<side; ++y) {
        const float x0=10.0F+static_cast<float>(x)*2.0F;
        const float y0=10.0F+static_cast<float>(y)*2.0F;
        const Triangle tri{{x0,y0,0},{x0+1,y0,0},{x0,y0+1,0}};
        previous.triangles.push_back(tri);
        current.triangles.push_back(tri);
    }
    if (!build_triangle_mesh_bvh(current,4)) {
        std::cerr << "BVH build failed\n"; return 1;
    }
    TriangleMeshCollider brute=current;
    brute.bvh_nodes.clear(); brute.bvh_triangle_indices.clear();
    constexpr unsigned repetitions=100;
    const Vec3 sphere{0,0,0};
    DeformingMeshSweepStats stats;
    const auto accelerated_run=[&]() {
        const auto begin=Clock::now();
        SweptMeshContact hit;
        for(unsigned i=0;i<repetitions;++i)
            hit=sweep_sphere_against_deforming_mesh(sphere,sphere,0.1F,previous,current,32,&stats);
        return std::pair<double,SweptMeshContact>{
            std::chrono::duration<double,std::milli>(Clock::now()-begin).count(),hit};
    };
    const auto brute_run=[&]() {
        const auto begin=Clock::now();
        SweptMeshContact hit;
        for(unsigned i=0;i<repetitions;++i)
            hit=sweep_sphere_against_deforming_mesh(sphere,sphere,0.1F,previous,brute);
        return std::pair<double,SweptMeshContact>{
            std::chrono::duration<double,std::milli>(Clock::now()-begin).count(),hit};
    };
    const auto accelerated=accelerated_run();
    const auto measured=stats;
    const auto reference=brute_run();
    if(!accelerated.second.hit||!reference.second.hit||
       std::abs(accelerated.second.time_of_impact-reference.second.time_of_impact)>1.0e-4F||
       length(accelerated.second.position-reference.second.position)>1.0e-4F||
       length(accelerated.second.normal-reference.second.normal)>1.0e-4F||
       measured.candidate_triangles>=current.triangles.size()/10) {
        std::cerr << "Deforming CCD correctness/candidate guard failed\n"; return 2;
    }
    std::cout << "PHYDEC deforming-mesh CCD candidate benchmark\n"
              << "Triangles: " << current.triangles.size() << '\n'
              << "Repeated queries: " << repetitions << '\n'
              << "BVH nodes visited/query: " << measured.bvh_nodes_visited << '\n'
              << "BVH candidate triangles/query: " << measured.candidate_triangles << '\n'
              << "Brute-force triangles/query: " << brute.triangles.size() << '\n'
              << std::fixed << std::setprecision(3)
              << "BVH total: " << accelerated.first << " ms\n"
              << "Brute-force total: " << reference.first << " ms\n"
              << "Timing ratio (brute/BVH): " << reference.first/accelerated.first << "x\n"
              << "Correctness and candidate-reduction guards: passed\n"
              << "Timing is informational; compare on the same machine/build.\n";
    return 0;
}
