#include <phydec/phydec.hpp>

int main() {
    phydec::SecondaryMotionChain chain({
        phydec::Vec3{0.0F, 0.0F, 0.0F},
        phydec::Vec3{0.0F, 0.0F, 1.0F}
    });
    chain.set_anchors({
        phydec::Vec3{0.0F, 0.0F, 0.0F},
        phydec::Vec3{0.0F, 0.0F, 1.0F}
    });
    chain.step(phydec::SecondaryMotionSettings{});
    return chain.valid() ? 0 : 1;
}
