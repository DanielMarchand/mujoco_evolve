#include <mujoco/mujoco.h>
#include <iostream>

int main() {
    std::cout << "mujoco_evolve v0.1.0\n";
    std::cout << "MuJoCo version: " << mj_versionString() << "\n";
    return 0;
}
