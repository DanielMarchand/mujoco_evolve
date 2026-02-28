#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <cstring>
#include <string>

// ── Verify the MuJoCo library is present and reports the expected version.
TEST(MuJoCo, VersionString) {
    const char* version = mj_versionString();
    ASSERT_NE(version, nullptr);
    EXPECT_STRNE(version, "");
    // We built against 3.2.7
    EXPECT_STREQ(version, "3.2.7");
}

TEST(MuJoCo, VersionNumber) {
    int ver = mj_version();
    EXPECT_GT(ver, 0);
}

// ── Verify we can compile a minimal MJCF model from an in-memory string.
static constexpr char kMinimalXml[] = R"(
<mujoco>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1"/>
    <body name="capsule" pos="0 0 1">
      <joint name="slide" type="slide" axis="0 0 1"/>
      <geom type="capsule" size="0.1" fromto="0 0 0 0 0 0.5" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="slide" gear="100"/>
  </actuator>
  <sensor>
    <jointpos joint="slide"/>
    <jointvel joint="slide"/>
  </sensor>
</mujoco>
)";

TEST(MuJoCo, LoadModelFromString) {
    char error[1000] = "";
    mjModel* m = mj_loadXML("<string>", nullptr, error, sizeof(error));

    // That was an invalid path — now load from a VFS instead.
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    mj_addBufferVFS(&vfs, "test.xml", kMinimalXml, std::strlen(kMinimalXml));

    m = mj_loadXML("test.xml", &vfs, error, sizeof(error));
    ASSERT_NE(m, nullptr) << "mj_loadXML failed: " << error;

    EXPECT_EQ(m->njnt, 1);      // one slide joint
    EXPECT_EQ(m->nu, 1);        // one motor actuator
    EXPECT_EQ(m->nsensordata, 2); // jointpos + jointvel

    mj_deleteVFS(&vfs);
    mj_deleteModel(m);
}

// ── Verify we can step the simulation and state changes.
TEST(MuJoCo, StepSimulation) {
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    mj_addBufferVFS(&vfs, "test.xml", kMinimalXml, std::strlen(kMinimalXml));

    char error[1000] = "";
    mjModel* m = mj_loadXML("test.xml", &vfs, error, sizeof(error));
    ASSERT_NE(m, nullptr) << error;

    mjData* d = mj_makeData(m);
    ASSERT_NE(d, nullptr);

    // Record initial position
    double z0 = d->qpos[0];

    // Step a few hundred times with no control — capsule should fall under gravity
    for (int i = 0; i < 500; ++i) {
        mj_step(m, d);
    }

    double z1 = d->qpos[0];
    EXPECT_LT(z1, z0) << "Capsule should fall under gravity (slide joint along z)";
    EXPECT_GT(d->time, 0.0);

    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deleteVFS(&vfs);
}

// ── Verify simulation determinism: two runs with same seed produce identical state.
TEST(MuJoCo, Determinism) {
    auto run_sim = []() -> double {
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        mj_addBufferVFS(&vfs, "test.xml", kMinimalXml, std::strlen(kMinimalXml));

        char error[1000] = "";
        mjModel* m = mj_loadXML("test.xml", &vfs, error, sizeof(error));
        if (!m) return -999.0;

        mjData* d = mj_makeData(m);
        for (int i = 0; i < 1000; ++i) {
            mj_step(m, d);
        }
        double result = d->qpos[0];

        mj_deleteData(d);
        mj_deleteModel(m);
        mj_deleteVFS(&vfs);
        return result;
    };

    double run1 = run_sim();
    double run2 = run_sim();
    EXPECT_DOUBLE_EQ(run1, run2) << "Identical simulations must produce identical results";
}
