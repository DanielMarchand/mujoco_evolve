#include <gtest/gtest.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <cstring>
#include <numeric>
#include <vector>

// ── Minimal model for rendering tests (sphere on a plane with a light).
static constexpr char kRenderXml[] = R"(
<mujoco model="render_test">
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    <body name="ball" pos="0 0 1">
      <geom type="sphere" size="0.2" rgba="0.9 0.2 0.2 1"/>
    </body>
  </worldbody>
</mujoco>
)";

// ── Test fixture: creates a hidden GLFW window for an OpenGL context,
//    loads a model, and initialises the MuJoCo visualisation pipeline.
//
//    MuJoCo does NOT create an OpenGL context itself — we must provide one.
//    A GLFW window with GLFW_VISIBLE=false gives us a valid GL context
//    without actually showing anything on screen.
class OffscreenRenderTest : public ::testing::Test {
 protected:
    GLFWwindow* window_ = nullptr;
    mjModel*    m_       = nullptr;
    mjData*     d_       = nullptr;
    mjvScene    scn_;
    mjvCamera   cam_;
    mjvOption   opt_;
    mjrContext  con_;

    void SetUp() override {
        // ── Create an invisible GLFW window just to get a GL context.
        ASSERT_TRUE(glfwInit()) << "glfwInit failed — is an X/Wayland display available?";

        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);       // hidden window
        glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);   // single-buffer for offscreen

        window_ = glfwCreateWindow(800, 600, "offscreen", nullptr, nullptr);
        ASSERT_NE(window_, nullptr) << "glfwCreateWindow failed";
        glfwMakeContextCurrent(window_);

        // ── Load model from string via VFS.
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        mj_addBufferVFS(&vfs, "render.xml", kRenderXml, std::strlen(kRenderXml));

        char error[1000] = "";
        m_ = mj_loadXML("render.xml", &vfs, error, sizeof(error));
        mj_deleteVFS(&vfs);
        ASSERT_NE(m_, nullptr) << "mj_loadXML failed: " << error;

        d_ = mj_makeData(m_);
        mj_forward(m_, d_);   // populate xpos, sensordata, etc.

        // ── Initialise visualisation structs.
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        mjv_makeScene(m_, &scn_, 2000);       // allocate scene with max 2000 geoms
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);   // create GPU resources
    }

    void TearDown() override {
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
    }

    // ── Helper: render the current scene to the offscreen buffer and
    //    return the RGB pixel data (3 bytes per pixel, row-major).
    std::vector<unsigned char> RenderToPixels() {
        mjr_setBuffer(mjFB_OFFSCREEN, &con_);

        mjrRect viewport = mjr_maxViewport(&con_);
        mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        mjr_render(viewport, &scn_, &con_);

        const int W = viewport.width;
        const int H = viewport.height;
        std::vector<unsigned char> rgb(3 * W * H, 0);
        mjr_readPixels(rgb.data(), nullptr, viewport, &con_);
        return rgb;
    }
};

// ── Verify that the offscreen framebuffer has non-zero dimensions.
TEST_F(OffscreenRenderTest, OffscreenBufferExists) {
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    mjrRect viewport = mjr_maxViewport(&con_);

    EXPECT_GT(viewport.width, 0)  << "Offscreen buffer has zero width";
    EXPECT_GT(viewport.height, 0) << "Offscreen buffer has zero height";
}

// ── Verify that rendering produces a non-black image (i.e. something
//    was actually drawn — the light, ground plane, and sphere).
TEST_F(OffscreenRenderTest, RenderProducesNonBlackImage) {
    auto rgb = RenderToPixels();

    long total = std::accumulate(rgb.begin(), rgb.end(), 0L);
    EXPECT_GT(total, 0) << "Rendered image is completely black — nothing was drawn";
}

// ── Verify that we can render a second time after stepping physics
//    (catches state-management bugs in the visualisation pipeline).
TEST_F(OffscreenRenderTest, RenderAfterPhysicsStep) {
    // Step physics so the sphere falls.
    for (int i = 0; i < 100; ++i) {
        mj_step(m_, d_);
    }

    auto rgb = RenderToPixels();

    long total = std::accumulate(rgb.begin(), rgb.end(), 0L);
    EXPECT_GT(total, 0) << "Rendered image is black after physics stepping";
}
