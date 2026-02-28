# MuJoCo 3.2.7 — Deep Dive Notes for mujoco_evolve

> Cross-referenced against `research.md` and `planning.md`.
> Source: `third_party/mujoco/` (cloned from tag 3.2.7).


---

## 1. Architecture Overview

### 1.1 The mjModel / mjData Split

MuJoCo enforces a strict **static vs. dynamic** dichotomy.

| Struct     | Lifetime        | Contents                                                | Mutability during sim |
|------------|-----------------|--------------------------------------------------------|-----------------------|
| `mjModel`  | Compile → delete | Body tree, joint types, geom shapes, actuator params, sensor config, solver options | **Read-only** (const in all stepping functions) |
| `mjData`   | Per-rollout      | `qpos`, `qvel`, `act`, `ctrl`, `sensordata`, `xpos`, contact arrays, constraint forces | **Read-write** (mutated by `mj_step`) |

**Key fields on `mjModel` we will query programmatically:**

```
m->nq             // dim of generalized position (qpos)
m->nv             // dim of generalized velocity (qvel)  — NOTE: nv != nq for free/ball joints
m->nu             // number of actuators = dim(ctrl)
m->na             // number of activation states = dim(act)
m->nbody          // number of bodies (including world body 0)
m->njnt           // number of joints
m->ngeom          // number of geoms
m->nsensor        // number of sensor objects
m->nsensordata    // total scalar sensor readings = dim(sensordata)
```

**Implication for planning TODO 6 (Sensor-Actuator Wiring):**
- Network input size = `m->nsensordata` (NOT `m->nsensor` — one sensor can produce multiple scalars, e.g. `framequat` outputs 4 floats).
- Network output size = `m->nu`.
- These are only known *after* `mj_compile()` or `mj_loadXML()`.

### 1.2 Model Instance Representation

```
File/memory           High-level         Low-level
─────────────────────────────────────────────────
XML string   ↔  mjSpec (editable)  →  mjModel (compiled, static)
MJB binary              —           ↔  mjModel
```

Two paths to get an `mjModel`:
1. **XML route:** build XML string → `mj_loadXML()` (parses XML, creates internal spec, compiles).
2. **mjSpec route:** `mj_makeSpec()` → add elements → `mj_compile(spec, vfs)` → `mjModel*`.

The mjSpec route **skips all XML parsing** — this is significant for our GA evaluation loop.


---

## 2. The mjSpec Programmatic API (New in 3.2.0)

### 2.1 Workflow

```c++
mjSpec* spec = mj_makeSpec();

// Access pre-existing world body
mjsBody* world = mjs_findBody(spec, "world");

// Add a child body
mjsBody* torso = mjs_addBody(world, /*default_class=*/0);
mjs_setString(torso->name, "torso");
torso->pos[0] = 0; torso->pos[1] = 0; torso->pos[2] = 0.5;

// Add geometry to the body
mjsGeom* geom = mjs_addGeom(torso, 0);
geom->type = mjGEOM_CAPSULE;
geom->size[0] = 0.05;  // radius
geom->size[1] = 0.2;   // half-length
// If using fromto instead: set geom->fromto[0..5] (NOT compatible with size[1])

// Add a joint (attaches between parent and this body)
mjsJoint* jnt = mjs_addJoint(torso, 0);
jnt->type = mjJNT_HINGE;
jnt->axis[0] = 0; jnt->axis[1] = 1; jnt->axis[2] = 0;
mjs_setString(jnt->name, "joint_0");

// Add an actuator (spec-level, NOT body-level)
mjsActuator* act = mjs_addActuator(spec, 0);
mjs_setString(act->target, "joint_0");
act->trntype = mjTRN_JOINT;
act->gainprm[0] = 1.0;   // gain (for motor: torque = gain * ctrl)
// For a simple motor: gaintype = mjGAIN_FIXED (default), biastype = mjBIAS_NONE (default)

// Add sensors (spec-level)
mjsSensor* sens = mjs_addSensor(spec);
sens->type = mjSENS_JOINTPOS;
sens->objtype = mjOBJ_JOINT;
mjs_setString(sens->objname, "joint_0");

// Compile
mjModel* model = mj_compile(spec, /*vfs=*/NULL);  // returns NULL on error
// ... use model ...

mj_deleteSpec(spec);
mj_deleteModel(model);
```

### 2.2 Key Observations for Our Project

1. **Actuators are NOT added to bodies.** They live at the `mjSpec` level. You reference their target joint/tendon/site by *name string*. This means our genotype-to-phenotype translator must:
   - First traverse the body tree, adding bodies/geoms/joints and recording joint names.
   - Then loop over all joints and add actuators referencing them by name.
   - Same for sensors.

2. **String properties use `mjs_setString()`** (opaque `mjString*`). In C++ mode, `mjString` is literally `std::string`, so you can also directly assign: `*torso->name = "torso";` — but `mjs_setString` is the safe C-compatible way.

3. **The `fromto` field on `mjsGeom`** is an alternative to specifying `pos + quat + size[1]` for capsules/cylinders. It defines two endpoints: `fromto[0..2]` and `fromto[3..5]`. MuJoCo computes the length and orientation automatically. This is convenient for our capsule-based morphologies.

4. **`mj_compile()` can be called multiple times** on the same spec. Each call returns a new `mjModel*`. The spec remains editable. This matters if we want to modify and recompile.

5. **`mj_recompile(spec, vfs, model, data)`** does in-place recompilation — returns -1 if sizes changed (need fresh `mj_compile` + `mj_makeData` instead). Probably not useful for us since morphology mutations change body count.

6. **Defaults / classes:** `mjsDefault` provides per-class defaults for joints, geoms, actuators, etc. We could use a single default class for all our geoms to set common properties (contype, conaffinity, friction) without repeating them per-geom.

### 2.3 mjSpec vs. XML String — Performance Comparison

| Approach       | Disk I/O | XML Parsing | DOM Validation | Compile | Total for 1000 evals |
|----------------|----------|-------------|----------------|---------|----------------------|
| XML string + VFS | None (memory) | TinyXML parse | Yes | Yes | Medium |
| `mjSpec` API   | None     | None        | None           | Yes     | **Fastest**          |

**Recommendation for planning TODO 4:** Start with XML string generation (simpler to debug, human-readable output). Add mjSpec path as the "stretch goal" optimization. Both are valid.

### 2.4 Saving mjSpec to XML

```c++
// Save spec to file (useful for debugging / visualization)
char err[1000];
mj_saveXML(spec, "output.xml", err, sizeof(err));

// Or to a string buffer
char xml_buf[100000];
mj_saveXMLString(spec, xml_buf, sizeof(xml_buf), err, sizeof(err));
```

Also: `mj_saveLastXML(filename, model, err, err_sz)` saves the XML from the *last loaded model* — works even if the model was compiled from mjSpec.


---

## 3. Simulation Pipeline

### 3.1 The `mj_step()` Pipeline

```
mj_step(m, d):
  mj_step1(m, d):        // position + velocity dependent computations
    mj_fwdPosition(m,d)  // kinematics, CoM, tendon, transmission, crb, factorM
    mj_fwdVelocity(m,d)  // com velocities, passive forces (spring-damper, gravity comp)
    mj_fwdActuation(m,d) // actuator forces: gain * ctrl + bias → qfrc_actuator
    mj_fwdAcceleration(m,d)  // sum all non-constraint forces → qacc_smooth
    mj_fwdConstraint(m,d)    // collision detection → contact constraints → solve

  mj_step2(m, d):        // integration
    mj_Euler / mj_RungeKutta / mj_implicit  // integrate (time, qpos, qvel, act)
    mj_sensorPos / mj_sensorVel / mj_sensorAcc   // evaluate sensors
    mj_energyPos / mj_energyVel  // update energy terms
```

**IMPORTANT:** The `step1/step2` split is documented to give control between collision/constraint solving and integration, BUT:
- `step1/step2` only works correctly with the **Euler** integrator.
- For RK4 or implicit integrators, use `mj_step()` only.

For our controller loop, the pattern is:
```c++
while (d->time < T) {
    // Read sensors (computed at end of PREVIOUS step, or at init via mj_forward)
    // Neural forward pass → d->ctrl
    mj_step(m, d);   // physics + sensors
}
```

Or with the explicit split (Euler only):
```c++
mj_step1(m, d);   // compute everything up to constraints
// Now d->sensordata is STALE (from previous step)
// But d->qpos, d->qvel are current
// Set d->ctrl here
mj_step2(m, d);   // integrate + compute sensors
```

**Recommendation:** Use `mj_step()` with Euler integrator. The step1/step2 split adds no benefit for our use case since we read `sensordata` (computed at end of prev step) → set `ctrl` → call `mj_step()`.

### 3.2 State and Control Vectors

**State:** `(d->time, d->qpos[nq], d->qvel[nv], d->act[na])`
- For our project: `na = 0` (no actuator activation dynamics — we use simple motors with `mjDYN_NONE`).
- So effective state = `(time, qpos, qvel)`.

**Control:** `d->ctrl[nu]` — the only input we set from the neural network.
- Other inputs (`d->qfrc_applied[nv]`, `d->xfrc_applied[nbody×6]`, `d->mocap_pos/quat`) exist but we won't use them.

### 3.3 Sensor Data

Sensors are defined in the model. Each sensor produces 1 or more scalar values packed into `d->sensordata[nsensordata]`.

| Sensor Type      | `mjtSensor` enum | Scalars | Object Type Required |
|------------------|-------------------|---------|---------------------|
| Joint position   | `mjSENS_JOINTPOS` | 1       | `mjOBJ_JOINT`       |
| Joint velocity   | `mjSENS_JOINTVEL` | 1       | `mjOBJ_JOINT`       |
| Frame position   | `mjSENS_FRAMEPOS` | 3       | `mjOBJ_BODY/SITE`   |
| Frame quaternion | `mjSENS_FRAMEQUAT`| 4       | `mjOBJ_BODY/SITE`   |
| Frame lin. vel.  | `mjSENS_FRAMELINVEL`| 3     | `mjOBJ_BODY/SITE`   |
| Frame ang. vel.  | `mjSENS_FRAMEANGVEL`| 3     | `mjOBJ_BODY/SITE`   |
| Accelerometer    | `mjSENS_ACCELEROMETER`| 3   | `mjOBJ_SITE`        |
| Gyro             | `mjSENS_GYRO`    | 3       | `mjOBJ_SITE`        |
| Touch            | `mjSENS_TOUCH`   | 1       | `mjOBJ_SITE`        |
| Actuator pos     | `mjSENS_ACTUATORPOS`| 1     | `mjOBJ_ACTUATOR`    |
| Actuator vel     | `mjSENS_ACTUATORVEL`| 1     | `mjOBJ_ACTUATOR`    |
| Actuator force   | `mjSENS_ACTUATORFRC`| 1     | `mjOBJ_ACTUATOR`    |
| Subtree CoM      | `mjSENS_SUBTREECOM`| 3      | `mjOBJ_BODY`        |

**For planning TODO 4 (sensor block in MJCF):**
- Minimum: `jointpos` + `jointvel` per joint gives `2 × njnt` scalars.
- Root proprioception: `framequat` (4) + `framelinvel` (3) + `frameangvel` (3) on root body = 10 extra.
- So `nsensordata ≈ 2 × njnt + 10` for a basic configuration.

**Sensor noise:** Controlled by `sensor_noise[i]` in `mjModel`. When noise > 0, MuJoCo adds Gaussian noise using `rand()`. **The research.md correctly notes that `srand()` must be called AFTER `mj_loadXML`/`mj_compile`** because the compiler internally calls `srand(123)` for procedural textures.

### 3.4 Determinism

MuJoCo is fully deterministic given the same:
- `mjModel` (identical physics params)
- `mjData` initial state
- Control sequence
- Same MuJoCo version and build

No global mutable state is touched during `mj_step()` (confirmed by thread-safety design). Floating-point results are reproducible across runs on the same platform.

### 3.5 Multi-Threading Model

From `doc/programming/simulation.rst`:

> "The main consideration for multi-threading is that the model and all constant computations derived from it are shared. Changes in mjModel fields are not allowed by worker threads."

**Thread-safe pattern:**
```
Main thread:
  mjModel* m = mj_loadXML(...);   // or mj_compile(spec, ...)

Worker threads (each):
  mjData* d = mj_makeData(m);     // thread-local data
  // ... run simulation loop on (m, d) ...
  mj_deleteData(d);

Main thread:
  mj_deleteModel(m);
```

**For our thread pool (TODO 11):**
- Workers with IDENTICAL morphologies can share a single `const mjModel*` + each creates own `mjData`.
- Workers with DIFFERENT morphologies must each compile their own `mjModel` in thread-local scope.
- `mj_loadXML` / `mj_compile` themselves are **NOT documented as thread-safe**. The safest approach: compile models one at a time (e.g., in the master thread, or protected by a mutex), then distribute the `const mjModel*` to workers. OR compile in workers but with a mutex around `mj_compile`.

**CORRECTION to research.md:** The research says "executing completely distinct mjModel and mjData instances in separate, isolated threads is entirely thread-safe." This is true for `mj_step()` and all simulation functions. But the research does NOT distinguish compilation thread safety. We should compile under a mutex or in the master thread.

**Model caching strategy (TODO 12):** Compute a hash of the `BodyGene` tree → check if an `mjModel*` already exists → if yes, skip compilation, just `mj_makeData(cached_model)`. Use `std::shared_ptr<const mjModel>` with custom deleter.


---

## 4. Collision Detection In Depth

### 4.1 The Collision Function Table

Found in `src/engine/engine_collision_driver.c`:

```c
mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES] = {
/*              PLANE  HFIELD  SPHERE            CAPSULE             ELLIPSOID         CYLINDER            BOX               MESH              SDF */
/*PLANE     */ {0,     0,      mjc_PlaneSphere,  mjc_PlaneCapsule,   mjc_PlaneConvex,  mjc_PlaneCylinder,  mjc_PlaneBox,     mjc_PlaneConvex,  mjc_PlaneConvex},
/*HFIELD    */ {0,     0,      mjc_ConvexHField, mjc_ConvexHField,   mjc_ConvexHField, mjc_ConvexHField,   mjc_ConvexHField, mjc_ConvexHField, mjc_HFieldSDF},
/*SPHERE    */ {0,     0,      mjc_SphereSphere, mjc_SphereCapsule,  mjc_Convex,       mjc_SphereCylinder, mjc_SphereBox,    mjc_Convex,       mjc_SDF},
/*CAPSULE   */ {0,     0,      0,                mjc_CapsuleCapsule, mjc_Convex,       mjc_Convex,         mjc_CapsuleBox,   mjc_Convex,       mjc_SDF},
/*ELLIPSOID */ {0,     0,      0,                0,                  mjc_Convex,       mjc_Convex,         mjc_Convex,       mjc_Convex,       mjc_SDF},
/*CYLINDER  */ {0,     0,      0,                0,                  0,                mjc_Convex,         mjc_Convex,       mjc_Convex,       mjc_SDF},
/*BOX       */ {0,     0,      0,                0,                  0,                0,                  mjc_BoxBox,       mjc_Convex,       mjc_SDF},
/*MESH      */ {0,     0,      0,                0,                  0,                0,                  0,                mjc_Convex,       mjc_MeshSDF},
/*SDF       */ {0,     0,      0,                0,                  0,                0,                  0,                0,                mjc_SDF}
};
```

### 4.2 Specialized vs Generic Functions

| Collision pair         | Function              | Type        |
|------------------------|-----------------------|-------------|
| Sphere–Sphere          | `mjc_SphereSphere`    | **Specialized** (trivial distance check) |
| Sphere–Capsule         | `mjc_SphereCapsule`   | **Specialized** (point-to-segment) |
| Capsule–Capsule        | `mjc_CapsuleCapsule`  | **Specialized** (segment-to-segment) |
| Sphere–Box             | `mjc_SphereBox`       | **Specialized** |
| Capsule–Box            | `mjc_CapsuleBox`      | **Specialized** |
| Box–Box                | `mjc_BoxBox`          | **Specialized** |
| Plane–Sphere/Capsule/Cyl/Box | `mjc_Plane*`    | **Specialized** |
| Sphere–Cylinder        | `mjc_SphereCylinder`  | **Specialized** |
| Plane–Cylinder         | `mjc_PlaneCylinder`   | **Specialized** |
| Ellipsoid–anything     | `mjc_Convex`          | **Generic GJK/EPA** |
| Cylinder–Cylinder      | `mjc_Convex`          | Generic |
| Cylinder–Box           | `mjc_Convex`          | Generic |
| Mesh–any non-plane     | `mjc_Convex`          | Generic |
| SDF–anything           | `mjc_SDF`             | SDF-specific |

### 4.3 Implications for Our Geom Selection (TODO 3–4)

**research.md claim CONFIRMED:** "the system triggers MuJoCo's specialized, significantly cheaper primitive-primitive collision algorithms" when using primitives.

**Optimal primitive set for our morphologies:**
- **CAPSULE** — best all-around. Capsule–Capsule and Capsule–Plane are both specialized. Capsules have only 2 size params (radius, half-length), which is great for a compact genotype. The `fromto` field makes placement intuitive.
- **SPHERE** — useful for joint knobs or feet. All sphere pairs are specialized.
- **BOX** — useful for flat/blocky segments. Box–Box is specialized. Box–Capsule is specialized.

**Avoid:** Ellipsoids (all pairs fall to `mjc_Convex`), Cylinders (Cyl-Cyl and Cyl-Box both fall to `mjc_Convex`), Meshes and SDFs.

### 4.4 Collision Filtering

```c
// Filter based on bitmask: contact only if (contype1 & conaffinity2) || (contype2 & conaffinity1)
static int filterBitmask(int contype1, int conaffinity1, int contype2, int conaffinity2) {
  return !(contype1 & conaffinity2) && !(contype2 & conaffinity1);
}
```

This means we can prevent self-collision between adjacent body segments by setting their `contype` / `conaffinity` bits carefully. For example, parent-child pairs could share the same contype bit but have conaffinity clear for that bit.

Alternatively, use `<exclude body1="..." body2="..."/>` in MJCF (or `mjsExclude` in mjSpec) to explicitly exclude collision pairs.

**For our GA:** self-collision filtering is critical in early generations where random morphologies generate tangled, overlapping geometries. Without filtering, the solver will waste time on thousands of self-contacts.


---

## 5. Actuator System Details

### 5.1 Actuator Types

For our evolution project, we want the simplest motor model:

```
force = gain * ctrl + bias
```

- **`gaintype = mjGAIN_FIXED`** (default): `gain = gainprm[0]` (a constant).
- **`biastype = mjBIAS_NONE`** (default): `bias = 0`.
- **`dyntype = mjDYN_NONE`** (default): no internal dynamics, `ctrl` directly used.
- **`trntype = mjTRN_JOINT`**: force applied to a single joint.

So for a basic motor: `qfrc_actuator[dof] = gainprm[0] * ctrl[i]`.

With `ctrl ∈ [-1, 1]` (from tanh network) and `gainprm[0] = max_torque`, the force is bounded to `[-max_torque, max_torque]`.

### 5.2 Control Clamping

Can also enforce control limits explicitly:
```c
actuator->ctrllimited = mjLIMITED_TRUE;
actuator->ctrlrange[0] = -1.0;
actuator->ctrlrange[1] = 1.0;
```
This clamps `ctrl[i]` to `[-1, 1]` inside MuJoCo, providing a safety net even if the network outputs extreme values. Since `tanh` already bounds to `[-1, 1]`, this is belt-and-suspenders.

### 5.3 The `mjcb_act_bias` and `mjcb_act_gain` Callbacks

**Found in `mujoco.h` line 60–61:**
```c
MJAPI extern mjfAct mjcb_act_dyn;
MJAPI extern mjfAct mjcb_act_gain;
MJAPI extern mjfAct mjcb_act_bias;
```

**Callback signature:** `mjtNum (*mjfAct)(const mjModel* m, const mjData* d, int id);`

These are called per-actuator during `mj_fwdActuation()` when `gaintype == mjGAIN_USER` or `biastype == mjBIAS_USER`. The `id` parameter is the actuator index.

**research.md claim:** "By assigning a custom C++ function to the global `mjcb_act_bias` callback pointer, a neural network can bypass MuJoCo's standard motor torque calculations."

**CORRECTION/CLARIFICATION:** This callback is only invoked for actuators with `biastype = mjBIAS_USER` (not for all actuators). You can't use it to "bypass" standard motors — you use it to *implement* custom ones. For our project, the simple `d->ctrl` approach is correct and sufficient. `mjcb_act_bias` is an advanced feature we don't need.

### 5.4 Other Useful Callbacks

| Callback | Signature | When Called | Use Case |
|----------|-----------|-------------|----------|
| `mjcb_control` | `void(const mjModel*, mjData*)` | Inside `mj_step1`, after forces computed, before constraints | **Alternative control injection point** — set `d->ctrl` here instead of in the outer loop |
| `mjcb_passive` | `void(const mjModel*, mjData*)` | Inside `mj_passive()` | Add custom passive forces (e.g. joint friction) |
| `mjcb_sensor` | `void(const mjModel*, mjData*, int stage)` | During sensor evaluation | Compute custom sensor values |
| `mjcb_contactfilter` | `int(const mjModel*, mjData*, int geom1, int geom2)` | During collision detection | Dynamically filter contacts at runtime |

**WARNING:** These are **global** function pointers. In a multithreaded setup, they cannot vary per-thread. If we use `mjcb_control`, all threads share the same callback. This is fine if the callback reads per-`mjData` state, but the callback can't hold per-individual closure state. For our project, the explicit loop approach (read sensors → neural forward → set ctrl → mj_step) is safer and clearer.


---

## 6. Joint Types and Their Dimensions

| Type | `mjtJoint` enum | qpos dim | qvel dim | Notes |
|------|-----------------|----------|----------|-------|
| Free | `mjJNT_FREE` | 7 (pos3 + quat4) | 6 (vel3 + angvel3) | Root body floating in space |
| Ball | `mjJNT_BALL` | 4 (quaternion) | 3 (angular velocity) | 3-DOF rotation |
| Slide | `mjJNT_SLIDE` | 1 | 1 | Translation along axis |
| Hinge | `mjJNT_HINGE` | 1 | 1 | Rotation around axis |

**IMPORTANT:** `nq ≠ nv` when the model contains free or ball joints!
- Hinge: nq contribution = 1, nv contribution = 1
- Slide: nq contribution = 1, nv contribution = 1
- Ball: nq contribution = 4, nv contribution = 3
- Free: nq contribution = 7, nv contribution = 6

**For planning TODO 3:** Start with `HINGE` only (as recommended). If we later add `FREE` joint as the root (for a floating robot), the root alone adds 7 to nq and 6 to nv, but we likely won't motorize it (no actuator on free joints).

### 6.1 Joint Properties Relevant to Evolution

From `mjsJoint`:
```c
int type;             // mjtJoint: HINGE, SLIDE, BALL, FREE
double axis[3];       // rotation/slide axis (for hinge and slide)
double range[2];      // joint limits [min, max] (radians for hinge)
int limited;          // mjtLimited: FALSE, TRUE, AUTO
double stiffness;     // spring stiffness
double damping;       // viscous damping
double armature;      // rotor inertia (stabilizes simulation)
double frictionloss;  // dry friction
```

**Key insight for `JointGene`:** We should evolve at minimum `axis`, `range`, and potentially `armature` and `damping`. The `armature` parameter is especially important — it adds diagonal inertia to the mass matrix, making the system more numerically stable. MuJoCo's own examples typically use `armature="0.01"` or similar small values.

**The `damping` field creates passive viscous forces** proportional to joint velocity. This acts as a natural stabilizer and is critical for preventing chaotic oscillations in randomly-evolved morphologies. Consider setting a small default damping (e.g., 0.1) via a default class.


---

## 7. Body and Geom Details

### 7.1 `mjsBody` Fields for Evolution

```c
double pos[3];       // position relative to parent body frame
double quat[4];      // orientation relative to parent
double mass;         // body mass (overridden by geom if inertiafromgeom=true)
double inertia[3];   // diagonal inertia (auto-computed from geoms if inertiafromgeom=auto)
```

**Inertia inference:** By default (`inertiafromgeom = auto`), MuJoCo computes body inertia from child geoms. This means we do NOT need to manually specify mass/inertia in `mjsBody` — just specify `geom->density` or `geom->mass` and let MuJoCo handle it.

**For `BodyGene`:** we can either:
- Set `geom->density` (uniform), let mass = density × volume (auto-computed from geom shape).
- Set `geom->mass` directly, which overrides density.
- Either way, body-level mass/inertia is auto-computed from all child geoms.

### 7.2 `mjsGeom` Fields for Evolution

```c
int type;             // mjtGeom: CAPSULE, SPHERE, BOX
double size[3];       // type-dependent: [radius] for sphere, [radius, half-len] for capsule, [x,y,z half-sizes] for box
double fromto[6];     // alternative to pos+quat for capsule/cylinder (two endpoints)
double pos[3];        // position in body frame
double quat[4];       // orientation in body frame
double mass;          // mass (0: use density)
double density;       // density (default 1000)
int contype;          // collision type bitmask (default 1)
int conaffinity;      // collision affinity bitmask (default 1)
int condim;           // contact dimensionality: 1 (frictionless), 3, 4, or 6
double friction[3];   // slide, roll, spin friction coefficients
float rgba[4];        // color for visualization
```

**Capsule geometry:**
- `size[0]` = radius
- `size[1]` = half-length (of the cylinder part, total length = 2*size[1] + 2*size[0])
- OR use `fromto[6]` to specify two endpoints; MuJoCo auto-computes size[1] and orientation.

**For `BodyGene.dimensions`:** For capsules, store `[radius, half_length]`. For spheres, `[radius]`. For boxes, `[hx, hy, hz]`.


---

## 8. The mjOption (Physics Settings)

From `mjSpec.option` (type `mjOption`):

```c
mjtNum timestep;        // simulation timestep (default 0.002 = 2ms)
mjtNum gravity[3];      // gravitational acceleration (default [0, 0, -9.81])
int integrator;         // mjtIntegrator: EULER (0), RK4 (1), IMPLICIT (2), IMPLICITFAST (3)
int solver;             // mjtSolver: PGS (0), CG (1), NEWTON (2)
int iterations;         // max solver iterations (default 100)
int cone;               // cone type: PYRAMIDAL (0), ELLIPTIC (1)
```

**Recommended defaults for our project:**
- `timestep = 0.002` (500 Hz — standard for locomotion)
- `integrator = mjINT_EULER` (simplest, fastest, sufficient for our needs; allows step1/step2 split if desired)
- `solver = mjSOL_NEWTON` (most accurate for contact-rich scenarios)
- `gravity = [0, 0, -9.81]`
- `iterations = 100` (can reduce to 50 for speed if contact quality is acceptable)

These can be set programmatically:
```c++
spec->option.timestep = 0.002;
spec->option.integrator = mjINT_EULER;
spec->option.gravity[2] = -9.81;
```


---

## 9. Cross-Reference: Research Assumptions vs. Reality

### 9.1 Confirmed Claims

| Research Claim | Verified? | Source |
|---------------|-----------|--------|
| MuJoCo uses generalized coordinates | ✅ Yes | `mjModel`: nq, nv, qpos, qvel |
| mjModel is const during simulation | ✅ Yes | All stepping functions take `const mjModel*` |
| mjData contains time-varying state | ✅ Yes | `mjData`: qpos, qvel, ctrl, sensordata |
| Primitive collision is cheaper than generic | ✅ Yes | Collision function table shows specialized functions |
| `mj_loadXML` + VFS avoids disk I/O | ✅ Yes | VFS API: `mj_addBufferVFS` |
| `nsensordata` sizes the network input | ✅ Yes | `mjModel.nsensordata` = total scalar readings |
| `nu` sizes the network output | ✅ Yes | `mjModel.nu` = number of actuators |
| Per-thread mjData is safe | ✅ Yes | Documented in simulation.rst |
| `srand()` needed after compile for deterministic noise | ✅ Yes | `srand(123)` called internally by compiler |
| `mjcb_act_bias` exists for custom actuators | ✅ Yes | Global callback, `mjfAct` signature |

### 9.2 Corrections / Clarifications

| Research Claim | Correction |
|---------------|-----------|
| "mjcb_act_bias can bypass standard motor torque" | Only invoked for `biastype = mjBIAS_USER` actuators. Not a bypass — it's the implementation mechanism for custom actuator types. For simple motors, just use `d->ctrl`. |
| `find_package(mujoco 2.3.7 REQUIRED)` | We build from source with `add_subdirectory`, not `find_package`. Also, API version is 3.2.7 in our build. |
| "DAG" representation for morphology | MuJoCo strictly requires a **kinematic tree** (each body has exactly one parent). The genotype may be a DAG conceptually, but the phenotype MUST be a tree. research.md does mention this constraint. |
| Thread safety for compilation | `mj_loadXML` and `mj_compile` are NOT explicitly documented as thread-safe. Simulation functions (`mj_step` etc.) are safe with isolated data. Compilation should be serialized or mutexed. |
| "mj_step1/mj_step2 split" for control | Only valid with Euler integrator. The research doesn't mention this constraint. |

### 9.3 Gaps in Research

| Topic | What's Missing |
|-------|---------------|
| `mjSpec` API | research.md mentions it briefly but doesn't detail the API. Now documented above. |
| `armature` parameter | Not mentioned in research, but critical for simulation stability. |
| `damping` parameter | Not mentioned but important for preventing chaotic oscillations. |
| `condim` (contact dimensionality) | Not discussed. Default is 3 (sliding friction in 2D tangent plane). Setting condim=1 gives frictionless contacts (faster). |
| Compiler options (`boundmass`, `boundinertia`) | Not mentioned. Setting `compiler.boundmass` to a small value (e.g., 0.01) prevents zero-mass bodies from crashing the solver. |
| `mj_copyModel` for model caching | research.md mentions caching but not the specific API. `mj_copyModel(NULL, src)` creates a deep copy. |
| `mj_saveLastXML` | Useful for debugging — save any compiled model back to XML for inspection. |


---

## 10. Practical Recipes for Our Implementation

### 10.1 Recipe: Build a Simple 2-Segment Creature via mjSpec

```c++
mjSpec* spec = mj_makeSpec();
spec->option.timestep = 0.002;
spec->option.integrator = mjINT_EULER;
spec->compiler.inertiafromgeom = mjINERTIAFROMGEOM_TRUE;
spec->compiler.boundmass = 0.001;         // prevent zero-mass bodies
spec->compiler.boundinertia = 0.001;      // prevent zero-inertia bodies

mjsBody* world = mjs_findBody(spec, "world");

// Ground plane
mjsGeom* ground = mjs_addGeom(world, 0);
ground->type = mjGEOM_PLANE;
ground->size[0] = 10; ground->size[1] = 10; ground->size[2] = 0.01;
mjs_setString(ground->name, "ground");

// Torso (free-floating root)
mjsBody* torso = mjs_addBody(world, 0);
mjs_setString(torso->name, "torso");
torso->pos[2] = 0.5;

mjsJoint* root_jnt = mjs_addJoint(torso, 0);
root_jnt->type = mjJNT_FREE;
mjs_setString(root_jnt->name, "root");

mjsGeom* torso_geom = mjs_addGeom(torso, 0);
torso_geom->type = mjGEOM_CAPSULE;
torso_geom->size[0] = 0.05;   // radius
torso_geom->size[1] = 0.15;   // half-length
torso_geom->density = 1000;

// Leg
mjsBody* leg = mjs_addBody(torso, 0);
mjs_setString(leg->name, "leg");
leg->pos[0] = 0; leg->pos[1] = 0; leg->pos[2] = -0.2;

mjsJoint* hip = mjs_addJoint(leg, 0);
hip->type = mjJNT_HINGE;
hip->axis[0] = 0; hip->axis[1] = 1; hip->axis[2] = 0;
hip->range[0] = -1.57; hip->range[1] = 1.57;
hip->limited = mjLIMITED_TRUE;
hip->damping = 0.1;
hip->armature = 0.01;
mjs_setString(hip->name, "hip");

mjsGeom* leg_geom = mjs_addGeom(leg, 0);
leg_geom->type = mjGEOM_CAPSULE;
leg_geom->size[0] = 0.04;
leg_geom->size[1] = 0.15;
leg_geom->density = 1000;

// Actuator for hip
mjsActuator* motor = mjs_addActuator(spec, 0);
mjs_setString(motor->name, "hip_motor");
mjs_setString(motor->target, "hip");
motor->trntype = mjTRN_JOINT;
motor->gainprm[0] = 50.0;   // max torque = 50 Nm
motor->ctrllimited = mjLIMITED_TRUE;
motor->ctrlrange[0] = -1.0;
motor->ctrlrange[1] = 1.0;

// Sensors
mjsSensor* s1 = mjs_addSensor(spec);
s1->type = mjSENS_JOINTPOS;
s1->objtype = mjOBJ_JOINT;
mjs_setString(s1->objname, "hip");
mjs_setString(s1->name, "hip_pos");

mjsSensor* s2 = mjs_addSensor(spec);
s2->type = mjSENS_JOINTVEL;
s2->objtype = mjOBJ_JOINT;
mjs_setString(s2->objname, "hip");
mjs_setString(s2->name, "hip_vel");

// Compile
mjModel* m = mj_compile(spec, NULL);
assert(m != NULL);
// m->nu == 1, m->nsensordata == 2 (1 jointpos + 1 jointvel)

mjData* d = mj_makeData(m);
// ... run simulation ...
mj_deleteData(d);
mj_deleteModel(m);
mj_deleteSpec(spec);
```

### 10.2 Recipe: XML String Equivalent

```xml
<mujoco>
  <option timestep="0.002" integrator="Euler"/>
  <compiler inertiafromgeom="true" boundmass="0.001" boundinertia="0.001"/>

  <worldbody>
    <geom name="ground" type="plane" size="10 10 0.01"/>
    <body name="torso" pos="0 0 0.5">
      <freejoint name="root"/>
      <geom name="torso_g" type="capsule" size="0.05 0.15" density="1000"/>
      <body name="leg" pos="0 0 -0.2">
        <joint name="hip" type="hinge" axis="0 1 0"
               range="-90 90" limited="true" damping="0.1" armature="0.01"/>
        <geom name="leg_g" type="capsule" size="0.04 0.15" density="1000"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="hip_motor" joint="hip" gear="50" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>

  <sensor>
    <jointpos name="hip_pos" joint="hip"/>
    <jointvel name="hip_vel" joint="hip"/>
  </sensor>
</mujoco>
```

### 10.3 Recipe: Load XML from Memory (VFS)

```c++
std::string xml = gen_mjcf(body_root);

mjVFS vfs;
mj_defaultVFS(&vfs);
mj_addBufferVFS(&vfs, "model.xml", xml.data(), xml.size());

char err[1000];
mjModel* m = mj_loadXML("model.xml", &vfs, err, sizeof(err));
mj_deleteVFS(&vfs);

if (!m) {
    // Handle error: err contains the message
}
```

### 10.4 Recipe: Fitness Evaluation Loop

```c++
double evaluate(const BodyGene& body, const std::vector<double>& weights, int n_hidden) {
    std::string xml = gen_mjcf(body);

    // Load model
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    mj_addBufferVFS(&vfs, "m.xml", xml.data(), xml.size());
    char err[1000];
    mjModel* m = mj_loadXML("m.xml", &vfs, err, sizeof(err));
    mj_deleteVFS(&vfs);
    if (!m) return 0.0;  // invalid morphology

    mjData* d = mj_makeData(m);

    // Size the neural network
    int n_in = m->nsensordata;
    int n_out = m->nu;
    NeuralController brain(n_in, n_hidden, n_out);
    brain.set_weights(weights);  // may need resize logic

    // Record initial position
    double x0 = d->xpos[3*1 + 0];  // body 1 (torso) x position
    double y0 = d->xpos[3*1 + 1];

    // Run initial forward to populate sensors
    mj_forward(m, d);

    double energy = 0.0;
    double T = 10.0;  // seconds

    while (d->time < T) {
        // Read sensors
        std::vector<double> sensors(d->sensordata, d->sensordata + n_in);

        // Neural forward pass
        auto ctrl = brain.forward(sensors);

        // Set controls
        for (int i = 0; i < n_out; ++i) {
            d->ctrl[i] = ctrl[i];
            energy += ctrl[i] * ctrl[i];  // accumulate energy
        }

        mj_step(m, d);
    }

    // Compute fitness
    double xT = d->xpos[3*1 + 0];
    double yT = d->xpos[3*1 + 1];
    double dist = std::sqrt((xT-x0)*(xT-x0) + (yT-y0)*(yT-y0));

    double zT = d->xpos[3*1 + 2];
    double z0 = 0.3;  // minimum height threshold
    double fall_penalty = (zT < z0) ? 10.0 : 0.0;
    double energy_penalty = 0.001 * energy;

    double fitness = std::max(0.0, dist - energy_penalty - fall_penalty);

    mj_deleteData(d);
    mj_deleteModel(m);
    return fitness;
}
```


---

## 11. Important Constants and Limits

```c
#define mjNDYN      10    // max actuator dynamics parameters
#define mjNGAIN     10    // max actuator gain parameters
#define mjNBIAS     10    // max actuator bias parameters
#define mjNREF      2     // solver reference parameters
#define mjNIMP      5     // solver impedance parameters
#define mjNEQDATA   11    // equality constraint data size
#define mjNFLUID    12    // fluid interaction parameters per geom
```

These are fixed compile-time constants. Actuator `gainprm`, `biasprm`, `dynprm` are all arrays of size 10, even though most actuator types only use the first 1–3 entries.


---

## 12. API Quick Reference

### Creation / Deletion
| Function | Description |
|----------|-------------|
| `mj_makeSpec()` | Create empty model spec |
| `mj_parseXMLString(xml, vfs, err, err_sz)` | Parse XML string into spec |
| `mj_compile(spec, vfs)` → `mjModel*` | Compile spec to model |
| `mj_loadXML(file, vfs, err, err_sz)` → `mjModel*` | Load XML file/VFS to model |
| `mj_makeData(model)` → `mjData*` | Create simulation state |
| `mj_copyModel(NULL, src)` → `mjModel*` | Deep copy model |
| `mj_copyData(NULL, model, src)` → `mjData*` | Deep copy data |
| `mj_deleteSpec(spec)` | Free spec |
| `mj_deleteModel(model)` | Free model |
| `mj_deleteData(data)` | Free data |

### Spec Building
| Function | Description |
|----------|-------------|
| `mjs_findBody(spec, "world")` | Get world body |
| `mjs_addBody(parent, default)` | Add child body |
| `mjs_addGeom(body, default)` | Add geom to body |
| `mjs_addJoint(body, default)` | Add joint to body |
| `mjs_addSite(body, default)` | Add site to body |
| `mjs_addActuator(spec, default)` | Add actuator (spec-level!) |
| `mjs_addSensor(spec)` | Add sensor (spec-level!) |
| `mjs_setString(dest, text)` | Set string field |
| `mjs_delete(element)` | Delete any element |

### Simulation
| Function | Description |
|----------|-------------|
| `mj_step(m, d)` | Full simulation step |
| `mj_step1(m, d)` | Position/velocity/force computations (Euler only) |
| `mj_step2(m, d)` | Integration + sensors (Euler only) |
| `mj_forward(m, d)` | Forward dynamics without integration (useful for initial sensor read) |
| `mj_resetData(m, d)` | Reset data to defaults |

### Utility
| Function | Description |
|----------|-------------|
| `mj_name2id(m, type, name)` | Find object ID by name |
| `mj_id2name(m, type, id)` | Find object name by ID |
| `mj_saveLastXML(file, m, err, sz)` | Save model back to XML |
| `mj_saveXML(spec, file, err, sz)` | Save spec to XML |
| `mj_versionString()` | Get version string |

### VFS
| Function | Description |
|----------|-------------|
| `mj_defaultVFS(&vfs)` | Initialize VFS |
| `mj_addBufferVFS(&vfs, name, buf, sz)` | Add in-memory file |
| `mj_deleteVFS(&vfs)` | Free VFS |


---

## 13. Summary of Recommendations for Each Planning TODO

| TODO | Key MuJoCo Insight |
|------|--------------------|
| **3. Morphology Genotype** | Use `CAPSULE` as default geom. Include `armature` and `damping` in `JointGene`. Consider `density` instead of explicit mass. Enforce tree structure (one parent per body). |
| **4. MJCF Translator** | Start with XML string + VFS route. Add `<compiler inertiafromgeom="true" boundmass="0.001"/>`. Sensors go in `<sensor>` block OUTSIDE `<worldbody>`. Actuators likewise. Use `mjSpec` route as stretch optimization. |
| **5. Neural Network** | Input = `m->nsensordata`, Output = `m->nu`. Only known after model compilation. |
| **6. Sensor-Actuator Wiring** | Use `mj_forward(m, d)` once before the loop to populate initial `sensordata`. Then `read sensors → forward → set ctrl → mj_step` loop. |
| **7. Unified Genotype** | Weight vector size = `(nsensordata × n_hidden) + n_hidden + (n_hidden × nu) + nu`. Resize on morphology change. |
| **8. Fitness** | Use `d->xpos[3*body_id + ...]` for root body position. Call `mj_forward`before starting to get initial `xpos`. |
| **9. Evolutionary Operators** | After structural mutation, mark weights stale. Consider `<exclude>` pairs for adjacent bodies to manage self-collision. |
| **11. Thread Pool** | Compilation (`mj_compile`/`mj_loadXML`) should be serialized. Simulation is safe with per-thread `mjData`. |
| **12. Model Caching** | Hash `BodyGene` tree → `std::shared_ptr<const mjModel>` → workers call `mj_makeData(cached)`. Use `mj_copyModel` if needed. |
| **13. Visualization** | Use `mj_saveLastXML` to export best creature for viewing in MuJoCo's `simulate` app. |
