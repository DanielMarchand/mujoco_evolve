# MuJoCo Evolve — Project Planning

> **Naming convention:** Sub-steps are numbered as `<parent>.<sub>` (e.g., Step 2.1, Step 9a). Never use bare step numbers for sub-steps.

> **Documentation lifecycle:** As each TODO is completed, move its detailed breakdown out of this planning document and into the project's proper documentation (see TODO 14). Planning.md should converge toward a thin checklist; the docs site becomes the source of truth.

## TODO List

- [x] **1. Project Scaffolding & CMake Build System** — Set up the C++ project structure, CMake configuration, and verify MuJoCo links and runs.
- [x] **2. MuJoCo Sanity Check** — Load a hard-coded MJCF XML, run a basic simulation loop, and confirm physics stepping works.
- [ ] **3. Morphology Genotype Representation** — Implement the DAG-based genetic encoding for robot body structures (`BodyGene`, `JointGene`, kinematic tree).
- [ ] **4. Genotype-to-MJCF Translator** — Build the DFS traversal that converts a genotype graph into a valid MJCF XML string (or `mjSpec` calls).
- [ ] **5. Feedforward Neural Network** — Implement a lightweight, dynamically-sized FFNN in plain C++ with `tanh` activation.
- [ ] **6. Sensor-Actuator Wiring** — Wire up the neural network to read `mjData->sensordata` and write to `mjData->ctrl` inside the physics loop.
- [ ] **7. Unified Genotype & Fitness Evaluation Pipeline** — Create the container that holds both body graph + neural weights, and orchestrate the full decode → compile → simulate → score lifecycle.
- [ ] **8. Fitness Function** — Implement the distance-based objective with energy-expenditure and fall penalties.
- [ ] **9. Evolutionary Operators** — Implement mutation (morphological + neural) and crossover (sub-tree swap + weight mixing).
- [ ] **10. Genetic Algorithm Main Loop** — Wire up population initialization, selection, reproduction, and generational iteration.
- [ ] **11. Multithreaded Evaluation (Thread Pool)** — Implement the Master-Worker thread pool for parallel fitness evaluation with proper memory isolation.
- [ ] **12. Model Caching & Performance Optimization** — Cache compiled `mjModel` instances for identical morphologies to skip redundant XML parsing.
- [ ] **13. Logging, Serialization & Visualization** — Persist best genotypes, log generational statistics, and enable replay of evolved creatures.
- [ ] **14. Project Documentation Setup** — Set up Sphinx + Furo (same stack as MuJoCo) and present documentation options.

---

## Detailed Breakdown

### 1. Project Scaffolding & CMake Build System

**Goal:** Establish a clean, modular C++ project that compiles against MuJoCo.

**Details:**

- Create a directory layout separating concerns:
  ```
  mujoco_evolve/
  ├── CMakeLists.txt
  ├── src/
  │   ├── main.cpp
  │   ├── ga/           # Genetic algorithm logic
  │   ├── morphology/   # Genotype graph, MJCF generation
  │   ├── brain/        # Neural network implementation
  │   ├── sim/          # MuJoCo wrappers, evaluation
  │   └── utils/        # Threading, RNG, logging
  ├── include/          # Public headers mirroring src/
  ├── tests/            # Unit / integration tests
  └── agent_notes/      # Research & planning docs
  ```
- CMake must target **C++17** (`CMAKE_CXX_STANDARD 17`) for `std::thread`, `std::mutex`, `std::optional`, structured bindings, and `<random>`.
- MuJoCo 3.2.7 is built from source via `add_subdirectory(third_party/mujoco)` and linked via the `mujoco` CMake target. Compiler warnings (`-Wall -Wextra -Wpedantic`) are applied only to our own targets via `target_compile_options(... PRIVATE ...)` to avoid triggering warnings inside MuJoCo's C sources.
- Set compiler optimisation to `-O3` for release builds.
- Confirm the build produces a running executable that prints the MuJoCo version string.

**Acceptance criteria:** `cmake --build build && ./build/evo` runs without linker errors and prints `MuJoCo version: 3.2.7`. ✅ Done.

---

### 2. MuJoCo Sanity Check

**Goal:** Prove end-to-end that we can load a model, step physics, and read state.

**Details:**

This TODO is covered by the GoogleTest suite in `tests/test_mujoco.cpp`. Below is a walkthrough of what each test does and why.

#### Step 2.1 — Define a minimal MJCF model as a string constant

```cpp
static constexpr char kMinimalXml[] = R"(
<mujoco>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>          <!-- light so rendering works -->
    <geom type="plane" size="5 5 0.1"/>         <!-- infinite ground plane -->
    <body name="capsule" pos="0 0 1">           <!-- a body floating 1m up -->
      <joint name="slide" type="slide" axis="0 0 1"/>   <!-- can slide vertically -->
      <geom type="capsule" size="0.1" fromto="0 0 0 0 0 0.5" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="slide" gear="100"/>           <!-- motor that pushes the joint -->
  </actuator>
  <sensor>
    <jointpos joint="slide"/>   <!-- reads joint position (1 scalar) -->
    <jointvel joint="slide"/>   <!-- reads joint velocity (1 scalar) -->
  </sensor>
</mujoco>
)";
```

This gives us the simplest possible physics scenario: a capsule that can slide up and down, with one motor to push it and two sensors to read position/velocity. We use it across all the tests below.

#### Step 2.2 — Load the model from memory (test: `LoadModelFromString`)

MuJoCo normally loads XML from disk, but we can use a **Virtual File System (VFS)** to load from a string in memory — handy for tests and for our GA where we'll generate MJCF dynamically.

```cpp
mjVFS vfs;
mj_defaultVFS(&vfs);                                          // initialize VFS
mj_addBufferVFS(&vfs, "test.xml", kMinimalXml, strlen(kMinimalXml));  // register our string as a "file"

char error[1000] = "";
mjModel* m = mj_loadXML("test.xml", &vfs, error, sizeof(error));  // parse + compile
// m is now a compiled physics model — m->njnt==1, m->nu==1, m->nsensordata==2
```

`mj_loadXML` parses the XML, validates it, and produces an `mjModel` — a read-only struct containing all the physics parameters (masses, geometries, joint axes, etc.). If parsing fails, `m` is `nullptr` and `error` describes the problem.

#### Step 2.3 — Step physics and observe state changes (test: `StepSimulation`)

```cpp
mjData* d = mj_makeData(m);     // allocate simulation state (positions, velocities, forces)
double z0 = d->qpos[0];         // initial position of the slide joint

for (int i = 0; i < 500; ++i) {
    mj_step(m, d);               // advance one timestep (default 0.002s)
}

double z1 = d->qpos[0];
// z1 < z0 — the capsule fell under gravity!
```

`mjData` is the mutable simulation state: joint positions (`qpos`), velocities (`qvel`), sensor readings (`sensordata`), actuator commands (`ctrl`), etc. Each call to `mj_step` advances the simulation by one timestep. With no motor input, gravity pulls the capsule down.

#### Step 2.4 — Verify determinism (test: `Determinism`)

MuJoCo's physics are fully deterministic — two runs from the same initial state always produce identical results. This is **not** a design requirement for our GA (evolution is stochastic by nature), but it's essential for writing reliable unit tests:

```cpp
double run1 = run_sim();   // run 1000 steps, return final qpos[0]
double run2 = run_sim();   // same 1000 steps from same initial state
EXPECT_DOUBLE_EQ(run1, run2);  // bit-identical
```

#### Step 2.5 — Clean up

```cpp
mj_deleteData(d);    // free simulation state
mj_deleteModel(m);   // free compiled model
mj_deleteVFS(&vfs);  // free virtual file system
```

Every `mj_makeData` must be paired with `mj_deleteData`, and every successful `mj_loadXML` with `mj_deleteModel`. Missing these causes memory leaks.

#### Step 2.6 — Memory checking options

To catch leaks automatically, we have two main tools:

| Tool | What it does | How to use | Trade-offs |
|------|-------------|------------|------------|
| **AddressSanitizer (ASan)** | Compiler-instrumented memory checker. Detects leaks, use-after-free, buffer overflows, stack overflows. | Add `-fsanitize=address` to compile **and** link flags in CMake. Run your binary normally — it prints errors to stderr at exit. | ~2× slower, ~3× more memory. No extra install needed (built into GCC/Clang). **Recommended as default.** |
| **Valgrind (memcheck)** | External tool that intercepts every memory operation at runtime. Detects leaks, uninitialized reads, invalid accesses. | Install: `sudo apt install valgrind`. Run: `valgrind --leak-check=full ./build/evo_tests`. | ~20× slower. More thorough for uninitialized memory. Useful as a secondary check. |

**Practical recommendation:** Enable ASan in a CMake debug preset (`-DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer"`) and run tests through it routinely. Use Valgrind only when investigating specific issues or before release milestones.

**Acceptance criteria:** Reproducible unit tests pass; clean memory under ASan. ✅ Done — covered by GoogleTest suite in `tests/test_mujoco.cpp` (VersionString, LoadModelFromString, StepSimulation, Determinism).

---

### 3. Morphology Genotype Representation

**Goal:** Define the C++ data structures that encode an evolvable robot body.

**Details:**

- `JointGene` struct:
  - `joint_type` — enum: `HINGE`, `BALL`, `SLIDE` (start with hinge only).
  - `axis` — `std::array<double,3>`, the local rotation axis.
  - `range` — `std::pair<double,double>`, joint limits in radians.
  - `gear` — `double`, actuator gear ratio (maps to `gainprm[0]` on the motor).
  - `armature` — `double`, rotor inertia added to the mass matrix diagonal. Critical for numerical stability; default ≈ 0.01. (See research_mujoco §6.1.)
  - `damping` — `double`, viscous damping proportional to joint velocity. Prevents chaotic oscillations in random morphologies; default ≈ 0.1.
- `BodyGene` struct:
  - `geom_type` — enum: `CAPSULE`, `SPHERE`, `BOX`. Prefer capsule — all capsule pairs use MuJoCo's specialized collision functions (see research_mujoco §4.2). **Avoid** ellipsoid and cylinder (fall to expensive generic GJK/EPA).
  - `dimensions` — `std::vector<double>` (for capsule: `[radius, half_length]`; sphere: `[radius]`; box: `[hx, hy, hz]`).
  - `density` — `double` (default 1000). Preferred over explicit mass — MuJoCo auto-computes mass and full inertia tensor from geom shape × density when `inertiafromgeom=true`.
  - `pos_offset` — `std::array<double,3>`, position relative to parent frame.
  - `children` — `std::vector<std::pair<JointGene, BodyGene>>`, ordered child attachments.
- The root `BodyGene` has no parent joint; it represents the torso.
- Enforce the **kinematic tree** invariant: every node has exactly one parent (no loops, no DAG fan-in). Validate on construction and after every mutation.
- Provide utility helpers: `tree_depth()`, `total_joints()`, `total_bodies()`, `flatten()` for iteration.

**Acceptance criteria:** Unit tests that build various tree topologies, verify invariants, and round-trip through serialization.

---

### 4. Genotype-to-MJCF Translator

**Goal:** Convert a `BodyGene` tree into a complete, valid MJCF XML string that MuJoCo can compile.

**Details:**

- Implement a recursive DFS function `std::string gen_mjcf(const BodyGene& root)` that:
  1. Opens `<mujoco>` with sensible `<option>` defaults (`timestep="0.002"`, `integrator="Euler"`, gravity default).
  2. Emits `<compiler inertiafromgeom="true" boundmass="0.001" boundinertia="0.001"/>` — `boundmass`/`boundinertia` prevent zero-mass bodies from crashing the solver on random morphologies.
  3. Emits a `<worldbody>` containing a ground plane `<geom>` and a `<light>`.
  4. Recursively emits `<body>`, `<geom>`, and `<joint>` tags for each node/edge in the tree. Includes `armature` and `damping` on each joint.
  5. After the worldbody, emits an `<actuator>` block with one `<motor>` per joint, referencing joint names. Each motor sets `gear="<JointGene.gear>"`, `ctrllimited="true"`, `ctrlrange="-1 1"` (belt-and-suspenders with tanh output).
  6. Emits a `<sensor>` block: `jointpos` + `jointvel` per joint, plus `framequat` + `framelinvel` + `frameangvel` on the root body. Expected `nsensordata ≈ 2 × njnt + 10`.
- **Collision filtering:** Add `<exclude body1="parent" body2="child"/>` pairs for directly-connected bodies to prevent solver waste on trivially overlapping parent-child geoms. Alternatively, manage `contype`/`conaffinity` bitmasks per-body (see research_mujoco §4.4). This is critical in early GA generations.
- Use **primitive geom types only** (capsule, sphere, box) — verified that all pairwise combinations use MuJoCo's specialized collision functions (research_mujoco §4.2).
- Name every body, joint, and actuator deterministically (e.g., `body_0`, `joint_0_1`, `motor_0_1`) so sensor/actuator ordering is predictable.
- Write integration tests: generate MJCF from a known tree, call `mj_loadXML`, assert `m->njnt`, `m->nu`, `m->nsensordata` match expectations.
- *Stretch:* Implement the same via the `mjSpec` programmatic API to bypass XML string overhead. Key difference: actuators and sensors are added at the `mjSpec` level (not inside bodies), referencing joints by name string. See recipe in research_mujoco §10.1.

**Acceptance criteria:** Any valid `BodyGene` tree compiles into an `mjModel` without errors; `mjModel` field counts match the tree structure.

---

### 5. Feedforward Neural Network

**Goal:** A minimal, header-friendly FFNN class with dynamic input/output sizing, no external dependencies.

**Details:**

- Class `NeuralController`:
  - Constructor takes `(N_in, N_hidden, N_out)`.
  - Internally allocates weight matrices as flat `std::vector<double>`:
    - `W1` of size `N_hidden × N_in`, `B1` of size `N_hidden`.
    - `W2` of size `N_out × N_hidden`, `B2` of size `N_out`.
  - Total parameter count: `(N_in × N_hidden) + N_hidden + (N_hidden × N_out) + N_out`.
  - `set_weights(const std::vector<double>& flat)` — unpacks a flat vector into W1, B1, W2, B2.
  - `get_weights() -> std::vector<double>` — packs back to a flat vector.
  - `forward(const std::vector<double>& input) -> std::vector<double>` — standard feedforward:
    ```
    H = tanh(W1 * input + B1)
    U = tanh(W2 * H + B2)
    ```
  - All matrix ops implemented manually (row-major dot products); no BLAS needed at this scale.
- `tanh` is used as the activation for both layers — outputs naturally in `[-1, 1]`, matching normalized actuator ranges.
- Unit tests: verify output dimensions, weight round-trip, and known hand-computed forward passes.

**Acceptance criteria:** `forward()` produces correct outputs for a hand-verified 2→3→1 network; weights survive pack/unpack.

---

### 6. Sensor-Actuator Wiring

**Goal:** Connect the FFNN to MuJoCo so the neural controller drives the simulated robot in real time.

**Details:**

- After compiling a genotype to an `mjModel`:
  - Read `m->nsensordata` → `N_in`. **Note:** use `nsensordata` (total scalar readings), NOT `nsensor` (number of sensor objects) — a single sensor like `framequat` produces 4 scalars.
  - Read `m->nu` → `N_out`.
  - Choose `N_hidden` from a global hyperparameter (e.g., 32).
  - Instantiate `NeuralController(N_in, N_hidden, N_out)` and load weights from the genotype.
- **Call `mj_forward(m, d)` once** before entering the loop to populate initial `sensordata` and `xpos` (otherwise the first control decision reads zeros).
- Inside the simulation loop:
  ```cpp
  mj_forward(m, d);  // populate initial sensors + positions
  while (d->time < T) {
      std::vector<double> sensors(d->sensordata, d->sensordata + m->nsensordata);
      auto ctrl = brain.forward(sensors);
      for (int i = 0; i < m->nu; ++i) d->ctrl[i] = ctrl[i];
      mj_step(m, d);  // integrates + recomputes sensors for next iteration
  }
  ```
- Verify by loading a known capsule-with-motor model, feeding constant sensor→control, and observing expected joint motion.

**Acceptance criteria:** A neural controller drives a compiled model through a full rollout without crashes; actuator signals stay in `[-1, 1]`.

---

### 7. Unified Genotype & Fitness Evaluation Pipeline

**Goal:** A single `Genotype` struct that owns both body and brain, plus a function `evaluate(Genotype&) -> double` that orchestrates the entire lifecycle.

**Details:**

- `Genotype` struct:
  - `BodyGene body_root` — the morphological tree.
  - `std::vector<double> neural_weights` — flat parameter vector for the FFNN.
  - `double fitness` — computed score, default `−∞`.
  - `int n_hidden` — hidden layer size (global constant or per-individual).
- `evaluate()` function:
  1. `std::string xml = gen_mjcf(body_root);`
  2. `mjModel* m = mj_loadXML(...)` — if compile fails (invalid body), assign fitness = 0 and return early.
  3. Compute required weight count: `(m->nsensordata * n_hidden) + n_hidden + (n_hidden * m->nu) + m->nu`.
  4. If `neural_weights.size()` doesn't match, resize and randomly initialize the mismatched portion (preserving existing weights where possible).
  5. Instantiate `NeuralController`, run the simulation loop, compute fitness.
  6. `mj_deleteData(d); mj_deleteModel(m);` — clean up.
  7. Store computed fitness in the genotype.

**Acceptance criteria:** `evaluate()` runs on a random genotype, produces a finite fitness value, and leaks no memory.

---

### 8. Fitness Function

**Goal:** A scalar objective that rewards forward locomotion while penalizing exploits.

**Details:**

- **Base fitness** — Euclidean displacement of the root body in the XY plane:
  $$F_{base} = \sqrt{(\Delta x)^2 + (\Delta y)^2}$$
  Read from `d->xpos[3*root_body_id + 0]` (x) and `d->xpos[3*root_body_id + 1]` (y). The root body (torso) is typically body index 1 (body 0 is the world). Call `mj_forward(m, d)` before the loop to populate initial `xpos`.
- **Energy penalty** — Accumulated squared actuator effort over the rollout:
  $$P_{energy} = \lambda_1 \sum_{t} \sum_{i} u_{t,i}^2$$
  Accumulated incrementally inside the stepping loop; `λ₁` is a tunable coefficient (start ≈ 0.001).
- **Fall penalty** — A large discrete penalty if the root body's Z coordinate drops below a threshold `z_min`:
  $$P_{fall} = \lambda_2 \cdot \mathbb{1}[z_{root} < z_{min}]$$
  `λ₂` should be large enough to dominate (e.g., 10.0). `z_min` depends on the individual's body height — use a fraction of the initial root Z.
- **Combined fitness:**
  $$F = F_{base} - P_{energy} - P_{fall}$$
  Clamp to a minimum of 0 to avoid negative fitness confusing selection operators.
- Track all components separately for logging.

**Acceptance criteria:** A robot that walks forward scores higher than one that stays still; a robot that falls scores near zero; a robot that uses excessive torque scores less than one that is efficient.

---

### 9. Evolutionary Operators

**Goal:** Implement mutation and crossover operators for both morphology and neural weights.

#### 9a. Morphological Mutation

- **Add limb:** With probability `p_add`, select a random leaf node and attach a new random `BodyGene` child with a random `JointGene`. Enforce a maximum tree depth to prevent runaway growth.
- **Remove limb:** With probability `p_remove`, select a random non-root leaf node and prune it (along with its entire sub-tree).
- **Perturb parameters:** With probability `p_perturb`, for each node/joint, apply Gaussian noise to continuous parameters (dimensions, mass, axis, range, gear). Clamp to physically valid ranges.
- After any structural mutation, mark the genotype's neural weights as "stale" so `evaluate()` will resize them.
- After any structural mutation, re-generate `<exclude>` pairs (or `contype`/`conaffinity` bits) for collision filtering between adjacent bodies. Without this, early-generation random morphologies generate massive self-collision overhead.

#### 9b. Neural Mutation

- For each weight `w_i` in the flat vector, with probability `ρ_m` (e.g., 0.05):
  $$w_i' = w_i + \mathcal{N}(0, \sigma^2)$$
  Use `σ ≈ 0.3` initially; consider decay over generations.

#### 9c. Crossover

- **Morphological crossover (sub-tree swap):** Select two parents, find nodes at similar depths, swap sub-trees. Validate the resulting tree (depth limit, body count limit).
- **Neural crossover (uniform):** If two parents share the same morphology (identical body graph hash), perform uniform crossover on the weight vectors: for each weight, pick from Parent A or B with 50/50 probability.

**Acceptance criteria:** Operators produce valid genotypes that compile to `mjModel` without errors; statistical tests show diversity is maintained.

---

### 10. Genetic Algorithm Main Loop

**Goal:** Orchestrate generational evolution: init → evaluate → select → reproduce → repeat.

**Details:**

- **Hyperparameters:**
  - Population size `N` (start with 200–500 for debugging, scale to 1000–5000).
  - Generations `G`.
  - Elitism count `E` (top `E` individuals copied unchanged to next generation).
  - Tournament size `K` for selection.
  - Mutation rates, crossover probability.
- **Initialization:** Generate `N` random genotypes (random trees of depth 1–3, random weights).
- **Selection:** Tournament selection — pick `K` random individuals, select the one with highest fitness as a parent.
- **Reproduction:**
  1. Copy top `E` elites.
  2. Fill remaining `N − E` slots by selecting two parents, applying crossover with probability `p_c`, then mutating the offspring.
- **Termination:** Run for `G` generations or until a fitness plateau is detected.
- Print per-generation statistics: best / mean / worst fitness, diversity metrics, morphology statistics.

**Acceptance criteria:** Fitness improves monotonically (best-of-generation) over initial generations on a flat-ground locomotion task.

---

### 11. Multithreaded Evaluation (Thread Pool)

**Goal:** Parallelize fitness evaluation across all CPU cores using a Master-Worker thread pool.

**Details:**

- Implement a `ThreadPool` class:
  - Constructor spawns `std::thread::hardware_concurrency()` persistent worker threads.
  - Workers block on a `std::condition_variable`, waiting for tasks.
  - `enqueue(std::function<void()> task)` pushes tasks to a `std::queue` guarded by `std::mutex`.
  - `wait_all()` blocks until all enqueued tasks are complete (use a counter + condition variable or `std::future`).
- At the start of each generation, enqueue one task per genotype: `pool.enqueue([&g]{ evaluate(g); });`
- **Memory isolation rules:**
  - `mjModel` and `mjData` are allocated and freed **entirely within the worker's task scope** — never shared (unless using model caching, see TODO 12).
  - Neural weight vectors are **read-only** during evaluation; the worker copies them into a local `NeuralController`.
  - The only shared write is `genotype.fitness`, which is a single `double` written once per task — no lock needed (distinct memory locations).
- **Compilation thread safety:** `mj_loadXML` and `mj_compile` are NOT documented as thread-safe. Either compile models in the master thread before distributing to workers, or guard compilation with a `std::mutex`. Simulation functions (`mj_step`, `mj_forward`, etc.) with isolated `mjData` are fully thread-safe.
- **Global callbacks (`mjcb_control`, `mjcb_passive`, etc.) are process-wide function pointers** — they cannot vary per thread. We use the explicit loop approach (read sensors → neural forward → set `ctrl` → `mj_step`) which avoids callbacks entirely.
- Error handling: if `mj_loadXML` fails inside a worker (malformed XML), catch the error, assign fitness = 0, and continue — never crash the pool.

**Acceptance criteria:** Wall-clock time for evaluating a generation scales roughly linearly with core count (e.g., 8× speedup on an 8-core machine). No data races under ThreadSanitizer.

---

### 12. Model Caching & Performance Optimization

**Goal:** Eliminate redundant XML parsing for identical morphologies.

**Details:**

- Compute a hash of the `BodyGene` tree (structural + parameter hash, ignoring neural weights).
- Maintain a generation-scoped `std::unordered_map<size_t, std::shared_ptr<const mjModel>>` cache.
  - Before evaluating, check if the morphology hash already exists in the cache.
  - If hit: skip `mj_loadXML`, just call `mj_makeData` on the cached model.
  - If miss: compile normally, insert into the cache.
- Use `std::shared_ptr` with a custom deleter that calls `mj_deleteModel` so the model is freed when the last reference drops (after all workers using it have finished).
- Workers with a cache hit call `mj_makeData(cached_model)` to get a thread-local `mjData`. `mj_copyModel(NULL, src)` can deep-copy a model if an owned copy is needed.
- Clear the cache between generations to prevent unbounded memory growth.
- Benchmark: measure compilation time saved; expect significant gains in later generations when morphologies converge.

**Acceptance criteria:** Cache hit rate reported per generation; measurable wall-clock improvement when hit rate > 50%.

---

### 13. Logging, Serialization & Visualization

**Goal:** Make evolutionary runs observable, reproducible, and resumable.

**Details:**

- **Generational logging:** Write a CSV per run with columns: `generation, best_fitness, mean_fitness, worst_fitness, diversity, elapsed_sec`.
- **Genotype serialization:** Serialize the top-N genotypes each generation to JSON or binary. Must support deserialization to resume evolution or replay creatures.
- **MJCF export:** Save the best individual's MJCF XML each generation so it can be loaded directly into MuJoCo's `simulate` viewer for visual inspection. Use `mj_saveLastXML(filename, model, err, err_sz)` to export any compiled `mjModel` back to XML — works even for models built via `mjSpec`.
- **Replay:** Write a utility that loads a serialized genotype, compiles it, runs the simulation, and records the trajectory to a MuJoCo `.mjcf` + keyframe file or renders frames to a video.
- *Stretch:* Real-time visualization using MuJoCo's built-in `mjv_*` / `mjr_*` rendering API or the `simulate` app as a subprocess.

**Acceptance criteria:** A completed evolutionary run produces a CSV of generational stats, a folder of best-creature MJCF files, and at least one can be visually replayed.

---

### 14. Project Documentation Setup

**Goal:** Set up a proper documentation site so completed planning content has somewhere to live, and present the available options.

**Details:**

MuJoCo uses **Sphinx** (Python-based documentation generator) with the **Furo** theme. Their setup includes:
- reStructuredText (`.rst`) source files
- Extensions: `sphinxcontrib.katex` (math), `sphinxcontrib.bibtex` (references), `sphinx_copybutton`, `sphinx_favicon`, `sphinx_toolbox.collapse`, `sphinx_toolbox.github`
- Build: `make html` in the `doc/` directory → static HTML site

We should evaluate these options:

| Option | Format | Pros | Cons |
|--------|--------|------|------|
| **Sphinx + Furo** (MuJoCo's choice) | `.rst` or `.md` (via MyST) | Same stack as MuJoCo; excellent C++ support via Breathe+Doxygen; math rendering; professional output | Requires Python toolchain; `.rst` has a learning curve |
| **Sphinx + MyST** | `.md` (Markdown) | Sphinx power with Markdown syntax; lower friction since we already write `.md` | Slightly less mature than raw `.rst` for advanced features |
| **Doxygen standalone** | C++ comments | Zero extra files — docs live in source code; auto-generates API reference | Poor narrative documentation; ugly default theme; limited customization |
| **MkDocs + Material** | `.md` | Very simple setup; great search; live reload; Material theme looks modern | No native C++ API doc extraction; would need Doxygen bridge |

**Chosen approach:** **Sphinx + Furo** — the same stack MuJoCo uses. We write narrative docs in reStructuredText (`.rst`), use Breathe + Doxygen for auto-generated C++ API reference, and Furo for the theme. This keeps us aligned with MuJoCo's own documentation and gives us first-class C++ support.

**Sub-steps:**
- **14.1** — ✅ Done. Chose Sphinx + Furo.
- **14.2** — Install Sphinx, Furo, Breathe, sphinx-copybutton, sphinxcontrib-katex, and Doxygen.
- **14.3** — Create `docs/` directory with `conf.py`, `index.rst`, `Makefile`, and initial page structure.
- **14.4** — Create a `Doxyfile` in the project root; configure Breathe in `conf.py` to pull C++ API docs from Doxygen XML output.
- **14.5** — Migrate completed TODO content (TODOs 1 & 2) into the docs site as the first pages (e.g., `getting_started.rst`).
- **14.6** — Add a top-level `make docs` convenience target (or CMake custom command) that runs Doxygen then Sphinx.

**Acceptance criteria:** Running the docs build produces a browsable HTML site with an index page, a "Getting Started" page (from TODOs 1–2), and auto-generated C++ API stubs from Doxygen.

---

## Suggested Implementation Order

```
Phase 1 — Foundation        : TODOs 1 → 2
Phase 2 — Morphology        : TODOs 3 → 4
Phase 3 — Brain             : TODOs 5 → 6
Phase 4 — Evolution Core    : TODOs 7 → 8 → 9 → 10
Phase 5 — Performance       : TODOs 11 → 12
Phase 6 — Polish            : TODOs 13 → 14
```

TODO 14 (documentation) can also be started earlier — setting up the docs skeleton in Phase 1 and incrementally populating it as each TODO completes is ideal.

Each phase should be independently testable before moving to the next. Phase 1–3 can be developed with hard-coded test fixtures; Phase 4 brings it all together; Phase 5 makes it fast; Phase 6 makes it usable.
