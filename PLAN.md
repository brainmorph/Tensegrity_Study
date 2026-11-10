# Tensegrity Study — Plan

## 1. Goal

Explore how geometry and physics combine to produce stable truss-like shapes that hold with very little effort. The long-term target is a highly realistic but simplified model of a bird wing, in which locking happens through tendon-like cables and joint geometry rather than muscle force.

This is an exploratory study. There is no specific bird being modeled. The aim is to build intuition for how stable, low-effort configurations arise.

**Staged scope:**
- **Stage A (now): 2D, planar.** Rigid links in a plane, pin joints, cables, gravity, and optional joint actuators. Enough to show tensegrity-style stability and over-center locking.
- **Stage B (later): 3D.** Full 3D rigid bodies, the same physics extended to 3D. The 2D work is designed so the body and force interfaces can be extended without rewriting the integrator.

Two cooperating parts:

1. **Core simulation (C#)** — a physically realistic, deterministic simulation. It writes a **run output** to disk.
2. **Visualizer (Godot 4.7, mono)** — loads run output and plays it back: rendering, camera or view control, timeline scrubbing, tension coloring, and comparison of runs.

The simulation is the source of truth. Physical realism is the top priority for the core; visual polish comes second. Godot has zero impact on simulation logic: it only reads and presents recorded runs (see constraint 5 in Section 1a). This keeps the sim testable headless and lets runs be re-rendered without re-simulating.

## 1a. Project constraints

These rules apply to every milestone and every change:

1. **No third-party downloads.** Do not download or install any library, package, SDK, binary, or code from outside the machine. Use only what is already installed or cached locally (see Section 3).
2. **Web access is allowed for documentation only.** Browsing and downloading documentation (specs, API references, papers) is fine. Downloading code or libraries from that documentation is not.
3. **Build what we need.** If a capability is missing locally, implement it in this repo rather than pulling a dependency. Prefer the .NET base class library (`System.Numerics`, `System.Text.Json`, etc.) over anything else.
4. **Anything new that needs a download requires explicit approval first.**
5. **Godot has zero impact on simulation logic.** All physics happens in `Tensegrity.Core`. Godot only reads recorded run output and renders it. Specifically:
   - Godot does not run a physics engine (no `RigidBody*`, `CharacterBody*`, `PhysicsServer` calls, Jolt/Godot Physics, or collision shapes that affect motion).
   - Godot does not compute, correct, smooth, or extrapolate any state that changes what the run means. Positions, orientations, and tensions are shown exactly as recorded.
   - Any quantity Godot displays (e.g. tension colors, energy plots) must be written by the core into the run output. Godot does not derive physics quantities itself.
   - Godot does not write back into the simulation or its inputs. The only data flow is `Core → run directory → Godot`.
   - Godot does not reference `Tensegrity.Core`. It references only the read-only run-format project (see Section 5), so it cannot call solver code.
6. **Warn before big downloads or complex tasks.** Before downloading anything large (roughly over 100 MB, or any multi-file dataset) or starting a complex, long, or multi-step task that will take a while or change many files, warn the user first and wait for a go-ahead. Name what it is, its approximate size or scope, and why it's needed. Documentation pages and small reference files are exempt.
7. **Regression protection.** Every milestone must leave the regression suite green. Changes that alter a recorded baseline must be deliberate and explained (see Section 7a). Never update a baseline just to make a failing test pass.
8. **Google-style commit messages.** Follow Google's CL description conventions ([eng-practices](https://google.github.io/eng-practices/review/developer/cl-descriptions.html)):
   - **First line:** a short summary of specifically what the change does, as a complete sentence written as an imperative order (e.g. `Add RK4 integrator`, not `Added` or `Adds`). Keep it at 72 characters or fewer. Avoid vague summaries such as `Fix bug` or `Update code`; say what was broken or what was added.
   - **Blank line**, then a **body** that explains what changed and why, in plain prose, wrapped at about 72 characters. Include the reason for any golden-baseline change (Rule 7).
   - **Scope:** one logical change per commit. Don't mix refactors with behavior changes.
   - **Trailers** are added at the end as required by the session (currently `Co-Authored-By`).
   - Commits happen only when you ask for them.

Local resources in scope: .NET SDK 8.0 and 10.0; Godot 4.7 mono and its `GodotSharp` assemblies; the NuGet global cache at `~/.nuget/packages` (already contains xUnit, Microsoft.NET.Test.Sdk, coverlet, MathNet.Numerics, and others).

## 2. Non-goals (for now)

- 3D physics (deferred to Stage B).
- Real-time interactive simulation inside Godot (GDExtension binding). Revisit after the offline pipeline works.
- Automated form-finding / optimization of topologies. Hand-authored topologies first.
- Material fatigue, plasticity, or large-deformation fracture.
- Modeling a specific real bird. Parameters are generic and chosen for exploration.
- Feather aerodynamics and flight. The wing is modeled as a mechanical structure only.
- Publishing or packaging for end users.

## 3. Environment

- .NET SDK 10.0.x (present: `dotnet 10.0.114`), also 8.0.128.
- Godot 4.7 stable, **mono build** (C#-capable), at `~/Desktop/Godot_v4.7-stable_mono_linux_x86_64/Godot_v4.7-stable_mono_linux.x86_64`. Verified: reports `4.7.stable.mono.official`. Ships `GodotSharp/` alongside it.
- OS: Linux. All paths and commands in this plan assume Linux.
- Repo is under git.
- Test framework: **xUnit 2.9.3** (runner 3.1.4, Microsoft.NET.Test.Sdk 17.14.1) from the local NuGet cache. Verified in M0: the repo `nuget.config` clears all remote sources and lists only `~/.nuget/packages`, and restore succeeds offline. A package or exact version not in the cache fails the restore instead of downloading.
- Godot 4.7's C# assemblies target **.NET 8** (`GodotSharp.4.7.0`, `lib/net8.0`), so `Tensegrity.RunFormat` targets `net8.0`. All other projects target `net10.0`. `Godot.NET.Sdk 4.7.0` is in the local cache, so the Godot project can restore under the same `nuget.config`.

## 4. Physics model (Stage A: 2D)

### 4.1 Elements

All quantities are `float64` (double) inside the core. Nothing is downcast until the output writer. Units are SI in the plane: meters, kilograms, seconds, newtons, newton-meters, radians.

- **Link (strut) — rigid body in 2D.** Each link is a planar rigid body:
  - State: center-of-mass position `(x, y)`, orientation angle `θ`, and velocities `(ẋ, ẏ, θ̇)`. Three degrees of freedom per link.
  - Mass `m` and scalar moment of inertia `I` about the center of mass, computed from geometry (slender bar: `I = m·ℓ²/12`).
  - Compression-only in the intended design. A strut under tension is reported as a configuration error for the chosen topology. An optional debug flag allows tension to validate the solver.
  - Attachment points are body-fixed offsets in the link frame, so cables and joints act at the true points and produce moments.
- **Pin joint.** Connects a point on one body to a point on another body, or to the ground frame (a fixed pivot). Enforced as a holonomic constraint: the two attachment points coincide. Modeled with Lagrange multipliers (the constraint reaction forces are part of the solution, not inputs).
- **Joint limit (optional, per pin).** Stops on the joint angle `φ = θ_A − θ_B` (a ground end counts as `θ = 0`), standing in for bone-on-bone contact: two pinned links can only meet by folding about their shared pin. Past a stop, a stiff torsional penalty spring-damper pushes back with a couple `max(0, k·δ + c·δ̇)`, where `δ` is the penetration past the stop. It never pulls. As with cables, the switches between **clear**, **pressing** and **recoiling** are events. Either stop may be left open. The same one-sided law (`ContactLaw`) serves the ground line.
- **Joint actuator (muscle stand-in).** A torque applied at a pin joint as a function of time or a feedback rule. Used to ask: how much effort does it take to hold or move a configuration? Default is zero torque, so the structure has to hold itself.
- **External load (perturbation).** A force given in the world frame at a body-fixed point on a link, piecewise-linear in time like an actuator schedule, with each breakpoint split as an event. An impulse is a short triangular force pulse with the requested area (`ExternalLoad.Pulse`), not a jump in velocity. That keeps the trajectory continuous and pins satisfied, and real impacts last a finite time anyway. Its work `F·v` at the point is integrated as **external work**.
- **Cable — elastic tendon.** Connects two body-fixed attachment points.
  - Axial stiffness `EA` and rest length `L0`, with `k = EA / L0` and `c` a viscous damping coefficient. Tension:
    - `T = 0` if `L < L0` (**slack**), however fast the cable is lengthening.
    - `T = max(0, k·(L − L0) + c·L̇)` if `L ≥ L0`: **taut** when the bracket is non-negative, **recoiling** (stretched but shortening too fast for the spring, so `T = 0`) when it is negative.
    - The damper can lower the tension but never make it compressive, and a slack cable never pulls. (An earlier draft used `max(0, k·(L − L0) + c·L̇)` for all `L`, which lets a slack cable that is lengthening fast pull before it reaches `L0`.)
  - Unilateral: a cable never pushes. Switches between slack, taut and recoiling are **events** (see 4.2).
  - Optional cable mass, lumped at its ends. Default: off.
- **Gravity**: uniform `g = (0, −9.81)` m/s² applied to each body's center of mass.
- **Ground line** at `y = h` (default 0), acting on listed body-fixed contact points (`GroundLine`):
  - **Normal:** the penalty spring-damper `ContactLaw` on the penetration `δ = h − y`, never pulling, with clear, pressing and recoiling modes.
  - **Friction (exact Coulomb, stick and slip):** a tangential spring `k_t` and damper `c_t` in parallel, in series with a Coulomb slider of strength `μ·N`. Each contact point carries the spring's extension `z` as a state variable. The point **sticks** while `|k_t·z + c_t·v| ≤ μ·N` (friction `−(k_t·z + c_t·v)`, `ż = v`). Otherwise it **slips** in direction `σ`, with friction exactly `−σ·μ·N` and `ż = (σ·μ·N − k_t·z)/c_t`. Off the ground it is **free**: no friction, and `z` relaxes as `ż = −k_t·z/c_t`.
  - **Moving ground (ground drop):** the height may follow a piecewise-linear schedule instead of staying fixed. The height is a state entry integrated from the schedule's slope, so every contact and energy quantity stays a function of the state. The penetration rate becomes `δ̇ = ḣ − ẏ`, and the line does work `N·ḣ` on each point it pushes, counted as external work. A line that falls faster than the contact dampers can follow (`c·ḣ` beyond the spring force) recoils at once and does no work.
  - The mode depends on the state alone, so stick and slip are ordinary events, the force is continuous at both switches, and `z` never jumps. A loaded point at rest sticks at a fixed `z` with no creep. Points that stop sliding can keep equal and opposite friction locked in (residual stress), each inside its cone.
- **Prestress**: each cable's rest length `L0` is shorter than its geometric length in the initial configuration, so the structure is in self-equilibrium under gravity. The prestress magnitude is part of the topology file.

**Limitations of rigid links:** they cannot buckle or bend, so the model covers the rigid-strut regime only. This is acceptable for the first study and should be stated in any result.

### 4.2 Integration and constraints

- **Method: classical fixed-step Runge–Kutta 4 (RK4)** on the dynamics. This is the minimum accepted integrator.
- **Constraint handling (decision needed, see Section 9):** pin joints are holonomic constraints. Default approach:
  - Maximal coordinates `(x, y, θ)` per body, with pin constraints enforced through Lagrange multipliers. Each RK4 stage solves a small linear system for the constraint forces.
  - Constraint drift is controlled by a Baumgarte-style stabilization term or by projecting positions and velocities back onto the constraint manifold after each step. The drift is logged and tested.
- **Step size**: fixed `dt`, chosen per topology from a stability and convergence study (Section 7). Start point: `dt` ≤ 1e-4 s for stiff prestressed cables; verified rather than assumed.
- **Non-smooth cable forces**: the switches between slack, taut and recoiling cause RK4 to lose its order if a step straddles one. Plan:
  1. Latch each cable's mode at the start of a step and hold it for the whole step, so every RK4 stage sees a smooth right-hand side (in the taut mode the unclamped `k·(L − L0) + c·L̇` is used).
  2. If any cable's mode at the end of the step differs from its latched mode, locate the switch time by bisection to a tolerance well below `dt` (default 1e-12 s).
  3. Split the step just past the switch, re-latch, and continue. This preserves RK4 accuracy across events (verified: error ratio ≈ 16 per `dt` halving through repeated slack/taut cycles, where unsplit RK4 is erratic and 10³–10⁵× worse).
  - A fallback is a C¹-smoothed `softplus` transition with a tunable width. It is not the default because it changes the physics slightly.
- **Determinism**: fixed element iteration order, no parallel reductions with non-deterministic ordering, seeded RNG if any randomness is used. Same input + seed → bit-identical output on the same platform.
- **Solver interface**: `IIntegrator.Step(State, dt, Derivatives)`. RK4 is the first implementation. The interface must let us swap in an adaptive or implicit method later without touching element code.

### 4.3 Damping and energy

- **Physical damping, not numerical**: cable viscous damping `c` (from a damping ratio per cable, default small). Joint friction is off by default and can be added as a per-joint parameter.
- **Global damping** only for settling experiments, and it is off in realistic runs unless the topology file requests it (`global_damping`, γ in 1/s). Each body feels `−γ·m·v` and `−γ·I·ω`, so a free body's speed decays as `e^(−γt)`, and the loss `γ·Σ(m·v² + I·ω²)` counts as dissipated energy. Its value is recorded in the manifest.
- **Energy accounting every frame**: kinetic (translational + rotational), gravitational potential, elastic potential stored in cables, joint-limit stops and ground contact springs (normal and tangential), work done by actuators and external loads, and cumulative dissipated energy from cable damping, joint friction, joint-limit damping and ground contact (normal and tangential dampers, and the friction slider's `μ·N·|u|`). Energy balance is checked against the work from external inputs.

## 5. Solution architecture

```
Tensegrity_Study/
├─ PLAN.md
├─ src/
│  ├─ Tensegrity.Core/        # pure C#: 2D rigid links, pins, cables, actuators, RK4, constraints, events, energy, run writer
│  ├─ Tensegrity.RunFormat/   # read-only shared data types + manifest/frame reader (no solver code)
│  ├─ Tensegrity.Topology/    # topology definitions + generators (planar tensegrity, wing chains, ...)
│  └─ Tensegrity.Cli/         # `dotnet run` entry: load topology + params, run sim, write run dir
├─ tests/
│  └─ Tensegrity.Core.Tests/  # unit, analytic, convergence, energy, determinism, isolation
├─ topologies/                # hand-authored topology JSON files
├─ runs/                      # run output (git-ignored)
└─ godot/                     # Godot 4.7 mono project: rendering only, references Tensegrity.RunFormat
```

- `Tensegrity.Core` has no dependency on Godot or any UI.
- `Tensegrity.RunFormat` holds the file format types and the reader. It contains no physics, and both the core (writer) and Godot (reader) depend on it. Keeping it separate means Godot never references the solver.
- `Topology` produces the same `Structure` object regardless of whether it came from code or JSON.
- The CLI is the only place with file-system side effects beyond run output.
- The `godot/` project contains scenes, materials, view and camera, timeline UI, and playback code only. Any physics-related code in it is a defect.
- Stage B (3D) will add a 3D body type and 3D forces. The integrator and constraint layer are written against a generic state vector so they carry over.

## 6. Run output format

Each run is a directory, e.g. `runs/2026-11-09_planar-x_seed42/`:

- `manifest.json` — human-readable, versioned:
  - `format_version`, `created_utc`, sim version (git SHA if available), `dimensions: 2`
  - `units` (SI, planar)
  - `integrator` (`"rk4"`), `constraint_method`, `dt`, `total_steps`, `frame_stride`, `frame_count`
  - `event_tolerance` (time tolerance for cable slack/taut events)
  - `gravity`, `ground` parameters, damping parameters, actuator schedule, RNG seed
  - `topology` — links (id, mass, inertia, attachment points in body frame, initial pose), pins (id, body/ground refs, local points), cables (id, endpoint refs, `L0`, `EA`, damping, prestress), actuators
  - `diagnostics` — energy series, constraint residual maximum, number of event splits, any compression-violation flags for links
- `frames.bin` — little-endian `float32`, one frame after another, as named channels listed in the manifest (`frame_layout.channels`) so Godot can read it without guessing. Per frame: `time`; per link `x`, `y`, `θ`; per cable tension and both world endpoints; per pin the reaction force (useful for showing where effort goes) and its world position; per actuator its torque; per pin with a joint limit the limit torque on its end A; per external load its force and the world position of its point; with a ground, the line's height; per ground contact point the normal and friction forces on it and its world position. Endpoints and pin positions are recorded so the renderer places numbers and computes no geometry.

Why this split: the manifest is small and inspectable; frames are large and binary. Godot's reader parses the JSON and reads floats from a `FileAccess`.

Precision: state is `float64` in the sim. Frames are `float32` for size; the energy series and diagnostics are `float64`.

## 7. Validation strategy

Tests live in `tests/Tensegrity.Core.Tests`. Realism is verified by these tests, not assumed.

1. **Unit — force laws**: cable tension-only behavior, slack cable exerts zero force, damping sign correct, link body-frame to world-frame transforms correct.
2. **Analytic — free body**: a free link under gravity follows `y(t) = y0 − ½gt²` (within RK4 error). A free link spinning at a set angular velocity conserves angular momentum and kinetic energy.
3. **Analytic — pendulum**: a single link pinned at one end with small amplitude matches the closed-form period `T = 2π·√(I_pivot / (m·g·d))`, where `d` is the distance from pivot to center of mass.
4. **Analytic — cable oscillator**: a single mass on a cable with known `k` and `m` matches `ω = √(k/m)` and the damped decay rate.
5. **Convergence (RK4 order)**: halving `dt` reduces global error on a smooth test problem by ≈16×. This is the test that proves the integrator is what we claim.
6. **Constraint**: pin constraint residual stays below tolerance over a long run, with drift controlled as designed.
7. **Energy**: with damping off and no actuators, total mechanical energy drift is below a set tolerance and scales as `dt⁴` on smooth segments.
8. **Event handling**: a cable that goes slack and taut again produces the same trajectory under `dt` halving within tolerance.
9. **Static equilibrium — planar tensegrity**: the planar two-strut topology (Section 8, M2) holds its prestressed shape at rest with no actuators.
10. **Over-center lock — DEFINITION: a pose is locked if it is a stable equilibrium at zero actuator torque.** Test: the structure stays put with zero torque, and returns to the pose after a small perturbation. A two-link chain with a cable across the joint must pass this, and must collapse when the cable's rest length is increased past the threshold. The threshold matches the geometric prediction within tolerance.
11. **Effort measurement — DECIDED metric: required actuator torque at each joint** to hold a pose still (the minimum torque that keeps joint angular acceleration at zero). Zero means the structure holds itself. The torque is recorded per pose and per topology. Elastic energy and lock margin are recorded as secondary diagnostics, not as the primary measure.
12. **Determinism**: same topology + seed run twice produces byte-identical `frames.bin`.
13. **Round-trip**: writing a run then reading it back reproduces positions within `float32` precision.
14. **Stability sweep**: `dt` halved produces converged results; the chosen `dt` for each topology is recorded with its evidence.
15. **Isolation (automated)**: a test scans the `godot/` project and fails on any reference to `Tensegrity.Core`, any physics node or `PhysicsServer` use, or any write to a run directory.
16. **Render fidelity**: a headless check loads a run and confirms the Godot transforms equal the recorded frame values exactly (or within `float32` precision).

Visual validation (from M4 onward): the Godot view of a known structure visibly holds shape at rest, and locks and releases as predicted.

## 7a. Regression framework

Purpose: when we make progress, we find out immediately if something that used to work has broken. This sits alongside the validation tests in Section 7, which check physics correctness against theory. Regression checks compare the current output against recorded known-good results.

**Structure:**
- `tests/Tensegrity.Regression/` (or a `Regression` folder inside the test project): a test suite that runs each reference topology with a fixed seed and compares the result against a stored **golden baseline**.
- `tests/golden/<topology>/`: baseline files. Each baseline holds the manifest, a summary of key diagnostics (final pose, energy series, max constraint residual, lock torques), and a short sample of frames. Baselines are committed to git, so every change to them is visible in history.
- Comparison uses explicit tolerances per quantity, stored with the baseline. Positions and angles use a tolerance in `float32` precision; energy and torques use a relative tolerance. Exact byte equality is used where determinism is required (test 12).

**Rules:**
- A regression failure is a stop signal. Investigate before changing anything.
- Updating a baseline requires: (1) an explicit explanation of why the output changed, (2) confirmation that the change is intended, and (3) the change recorded in the commit message. Never update a baseline just to make a test pass.
- **How to update:** `TENSEGRITY_UPDATE_GOLDEN=<name>[,<name>…] dotnet test`. Each named scenario's `baseline.json` is rewritten from current output, and its test fails on purpose so an update never passes silently. Rerun without the variable once the diff has been reviewed. There is no "update all" option. `tolerances.json` is always written by hand and must exist first.
- Every folder in `tests/golden/` must have a matching scenario, so a baseline cannot silently stop being checked.
- Each milestone adds its reference topologies to the regression suite before it is considered done.
- The suite runs with one command, for example `dotnet test` with the regression project included, and it must stay fast enough to run after every change (target: under 60 seconds at the current scale).

**Reference set (grows with milestones):**
- M1: free-body fall, spinning body, pendulum.
- M2: planar two-strut tensegrity (`planar_x` on pinned feet under gravity, `planar_x_free` floating), single cable oscillator (`cable_oscillator`, damped, through taut, recoiling and slack).
- M5: two-link locking arm (`locking_arm` locked after a kick, `locking_arm_collapse` with the elbow tendon past its threshold). At M6 the arm gained an elbow fold stop at 150°, and the collapse baseline was updated on purpose to include the strike and rebound.
- M6: ground contact (`ground_drop`, a bar dropped tilted on to the ground line: it lands, slips, bounces clear twice and comes to rest), and perturbations (`perturbed_bar`, the bar at rest when the ground falls away under it, then knocked at one end by a force pulse).

## 8. Milestones

Completed milestones are struck through, with the commit that finished them. Anything a finished milestone left open is carried forward as a live item in a later milestone.

### ~~M0 — Scaffolding~~ — done in `e3f2132`
- ~~`git init`, `.gitignore` (bin/, obj/, runs/, .godot/, .import/), empty solution `Tensegrity.sln` with the projects above (Core, RunFormat, Topology, Cli, and the test project).~~
- ~~Regression harness in place (Section 7a): the regression project, the `tests/golden/` folder, the tolerance comparison helper, and one smoke-test baseline.~~
- ~~Build and test pass, including the regression suite.~~

### ~~M1 — 2D numerics core~~ — done in `a6e8889`
- ~~2D vector and rotation utilities in `float64`. Planar link (mass, inertia, state). RK4 integrator. Gravity.~~
- ~~Tests 2 (free body), 3 (pendulum), 5 (RK4 convergence) green.~~
- ~~Pin constraints arrive in M2, so the M1 pendulum is a test-side stand-in: the link is integrated in its single angle about the pivot, and the full link pose is reconstructed from it.~~ Carried forward to M2.

### ~~M2 — Planar tensegrity~~ — done in `213b85c`, `2c00e80`, `0842d02`, `2aa2974`
- ~~Pin constraints (Lagrange multipliers, stabilization). Cables with `EA`, `L0`, damping, unilateral behavior, event splitting. Energy accounting.~~
- ~~**Carried from M1:** reproduce the `pendulum` baseline with a real Lagrange-multiplier pin, then retire the test-side `ReducedPendulum` stand-in.~~ Reproduced to 5e-13 with no numeric baseline change.
- ~~Topology: **planar two-strut tensegrity** — two crossing struts with four cables, the 2D analog of a tensegrity X. Simple, and it has a known prestressed equilibrium to check against.~~ Built floating (no gravity, four cables) and standing on two pinned feet under gravity (three cables).
- ~~Tests 1, 4, 6, 7, 8, 9 green.~~
- Left open, carried forward: topology JSON and strut-compression reporting (M3), the X's step size (M3), joint friction and actuator work in the energy balance (M5), global settling damping (M6).

### ~~M3 — Run output and CLI~~ — done in `4b49810`, `3a9f6ca`, `a5bba0e`, `f98f4e6`, `78ce212`, `07af9a4`
- ~~Manifest and frame writer, reader in Core for tests.~~ The reader lives in `Tensegrity.RunFormat` (Section 5), so Godot uses the same one; the writer is `RunRecorder` in Core.
- ~~CLI: `dotnet run --project src/Tensegrity.Cli -- --topology topologies/planar_x.json --seed 42 --out runs/...`~~ As `tensegrity run --topology …`, plus `dt-study`, `export-csv` and `export-topologies`.
- ~~Tests 12, 13 green. Test 14 documented per topology.~~ Study in `topologies/README.md`: every reference topology runs at 1e-4 s.
- ~~**Carried from M2:** topology JSON files (Section 9.7) that load into the same `PlanarModel` the C# generators (`PlanarTensegrityX`) build, so the CLI's `--topology` has something to read.~~ Run settings live in the file's `simulation` block, overridable from the command line.
- ~~**Carried from M2:** report each strut's axial force and flag any strut in tension (Section 4.1) in the manifest diagnostics, alongside the event count, unresolved event steps and maximum pin residual the stepper and system already provide.~~ Axial force is the midpoint section force, including the strut's own weight and inertia; the CLI exits with code 3 when a strut goes into tension.
- ~~**Carried from M2:** for test 14, at `dt` = 1e-4 the planar X's energy-balance error falls only 12.6–12.8× per halving (its stiff cable modes put it at the start of RK4's asymptotic range; 5e-5 gives 14.4×). Decide its recorded `dt` from that study.~~ Its trajectory is asymptotic at 1e-4 s (16× per halving); 1e-4 s kept.
- Left open, carried forward: actuators in the manifest and frames (M5).

### ~~M4 — Godot visualizer (2D first light)~~ — done in `9e55287`, `806fd0e`, `cfbfa28`, `82bab9c`
- ~~Godot 4.7 mono project under `godot/`. Loads a run directory via `Tensegrity.RunFormat`. Renders links as bars at their recorded position and angle, cables as lines whose color follows recorded tension, pins as dots. Timeline slider.~~ Runs open from `-- --run <dir>` (plus `--play`) or an in-app folder picker. Cable color uses one scale per run, from 0 N to the highest tension in the manifest diagnostics, and slack cables are dashed. Usage is in `godot/README.md`.
- ~~Rendering uses only recorded frames: no interpolation between frames unless explicitly labeled visual-only. Default is frame-exact display.~~ There is no interpolation at all: playback shows the last frame recorded at or before the playback clock.
- ~~No physics nodes or physics-driven transforms in the Godot scene. Transforms are set directly from recorded data.~~ Both Godot physics engines are also set to the `Dummy` server.
- Tests 15 and 16 green. Test 15 scans `godot/` on every `dotnet test`. Test 16 runs Godot headless (`--fidelity-dump`) on `planar_x` and `cable_oscillator` and checks every frame for exact float32 equality. It is skipped when the Godot executable is missing (`TENSEGRITY_GODOT`, else the path in Section 3).
- Visual validation: `planar_x` holds its shape at rest over the full run, checked on rendered frames 0 and 200.
- Left open, carried forward: recorded pin reaction forces are not drawn yet (M6, with the plots). Interactive use (mouse pan and zoom, folder picker) has only been checked by building and launching; a hands-on pass is still due.

### ~~M5 — Locking wing (2D)~~ — done in `6a61114`, `51b9e26`, `b02ac32`, `b06b15b`, `09310a5`, `74d0b39`, `03752cc`, `3d607e5`
- ~~**Carried from M3:** record actuators and their schedules in the manifest's `actuators` list (empty today), and their torques as frame channels.~~ Done in `b02ac32`: the list holds the same entries as `topology.actuators`, each actuator has an `actuator.<id>.torque` channel, and the energy series gains `actuator_work`.
- ~~**Carried from M2:** joint actuators add their work to the energy balance (`PlanarSystem.TotalEnergy` covers kinetic, gravitational, cable elastic and cable-damping dissipation today), and optional per-joint friction (Section 4.3) adds its dissipation the same way.~~ Done in `6a61114` and `51b9e26`. An actuator is a torque couple across a pin (+τ on end A) following a piecewise-linear schedule whose breakpoints are split like cable events. Joint friction is viscous (`c·Δω`), so it needs no events.
- ~~Topology: a **two-to-three link planar arm** pinned at a shoulder, with cables forming a tendon network across the elbow (and wrist, if three links). This is the first real test of the central question: does a cable arrangement lock the joints in an extended pose with zero actuator torque?~~ Done in `09310a5` with two links (Section 9.3). `LockingArm` holds each joint with one tendon, running from a point behind and above the joint to the next link out: a body anchor at the shoulder and a post on the upper link at the elbow. Tensions are solved so the straight-out pose is an exact zero-torque equilibrium. Answer to the central question: **yes**. The pose is locked, and it stays locked until a tendon is 8.2% (elbow) or 8.5% (shoulder) longer than designed.
- ~~Test 10 green.~~ Done in `74d0b39`. `ArmStatics` is the geometric prediction: the arm's potential energy in its two joint angles, independent of the simulator. Lengthening a tendon moves the arm along a branch of rest poses until a fold. The simulator holds within 1e-4 of the predicted rest-length scale (0.014 mm) and collapses 1e-4 past it. ~~Test 11 green.~~ Done in `b06b15b`: `PlanarSystem.SolveHoldingTorques` gives the torques that make every acceleration zero, and reports the leftover acceleration when the chosen joints cannot hold the pose. ~~Effort curves recorded per configuration.~~ Done in `03752cc`: the `effort` command, with the arm's curves in `topologies/README.md`. Holding the arm *straight* costs torque as soon as a tendon is 0.70% (elbow) or 2.25% (shoulder) long, but the *lock* lasts to the fold, because the arm droops to a new stable pose.
- Regression: `locking_arm` (kicked, holds) and `locking_arm_collapse` (elbow tendon 10% long, falls) in `3d607e5`, as Section 7a lists. Visual validation: Godot renders the locked arm straight at t = 1.95 s, and the collapsing forearm slack-tendoned and hanging by t = 0.29 s. Frames were rendered with Godot's `--write-movie`.
- Left open, carried forward: the three-link arm with a wrist (M6). Links pass through each other because there is no contact or joint limit, and the collapsing forearm swings through the upper link (M6, with the ground line). `SolveHoldingTorques` rejects redundant joints instead of giving the minimum-norm torques (M6, with parameter sweeps). Effort against joint angle, rather than against rest length, is not yet recorded (M6).

### M6 — Exploration tools
- ~~Perturbation inputs (impulse on a link, external force over time, ground drop with the ground line).~~ Done in `ab8948c`, `0d31506`, `c1b8a7f`, `66dd36f` and `d5e3759`. External loads are scheduled world-frame forces at body points, and an impulse is a short triangular force pulse. The ground drop is a ground line moving on a height schedule; its height is a state entry, and its work `N·ḣ` counts as external work. `perturbed_bar` is in the regression suite, and Godot moves the line with its recorded height. Left open: drawing load forces (with the arrows below), and command-line flags for adding perturbations to a topology without editing its file (with the parameter sweeps).
- ~~**Carried from M2:** optional global damping for settling experiments (Section 4.3), off by default and recorded in the manifest when used. M2 did not need it: its equilibria are exact by construction.~~ Done in `d116fd8`: `−γ·m·v` and `−γ·I·ω` on every body, set by the topology's `global_damping`, with its loss counted as dissipated energy.
- Parameter sweeps: vary cable rest lengths and attachment points, record which configurations lock and how much effort each needs.
- Side-by-side run comparison in Godot.
- **Carried from M4:** draw the recorded pin reaction forces (`pin.<id>.fx`, `.fy`) as arrows, scaled per run like cable tension.
- Plots of energy, per-cable tension, and actuator effort over time (Godot or exported CSV).
- **Carried from M5:** the three-link arm, adding a wrist with its own tendon (Section 9.3), checked against the same kind of statics prediction as test 10.
- ~~**Carried from M5:** link–link contact or joint limits. Without them, the collapsing forearm swings through the upper link. Treat them like the ground line, as a penalty or unilateral constraint with events.~~ Done with joint limits in `624b35d` and `b9b2cae`: optional stops on a pin's joint angle, each a one-sided torsional penalty spring-damper (`ContactLaw`) with events, as Section 4.1 describes. The arm's elbow folds no further than 150°. Measuring showed the old forearm folded to −169.5°, not through the upper link; the overlap seen in Godot came from drawing thick bars at a sharp fold. Segment–segment contact between links that are not pinned together is left until a topology needs it.
- ~~**Carried from M2 (Section 9.2):** the ground line at `y = 0`, a penalty spring-damper with Coulomb friction (Section 4.1). It reuses `ContactLaw` for the normal force, and the run manifest's `ground` field records it.~~ Done in `7b4c586`, `953411a` and `c7099eb`. Friction is exact Coulomb with stick and slip, through a tangential spring-damper in series with a slider (Section 4.1), so a loaded foot holds with no creep. `ground_drop` is in the regression suite, and Godot draws the line. Left open: drawing contact forces (with the pin-reaction arrows below), and the ground-drop perturbation itself (above).
- **Carried from M5:** minimum-norm holding torques when the actuated joints are redundant (`SolveHoldingTorques` throws today), and effort curves against joint angle (`effort` sweeps rest length only).

### M7 — Refinement and Stage B (optional)
- Adaptive RK (Dormand–Prince) or implicit integrator if stiff cases demand it.
- Form-finding by dynamic relaxation from a loose initial guess.
- Stage B: 3D rigid bodies, same topologies extended to 3D.
- Export a single run to a standalone video/GIF.

## 9. Open questions

1. **Constraint method — DECIDED: Lagrange multipliers + stabilization.** Each RK4 stage solves a small linear system for constraint forces. Alternatives considered: penalty joints (simpler, but adds artificial compliance that blurs the locking threshold) and reduced coordinates (exact for open chains, but hard for the closed loops in the planar tensegrity).
2. **Ground contact — DECIDED: defer to M6.** The locking question does not need a floor, so the core is validated in free space first.
3. **Wing topology — DECIDED: two links, one elbow, first.** Three links (adding the wrist) follows once the two-link locking behavior is validated.
4. **Cable mass — DECIDED: massless for now.** Cable inertia can be added later as an option, once a test shows it matters.
5. **Frame format — DECIDED: binary `frames.bin` + JSON `manifest.json`.** A CSV export tool for small runs is added for inspection.
6. **Godot language — DECIDED: C#.** The visualizer references only `Tensegrity.RunFormat`, so the reader and data types are shared with the sim without exposing solver code.
7. **Topology source — DECIDED: JSON files, generated from C# helpers.** Topologies are stored as JSON in `topologies/`. C# generators in `Tensegrity.Topology` build common shapes and export them, and both paths produce the same `Structure`.

## 10. Immediate next steps

1. Confirm answers to Section 9 (or accept defaults).
2. Run M0: git init, solution scaffolding, empty tests passing.
3. M1: 2D numerics core with RK4, verified by tests 2, 3, and 5 before any tensegrity physics is written.
