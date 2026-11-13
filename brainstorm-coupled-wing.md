# Brainstorm: a one-input coupled wing

Status: **built and committed, 2026-11-13.** All six build steps are done, the full suite is green, and M7 is struck in PLAN.md. The Godot look at the two runs was done by the user on 2026-11-13 and matched the prediction (M8 item struck). No follow-up is agreed. This file is the handoff; read all of "Handoff" before doing anything.

## Handoff (read this first)

**Goal:** build intuition for how geometric locking in a bird wing is achieved and controlled by one actuator. The model is a planar wing whose elbow and wrist are coupled by a four-bar (ulna and radius), so one servo at the elbow moves the whole wing between two passively locked poses, open and folded.

**How the user works** (also in memory):
- Ask clarifying questions one at a time, with numbered options and the recommended one marked.
- Give an ELI5 when asked. The user is studying bird-wing physics for intuition, so explain plainly.
- Record each answer in this file right away.
- Warn and wait before big multi-step work.
- Google-style commits, one logical change each, only when the user asks or agrees.
- Keep the regression suite green; baselines change only deliberately (PLAN.md 7a).
- Run tests as `timeout 600 dotnet test Tensegrity.sln --disable-build-servers > <log> 2>&1`, then grep the log. Piping can hang on orphaned MSBuild nodes.

**The user's go-ahead (question 13, answered "1"):** build the plan in "Build plan" below with its default numbers, and make it PLAN.md milestone "M7 — Coupled wing", renumbering the current M7 (Stage B) to M8.

**Where things stand:**
- Done and committed earlier (the five-bar detour, now parked):
  - `2bac601`: SDK pin.
  - `5269b08`: five-bar statics.
  - `15c8cc0`: `minima` command and brainstorm record.
  - See `brainstorm-multistable-servo.md`.
- Committed for the wing: `51a821a` (statics, generator and their tests), `79a71bf` (reference runs, baselines, README rows), and a third commit with PLAN.md and this file. The files:
  - `src/Tensegrity.Core/Analysis/CoupledWingStatics.cs`: `CoupledWingGeometry` (record with the defaults), `CoupledWingPose`, `CoupledWingRest` and `CoupledWingStatics`.
  - `src/Tensegrity.Topology/CoupledWing.cs`: the generator, the `FoldAndOpen(torque, foldStart, openStart, hold, ramp)` servo schedule and `WingtipGust`.
  - `tests/Tensegrity.Core.Tests/Analysis/CoupledWingTests.cs`: 35 tests of the statics against closed form and the simulator, and of the gust and fold-and-open runs. (The temporary probes are deleted.)
  - `src/Tensegrity.Topology/ReferenceTopologies.cs`: `coupled_wing` and `coupled_wing_switch`, with the servo pulse constants. Their scenarios are in `tests/Shared/Scenarios/CoupledWingScenarios.cs`, registered in `ReferenceTopologyTests` and `ReferenceScenarioTests` ("// M7").
  - `tests/golden/coupled_wing/` and `tests/golden/coupled_wing_switch/`: hand-written tolerances and new baselines. No existing baseline changed.
  - `topologies/coupled_wing.json`, `topologies/coupled_wing_switch.json`, and their rows in `topologies/README.md` (file table and dt study).
  - `PLAN.md`: "M7 — Coupled wing" added and struck, Stage B renumbered to M8, the 7a reference set extended.
  - This file.
- The full suite is green: 488 passed in core, 40 in regression (checked 2026-11-13).
- Saved runs to view in Godot: `runs/2026-11-13_111858_coupled_wing_switch` and `runs/2026-11-13_111905_coupled_wing` (`scripts/view-latest.sh` opens the newest).

**Immediate next step:** none is agreed. The Godot visual check is done (user confirmed 2026-11-13). "Ideas for later" lists possible follow-ups; ask the user which, if any.

## Decisions (all 2026-11-13)

1. **Coupling: two forearm bones (question 10).** Ulna and radius, each pinned to the upper arm and to the hand. With the radius as long as the ulna and equal offsets, the four-bar is a parallelogram ("drawing parallels"), so the hand stays parallel to the upper arm while the elbow folds: a Z-fold, as in a folded bird wing. How strict this coupling is in real birds is debated, which is fine for an idealized model.
2. **Shoulder bolted, servo at the elbow (question 11).** The upper arm is the ground frame, so the wing has one degree of freedom, the elbow angle. "Locked" means that with the servo idle, the wing rests open and rests folded: two minima of V(elbow).
3. **Locks: an over-center ligament pressing the elbow into bone stops (question 12).**
   - The stops are open at 0° and folded at −150°.
   - One preloaded elastic ligament crosses the elbow, and its line passes over the joint at −75°. Near each end it presses the elbow into that end's stop.
   - This relaxes the five-bar's "no help from stops" rule on purpose: real wings have bony stops.
4. **Go ahead with the defaults and a PLAN.md milestone (question 13).**
5. **Stops of 100 N·m/rad with 2.5 N·m·s/rad (question 14).** The M5/M6 values (20 and 0.2) let the elbow overshoot to the parallelogram's change point; see "Solved problem". The user chose a firm, well-damped stop over a soft padded one (20 / 2), a hard one (500 / 5) and servo braking.

## Design as built (defaults in `CoupledWingGeometry`)

- **Frame:** the shoulder is at the origin, and the upper arm (ground) runs along +x to the elbow at (0.30, 0). Gravity is −y, and the wing folds downward. The elbow angle φ is the ulna's world angle, which is also the simulator's joint angle at pin `elbow` (end A is the ulna, end B the ground).
- **Bones (slender bars):**

  | Bone | Length (m) | Mass (kg) |
  |---|---|---|
  | Ulna | 0.35 | 0.12 |
  | Radius | 0.35 | 0.04 |
  | Hand | 0.25 | 0.06 |

  The lengths and masses match the M6 `LockingWing`.
- **Radius pins:** 0.02 m from the elbow and from the wrist, in direction 15° (world for the root, the hand's frame for the tip). A parallelogram jams where the ulna lines up with the offset. With travel from 0° to −150°, a 15° offset clears both stops by 15°.
- **Stops:** stiffness 100 N·m/rad and damping 2.5 N·m·s/rad (decision 5), about critical damping for the wing's inertia about the elbow (0.014 kg·m²). Each stop is placed preload/κ short of its design pose, so the poses are exact equilibria. That puts the stops at −149.68° and −0.29°.
- **Ligament:**
  - Anchor on the upper arm, 0.04 m from the elbow at angle π + (−75°) = 105°. Insertion on the ulna, 0.04 m from the elbow along its axis.
  - EA 200 N and damping 5 N·s/m.
  - Rest length designed by bisection so that the weaker stop is pressed with a 0.5 N·m preload beyond the weight.
  - It is elastic, not a tendon. Anything that goes over center across a 150° fold must stretch at least 26% between the stop and the over-center point. EA 1000 (the wing's tendons) needs 3.0 N·m of servo torque at the peak, too close to the 3.4 N·m limit. EA 200 needs 0.99 N·m.
- **Servo:** a `JointActuator` with id `servo` on pin `elbow`, following a `Schedule` (constant 0 by default). Torque limit 3.4 N·m, carried over from the five-bar spec.
- **Pins:**
  - `elbow`: ulna to ground, with the stops.
  - `radius_root`: radius to ground.
  - `wrist`: hand to ulna.
  - `radius_tip`: radius to the hand, at the offset.
- **Cable:** `ligament`, from a ground anchor to the ulna.
- **Viewer:** the upper arm is the ground frame, so it is not drawn. The elbow and radius-root pins show as ground bases. (A body pinned to the ground at two points makes the constraint matrix singular, so the upper arm cannot be a body.)

## Results

The statics numbers below do not depend on the stop values, except d²V/dφ² at the rest poses (about 21.8 N·m/rad with the first 20 N·m/rad stops; about 100 more with the final ones). The pulse-trial table is from the first stops and is kept as the record of the problem.

The final runs, with the 100 / 2.5 stops (`ReferenceTopologies`):

- **`coupled_wing_switch`:** fold pulse of 2 N·m for 0.15 s from 0.05 s, open pulse the same from 0.5 s, 10 ms ramps.
  - The elbow passes −75° at about 0.22 s, reaches the folded stop at 0.30 s, overshoots to −154.4° and settles at −150.00° by 0.45 s.
  - The open pulse brings it back to the open stop at 0.77 s, overshooting to +3.8°, and it settles at 0.00° by 0.95 s.
  - The hand's angle stays within 1e-11° of zero throughout.
  - Peak stop torque is 48.5 N·m. Energy balance error is 8.4e-9 J, with 20 events.
- **`coupled_wing`:** the locking wing's gust (0.2 N·s over 20 ms at the wingtip) folds the open wing to −13.5° at 0.205 s. It returns to the stop at 0.30 s, rebounds to +0.9° and settles. Energy balance error is 2.5e-10 J, with 6 events.
- **dt study:** both converge at 16× per halving, and the change on halving dt = 1e-4 s is 2.2e-11 and 2.5e-11.

### Statics and first pulse trials (from the probe)

- The designed rest length is 0.05283 m. The open stop is pressed with 0.500 N·m and the folded stop with 0.564 N·m.
- `RestPoses()` finds exactly two rest poses, at −150.000000° and 0.000000°. d²V/dφ² is about 21.8 N·m/rad at both.
- The hand stays at 0.0000° and the radius equals the elbow angle at every elbow angle: the parallelogram works.
- Servo torque to hold the elbow, dV/dφ (N·m):

  | Elbow (deg) | 0 | −15 | −30 | −45 | −60 | −75 | −90 | −105 | −120 | −135 | −150 |
  |---|---|---|---|---|---|---|---|---|---|---|---|
  | Torque (N·m) | 0 | −0.78 | −0.81 | −0.62 | −0.28 | +0.12 | +0.52 | +0.83 | +0.98 | +0.91 | 0 |

  - Peak 0.986 N·m at −122.9°.
  - The energy peaks at −70.5° (0.940 J), against 0.2225 J open and −0.0161 J folded. The barrier is 0.72 J out of open and 0.96 J out of folded. The five-bar's weight-held catches were millijoules, so this is the lesson of the preload.
- Ligament tension is 40.3 N open and 102.9 N at the over-center point.
- Servo pulse trials (`FoldAndOpen`, fold pulse from 0.05 s, open pulse from 0.5 s, 10 ms ramps, 1 s at dt 1e-4):

  | Pulse | Deepest elbow | Elbow at 0.5 s | Elbow at the end |
  |---|---|---|---|
  | 1.0 N·m for 0.10 s | −19.1° | −5.45° | 0.44° |
  | 1.0 N·m for 0.15 s | −32.3° | 1.69° | 0.56° |
  | 1.0 N·m for 0.20 s | −51.5° | −35.82° | 1.37° |
  | 1.5 N·m for 0.10 s | −49.9° | −3.80° | −0.33° |

  None of these got past −75°, so the wing sprang back open each time. Energy-balance error was about 1e-12 J (2.6e-7 J in one run).

## Solved problem: overshoot through the folded stop

`1.5 N·m for 0.15 s` threw `InvalidOperationException: Cable 'ligament' has zero length`.

**Cause (confirmed 2026-11-13 by tracing φ(t)):** the elbow swings past the folded stop to −165.5°. At −165° the ulna lines up with the radius offset: the parallelogram's change point, where the four-bar jams and the simulation breaks down. The zero-length ligament is a later symptom.

The stock stops (20 N·m/rad, 0.2 N·m·s/rad) cannot prevent this for any pulse. Energy shows why: the barrier top is V(−70.5°) = 0.940 J, but V(−165°) = 0.745 J and V(+14.5°) = 0.926 J. A wing that just clears the barrier still has enough energy to reach the change point on either side, and the stops dissipate almost nothing. So the stops must change; a gentler pulse is not enough.

Stop trials (fold pulse at 0.05 s, open pulse at 0.5 s, 1 s run). "Deepest" is the furthest elbow angle in the fold phase; the change point is −165°.

| Stop κ (N·m/rad) | Stop damping (N·m·s/rad) | Pulses that fold then reopen (of 1.5/0.15, 2/0.1, 2/0.15) | Deepest |
|---|---|---|---|
| 20 | 0.2 (stock) | none; reaches the change point | −165.5° to −171.5° |
| 20 | 1 | all three | −160.0° |
| 20 | 2 | all three | −155.3° |
| 100 | 1 | none; bounces back over the barrier | −157.6° |
| 100 | 2.5 | all three | −154.4° |
| 200 | 2 | one of three; bounces | −154.6° |
| 200 | 3.5 | all three | −153.2° |
| 500 | 5 | all three | −152.2° |

Weak pulses (1 N·m for 0.25 s, 1.5 N·m for 0.12 s) do pass −75° but take longer than 0.45 s to finish folding, so the open pulse must wait or the pulse be stronger.

The damping matters more than the stiffness: an under-damped stiff stop bounces the wing back over the barrier. The user chose 100 / 2.5 (question 14, decision 5), now the `CoupledWingGeometry` defaults.

## Build plan (from question 13)

1. ~~`CoupledWing` generator: the upper arm as ground, ulna, radius and hand, four pins, the stops and the ligament.~~ Done in `51a821a`.
2. ~~Independent statics: the four-bar in closed form, V(φ), the two rest poses, the servo torque along the fold.~~ Done in `51a821a`.
3. ~~Tests in `tests/Tensegrity.Core.Tests/Analysis/CoupledWingTests.cs`.~~ Done in `51a821a` and `79a71bf`: the closed-form parallelogram, energy and tension against `PlanarSystem`, holding torque along the fold, two rest poses with zero servo torque, a released rest pose staying put, the gust run and the fold-and-open run.
4. ~~Switching run, viewable in Godot.~~ Saved under `runs/`; the user has not looked at it yet.
5. ~~Regression and plan: reference topologies, scenarios, tolerances and baselines, exported JSON, README rows, PLAN.md M7.~~ Done in `79a71bf`. `PlanarRecording`'s 1 s was enough: the switch finishes by 0.95 s.
6. ~~Commit.~~ Done in three commits at the user's request: `51a821a`, `79a71bf`, and the PLAN.md and brainstorm commit.

## Lessons carried from the five-bar (see `brainstorm-multistable-servo.md`)

1. A lock is a catch: the load pushes a joint into a taut tendon or a stop.
2. Each independent degree of freedom needs its own taut tendon or stop at rest.
3. Where a tendon attaches decides what it constrains.
4. Locks held by light links' weight are shallow; preload makes them deep. The coupled wing confirms this: barriers of 0.7–1 J, against millijoules.

And one from the coupled wing itself:

5. A deep lock hits hard. The ligament hands the whole barrier's energy to the stop, so the stop must dissipate it. A stop that is only stiff bounces the wing back out of the lock.

## Ideas for later (not agreed)

- Free the shoulder, held by its own tendon (question 11, option 2).
- A non-parallelogram radius, so the wrist folds faster or slower than the elbow.
- A position-controlled servo.
- `tensegrity` CLI support for the coupled wing (like `minima`), to sweep the ligament geometry.
