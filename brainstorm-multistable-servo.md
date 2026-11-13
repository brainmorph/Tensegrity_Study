# Brainstorm: a 2D multistable shape cycled by one servo

Status: parked 2026-11-13. The five-bar exploration is finished and kept for reference; the study moves to a one-input coupled wing (see "Decision: park the five-bar"). Not part of PLAN.md.

## Handoff (read this first)

**Goal:** a 2D, gull-sized (about 40 cm) compliant shape with exactly four passively stable states. One hobby servo, driven only when triggered, moves it monotonically around the four states and back along the same path. Resting states need no energy input, with no ratchets, detents or clutches. Only springs, rods and elastic members.

**Working style the user asked for:** ask clarifying questions one at a time, each with numbered or labeled options, and give an ELI5 when asked. Record each answer in this file right away.

**Where things stand:**
- All constraints in "Decisions so far" are settled and override earlier text.
- Direction C (compliant snap-through) is the chosen direction. Candidate 1 (five-bar, below) is the first geometry, now built and tested (see "Tooling" and "Findings").
- The statics checker (`tensegrity minima`) works and is tested against the simulator. No geometry has passed the spec yet: three tendons have given at most three minima, and two of them always share a servo angle (see "Findings").

**Which earlier text is superseded:**
- "Hard tension" and the ratchet or clutch options (1, 2, 3 and the Geneva/escapement analogies) are kept for reference only. The user rejected ratchets, detents and clutches as a source of stability. A standard hobby servo is still connected and parked at the stable angle.
- "Candidate directions" A, B and D are kept for reference only. C was chosen.
- Question 5 is listed twice. The first "5" is decided (gravity matters). The second, about a standard hobby servo, is answered in "Decisions so far" (a standard hobby servo).

**Open items:**
1. Spring anchor positions and pre-stress for Candidate 1. Not chosen.
2. Whether Candidate 1 can give four usable minima. Not with three tendons (see "Findings"). With a fourth tendon on coupler C–E it gives four, but two share a servo angle (finding 4).
3. Spring stiffness values, roughly bone and tendon-like. Not specified.
4. Servo torque and speed numbers are approximate. They need checking against a real large RC car servo.

**Next step:** none here. The five-bar is parked; see "Decision: park the five-bar" at the end.

## The idea

A 2D geometry with four separate stable configurations. A single actuator (a servo) pushes the geometry through all four in a cycle. In any stable configuration the servo does zero, or close to zero, work. The shape holds itself; the servo is not needed to keep it there. The servo only works while the shape is moving from one stable state to the next.

## Restated as physics

- **Stable configuration** = a local minimum of the passive potential energy V(q), where q is the shape's coordinates. Stability means V'' > 0 there. Four stable states means four separate minima.
- **Zero work while holding** = the servo's torque (or force) is zero at each minimum. The minimum has to be an equilibrium of the passive system, with no help from the servo.
- **Work during a transition** = the integral of τ dq along the path between two minima. The servo must supply enough to get over the barrier between them, or to pull the shape along a path where the passive forces are against it.
- **Closed cycle** = the input moves through all four states and comes back. With purely conservative springs and gravity, the net work over a closed cycle is zero. So the servo's total work over one cycle equals the energy lost to friction, damping and impacts, plus any energy the design deliberately lets it inject. A cycle that costs almost nothing has to recover most of its spring energy during the cycle.

## Hard tension to keep in mind

A servo does not naturally produce zero holding torque. A standard hobby servo holds position by driving against the error, so it draws current and does work against its own control loop even when the shape is still. "Zero work at rest" therefore needs one of these:

1. **Passive hold, servo disengaged.** A ratchet, detent, clutch or brake releases the servo during the hold. The servo only engages to move between states.
2. **Backdrivable servo or motor, zero torque at rest.** The servo is not holding anything. The minima are passive, and the servo output shaft is free to rotate when the shape is stable.
3. **Mechanism that moves itself through the states.** The servo only "primes" a spring or a cam, and the shape snaps through by itself. The servo's work goes into storing energy, not into moving the shape.

These are different machines. Choosing among them is the first real design decision.

## Analogies and known mechanisms to look at

- **Geneva drive / indexing mechanism.** Continuous rotation of the input produces intermittent motion with dwells. The output is locked during each dwell, and the input does no work to hold it. This is the closest mechanical match to "cycle through discrete states, zero work at each one."
- **Clockwork escapement.** A continuous drive is released in discrete steps. The escapement holds, then releases, then holds again.
- **Bistable snap-through mechanisms.** Compliant or pre-loaded linkages with two stable states, where a small input snaps the mechanism over a barrier. Extending this to four states would be the natural next step.
- **Multistable origami.** Waterbomb and Kresling-style folds have more than one stable state, and some have several. Folding patterns can be designed for specific stable states.
- **Four-bar linkages with preloaded springs.** A single crank input, with the coupler and springs setting the number of stable configurations. This is the most mechanism-like option and is easy to simulate.
- **Ratchet and pawl with a spring-loaded return.** Holds each step passively and allows one-way motion only.

## Candidate directions (not yet compared)

A. **Four-bar with preloaded springs, servo as crank, detent-based hold.** Simplest to model. Holds need a detent, so the zero-work requirement is met mechanically, not by the servo.

B. **Geneva-style indexing wheel with the shape on the output.** Dwells give zero-work holds by construction. Four slots, four stable states. The shape would be rigid links, and the output would rotate between four fixed angles.

C. **Multistable compliant shape (snap-through).** The shape is a set of elastic members. Four stable states come from the geometry and pre-stress. The servo only drives the input and the shape snaps. Zero work at rest is natural, but cycling in one direction needs a non-reciprocal path, which may be hard.

D. **Hybrid: passive multistable shape plus a ratchet or clutch.** The shape supplies the states. A ratchet or clutch lets the servo engage only during transitions.

## Questions to settle before any modeling

1. **Decided: zero holding torque.** The servo applies no torque while the shape sits in a stable state. Transitions may cost work.
2. **Decided: the cycle can reverse.** The servo must drive the shape both ways around the four states, so the design cannot rely on one-way ratchets alone.
3. **Decided: compliant members.** The shape is made of bending or stretching elastic members, and its stable states come from pre-stress and geometry (snap-through style). Options A and B above assumed rigid links, so they need revisiting. (The Tensegrity project's current model is rigid links with cables and pin joints, so option A or B fits it most directly.)
4. **Decided: the actuator is a standard hobby servo.** Its control loop always draws power to hold position, so zero holding torque needs a clutch, a ratchet or detent, or a linkage that disengages the servo during holds. Options 1 and 2 above (ratchet, clutch) are now the main route. The servo still has to be able to drive the shape both ways.
5. **Decided: gravity matters.** It must be in the energy model, and the shape's orientation relative to gravity is part of the design. (Original question: is gravity allowed to act on the shape?) It can help hold states or break them, depending on orientation.
6. (Answered in "Decisions so far") Actuator type: a standard hobby servo.

## Decisions so far

- Each stable state is passively stable: no energy input, and no ratchets, pawls, detents or clutches. Only springs, trusses and elastic members.
- The servo stays connected during a hold, parked at the stable state's angle.
- The four stable states do not need equal stability. Some may be deeper minima than others.
- The servo visits the states in a fixed cyclic order, forward or reverse. It does not choose freely between them.
- Forward and reverse use the same path between neighboring states.
- The servo directly controls the angle of one joint. The rest of the shape follows from the springs and members.
- That joint's angle is limited to 0 to 180 degrees.
- Gravity acts in the shape's plane, so it can help or hurt each stable state.
- Stable states may sit anywhere in the joint's 0 to 180 degree range, not only at the stops. The springs and members must hold them without help from the stops.
- The shape may have three or more joints in total, including the servo joint.
- Elastic members are linear springs and rods only (stretch-only, no bending beams or flexures).
- The shape is limited to a plane for now.
- Size: gull-sized, about 40 cm across.
- One full four-state cycle takes a few seconds.
- The servo runs only when triggered. It does not cycle continuously.
- Each trigger moves the shape one state along the cycle, then the shape rests passively in that state.
- The shape always rests in one of the four stable states. Any other configuration relaxes passively, like gradient descent, into the nearest stable state. Only the four minima are allowed as resting points. Every configuration in the joint's range must flow into one of the four states (no ungated regions).
- Stable states may cluster, but any two must differ by at least 15 degrees at the servo joint. Other joints may differ by any amount.
- The cycle is monotonic in the servo joint's angle: the states are visited in increasing (or decreasing) angle order, one direction at a time.
- The servo's torque limit is checked at every point along each transition, not only at sampled points or the peak.
- The shape has exactly four stable states. This count is fixed, not a target to relax.
- Stiffness is not specified exactly. Members are roughly modeled on bird bones (stiff, light, hollow) and tendons (elastic, stretch-only). Exact values are left for later. Tendon-like members may go slack: they pull only when stretched and exert no force when shortened. Bone-like members carry both tension and compression, and they stay straight (no buckling is modeled).
- The servo joint is a plain hinge with no built-in stops. Its 0 to 180 degree limit is a range of motion, not a physical stop.
- All other joints are also plain hinges with no stops. The springs and members alone set the stable states.
- The shape may be asymmetric.
- The servo is bolted to a fixed base, and one end of the shape is pinned to the same base. The shape moves only relative to that base.
- If a transition needs more torque than the servo can give, the move may be slowed down, but the servo must never exceed its maximum torque. The whole cycle must still finish within a few seconds. Slowing down only reduces inertia torque, so if the springs alone push back harder than the servo's maximum, the geometry is rejected.
- Transitions use full dynamics: inertia, momentum and the few-second cycle time are modeled. The shape can overshoot a state, which matters for snap-through.
- Resolved: the return from the last state to the first retraces the same path. The only return route is the one used going forward, so the full span is swept once each way per cycle.
- Servo limit: whatever a large RC car servo can do. Rough reference, to confirm against a real part: about 20 to 35 kg·cm of torque (2 to 3.4 N·m) and about 0.15 s per 60 degrees.
- A stable state counts if a small disturbance returns the shape to it (a true energy minimum). Zero net joint torque at that state follows automatically.
- The cycle can reverse.
- The shape is compliant members (pre-stress and geometry set the stable states).
- The actuator is a standard hobby servo.
- Gravity matters and must be in the energy model.
- First direction to develop: **C, compliant snap-through.**
- Candidate 1 is a true five-bar with two degrees of freedom: the ground plus links A–C, C–E, E–D and D–B. The servo drives A–C, and the angle at B is passive. (Decided 2026-11-13.)
- Candidate 1's elastic members are three tendons, from C, D and E to ground anchors. There is no strut across C–D. (Decided 2026-11-13.)
- Candidate 1 gets a fourth tendon, because three tendons give at most three gravity catches (see "Findings"). It runs from a ground anchor to the middle of coupler C–E. (Decided 2026-11-13.)
- Spec torque limit: 3.4 N·m (the high end of the large RC car servo range). The servo's torque, spring pushback plus inertia, must stay at or below 3.4 N·m at every point of every transition. This assumes a top-end servo. (Decided 2026-11-13.)

**Clarified constraint (replaces the earlier "zero holding torque" wording):** each stable state is passively stable. It needs no energy input to stay there, and the design uses only springs, trusses and elastic members. No ratchets, pawls, detents or clutches are allowed as the source of stability.

**Servo link (decided):** the servo stays connected during a hold and is parked at the stable state's angle. The shape's passive forces then give zero net torque on the servo. Residual draw from friction and deadband is accepted as control effort, not as holding work.

## Spec (numeric, for the statics checker)

Written 2026-11-13. Each item is a pass/fail check the `minima` command reports.

**Coordinates.** A is at the origin and B at (base width, 0), with gravity along −y. The servo angle α is the world angle of link A–C, measured from +x, so 0 to 180 degrees sweeps the half-plane above the base. The passive angle β is the world angle of link B–D. A configuration is (α, β) plus which side of the line C–D the hinge E sits on.

**Potential.** V = gravity on the four links (slender bars) + Σ ½·k·(L − L0)² over the stretched tendons, with k = EA / L0. Slack tendons contribute nothing. This is the same tendon law as the simulator's cables.

**Relaxation.** "Relaxes passively" means the overdamped flow M·q̇ = −∇V on the five-bar's configuration manifold, with M the mass matrix. This is the slow-motion limit of the simulator with heavy global damping. A start's basin is the minimum its flow ends at.

**Checks:**
1. **Count.** Exactly four distinct local minima (positive-definite reduced Hessian) with 0 ≤ α ≤ 180 degrees.
2. **Range.** No minimum lies outside 0 to 180 degrees, and no start inside the range flows out of it. Every valid start inside the range ends at one of the four.
3. **Gaps.** Sorted by α, consecutive minima differ by at least 15 degrees.
4. **Transitions.** Hold α at each value from one minimum to the next, in 0.25-degree steps, and let the passive coordinate relax at each step. The path must end at the next minimum, in both directions.
5. **Torque.** Along every path in both directions, the quasi-static servo torque stays within ±3.4 N·m. Inertia is checked later, in dynamic runs.
6. **Holding.** At each minimum, the simulator's holding torque at the servo pin is zero to round-off. This is the cross-check against `PlanarSystem`.

Snaps are reported, not judged. A snap is a jump of the passive coordinate while α is held, where the conditional minimum disappears. Their size and torque matter for the dynamic phase.

## Candidate 1 (proposed, untested)

A planar five-bar linkage with two free degrees of freedom. The servo drives one of them, and the other is passive.

- **Base:** two ground pivots, A (servo) and B (plain passive hinge), about 40 cm apart.
- **Links (decided, replaces the earlier coupler C–D):** A–C, C–E, E–D and D–B, so the shape is a true five-bar: the ground plus four moving links. C, D and E are plain hinges. The servo drives A–C, and the angle at B is the passive coordinate. (The earlier list, A–C, a coupler C–D and B–D, was only a four-bar with one degree of freedom, which left nothing passive to snap through.) Lengths are not chosen yet. The earlier draft had about 20 cm for A–C, B–D and the coupler.
- **Servo:** turns link A–C at A, limited to 0 to 180 degrees.
- **Springs (decided, replaces the earlier strut and coupler-midpoint tendon):** three tendon-like members (stretch-only), running C → ground anchor, D → ground anchor and E → ground anchor. There is no strut. A rigid strut across C–D would lock C, E and D into a triangle and remove the passive degree of freedom. Whether C–E–D folds through is left to the tendons and gravity. Anchor positions and rest lengths are not chosen yet.
- **Stable states:** four minima of the potential in the two coordinates (servo angle and passive angle), each at least 15 degrees apart in servo angle, and linked by a single monotonic valley.

**Why it's a candidate:** it has the minimum DOF needed (two) for more than two resting spots, and it uses only hinges, springs, struts and tendons.

**Why it might fail:** a two-DOF potential with three springs may have only two or three minima, not four. Whether four exists depends on the spring placement and pre-stress, which we have not chosen.

**First test:** write this as a topology in the existing planar model, then search the potential over a grid of the two angles for local minima. The search gives the count, positions, and servo-angle gaps directly. If it finds fewer than four, change one spring anchor and rerun.

## Tooling (built 2026-11-13, not committed)

- `src/Tensegrity.Core/Analysis/FiveBarGeometry.cs`: the geometry record and the named parameters the command line sets and sweeps.
- `src/Tensegrity.Core/Analysis/FiveBarStatics.cs`: the potential, the overdamped relaxation on the configuration manifold, the minima search from a start grid, and the quasi-static transitions. It is independent of `PlanarSystem`.
- `src/Tensegrity.Topology/FiveBar.cs`: the five-bar as a simulator model, posed at any configuration.
- `src/Tensegrity.Cli/CliApp.Minima.cs`: `tensegrity minima`, which prints a single report (with `--map yes` for the basins and `--save FILE --at N` for a topology at minimum N), or sweep maps with `--vary NAME=VALUES` (one or two).
- Tests: `tests/Tensegrity.Core.Tests/Analysis/FiveBarStaticsTests.cs` and `tests/Tensegrity.Core.Tests/Cli/MinimaCommandTests.cs`.
  - A gravity-only hanging chain matches the closed form to 1e-9 rad.
  - Energy and tensions match the simulator in any pose.
  - Every minimum has zero holding torque at the servo pin (spec check 6), and nothing accelerates there.
  - Held servo torques match the simulator's.
  - A released minimum stays put in the simulator.
- `Directory.Build.targets`: SDK 10.0.115 (installed 2026-11-12) wants the 8.0.34 targeting packs for net8.0, which are not cached. This pins them to the cached 8.0.33. This change is unrelated to the brainstorm.

Example: `dotnet run --project src/Tensegrity.Cli -- minima --set tendon_e.rest=0.26 --vary tendon_c.rest=0.20:0.26:7`.

## Findings (2026-11-13)

1. **The default geometry has three minima, all gravity catches.** Each tendon's rest length is a few millimetres short of reaching a symmetric arch. The tendons hold only about 0.1 N, and the quasi-static servo torque between states peaks at about 0.03 N·m, 1% of the 3.4 N·m limit. With 20 g links, gravity forces are about 0.2 N, so the states are shallow, with millijoule differences.
2. **Three tendons give at most three catches, and two always share a servo angle.**
   - A state held by gravity has two taut tendons (two constraints for two degrees of freedom). Three tendons give three pairs: {C,D}, {C,E} and {D,E}.
   - Hinge C's position depends on the servo angle alone. So in both states where tendon C is taut, the servo angle is wherever tendon C just reaches, and the two angles are identical. That fails the 15-degree gap by construction.
   - Seen in every sweep so far: rest lengths of C and D at 0.20 to 0.26 m, E's anchor x against its rest length, and anchors directly above A and B.
3. **Tendon C cannot catch the servo link on both sides of upright.** Even with tendon C's slack arc spanning the top of the servo link's swing, the couplers' weight hangs between C and D and always pulls C toward the middle. Only the inner catch is stable.

4. **A fourth tendon, to the middle of coupler C–E, gives four minima, but not yet four usable ones.** (Question 7, decided 2026-11-13: option 1.)
   - Built as `tendon_ce`, with parameters `tendon_ce.x`, `.y`, `.rest` and `.at`. A test checks it taut against the simulator.
   - The default anchor is (−0.05, 0.50) with L0 0.32 m. The default geometry now has four minima, at 61.9, 62.0, 69.3 and 91.4 degrees.
   - Maps of its anchor x and y against its rest length show four minima in five cells, every one with two minima at 61.9 degrees. Those are the tendon-C catches, which share the servo angle as in finding 2.
   - All the minima so far lie between 60 and 92 degrees, which is too narrow for three gaps of 15 degrees once the duplicate is removed.
   - Next lever: tendon C's anchor and rest length. Move its catch angle away from the others, and leave only one minimum that uses it.

**Question 6 (decided 2026-11-13: option 1, add a fourth tendon): how to get past three minima?** Options put to the user:
1. Add a fourth tendon, giving up to six tendon pairs. The new tendon should not pin the servo angle by itself.
2. Move tendon C off hinge C onto the coupler C–E, so that no tendon fixes the servo angle alone. This keeps three tendons, so at most three gravity catches, and the count problem stays.
3. Shorten the tendons into a prestressed regime, where the tendons fight each other instead of gravity. States become deeper and torques larger, and the minima come from geometry rather than catches.

## Next step if this goes further

Direction C was chosen, and Candidate 1 is the proposed first geometry. See "Handoff" at the top for the current next step. Before building, write a short spec: the four target configurations, the input coordinate, the zero-work criterion as a number (for example, peak holding torque below some threshold), and the cycle order. That spec can go into a numbered test before any topology is built.

## Findings, continued (2026-11-13)

5. **Moving tendon C only moves the duplicate pair.** (Question 8, decided: map tendon C.)
   - Maps of tendon C's anchor y, then its anchor x, against its rest length found four minima in 12 cells, each with a pair at one servo angle. Wherever tendon C catches, the passive side can still rest two ways (caught by D, or by E and C–E).
   - The other minima stay at 69.3 and 91.4 degrees in almost every cell. Tendons D, E and C–E were all tuned to the same symmetric arch, so the states cluster around it.
6. **Weight-only catches are fragile.** In the simulator, each default minimum was knocked by a 2 mN·s pulse at hinge E.
   - Minima A (61.9°), C (69.3°) and D (91.4°) hold.
   - Minimum B (62.0°) is knocked into C.
   - Depth needs preload, as the M6 preloaded wing showed.

## Decision: park the five-bar (2026-11-13)

Question 9 was put to the user three ways: finish the four-state puzzle by designing for target poses, build a coupled wing, or read the literature first. The user chose the **coupled wing**. Their aim is intuition for how geometric locking in a bird wing is achieved and controlled by one actuator. Four states of a five-bar is a detour from that.

What carries over to the wing:
1. **A lock is a catch.** A joint rests where the load pushes it into a taut tendon or a stop.
2. **Each independent degree of freedom needs its own taut tendon (or stop) at rest.** With n tendons, the resting states are limited to combinations of them.
3. **The attachment decides what a tendon constrains.** On the tip of one link it pins that joint alone; further out it constrains a combination of joints.
4. **Locks held by light links' weight are shallow.** Preload (tendons against stops or each other) makes them deep.

The tooling stays: `tensegrity minima`, `FiveBarStatics` and `FiveBar`. Its relaxation and transition code is the starting point for the wing's statics.

Next: a one-input wing whose elbow and wrist are coupled by a linkage (as a bird's radius and ulna couple them), with two locked states, extended and folded, and the servo moving it between them. This is to be planned with the user, likely as a PLAN.md milestone.

