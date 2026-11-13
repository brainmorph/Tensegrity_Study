# Topologies

Planar topologies as JSON (PLAN.md 9.7). Each file describes the links (role, mass, inertia, length, initial pose and velocity), pins, cables (rest length, axial stiffness, damping), gravity, constraint stabilization, and a `simulation` block with the default `dt`, `duration`, `frame_stride` and `event_tolerance`. Command-line flags override the simulation block, and the run manifest records the values actually used.

Run one with:

```
dotnet run --project src/Tensegrity.Cli -- run --topology topologies/planar_x.json --seed 42
```

To perturb a run without editing its file, add `--impulse LINK,X,Y,JX,JY,START,DURATION` (a triangular force pulse), `--force LINK,X,Y,T,FX,FY[,T,FX,FY...]` (a piecewise-linear force) or `--ground-drop START,DURATION,DEPTH`. Forces are in the world frame and points are in the link's body frame. `--impulse` and `--force` may be repeated. They become loads named `cli_impulse_N` and `cli_force_N`, so the run manifest records them. `run`, `dt-study` and `sweep` all take them. For example, to knock the locked arm's forearm tip down with 0.05 N s at 0.2 s:

```
dotnet run --project src/Tensegrity.Cli -- run --topology topologies/locking_arm.json --impulse lower,0.175,0,0,-0.05,0.2,0.005
```

## Reference files

These seven files are generated, not hand-edited. They come from `ReferenceTopologies` in `src/Tensegrity.Topology`. Regenerate them with:

```
dotnet run --project src/Tensegrity.Cli -- export-topologies
```

Tests hold each file equal to the catalog, and each catalog entry bit-identical to the regression scenario of the same name. A run of a reference file therefore starts from exactly what the golden baselines protect.

| File | What it is |
|---|---|
| `pendulum.json` | 1 kg, 1 m bar pinned at one end, released 1 rad from hanging. |
| `cable_oscillator.json` | 2 kg weight on one damped cable, bouncing through taut, recoiling and slack. |
| `planar_x.json` | Two-strut tensegrity on pinned feet under gravity, one strut kicked. Rings down to its shape. |
| `planar_x_rest.json` | The same X at rest. It holds its prestressed shape with no actuators. |
| `planar_x_free.json` | The X floating with no gravity, its shape disturbed with zero total momentum. Rings down to its shape. |
| `locking_arm.json` | Two-link arm held straight out by one tendon per joint, with no muscle torque. The lower link is kicked downward, the lock holds, and the arm rings back to straight out. |
| `locking_arm_collapse.json` | The same arm with its elbow tendon 10% too long, past the lock threshold. Released from straight out, the lower link collapses, strikes the elbow's fold stop at 150° and rebounds. |
| `ground_drop.json` | A 1 m bar dropped tilted on to the ground line (penalty contact, Coulomb friction μ = 0.5). It lands on one end, slips, bounces clear twice, and comes to rest lying flat by 0.5 s. |
| `locking_wing.json` | The three-link wing (a hand added at the wrist), each joint held straight out by one tendon. A downward gust at the wingtip bends the wrist 23°, two thirds of the way to its fold; on the rebound the hand flicks up 52° with its tendon slack. The lock holds. |
| `locking_wing_collapse.json` | The same wing with its wrist tendon 8% too long, past the 6.3% threshold. Released from straight out, the hand collapses and swings from the wrist. |
| `locking_wing_preloaded.json` | The same wing with stops against bending upward at the elbow and wrist, its tendons shortened to press each joint into its stop with about the joint's own gravity moment. The same gust bends the wrist 23° as before but the elbow only 6° (10° without), and the stops catch the rebound at +1.4° and +0.8°. The lock holds. |
| `perturbed_bar.json` | The same bar at rest on the ground, then perturbed. At 0.1 s the ground falls 0.1 m in 10 ms; the bar falls freely and lands at 0.2425 s, as free fall predicts. At 0.5 s a 5 ms force pulse knocks its right end up and sideways; it rocks, lands and settles flat. |
| `coupled_wing.json` | A one-input wing. The upper arm is bolted to the body, and the forearm is two bones (ulna and radius) pinned 2 cm apart at each end: a parallelogram, so the hand stays parallel to the upper arm while the elbow folds. The elbow has stops at open (0°) and folded (−150°), and an elastic ligament that passes over the elbow at −75° presses it into the nearer stop. The wing rests open with the elbow servo idle. A downward gust at the wingtip folds the whole wing 13.5°, well short of over-center, and the ligament pulls it back on to the open stop. The lock holds. |
| `coupled_wing_switch.json` | The same wing, with no gust. A 2 N·m servo pulse of 0.15 s carries the elbow past over-center, and the ligament finishes the fold and holds it on the folded stop by 0.3 s, servo idle. A second pulse the other way at 0.5 s opens the wing again by 0.8 s. The stops catch it 4.4° past folded and 3.8° past open; the parallelogram jams 15° beyond either. |

Hand-authored topologies can live here too. Files may contain `//` comments.

## Step size (test 14)

Every reference topology runs at **dt = 1e-4 s**. The evidence comes from `dt-study`, which runs each file for 2 s at dt = 4e-4 s and four successive halvings. It reports the largest change in any position or angle between one level and the next, over ten shared sample times:

```
dotnet run --project src/Tensegrity.Cli -c Release -- dt-study --topology topologies/<name>.json
```

Results at commit `78ce212`. The change on halving dt = 1e-4 s estimates the error at dt = 1e-4 s:

| Topology | Change on halving dt = 1e-4 s | Ratio per halving (4e-4 → 5e-5) | Energy balance error at 1e-4 s | Events per level |
|---|---|---|---|---|
| pendulum | 8.2e-14 | 16.0, 19.0, then at the rounding floor | 1.6e-13 J | 0 |
| cable_oscillator | 1.1e-13 | 16.2, 17.0, then at the rounding floor | 1.7e-12 J | 9 at every level |
| planar_x | 5.6e-12 | 16.6, 16.3, 16.3 | 5.8e-11 J | 0 |
| planar_x_rest | 9.4e-18 | (static: rounding only) | 5.3e-15 J | 0 |
| planar_x_free | 1.4e-12 | 15.0, 14.4, 15.3 | 3.2e-10 J | 0 |
| locking_arm | 2.0e-14 | 16.5, 17.3, then at the rounding floor | 7.4e-13 J | 0 |
| locking_arm_collapse | 8.6e-12 | 16.0, 18.4, 9.7 | 9.8e-13 J | 13 at every level |
| ground_drop | 2.3e-10 | 15.5, 16.3, 15.8 | 2.7e-8 J | 22 at every level |
| perturbed_bar | 3.0e-11 | 16.1, 15.9, 14.9 | 3.3e-8 J | 33 at every level |
| locking_wing | 2.4e-10 | 16.2, 16.1, 16.0 | 6.5e-10 J | 9 at every level |
| locking_wing_collapse | 5.3e-13 | 17.0, 15.0, then at the rounding floor | 1.0e-13 J | 12 at every level |
| locking_wing_preloaded | 1.6e-10 | 16.2, 16.0, 16.2 | 6.3e-10 J | 20 at every level |
| coupled_wing | 2.2e-11 | 15.6, 16.1, 16.0 | 2.5e-10 J | 6 at every level |
| coupled_wing_switch | 2.5e-11 | 16.2, 15.2, 7.7 | 8.4e-9 J | 20 at every level |

The locking arm rows were added at M5 (after commit `67acd3b`). The collapse row was rerun at M6, when the arm gained its elbow fold stop (Section 4.1 of PLAN.md): the falling forearm now strikes the stop at 150° instead of folding on to about 170°. Its 13 tendon and stop switches are located to 1e-12 s at every level. The ratio is 16–18 down to 1e-4 s and drops to 9.7 on the last halving, where the change (8.9e-13) nears the rounding floor of a run through large angles and many events. The change at 1e-4 s is 8.6e-12, far below float32 precision.

The ground_drop row was added at M6. Its 22 contact and stick/slip switches are found at every level, and the trajectory falls by 16 per halving throughout. Its change at 1e-4 s, 2.3e-10, is the largest of the set: the 1e4 N/m contact springs (about 140 rad/s) are the stiffest elements so far. It is still about 300 times below float32 precision. The energy balance error, 2.7e-8 J against 5.2 J dissipated, also falls by 16 per halving, so it is integration error at the impacts and not a leak in the contact model. The perturbed_bar row, also M6, behaves the same way: its 33 ground, load and contact switches are found at every level. So do the wing rows; locking_wing_preloaded adds the hyperextension stops' contact switches. The gust's start is set off the step grid (0.10037 s): with breakpoints on it, a breakpoint landing exactly on a step end at one level is not counted as a split there, which changed the event count with no change to the run.

The coupled wing rows were added at M7 (after commit `15c8cc0`). Both find the same switches at every level: gust or servo-schedule breakpoints and the elbow's stop contacts. In coupled_wing_switch the ligament throws the elbow at each stop at about 17 rad/s, the hardest strikes in the set (48 N·m peak stop torque). Its energy balance error, 8.4e-9 J, falls by 16 per halving throughout, so it is integration error at the strikes. Its trajectory ratio drops to 7.7 on the last halving, where the change (3.3e-12) nears the rounding floor of a run through 300° of rotation.

Why 1e-4 s:

- **Converged.** Halving the step changes no position or angle by more than 3e-11 m or rad, except 2.3e-10 for ground_drop, 2.4e-10 for locking_wing and 1.6e-10 for locking_wing_preloaded. That is at least 2000 times (those three: about 250 times) below the float32 precision the frames are stored in (about 6e-8 at 1 m).
- **In RK4's asymptotic range.** The change falls by about 16 per halving from 4e-4 s down, until it reaches the rounding floor. Event-splitting keeps the cable oscillator at this order, and it finds the same 9 switches at every step size.
- **Energy of the X.** The X's energy balance error falls more slowly at the larger steps: 7.5–8× from 4e-4 s, then 12.6–12.8×, then 14.4–15×. Its stiff cable modes (ω·dt ≈ 0.03) are still entering the asymptotic range there. The trajectory is already asymptotic, and the balance error is at most 3.2e-10 J, so the X needs no smaller step.
- **Consistent.** All golden baselines and test tolerances were set at 1e-4 s.

Rerun the study when a topology changes, and record the new numbers here.

## Effort curves (test 11, M5)

Effort is the joint torque needed to hold a pose still (PLAN.md 7, test 11). `effort` scales one cable's rest length and prints the holding torque in the file's initial pose, taken at rest:

```
dotnet run --project src/Tensegrity.Cli -- effort --topology topologies/locking_arm.json --cable elbow_tendon --from 0.98 --to 1.03 --steps 11
```

A positive torque lifts the arm. The design rest lengths (scale 1) hold the arm straight out with zero torque at both joints. Each tendon changes only its own joint's torque, because the elbow tendon pulls between two links of the arm and adds nothing about the shoulder.

| Rest-length scale | Elbow tendon tension (N) | Elbow torque (N m) | Shoulder tendon tension (N) | Shoulder torque (N m) |
|---|---|---|---|---|
| 0.980 | 27.556 | −0.6044 | 43.358 | −0.7919 |
| 0.990 | 17.177 | −0.2991 | 32.819 | −0.3919 |
| 0.995 | 12.065 | −0.1488 | 27.629 | −0.1950 |
| 1.000 | 7.005 | 0.0000 | 22.491 | 0.0000 |
| 1.005 | 1.995 | 0.1473 | 17.404 | 0.1930 |
| 1.010 | 0 (slack) | 0.2060 | 12.367 | 0.3842 |
| 1.020 | 0 (slack) | 0.2060 | 2.442 | 0.7608 |
| 1.030 | 0 (slack) | 0.2060 | 0 (slack) | 0.8535 |

What the curves say:

- **Holding the arm straight is sensitive.** Torque grows almost linearly as a tendon lengthens, until the tendon goes slack: at scale 1.0070 for the elbow (its 0.70% design strain) and 1.0225 for the shoulder (2.25%). Past that point the joint carries its link's full weight moment: 0.206 N m at the elbow and 0.854 N m at the shoulder. A short tendon over-lifts, so the torque turns negative.
- **The lock is not.** With zero torque, a lengthened tendon does not drop the arm. The arm droops to a new stable pose where the tendon is taut again. The lock is lost only at the fold that test 10 predicts and checks: scale 1.0819 for the elbow (+11.1 mm) and 1.0850 for the shoulder (+13.1 mm). This margin comes from the tendon geometry (the post behind the joint), not from pre-tension.

Computed at commit `74d0b39` plus the `effort` command.

## Effort against joint angle (M6)

`effort --bend PIN` keeps the cables as designed and bends one joint instead, by `--from` to `--to` degrees. The links on the pin's outer side turn rigidly about it. Each row gives every cable's tension and the holding torque in the bent pose, at rest. This is the stiffness of a tendon lock: how hard the structure pushes back when a joint is moved off its pose.

```
dotnet run --project src/Tensegrity.Cli -- effort --topology topologies/locking_wing.json --bend wrist --from -20 --to 20 --steps 9
dotnet run --project src/Tensegrity.Cli -- effort --topology topologies/locking_wing.json --bend elbow --from -20 --to 20 --steps 9
```

Negative bends fold the joint down. The torques are the holding torque at the bent joint, in N m. The other joints need almost none.

| Bend (deg) | Wrist tendon (N) | Wrist torque | Elbow tendon (N) | Elbow torque |
|---|---|---|---|---|
| −20 | 56.728 | −0.5786 | 79.774 | −1.0525 |
| −15 | 46.213 | −0.5595 | 66.549 | −0.9706 |
| −10 | 33.844 | −0.4636 | 51.569 | −0.7746 |
| −5 | 19.648 | −0.2803 | 34.875 | −0.4540 |
| 0 | 3.658 | 0.0000 | 16.512 | 0.0000 |
| +5 | 0 (slack) | 0.0733 | 0 (slack) | 0.4837 |
| +20 | 0 (slack) | 0.0691 | 0 (slack) | 0.4563 |

What the curves say:

- **The lock is one-sided.** Folding a joint stretches its tendon, which pushes back hard. Bending it upward slackens the tendon within 5°, and from then on the joint carries only its outer links' weight moment. The hyperextension stops of `locking_wing_preloaded.json` exist for exactly this side.
- **The stiffness fades toward the fold.** The wrist's restoring torque grows by 0.28 N m over the first 5° but by only 0.02 N m from −15° to −20°, though the tension still climbs. Folding the hand turns the tendon toward the pin, so its moment arm shrinks faster than its tension grows. This is the approach to the fold that test 10 locates at about −33°.

When the swept pins are redundant, as in a closed loop with every joint listed, both `effort` modes give the torques with the smallest sum of squares and say so. A pin in a closed loop cannot be bent alone.

## Parameter sweeps (M6)

`sweep` runs a topology over a grid of one or two parameters and reports which configurations lock and how much effort each needs:

- `--vary rest:CABLE=VALUES` scales a rest length.
- `--vary anchor:CABLE.a.y=VALUES` moves one coordinate of a cable end (`a` or `b`, `x` or `y`) by an offset in m. The rest length is scaled with the cable's length, so the initial tension stays as designed and only the geometry changes.
- `VALUES` is `FROM:TO:N` or a comma-separated list.

Each configuration runs from the file's initial state, its loads included. It **collapses** when any joint angle moves more than `--collapse-angle` (default 90°) from where it started. The default is wide enough that the plain wing's 52.5° upward flick on the gust's rebound still counts as held. The effort columns are the holding torque in the starting pose, as in `effort`. `--csv FILE` writes every column, including each cable's tension.

Sweeping the gusted wing tendon by tendon, in steps of 0.001:

```
dotnet run --project src/Tensegrity.Cli -- sweep --topology topologies/locking_wing.json --vary rest:wrist_tendon=1.0:1.02:21 --duration 6
```

| Tendon | Static fold (test 10) | Survives the gust up to | First collapse | Joint that gives way |
|---|---|---|---|---|
| Shoulder | 1.0733 | 1.025 | 1.026 | wrist |
| Elbow | 1.0681 | 1.030 | 1.031 | elbow |
| Wrist | 1.0633 | 1.008 | 1.009 | wrist |

What the sweep says:

- **A gust leaves far less margin than the statics.** The statics say how long a tendon can get before no stable pose is left. The sweep asks whether the wing still rides out its reference gust. For the wrist the answer shrinks from 6.3% to 0.8%: the gust alone already bends the wrist two thirds of the way to its fold.
- **A long tendon can fail at another joint.** With the shoulder tendon long, the whole wing droops and the gust then folds the *wrist*, not the shoulder.
- **Released from straight, a lock can fail short of its fold.** Without the gust, a wing whose tendon is too long swings from straight toward its drooped rest pose and overshoots. Short of the fold it holds (the tests check each tendon 0.02 short of its fold), but the dynamic boundary sits a little below the static one. For the wrist it falls between 1.05 and 1.06, against the 1.0633 fold.

A second `--vary` makes a grid and prints a map. Moving the wrist post (the tendon's end on the forearm) up or down against the wrist tendon's length, gust included:

```
dotnet run --project src/Tensegrity.Cli -- sweep --topology topologies/locking_wing.json --duration 6 --vary anchor:wrist_tendon.a.y=-0.01:0.02:7 --vary rest:wrist_tendon=1.0:1.02:11
```

```
rows anchor:wrist_tendon.a.y, columns rest:wrist_tendon; L locked, x collapsed.
columns: 1, 1.002, 1.004, 1.006, 1.008, 1.01, 1.012, 1.014, 1.016, 1.018, 1.02
         -0.01 | x x x x x x x x x x x
        -0.005 | x x x x x x x x x x x
             0 | L L L L L x x x x x x
         0.005 | L L L L L L L L L L L
          0.01 | L L L L L L L L L L L
         0.015 | L L L L L L L L L L L
          0.02 | L L L L L L L L L L L
```

The post's height decides whether the lock rides out the gust. Lowered 5 mm, at the same tension, the wing loses the hand even with its tendon at design length. Raised 5 mm, it holds across the whole 2% range. A higher post gives the tendon a longer moment arm, which delays the fold.

Computed at commit `1d70c27`.
