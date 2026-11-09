# Topologies

Planar topologies as JSON (PLAN.md 9.7). Each file describes the links (role, mass, inertia, length, initial pose and velocity), pins, cables (rest length, axial stiffness, damping), gravity, constraint stabilization, and a `simulation` block with the default `dt`, `duration`, `frame_stride` and `event_tolerance`. Command-line flags override the simulation block, and the run manifest records the values actually used.

Run one with:

```
dotnet run --project src/Tensegrity.Cli -- run --topology topologies/planar_x.json --seed 42
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
| `locking_arm_collapse.json` | The same arm with its elbow tendon 10% too long, past the lock threshold. Released from straight out, the lower link collapses. |

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
| locking_arm_collapse | 1.8e-12 | 14.9, 10.6, 10.6 | 5.2e-13 J | 9 at every level |

The locking arm rows were added at M5 (after commit `67acd3b`). The collapsing arm falls by about 10.6 rather than 16 per halving below 2e-4 s. Its 9 tendon slack/taut switches are located to 1e-12 s, and the trajectory swings through large angles, so some error does not scale with dt⁴. The change at 1e-4 s is still 1.8e-12, far below float32 precision.

Why 1e-4 s:

- **Converged.** Halving the step changes no position or angle by more than 6e-12 m or rad. That is at least 10⁴ times below the float32 precision the frames are stored in (about 6e-8 at 1 m).
- **In RK4's asymptotic range.** The change falls by about 16 per halving from 4e-4 s down, until it reaches the rounding floor. Event-splitting keeps the cable oscillator at this order, and it finds the same 9 switches at every step size.
- **Energy of the X.** The X's energy balance error falls more slowly at the larger steps: 7.5–8× from 4e-4 s, then 12.6–12.8×, then 14.4–15×. Its stiff cable modes (ω·dt ≈ 0.03) are still entering the asymptotic range there. The trajectory is already asymptotic, and the balance error is at most 3.2e-10 J, so the X needs no smaller step.
- **Consistent.** All golden baselines and test tolerances were set at 1e-4 s.

Rerun the study when a topology changes, and record the new numbers here.
