# RoboSub 2026 run scoring

Point values and supported limits live in [scoring.yaml](../tasks/2026/config/scoring.yaml).
Run options and their defaults live in [competition.yaml](../tasks/2026/competition.yaml);
heading and role coin flips start checked. New scoring conditions belong in
[the year's behavior](../tasks/2026/behavior/scoring.py).

Open **Run score** in the pool viewer. Select an intended role, then **Start run**.
This clears the task scene, reloads/disarms the actuators, resets the ledger and
starts an individual-run stopwatch. The robot stays where it is. Enable/arm the
robot using the existing controls. **Stop run** freezes both the stopwatch and
automatic awards. **Reset run & tasks** clears everything back to zero.

The stopwatch uses integrated simulation time, so it pauses with physics and is
independent of rendering speed. It has no automatic duration limit. A breach
outside the octagon ends *scoring* after the robot has submerged; the stopwatch
continues until manually stopped. The viewport always shows the points/time;
**Detailed scorecard** shows every award and a signed manual adjustment. You can
also launch the viewer with `show_scorecard:=true`.

## Rules and team clarifications

Sources read for this implementation:

- [2026 task descriptions, including PVC octagon](https://robonation.gitbook.io/robosub-resources/section-3-autonomy-challenge/3.2-task-descriptions)
- [2026 autonomy scoring and attempt rules](https://robonation.gitbook.io/robosub-resources/section-4-scoring-and-awards/4.2-autonomy-challenge-scoring)
- [Competition timing](https://robonation.gitbook.io/robosub-resources/section-3-autonomy-challenge/3.4-competition-sequence-of-events)
- [Vehicle requirements](https://robonation.gitbook.io/robosub-resources/section-5-rules-and-requirements/5.3-vehicle-requirements)
- [Marker/torpedo specifications](https://robonation.gitbook.io/robosub-resources/section-5-rules-and-requirements/5.4-competition-specifications)

Team clarifications (2026-09-18): the torpedo sequence adds **1400** to the two
600-point hits; both baskets count for the octagon facing/rotation signal;
surfacing, release and placement awards accumulate separately. Referee decisions
and weight scoring are manual for now. Only individual runs are timed: no
semifinal/final session history, countdown, or automatic remaining-time bonus.
The ledger reports time-bonus prerequisites for future session integration.

| Award | Automatic scoring |
| --- | --- |
| Gate | 100; heading coin flip +300 when confirmed before start; matched random role +150 |
| Gate style | Net 90-degree increments: yaw 100, roll/pitch 200, eight increments total |
| Each of three slalom rows | Any side 200 or matching gate side 400; depth overlap +200 |
| Markers | Any bin 300; correct-role distinct bin 800; at most two released markers |
| Magnet lights | 500 each, two distinct lights maximum |
| Torpedo openings | 600 each, at most two released torpedoes; either role's hole earns base points |
| Torpedo sequence | +1400 for the selected role's large then small opening, in launch order |
| Torpedo distance | +200 at 1 ft, or +400 at 1.5 ft, per successful torpedo |
| Surface within octagon | 800 |
| Surface facing sign | Any 200, own role 400, own role and matching basket count 700 |
| Surface with constrained object | 400 once per object |
| Release object | 200 once per grasped object released freely |
| Basket placement | Any 500; object in its matching basket 700 regardless of robot role, once per object |
| Signal basket count | Completed yaw turns within one of the current count: 500; exact: 1000 |
| Return home | 300 for a fully underwater reverse gate traversal after the initial pass |
| Random pinger | Future event hooks: first task 500, other task 1500 |

Higher tiers replace lower tiers; repeated events do not add duplicate points.
Two markers in the same correct bin yield 800+300, while one in each correct bin
yields 800+800. The first two releases of each payload kind are the scored rounds;
reloads within a run cannot create extra attempts. Pre-gate releases never score.
The first complete gate traversal fixes the scored role for that run, regardless
of the intended role. It also changes the existing payload success/wrong-target
classification. Repair selects fire and repair objects; Rescue selects blood and
medical objects. Select the already-known role before starting and check **Role
coin flip** beside **Heading coin flip** if it was randomly assigned. Its bonus
requires matching it at the gate; the simulator never rerolls the role. A heading coin flip is an attestation checkbox,
not an automatic teleport/reorientation of the robot.

Table inventory is based on settled, actual basket contacts, including both
baskets. Picking an object out removes it from the current count; previously
earned placement points remain. One object means face compass (repair) or buoy
(rescue); two or more means hammer/wrench or SOS. Zero objects earns at most the
own-role facing tier. Every unique object may independently earn its surfacing,
release and basket awards. Regrasping cannot duplicate them.

Restore clarification (2026-09-20): surfacing earns **800**, plus a separate
facing bonus of **200 / 400 / 700** (the highest applicable tier). Each object
independently earns **200** for pickup and release, plus **500 / 700** for basket
placement, plus **400** for surfacing with it. Correct sorting earns 700 for
either role's objects. For example, surfacing facing the correct sign and
surfacing with, releasing, and correctly sorting two objects earns
800 + 700 + 2 × (400 + 200 + 700) = **4100** Restore points, before the separate
500 / 1000 basket-count signal award.

## Geometry and simulation conventions

The octagon has eight white cylindrical PVC sides at water level, with 9 ft
(2.7432 m) between opposite sides, consistent with the existing sign positions
and legacy octagon collision dimensions. Half-inch nominal PVC uses a 0.84-inch
outside diameter. Existing inward-facing sign meshes remain at their mapped
positions, suspended below the ring. The renderer and physical collision proxies
use the same `octagon` dimensions from [the 2026 task config](../tasks/2026/config/tasks.yaml); the physical proxies
are narrow boxes around the pipes. No acoustic pinger is created.

The deterministic judge uses the plant's box collision envelope (COM coordinates
converted to base-link coordinates), plus the magnet tip. It is an approximation
to the vehicle's silhouette. Gate passage requires the whole envelope to move
from the approach side (+X in the gate frame) to the far side, within the gate
width and below the crossbar. Returning home uses the reverse traversal.
Slalom checks forward plane crossings within the row width and overlap with the
CAD row's vertical extent. Rows retain their highest award. Gate style is net
body-axis rotation during the 3 m gate attempt, finalized when leaving that
area or pressing Stop; reversing an unfinished rotation cancels it.
Pose jumps over 2 m discard the geometry history so teleporting between tasks
cannot sweep through a gate/slalom or manufacture rotations/surfacing points.

For surfacing, the entire projected envelope must be inside the octagon. The top
of the envelope must reach water level for 0.5 s after previously submerging.
This models surfacing at the waterline, not lifting the entire robot out of the
water. Facing means horizontal bow bearing within 15 degrees of the sign for
0.5 s. Basket signaling counts net full yaw turns within the table's 3 m task
buffer, underwater or at the surface, with a 15-degree turn tolerance and a
stationary-heading dwell. Leaving that buffer or changing the basket count
starts a new rotation measurement. Zero turns does not
earn the near-count award. These tolerances are simulator conventions, exposed
under `scoring` in the task YAML, not additional official rules.

Distance bonuses use the launched torpedo's nose-to-board-plane distance at
release, with exact foot conversions (0.3048 / 0.4572 m). Blocked shots receive no
automatic partial credit. Gate control, partial surfacing/capture/release,
weight/specification penalties and inter-vehicle communication are entered via
the manual adjustment. The field replaces the prior adjustment and accepts
negative values. Neither partial-credit judgments nor an official competition
total are claimed from simulated geometry alone.

## ROS interfaces

Relative to `/talos`:

- `simulator/run_score` (`std_msgs/String`): JSON containing `total`, `rows`,
  `elapsed`, `running`, `scoring_open`, roles, basket count, pinger state and status.
- `simulator/run_command` (`std_msgs/String`): JSON commands below. Rejections
  appear in `run_score.message` and the ROS log without clearing the ledger.
- `simulator/task_score`: retains the original success/wrong-target/miss counters
  for compatibility. It is diagnostic, not the handbook point total.
- `simulator/reset_tasks`: existing topic/service also resets the run ledger and
  stopwatch. Individual table/light resets are rejected during a scored run.

```json
{"action":"start","role":"repair","heading_coin":false,"role_coin":false}
{"action":"stop"}
{"action":"adjustment","points":150}
```

`role` accepts `repair` or `rescue`; `role_coin` records a role coin flip performed
before the run. Starting while a run is active is
rejected. Clock rewind clears the run rather than producing negative elapsed time.

Future acoustic integration may send these commands, after Start and before
earning points at either pinger task:

```json
{"action":"pinger_select","task":"restore","random":true}
{"action":"pinger_switch"}
```

`task` is `deploy` or `restore`. The future controller supplies the randomized
first selection. There is no award for merely detecting a ping; an eligible task
award must follow selection. Switching before scoring the first random task
reverts to specific-pinger mode with no random bonus. The second bonus requires
new points at the other task after switching. Repeated switches cannot earn the
second-task bonus by scoring again at the first task. Hooks default to disabled.

## Validation

```bash
python3 src/riptide_simulator/c_simulator/test/test_run_score.py
ROS_DOMAIN_ID=176 python3 src/riptide_simulator/c_simulator/test/ros_run_score_smoke.py
```

The ROS test requires a built, sourced workspace and runs its own task node in
an isolated domain. Optional `--capture /tmp/scorecard.png` starts the viewer to
capture the populated scorecard (requires an OpenGL display). Unit tests cover
award tiers, caps, duplicate events, gate prerequisite, roles, sequence order,
table accumulation, pinger switches, manual timing and geometry detection.
