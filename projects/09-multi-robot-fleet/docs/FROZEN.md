# 009 — frozen inputs

Frozen at **2026-09-19T01:50:53Z** on `GSYY`.

There is no git repository in this project (it was scaffolded from a handoff
bundle), so "these are the inputs" is written down as a hash list instead of a
commit id. `scripts/verify_independence.sh` re-derives this list inside a
temporary copy of the tree; if the two disagree, the copy was not the same project.

## Environment

| | |
|---|---|
| host | `GSYY` |
| kernel | `6.6.87.2-microsoft-standard-WSL2` |
| python | `3.14.4` |
| ROS | `lyrical` |
| ROS_DOMAIN_ID | `` |
| RMW | `` |
| gz sim | `10.4.0` |

## Static guards run by `build.sh`

- `scripts/check_batch_manifest.py`
- `scripts/check_launch_robots.py`
- `scripts/check_pose_freshness.py`
- `scripts/check_ros_callback_arity.py`
- `scripts/check_ros_params.py`
- `scripts/check_scenario_budgets.py`
- `scripts/check_truth_liveness.py`
- `scripts/check_undefined_names.py`
- `scripts/check_xml_wellformed.py`
- `scripts/validate_assets.py`
- `scripts/validate_traffic_geometry.py`

## Scenario budgets declared at freeze time

These are the numbers a scenario states up front. `fleet.launch.py` applies them
and prints them, so a run's log says which of them were in force.

### `f_fault_triggers.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=120.0`, `max_retries=1`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `crossing_queue_s=90.0`, `crossing_timeout_s=300.0`, `leg_timeout_s=120.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=1.0`
- **r02** (per robot): `start_battery_fraction=1.0`
- **r03** (per robot): `start_battery_fraction=1.0`

### `n01_crossing.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `crossing_timeout_s=180.0`, `leg_timeout_s=240.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=1.0`

### `n02_transfer_round_trip.yaml`

- **leg_executor**: `leg_timeout_s=180.0`, `max_retries=2`
- **task_service**: `leg_timeout_s=180.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=1.0`

### `n03_two_halves.yaml`

- **leg_executor**: `leg_timeout_s=180.0`, `max_retries=2`
- **task_service**: `leg_timeout_s=180.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=1.0`
- **r02** (per robot): `start_battery_fraction=1.0`

### `n04_face_to_face_r01_first.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `crossing_queue_s=90.0`, `crossing_timeout_s=300.0`, `leg_timeout_s=240.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=1.0`
- **r02** (per robot): `start_battery_fraction=1.0`

### `n05_face_to_face_r02_first.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `crossing_queue_s=90.0`, `crossing_timeout_s=300.0`, `leg_timeout_s=240.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=1.0`
- **r02** (per robot): `start_battery_fraction=1.0`

### `n06_two_west_queue.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `crossing_queue_s=90.0`, `crossing_timeout_s=300.0`, `leg_timeout_s=240.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=1.0`
- **r03** (per robot): `start_battery_fraction=1.0`

### `n07_three_robots_six_tasks.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `crossing_queue_s=90.0`, `crossing_timeout_s=300.0`, `leg_timeout_s=240.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=1.0`
- **r02** (per robot): `start_battery_fraction=1.0`
- **r03** (per robot): `start_battery_fraction=1.0`

### `n08_three_robots_task_stream.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=180.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `crossing_queue_s=90.0`, `crossing_timeout_s=300.0`, `leg_timeout_s=180.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=0.18`
- **r02** (per robot): `start_battery_fraction=1.0`
- **r03** (per robot): `start_battery_fraction=0.18`

### `p5_capability.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `leg_timeout_s=240.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`

### `p5_charge_queue.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `leg_timeout_s=240.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=0.15`
- **r02** (per robot): `start_battery_fraction=0.13`
- **r03** (per robot): `start_battery_fraction=0.11`

### `p5_low_battery.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `leg_timeout_s=240.0`, `orphan_grace_s=30.0`, `tick_hz=2.0`
- **r01** (per robot): `start_battery_fraction=0.15`

### `p5_restart.yaml`

- **coordinator**: `occupancy_settle_s=5.0`, `permit_rate_hz=5.0`, `pose_timeout_s=1.5`
- **leg_executor**: `leg_timeout_s=240.0`, `max_retries=2`, `publish_hz=10.0`
- **task_service**: `cancel_confirm_s=15.0`, `leg_timeout_s=240.0`, `orphan_grace_s=45.0`, `tick_hz=2.0`

### `regression_v1.yaml`


## Files (248)

`sha256` of every input that defines behaviour. Outputs (`reports/`, `runtime/`,
`build/`, `install/`, `log/`) are deliberately excluded: a manifest that changes
whenever a run happens measures nothing.

| file | sha256 |
|---|---|
| `AGENTS.md` | `8e08f61f6bd4fd83…` |
| `LICENSE` | `a60021c6f197d1b8…` |
| `README.md` | `6a9ec6d041397a2a…` |
| `THIRD_PARTY_NOTICES.md` | `72fdb531f74f2416…` |
| `assets/.gitkeep` | `e3b0c44298fc1c14…` |
| `assets/maps/warehouse.pgm` | `a9d64bdb7d815bbe…` |
| `assets/maps/warehouse.yaml` | `449486bb9080cd16…` |
| `assets/models/amr_4wd/model.config` | `cb47b8d178f0abce…` |
| `assets/models/amr_4wd/model.sdf.in` | `3f50c4ff8ee451fe…` |
| `assets/rviz/fleet.rviz` | `1bca042e737eece3…` |
| `assets/worlds/warehouse.sdf` | `7b7d3f6a49ee18ee…` |
| `config/.gitkeep` | `e3b0c44298fc1c14…` |
| `config/cyclonedds.xml` | `ec59c6d9cde319ae…` |
| `config/fleet.yaml` | `de62804384251152…` |
| `config/resources.yaml` | `974ff6226e2c1482…` |
| `config/scenarios/f_fault_triggers.yaml` | `7c2b644052d9a163…` |
| `config/scenarios/n01_crossing.yaml` | `38fe831067b2fef5…` |
| `config/scenarios/n02_transfer_round_trip.yaml` | `3cbbad6d77895a44…` |
| `config/scenarios/n03_two_halves.yaml` | `b267b6d2d80c7630…` |
| `config/scenarios/n04_face_to_face_r01_first.yaml` | `cfc770a4df5fd808…` |
| `config/scenarios/n05_face_to_face_r02_first.yaml` | `ad2928054b3b6abd…` |
| `config/scenarios/n06_two_west_queue.yaml` | `3f23887ae7ea7e06…` |
| `config/scenarios/n07_three_robots_six_tasks.yaml` | `3c4c063f0d22f989…` |
| `config/scenarios/n08_three_robots_task_stream.yaml` | `ff2aa7bbdde8b385…` |
| `config/scenarios/p5_capability.yaml` | `3364b4f4f720b4f9…` |
| `config/scenarios/p5_charge_queue.yaml` | `fde9b523c3bb3f40…` |
| `config/scenarios/p5_low_battery.yaml` | `0ba15a7469ac3dd4…` |
| `config/scenarios/p5_restart.yaml` | `7ddc59ddb14192ff…` |
| `config/scenarios/regression_v1.yaml` | `439663365621e435…` |
| `config/scenarios/regression_v1.yaml.rebuilt_cases` | `19453b15de0b604d…` |
| `config/spawns.yaml` | `14084429fb807af8…` |
| `docs/AI_EXECUTION_PROMPTS.md` | `02860c6db61bd803…` |
| `docs/ARCHITECTURE.md` | `975a774ac9f64b2d…` |
| `docs/CONTRACTS.md` | `7c2dcbb3f2548ec6…` |
| `docs/DECISIONS.md` | `703e4bcc952722b8…` |
| `docs/ENVIRONMENT.md` | `6bb1ab95383ede8c…` |
| `docs/HANDOFF_README.md` | `4034e3ff88d3c46d…` |
| `docs/IMPLEMENTATION_STATUS.md` | `f08390725204f109…` |
| `docs/LIMITATIONS.md` | `d34e2a3bc3399754…` |
| `docs/MASTER_PLAN.md` | `e93007181de259f6…` |
| `docs/TEST_AND_ACCEPTANCE.md` | `e027156c14ff9e23…` |
| `docs/TROUBLESHOOTING.md` | `ea3f7185c4c56346…` |
| `pyproject.toml` | `df40289d6762bef4…` |
| `run_demo.sh` | `b2ce23c20e947fea…` |
| `scripts/_dds_capacity_node.py` | `1b76f46e99b276f5…` |
| `scripts/_reverify.sh` | `8254f0e8a810a998…` |
| `scripts/acceptance_p4.py` | `a4e6268e8de666e1…` |
| `scripts/acceptance_p4.sh` | `a53aec64f9cb9e8a…` |
| `scripts/analyze_gz_stats.py` | `939bb88b27aaab93…` |
| `scripts/batch.py` | `a62da86f2f19f310…` |
| `scripts/batch.sh` | `e69138585238c16d…` |
| `scripts/build.sh` | `80928851693680c2…` |
| `scripts/check_asking_points.py` | `7f5c21139ae62d2a…` |
| `scripts/check_batch_manifest.py` | `71948db9c28c8399…` |
| `scripts/check_fleet_isolation.py` | `fe130f98cb3a67aa…` |
| `scripts/check_guards.sh` | `903444b30e51972a…` |
| `scripts/check_launch_robots.py` | `76b45c4aa7c464d1…` |
| `scripts/check_p6_dashboard.sh` | `aba8c4a1940ed517…` |
| `scripts/check_p6_live.py` | `c291f87d82a6973f…` |
| `scripts/check_p6_live.sh` | `6a670407409f964d…` |
| `scripts/check_p7_gates.py` | `b7039468d08eb5ad…` |
| `scripts/check_pose_freshness.py` | `fac7ae6a053dd3e5…` |
| `scripts/check_ros_callback_arity.py` | `2c6ad463f6ad1c8d…` |
| `scripts/check_ros_params.py` | `786bfe19e0706f8a…` |
| `scripts/check_scenario_budgets.py` | `79240a7c0bd0f0c0…` |
| `scripts/check_truth_liveness.py` | `0ba24e35cb9202e3…` |
| `scripts/check_undefined_names.py` | `2e1be21ffb7706e0…` |
| `scripts/check_xml_wellformed.py` | `81cd1cb4cb02f4f5…` |
| `scripts/doctor.sh` | `fe8ba812ba5a2a28…` |
| `scripts/dump_truth_all.py` | `d68944c229fbdda8…` |
| `scripts/dump_truth_message.py` | `c2213f93591147d4…` |
| `scripts/env.sh` | `bba96ac43a57292b…` |
| `scripts/fake_demo.py` | `8fbd1d3c24c3235f…` |
| `scripts/fix_sdf_comments.py` | `c75938f81a41cd00…` |
| `scripts/freeze_manifest.py` | `c89a52499696de18…` |
| `scripts/gen_nav2_params.py` | `7325b57d9f534bcf…` |
| `scripts/inspect_graph.py` | `1116c319152cb32a…` |
| `scripts/nav_goal.py` | `fa22b326af626f69…` |
| `scripts/nav_goal_measured.py` | `8ebfaa3a97808a3a…` |
| `scripts/nudge.py` | `a2f955d156b8f29e…` |
| `scripts/probe_acquire.py` | `27105ceea0b368e6…` |
| `scripts/probe_corridor_plan.py` | `60fced6a8c4eee35…` |
| `scripts/probe_crossing.py` | `7a4c9738b194a110…` |
| `scripts/probe_gate_state.py` | `d7c0bab52858378e…` |
| `scripts/probe_gate_state.sh` | `01ce585aa831398b…` |
| `scripts/probe_ground_truth.py` | `5ce91050c68513b7…` |
| `scripts/probe_odom_truth.py` | `5612e8598cca0b83…` |
| `scripts/probe_pipeline_lag.py` | `ce3bb2ef19b68ce6…` |
| `scripts/probe_pose_lag.py` | `fd8c05602fc7eb4d…` |
| `scripts/probe_recorder_stop.py` | `30fb2ebd53e38cb3…` |
| `scripts/probe_recorder_stop.sh` | `6c356e8daae11980…` |
| `scripts/probe_scale.py` | `7e2edf2053063153…` |
| `scripts/probe_scan_qos.py` | `97b24def04e21f50…` |
| `scripts/probe_scan_tf.py` | `2422ef02581b83f7…` |
| `scripts/probe_truth_slots.py` | `46728827edc34eec…` |
| `scripts/probe_truth_slots.sh` | `c009050642bafd8f…` |
| `scripts/render_model.py` | `14cabf9071380bfe…` |
| `scripts/run_demo.sh` | `d8995b6209be2d89…` |
| `scripts/run_p5.sh` | `0b1bf25eac08f24e…` |
| `scripts/run_p7_matrix.sh` | `553931dbf706af9a…` |
| `scripts/run_scenarios12.py` | `08ea70ba89a22a62…` |
| `scripts/run_scenarios3456.py` | `db3510fc63ee622b…` |
| `scripts/run_scenarios3456.sh` | `e4a130e87358b3dd…` |
| `scripts/sample_pose.py` | `9a07233e9d1deb34…` |
| `scripts/stop_demo.sh` | `20b2b0b578285ec8…` |
| `scripts/test_core.sh` | `baf80d193e82f3ce…` |
| `scripts/validate_assets.py` | `21ffa0feaba40a5e…` |
| `scripts/validate_traffic_geometry.py` | `1a4648a5db29b824…` |
| `scripts/verify_independence.sh` | `5290423c2e234197…` |
| `src/fleet_adapter/CMakeLists.txt` | `333e8cb2cf95cb77…` |
| `src/fleet_adapter/fleet_adapter/__init__.py` | `2816cd8196f60011…` |
| `src/fleet_adapter/fleet_adapter/adapter_base.py` | `d65011a23ac288f0…` |
| `src/fleet_adapter/fleet_adapter/fake_adapter.py` | `db0c61945f9168e9…` |
| `src/fleet_adapter/fleet_adapter/nav2_readiness.py` | `30e2bd2361b2273d…` |
| `src/fleet_adapter/fleet_adapter/safety_gate.py` | `2a5fc0d4112a75af…` |
| `src/fleet_adapter/fleet_adapter/zone_guard.py` | `1c38adbe07685528…` |
| `src/fleet_adapter/package.xml` | `268471bf8ce0718b…` |
| `src/fleet_adapter/setup.py` | `a3ae2fe0714b9ec1…` |
| `src/fleet_bringup/CMakeLists.txt` | `c4bb19e89f06f89c…` |
| `src/fleet_bringup/config/nav2_params.yaml` | `7b9baf5f7d6ab6c3…` |
| `src/fleet_bringup/launch/_common.py` | `7abe74b01979d829…` |
| `src/fleet_bringup/launch/fleet.launch.py` | `153c2edbc700f4b0…` |
| `src/fleet_bringup/launch/robot.launch.py` | `c19e153aa46f5fe9…` |
| `src/fleet_bringup/launch/world.launch.py` | `87694cd5defabad2…` |
| `src/fleet_bringup/package.xml` | `e407aba0a937e020…` |
| `src/fleet_bringup/urdf/amr_4wd.urdf` | `f7ecc55b2e38dcb9…` |
| `src/fleet_core/CMakeLists.txt` | `70f2621b699f4a2f…` |
| `src/fleet_core/fleet_core/__init__.py` | `85140e70343fd876…` |
| `src/fleet_core/fleet_core/allocator.py` | `efd8bfdbf8859f0f…` |
| `src/fleet_core/fleet_core/battery.py` | `58770f1e9d5c8c5b…` |
| `src/fleet_core/fleet_core/charging.py` | `9a442471b0ab4d0c…` |
| `src/fleet_core/fleet_core/clock.py` | `3de1ab5ae9dfd565…` |
| `src/fleet_core/fleet_core/crossing_ledger.py` | `17eb03f4f48ce7db…` |
| `src/fleet_core/fleet_core/crossing_release.py` | `084354464b3a0384…` |
| `src/fleet_core/fleet_core/domain.py` | `b9720db97d31018f…` |
| `src/fleet_core/fleet_core/events.py` | `cd6a4e8795dc09db…` |
| `src/fleet_core/fleet_core/geometry.py` | `40cdbccd451ff83d…` |
| `src/fleet_core/fleet_core/ledger.py` | `746841c40fc19b6c…` |
| `src/fleet_core/fleet_core/legs.py` | `26afccc19b9defb6…` |
| `src/fleet_core/fleet_core/loaded_battery_policy.py` | `72ea160ad8a82ad5…` |
| `src/fleet_core/fleet_core/pose_source.py` | `b7195f3225673d80…` |
| `src/fleet_core/fleet_core/recovery.py` | `6af06100f6eccff0…` |
| `src/fleet_core/fleet_core/resources.py` | `d3e2737f8ff6a0b7…` |
| `src/fleet_core/fleet_core/retreat_policy.py` | `7a72d5feccc48240…` |
| `src/fleet_core/fleet_core/stage2_capabilities.py` | `356b582e7ef9fd6a…` |
| `src/fleet_core/fleet_core/task_machine.py` | `cce5fbc0420bdf4e…` |
| `src/fleet_core/fleet_core/traffic.py` | `2f9dc31858ed8da8…` |
| `src/fleet_core/fleet_core/trigger.py` | `68b25bcc5d32fe4d…` |
| `src/fleet_core/fleet_core/validation.py` | `2c6e602a9cfa674d…` |
| `src/fleet_core/fleet_core/wait_for.py` | `9e7e0e277dbf6e18…` |
| `src/fleet_core/package.xml` | `fd18aae191b074ea…` |
| `src/fleet_core/setup.py` | `fed1b63e4e22e01b…` |
| `src/fleet_evaluation/CMakeLists.txt` | `86b6eb5ca610c57e…` |
| `src/fleet_evaluation/fleet_evaluation/__init__.py` | `5fcaa580dc0ae180…` |
| `src/fleet_evaluation/fleet_evaluation/judge.py` | `df1f2841acf7677c…` |
| `src/fleet_evaluation/fleet_evaluation/recorder.py` | `313b38c271b59a97…` |
| `src/fleet_evaluation/package.xml` | `e627afd1759b5bf3…` |
| `src/fleet_evaluation/scripts/fleet_eval` | `e4956f4be472c846…` |
| `src/fleet_evaluation/scripts/fleet_recorder` | `d1f43f8eed60b8c9…` |
| `src/fleet_interfaces/CMakeLists.txt` | `69cc888879ae9b98…` |
| `src/fleet_interfaces/action/ExecuteLeg.action` | `238adf0e639442a5…` |
| `src/fleet_interfaces/msg/ResourcePermit.msg` | `3c1da569bb87c16b…` |
| `src/fleet_interfaces/msg/RobotState.msg` | `567d3ad24a095d0a…` |
| `src/fleet_interfaces/package.xml` | `32a7ff33e35056d3…` |
| `src/fleet_interfaces/srv/AcquirePassage.srv` | `f7da0e323c648c33…` |
| `src/fleet_interfaces/srv/CancelTask.srv` | `c4a80aa1009d9ff3…` |
| `src/fleet_interfaces/srv/ConfirmClear.srv` | `29bdd0c577a2a911…` |
| `src/fleet_interfaces/srv/FaultInject.srv` | `6083d1c14e13816b…` |
| `src/fleet_interfaces/srv/RenewPermit.srv` | `0e38b5d04eead8ed…` |
| `src/fleet_interfaces/srv/SubmitTask.srv` | `de2fa7e9bf580d97…` |
| `src/fleet_ros/CMakeLists.txt` | `7a5a1e47589fbc4d…` |
| `src/fleet_ros/fleet_ros/__init__.py` | `0d7c3231ff602cb5…` |
| `src/fleet_ros/fleet_ros/coordinator_node.py` | `4513e47525afec3b…` |
| `src/fleet_ros/fleet_ros/gate_node.py` | `0a66ec3c0ed147a3…` |
| `src/fleet_ros/fleet_ros/nav2_adapter_node.py` | `bfb61d46f475c2a5…` |
| `src/fleet_ros/fleet_ros/staged_crossing.py` | `cf02e1037e267c0b…` |
| `src/fleet_ros/fleet_ros/stop_distance_calibrator.py` | `15389495257c6c4c…` |
| `src/fleet_ros/fleet_ros/task_service_node.py` | `a16673340143855d…` |
| `src/fleet_ros/package.xml` | `0ff93a834278495a…` |
| `src/fleet_ros/scripts/coordinator_node` | `bf22a4e98d280850…` |
| `src/fleet_ros/scripts/gate_node` | `6fb52a3a512ef48a…` |
| `src/fleet_ros/scripts/nav2_adapter_node` | `e949af5a0f910821…` |
| `src/fleet_ros/scripts/staged_crossing` | `64b9fb87c04456de…` |
| `src/fleet_ros/scripts/stop_distance_calibrator` | `69d022bbf8fd8beb…` |
| `src/fleet_ros/scripts/task_service_node` | `b56dc951dc3ee9c6…` |
| `src/fleet_ros/setup.py` | `1b49d9b4559a2615…` |
| `src/fleet_tools/CMakeLists.txt` | `31d160160e8d1fd2…` |
| `src/fleet_tools/fleet_tools/__init__.py` | `0dc459e0ee48aed4…` |
| `src/fleet_tools/fleet_tools/cli.py` | `70fbae1791c55b40…` |
| `src/fleet_tools/fleet_tools/dashboard.py` | `5064618b2ab37ac4…` |
| `src/fleet_tools/fleet_tools/display.py` | `c25eb0a7d89aed41…` |
| `src/fleet_tools/package.xml` | `4541892804a385ce…` |
| `src/fleet_tools/scripts/fleet_cli` | `de1f8270cc7a9d41…` |
| `src/fleet_tools/scripts/fleet_dashboard` | `fad461bb75393d84…` |
| `tests/.gitkeep` | `e3b0c44298fc1c14…` |
| `tests/conftest.py` | `897b29a21bf0067e…` |
| `tests/test_eval_judge.py` | `ae74ad7f15ecd734…` |
| `tests/test_guard_scope.py` | `d43c3ea546cd78f1…` |
| `tests/test_p10_one_boundary.py` | `b098b54c13d001d1…` |
| `tests/test_p10_truth_slots.py` | `95c09804e37f3dca…` |
| `tests/test_p11_asking_points.py` | `177739490bf31424…` |
| `tests/test_p11_pose_source.py` | `c2a8193288dfa358…` |
| `tests/test_p12_pose_source.py` | `b43025cfd29b4b06…` |
| `tests/test_p14_crossing_release.py` | `1934f0db151ad004…` |
| `tests/test_p14_retreat_policy.py` | `a5bbe7ad467fb601…` |
| `tests/test_p15_refresh.py` | `c000dcd7bf66c7f5…` |
| `tests/test_p16_crossing_ledger.py` | `6814dce06ff33421…` |
| `tests/test_p16_wait_for.py` | `b6df60b64ecd9cb9…` |
| `tests/test_p16c_queue_budget.py` | `9106f38ddbec0361…` |
| `tests/test_p17_gate_pose_instrument.py` | `98e75771422a9973…` |
| `tests/test_p17_launch_robots.py` | `8cf57ce8c0387804…` |
| `tests/test_p17_loaded_battery.py` | `fed62269cee40c93…` |
| `tests/test_p17_nav_feedback.py` | `77a6a1670db99701…` |
| `tests/test_p17_reserve_and_infra.py` | `9d256119776e7111…` |
| `tests/test_p17_trigger.py` | `d49a9b930c8adcf0…` |
| `tests/test_p17_truth_liveness.py` | `3a3b9bfbf94270db…` |
| `tests/test_p1_adapter_no_ros.py` | `c50bf89d6bfb773f…` |
| `tests/test_p1_allocator.py` | `3a708e1507070858…` |
| `tests/test_p1_allocator_reason.py` | `4da3e734ffafa57a…` |
| `tests/test_p1_clock.py` | `fce0df4786545c66…` |
| `tests/test_p1_ledger.py` | `48cb83015f6719d1…` |
| `tests/test_p1_machine.py` | `6b84787646e060b3…` |
| `tests/test_p1_resources.py` | `c9005effa671ae74…` |
| `tests/test_p1_validation.py` | `1d52de6c4ff64e6a…` |
| `tests/test_p2_safety_gate.py` | `44cc9c18ac3b3db2…` |
| `tests/test_p4_epoch.py` | `c87dbb6d6bdd6c3c…` |
| `tests/test_p4_frames.py` | `2dd25ec45f5ea72c…` |
| `tests/test_p4_gate.py` | `b62c376400affc44…` |
| `tests/test_p4_gate_wiring.py` | `0bf42a2192d29152…` |
| `tests/test_p4_geometry.py` | `c700e07869029206…` |
| `tests/test_p4_mirror.py` | `d4e70948b5c1c344…` |
| `tests/test_p4_release.py` | `0ee226c8131c907c…` |
| `tests/test_p4_traffic.py` | `ffd12752dbd06d78…` |
| `tests/test_p5_charge_id.py` | `462abbf532cd9c4e…` |
| `tests/test_p5_core.py` | `5a9339609417061f…` |
| `tests/test_p5_crossing_from_tasks.py` | `6328b4d9769d1207…` |
| `tests/test_p5_custody.py` | `6c5baa37e067aaeb…` |
| `tests/test_p5_ledger_kind.py` | `edb60a994eadae78…` |
| `tests/test_p5_localization_gate.py` | `858010ca4809a836…` |
| `tests/test_p5_nav2_readiness.py` | `d4fae66a3d4416da…` |
| `tests/test_p5_reachable_pads.py` | `6d272118213b00a0…` |
| `tests/test_p5_restart_labels.py` | `63460607a43f39b2…` |
| `tests/test_p5_scenarios.py` | `3b1b1cf67847f8c9…` |
| `tests/test_p6_display.py` | `969b002da07d0b2c…` |
| `tests/test_p7_batch_manifest.py` | `c800bf7c2fea868e…` |
| `tests/test_p7_batch_truth.py` | `8842429557a6202e…` |
| `tests/test_p8_gate_state_wiring.py` | `69e3b8694b72916a…` |
| `tests/test_stage2_capabilities.py` | `b5c92918c7d0a3db…` |
