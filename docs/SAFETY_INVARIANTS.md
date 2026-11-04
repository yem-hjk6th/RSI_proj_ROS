# Safety invariants: enforcing code and tests

This audit covers the six invariants in `CLAUDE.md` and every clamp, latch and start refusal that the bridges, the
controller and the session start-up implement. Each row names the code that enforces the rule and the test(s) that
prove it.

**Paths.**
- Code `pkg/module.py:N` means `src/pkg/pkg/module.py`, line N. The exceptions are `krc_templates/...` and
  `config/...`, which are inside `src/krc_eki_bridge/`.
- Tests `pkg/test_x.py::name` means `src/pkg/test/test_x.py`, test `name`.

**Status.**
- **covered**: a test existed before this audit.
- **added**: the test was added by this audit (`test_invariant_audit.py` in each package).
- **violated (#N)**: the new test is `xfail(strict=True)` and issue #N describes it.

The *(ROS)* mark means the test needs `rclpy` and runs in CI only.

Nothing here has run on hardware. Every result comes from the simulator (`krc_sim`), the fake EKI server, or the
logged sessions in `data/`.

## Violations found

| Invariant | Input | Observed | Issue |
|---|---|---|---|
| 3 (EKI bridge) | `OverrideConfig(wd_s=inf)`, last command 90 at t = 0 | accepted; `output(3600)` = `(90, "control")`: the heartbeat never goes stale | #47 |
| 3 (EKI bridge) | `OverrideConfig(fallback_ramp_s=inf)`, last command 90 at t = 0 | accepted; `output(3600)` = `(90, "watchdog")`: the ramp never leaves 90 | #47 |

The RSI bridge refuses both values (`krc_rsi_bridge/decider.py:73`, `:75`). The PR that runs this audit leaves the
code unchanged.

## 1. RKorr = 0 in every reply

| Rule | Enforcing code | Tests | Status |
|---|---|---|---|
| The RKorr element is one constant zero string | `krc_rsi_bridge/rsi_xml.py:14` (`RKORR_ZERO`), `:17` (`_REPLY_HEAD`) | `krc_rsi_bridge/test_reply_format.py::test_rkorr_is_always_zero`; `krc_rsi_bridge/test_invariant_audit.py::test_reply_loop_sends_only_the_zero_rkorr_builders` | covered / added |
| Both reply builders use it, for any IPOC and any OvCmd | `krc_rsi_bridge/rsi_xml.py:20`, `:25` | `krc_rsi_bridge/test_reply_format.py::test_reply_bytes_equal_legacy_fixture`, `::test_zero_reply_is_ov_reply_without_ovcmd`; `krc_rsi_bridge/test_invariant_audit.py::test_reply_bytes_rkorr_zero_random_ipoc_and_override` | covered / added |
| The reply loop sends only those two builders | `krc_rsi_bridge/server.py:120` | `krc_rsi_bridge/test_invariant_audit.py::test_reply_loop_sends_only_the_zero_rkorr_builders` | added |
| Every reply of the running reply process has RKorr 0 (closed, shadow, step; random poses, broken packets) | `krc_rsi_bridge/server.py:120` | `krc_rsi_bridge/test_invariant_audit.py::test_every_reply_of_the_reply_process_has_zero_rkorr`; the simulator counts non-zero RKorr in every timing test (`krc_rsi_bridge/test/loop_helpers.py` `assert_pass`: `test_stall.py::test_parent_stall_does_not_delay_replies`, `test_round_trip.py::test_round_trip_20s`) | covered / added |
| Law B goes through the same zero-RKorr reply | `krc_rsi_bridge/rsi_xml.py:20` | `cl_controller/test_cage2_safety.py::test_reply_rkorr_zero` | covered |

## 2. Override clamped to `[ov_min, ov_max]` with the speed floor

| Rule | Enforcing code | Tests | Status |
|---|---|---|---|
| Bridge: speed floor `ceil(v_min x 100 / v100_min)` | `krc_rsi_bridge/decider.py:53` | `krc_rsi_bridge/test_clamp.py::test_floor_from_pattern`, `::test_effective_ov_min_includes_floor` | covered |
| Bridge: a floor without the pattern's v100 is refused | `krc_rsi_bridge/decider.py:58` | `krc_rsi_bridge/test_invariant_audit.py::test_speed_floor_needs_the_pattern_speed` | added |
| Bridge: every decided value is clamped (control, default, outside, forced, watchdog, step) | `krc_rsi_bridge/decider.py:93`, `:128` | `krc_rsi_bridge/test_clamp.py::test_controller_values_clamped`, `::test_outside_value_clamped`, `::test_step_values_clamped_to_floor`; `krc_rsi_bridge/test_invariant_audit.py::test_decider_output_always_in_window_random_inputs` (random configurations and inputs) | covered / added |
| Bridge: an exception in the decision gives `clamp(ov_default)` | `krc_rsi_bridge/decider.py:132-134` | `krc_rsi_bridge/test_clamp.py::test_fallback_value_clamped`; `krc_rsi_bridge/test_invariant_audit.py::test_decider_broken_positions_give_clamped_fallback` | covered / added |
| Bridge: start refusal: window `1 <= ov_min <= ov_max <= 100` (0 allowed in step mode) | `krc_rsi_bridge/decider.py:67-68` | `krc_rsi_bridge/test_clamp.py::test_bad_config_refused`; `krc_rsi_bridge/test_invariant_audit.py::test_bridge_refuses_non_finite_or_out_of_range_config` | covered / added |
| Bridge: start refusal: `ov_default` outside the window | `krc_rsi_bridge/decider.py:69-70` | `krc_rsi_bridge/test_clamp.py::test_bad_config_refused` | covered |
| Bridge: start refusal: unknown mode | `krc_rsi_bridge/decider.py:71-72` | `krc_rsi_bridge/test_clamp.py::test_bad_config_refused` | covered |
| Bridge: start refusal: `v_min_mm_s` NaN / inf | `krc_rsi_bridge/decider.py:59` (`ceil` raises) | `krc_rsi_bridge/test_invariant_audit.py::test_bridge_refuses_non_finite_or_out_of_range_config` | added |
| Bridge: the step test's unclamped `--limit-probe` is not ported | `krc_rsi_bridge/decider.py:15-16`; refused at start: `print_session/startup.py:208-209` | `print_session/test_startup.py::test_refusals` (`limit_probe` case) | covered |
| Controller: `ov_from_logk` is clamped for any ln k, including ±inf | `cl_controller/controller.py:105-107` | `cl_controller/test_legacy_behaviour.py::test_clamp_matches_legacy`; `cl_controller/test_invariant_audit.py::test_ov_from_logk_in_window_for_any_logk` | covered / added |
| Controller: a NaN or overflowing ln k raises, so no value is returned, and the node then publishes nothing (the watchdog takes over) | `cl_controller/controller.py:107`; `cl_controller/node.py:162` | `cl_controller/test_invariant_audit.py::test_ov_from_logk_raises_rather_than_returning_a_value`, `::test_step_with_any_law_state_is_clamped_or_raises`, `::test_node_publishes_nothing_when_the_step_raises` | added |
| Controller: speed floor raised into `ov_min` | `cl_controller/controller.py:169-170` | `cl_controller/test_legacy_behaviour.py::test_speed_floor_matches_legacy`, `::test_e1_floor_is_12` | covered |
| Controller: NaN / inf / absurd widths never step outside the window, and u stays finite | `cl_controller/law.py:229` (invalid estimate); `cl_controller/node.py:153` (non-finite Width dropped); `cl_controller/controller.py:107` | `cl_controller/test_invariant_audit.py::test_controller_with_nan_inf_and_absurd_widths_stays_in_window`, `::test_controller_with_only_invalid_widths_stays_in_window` | added |
| Controller: law B uses the same clamp and floor | `cl_controller/controller.py:458` (`command`) | `cl_controller/test_cage2_safety.py::test_clamp_and_floor` | covered |
| Session: the bridge gets the controller's window and floor | `print_session/startup.py:212-243` | `print_session/test_startup.py::test_bridge_gets_the_controller_limits` | covered |
| EKI: clamp to `[max(ov_min, ov_floor), ov_max]`; NaN / inf / non-numeric → `ov_default`, clamped | `krc_eki_bridge/safety.py:56-67` | `krc_eki_bridge/test_safety.py::test_clamp_window_and_floor`, `::test_commands_are_clamped`, `::test_default_outside_window_is_clamped`, `::test_forced_ramp_is_clamped`; `krc_eki_bridge/test_invariant_audit.py::test_supervisor_output_and_frames_always_in_window` | covered / added |
| EKI: the sent frame carries the supervisor's clamped value | `krc_eki_bridge/core.py:170-172` | `krc_eki_bridge/test_bridge_fake_eki.py::test_override_send_and_clamp` | covered |
| EKI: config refusals (window, floor above ov_max) | `krc_eki_bridge/safety.py:38-41` | `krc_eki_bridge/test_safety.py::test_config_rejects_bad_values`; `krc_eki_bridge/test_invariant_audit.py::test_config_refusals` | covered / added |
| EKI controller side: `sps.sub` clamps the received value | `krc_templates/sps_eki_override.sub` (NOT RUN) | `krc_eki_bridge/test_sps_model.py::test_clamps_received_value` | covered |

## 3. Stale heartbeat → linear ramp to `ov_default` over `fallback_ramp_s`; forced latch

| Rule | Enforcing code | Tests | Status |
|---|---|---|---|
| Bridge: the heartbeat is refreshed only by a new `seq` | `krc_rsi_bridge/bridge_node.py:84` | `krc_replay/test_realtime_chain.py::test_heartbeat_loss_ramps_to_default` *(ROS)* | covered |
| Bridge: no heartbeat ever → `ov_default` | `krc_rsi_bridge/shared.py:69` (`hb = -inf`), `krc_rsi_bridge/decider.py:117` | `krc_rsi_bridge/test_watchdog.py::test_no_commands_ever_gives_ov_default` | covered |
| Bridge: watchdog after `wd_s`, linear ramp over `fallback_ramp_s`, ended by a fresh heartbeat | `krc_rsi_bridge/decider.py:117-127` | `krc_rsi_bridge/test_watchdog.py::test_commands_stop_ramp_linear_to_default_over_ramp_s`, `::test_stale_heartbeat_triggers_ramp_fresh_heartbeat_ends_it`, `::test_ramp_zero_jumps`; `krc_rsi_bridge/test_invariant_audit.py::test_watchdog_ramp_is_exactly_linear_random_parameters` | covered / added |
| Bridge: the ramp in the reply bytes of the running reply process (UDP) | `krc_rsi_bridge/server.py:117-120` | `krc_rsi_bridge/test_watchdog.py::test_watchdog_end_to_end_with_simulator`; `krc_rsi_bridge/test_invariant_audit.py::test_stale_heartbeat_ramp_in_the_reply_bytes` (exact linear values) | covered / added |
| Bridge: start refusal: `wd_s` and `fallback_ramp_s` must be finite (`wd_s > 0`, ramp `>= 0`) | `krc_rsi_bridge/decider.py:73-76` | `krc_rsi_bridge/test_clamp.py::test_bad_config_refused`; `krc_rsi_bridge/test_invariant_audit.py::test_bridge_refuses_non_finite_or_out_of_range_config` | covered / added |
| Bridge: forced latch set by `source == "forced"`, never cleared, ramps and holds | `krc_rsi_bridge/bridge_node.py:80-83`, `krc_rsi_bridge/server.py:193-195`, `krc_rsi_bridge/shared.py:70`, `krc_rsi_bridge/decider.py:110` | `krc_rsi_bridge/test_watchdog.py::test_forced_latch_ramps_and_holds_with_fresh_heartbeat`, `::test_forced_latch_never_clears_in_server` | covered |
| Controller: the heartbeat stops when updates or RSI packets stop | `cl_controller/controller.py:344-352`; `cl_controller/node.py:167-168` | `cl_controller/test_cage2_safety.py::test_heartbeat_stops_without_updates`, `::test_bridge_ramp` | covered |
| Controller: the forced latch (speed check, wall stop rule) is set and never cleared | `cl_controller/controller.py:383`, `:429`, `:338-342`; only `__init__` clears it (`:253`) | `cl_controller/test_legacy_behaviour.py::test_speed_mismatch_latches_forced_like_legacy`; `cl_controller/test_walls_golden.py::test_stop_rule_latched_on_layer_25_not_on_layer_1`; `cl_controller/test_cage2_safety.py::test_forced_latch`; `cl_controller/test_invariant_audit.py::test_forced_latch_is_only_cleared_in_init`, `::test_forced_latch_holds_through_good_data` | covered / added |
| Controller: start refusal: `fallback_ramp_s` NaN / inf / < 0 | `cl_controller/params.py:219-220` | `cl_controller/test_invariant_audit.py::test_build_configs_refusals` | added |
| EKI: watchdog and linear ramp, a repeated seq is no heartbeat, a fresh heartbeat ends the ramp | `krc_eki_bridge/safety.py:87-126` | `krc_eki_bridge/test_safety.py::test_stale_heartbeat_ramps_linearly_to_default`, `::test_ramp_is_monotonic_and_within_window`, `::test_repeated_seq_is_not_a_heartbeat`, `::test_fresh_heartbeat_ends_ramp`, `::test_zero_ramp_jumps`; `krc_eki_bridge/test_bridge_fake_eki.py::test_heartbeat_ramp`; `krc_eki_bridge/test_same_as_rsi.py::test_same_override_at_every_step` | covered |
| EKI: forced latch | `krc_eki_bridge/safety.py:92-94`, `:128-137` | `krc_eki_bridge/test_safety.py::test_forced_latches_ramps_and_ignores_later_commands` and the five `test_forced_*` tests after it; `krc_eki_bridge/test_bridge_fake_eki.py::test_forced_latch_ramps_and_ignores_later_commands` | covered |
| EKI: config refusal: `wd_s <= 0` / NaN, `fallback_ramp_s < 0` / NaN | `krc_eki_bridge/safety.py:42-45` | `krc_eki_bridge/test_safety.py::test_config_rejects_bad_values`; `krc_eki_bridge/test_invariant_audit.py::test_config_refusals` | covered / added |
| EKI: config refusal: `wd_s = inf` | `krc_eki_bridge/safety.py:42` (missing) | `krc_eki_bridge/test_invariant_audit.py::test_infinite_watchdog_timeout_is_refused_or_ramps` (xfail strict) | **violated (#47)** |
| EKI: config refusal: `fallback_ramp_s = inf` | `krc_eki_bridge/safety.py:44` (missing) | `krc_eki_bridge/test_invariant_audit.py::test_infinite_ramp_is_refused_or_ramps` (xfail strict) | **violated (#47)** |
| EKI: the node passes the parameters unchecked (so #47 can be reached from a parameter file) | `krc_eki_bridge/node.py:38-44` | `krc_eki_bridge/test_invariant_audit.py::test_node_passes_the_parameters_unchecked_to_the_config` | added |
| EKI controller side: `sps.sub` falls back on loss | `krc_templates/sps_eki_override.sub` (NOT RUN) | `krc_eki_bridge/test_sps_model.py::test_falls_back_on_loss`; `krc_eki_bridge/test_bridge_fake_eki.py::test_controller_falls_back_when_bridge_stops` | covered |

## 4. The 4 ms reply loop never blocks

| Rule | Enforcing code | Tests | Status |
|---|---|---|---|
| The reply loop runs in its own process (spawned, without the parent's ROS threads) | `krc_rsi_bridge/server.py:165` | `krc_rsi_bridge/test_stall.py::test_parent_stall_does_not_delay_replies` (2.5 s GIL stall + 2.5 s blocked parent) | covered |
| A blocked ROS callback does not delay replies | `krc_rsi_bridge/bridge_node.py:79-106` (callbacks only touch shared values and the ring) | `krc_rsi_bridge/test_stall_ros.py::test_blocked_ros_callback_does_not_delay_replies` *(ROS)* | covered |
| No logging, lock, queue or pipe between packet and reply; lock-free shared values and ring | `krc_rsi_bridge/server.py:64-147`, `krc_rsi_bridge/shared.py` | `krc_rsi_bridge/test_invariant_audit.py::test_reply_loop_has_no_logging_and_no_blocking_ipc` | added |
| A slow logging handler (0.2 s per record) in the reply process and the parent does not delay replies | `krc_rsi_bridge/server.py:64-147` | `krc_rsi_bridge/test_invariant_audit.py::test_slow_logging_handler_does_not_delay_replies` | added |
| Reply latency and late-reply counting against the 100-packet Timeout | `krc_rsi_bridge/server.py:124-126` | `krc_rsi_bridge/test_round_trip.py::test_round_trip_20s`, `::test_replay_round_trip`; `krc_sim/test_sim.py::test_no_reply_stops_after_timeout`, `::test_outage_shorter_than_timeout_is_survived` | covered |
| EKI: the ROS side never blocks on the TCP link | `krc_eki_bridge/core.py:101-103` | `krc_eki_bridge/test_bridge_fake_eki.py::test_submit_command_never_blocks_while_disconnected` | covered |

## 5. No joint or Cartesian motion targets are streamed

| Rule | Enforcing code | Tests | Status |
|---|---|---|---|
| The RSI reply carries only Sen, RKorr (zero), OvCmd and IPOC | `krc_rsi_bridge/rsi_xml.py:14-31` | `krc_rsi_bridge/test_invariant_audit.py::test_reply_carries_no_motion_element` | added |
| The EKI command carries only Ov and Seq | `krc_eki_bridge/protocol.py:88-90`; `krc_templates/KrcEki.xml` RECEIVE | `krc_eki_bridge/test_protocol.py::test_command_has_no_motion_fields`; `krc_eki_bridge/test_templates.py::test_eki_xml_receives_only_override_and_seq` | covered |
| No KRL template moves the robot | `krc_templates/*.src`, `*.sub` | `krc_eki_bridge/test_templates.py::test_no_motion_in_krl` | covered |
| No package depends on or imports a motion-command interface | every `src/*/package.xml` and node source | `krc_rsi_bridge/test_invariant_audit.py::test_no_package_depends_on_or_imports_a_motion_command_interface` | added |
| The bridges subscribe only to `OverrideCmd`, which has no motion field | `src/krc_interfaces/msg/OverrideCmd.msg`; `krc_rsi_bridge/bridge_node.py:67`; `krc_eki_bridge/node.py` | `krc_rsi_bridge/test_invariant_audit.py::test_interfaces_carry_no_motion_target` | added |

## 6. Controller-side files are NOT RUN templates

| Rule | Enforcing code | Tests | Status |
|---|---|---|---|
| Every krc_templates file is headed NOT RUN | `src/krc_eki_bridge/krc_templates/*` | `krc_eki_bridge/test_templates.py::test_not_run_header` | covered |
| The only controller-side files in `src/` (`.src .sub .dat .krl .rsi .rsix .diagram`, XML other than package.xml) are those templates, and each is headed NOT RUN | `src/**` | `krc_eki_bridge/test_invariant_audit.py::test_controller_side_files_are_only_the_templates`, `::test_every_controller_side_file_is_headed_not_run` | added |
| Every template that `setup.py` installs is headed NOT RUN | `src/krc_eki_bridge/setup.py:16` | `krc_eki_bridge/test_invariant_audit.py::test_every_installed_template_is_headed_not_run` | added |
| `$OV_PRO` is written only under the AUT / EXT guard | `krc_templates/sps_eki_override.sub` | `krc_eki_bridge/test_templates.py::test_ov_pro_written_only_in_aut_ext`; `krc_eki_bridge/test_sps_model.py::test_writes_only_in_aut_ext_and_holds_otherwise` | covered |

The KRL and RSI files in `legacy/krc_config/` are the lab's own files from before the port. They are inputs, not
templates of this phase, so the header rule does not apply to them.

## Start refusals: `cl_controller` parameters (`params.build_configs`, `build_law_cfg`, `cage2_refusals`, `validate_law_cfg`, `apply_speed_rules`, `Law`, `Controller`)

| Refusal | Enforcing code | Tests | Status |
|---|---|---|---|
| mode not shadow / closed / step | `cl_controller/params.py:202-203` | `cl_controller/test_params.py::test_refused_like_run_cl` (`mode=fast`) | covered |
| no plan (closed / shadow) | `cl_controller/params.py:204-205` | `cl_controller/test_params.py::test_refused_like_run_cl` (`plan=""`) | covered |
| fan / semicircle pattern (`geometry`, T16) | `cl_controller/params.py:148-152` | `cl_controller/test_cage2_refusals.py::test_refused_geometry_pattern`; `print_session/test_cage2.py::test_refused_geometry_pattern` | covered |
| `w12_t` and `w_target` disagree | `cl_controller/params.py:208-209` | `cl_controller/test_invariant_audit.py::test_build_configs_refusals` | added |
| `v_min_mm_s` NaN / inf / < 0 | `cl_controller/params.py:217-218` | `cl_controller/test_params.py::test_refused_like_run_cl` (`-1`); `cl_controller/test_invariant_audit.py::test_build_configs_refusals` | covered / added |
| `fallback_ramp_s` NaN / inf / < 0 | `cl_controller/params.py:219-220` | `cl_controller/test_invariant_audit.py::test_build_configs_refusals` | added |
| `sat_stop_s` < 0 / NaN / inf on a wall pattern | `cl_controller/params.py:234-235` | `cl_controller/test_walls_ml.py::test_bad_sat_stop_s_refused_on_walls`; `cl_controller/test_invariant_audit.py::test_sat_stop_s_negative_refused_on_a_wall_pattern` | covered / added |
| open-loop schedule with a plan other than Bpp | `cl_controller/params.py:168-169`; `cl_controller/controller.py:208-209` | `cl_controller/test_params.py::test_refused_like_run_cl`; `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refuses_a_schedule_without_plan_bpp` | covered / added |
| open-loop schedule not parseable | `cl_controller/params.py:171-174` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` (`12,x`) | added |
| `control_sources` with no source | `cl_controller/params.py:183-184` | `cl_controller/test_cage2_refusals.py::test_refused_control_sources_without_a_source_run_cl_591` | covered |
| `force_seg` malformed | `cl_controller/params.py:189-191` | `cl_controller/test_params.py::test_refused_like_run_cl` (`x`) | covered |
| `tau_slow` ≤ 0 / non-finite | `cl_controller/controller.py:187-188` | `cl_controller/test_params.py::test_refused_like_run_cl`; `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | covered / added |
| `gain` outside (0, 1.5] / non-finite | `cl_controller/controller.py:189-192` | `cl_controller/test_params.py::test_refused_like_run_cl`; `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | covered / added |
| `db` < 0 / NaN | `cl_controller/controller.py:193-194` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| `u0` non-finite | `cl_controller/controller.py:195-196` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| `tau_model` < 0 | `cl_controller/controller.py:197-198` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| `start_mix` outside [0, 1] | `cl_controller/controller.py:199-200` | `cl_controller/test_params.py::test_refused_like_run_cl`; `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | covered / added |
| `blind_tau` < 0 | `cl_controller/controller.py:201-202` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| `v_ref_mm_s` ≤ 0 / NaN | `cl_controller/controller.py:203-204` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| `v_start_mm_s` ≤ 0 / inf | `cl_controller/controller.py:205-206` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| open-loop schedule speed ≤ 0 | `cl_controller/controller.py:210-211` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| `u0` with an open-loop schedule | `cl_controller/controller.py:212-213` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| target schedule with an open-loop schedule | `cl_controller/controller.py:214-215` | `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | added |
| `force_seg` with a plan other than B | `cl_controller/controller.py:216-217` | `cl_controller/test_params.py::test_refused_like_run_cl` (`3:bore1`); `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refuses_a_schedule_without_plan_bpp` | covered / added |
| `control_sources` without the primary / unknown source | `cl_controller/controller.py:218-222`; `cl_controller/law.py:140-143` | `cl_controller/test_cage2_refusals.py::test_refused_sources_without_primary_run_cl_594`; `cl_controller/test_walls_ml.py::test_bad_sources_refused_like_legacy_ml`; `cl_controller/test_invariant_audit.py::test_validate_law_cfg_refusals` | covered / added |
| Law: plan, primary, blind_ref / blind_dir, tgt_schedule | `cl_controller/law.py:130-131`, `:137-138`, `:170-173` | `cl_controller/test_law_diff.py::test_invalid_config_refused_like_legacy` | covered |
| V_max of the pattern unknown (outside step) | `cl_controller/controller.py:153-155` | `cl_controller/test_legacy_behaviour.py::test_speed_floor_matches_legacy` (patterns without V_max); `cl_controller/test_invariant_audit.py::test_apply_speed_rules_refuses_a_pattern_without_v_max` | covered / added |
| `ov_base` disagrees with V_max | `cl_controller/controller.py:161-162` | `cl_controller/test_params.py::test_refused_like_run_cl` (`ov_base=55`) | covered |
| open-loop schedule outside the window | `cl_controller/controller.py:172-175` | `cl_controller/test_invariant_audit.py::test_build_configs_refusals` | added |
| window not `1 <= ov_min <= ov_max <= 100` (0 in step) | `cl_controller/controller.py:176-178` | `cl_controller/test_params.py::test_refused_like_run_cl` (`50 / 40`); `cl_controller/test_invariant_audit.py::test_build_configs_refusals` | covered / added |
| `ov_default` outside the window | `cl_controller/controller.py:179-180` | `cl_controller/test_invariant_audit.py::test_build_configs_refusals` | added |
| `elp_b` not on / off | `cl_controller/params.py:118-119` | `cl_controller/test_cage2_refusals.py::test_refused_elp_b_value` | covered |
| `bore2` not off / shadow / fallback / primary | `cl_controller/params.py:120-121` | `cl_controller/test_cage2_refusals.py::test_refused_bore2_value` | covered |
| bore offsets > 0.30 mm, `bore_s_use` not 0 or 6..15 | `cl_controller/params.py:122-130` | `cl_controller/test_cage2_refusals.py::test_refused_offset_above_030_run_cl_1127`, `::test_refused_bore_s_use_run_cl_1130` | covered |
| bore2 fallback / primary without elp_b | `cl_controller/params.py:132-133`; `cl_controller/controller.py:268-271` | `cl_controller/test_cage2_refusals.py::test_refused_bore2_without_elp_b_run_cl_1196`, `::test_controller_refuses_bore2_without_elp_b` | covered |
| bore2 fallback / primary without a finite `bore_sd_max` | `cl_controller/params.py:134-136` | `cl_controller/test_cage2_refusals.py::test_refused_bore2_without_finite_sd_max_run_cl_1198` | covered |
| `b_from_leg` < 1 | `cl_controller/params.py:137-138` | `cl_controller/test_cage2_refusals.py::test_refused_b_from_leg_below_1_run_cl_1202` | covered |
| elp_b on without elp in control_sources | `cl_controller/params.py:139-140` | `cl_controller/test_cage2_refusals.py::test_refused_elp_b_without_elp_source_run_cl_1212` | covered |
| elp_b on with a multi-layer pattern | `cl_controller/params.py:141-143` | `cl_controller/test_cage2_refusals.py::test_refused_elp_b_with_multi_layer_pattern` | covered |
| elp_b on with a pattern without +X lines | `cl_controller/params.py:144-145` | `cl_controller/test_cage2_refusals.py::test_refused_pattern_without_b_lines_run_cl_1296` | covered |
| The node does not start on a refusal | `cl_controller/node.py:206` | `print_session/test_bringup.py::test_bad_mode_or_refused_start` *(ROS)* | covered |

## Start refusals: `print_session` start-up (`startup.plan`, `bore1_check`)

| Refusal | Enforcing code | Tests | Status |
|---|---|---|---|
| mode missing or not shadow / closed / step | `print_session/startup.py:177-178` | `print_session/test_startup.py::test_refusals` (`fast`); `print_session/test_bringup.py::test_missing_mode_refuses` *(ROS)* | covered |
| controller section without a pattern | `print_session/startup.py:180-181` | `print_session/test_invariant_audit.py::test_refused_without_pattern` | added |
| pattern not found | `print_session/startup.py:183-184` | `print_session/test_invariant_audit.py::test_refused_pattern_not_found` | added |
| shadow mode unless `krc_rsi_config` is RSI_MIN (RSI_OV4 → 1 % override) | `print_session/startup.py:190-192` | `print_session/test_startup.py::test_shadow_refused_on_rsi_ov4`, `::test_shadow_mode_warns_about_rsi_ov4` | covered |
| shadow / closed without the ELP readout | `print_session/startup.py:195-196` | `print_session/test_startup.py::test_refusals` (`ELP`) | covered |
| a bore1-led plan without the bore1 readout | `print_session/startup.py:200-203` | `print_session/test_startup.py::test_refusals` (`bore1 readout`) | covered |
| `ceiling_light` not on / off | `print_session/startup.py:206-207` | `print_session/test_invariant_audit.py::test_refused_ceiling_light_value` | added |
| `limit_probe` (unclamped OvCmd) | `print_session/startup.py:208-210` | `print_session/test_startup.py::test_refusals` (`limit_probe`) | covered |
| the bridge section sets a value derived from the controller | `print_session/startup.py:220-231` | `print_session/test_startup.py::test_refusals` (`ov_default`) | covered |
| the bridge refuses the derived limits | `print_session/startup.py:236-239` | `print_session/test_invariant_audit.py::test_refused_bridge_limits` (defence in depth; reached by monkeypatch) | added |
| bridge window differs from the controller's | `print_session/startup.py:240-243` | `print_session/test_invariant_audit.py::test_refused_bridge_window_differs_from_controller` (defence in depth; reached by monkeypatch) | added |
| cage 2 without an explicit `elp_b` | `print_session/startup.py:248-250` | `print_session/test_cage2.py::test_refused_cage2_without_elp_b_run_cl_1154` | covered |
| elp_b on without cage 2 | `print_session/startup.py:251-253` | `print_session/test_cage2.py::test_refused_elp_b_on_without_cage2_run_cl_1158` | covered |
| bore2 without cage 2 | `print_session/startup.py:255-256` | `print_session/test_cage2.py::test_refused_bore2_without_cage2` | covered |
| ELP readout with another pattern | `print_session/startup.py:264-265` | `print_session/test_startup.py::test_refusals` (`pattern`) | covered |
| rig of the other role | `print_session/startup.py:352-357` | `print_session/test_cage2.py::test_refused_rig_of_the_other_role` | covered |
| the same rig for both ELP readouts | `print_session/startup.py:273-274` | `print_session/test_cage2.py::test_refused_same_rig_for_both` | covered |
| ELP-B readout with another pattern | `print_session/startup.py:276-277` | `print_session/test_invariant_audit.py::test_refused_elp_b_readout_with_another_pattern` | added |
| ELP-B section with kind / width_topic / camera of ELP-A | `print_session/startup.py:278-280` | `print_session/test_cage2.py::test_refused_elp_b_section_values` | covered |
| bore section overrides `sd_max` / `w_offset` / topics derived from the controller | `print_session/startup.py:366-370` | `print_session/test_cage2.py::test_refused_bore_section_overriding_the_controller`, `::test_refused_offsets_and_flags_of_the_controller` | covered |
| bore section with another pattern | `print_session/startup.py:372-373` | `print_session/test_startup.py::test_bore1_readout_refusals` | covered |
| bore section with `lockstep` | `print_session/startup.py:374-375` | `print_session/test_startup.py::test_bore1_readout_refusals` | covered |
| bore head not torch / replay | `print_session/startup.py:376-377` | `print_session/test_startup.py::test_bore1_readout_refusals` | covered |
| bore frame_time unknown | `print_session/startup.py:378-379` | `print_session/test_startup.py::test_bore1_readout_refusals` | covered |
| bore node parameter check (camera, weights, w_offset, quality_json, the other bore's topics) | `print_session/startup.py:380-383` → `width_readout_bore1/params.py:55-77` | `print_session/test_cage2.py::test_refused_bore2_without_quality_run_cl_1125`, `::test_refused_bore1_weights_without_quality_run_cl_1120`; `width_readout_bore1/test_bore2.py::test_refused_quality_file_missing_run_cl_1123`, `::test_refused_offset_run_cl_1127`, `::test_refused_camera_and_topics` | covered |
| torch head without a weight folder / with missing weights | `print_session/startup.py:384-391` | `print_session/test_cage2.py::test_refused_bore2_without_head_run_cl_1365` | covered |
| replay head without `replay_csv` | `print_session/startup.py:393-396` | `print_session/test_startup.py::test_bore1_readout_refusals` | covered |
| readout / camera switches of the launch file | `print_session/launch/bringup.launch.py` | `print_session/test_bringup.py::test_readout_and_camera_switch_refusals` *(ROS)* | covered |

## How this table was built

Every `raise Refused` / `raise ValueError` line and every clamp in `cl_controller`, `print_session`,
`krc_rsi_bridge`, `krc_eki_bridge` and `width_readout_bore1/params.py` was matched against per-test line coverage
(`pytest --cov-context=test`) of the pure-Python suites. Every line that no test reached got a test of its own.
Tests marked *(ROS)* were matched by reading them, because they need `rclpy`.
