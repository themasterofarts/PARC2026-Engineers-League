# Run history

| Timestamp | Result | Elapsed | Log | Bag |
|---|---|---|---|---|
| 20260829_204229 | SUCCEEDED | 1m 30s | logs/run_20260829_204229.log | bags/run_20260829_204229 |
| 20260904_185005 | FAILED | ~1m 25s (est., harness hung before logging) | logs/run_20260904_185005.log | bags/run_20260904_185005 (killed mid-record, likely unfinalized) |
| 20260904_191010 | SUCCEEDED | ~0m 52s (est., harness hung before logging) | logs/run_20260904_191010.log | bags/run_20260904_191010 (killed mid-record, likely unfinalized) |

Note (2026-09-04): both of the runs above were driven manually because `run_and_log.sh` hangs
after `task_solution.py` prints its result and shuts down its embedded Nav2 launch — the
outer `ros2 run ... | tee` never returns, so the script never reaches its own elapsed-time
calculation or history append (confirmed on both a SUCCEEDED and a FAILED outcome, so it's
independent of task result). Elapsed times above were computed from log timestamps
(`basic_navigator` start vs. `Goal reached.`), not the harness. Each run had to be killed
manually (`kill -9` on the `ros2 run`/`ros2 launch`/`ros2 bag record`/`gz sim` PIDs) — the
rosbag for each was therefore not cleanly finalized and may need `ros2 bag reindex` before use.
This hang is worth root-causing in `task_solution.py`'s shutdown path before relying on
`run_and_log.sh` again. Also note the FAILED run's cause: `RegulatedPurePursuitController`
reported "collision ahead" repeatedly while distance-remaining grew instead of shrank, until
the local costmap drifted out of the global costmap bounds and planning aborted — the same
RPLIDAR self-collision failure mode described above, recurring despite the `obstacle_min_range`
fix, suggesting the sim isn't fully deterministic run-to-run.
| 20260904_201914 | SUCCEEDED | 1m 49s | logs/run_20260904_201914.log | bags/run_20260904_201914 |
| 20260927_194831 | SUCCEEDED | 3m 24s | logs/run_20260927_194831.log | bags/run_20260927_194831 |
| 20260927_201050 | SUCCEEDED | 2m 27s | logs/run_20260927_201050.log | bags/run_20260927_201050 |
| 20260927_201534 | SUCCEEDED | 2m 3s | logs/run_20260927_201534.log | bags/run_20260927_201534 |
| 20260927_204735 | SUCCEEDED | 1m 37s | logs/run_20260927_204735.log | bags/run_20260927_204735 |
| 20260927_205047 | SUCCEEDED | 2m 22s | logs/run_20260927_205047.log | bags/run_20260927_205047 |
| 20260927_205812 | FAILED | 1m 47s | logs/run_20260927_205812.log | bags/run_20260927_205812 |
| 20260927_210850 | SUCCEEDED | 0m 55s | logs/run_20260927_210850.log | bags/run_20260927_210850 |
| 20260927_212140 | SUCCEEDED | 0m 58s | logs/run_20260927_212140.log | bags/run_20260927_212140 |
| 20260927_224929 | SUCCEEDED | 1m 26s | logs/run_20260927_224929.log | bags/run_20260927_224929 |
| 20260927_225150 | SUCCEEDED | 0m 43s | logs/run_20260927_225150.log | bags/run_20260927_225150 |
| 20260927_225310 | FAILED | 1m 6s | logs/run_20260927_225310.log | bags/run_20260927_225310 |
| 20260927_225440 | SUCCEEDED | 0m 44s | logs/run_20260927_225440.log | bags/run_20260927_225440 |
| 20260927_225607 | FAILED | 1m 27s | logs/run_20260927_225607.log | bags/run_20260927_225607 |
| 20260927_225758 | SUCCEEDED | 2m 1s | logs/run_20260927_225758.log | bags/run_20260927_225758 |
| 20260927_230021 | SUCCEEDED | 0m 59s | logs/run_20260927_230021.log | bags/run_20260927_230021 |
| 20260927_232015 | SUCCEEDED | 0m 52s | logs/run_20260927_232015.log | bags/run_20260927_232015 |
| 20260927_232128 | SUCCEEDED | 0m 50s | logs/run_20260927_232128.log | bags/run_20260927_232128 |
| 20260927_232239 | SUCCEEDED | 0m 52s | logs/run_20260927_232239.log | bags/run_20260927_232239 |
| 20260927_232721 | SUCCEEDED | 1m 4s | logs/run_20260927_232721.log | bags/run_20260927_232721 |
| 20260927_232949 | SUCCEEDED | 0m 55s | logs/run_20260927_232949.log | bags/run_20260927_232949 |
| 20260927_233145 | SUCCEEDED | 0m 58s | logs/run_20260927_233145.log | bags/run_20260927_233145 |
| 20260928_002734 | SUCCEEDED | 1m 0s | logs/run_20260928_002734.log | bags/run_20260928_002734 |
| 20260928_003001 | SUCCEEDED | 0m 54s | logs/run_20260928_003001.log | bags/run_20260928_003001 |
| 20260928_003210 | SUCCEEDED | 0m 55s | logs/run_20260928_003210.log | bags/run_20260928_003210 |
| 20260928_005128 | SUCCEEDED | 0m 59s | logs/run_20260928_005128.log | bags/run_20260928_005128 |
| 20260928_005314 | SUCCEEDED | 0m 54s | logs/run_20260928_005314.log | bags/run_20260928_005314 |
| 20260928_014718 | FAILED (odom_imu disabled on purpose: before-fix demo video) | 2m 0s | logs/run_20260928_014718.log | bags/run_20260928_014718 |
| 20260928_025005 | SUCCEEDED (smoother in loop) | 0m 50s | logs/run_20260928_025005.log | bags/run_20260928_025005 |
| 20260928_025136 | SUCCEEDED (smoother in loop) | 0m 50s | logs/run_20260928_025136.log | bags/run_20260928_025136 |
| 20260928_025249 | SUCCEEDED (smoother in loop) | 0m 57s | logs/run_20260928_025249.log | bags/run_20260928_025249 |
| 20260928_025406 | SUCCEEDED (smoother in loop) | 0m 52s | logs/run_20260928_025406.log | bags/run_20260928_025406 |
| 20260928_025523 | SUCCEEDED (smoother in loop) | 0m 53s | logs/run_20260928_025523.log | bags/run_20260928_025523 |
| 20260928_025639 | SUCCEEDED (smoother in loop) | 0m 51s | logs/run_20260928_025639.log | bags/run_20260928_025639 |
| 20260928_025818 | FAILED (person test: floating visitor on route, LiDAR-only) | 1m 28s | logs/run_20260928_025818.log | bags/run_20260928_025818 |
| 20260928_030101 | SUCCEEDED (person test: visitor on floor, on route) | 0m 50s | logs/run_20260928_030101.log | bags/run_20260928_030101 |
| 20260928_030223 | SUCCEEDED (person test: visitor on floor appears 3 m ahead) | 2m 5s | logs/run_20260928_030223.log | bags/run_20260928_030223 |
| 20260928_031746 | FAILED (camera experiment (marking-only): floating visitor) | 2m 30s | logs/run_20260928_031746.log | bags/run_20260928_031746 |
| 20260928_032507 | FAILED (camera experiment (voxel, 0.3 m): floating visitor) | 1m 44s | logs/run_20260928_032507.log | bags/run_20260928_032507 |
| 20260928_032723 | FAILED (camera experiment (voxel, 0.3 m)) | 2m 26s | logs/run_20260928_032723.log | bags/run_20260928_032723 |
| 20260928_033241 | SUCCEEDED (camera experiment (voxel, 0.6 m)) | 0m 59s | logs/run_20260928_033241.log | bags/run_20260928_033241 |
| 20260928_033415 | FAILED (camera experiment (voxel, 0.6 m): floating visitor) | 1m 30s | logs/run_20260928_033415.log | bags/run_20260928_033415 |
| 20260928_033649 | SUCCEEDED (camera experiment (voxel, 0.6 m)) | 0m 56s | logs/run_20260928_033649.log | bags/run_20260928_033649 |
| 20260928_033813 | SUCCEEDED (camera experiment (voxel, 0.6 m)) | 0m 58s | logs/run_20260928_033813.log | bags/run_20260928_033813 |
| 20260928_033934 | FAILED (camera experiment (voxel, 0.6 m)) | 3m 47s | logs/run_20260928_033934.log | bags/run_20260928_033934 |
| 20260928_034516 | SUCCEEDED (tolerance test, smoother in loop, xy 0.10) | 0m 53s | logs/run_20260928_034516.log | bags/run_20260928_034516 |
| 20260928_034638 | SUCCEEDED (tolerance test, smoother in loop, xy 0.10, touched cafe_table_6) | 1m 42s | logs/run_20260928_034638.log | bags/run_20260928_034638 |
| 20260928_034844 | SUCCEEDED (tolerance test, smoother in loop, xy 0.10) | 0m 54s | logs/run_20260928_034844.log | bags/run_20260928_034844 |
| 20260928_035005 | FAILED (tolerance test, smoother in loop, xy 0.07, touched cafe_table_6) | 2m 42s | logs/run_20260928_035005.log | bags/run_20260928_035005 |
| 20260928_035318 | SUCCEEDED (tolerance test, smoother in loop, xy 0.07) | 0m 54s | logs/run_20260928_035318.log | bags/run_20260928_035318 |
| 20260928_035436 | SUCCEEDED (tolerance test, smoother in loop, xy 0.07) | 0m 57s | logs/run_20260928_035436.log | bags/run_20260928_035436 |
| 20260928_035631 | SUCCEEDED (A/B: smoother bypassed) | 1m 2s | logs/run_20260928_035631.log | bags/run_20260928_035631 |
| 20260928_035759 | SUCCEEDED (A/B: smoother in loop, touched cafe_table_6) | 0m 59s | logs/run_20260928_035759.log | bags/run_20260928_035759 |
| 20260928_035924 | SUCCEEDED (A/B: smoother bypassed) | 0m 50s | logs/run_20260928_035924.log | bags/run_20260928_035924 |
| 20260928_040040 | SUCCEEDED (A/B: smoother in loop, touched cafe_table_6) | 1m 39s | logs/run_20260928_040040.log | bags/run_20260928_040040 |
| 20260928_040242 | SUCCEEDED (A/B: smoother bypassed) | 0m 52s | logs/run_20260928_040242.log | bags/run_20260928_040242 |
| 20260928_040355 | SUCCEEDED (A/B: smoother in loop) | 0m 56s | logs/run_20260928_040355.log | bags/run_20260928_040355 |
| 20260928_040514 | SUCCEEDED (A/B: smoother bypassed) | 0m 51s | logs/run_20260928_040514.log | bags/run_20260928_040514 |
| 20260928_040627 | SUCCEEDED (A/B: smoother in loop) | 0m 51s | logs/run_20260928_040627.log | bags/run_20260928_040627 |
| 20260928_040835 | SUCCEEDED | 0m 56s | logs/run_20260928_040835.log | bags/run_20260928_040835 |
| 20260928_040959 | SUCCEEDED | 0m 54s | logs/run_20260928_040959.log | bags/run_20260928_040959 |
| 20260928_041117 | SUCCEEDED | 0m 56s | logs/run_20260928_041117.log | bags/run_20260928_041117 |
