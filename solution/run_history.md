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
| 20260928_191709 | FAILED (map dev: navigator on wall-clock time, AMCL ignored the initial pose) | 0m 45s | logs/run_20260928_191709.log | bags/run_20260928_191709 |
| 20260928_192213 | FAILED (map dev: robot's own footprint in the map, no plan from the start) | 0m 46s | logs/run_20260928_192213.log | bags/run_20260928_192213 |
| 20260928_192600 | FAILED (map dev: self-hit trail in the map, detoured into cafe_table_1) | 1m 56s | logs/run_20260928_192600.log | bags/run_20260928_192600 |
| 20260928_193334 | SUCCEEDED (map, AMCL defaults) | 1m 34s | logs/run_20260928_193334.log | bags/run_20260928_193334 |
| 20260928_193543 | SUCCEEDED (map, AMCL defaults) | 3m 57s | logs/run_20260928_193543.log | bags/run_20260928_193543 |
| 20260928_194007 | SUCCEEDED (map, AMCL defaults) | 2m 42s | logs/run_20260928_194007.log | bags/run_20260928_194007 |
| 20260928_194314 | SUCCEEDED (map, AMCL defaults) | 1m 8s | logs/run_20260928_194314.log | bags/run_20260928_194314 |
| 20260928_194629 | SUCCEEDED (map, AMCL tuned, yaw tol 0.4 (circled the goal)) | 2m 25s | logs/run_20260928_194629.log | bags/run_20260928_194629 |
| 20260928_194919 | SUCCEEDED (map, AMCL tuned, yaw tol 0.4) | 1m 5s | logs/run_20260928_194919.log | bags/run_20260928_194919 |
| 20260928_195052 | SUCCEEDED (map, AMCL tuned, yaw tol 0.4) | 0m 59s | logs/run_20260928_195052.log | bags/run_20260928_195052 |
| 20260928_195333 | SUCCEEDED (map, stock recovery tree) | 1m 0s | logs/run_20260928_195333.log | bags/run_20260928_195333 |
| 20260928_195457 | SUCCEEDED (map, stock recovery tree) | 1m 1s | logs/run_20260928_195457.log | bags/run_20260928_195457 |
| 20260928_195623 | SUCCEEDED (map, stock recovery tree) | 1m 1s | logs/run_20260928_195623.log | bags/run_20260928_195623 |
| 20260928_195748 | SUCCEEDED (map, stock recovery tree) | 1m 3s | logs/run_20260928_195748.log | bags/run_20260928_195748 |
| 20260928_200937 | SUCCEEDED (map, stock recovery tree; ROS graph captured) | 0m 57s | logs/run_20260928_200937.log | bags/run_20260928_200937 |
| 20260928_201203 | SUCCEEDED (map, stock recovery tree: recorded; brushed cafe_table_7, 18 contacts) | 1m 24s | logs/run_20260928_201203.log | bags/run_20260928_201203 |
| 20260928_201543 | SUCCEEDED (map, stock recovery tree) | 1m 10s | logs/run_20260928_201543.log | bags/run_20260928_201543 |
| 20260928_201716 | SUCCEEDED (map, stock recovery tree) | 1m 13s | logs/run_20260928_201716.log | bags/run_20260928_201716 |
| 20260928_201849 | SUCCEEDED (map, stock recovery tree) | 1m 7s | logs/run_20260928_201849.log | bags/run_20260928_201849 |
| 20260928_202021 | SUCCEEDED (map, stock recovery tree) | 1m 2s | logs/run_20260928_202021.log | bags/run_20260928_202021 |
| 20260928_202213 | SUCCEEDED (map, stock recovery tree: recorded, solution 2 video) | 1m 4s | logs/run_20260928_202213.log | bags/run_20260928_202213 |
| 20260928_230802 | SUCCEEDED (map, stock recovery tree: RViz screenshot run) | 1m 11s | logs/run_20260928_230802.log | bags/run_20260928_230802 |
| 20260928_231011 | SUCCEEDED (map, stock recovery tree: RViz screenshot run) | 1m 10s | logs/run_20260928_231011.log | bags/run_20260928_231011 |
| 20260928_234015 | SUCCEEDED (experiment: despeckled map, rejected) | 1m 9s | logs/run_20260928_234015.log | bags/run_20260928_234015 |
| 20260928_234149 | FAILED (experiment: despeckled map, rejected) | 2m 35s | logs/run_20260928_234149.log | bags/run_20260928_234149 |
| 20260928_234446 | FAILED (experiment: despeckled map, rejected) | 2m 2s | logs/run_20260928_234446.log | bags/run_20260928_234446 |
| 20260928_234709 | FAILED (experiment: despeckled map, rejected) | 1m 45s | logs/run_20260928_234709.log | bags/run_20260928_234709 |
| 20260928_234914 | SUCCEEDED (experiment: despeckled map, rejected) | 1m 17s | logs/run_20260928_234914.log | bags/run_20260928_234914 |
| 20260928_235057 | FAILED (experiment: despeckled map, rejected) | 1m 33s | logs/run_20260928_235057.log | bags/run_20260928_235057 |
| 20260928_235259 | SUCCEEDED (experiment: despeckled map, rejected) | 1m 11s | logs/run_20260928_235259.log | bags/run_20260928_235259 |
| 20260928_235439 | FAILED (experiment: despeckled map, rejected) | 2m 42s | logs/run_20260928_235439.log | bags/run_20260928_235439 |
| 20260928_235919 | SUCCEEDED (map, final config: recovery without spin/backup) | 1m 1s | logs/run_20260928_235919.log | bags/run_20260928_235919 |
| 20260929_000049 | SUCCEEDED (map, final config: recovery without spin/backup) | 0m 57s | logs/run_20260929_000049.log | bags/run_20260929_000049 |
| 20260929_000214 | SUCCEEDED (map, final config: recovery without spin/backup) | 0m 59s | logs/run_20260929_000214.log | bags/run_20260929_000214 |
| 20260929_000336 | SUCCEEDED (map, final config: recovery without spin/backup) | 1m 1s | logs/run_20260929_000336.log | bags/run_20260929_000336 |
| 20260929_000503 | SUCCEEDED (map, final config: recovery without spin/backup) | 0m 58s | logs/run_20260929_000503.log | bags/run_20260929_000503 |
| 20260929_000630 | SUCCEEDED (map, final config: recovery without spin/backup) | 0m 58s | logs/run_20260929_000630.log | bags/run_20260929_000630 |
| 20260929_000751 | SUCCEEDED (map, final config: recovery without spin/backup) | 1m 3s | logs/run_20260929_000751.log | bags/run_20260929_000751 |
| 20260929_000920 | SUCCEEDED (map, final config: recovery without spin/backup) | 0m 58s | logs/run_20260929_000920.log | bags/run_20260929_000920 |
