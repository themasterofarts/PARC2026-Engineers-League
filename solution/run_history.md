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
