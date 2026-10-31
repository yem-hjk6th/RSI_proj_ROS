# Reply-latency tests of run_cl.py with the RSI reply in its own process (line C, 2026-10-18, after review F2).
# Uses the reviewer's harness unchanged: verify/control_field_C_r1/latency_harness.py + fake_krc_proc.py (fake KRC in a
# separate process, records send -> reply time per packet). Cores 9-12, below normal. No hardware.
# powershell -File run_latency_tests.ps1
$env:PYTHONUTF8 = "1"; $env:PYTHONDONTWRITEBYTECODE = "1"; $env:OMP_NUM_THREADS = "2"
$Y4F = "<REPO>/Y4F"
$PY = "<HOME>/.conda/envs/ffs/python.exe"
$LH = "$Y4F/1018/desktop_5080/verify/control_field_C_r1/latency_harness.py"
$PIPE = "$Y4F/1015/pkg_261014/Y4F/pipeline/pipeline_cage1012"
$DATA = "$Y4F/1018/pkg_261017/Y4F"
$OUT = "$Y4F/1018/desktop_5080/control/realtime/latency_out"
New-Item -ItemType Directory -Force $OUT | Out-Null
$runs = @(
  @("closed_nohog", "--mode closed"),
  @("closed_hog2", "--mode closed --hog 2 --hog-at 40"),
  @("step_nohog", "--mode step")
)
foreach ($r in $runs) {
  $tag, $extra = $r
  $log = "$OUT/console_$tag.txt"
  cmd /c "start `"`" /B /WAIT /BELOWNORMAL /AFFINITY 1E00 $PY $LH --pipeline-dir $PIPE --data $DATA --session 20261017_185445 --skip 10 --seconds 80 --out $OUT --tag $tag $extra > `"$log`" 2>&1"
}
