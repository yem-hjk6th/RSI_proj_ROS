# Full-print replay smoke tests (B, rep2) on core 9, below normal. Line C, 2026-10-18.
# powershell -File run_smoke_full.ps1
$env:PYTHONUTF8 = "1"; $env:PYTHONDONTWRITEBYTECODE = "1"; $env:OMP_NUM_THREADS = "1"
$Y4F = "<REPO>/Y4F"
$RT = "$Y4F/1018/desktop_5080/control/realtime"
$PY = "<HOME>/.conda/envs/ffs/python.exe"
foreach ($ses in "20261017_185445", "20261017_215453") {
  $log = "$RT/smoke_out/console_full_$ses.txt"
  $t0 = Get-Date
  cmd /c "start `"`" /B /WAIT /BELOWNORMAL /AFFINITY 200 $PY $RT/replay_smoke.py --data $Y4F/1018/pkg_261017/Y4F --session $ses --root $Y4F --rig $Y4F/1015/desktop_5080/r1_first/rig_cage1012.json --pattern $RT/pattern/pattern_usr_ELP_R1v_261016.json --int-dir $Y4F/1018/desktop_5080/readout/int_$ses --out $RT/smoke_out > `"$log`" 2>&1"
  $t1 = Get-Date
  Add-Content -Path $log -Value ("[run_smoke_full] {0:yyyy-MM-dd HH:mm:ss} -> {1:HH:mm:ss}, {2:N0} s, exit {3}" -f $t0, $t1, ($t1 - $t0).TotalSeconds, $LASTEXITCODE)
}
