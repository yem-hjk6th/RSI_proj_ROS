# Program speed override requested by a controller node. The bridge clamps it and applies the watchdog.
builtin_interfaces/Time stamp
int32 value_pct                 # requested override [%]
string source                   # elp | bore1 | model | forced | default; cage 2 law B: elp_b | bore2 | model
uint64 seq                      # increments with every command; the bridge uses it as heartbeat
