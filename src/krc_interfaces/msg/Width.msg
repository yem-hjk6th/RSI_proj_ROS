# One bead-width reading.
builtin_interfaces/Time stamp   # frame time
string source                   # elp | bore1
int32 leg                       # leg number as in the pattern json and the legacy logs (0-based; legs 0..10 in E1)
float64 s_mm                    # arc length along the leg where the material was laid
float64 d_mm                    # distance behind the nozzle of the reading
float64 width_mm
bool valid
