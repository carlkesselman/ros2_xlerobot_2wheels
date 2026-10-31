# An object a plan refers to: what to look for and where it was last seen. Skills re-detect it and associate the
# detection with this reference within `gate_m`; more than one match is AMBIGUOUS, none is NOT_FOUND.
string prompt                    # detector prompt, e.g. "a cube"
geometry_msgs/PoseStamped pose   # last known pose (centre of the object), any frame TF can transform, usually map
float32 size_m                   # edge length for cube-shaped objects; 0 = the detector's default
float32 gate_m                   # association distance; 0 = the skill's default
