std_msgs/Header header
string action          # e.g. "pick_and_place", "pick", "place"
string object_label    # e.g. "screw", "rivet"
string target_location # e.g. "bin_A", "bin_B"
string raw_command     # original user text (for logging)
string[] modifiers     # e.g. ["smallest", "leftmost"]
