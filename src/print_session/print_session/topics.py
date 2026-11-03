"""Topics of a print session and how the recorder selects them."""
STATE = "/krc/state"                 # krc_interfaces/KrcState
OVERRIDE = "/krc/override_cmd"       # krc_interfaces/OverrideCmd
REPLY = "/krc/reply_record"          # krc_interfaces/ReplyRecord
CTRL = "/controller/ctrl_record"     # krc_interfaces/CtrlRecord
WIDTH_ELP = "/width/elp"             # krc_interfaces/Width
WIDTH_BORE1 = "/width/bore1"         # krc_interfaces/Width
BORE1_RECORD = "/width/bore1/record"  # krc_interfaces/Bore1Record (one per processed bore1 frame)

TYPES = {
    STATE: "krc_interfaces/msg/KrcState",
    OVERRIDE: "krc_interfaces/msg/OverrideCmd",
    REPLY: "krc_interfaces/msg/ReplyRecord",
    CTRL: "krc_interfaces/msg/CtrlRecord",
    WIDTH_ELP: "krc_interfaces/msg/Width",
    WIDTH_BORE1: "krc_interfaces/msg/Width",
    BORE1_RECORD: "krc_interfaces/msg/Bore1Record",
}

# ros2 bag record --regex: the four fixed topics and everything under /width/ (/width/bore1* included)
RECORD_REGEX = r"^/(krc/(state|override_cmd|reply_record)|controller/ctrl_record|width/.+)$"
STORAGE_ID = "mcap"
