# The Robot mode message encapsulates the mode/teach state of the robot
# Typically this is controlled by the pendant key switch, but not always

int8 val

# enumerated values
int8 UNKNOWN=-1                 # Unknown or unavailable         
int8 MANUAL=1 			 # Teach OR manual mode
int8 AUTO=2                     # Automatic mode

