#操作任务结果
# Result codes for manipulation tasks

# task completed as expected
# generally means you can proceed as planned
int32 SUCCESS = 1

# task not possible (e.g. out of reach or obstacles in the way)
# generally means that the world was not disturbed, so you can try another task
int32 UNFEASIBLE = -1

# task was thought possible, but failed due to unexpected events during execution
# it is likely that the world was disturbed, so you are encouraged to refresh
# your sensed world model before proceeding to another task
int32 FAILED = -2

# a lower level error prevented task completion (e.g. joint controller not responding)
# generally requires human attention
int32 ERROR = -3

# means that at some point during execution we ended up in a state that the collision-aware
# arm navigation module will not move out of. The world was likely not disturbed, but you 
# probably need a new collision map to move the arm out of the stuck position
int32 ARM_MOVEMENT_PREVENTED = -4

# specific to grasp actions
# the object was grasped successfully, but the lift attempt could not achieve the minimum lift distance requested
# it is likely that the collision environment will see collisions between the hand/object and the support surface
int32 LIFT_FAILED = -5

# specific to place actions
# the object was placed successfully, but the retreat attempt could not achieve the minimum retreat distance requested
# it is likely that the collision environment will see collisions between the hand and the object
int32 RETREAT_FAILED = -6

# indicates that somewhere along the line a human said "wait, stop, this is bad, go back and do something else"
int32 CANCELLED = -7

# the actual value of this error code
int32 value
