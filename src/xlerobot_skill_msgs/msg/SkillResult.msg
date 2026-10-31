# The outcome every skill reports (ADR-0006). The executive and the planner act on `code`; `message` is for people and logs.
uint8 SUCCEEDED=0
uint8 TIMEOUT=1
uint8 NOT_REACHABLE=2      # no plan or no IK for the goal from where the robot is now
uint8 NOT_FOUND=3          # the target object or waypoint is not there
uint8 AMBIGUOUS=4          # more than one candidate matches the target
uint8 LOST_OBJECT=5        # the object was there and is gone
uint8 COLLISION=6          # stopped by contact or a collision check
uint8 SAFETY_STOP=7        # presence lost, e-stop, or the safety supervisor; never retried by a plan
uint8 PREEMPTED=8          # cancelled by the executive or the operator
uint8 FAILED=9             # anything else; the message says what

uint8 code
string message
