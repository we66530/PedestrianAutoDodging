from controller import Supervisor
import math

# This controller is attached to DEF WorldSupervisor (a supervisor robot)
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Find the pedestrian by DEF
ped = supervisor.getFromDef("Ped1")
if ped is None:
    raise RuntimeError("Could not find DEF Ped1. Make sure the pedestrian has `DEF Ped1` in the world.")

ped_translation = ped.getField("translation")
pos = ped_translation.getSFVec3f()   # [x, y, z]

# Move along +Y to simulate "left -> right" crossing
speed = 0.01            # meters per step
start_y = pos[1]
target_y = start_y + 2.0   # cross 2 meters to the right side
arrive_eps = 0.01

while supervisor.step(timestep) != -1:
    pos = ped_translation.getSFVec3f()   # refresh current position
    y = pos[1]

    # Move toward target_y on Y axis; keep X,Z unchanged
    dy = target_y - y
    if abs(dy) > arrive_eps:
        step = math.copysign(min(abs(dy), speed), dy)
        pos[1] = y + step
        ped_translation.setSFVec3f(pos)
    else:
        # Reached crossing target; stop here
        pass
