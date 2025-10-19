from controller import Supervisor
import math

sup = Supervisor()
timestep = int(sup.getBasicTimeStep())

ped = sup.getFromDef("Ped1")
if ped is None:
    raise RuntimeError("DEF Ped1 not found. Add `DEF Ped1` to the pedestrian robot.")

t_field = ped.getField("translation")
pos = t_field.getSFVec3f()  # [x, y, z]

# Move along +X (head-on toward Agent that’s to the right)
speed = 0.015        # m/step
target_x = pos[0] + 4.0  # walk ~4 meters to the right
eps = 1e-2

while sup.step(timestep) != -1:
    pos = t_field.getSFVec3f()
    dx = target_x - pos[0]
    if abs(dx) > eps:
        step = math.copysign(min(abs(dx), speed), dx)
        pos[0] += step          # X changes; Y,Z stay
        t_field.setSFVec3f(pos)
    # else: arrived — do nothing / stop
