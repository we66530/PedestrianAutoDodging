from controller import Supervisor

ped = Supervisor()
timestep = int(ped.getBasicTimeStep())

self_node = ped.getSelf()
t_field = self_node.getField("translation")

pos = t_field.getSFVec3f()
speed = 0.01   # slower than Agent

while ped.step(timestep) != -1:
    pos = t_field.getSFVec3f()
    pos[0] -= speed    # move toward negative X (same direction as Agent)
    t_field.setSFVec3f(pos)
