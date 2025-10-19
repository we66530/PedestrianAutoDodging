from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

ped = robot.getSelf()
t_field = ped.getField("translation")

pos = t_field.getSFVec3f()
speed = 0.015

while robot.step(timestep) != -1:
    pos = t_field.getSFVec3f()
    pos[0] -= speed   # move left (negative X)
    pos[1] += speed   # move up (positive Y) → diagonal 45°
    t_field.setSFVec3f(pos)
