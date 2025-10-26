# ped3_overtake_supervisor.py
from controller import Supervisor

class Ped3Overtake(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.node = self.getSelf()
        self.t_field = self.node.getField("translation")
        self.pos = self.t_field.getSFVec3f()

        self.speed = 0.01   # slower than Agent (0.02)
        self.min_x = -4.0
        self.max_x =  0.0   # stays in front, doesn’t loop too far

    def run(self):
        while self.step(self.dt) != -1:
            # Move leftward slowly
            self.pos[0] -= self.speed

            if self.pos[0] < self.min_x:
                self.pos[0] = self.max_x  # reset in front

            self.t_field.setSFVec3f(self.pos)

controller = Ped3Overtake()
controller.run()
