# ped2_cross_opposite.py
from controller import Supervisor

class Ped2Cross(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.node = self.getSelf()
        self.t_field = self.node.getField("translation")
        self.pos = self.t_field.getSFVec3f()
        self.speed = 0.03
        self.min_x, self.max_x = -2.0, 2.0
        self.direction = -1  # opposite direction

    def run(self):
        while self.step(self.dt) != -1:
            self.pos[1] += self.speed * self.direction
            if self.pos[1] > self.max_x:
                self.pos[1] = self.max_x; self.direction = -1
            elif self.pos[1] < self.min_x:
                self.pos[1] = self.min_x; self.direction = 1
            self.t_field.setSFVec3f(self.pos)

controller = Ped2Cross()
controller.run()
