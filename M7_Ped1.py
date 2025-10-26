# ped1_headon_supervisor.py
from controller import Supervisor

class Ped1HeadOn(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.node = self.getSelf()
        self.t_field = self.node.getField("translation")
        self.pos = self.t_field.getSFVec3f()

        self.speed = 0.015  # move steadily toward Agent
        self.min_x = -4.0   # start far left
        self.max_x =  2.0   # stop near Agent

    def run(self):
        while self.step(self.dt) != -1:
            # Move rightward (toward Agent, assumed at ~0,0)
            self.pos[0] += self.speed

            if self.pos[0] > self.max_x:
                self.pos[0] = self.min_x  # reset loop

            self.t_field.setSFVec3f(self.pos)

controller = Ped1HeadOn()
controller.run()
