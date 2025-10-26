# ped3_static.py
from controller import Supervisor

class Ped3Static(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.node = self.getSelf()
        self.t_field = self.node.getField("translation")
        self.pos = self.t_field.getSFVec3f()  # remains fixed

    def run(self):
        while self.step(self.dt) != -1:
            self.t_field.setSFVec3f(self.pos)  # just keep still

controller = Ped3Static()
controller.run()
