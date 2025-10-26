# agent_m6.py
from controller import Supervisor
import math

class AgentM6(Supervisor):
    def __init__(self, destination):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.node = self.getSelf()
        self.t_field = self.node.getField("translation")
        self.r_field = self.node.getField("rotation")
        self.pos = self.t_field.getSFVec3f()
        self.yaw = 0.0

        self.destination = destination
        self.speed = 0.02
        self.goal_eps = 0.05
        self.avoid_radius = 0.6

        # Pedestrians
        self.p1_t = self.getFromDef("Ped1").getField("translation")
        self.p2_t = self.getFromDef("Ped2").getField("translation")
        self.p3_t = self.getFromDef("Ped3").getField("translation")

        # For velocity estimation
        self.prev_p1 = None
        self.prev_p2 = None

    @staticmethod
    def _norm(x, y):
        n = math.hypot(x, y)
        return (x/n, y/n) if n > 1e-6 else (0, 0)

    def run(self):
        while self.step(self.dt) != -1:
            # Goal vector
            dx, dy = self.destination[0]-self.pos[0], self.destination[1]-self.pos[1]
            dist_goal = math.hypot(dx, dy)
            if dist_goal < self.goal_eps:
                print("✅ Reached goal")
                continue
            gx, gy = self._norm(dx, dy)

            # Ped positions
            p1 = self.p1_t.getSFVec3f()
            p2 = self.p2_t.getSFVec3f()
            p3 = self.p3_t.getSFVec3f()
            d1 = math.hypot(p1[0]-self.pos[0], p1[1]-self.pos[1])
            d2 = math.hypot(p2[0]-self.pos[0], p2[1]-self.pos[1])
            d3 = math.hypot(p3[0]-self.pos[0], p3[1]-self.pos[1])

            dir_x, dir_y = gx, gy

            # Velocities
            v1x=v1y=v2x=v2y=0.0
            if self.prev_p1: v1x, v1y = p1[0]-self.prev_p1[0], p1[1]-self.prev_p1[1]
            if self.prev_p2: v2x, v2y = p2[0]-self.prev_p2[0], p2[1]-self.prev_p2[1]
            self.prev_p1, self.prev_p2 = (p1[0], p1[1]), (p2[0], p2[1])

            # --- Avoidance priority ---
            if d3 < self.avoid_radius:  # Static obstacle first
                avoid = (0, 1) if p3[1] < self.pos[1] else (0, -1)
                dir_x = 0.5*gx + 0.5*avoid[0]
                dir_y = 0.5*gy + 0.5*avoid[1]

            elif d1 < self.avoid_radius and d2 < self.avoid_radius:
                # Combine P1 and P2 (multi-lane flow)
                w1, w2 = 1.0/max(d1,1e-3), 1.0/max(d2,1e-3)
                sum_vx, sum_vy = w1*v1x+w2*v2x, w1*v1y+w2*v2y
                avoid_x, avoid_y = -sum_vy, sum_vx
                ax, ay = self._norm(avoid_x, avoid_y)
                dir_x = 0.5*gx + 0.5*ax
                dir_y = 0.5*gy + 0.5*ay

            elif d1 < self.avoid_radius:
                avoid = (0, 1) if p1[1] < self.pos[1] else (0, -1)
                dir_x = 0.6*gx + 0.4*avoid[0]
                dir_y = 0.6*gx + 0.4*avoid[1]

            elif d2 < self.avoid_radius:
                avoid = (0, 1) if p2[1] < self.pos[1] else (0, -1)
                dir_x = 0.6*gx + 0.4*avoid[0]
                dir_y = 0.6*gx + 0.4*avoid[1]

            # Normalize
            dir_x, dir_y = self._norm(dir_x, dir_y)

            # Move
            self.pos[0] += dir_x*self.speed
            self.pos[1] += dir_y*self.speed
            self.t_field.setSFVec3f(self.pos)

            # Rotate
            self.yaw = math.atan2(dir_y, dir_x)
            self.r_field.setSFRotation([0,0,1,self.yaw])

# Run
controller = AgentM6(destination=[-2.0, 0.0])
controller.run()
