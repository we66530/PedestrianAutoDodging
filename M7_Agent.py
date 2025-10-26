from controller import Supervisor
import math

class AgentM7(Supervisor):
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

        # Pedestrians
        self.p1_t = self.getFromDef("Ped1").getField("translation")  # head-on
        self.p2_t = self.getFromDef("Ped2").getField("translation")  # crossing path
        self.p3_t = self.getFromDef("Ped3").getField("translation")  # overtaking

        # Parameters
        self.avoid_radius = 0.35
        self.cross_radius = 0.6

        # Ped2 state
        self.focus_ped2 = False
        self.ped2_start_y = None
        self.flee_from_ped2 = False
        self.dodge_dir_ped2 = (0, 0)

    @staticmethod
    def _norm(x, y):
        n = math.hypot(x, y)
        return (x/n, y/n) if n > 1e-6 else (0, 0)

    def run(self):
        while self.step(self.dt) != -1:
            # Destination vector
            dx = self.destination[0] - self.pos[0]
            dy = self.destination[1] - self.pos[1]
            dist_goal = math.hypot(dx, dy)

            if dist_goal < self.goal_eps:
                print("✅ Destination reached")
                continue

            gx, gy = self._norm(dx, dy)
            dir_x, dir_y = gx, gy  # default

            # Pedestrian positions
            p1 = self.p1_t.getSFVec3f()
            p2 = self.p2_t.getSFVec3f()
            p3 = self.p3_t.getSFVec3f()

            d1 = math.hypot(p1[0] - self.pos[0], p1[1] - self.pos[1])
            d2 = math.hypot(p2[0] - self.pos[0], p2[1] - self.pos[1])
            d3 = math.hypot(p3[0] - self.pos[0], p3[1] - self.pos[1])

            # === PRIORITY 1: Ped2 crossing ===
            if d2 < self.cross_radius or self.focus_ped2 or self.flee_from_ped2:
                if not self.focus_ped2 and not self.flee_from_ped2:
                    self.focus_ped2 = True
                    self.ped2_start_y = self.pos[1]
                    self.dodge_dir_ped2 = (0, 1) if p2[1] < self.pos[1] else (0, -1)
                    print("⚠️ Focus mode: Ped2 crossing")

                if self.focus_ped2:
                    # Full dodge
                    dir_x, dir_y = self.dodge_dir_ped2
                    # Switch to flee mode after 0.07m lateral shift
                    if self.ped2_start_y is not None and abs(self.pos[1] - self.ped2_start_y) >= 0.07:
                        self.focus_ped2 = False
                        self.flee_from_ped2 = True
                        print("🏃 Switching to flee mode from Ped2")

                elif self.flee_from_ped2:
                    # Flee opposite to Ped2 direction, but only if Ped1 and Ped3 are not threats
                    if d1 >= self.avoid_radius and d3 >= self.avoid_radius:
                        # Opposite of Ped2's position relative to Agent
                        relx, rely = self.pos[0] - p2[0], self.pos[1] - p2[1]
                        dir_x, dir_y = self._norm(relx, rely)
                        print("↩️ Fleeing away from Ped2")
                        # Exit flee once sufficiently separated
                        if d2 > self.cross_radius:
                            self.flee_from_ped2 = False
                            print("✅ Ped2 cleared, resuming goal")
                    else:
                        # If Ped1 or Ped3 present, handle them normally
                        self.flee_from_ped2 = False

            # === PRIORITY 2: Ped3 overtaking ===
            elif d3 < self.avoid_radius and p3[0] < self.pos[0]:
                avoid = (0, 1) if p3[1] < self.pos[1] else (0, -1)
                dir_x = 0.4*gx + 0.6*avoid[0]
                dir_y = 0.4*gy + 0.6*avoid[1]
                print("↔️ Overtaking Ped3")

            # === PRIORITY 3: Ped1 head-on ===
            elif d1 < self.avoid_radius:
                avoid = (0, 1) if p1[1] < self.pos[1] else (0, -1)
                dir_x = 0.5*gx + 0.5*avoid[0]
                dir_y = 0.5*gy + 0.5*avoid[1]
                print("⬅️ Avoiding Ped1")

            # Normalize
            dir_x, dir_y = self._norm(dir_x, dir_y)

            # Move
            self.pos[0] += dir_x * self.speed
            self.pos[1] += dir_y * self.speed
            self.t_field.setSFVec3f(self.pos)

            # Rotate
            self.yaw = math.atan2(dir_y, dir_x)
            self.r_field.setSFRotation([0, 0, 1, self.yaw])


# Example run
controller = AgentM7(destination=[-2.0, 0.0])
controller.run()
