from controller import Supervisor
import math

class AvoidStaticPedestrian(Supervisor):
    def __init__(self, destination):
        Supervisor.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # Movement parameters
        self.ROOT_HEIGHT = 1.27
        self.base_speed = 0.1
        self.lateral_speed = 0.05
        self.turn_speed = 0.03
        self.goal_tolerance = 0.15

        # Node references
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        self.position = self.root_translation_field.getSFVec3f()
        self.rotation_angle = 0.0

        # Camera setup
        self.camera = self.getDevice("CAM")
        if self.camera:
            self.camera.enable(self.time_step)
            self.camera.recognitionEnable(self.time_step)
            self.img_width = self.camera.getWidth()
            self.img_height = self.camera.getHeight()
            print(f"✅ Camera recognition enabled ({self.img_width}x{self.img_height})")
        else:
            print("❌ Camera not found")

        # Destination
        self.destination = destination

    def compute_angle_to_point(self, target):
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        return math.atan2(dy, dx)

    def distance_to_destination(self):
        dx = self.destination[0] - self.position[0]
        dy = self.destination[1] - self.position[1]
        return math.sqrt(dx*dx + dy*dy)

    def run(self):
        while self.step(self.time_step) != -1:
            # --- Stop if reached goal ---
            if self.distance_to_destination() < self.goal_tolerance:
                print("🎯 Reached destination")
                continue

            # Default forward direction
            dest_angle = self.compute_angle_to_point(self.destination)
            angle_diff = (dest_angle - self.rotation_angle + math.pi) % (2*math.pi) - math.pi
            self.rotation_angle += max(-self.turn_speed, min(self.turn_speed, angle_diff))

            forward_dx = math.cos(self.rotation_angle) * self.base_speed
            forward_dy = math.sin(self.rotation_angle) * self.base_speed
            lateral_dx, lateral_dy = 0.0, 0.0

            # --- Check for static pedestrian obstacle ---
            if self.camera and self.camera.hasRecognition():
                for obj in self.camera.getRecognitionObjects():
                    obj_pos = obj.getPosition()  # 3D in camera frame (x,y,z)
                    abs_dist = math.sqrt(obj_pos[0]**2 + obj_pos[1]**2 + obj_pos[2]**2)

                    # Consider obstacle only if close (<1.5m)
                    if abs_dist < 1.5:
                        px, py = obj.getPositionOnImage()
                        left_bound = self.img_width // 2 - 80
                        right_bound = self.img_width // 2 + 80

                        # If obstacle in front-center → sidestep
                        if left_bound < px < right_bound:
                            if px > self.img_width // 2:
                                # obstacle slightly right → sidestep left
                                lateral_dx = -math.sin(self.rotation_angle) * self.lateral_speed
                                lateral_dy = math.cos(self.rotation_angle) * self.lateral_speed
                            else:
                                # obstacle slightly left → sidestep right
                                lateral_dx = math.sin(self.rotation_angle) * self.lateral_speed
                                lateral_dy = -math.cos(self.rotation_angle) * self.lateral_speed

                            print(f"🚶 Avoiding static pedestrian (dist={abs_dist:.2f}m, px={px})")

            # --- Apply movement ---
            self.position[0] += forward_dx + lateral_dx
            self.position[1] += forward_dy + lateral_dy

            self.root_translation_field.setSFVec3f([
                self.position[0],
                self.position[1],
                self.ROOT_HEIGHT
            ])
            self.root_rotation_field.setSFRotation([0, 0, 1, self.rotation_angle])


# Example usage
destination_point = [2.0, -10.0]
controller = AvoidStaticPedestrian(destination_point)
controller.run()
