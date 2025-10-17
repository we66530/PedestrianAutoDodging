from controller import Supervisor
import math

class AvoidStaticPedestrian(Supervisor):
    def __init__(self, destination):
        # --- Initialize Supervisor ---
        Supervisor.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # --- Movement parameters ---
        self.ROOT_HEIGHT = 1.27      # fixed z-height of pedestrian
        self.base_speed = 0.1        # forward walking speed
        self.lateral_speed = 0.05    # sidestep (left/right) speed
        self.turn_speed = 0.03       # maximum rotation per step (radians)
        self.goal_tolerance = 0.15   # stopping radius near goal

        # --- Pedestrian’s body reference ---
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        self.position = self.root_translation_field.getSFVec3f()  # current (x,y,z)
        self.rotation_angle = 0.0    # facing angle (yaw)

        # --- Camera setup ---
        self.camera = self.getDevice("CAM")
        if self.camera:
            self.camera.enable(self.time_step)
            self.camera.recognitionEnable(self.time_step)  # allow object detection
            self.img_width = self.camera.getWidth()
            self.img_height = self.camera.getHeight()
            print(f"✅ Camera recognition enabled ({self.img_width}x{self.img_height})")
        else:
            print("❌ Camera not found")

        # --- Goal location (x,y) ---
        self.destination = destination

    # --- Utility: compute heading angle toward target point ---
    def compute_angle_to_point(self, target):
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        return math.atan2(dy, dx)

    # --- Utility: compute Euclidean distance to goal ---
    def distance_to_destination(self):
        dx = self.destination[0] - self.position[0]
        dy = self.destination[1] - self.position[1]
        return math.sqrt(dx*dx + dy*dy)

    # --- Main loop controlling pedestrian ---
    def run(self):
        while self.step(self.time_step) != -1:

            # --- Stop condition: close enough to goal ---
            if self.distance_to_destination() < self.goal_tolerance:
                print("🎯 Reached destination")
                continue  # skip rest, don’t move anymore

            # --- Compute desired heading toward goal ---
            dest_angle = self.compute_angle_to_point(self.destination)
            # Normalize angle difference into [-pi, pi]
            angle_diff = (dest_angle - self.rotation_angle + math.pi) % (2*math.pi) - math.pi
            # Apply limited turn per step (smooth rotation)
            self.rotation_angle += max(-self.turn_speed, min(self.turn_speed, angle_diff))

            # --- Default forward motion (no obstacle yet) ---
            forward_dx = math.cos(self.rotation_angle) * self.base_speed
            forward_dy = math.sin(self.rotation_angle) * self.base_speed
            lateral_dx, lateral_dy = 0.0, 0.0  # will be updated if obstacle detected

            # --- Check for obstacles using camera recognition ---
            if self.camera and self.camera.hasRecognition():
                for obj in self.camera.getRecognitionObjects():
                    obj_pos = obj.getPosition()  # (x,y,z) in camera coordinates
                    abs_dist = math.sqrt(obj_pos[0]**2 + obj_pos[1]**2 + obj_pos[2]**2)

                    # Consider only near obstacles (<1.5 m away)
                    if abs_dist < 1.5:
                        px, py = obj.getPositionOnImage()  # obstacle pixel location
                        # Define central "danger zone" in camera view
                        left_bound = self.img_width // 2 - 80
                        right_bound = self.img_width // 2 + 80

                        # If obstacle sits inside central zone → must dodge
                        if left_bound < px < right_bound:
                            if px > self.img_width // 2:
                                # Obstacle on right → sidestep left
                                lateral_dx = -math.sin(self.rotation_angle) * self.lateral_speed
                                lateral_dy = math.cos(self.rotation_angle) * self.lateral_speed
                                print(f"⬅️ Sidestep LEFT (obs right, dist={abs_dist:.2f}m)")
                            else:
                                # Obstacle on left → sidestep right
                                lateral_dx = math.sin(self.rotation_angle) * self.lateral_speed
                                lateral_dy = -math.cos(self.rotation_angle) * self.lateral_speed
                                print(f"➡️ Sidestep RIGHT (obs left, dist={abs_dist:.2f}m)")

            # --- Update final movement vector (forward + lateral) ---
            self.position[0] += forward_dx + lateral_dx
            self.position[1] += forward_dy + lateral_dy

            # --- Write new position and orientation back to Webots ---
            self.root_translation_field.setSFVec3f([
                self.position[0],
                self.position[1],
                self.ROOT_HEIGHT
            ])
            self.root_rotation_field.setSFRotation([0, 0, 1, self.rotation_angle])


# --- Example usage ---
destination_point = [2.0, -10.0]  # target destination in (x,y)
controller = AvoidStaticPedestrian(destination_point)
controller.run()
