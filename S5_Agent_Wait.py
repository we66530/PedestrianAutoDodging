from controller import Supervisor
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# === Self (Agent) ===
agent_node = robot.getSelf()
t_field = agent_node.getField("translation")
r_field = agent_node.getField("rotation")

# === Ped1 ===
ped_node = robot.getFromDef("Ped1")
ped_t_field = ped_node.getField("translation")

# === Camera ===
camera = robot.getDevice("CAM")
camera.enable(timestep)
camera.recognitionEnable(timestep)

# === Destination ===
destination = [-4.0, 0.0]   # goal (leftward)
speed = 0.02
arrive_eps = 0.05
avoid_radius = 0.35

# Stop timer
stop_steps = 0
stop_duration = 15  # stop ~0.3s if timestep=20ms

while robot.step(timestep) != -1:
    pos = t_field.getSFVec3f()
    ax, ay = pos[0], pos[1]

    ped_pos = ped_t_field.getSFVec3f()
    px, py = ped_pos[0], ped_pos[1]

    dx, dy = destination[0] - ax, destination[1] - ay
    dist_goal = math.hypot(dx, dy)

    if dist_goal > arrive_eps:
        goal_x, goal_y = dx / dist_goal, dy / dist_goal

        # Distance to Ped1
        dxp, dyp = px - ax, py - ay
        dist_ped = math.hypot(dxp, dyp)

        if stop_steps > 0:
            # Currently waiting
            stop_steps -= 1
            dir_x, dir_y = 0.0, 0.0
        elif dist_ped < avoid_radius:
            # Ped1 too close → option 1: stop
            stop_steps = stop_duration
            dir_x, dir_y = 0.0, 0.0

            # Option 2 (alternative): dodge away from Ped1 direction
            # ped_motion = (px - old_px, py - old_py)   # you'd track this across steps
            # dodge_dir = (-ped_motion[0], -ped_motion[1])  # opposite to Ped1's motion
            # normalize and blend with goal instead of stopping
        else:
            dir_x, dir_y = goal_x, goal_y

        # Normalize
        norm = math.hypot(dir_x, dir_y)
        if norm > 1e-6:
            dir_x /= norm
            dir_y /= norm

        # Move
        pos[0] += dir_x * speed
        pos[1] += dir_y * speed
        t_field.setSFVec3f(pos)

        # Rotate to face direction (skip if stopping)
        if norm > 1e-6:
            yaw = math.atan2(dir_y, dir_x)
            r_field.setSFRotation([0, 0, 1, yaw])
    else:
        print("Destination reached!")

    # Camera recognition logging
    img = camera.getImage()
    if img:
        for obj in camera.getRecognitionObjects():
            cx, cy = obj.getPositionOnImage()
            w, h = obj.getSizeOnImage()
            print(f"[CAM] Detected ID {obj.getId()} at ({cx:.1f},{cy:.1f}), size=({w:.1f},{h:.1f})")
