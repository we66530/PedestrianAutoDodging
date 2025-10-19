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
destination = [-4.0, 0.0]   # goal
speed = 0.02
arrive_eps = 0.05
avoid_radius = 0.4   # how close before reacting

# === State tracking ===
prev_px, prev_py = None, None
stop_steps = 0
stop_duration = 15   # ~0.3s pause if Ped1 is fast

while robot.step(timestep) != -1:
    # Agent pos
    pos = t_field.getSFVec3f()
    ax, ay = pos[0], pos[1]

    # Ped pos
    ped_pos = ped_t_field.getSFVec3f()
    px, py = ped_pos[0], ped_pos[1]

    # Goal vector
    dx, dy = destination[0] - ax, destination[1] - ay
    dist_goal = math.hypot(dx, dy)

    if dist_goal > arrive_eps:
        goal_x, goal_y = dx / dist_goal, dy / dist_goal
        dir_x, dir_y = goal_x, goal_y  # default

        # Distance to Ped1
        dxp, dyp = px - ax, py - ay
        dist_ped = math.hypot(dxp, dyp)

        if stop_steps > 0:
            # currently paused
            stop_steps -= 1
            dir_x, dir_y = 0.0, 0.0
        elif dist_ped < avoid_radius and prev_px is not None:
            # Compute Ped1 velocity
            vpx = px - prev_px
            vpy = py - prev_py
            vlen = math.hypot(vpx, vpy)

            if vlen > 0.01:  # Ped1 moving
                if vlen > 0.02:
                    # Ped1 moving fast → pause
                    stop_steps = stop_duration
                    dir_x, dir_y = 0.0, 0.0
                else:
                    # Ped1 moving slow → dodge
                    dodge_x, dodge_y = -vpx / vlen, -vpy / vlen
                    dir_x = 0.3 * goal_x + 0.7 * dodge_x
                    dir_y = 0.3 * goal_y + 0.7 * dodge_y
            else:
                # Ped1 almost static → dodge sideways
                dodge_x, dodge_y = -dyp, dxp
                norm = math.hypot(dodge_x, dodge_y)
                if norm > 1e-6:
                    dodge_x, dodge_y = dodge_x / norm, dodge_y / norm
                dir_x = 0.3 * goal_x + 0.7 * dodge_x
                dir_y = 0.3 * goal_y + 0.7 * dodge_y

        # Normalize
        norm = math.hypot(dir_x, dir_y)
        if norm > 1e-6:
            dir_x, dir_y = dir_x / norm, dir_y / norm

        # Move Agent
        pos[0] += dir_x * speed
        pos[1] += dir_y * speed
        t_field.setSFVec3f(pos)

        # Rotate Agent (only if moving)
        if norm > 1e-6:
            yaw = math.atan2(dir_y, dir_x)
            r_field.setSFRotation([0, 0, 1, yaw])
    else:
        print("Destination reached!")

    # Update Ped1 history
    prev_px, prev_py = px, py

    # Camera recognition
    img = camera.getImage()
    if img:
        for obj in camera.getRecognitionObjects():
            cx, cy = obj.getPositionOnImage()
            w, h = obj.getSizeOnImage()
            print(f"[CAM] Detected ID {obj.getId()} at ({cx:.1f},{cy:.1f}), size=({w:.1f},{h:.1f})")
