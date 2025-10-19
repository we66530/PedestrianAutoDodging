from controller import Supervisor
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# === Self node (Agent) ===
agent_node = robot.getSelf()
translation_field = agent_node.getField("translation")
rotation_field = agent_node.getField("rotation")

# === Pedestrian node ===
ped_node = robot.getFromDef("Ped1")
ped_translation_field = ped_node.getField("translation")

# === Camera with recognition ===
camera = robot.getDevice("CAM")
camera.enable(timestep)
camera.recognitionEnable(timestep)

# === Destination (XY plane only) ===
destination = [-2.0, 0.0]   # (x, y)

# Motion params
speed = 0.02
arrive_eps = 0.05
avoid_radius = 0.25

while robot.step(timestep) != -1:
    # Current Agent pos
    pos = translation_field.getSFVec3f()
    ax, ay = pos[0], pos[1]

    # Current Ped1 pos
    ped_pos = ped_translation_field.getSFVec3f()
    px, py = ped_pos[0], ped_pos[1]

    # Goal vector
    dx, dy = destination[0] - ax, destination[1] - ay
    dist_goal = math.hypot(dx, dy)

    if dist_goal > arrive_eps:
        # Goal direction
        goal_x, goal_y = dx / dist_goal, dy / dist_goal

        # Distance to Ped1
        dxp, dyp = ax - px, ay - py
        dist_ped = math.hypot(dxp, dyp)

        if dist_ped < avoid_radius:
            # Avoidance vector (perpendicular)
            avoid_x, avoid_y = -dyp, dxp
            avoid_len = math.hypot(avoid_x, avoid_y)
            if avoid_len > 1e-6:
                avoid_x /= avoid_len
                avoid_y /= avoid_len
            dir_x = 0.45 * goal_x + 0.55 * avoid_x
            dir_y = 0.45 * goal_y + 0.55 * avoid_y
        else:
            dir_x, dir_y = goal_x, goal_y

        # Normalize
        norm = math.hypot(dir_x, dir_y)
        if norm > 1e-6:
            dir_x /= norm
            dir_y /= norm

        # Step
        pos[0] += dir_x * speed
        pos[1] += dir_y * speed
        translation_field.setSFVec3f(pos)

        # Face direction
        yaw = math.atan2(dir_y, dir_x)
        rotation_field.setSFRotation([0, 0, 1, yaw])
    else:
        print("Destination reached!")

    # Camera recognition info
    img = camera.getImage()
    if img:
        for obj in camera.getRecognitionObjects():
            cx, cy = obj.getPositionOnImage()
            w, h = obj.getSizeOnImage()
            print(f"[CAM] Detected ID {obj.getId()} at ({cx:.1f},{cy:.1f}), size=({w:.1f},{h:.1f})")
