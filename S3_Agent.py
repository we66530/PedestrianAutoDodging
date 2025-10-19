from controller import Supervisor
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# === Self (Agent) ===
agent_node = robot.getSelf()
translation_field = agent_node.getField("translation")
rotation_field = agent_node.getField("rotation")

# === Ped1 ===
ped_node = robot.getFromDef("Ped1")
ped_translation_field = ped_node.getField("translation")

# === Camera ===
camera = robot.getDevice("CAM")
camera.enable(timestep)
camera.recognitionEnable(timestep)

# === Destination ===
destination = [-2.0, 0.0]   # Agent moves leftwards
speed = 0.02
arrive_eps = 0.05
avoid_radius = 0.25

while robot.step(timestep) != -1:
    # Agent position
    pos = translation_field.getSFVec3f()
    ax, ay = pos[0], pos[1]

    # Ped position
    ped_pos = ped_translation_field.getSFVec3f()
    px, py = ped_pos[0], ped_pos[1]

    # Goal vector
    dx, dy = destination[0] - ax, destination[1] - ay
    dist_goal = math.hypot(dx, dy)

    if dist_goal > arrive_eps:
        goal_x, goal_y = dx / dist_goal, dy / dist_goal

        # Distance to Ped1
        dxp, dyp = ax - px, ay - py
        dist_ped = math.hypot(dxp, dyp)

        if dist_ped < avoid_radius:
            # Head-on dodge: step sideways (perpendicular)
            avoid_x, avoid_y = -dyp, dxp
            avoid_len = math.hypot(avoid_x, avoid_y)
            if avoid_len > 1e-6:
                avoid_x /= avoid_len
                avoid_y /= avoid_len
            dir_x = 0.3 * goal_x + 0.8 * avoid_x   # stronger avoidance
            dir_y = 0.3 * goal_y + 0.8 * avoid_y
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

        # Rotate to face movement
        yaw = math.atan2(dir_y, dir_x)
        rotation_field.setSFRotation([0, 0, 1, yaw])
    else:
        print("Destination reached!")

    # Recognition info
    img = camera.getImage()
    if img:
        for obj in camera.getRecognitionObjects():
            cx, cy = obj.getPositionOnImage()
            w, h = obj.getSizeOnImage()
            print(f"[CAM] Detected ID {obj.getId()} at ({cx:.1f},{cy:.1f}), size=({w:.1f},{h:.1f})")
