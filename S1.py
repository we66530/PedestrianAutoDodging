from controller import Supervisor
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# === Self node (Agent) ===
agent_node = robot.getSelf()
translation_field = agent_node.getField("translation")
rotation_field = agent_node.getField("rotation")

# === Pedestrian node (must have DEF Ped1 in world file) ===
ped_node = robot.getFromDef("Ped1")
ped_translation_field = ped_node.getField("translation")

# === Camera with recognition ===
camera = robot.getDevice("CAM")
camera.enable(timestep)
camera.recognitionEnable(timestep)   # enable object recognition

# === Destination (XY plane only) ===
destination = [-2.0, 0.0]   # (x, y)

# Motion parameters
speed = 0.02
arrive_eps = 0.05
avoid_radius = 0.25

while robot.step(timestep) != -1:
    # Current position of Agent
    pos = translation_field.getSFVec3f()
    ax, ay = pos[0], pos[1]

    # Current position of Ped1
    ped_pos = ped_translation_field.getSFVec3f()
    px, py = ped_pos[0], ped_pos[1]

    # Vector to destination
    dx = destination[0] - ax
    dy = destination[1] - ay
    dist_goal = math.hypot(dx, dy)

    if dist_goal > arrive_eps:  # not arrived yet
        # Normalized goal direction
        goal_x = dx / dist_goal
        goal_y = dy / dist_goal

        # Distance to Ped1
        dxp = ax - px
        dyp = ay - py
        dist_ped = math.hypot(dxp, dyp)

        if dist_ped < avoid_radius:  # too close, dodge
            # Avoidance vector: perpendicular to Ped1->Agent
            avoid_x = -dyp
            avoid_y = dxp
            avoid_len = math.hypot(avoid_x, avoid_y)
            if avoid_len > 1e-6:
                avoid_x /= avoid_len
                avoid_y /= avoid_len
            # Blend goal and avoidance
            dir_x = 0.5 * goal_x + 0.5 * avoid_x
            dir_y = 0.5 * goal_y + 0.5 * avoid_y
        else:
            dir_x, dir_y = goal_x, goal_y

        # Normalize
        norm = math.hypot(dir_x, dir_y)
        if norm > 1e-6:
            dir_x /= norm
            dir_y /= norm

        # Move Agent
        pos[0] += dir_x * speed
        pos[1] += dir_y * speed
        translation_field.setSFVec3f(pos)

        # Rotate to face direction
        angle = math.atan2(dir_y, dir_x)
        rotation_field.setSFRotation([0, 0, 1, angle])
    else:
        print("Destination reached!")

    # === Camera recognition ===
    image = camera.getImage()
    if image:
        objects = camera.getRecognitionObjects()
        for obj in objects:
            cx, cy = obj.getPositionOnImage()
            w, h = obj.getSizeOnImage()
            print(f"[CAM] Detected ID {obj.getId()} at ({cx:.1f},{cy:.1f}), size=({w:.1f},{h:.1f})")
        # NOTE: Webots will automatically draw bounding boxes in Camera window
