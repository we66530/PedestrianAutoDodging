from controller import Supervisor
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Self (Agent)
agent_node = robot.getSelf()
t_field = agent_node.getField("translation")
r_field = agent_node.getField("rotation")

# Ped1
ped_node = robot.getFromDef("Ped1")
ped_t_field = ped_node.getField("translation")

# Camera
camera = robot.getDevice("CAM")
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Destination
destination = [-2.0, 0.0]   # goal further along -X
speed = 0.02
arrive_eps = 0.05
safe_distance = 0.3   # trigger overtaking if closer than this

while robot.step(timestep) != -1:
    # Agent pos
    pos = t_field.getSFVec3f()
    ax, ay = pos[0], pos[1]

    # Ped pos
    ped_pos = ped_t_field.getSFVec3f()
    px, py = ped_pos[0], ped_pos[1]

    # Vector to destination
    dx, dy = destination[0] - ax, destination[1] - ay
    dist_goal = math.hypot(dx, dy)

    if dist_goal > arrive_eps:
        goal_x, goal_y = dx / dist_goal, dy / dist_goal

        # Check distance to Ped1 (only if ahead in -X direction)
        dxp, dyp = px - ax, py - ay
        dist_ped = math.hypot(dxp, dyp)

        if dist_ped < safe_distance and px < ax:  
            # Ped1 is ahead & too close → OVERTAKE to the side
            avoid_x, avoid_y = 0.0, 1.0    # slide sideways (positive Y)
            dir_x = 0.4 * goal_x + 0.6 * avoid_x
            dir_y = 0.4 * goal_y + 0.6 * avoid_y
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

        # Rotate to face direction
        yaw = math.atan2(dir_y, dir_x)
        r_field.setSFRotation([0, 0, 1, yaw])
    else:
        print("Destination reached!")

    # Camera recognition
    img = camera.getImage()
    if img:
        for obj in camera.getRecognitionObjects():
            cx, cy = obj.getPositionOnImage()
            w, h = obj.getSizeOnImage()
            print(f"[CAM] Detected ID {obj.getId()} at ({cx:.1f},{cy:.1f}), size=({w:.1f},{h:.1f})")
