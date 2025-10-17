# PedestrianAutoDodging
The project is built on Webots, aim at dodging the pedestrian on the road

| Scenario ID | Pedestrian 1                  | Pedestrian 2                       | Pedestrian 3 | Training Goal                                     |
| ----------- | ----------------------------- | ---------------------------------- | ------------ | ------------------------------------------------- |
| **S1**      | Static (standing still)       | –                                  | –            | Learn to treat stationary pedestrian as obstacle. |
| **S2**      | Crossing path (left → right)  | –                                  | –            | Basic lateral dodging.                            |
| **S3**      | Head-on (toward agent)        | –                                  | –            | Evasive maneuver against frontal collision.       |
| **S4**      | Same direction (slower ahead) | –                                  | –            | Learn overtaking behavior.                        |
| **S5**      | Diagonal approach (45° angle) | –                                  | –            | Handle near-collision from oblique entry.         |
| **M1**      | Static                        | Crossing path                      | –            | Combine stationary & moving obstacle.             |
| **M2**      | Head-on                       | Head-on                            | –            | Multiple frontal threats.                         |
| **M3**      | Crossing path                 | Diagonal approach                  | –            | Learn to prioritize dodging order.                |
| **M4**      | Static                        | Head-on                            | Overtaking   | Mixed obstacle types in sequence.                 |
| **M5**      | Diagonal approach             | Random walking                     | –            | Adapt to unpredictable movement.                  |
| **M6**      | Crossing path                 | Crossing path (opposite direction) | Static       | Multi-lane crossing with distraction.             |
| **M7**      | Head-on                       | Crossing path                      | Overtaking   | High-stress mixed scenario.                       |
