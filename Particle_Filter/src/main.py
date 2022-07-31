
import cv2
import numpy as np

# Particle pose
robot_x = 0
robot_y = 0
robot_orientation = 0 # np.pi/2
depth = 2
METRE_TO_PIXEL_SCALE = 50

map_image = cv2.imread("./fishermans_small.png")
robot_image = np.zeros((400, 400, 3))


K = np.array([
[238.3515418007097/depth, 0.0, 200.5], [0.0, 238.3515418007097/depth, 200.5], [0.0, 0.0, 1.0]
])

T_camera_offset = np.array([
    [1, 0, -0.07],
    [0, 1, -0.384],
    [0, 0, 1]
])

R = np.array([
    [np.cos(np.pi + robot_orientation), np.sin(np.pi + robot_orientation), 0],
    [-np.sin(np.pi + robot_orientation), np.cos(np.pi + robot_orientation), 0],
    [0, 0, 1]
])

T_robot = np.array([
    [1, 0, -8.0-robot_x],
    [0, 1, -25.2-robot_y],
    [0, 0, 1]
])

T_inv = METRE_TO_PIXEL_SCALE * np.linalg.inv(np.matmul(np.matmul(np.matmul(K, T_camera_offset), R),  T_robot))

x = np.matmul(T_inv, [0,0,1])

# u,v = 0,0 -> 0, 0
# u,v = 0,1 -> 0, 0.2
for u in range(400):
    for v in range(400):
        coord_reef = np.matmul(T_inv, [u, v, 1])
        robot_image[u,v] = map_image[map_image.shape[0]-int(coord_reef[1]), int(coord_reef[0])]

N_particles = 100

span = 1
center = 200
for i in range(N_particles):
    diff = 0
    for u in range(center-span, center+span):
        for v in range(center - span, center + span):
            coord_reef = np.matmul(T_inv, [u, v, 1])
            reef_pixel = map_image[map_image.shape[0] - int(coord_reef[1]), int(coord_reef[0])]
            # diff += np.diff

# cv2.imshow("img", robot_image/255)
cv2.imwrite("./output.png", robot_image)
# cv2.waitKey(0)