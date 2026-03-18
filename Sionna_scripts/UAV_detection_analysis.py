import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from sionna.rt import load_scene, Transmitter, Receiver, PlanarArray, PathSolver
from sionna.rt import *
import time as time
import json
import random

start = time.time()

scene = load_scene("/home/bkumar2/Documents/Research/OSM_maps_and_terrain_files/Drone.xml", merge_shapes=False)
scene.frequency = 3.6192e9

BS_locations = np.array([np.random.randint(-50, 50, size=10), 10*np.ones(10), np.random.randint(-50, 50, size=10)])

for iter in range(0, 10):
    visible_drone = np.zeros([200, 200])

    for i_x in range(0, 200):
        for i_y in range(0, 200):
            tx_location = np.array([i_x-100, 1, i_y-100]) # Location of UE(s)
            rx_location =  BS_locations[:, iter]# Location of BS

            print("No. of iterations done = {}".format(iter*200*200 + 200*i_x + i_y))

            scene.tx_array = PlanarArray(num_rows=1,
                                        num_cols=1,
                                        vertical_spacing=0.5,
                                        horizontal_spacing=0.5,
                                        pattern="iso",
                                        polarization="VH")

            # Configure antenna array for all receivers
            scene.rx_array = PlanarArray(num_rows=1,
                                        num_cols=1,
                                        vertical_spacing=0.5,
                                        horizontal_spacing=0.5,
                                        pattern="iso",
                                        polarization="V")

            tx = Transmitter(name="tx",
                            position=tx_location,
                            display_radius=0.2)

            scene.add(tx)

            rx = Receiver(name="rx",
                        position=rx_location,
                        display_radius=0.2)

            scene.add(rx)

            # Instantiate a path solver
            # The same path solver can be used with multiple scenes
            p_solver  = PathSolver()

            # Compute propagation paths
            paths = p_solver(scene=scene,
                            max_depth=1,
                            #max_num_paths_per_src=3,
                            los=True,
                            specular_reflection=True,
                            diffuse_reflection=True,
                            refraction=True,
                            synthetic_array=False,
                            seed=41)
            
            if np.shape(paths.vertices)[-2] > 2:
                visible_drone[i_x, i_y] = 1

            scene.remove("tx")
            scene.remove("rx")
    
    np.savetxt('visible_drone' + str(iter) + '.csv', visible_drone, fmt='%d', delimiter=',')


end = time.time()

print("Execution time {}".format(end-start))

print(BS_locations)

# visible_drone = np.loadtxt('visible_drone1.csv', delimiter=',')
# print(visible_drone)

plt.imshow(visible_drone, extent=[-100, 100, 100, -100])
plt.xlabel("Y-axis", fontsize=40)
plt.ylabel("X-axis", fontsize=40)
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.show()


# BS locations
#[[ 32. -24. -21. -21.   9. -30. -42. -45.  32.  44.],
# [ 10.  10.  10.  10.  10.  10.  10.  10.  10.  10.],
# [ 19. -27.   1. -41. -43.  46. -26.  37. -41.  46.]]