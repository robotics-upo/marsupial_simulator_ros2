import matplotlib.pyplot as plt
import numpy as np

num_elements = np.array([100, 200, 300, 400, 500, 600, 700])
# 32 GB of RAM, a 13th Gen Intel® Core™ i7-13620H, and an NVIDIA GeForce RTX 4060 Laptop GPU
real_time_factor_laptop = np.array([0.92, 0.53, 0.36, 0.25, 0.20, 0.17, 0.14])
# 64 GB of RAM, a 12th Gen Intel® Core™ i9-12900F, and an NVIDIA GeForce RTX 3060 GPU
real_time_factor_pc = np.array([0.99, 0.63, 0.47, 0.35, 0.28, 0.22, 0.19])

plt.figure(figsize=(10, 4))

# Laptop line (solid)
plt.plot(num_elements, real_time_factor_laptop, marker='o', linestyle='-', color='b', label='Laptop')

# PC line (dashed)
plt.plot(num_elements, real_time_factor_pc, marker='s', linestyle='--', color='r', label='Desktop PC')

plt.xlabel('Number of Tether Elements')
plt.ylabel('Real Time Factor')
plt.title('Effect of Tether Discretization on Simulation Performance')
plt.grid(True)
plt.legend()

# Save the plot in the specified directory
plt.savefig('/home/upo/marsupial/src/marsupial_simulator_ros2/plots/performance_evaluation.png', dpi=300)

plt.show()
