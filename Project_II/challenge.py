import matplotlib.pyplot as plt
import numpy as np

grid = np.load("map.npy")

plt.imshow(grid, origin='lower', cmap='viridis')
plt.colorbar()
plt.title("Map")
plt.show()