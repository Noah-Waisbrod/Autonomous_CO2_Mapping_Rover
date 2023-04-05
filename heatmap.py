import numpy as np
import matplotlib.pyplot as plt

# create 100x100 array of 0's
arr = np.zeros((100, 100), dtype=int)

# define circle centers and radius
circles = [(20,20), (20,50), (20,80), (50,20), (50,50), (80,50), (80,20), (50,80), (80,80)]
radius = 15

# define circle intensities
intensities = [400, 300, 300, 500, 400, 200, 400, 200, 100]

# loop over each element in array
for i in range(100):
    for j in range(100):
        # calculate distance from each circle center
        distances = [np.sqrt((i-x)**2 + (j-y)**2) for (x,y) in circles]
        # calculate value based on distance and radius
        value = sum([max(0, radius-d) * intensity for d, intensity in zip(distances, intensities) if d <= radius])
        # set value in array
        arr[i,j] = value

# scale the array to have a maximum value of 1
arr = 1000*arr / arr.max()

# plot heatmap using Matplotlib
plt.imshow(arr, cmap='hot', interpolation='nearest')
plt.colorbar()
plt.show()