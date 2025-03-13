import numpy as np
import matplotlib.pyplot as plt
with open('/root/racecar_ws/src/wall_follower_megan/wall_follower/data/unobs.npy', 'rb') as f:
    err = np.load(f)
    times = np.load(f)

plt.plot(times, err, marker='_', label='Error', color='r')
plt.savefig('whatever.png')
#print(np.min(err))