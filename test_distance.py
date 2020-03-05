import sys
import numpy as np



def dist(x, y):
  return np.sqrt(np.sum((x-y)**2)) * 100

p1 = np.array([-0.41271746158599854, 0.11680654436349869, 0.9780000448226929])
p2 = np.array([0.13187752664089203, 0.10235363245010376, 1.156000018119812])


d = dist(p1, p2)

print("Distance: ", d)


