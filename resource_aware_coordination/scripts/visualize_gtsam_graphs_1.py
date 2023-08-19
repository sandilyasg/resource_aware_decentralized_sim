import numpy as np
import matplotlib.pyplot as plt
import os
import pdb
import sys
import re

cwd = os.getcwd()  # Get the current working directory (cwd)
print(cwd)
cwdsplit = cwd.split('/')
if cwdsplit[-1] == 'src':
  os.chdir('resource_aware_coordination/data')
else:
    os.chdir('../data')
print(os.getcwd())

# datafile = 'tb3_0_graph1.txt'
datafile = 'tb3_0_graph2.txt'

# Parse poses
poses = []
for line in open(datafile):
  if re.match(r'Value \d+:', line):
    id = int(line.split()[1][:-1])
    x, y, theta = [float(s) for s in line.split('(')[2].split(')')[0].split(',')]
    poses.append((id, np.array([x, y, theta])))

poses = dict(poses)

# Parse landmarks  
landmarks = []
for line in open(datafile):
  if line.startswith('Value l'):
    id = line.split()[1][:-1]
    x, y = [float(s) for s in line.split('(')[2].split(')')[0].split(',')]    
    landmarks.append((id, np.array([x, y])))
landmarks = dict(landmarks)

# Parse edges
edges = []
for line in open(datafile):
  if line.startswith('Edge between'):
    id1, id2 = line.split('and')[-2:]
    id1 = id1.split(' ')[1]
    id2 = id2[:-1]
    edges.append((id1.strip(), id2.strip()))

# Plot
fig, ax = plt.subplots()
ax.set_aspect(1)
ax.grid()
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

# Plot poses
# pose_xs = [p[1][0] for p in poses.values()]
pose_xs = [p[0] for p in poses.values()]
pose_ys = [p[1] for p in poses.values()]
# ax.plot(pose_xs, pose_ys, 'go')
# ax.plot(pose_xs, [-y for y in pose_ys], 'go')
ax.plot(pose_ys, [-y for y in pose_xs], 'go', markersize=7)

for i, id1 in enumerate(poses.keys()):
  if i < len(poses) - 1:
    id2 = list(poses.keys())[i+1]
    # ax.plot([poses[id1][0], poses[id2][0]],[poses[id1][1], poses[id2][1]], 'b-')
    ax.plot([poses[id1][1], poses[id2][1]], [-poses[id1][0], -poses[id2][0]], 'b-')
          
# Plot landmarks        
landmark_xs = [l[0] for l in landmarks.values()]
landmark_ys = [l[1] for l in landmarks.values()]
# ax.plot(landmark_xs, landmark_ys, 'rp')
# ax.plot(landmark_xs, [-y for y in landmark_ys], 'rp')
ax.plot(landmark_ys, [-y for y in landmark_xs], 'p', color='red', markersize=10)

for id1, id2 in edges:
  if id2[0] == 'l':
    # id2 is a landmark  
    # ax.plot([poses[int(id1)][0], landmarks[id2][0]], 
    #         [poses[int(id1)][1], landmarks[id2][1]], 'r-')        
      
    ax.plot([poses[int(id1)][1], landmarks[id2][1]], [-poses[int(id1)][0], -landmarks[id2][0]],'y-')          

plt.show()