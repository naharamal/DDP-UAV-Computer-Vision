import pickle as pkl
from matplotlib import pyplot as plt

# with open("trajectory.pkl",'rb') as f:
#         data = pkl.load(f)
data = pkl.load( open( "trajectory.pkl", "rb" ) )
fig = plt.figure()
ax = plt.axes(projection='3d')

ax = plt.axes(projection='3d')

# Data for a three-dimensional line
zline = [-i[2] for i in data]
xline = [i[0] for i in data]
yline = [i[1] for i in data]
ax.plot3D(xline, yline, zline, 'gray')

plt.show()