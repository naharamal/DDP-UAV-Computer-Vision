import pickle as pkl
from matplotlib import pyplot as plt

with open("trajectory.pkl",'rb') as f:
        data = pkl.load(f)

# ax = plt.axes(projection='3d')

# ax = plt.axes(projection='3d')
plt.axis('equal')

zline = [-i[2] for i in data]
xline = [i[0] for i in data]
yline = [i[1] for i in data]
# ax.plot3D(xline, yline, zline, 'gray')
plt.plot(xline,yline)
plt.show()
# class plotting:
#     def __init__(self):
#         data = pkl.load( open( "error.pkl", "rb" ) )
#         yerror = [i[0] for i in data]
#         xerror = [i[1] for i in data]
#         aerror = [i[2] for i in data]
#         x = range(len(yerror))

#         fig = plt.figure()
#         plt.ylabel("yerror")
#         plt.xlabel("time")
#         plt.plot(x,yerror)

#         fig = plt.figure()
#         plt.ylabel("xerror")
#         plt.xlabel("time")
#         plt.plot(x,xerror)

#         fig = plt.figure()
#         plt.ylabel("aerror")
#         plt.xlabel("time")
#         plt.plot(x,aerror)

#         plt.show()
# # plotting()