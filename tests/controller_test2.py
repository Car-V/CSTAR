from pydualsense import pydualsense

ds = pydualsense()
ds.init()

print(ds.state.LX, ds.state.LY)
