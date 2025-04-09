from pydualsense import pydualsense, TriggerModes
from time import sleep

# def cross_pressed(state):
#     print(f"Cross button pressed: {state}")

ds = pydualsense()
ds.init()

try:
    while True:
        mesg = "True" if abs(ds.state.LX) < 10 and abs(ds.state.LY) < 10 else "False"
        print(f"LX: {ds.state.LX}, LY: {ds.state.LY}, {mesg}")
        sleep(0.5)
except KeyboardInterrupt:
    ds.close()
    print("Exiting")

# ds.cross_pressed += cross_pressed

# ds.light.setColorI(255, 0, 0)

# ds.triggerL.setMode(TriggerModes.Rigid)
# ds.triggerL.setForce(1, 255)


# try:
#     while True:
#         pass
# except KeyboardInterrupt:
#     print("Exiting")
#     ds.close()
