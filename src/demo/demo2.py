#!/usr/bin/env python

from output.base_output import MOVO_output
from scipy.spatial.transform import Rotation

# Create a rotation object from Euler angles specifying axes of rotation
rot = Rotation.from_euler('xyz', [90, 45, 30], degrees=True)

# Convert to quaternions and print
rot_quat = rot.as_quat()
print(rot_quat)
print(rot.as_euler('xyz', degrees=True))

rot = Rotation.from_quat(rot_quat)

# Convert the rotation to Euler angles given the axes of rotation
print(rot.as_euler('xyz', degrees=True))


class Demo1(object):
    def __init__(self):
        pass

    def main(self):
        op = MOVO_output()
        op.target('Linear', )


"""draft
1. camera, pull value
2. trigger
3. check loop publish, output
4. finish and continue
5. finish line

"""
