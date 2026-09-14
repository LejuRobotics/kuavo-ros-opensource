""" Define the input and output of this package to integrate with the ROS framework """

import argparse
import pathlib
import time
from kuavo_gmr import GeneralMotionRetargeting as GMR
from kuavo_gmr import RobotMotionViewer
from kuavo_gmr.utils.lafan1 import load_bvh_file
from kuavo_gmr.utils.pico import load_pico_file
from rich import print
from tqdm import tqdm
import os
import numpy as np
from scripts.gmr_ros_node_dev import GMRROSNodeAbs


if __name__ == "__main__":

    retargeter = GMRROSNodeAbs()
    
    try:
        # NOTE legal input for the GMR retargeter node
        # The frame names and relative definition can be seen in debug/pico_source_skeleton.png
        pico_frame = {name: [np.zeros(3), np.ones(4) / 2] for name in retargeter.pico_body_names}
        # NOTE the retarget output is dictionary as shown in the class function retarget(self, pico_frame: dict) -> dict
        retarget_output = retargeter.retarget(pico_frame)
        print("Correct input of the retargeter:")
        print(pico_frame)
        print("--------------------------------")
        print("Output of the retargeter:")
        print(retarget_output)
        print("--------------------------------")
    except ValueError as e:
        print("Illegal input: wrong bone data length. Data of the input pico frame")
        print(pico_frame)
        print("should be like:")
        print([[np.zeros(3), np.ones(4) / 2] for name in retargeter.pico_body_names])
        print("--------------------------------")
