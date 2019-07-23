#!/usr/bin/env python

import rospy
import argparse

class CrazyflieParser:
    def __init__(self, index, initialPosition):
        parser = argparse.ArgumentParser()
        parser.add_argument("--sim", help="Run using simulation", action="store_true")
        parser.add_argument("--vis", help="(sim only) Visualization backend [mpl]", choices=['mpl', 'vispy'], default="mpl")
        parser.add_argument("--dt", help="(sim only) dt [0.1s]", type=float, default=0.1)
        parser.add_argument("--writecsv", help="(sim only) Enable CSV output", action="store_true")
        args, unknown = parser.parse_known_args()

        if args.sim:
            from crazyflieSim import Crazyflie, TimeHelper
            self.timeHelper = TimeHelper(args.vis, args.dt, args.writecsv)
            cf = Crazyflie(index, initialPosition, self.timeHelper)
            self.crazyflies = []
            self.crazyflies.append(cf)
            self.timeHelper.crazyflies = self.crazyflies
        else:
            from crazyflie import Crazyflie, TimeHelper
            self.timeHelper = TimeHelper()
            rospy.init_node('CrazyflieParser')
            cf = Crazyflie(index, initialPosition, self.timeHelper)
            self.crazyflies = []
            self.crazyflies.append(cf)
