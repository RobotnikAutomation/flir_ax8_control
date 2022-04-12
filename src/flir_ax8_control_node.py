#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flir_ax8_control import FlirAx8Control


def main():

    rospy.init_node("flir_ax8_control_node")

    rc_node = FlirAx8Control()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
