import os
import time

import rclpy

from sound_play.libsoundplay import SoundClient

from sound_msgs.msg import SoundRequest

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('soundplay_test' + str(os.getpid()))
    soundhandle = SoundClient(node)

    time.sleep(1)

    soundhandle.stopAll()

    node.get_logger().info(
        "This script will run continuously until you hit CTRL+C, "
        "testing various sound_node sound types.")

    node.get_logger().info('wave')
    soundhandle.playWave('Darth-vader-breathing.wav')
    time.sleep(2)

