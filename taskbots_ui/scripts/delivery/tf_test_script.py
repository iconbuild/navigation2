import factory_utils as f_util
import tf2_ros
import rclpy
from rclpy.node import Node
import numpy as np

class FrameListener(Node):

    def __init__(self):
        super().__init__('tf2_frame_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def get_tf(self,target_link: str,
                ref_link=None,
                target_time=None) -> np.ndarray:
            """
            This will provide the transformation of the marker,
            if ref_link is not provided, we will use robot's base_link as ref
            :param now :
            :return : 4x4 homogenous matrix, None if not avail
            """
            if ref_link is None:
                ref_link = 'base_link'
            if target_time is None:
                target_time = rclpy.time.Time()
            try:                
                return f_util.get_mat_from_transfrom_msg(
                    self.tf_buffer.lookup_transform(
                        ref_link, target_link, target_time,
                        rclpy.duration.Duration(seconds=1.0))
                )
           
            except:
                return None

def main(args=None):
    rclpy.init(args=args)
    # rclpy.init()
    fl = FrameListener()
    try:
        rclpy.spin(fl)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()



if __name__ == '__main__':
    main()
