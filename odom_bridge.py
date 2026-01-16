import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # QoS profile to match PX4 /fmu/out/vehicle_odometry publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribe to PX4's vehicle_odometry topic
        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        # Publish a standard ROS2 Odometry message for Unity
        self.pub = self.create_publisher(Odometry, '/drone/odom', 10)

        self.get_logger().info('✅ odom_bridge started: /fmu/out/vehicle_odometry -> /drone/odom')

    def odom_callback(self, msg):
    	odom = Odometry()

    	odom.header.frame_id = 'map'
    	odom.child_frame_id = 'base_link'

    	# PX4 -> ROS2 : cast to Python float
    	odom.pose.pose.position = Point(
        	x=float(msg.position[0]),
        	y=float(msg.position[1]),
        	z=float(msg.position[2])
    	)

    	# PX4 quaternion order is [w, x, y, z] -> ROS needs (x,y,z,w)
    	odom.pose.pose.orientation = Quaternion(
        	x=float(msg.q[1]),
        	y=float(msg.q[2]),
        	z=float(msg.q[3]),
        	w=float(msg.q[0])
    	)

    	odom.twist.twist.linear.x = float(msg.velocity[0])
    	odom.twist.twist.linear.y = float(msg.velocity[1])
    	odom.twist.twist.linear.z = float(msg.velocity[2])

    	self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
