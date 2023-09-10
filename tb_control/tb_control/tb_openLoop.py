import rclpy
from rclpy.node import Node
from tb_control.srv import SetGoal

from geometry_msgs.msg import Twist

class VelPubNode(Node):
    def __init__(self):
        super().__init__('openloop_controller')

        # Publisher for the turtlebot's velocity
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Set max velocity limit for the turtlebot
        self.BURGER_MAX_LIN_VEL = 0.5

        self.commmand_srv = self.create_service(SetGoal, 'set_goal', self.executeNavCB)

    def executeNavCB(self, request, response):
        self.get_logger().info('Incoming request\ndist: %d time: %d' % (request.dist, request.time))

        # Calculate the velocity required to cover the distance in the specified
        # amount of time
        desired_vel = float(request.dist)/float(request.time)

        # Create a new twist message and populate it with the desired x velocity
        vel_msg = Twist()
        vel_msg.linear.x = desired_vel

        # TODO: Make velocity profile so it starts up, drives for a while at the desired speed,
        # then comes to a stop

        # Publish the message to the turtlebot
        self.vel_pub.publish(vel_msg)

        response.status = desired_vel
        return response


def main():
    # Initialize the node
    rclpy.init()

    # Create an instance of the Node
    vel_node = VelPubNode()

    # Start the node to run until it is killed
    rclpy.spin(vel_node)

    # Shutdown the node once it exits
    rclpy.shutdown()


if __name__ == '__main__':
    main()
