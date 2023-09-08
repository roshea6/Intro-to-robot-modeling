import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class VelPubNode(Node):
    # def __init__(self):
    #     super().__init__('openloop_controller')

    #     # Publisher for the turtlebot's velocity
    #     self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    #     # Set max velocity limit for the turtlebot
    #     self.BURGER_MAX_LIN_VEL = 0.5

    def __init__(self):
        super().__init__('minimal_service')
        # self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    print('Hi from tb_control.')


if __name__ == '__main__':
    main()
