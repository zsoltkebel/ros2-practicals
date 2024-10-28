import rclpy
import math
from rclpy.node import Node
import random
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from std_msgs.msg import String
from turtlesim.srv import Spawn, Kill
from catch_the_turtle.autopilot import Autopilot, TurtleMover

target_turtle_name = 'target'
speed = 1.5

class TurtleSpawner(Node):
    """Publishes a service request to turtlesim spawn service"""

    def __init__(self):
        super().__init__('turtle_spawner')
        self.cli = self.create_client(Spawn, '/turtlesim1/spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('spawn service available')

        self.req = Spawn.Request()

    def spawn_random(self, name: str="target"):
        x = float(random.uniform(0, 10))
        y = float(random.uniform(0, 10))
        theta = 0.0

        self.spawn(x, y, theta, name)

        return x, y, theta
    
    def spawn(self, x: float, y: float, theta: float, name: str):
        """floats are float32"""
        self.get_logger().info('Trying to spawn turtle' + name)
        # Message parameters for Spawn
        # http://docs.ros.org/en/noetic/api/turtlesim/html/srv/Spawn.html
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.req.name = name

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class TurtleKiller(Node):

    def __init__(self):
        super().__init__('turtle_killer')
        self.client = self.create_client(Kill, 'kill')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')
        
        self.get_logger().info('kill service is available')

    def kill_turtle(self, turtle_name: str):
        request = Kill.Request()
        request.name = turtle_name
        
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            self.get_logger().info(f'Successfully killed turtle: {turtle_name}')
        else:
            self.get_logger().error(f'Failed to kill turtle: {turtle_name}')


class Tutle:
    def __init__(self, x, y, theta, linear_velocity, angular_velocity, name):
        self.x = x
        self.y = y
        self.theta = theta

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        self.name = name
    
    def update(self, x: float, y: float, theta: float, linear_velocity: float, angular_velocity: float):
        self.x = x
        self.y = y
        self.theta = theta

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

class TurtlePositionSubscriber(Node):

    def __init__(self):
        super().__init__('turtle_postition_subscriber')

        # set up turtle representations
        self.main_turtle = None
        self.other_turtle = None

        self.killer = TurtleKiller()
        self.spawner = TurtleSpawner()

        self.goal = self.spawner.spawn_random(target_turtle_name)
        self.pos = None
        self.lin = 0.0
        self.ang = 0.0

        self.subscription = self.create_subscription(
            Pose,
            '/turtlesim1/target/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.main_pose_subscription = self.create_subscription(
            Pose,
            '/turtlesim1/turtle1/pose',
            self.main_turtle_pose_listener,
            10)

        

        self.publisher = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)
        # self.timer = self.create_timer(0.5, self.moveTurtle)
        # self.m = TurtleMover()
        # self.autopilot = Autopilot()

        # self.autopilot.update_goal(*goal)

    def listener_callback(self, msg: Pose):
        # self.get_logger().info('I heard: "%d", %d, %d, %d, %d' % (msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity))
        if self.other_turtle:
            self.goal = msg.x, msg.y, msg.theta
            self.other_turtle.update(msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity)
        else:
            self.other_turtle = Tutle(msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity, "other")
        self.check_closeness()

    def main_turtle_pose_listener(self, msg: Pose):
        if self.main_turtle:
            self.pos = msg
            self.main_turtle.update(msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity)
        else:
            self.main_turtle = Tutle(msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity, "main")
        self.check_closeness()

    def check_closeness(self):
        if self.main_turtle and self.other_turtle:
            dist = math.dist([self.main_turtle.x, self.main_turtle.y], [self.other_turtle.x, self.other_turtle.y])
            # self.get_logger().info('Distance between turtles: "%f"' % dist)
            if dist < 0.5:
                self.get_logger().info('turtle caught!')
                self.killer.kill_turtle(target_turtle_name)
                self.goal = self.spawner.spawn_random(target_turtle_name)
            elif self.pos:
                self.lin = float(self.linear_velocity())
                self.ang = float(self.angular_velocity())

                self.moveTurtle(self.lin, self.ang)

        else:
            # self.get_logger().info('Can\'t calculate distance yet, turtles havent been initialised.')
            pass
    
    # movement

    def moveTurtle(self, linearSpeed=2.0, angularSpeed=1.0):
        twist = Twist()
        twist.linear.x = linearSpeed
        twist.angular.z = angularSpeed
        self.publisher.publish(twist)
        self.get_logger().info(f"Moving turtle with linear speed {linearSpeed} and angular speeed {angularSpeed}")

    def angle_to_goal(self):
        x_g, y_g, theta_g = self.goal
        x_p, y_p, theta_p = self.pos.x, self.pos.y, self.pos.theta

        theta = math.atan2(y_g - y_p, x_g - x_p)
        
        # if theta < 0:
        #     theta += math.pi
        return theta

    def turn_to_goal(self):
        a = self.angle_to_goal() - self.pos.theta
        self.get_logger().info(f'pos angle: {self.pos.theta} angle to goal: {self.angle_to_goal()} turn to goal: {a}')
        if abs(a) < abs(math.pi - a):
            return a
        return math.pi - a

    def angular_velocity(self):
        return self.turn_to_goal() / speed

    def distance_to_goal(self):
        x_g, y_g, _ = self.goal
        x_p, y_p = self.pos.x, self.pos.y

        return math.dist((x_g, y_g), (x_p, y_p))

    def linear_velocity(self):
        return self.distance_to_goal() / speed

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = TurtlePositionSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()