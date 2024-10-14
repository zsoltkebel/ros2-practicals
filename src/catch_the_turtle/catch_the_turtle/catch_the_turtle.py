import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    """Publishes a service request to turtlesim spawn service"""

    def __init__(self):
        super().__init__('turtle_spawner')
        self.cli = self.create_client(Spawn, '/turtlesim1/spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Spawn.Request()

    def spawn_turtle(self, x: float, y: float, theta: float, name: str):
        """floats are float32"""
        # Message parameters for Spawn
        # http://docs.ros.org/en/noetic/api/turtlesim/html/srv/Spawn.html
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.req.name = name

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    turtleSpawner = TurtleSpawner()
    response = turtleSpawner.spawn_turtle(5.0, 5.0, 0.0, "aha")
    # minimal_client.get_logger().info(
    #     'Result of spawn: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    turtleSpawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()