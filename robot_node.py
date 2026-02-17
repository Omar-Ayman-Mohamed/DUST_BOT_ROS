import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dustbot_interface.msg import GarbagePosition
from dustbot_interface.srv import SetDirection, LoadGarbage

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Internal Memory
        self.my_x = None
        self.my_y = None
        self.target_x = None
        self.target_y = None
        self.last_sent_direction = None
        self.pending_pickup = False
        self.mission_complete = False
        self.pickup_in_flight = False  

        # Subscribers
        self.create_subscription(Point, '/dustbot/global_position', self.pose_callback, 10)
        self.create_subscription(GarbagePosition, '/dustbot/garbage_position', self.garbage_callback, 10)

        # Service Clients
        self.dir_client = self.create_client(SetDirection, '/dustbot/set_direction')
        self.load_client = self.create_client(LoadGarbage, '/dustbot/load_garbage')

        # Wait for services
        while not self.dir_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for World Node...')

        self.get_logger().info(" Dustbot Robot Online")

    def pose_callback(self, msg):
        """Called every time the world tells us our position."""
        if self.mission_complete:
            return
            
        self.my_x = msg.x
        self.my_y = msg.y
        self.control_logic()

    def garbage_callback(self, msg):
        # Check for KILL SIGNAL
        if msg.x == -100 and msg.y == -100:
            self.mission_complete = True
            self.pending_pickup = False
            self.create_timer(0.5, lambda: raise_(SystemExit))
            return

        # Update Target
        self.target_x = msg.x
        self.target_y = msg.y
        
        if self.pending_pickup:
            self.get_logger().info(f" New Target Received: ({self.target_x}, {self.target_y})")

    def control_logic(self):
        # Guard: Mission complete
        if self.mission_complete:
            return

        # Guard: No data yet
        if self.my_x is None or self.target_x is None:
            return
        
        # Check if at target
        if self.my_x == self.target_x and self.my_y == self.target_y:
            if not self.pending_pickup and not self.pickup_in_flight:#can we remove this line ?
                self.pending_pickup = True
                self.pickup_in_flight = True
                self.call_load_garbage()
            return
        
        if self.pending_pickup:
            self.get_logger().info(f"Starting movement toward new target ({int(self.target_x)}, {int(self.target_y)})")
            self.pending_pickup = False
        
        # Movement Logic
        direction = None
        if self.my_x < self.target_x:
            direction = 'E'
        elif self.my_x > self.target_x:
            direction = 'W'
        elif self.my_y < self.target_y:
            direction = 'N'
        elif self.my_y > self.target_y:
            direction = 'S'
        
        # Only send direction if it changed
        if direction and direction != self.last_sent_direction:
            self.call_set_direction(direction)

    def call_set_direction(self, direction):
        """Async call to set direction."""
        req = SetDirection.Request()
        req.direction = direction
        
        self.last_sent_direction = direction
        
        future = self.dir_client.call_async(req)
        future.add_done_callback(self.direction_response_callback)
        

    def direction_response_callback(self, future):
        """Handles the answer from the World."""
        try:
            response = future.result()
            if response.success:
                pass  
            else:
                self.get_logger().warn(" Move rejected by World!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def call_load_garbage(self):
        """Async call to pick up trash."""
        self.last_sent_direction = None 
        
        req = LoadGarbage.Request()
        self.get_logger().info("Calling pickup service...")
        
        future = self.load_client.call_async(req)
        future.add_done_callback(self.pickup_response_callback)

    def pickup_response_callback(self, future):
        """Called when pickup service responds."""
        self.pickup_in_flight = False  # Clear the in-flight flag
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("PICKUP SUCCESSFUL! Waiting for new target...")
                self.target_x = None 
                self.target_y = None
            else:
                self.get_logger().warn("Pickup failed. Will retry...")
                self.pending_pickup = False  # Unlock so we can try again
        except Exception as e:
            self.get_logger().error(f"Pickup service call failed: {e}")
            self.pending_pickup = False

def raise_(ex):
    raise ex

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()