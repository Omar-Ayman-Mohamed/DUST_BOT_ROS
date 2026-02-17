import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dustbot_interface.msg import GarbagePosition
from dustbot_interface.srv import SetDirection, LoadGarbage
from enum import Enum

class State(Enum):
    IDLE = 0
    MOVING = 1
    PICKING_UP = 2
    COMPLETED = 3

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Internal Memory
        self.my_x = None
        self.my_y = None
        self.target_x = None
        self.target_y = None
        self.last_sent_direction = None
        self.state = State.IDLE

        # Subscribers
        self.create_subscription(Point, '/dustbot/global_position', self.pose_callback, 10)
        self.create_subscription(GarbagePosition, '/dustbot/garbage_position', self.garbage_callback, 10)

        # Service Clients
        self.dir_client = self.create_client(SetDirection, '/dustbot/set_direction')
        self.load_client = self.create_client(LoadGarbage, '/dustbot/load_garbage')


        self.create_timer(0.1, self.control_logic)
        self.get_logger().info(" Dustbot Robot Online")

    def pose_callback(self, msg):
        """Called every time the world tells us our position."""
        if self.state == State.COMPLETED:
            return
            
        self.my_x = msg.x
        self.my_y = msg.y
        #self.control_logic()

    def garbage_callback(self, msg):
        # Check for KILL SIGNAL
        if msg.x == -100 and msg.y == -100:
            self.state = State.COMPLETED
            self.create_timer(0.5, lambda: raise_(SystemExit))
            return

        # Update Target
        self.target_x = msg.x
        self.target_y = msg.y
        
        if self.state == State.IDLE:
            self.get_logger().info(f" New Target Received: ({self.target_x}, {self.target_y})")
            self.state = State.MOVING

    def control_logic(self):
        if not self.dir_client.service_is_ready():
            self.get_logger().warn("Waiting for World Node to start...", throttle_duration_sec=2.0)
            return  # Skip this loop iteration, try again in 0.1s
            
        if not self.load_client.service_is_ready():
            return


        if self.state == State.COMPLETED:
            return

        # Guard: No data yet
        if self.my_x is None or self.target_x is None:
            return
        
        # Check if at target
        if self.state == State.MOVING:
            self.handle_movment_state() 
        elif self.state == State.PICKING_UP:
            # This prevents sending multiple requests.
            pass 

        elif self.state == State.IDLE:
            # Waiting for garbage_callback to give us a target
            pass
    def handle_movment_state(self):
        if self.my_x == self.target_x and self.my_y == self.target_y:
                self.get_logger().info("Arrived at target. Transitioning to PICKING_UP.")
                self.state = State.PICKING_UP
                self.call_load_garbage()
                return
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
        if direction in {'N', 'E', 'S', 'W'} and direction != self.last_sent_direction:
            self.call_set_direction(direction)

    def call_set_direction(self, direction):
        """Async call to set direction."""
        req = SetDirection.Request()
        req.direction = direction
        
        self.last_sent_direction = direction
        self.direction_future = self.dir_client.call_async(req)
        

        

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
        
        self.pickup_future = self.load_client.call_async(req)
        self.pickup_future.add_done_callback(self.pickup_response_callback)

    def pickup_response_callback(self, future):        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("PICKUP SUCCESSFUL! Waiting for new target...")
                self.target_x = None 
                self.target_y = None
                self.state = State.IDLE
            else:
                self.get_logger().warn("Pickup failed. Will retry...")
                self.state = State.MOVING # Unlock so we can try again
        except Exception as e:
            self.get_logger().error(f"Pickup service call failed: {e}")
            self.state = State.MOVING

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