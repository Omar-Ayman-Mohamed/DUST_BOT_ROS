import rclpy
from rclpy.node import Node
import random

from geometry_msgs.msg import Point
from dustbot_interface.msg import GarbagePosition
from dustbot_interface.srv import SetDirection, LoadGarbage

class WorldNode(Node):
    def __init__(self):
        super().__init__('world_node')

        # 2. Declare Parameters
        self.declare_parameter('n_grid', 10)
        self.declare_parameter('p_pickups', 5)
        
        self.n = self.get_parameter('n_grid').value
        self.total_pickups = self.get_parameter('p_pickups').value
        self.pickups_done = 0
        self.game_over = False
        self.last_pickup_location = None  # Track last successful pickup location
        self.need_new_garbage = False 

        # 3. Set Initial State
        self.robot_x = 0
        self.robot_y = 0
        self.current_direction = None
        
        # Randomly place the first garbage
        self.garbage_x = random.randint(0, self.n - 1)
        self.garbage_y = random.randint(0, self.n - 1)

        # 4. Create Publishers
        self.pos_pub = self.create_publisher(Point, '/dustbot/global_position', 10)
        self.garb_pub = self.create_publisher(GarbagePosition, '/dustbot/garbage_position', 10)

        # 5. Create Services
        self.create_service(SetDirection, '/dustbot/set_direction', self.handle_set_direction)
        self.create_service(LoadGarbage, '/dustbot/load_garbage', self.handle_load_garbage)

        # 6. Create the Game Loop Timer (1 second = 1 cell/s)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(f"Dustbot World Started! Grid: {self.n}x{self.n}, Pickups: {self.total_pickups}")
        
        # Publish initial garbage position
        initial_garb = GarbagePosition()
        initial_garb.x = int(self.garbage_x)
        initial_garb.y = int(self.garbage_y)
        self.garb_pub.publish(initial_garb)

    def timer_callback(self):
        """This runs every 1 second to simulate physics/movement."""
        
        if self.game_over:
            return
        
        if self.need_new_garbage:
            self.garbage_x = random.randint(0, self.n - 1)
            self.garbage_y = random.randint(0, self.n - 1)
            self.need_new_garbage = False
            self.get_logger().info(f"New garbage spawned at ({self.garbage_x}, {self.garbage_y})")
            
        msg = GarbagePosition()
        msg.x = int(self.garbage_x)
        msg.y = int(self.garbage_y)
        self.garb_pub.publish(msg)
            

        
        # 1. Update Robot Position based on current direction
        if self.current_direction == 'N':
            self.robot_y += 1
        elif self.current_direction == 'S':
            self.robot_y -= 1
        elif self.current_direction == 'E':
            self.robot_x += 1
        elif self.current_direction == 'W':
            self.robot_x -= 1
            
        # 2. Wall Collision Logic
        self.robot_x = max(0, min(self.robot_x, self.n - 1))
        self.robot_y = max(0, min(self.robot_y, self.n - 1))

        # 3. Publish the Robot's Position
        pos_msg = Point()
        pos_msg.x = float(self.robot_x)
        pos_msg.y = float(self.robot_y)
        pos_msg.z = 0.0
        self.pos_pub.publish(pos_msg)

        # Log for debugging
        self.get_logger().info(f"Robot: ({self.robot_x}, {self.robot_y}) | Target: ({self.garbage_x}, {self.garbage_y})")

    def handle_set_direction(self, request, response):
        """Called when the robot wants to change direction."""
        if self.game_over:
            response.success = False
            return response
            
        req_dir = request.direction
        

        if self.current_direction == req_dir:
            response.success = True
            return response
        
        if req_dir in ['N', 'S', 'E', 'W']:
            self.current_direction = req_dir
            self.get_logger().info(f"Direction set to: {req_dir}")
            response.success = True
        else:
            self.get_logger().warn(f"Invalid direction request: {req_dir}")
            response.success = False
            
        return response

    def handle_load_garbage(self, request, response):
        """Called when the robot wants to pick up garbage."""
        # If game over, accept silently
        if self.game_over:
            response.success = True
            return response

 
        current_robot_location = (self.robot_x, self.robot_y)
        #guard to prevent double counting if robot calls pickup multiple times at same location
        if self.last_pickup_location == current_robot_location:
            response.success = True
            return response

        if self.robot_x == self.garbage_x and self.robot_y == self.garbage_y:
            # Record this location as processed
            self.last_pickup_location = current_robot_location
            self.pickups_done += 1
            
            # CASE A: Game Over - Final Pickup
            if self.pickups_done >= self.total_pickups:
                self.game_over = True
                self.current_direction = None  
                
                self.get_logger().info(f"SUCCESS! Trash collected. ({self.pickups_done}/{self.total_pickups})")
                self.get_logger().info("MISSION COMPLETE. Sending Kill Signal to Robot...")
                
                # Send kill signal
                kill_msg = GarbagePosition()
                kill_msg.x = -100
                kill_msg.y = -100
                self.garb_pub.publish(kill_msg)
                
                response.success = True
                
                self.create_timer(1.0, self.shutdown_system)
                return response

            # CASE B: Normal Pickup
            self.get_logger().info(f"SUCCESS! Trash collected. ({self.pickups_done}/{self.total_pickups})")
            
            self.current_direction = None 
            self.need_new_garbage = True
            
            response.success = True
            return response

        else:
            self.get_logger().warn("FAILED PICKUP: Robot is not at garbage location.")
            response.success = False
            return response

    def shutdown_system(self):
        """Cleanly shut down the world node."""
        self.get_logger().info("Shutting down World Node.")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()