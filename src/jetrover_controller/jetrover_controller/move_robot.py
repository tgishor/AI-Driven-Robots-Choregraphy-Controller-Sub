import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.movement_timer = None
        self.stop_requested = False
        self.current_twist = Twist()
        
        # Movement state
        self.movement_phase = 'idle'  # 'idle', 'moving', 'straightening', 'final_stop'
        self.phase_start_time = 0
        self.original_linear_speed = 0
        self.original_angular_speed = 0
        self.total_duration = 0
        
    def move_in_circle(self, linear_speed, angular_speed, duration):
        """Start moving in a circle using a timer instead of blocking loop"""
        # Cancel any existing movement
        self.stop_movement()
        
        # Reset stop flag and set up movement
        self.stop_requested = False
        self.movement_phase = 'moving'
        self.phase_start_time = time.time()
        self.original_linear_speed = linear_speed
        self.original_angular_speed = angular_speed
        self.total_duration = duration
        
        # Set the movement parameters
        self.current_twist.linear.x = linear_speed
        self.current_twist.angular.z = angular_speed
        
        self.get_logger().info(f"Moving in a circle: linear={linear_speed} m/s, angular={angular_speed} rad/s for {duration} sec")
        
        # Start movement timer
        self.movement_timer = self.create_timer(0.1, self.movement_callback)  # 10 Hz
        
    def movement_callback(self):
        """Single callback that handles all movement phases"""
        if self.stop_requested:
            # Stop requested - publish stop command and stop timer
            stop_twist = Twist()
            self.publisher_.publish(stop_twist)
            self.stop_movement()
            return
        
        current_time = time.time()
        phase_elapsed = current_time - self.phase_start_time
        
        if self.movement_phase == 'moving':
            # Check if main movement duration has elapsed
            if phase_elapsed >= self.total_duration:
                self.get_logger().info("I'm done with angular move")
                
                # Transition to straightening phase if we were turning
                if abs(self.original_angular_speed) > 0.01:
                    self.movement_phase = 'straightening'
                    self.phase_start_time = current_time
                    self.current_twist.linear.x = abs(self.original_linear_speed) * 0.7
                    self.current_twist.angular.z = 0.0
                    self.get_logger().info("Straightening the heading...")
                else:
                    # No straightening needed, go to final stop
                    self.movement_phase = 'final_stop'
                    self.phase_start_time = current_time
                    self.current_twist = Twist()  # All zeros
                    self.get_logger().info("Final stop phase")
            
            # Continue with current movement
            self.publisher_.publish(self.current_twist)
            self.get_logger().debug(f"Moving: linear={self.current_twist.linear.x:.2f}, angular={self.current_twist.angular.z:.2f}")
            
        elif self.movement_phase == 'straightening':
            # Check if straightening duration has elapsed (3 seconds)
            if phase_elapsed >= 3.0:
                self.get_logger().info("Done straightening")
                # Transition to final stop
                self.movement_phase = 'final_stop'
                self.phase_start_time = current_time
                self.current_twist = Twist()  # All zeros
                self.get_logger().info("Final stop phase")
            
            # Continue straightening
            self.publisher_.publish(self.current_twist)
            self.get_logger().debug("Straightening movement")
            
        elif self.movement_phase == 'final_stop':
            # Check if final stop duration has elapsed (1 second)
            if phase_elapsed >= 1.0:
                self.get_logger().info("Movement sequence completed")
                self.stop_movement()
                return
            
            # Continue final stop
            self.publisher_.publish(self.current_twist)  # Should be all zeros
        
    def stop_movement(self):
        """Stop all movement timers"""
        if self.movement_timer:
            self.destroy_timer(self.movement_timer)
            self.movement_timer = None
            
        # Reset movement state
        self.movement_phase = 'idle'
        self.current_twist = Twist()
    
    def stop(self):
        """Emergency stop - immediately stop all movement"""
        self.stop_requested = True
        self.stop_movement()
        
        twist = Twist()  # All zeros = stop everything
        self.get_logger().info("Sending stop commands to straighten wheels...")
        for _ in range(10):  # Publish 10 times over 1 second
            self.publisher_.publish(twist)
            time.sleep(0.1)
        self.get_logger().info("Robot fully stopped and wheels straightened.")


def main(args=None):
    rclpy.init(args=args)
    mover = RobotMover()
    mover.move_in_circle(0.1, 0.0, 3) # Straighten the robot legs
    # mover.move_in_circle(0.1, 0.5, 3)  # gentle forward-left curve for 4 sec
    # mover.move_in_circle(-0.1, 0.0, 1) 
    # mover.move_in_circle(0.1, -0.3, 3) 
    # mover.move_in_circle(-0.1, 0.0, 1) 
    mover.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

