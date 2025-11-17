#!/usr/bin/env python3

import math
import sys
import argparse
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2



class SimpleExplorationNode(Node):
    """Simple autonomous exploration with reactive obstacle avoidance."""

    # State definitions
    STATE_EXPLORING = 0
    STATE_STOPPED = 1
    STATE_ANALYZING = 2
    STATE_TURNING = 3
    STATE_REVERSING = 4
    STATE_RECOVERY = 5  # New recovery state
    
    def __init__(self, obstacle_threshold, min_safe_distance,
                 forward_speed, turn_speed, reverse_speed):
        super().__init__('simple_exploration_node')

         # Parameters parsed from command line or defaults
        self.OBSTACLE_THRESHOLD = obstacle_threshold
        self.MIN_SAFE_DISTANCE = min_safe_distance
        self.FORWARD_SPEED = forward_speed
        self.TURN_SPEED = turn_speed
        self.REVERSE_SPEED = reverse_speed
        
        # Robot state
        self.state = self.STATE_EXPLORING
        self.state_start_time = self.get_clock().now()
        self.target_angle = 0.0
        self.current_yaw = 0.0
        
        # Recovery tracking
        self.recovery_target = 0.0
        self.recovery_attempts = 0
        
        # Sensor data
        self.scan: Optional[LaserScan] = None
        self.camera_image: Optional[Image] = None
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribers and Publishers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        # Subscribe to camera (adjust topic name based on your robot)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Create CV2 window
        cv2.namedWindow('Robot Camera Feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Robot Camera Feed', 640, 480)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Simple Exploration Node Started')
        self.get_logger().info(f'Obstacle threshold: {self.OBSTACLE_THRESHOLD}m')
        self.get_logger().info('Camera feed window opened')
        self.get_logger().info('='*50)
    
    def scan_callback(self, msg: LaserScan):
        """Store latest LiDAR scan."""
        self.scan = msg
    
    def camera_callback(self, msg: Image):
        """Process and display camera feed."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Add overlay information
            self.add_overlay(cv_image)
            
            # Display in window
            cv2.imshow('Robot Camera Feed', cv_image)
            cv2.waitKey(1)  # Required for cv2.imshow to work
            
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')
    
    def add_overlay(self, image):
        """Add state and sensor information overlay to camera image."""
        state_names = {
            0: 'EXPLORING',
            1: 'STOPPED',
            2: 'ANALYZING',
            3: 'TURNING',
            4: 'REVERSING',
            5: 'RECOVERY'
        }
        
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Add semi-transparent black bar at top for text
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width, 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        
        # Current state
        state_text = f"State: {state_names.get(self.state, 'UNKNOWN')}"
        cv2.putText(image, state_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Front obstacle distance (if available)
        if self.scan:
            sectors = self.get_sectors()
            if sectors:
                front_dist = sectors.get('front', 0.0)
                dist_text = f"Front: {front_dist:.2f}m"
                
                # Color based on distance
                if front_dist < self.MIN_SAFE_DISTANCE:
                    color = (0, 0, 255)  # Red
                elif front_dist < self.OBSTACLE_THRESHOLD:
                    color = (0, 165, 255)  # Orange
                else:
                    color = (0, 255, 0)  # Green
                
                cv2.putText(image, dist_text, (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Add timestamp
        timestamp = self.get_clock().now().nanoseconds / 1e9
        time_text = f"Time: {timestamp:.1f}s"
        cv2.putText(image, time_text, (width - 200, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def get_sectors(self) -> Dict[str, float]:
        """
        Divide 360° LiDAR into 8 sectors and get minimum distance in each.
        
        Returns:
            Dictionary: {sector_name: min_distance}
        """
        if not self.scan:
            return {}
        
        ranges = self.scan.ranges
        n = len(ranges)
        
        # Define 8 sectors (45° each)
        sectors = {
            'front':       ranges[0:23] + ranges[338:n],
            'front_right': ranges[293:338],
            'right':       ranges[248:293],
            'back_right':  ranges[203:248],
            'back':        ranges[158:203],
            'back_left':   ranges[113:158],
            'left':        ranges[68:113],
            'front_left':  ranges[23:68]
        }
        
        # Get minimum distance in each sector
        result = {}
        for name, sector in sectors.items():
            valid = [r for r in sector if not math.isinf(r) and not math.isnan(r)]
            result[name] = min(valid) if valid else float('inf')
        
        return result
    
    def find_clearest_direction(self, sectors: Dict[str, float]) -> str:
        """
        Find the clearest direction from 360° scan.
        Prioritizes forward directions over backward.
        
        Returns:
            Name of clearest sector
        """
        # Prioritize forward-facing sectors
        forward_sectors = {
            k: v for k, v in sectors.items() 
            if k in ['front', 'front_left', 'front_right', 'left', 'right']
        }
        
        # Find clearest forward direction
        clearest = max(forward_sectors, key=forward_sectors.get)
        max_distance = forward_sectors[clearest]
        
        # If forward directions blocked, consider all directions
        if max_distance < self.MIN_SAFE_DISTANCE:
            clearest = max(sectors, key=sectors.get)
        
        return clearest
    
    def sector_to_angle(self, sector: str) -> float:
        """Convert sector name to angle offset in radians."""
        angles = {
            'front': 0.0,
            'front_left': math.pi / 4,
            'left': math.pi / 2,
            'back_left': 3 * math.pi / 4,
            'back': math.pi,
            'back_right': -3 * math.pi / 4,
            'right': -math.pi / 2,
            'front_right': -math.pi / 4
        }
        return angles.get(sector, 0.0)
    
    def control_loop(self):
        """Main control loop - state machine."""
        if not self.scan:
            return
        
        sectors = self.get_sectors()
        if not sectors:
            return
        
        # State machine
        if self.state == self.STATE_EXPLORING:
            self.handle_exploring(sectors)
        elif self.state == self.STATE_STOPPED:
            self.handle_stopped(sectors)
        elif self.state == self.STATE_ANALYZING:
            self.handle_analyzing(sectors)
        elif self.state == self.STATE_TURNING:
            self.handle_turning(sectors)
        elif self.state == self.STATE_REVERSING:
            self.handle_reversing(sectors)
        elif self.state == self.STATE_RECOVERY:
            self.handle_recovery(sectors)
    
    def handle_exploring(self, sectors: Dict[str, float]):
        """Move forward until obstacle detected."""
        # Check front obstacle
        if sectors['front'] < self.OBSTACLE_THRESHOLD:
            self.get_logger().info(
                f'Obstacle at {sectors["front"]:.2f}m - Stopping'
            )
            self.change_state(self.STATE_STOPPED)
            return
        
        # Move forward
        self.publish_velocity(self.FORWARD_SPEED, 0.0)
    
    def handle_stopped(self, sectors: Dict[str, float]):
        """Stop and prepare to analyze."""
        self.publish_velocity(0.0, 0.0)
        
        # Wait briefly (0.3 seconds)
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        if elapsed > 0.3:
            self.change_state(self.STATE_ANALYZING)
    
    def handle_analyzing(self, sectors: Dict[str, float]):
        """Analyze 360° and choose best direction."""
        self.publish_velocity(0.0, 0.0)
        
        # Find clearest direction
        clearest = self.find_clearest_direction(sectors)
        max_clearance = sectors[clearest]
        
        self.get_logger().info(
            f'Analysis: Clearest={clearest} ({max_clearance:.2f}m)'
        )
        
        # If all directions blocked, try reverse first
        if max_clearance < self.MIN_SAFE_DISTANCE:
            # Check if can reverse
            if sectors['back'] > 0.5:
                self.get_logger().warn('All blocked - Reversing')
                self.change_state(self.STATE_REVERSING)
            else:
                # Can't reverse, use progressive rotation recovery
                self.get_logger().warn('All blocked, cannot reverse - Starting recovery rotation')
                self.recovery_target = self.normalize_angle(self.current_yaw + math.pi/2)
                self.recovery_attempts = 0
                self.change_state(self.STATE_RECOVERY)
            return
        
        # Calculate target angle
        angle_offset = self.sector_to_angle(clearest)
        self.target_angle = self.normalize_angle(self.current_yaw + angle_offset)
        
        self.get_logger().info(f'Turning toward {clearest}')
        self.change_state(self.STATE_TURNING)
    
    def handle_turning(self, sectors: Dict[str, float]):
        """Turn toward target direction."""
        # Calculate angle difference
        angle_diff = self.normalize_angle(self.target_angle - self.current_yaw)
        
        # Check if turn complete
        if abs(angle_diff) < 0.15:  # ~8.6 degrees tolerance
            self.publish_velocity(0.0, 0.0)
            
            # Check if front now clear
            if sectors['front'] > self.OBSTACLE_THRESHOLD:
                self.get_logger().info('Turn complete - Resuming exploration')
                self.change_state(self.STATE_EXPLORING)
            else:
                self.get_logger().info('Still blocked - Re-analyzing')
                self.change_state(self.STATE_ANALYZING)
            return
        
        # Continue turning
        turn_direction = 1 if angle_diff > 0 else -1
        self.publish_velocity(0.0, self.TURN_SPEED * turn_direction)
        
        # Update current yaw estimate (simplified - not using odometry)
        dt = 0.1  # Timer period
        self.current_yaw += self.TURN_SPEED * turn_direction * dt
        self.current_yaw = self.normalize_angle(self.current_yaw)
    
    def handle_reversing(self, sectors: Dict[str, float]):
        """Reverse to escape tight situation."""
        # Double-check back clearance
        if sectors['back'] < 0.5:
            self.get_logger().warn('Back blocked during reverse - Starting recovery rotation')
            self.recovery_target = self.normalize_angle(self.current_yaw + math.pi/2)
            self.recovery_attempts = 0
            self.change_state(self.STATE_RECOVERY)
            return
        
        # Reverse for 2 seconds
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        if elapsed > 2.0:
            self.get_logger().info('Reverse complete')
            self.change_state(self.STATE_ANALYZING)
            return
        
        # Continue reversing
        self.publish_velocity(-self.REVERSE_SPEED, 0.0)
    
    def handle_recovery(self, sectors: Dict[str, float]):
        # Check if found clear path
        clearest = self.find_clearest_direction(sectors)
        
        if sectors[clearest] > self.MIN_SAFE_DISTANCE:
            # Found clear path!
            self.get_logger().info(f'✓ Recovery successful - found {clearest}')
            angle_offset = self.sector_to_angle(clearest)
            self.target_angle = self.normalize_angle(self.current_yaw + angle_offset)
            self.change_state(self.STATE_TURNING)
            return
        
        # Turn toward next 90° target
        angle_diff = self.normalize_angle(self.recovery_target - self.current_yaw)
        
        if abs(angle_diff) < 0.15:
            # Reached this 90° target
            self.recovery_attempts += 1
            
            if self.recovery_attempts >= 4:
                # Completed full 360° rotation
                self.get_logger().error('✗ Full rotation - no clear path found')
                self.get_logger().info('Attempting to move forward anyway')
                self.change_state(self.STATE_EXPLORING)
                return
            
            # Set next 90° target
            self.recovery_target = self.normalize_angle(self.recovery_target + math.pi/2)
            self.get_logger().info(f'Recovery {self.recovery_attempts}/4 - rotating 90° more')
        
        # Continue rotating
        turn_direction = 1 if angle_diff > 0 else -1
        self.publish_velocity(0.0, self.TURN_SPEED * turn_direction * 0.6)
        
        # Update yaw estimate
        dt = 0.1
        self.current_yaw += self.TURN_SPEED * turn_direction * 0.6 * dt
        self.current_yaw = self.normalize_angle(self.current_yaw)
    
    def change_state(self, new_state: int):
        """Transition to new state."""
        state_names = {
            0: 'EXPLORING',
            1: 'STOPPED',
            2: 'ANALYZING',
            3: 'TURNING',
            4: 'REVERSING',
            5: 'RECOVERY'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info(f'State: {old_name} → {new_name}')
        
        self.state = new_state
        self.state_start_time = self.get_clock().now()
    
    def publish_velocity(self, linear: float, angular: float):
        """Publish velocity command."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def shutdown(self):
        """Clean shutdown - close camera window."""
        self.get_logger().info('Shutting down - closing camera window')
        cv2.destroyAllWindows()


def main(args=None):
    
    parser = argparse.ArgumentParser(description="Simple Reactive Exploration Node")

    parser.add_argument(
        '--obstacle_threshold',
        type=float, 
        default=0.8,
        help='Distance to stop for obstacles (default: 0.8m)'
    )

    parser.add_argument(
        '--min_safe_distance', 
        type=float, 
        default=0.6,
        help='Minimum clearance considered safe (default: 0.6m)'
    )
    parser.add_argument(
        '--forward_speed', 
        type=float, 
        default=0.5,
        help='Linear velocity when moving forward (default: 0.5m/s)'
    )

    parser.add_argument(
        '--turn_speed', 
        type=float, 
        default=0.5,
        help='Angular velocity when turning (default: 0.5rad/s)'
    )

    parser.add_argument(
        '--reverse_speed', 
        type=float, 
        default=0.15,
        help='Linear velocity when reversing (default: 0.15m/s)'
    )

    if args is None:
        args = sys.argv[1:]

    # Parse only known args; ignore the rest
    parsed_args, unknown = parser.parse_known_args(args)

    rclpy.init(args=args)
    node = None

    try:
        node = SimpleExplorationNode(
            obstacle_threshold=parsed_args.obstacle_threshold,
            min_safe_distance=parsed_args.min_safe_distance,
            forward_speed=parsed_args.forward_speed,
            turn_speed=parsed_args.turn_speed,
            reverse_speed=parsed_args.reverse_speed
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if node:
            try:
                node.publish_velocity(0.0, 0.0)
                node.shutdown()
                node.destroy_node()
            except:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
