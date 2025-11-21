import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped
import tf2_ros
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import math
import numpy as np

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) 

        # --- Parameters ---
        self.declare_parameter('max_bias_samples', 25) 
        self.declare_parameter('lpf_alpha', 0.1) 
        self.declare_parameter('filter_gain', 0.98) 

        # --- Bias State Variables ---
        self.bias_samples = 0
        self.max_bias_samples = self.get_parameter('max_bias_samples').get_parameter_value().integer_value
        self.gyro_bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # --- Filter State Variables ---
        self.lpf_accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.alpha = self.get_parameter('filter_gain').get_parameter_value().double_value
        self.last_time = None
        self.q_estimated = [0.0, 0.0, 0.0, 1.0] 

        # --- Dead Reckoning State Variables ---
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.GRAVITY = 9.81 

        # --- ROS 2 Communication ---
        self.imu_pub = self.create_publisher(Imu, '/mobile_imu/filtered', 10)
        self.path_pub = self.create_publisher(Path, '/mobile_imu/path', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom" 
        
        self.subscription = self.create_subscription(
            Imu,
            '/mobile_imu/raw',
            self.imu_callback,
            10
        )
        self.get_logger().info('IMU Processor Node initialized. Collecting Gyro Bias...')


    def imu_callback(self, msg):
        
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        # --- 1. Bias Collection Phase ---
        if self.bias_samples < self.max_bias_samples:
            self.gyro_bias['x'] += msg.angular_velocity.x
            self.gyro_bias['y'] += msg.angular_velocity.y
            self.gyro_bias['z'] += msg.angular_velocity.z
            self.bias_samples += 1
            if self.bias_samples == self.max_bias_samples:
                self.gyro_bias['x'] /= self.max_bias_samples
                self.gyro_bias['y'] /= self.max_bias_samples
                self.gyro_bias['z'] /= self.max_bias_samples
                self.get_logger().info(f"Gyro Bias Calibrated: X={self.gyro_bias['x']:.4f}, Y={self.gyro_bias['y']:.4f}, Z={self.gyro_bias['z']:.4f}")
            return 

        # --- 2. Calculate Delta Time (dt) ---
        if self.last_time is None:
            self.last_time = current_time_sec
            return 

        dt = current_time_sec - self.last_time

        if dt <= 0:
            self.get_logger().warn("Invalid or zero time step (dt). Skipping integration.")
            return

        # --- 3. Bias Correction and LPF ---
        angular_vel_x = msg.angular_velocity.x - self.gyro_bias['x']
        angular_vel_y = msg.angular_velocity.y - self.gyro_bias['y']
        angular_vel_z = msg.angular_velocity.z - self.gyro_bias['z']

        lpf_alpha = self.get_parameter('lpf_alpha').get_parameter_value().double_value
        ax_lpf = lpf_alpha * msg.linear_acceleration.x + (1.0 - lpf_alpha) * self.lpf_accel['x']
        ay_lpf = lpf_alpha * msg.linear_acceleration.y + (1.0 - lpf_alpha) * self.lpf_accel['y']
        az_lpf = lpf_alpha * msg.linear_acceleration.z + (1.0 - lpf_alpha) * self.lpf_accel['z']
        self.lpf_accel['x'] = ax_lpf
        self.lpf_accel['y'] = ay_lpf
        self.lpf_accel['z'] = az_lpf

        # --- 4. Complementary Filter (Orientation) ---
        (roll_est, pitch_est, yaw_est) = euler_from_quaternion(self.q_estimated)
        
        roll_gyro = roll_est + angular_vel_x * dt
        pitch_gyro = pitch_est + angular_vel_y * dt
        yaw_gyro = yaw_est + angular_vel_z * dt 

        roll_accel = math.atan2(ay_lpf, az_lpf)
        pitch_accel = math.atan2(-ax_lpf, math.sqrt(ay_lpf*ay_lpf + az_lpf*az_lpf))
        
        roll_fused = self.alpha * roll_gyro + (1.0 - self.alpha) * roll_accel
        pitch_fused = self.alpha * pitch_gyro + (1.0 - self.alpha) * pitch_accel
        yaw_fused = yaw_gyro

        self.q_estimated = quaternion_from_euler(roll_fused, pitch_fused, yaw_fused)

        # Check if the phone is nearly flat (e.g., within 10 degrees tilt, ~0.2 radians)
        is_flat = abs(roll_fused) < 0.2 and abs(pitch_fused) < 0.2


        # --- 5. Dead Reckoning (Position) ---
        
        # 5a. Rotate Acceleration to World Frame
        R = quaternion_matrix(self.q_estimated)[:3, :3] 
        accel_phone = np.array([[ax_lpf], [ay_lpf], [az_lpf]])
        accel_world_raw = R @ accel_phone
        
        # 5b. Gravity Compensation and Noise Filtering
        
        # Slightly adjusted threshold to reduce drift while allowing small movements
        threshold = 0.08 
        
        accel_world = {
            'x': accel_world_raw[0, 0],
            'y': accel_world_raw[1, 0],
            'z': accel_world_raw[2, 0] - self.GRAVITY # Remove gravity from Z
        }

        # Apply XY Threshold for noise
        if abs(accel_world['x']) < threshold: accel_world['x'] = 0.0
        if abs(accel_world['y']) < threshold: accel_world['y'] = 0.0
        
        # DYNAMIC Z-AXIS ACCELERATION HANDLING
        if is_flat:
            # FIX A: When flat on the table, force Z acceleration to zero
            accel_world['z'] = 0.0
        else:
            # When tilted, apply a slightly higher threshold to filter vertical noise
            if abs(accel_world['z']) < 0.1:
                accel_world['z'] = 0.0 
        
        # 5c. First Integration (Velocity)
        self.velocity['x'] += accel_world['x'] * dt
        self.velocity['y'] += accel_world['y'] * dt
        self.velocity['z'] += accel_world['z'] * dt

        # Adjust velocity threshold for ZUPT
        velocity_threshold = 0.02 
        
        # Apply XY ZUPT
        if abs(self.velocity['x']) < velocity_threshold: self.velocity['x'] = 0.0
        if abs(self.velocity['y']) < velocity_threshold: self.velocity['y'] = 0.0

        # DYNAMIC ZUPT APPLICATION
        if is_flat:
            # FIX B: When flat, force Z velocity to zero to kill vertical drift
            self.velocity['z'] = 0.0
        else:
            # When tilted, apply a standard ZUPT if vertical motion is slow
            if abs(self.velocity['z']) < velocity_threshold: 
                self.velocity['z'] = 0.0

        # 5d. Second Integration (Position)
        self.position['x'] += self.velocity['x'] * dt
        self.position['y'] += self.velocity['y'] * dt
        self.position['z'] += self.velocity['z'] * dt

        # DYNAMIC Z-POSITION CONSTRAINT
        if is_flat:
            # FIX C: When flat, ensure Z position is exactly zero (constrain to table)
            self.position['z'] = 0.0
            
        self.last_time = current_time_sec

        # --- 6. Publish Messages ---
        
        # Publish Filtered IMU message
        filtered_msg = Imu()
        filtered_msg.header = msg.header 
        filtered_msg.orientation = Quaternion(
            x=self.q_estimated[0], y=self.q_estimated[1], 
            z=self.q_estimated[2], w=self.q_estimated[3]
        )
        filtered_msg.linear_acceleration.x = ax_lpf 
        filtered_msg.linear_acceleration.y = ay_lpf
        filtered_msg.linear_acceleration.z = az_lpf
        filtered_msg.angular_velocity.x = angular_vel_x
        filtered_msg.angular_velocity.y = angular_vel_y
        filtered_msg.angular_velocity.z = angular_vel_z
        self.imu_pub.publish(filtered_msg)
        
        # Publish Path message
        pose = PoseStamped()
        pose.header.stamp = filtered_msg.header.stamp
        pose.header.frame_id = "odom"
        
        pose.pose.position.x = self.position['x']
        pose.pose.position.y = self.position['y']
        pose.pose.position.z = self.position['z']
        pose.pose.orientation = filtered_msg.orientation 

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        
        # --- 7. Broadcast TF (odom -> phone_imu_link) ---
        t = TransformStamped()
        
        t.header.stamp = filtered_msg.header.stamp 
        t.header.frame_id = 'odom' 
        t.child_frame_id = 'phone_imu_link' 
        
        # Set the translation (Position)
        t.transform.translation.x = self.position['x']
        t.transform.translation.y = self.position['y']
        t.transform.translation.z = self.position['z']
        
        # Set the rotation (Orientation)
        t.transform.rotation.x = self.q_estimated[0]
        t.transform.rotation.y = self.q_estimated[1]
        t.transform.rotation.z = self.q_estimated[2]
        t.transform.rotation.w = self.q_estimated[3]
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    imu_processor = ImuProcessor()
    rclpy.spin(imu_processor)
    imu_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()