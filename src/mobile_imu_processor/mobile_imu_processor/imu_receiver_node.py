import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import json
import threading
from flask import Flask, request, jsonify

# Flask App setup
app = Flask(__name__)
# Global variable to hold the ROS 2 node instance
ros_node = None

@app.route('/imu_data', methods=['POST'])
def handle_imu_data():
    """Receives JSON data via HTTP POST, parses the nested structure, and publishes Imu message."""
    global ros_node
    try:
        data = request.json
        # ros_node.get_logger().info(f"Received JSON: {data}") # REMOVE or COMMENT OUT after debugging

        # Initialize placeholders for the data we need
        accel_data = None
        gyro_data = None
        
        # The sensor data is in a list called 'payload'
        for sensor_reading in data.get('payload', []):
            name = sensor_reading.get('name')
            values = sensor_reading.get('values')

            if name == 'accelerometer' and values:
                # Found calibrated acceleration data
                accel_data = values
            elif name == 'gyroscope' and values:
                # Found calibrated gyroscope data
                gyro_data = values
        
        # Check if we got both necessary sensors
        if accel_data is None or gyro_data is None:
            ros_node.get_logger().warn("Missing calibrated accelerometer or gyroscope data in payload.")
            return jsonify({"status": "partial_data"}), 200 # Acknowledge but skip publishing

        # --- Create and Populate the Imu Message ---
        imu_msg = Imu()
        imu_msg.header.stamp = ros_node.get_clock().now().to_msg()
        imu_msg.header.frame_id = "phone_imu_link"

        # Linear Acceleration (m/s^2)
        imu_msg.linear_acceleration.x = float(accel_data.get('x', 0.0))
        imu_msg.linear_acceleration.y = float(accel_data.get('y', 0.0))
        imu_msg.linear_acceleration.z = float(accel_data.get('z', 0.0))

        # Angular Velocity (rad/s)
        imu_msg.angular_velocity.x = float(gyro_data.get('x', 0.0))
        imu_msg.angular_velocity.y = float(gyro_data.get('y', 0.0))
        imu_msg.angular_velocity.z = float(gyro_data.get('z', 0.0))
        
        # Publish the raw data
        ros_node.imu_pub.publish(imu_msg)
        ros_node.get_logger().info(f"Published IMU data: A_z={imu_msg.linear_acceleration.z:.2f}, G_z={imu_msg.angular_velocity.z:.2f}")

    except Exception as e:
        ros_node.get_logger().error(f"Error processing IMU data: {e}")
        return jsonify({"status": "error", "message": str(e)}), 400
        
    return jsonify({"status": "received"}), 200

def flask_thread():
    """Function to run the Flask server in a separate thread."""
    # Use 0.0.0.0 to listen on all interfaces (necessary for Wi-Fi)
    app.run(host='0.0.0.0', port=8000, debug=False, use_reloader=False)

class ImuReceiver(Node):
    def __init__(self):
        super().__init__('imu_receiver_node')
        self.imu_pub = self.create_publisher(Imu, '/mobile_imu/raw', 10)
        
def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = ImuReceiver() # Assign node instance to the global variable
    
    # Start the Flask server thread
    flask_server_thread = threading.Thread(target=flask_thread)
    flask_server_thread.daemon = True # Allows ROS to exit even if the thread is running
    flask_server_thread.start()
    
    ros_node.get_logger().info('ROS 2 IMU Receiver Node is running and Flask server started on port 8000.')
    
    # Spin the ROS 2 node
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()