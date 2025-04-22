#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
from wifi_viz.srv import TriggerCriticalAction # Import the service type

class DefaultCriticalActionService(Node):
    """
    A simple ROS 2 service node that listens for TriggerCriticalAction requests.
    It parses the JSON data from the request and logs it in a pretty format.
    """
    def __init__(self):
        """Initializes the node and creates the service."""
        super().__init__('default_critical_action_service')
        self.srv = self.create_service(
            TriggerCriticalAction,
            'trigger_critical_action', # Default service name
            self.handle_request)
        self.get_logger().info('Default Critical Action Service started. Ready to receive requests on /trigger_critical_action.')

    def handle_request(self, request, response):
        """
        Callback function to handle incoming service requests.
        Parses JSON and logs it.
        """
        self.get_logger().info('Received critical action request.')
        try:
            # Parse the JSON data from the request string
            data = json.loads(request.json_data)

            # Pretty print the received dictionary
            pretty_data = json.dumps(data, indent=2)
            self.get_logger().info(f'Critical Data:\n{pretty_data}')

            # Set the service response
            response.success = True
            response.message = 'Data logged successfully.'

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON data: {e}')
            response.success = False
            response.message = f'JSON Decode Error: {e}'
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')
            response.success = False
            response.message = f'Unexpected Error: {e}'

        return response

def main(args=None):
    """Initializes rclpy, creates the service node, and spins."""
    rclpy.init(args=args)
    service_node = DefaultCriticalActionService()
    try:
        rclpy.spin(service_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly destroy the node and shut down rclpy
        service_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
