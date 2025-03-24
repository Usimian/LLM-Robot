#!/usr/bin/env python3
# This file is part of rosgpt package.

import json
import rclpy
from rclpy.node import Node
import requests


class ROSGPTClient(Node):
    def __init__(self):
        super().__init__('rosgpt_client')
        self.declare_parameter('server_url', 'http://localhost:5000/rosgpt')
        self.server_url = self.get_parameter('server_url').value

        self.get_logger().info('ROSGPT client node started')

        self.send_text_command()

    def send_text_command(self): # Send input to the ROSGPT system, then get a response from the LLM.
        while rclpy.ok():
            text_command = input("Enter a text command: ")
            data = {'text_command': text_command}

            try:
                print(f"Sending request to {self.server_url}")
                print(f"Prompt: {text_command}")
                response = requests.post(self.server_url, data=data)

                if response.status_code == 200:
                    try:
                        response_str = response.content.decode('utf-8')
                        response_dict = json.loads(response_str)

                        if 'is_text_only' in response_dict:
                            # If 'is_text_only' is in the response, print the response text
                            self.get_logger().info('Response: {}'.format(response_dict['text']))
                        else:
                            # If 'is_text_only' is not in the response, print the JSON action command
                            self.get_logger().info('JSON: {}'.format(json.loads(response_dict['json'])))
                    except Exception as e:
                        print('[Exception] Error parsing response:', str(e))
                        print('Raw response:', response.content.decode('utf-8', errors='replace'))
                else:
                    error_content = response.content.decode('utf-8', errors='replace')
                    self.get_logger().error(f'Error {response.status_code}: {error_content}')
            except requests.exceptions.ConnectionError:
                self.get_logger().error(f'Connection error: Could not connect to {self.server_url}. Is the server running?')
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    rosgpt_client = ROSGPTClient()

    rclpy.spin(rosgpt_client)

    rosgpt_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

