#!/usr/bin/env python3
# This file is part of rosgpt package.

import os
import json
# Set litellm log level using environment variable instead of deprecated set_verbose
os.environ['LITELLM_LOG'] = 'DEBUG'
import litellm
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, send_from_directory, jsonify
from flask_restful import Resource, Api
from flask_cors import CORS
from rclpy.executors import SingleThreadedExecutor
import subprocess

from ament_index_python import get_package_share_directory

# Instantiate a Flask application object with the given name
app = Flask(__name__)

# Enable Cross-Origin Resource Sharing (CORS) for the Flask app
CORS(app)

# Create an API object that wraps the Flask app to handle RESTful requests
api = Api(app)

# No need for OpenAI API key anymore
# Configure litellm for Ollama

# Initialize a threading lock for synchronizing access to shared resources
# when multiple threads are involved
spin_lock = threading.Lock()


class ROSGPTNode(Node):
    def __init__(self):
        """
        Initialize the ROSGPTNode class which is derived from the rclpy Node class.
        """
        # Call the superclass constructor and pass the name of the node
        super().__init__('chatgpt_ros2_node')\
        # Create a publisher for the 'voice_cmd' topic with a message queue size of 10
        self.publisher = self.create_publisher(String, 'voice_cmd', 10)

    def publish_message(self, message):
        """
        Publish the given message to the 'voice_cmd' topic.
        Args:
            message (str): The message to be published.
        """
        msg = String() # Create a new String message 
        msg.data = message # Convert the message to a JSON string and set the data field of the message
        self.publisher.publish(msg) # Publish the message using the publisher 
        #print('message Published: ', message) # Log the published message
        #print('msg.data Published: ', msg.data) # Log the published message


def process_and_publish_chatgpt_response(chatgpt_ros2_node, text_command, chatgpt_response, use_executors=True):
    """
    Process the chatbot's response and publish it to the 'voice_cmd' topic.

    Args:
        chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        text_command (str): The text command received from the user.
        chatgpt_response (str): The response from the chatbot.
        use_executors (bool, optional): Flag to indicate whether to use SingleThreadedExecutor. Defaults to True.
    """
    chatgpt_ros2_node.publish_message(chatgpt_response) # Publish the chatbot's response using the ROS2 node
    # If use_executors flag is True, use SingleThreadedExecutor
    if use_executors:
        executor = SingleThreadedExecutor()# Create a new executor for each request 
        executor.add_node(chatgpt_ros2_node) # Add the node to the executor
        executor.spin_once()#  Spin the executor once
        executor.remove_node(chatgpt_ros2_node) # Remove the node from the executor
    # If use_executors flag is False, use spin_lock to synchronize access
    else:
        with spin_lock:
            rclpy.spin_once(chatgpt_ros2_node)


class ROSGPTProxy(Resource):
    """
    A class derived from flask_restful.Resource, responsible for handling incoming HTTP POST requests.
    """

    def __init__(self, chatgpt_ros2_node):
        """
        Initialize the ROSGPTProxy class with the given ROS2 node.

        Args:
            chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        """
        self.chatgpt_ros2_node = chatgpt_ros2_node

    def askGPT(self, text_command):
        """
        Send a text command to the LLM model and receive a response.
        Args:
            text_command (str): The text command to be sent to the LLM.
        Returns:
            str: The response from the LLM as a JSON string.
        """
        # Create the prompt with example inputs and desired outputs
        prompt = '''You are a robot control assistant that can either:
                    1) Convert human commands into robot control JSON
                    2) Answer questions with normal text in json format

                    For robot commands, use this ontology:
                    {"action": "go_to_goal", "params": {"location": {"type": "str", "value": location}}}
                    {"action": "move", "params": {"linear_speed": linear_speed, "distance": distance, "is_forward": is_forward}}
                    {"action": "rotate", "params": {"angular_velocity": angular_velocity, "angle": angle, "is_clockwise": is_clockwise}}

                    Examples:

                    prompt: "Hello, who are you?"
                    returns: "I'm a robot control assistant that can help you control a turtle robot by converting your natural language commands into robot instructions."

                    prompt: "What can you do?"
                    returns: "I can help you control a turtle robot by understanding commands like "move forward 2 meters" or "rotate 90 degrees clockwise". Just tell me what you want the robot to do!"

                    prompt: "Move forward for 1 meter at a speed of 0.5 meters per second."
                    returns: {"action": "move", "params": {"linear_speed": 0.5, "distance": 1, "is_forward": true, "unit": "meter"}}

                    prompt: "Rotate 60 degree in clockwise direction at 10 degrees per second."
                    returns: {"action": "rotate", "params": {"angular_velocity": 10, "angle": 60, "is_clockwise": true, "unit": "degrees"}}
                    
                    prompt: "go to the bedroom, rotate 60 degrees and move 1 meter then stop"
                    returns: {"action": "sequence", "params": [{"action": "go_to_goal", "params": {"location": {"type": "str", "value": "bedroom"}}}, {"action": "rotate", "params": {"angular_velocity": 30, "angle": 60, "is_clockwise": false, "unit": "degrees"}}, {"action": "move", "params": {"linear_speed": 1, "distance": 1, "is_forward": true, "unit": "meter"}}, {"action": "stop"}]}
                    '''
        # Add the user's command to the prompt
        full_prompt = prompt + '\nprompt: ' + text_command
        print(f"Full prompt: {full_prompt}")

        # Try to send the request to the LLM and handle any exceptions
        try:
            # For Ollama, we need to use the simpler format without roles
            print(f"Attempting to connect to Ollama at http://localhost:11434")

            response = litellm.completion(
                model="ollama/gemma3",
                messages=[{"role": "user", "content": full_prompt}],
                api_base="http://localhost:11434"
            )
            print(f"Successfully got response from Ollama")

            data = response.json()
            print(f"response.json(): {data}")

            # Get the raw response text based on litellm.completion response format
            raw_response = ""

            if hasattr(response, 'choices') and len(response.choices) > 0:
                choice = response.choices[0]
                
                # Try to get the text from different possible attributes
                if hasattr(choice, 'text'):
                    raw_response = choice.text
                elif hasattr(choice, 'message') and hasattr(choice.message, 'content'):
                    raw_response = choice.message.content
                else:
                    # Last resort - convert the whole choice to string
                    raw_response = str(choice)
                    
                print(f"Raw response from LLM: {raw_response}")
            else:
                print("Could not extract text from LLM response")
                return None
                
        except Exception as e:
            print(f"Error with LLM call: {e}")
            print(f"Check if Ollama is running at http://localhost:11434")
            return None

        # Clean the response and extract the JSON
        try:
            # Try to find JSON in the response
            json_start = raw_response.find('{')
            json_end = raw_response.rfind('}') + 1
            
            if json_start >= 0 and json_end > json_start:
                # JSON found in the response
                json_text = raw_response[json_start:json_end]
                # Try to parse it to validate it's proper JSON
                json.loads(json_text)  # This will throw if invalid JSON
                
                # Check if the JSON contains 'action'
                if 'action' in json_text:
                    # If 'action' is present, format the response with both the full text and extracted JSON
                    formatted_response = json.dumps({
                        'text': raw_response,
                        'json': json_text
                    })
                else:
                    # If 'action' is not present, set 'is_text_only' to True
                    formatted_response = json.dumps({
                        'text': raw_response,
                        'is_text_only': True
                    })
                # print(f"Formatted response: {formatted_response}")
                return formatted_response
            else:
                # No JSON found - just return the text response
                print("No JSON found in response, returning raw text")
                formatted_response = json.dumps({
                    'text': raw_response,
                    'is_text_only': True
                })
                
                return formatted_response
                
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON from response: {e}")
            print(f"Returning raw text response")
            formatted_response = json.dumps({
                'text': raw_response,
                'is_text_only': True
            })
            return formatted_response



    def post(self):
        """
        Handles an incoming POST request containing a text command. The method sends the text command
        to the LLM and processes the response using the process_and_publish_chatgpt_response function in a separate thread.
        
        Returns:
            dict: A dictionary containing the LLM response as a JSON string.
        """

        text_command = request.form['text_command']
        print (f'[ROSGPT:post] Command received: {text_command}')

        chatgpt_response = self.askGPT(text_command)
        
        if chatgpt_response is None:
            error_msg = 'Failed to get response from LLM. Check if Ollama is running correctly.'
            print(f'[ROSGPT] Error: {error_msg}')
            return {'error': error_msg}, 500

        try:
            # Parse response to ensure it's valid JSON
            parsed_response = json.loads(chatgpt_response)
            # print('[ROSGPT] Response received from LLM. \n', str(parsed_response)[:60], '...')
            
            # Check if this is a text-only response or a command
            if 'is_text_only' in parsed_response:
                # For text-only responses, just return the text without publishing to ROS
                # print('[ROSGPT] Text-only response, not publishing to ROS.')
                return parsed_response
            else:
                # For command responses, publish to ROS
                threading.Thread(target=process_and_publish_chatgpt_response, 
                            args=(self.chatgpt_ros2_node, text_command, chatgpt_response, True)).start()
                
                return parsed_response
        except json.JSONDecodeError as e:
            error_msg = f'Invalid JSON response: {e}'
            print(f'[ROSGPT] Error: {error_msg}')
            return {'error': error_msg}, 500


@app.route('/')
def index():
    #print(os.path.join(app.root_path, 'webapp'))
    return send_from_directory(os.path.join(get_package_share_directory('rosgpt'), 'webapp'), 'index.html')


def main():
    rclpy.init(args=None)
    chatgpt_ros2_node = ROSGPTNode()
    api.add_resource(ROSGPTProxy, '/rosgpt', resource_class_args=(chatgpt_ros2_node,))
    # Make sure the port matches what's expected by the client
    app.run(debug=True, host='0.0.0.0', port=5000)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
