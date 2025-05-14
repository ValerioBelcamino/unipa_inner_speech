import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common_msgs.msg import Intent
import time
import ast
import json
import os, random
from dotenv import load_dotenv
load_dotenv()


class IntentOutputListener(Node):
    def __init__(self):
        super().__init__('test_listener')
        self.user_input = None
        self.action_name = None
        self.parameters = None
        self.subscription = self.create_subscription(
            Intent,
            '/user_intent',
            self.callback,
            10
        )

    def callback(self, intent_msg):
        self.get_logger().info('Received: "%s"\n' % intent_msg)
        self.user_input = intent_msg.user_input
        self.action_name = intent_msg.action_name
        self.parameters = json.loads(intent_msg.parameters)
        
    def reset(self):
        self.user_input = None
        self.action_name = None
        self.parameters = None


def extract_examples(filename='examples.json'):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    full_path = os.path.join(dir_path, filename)

    with open(full_path, 'r') as file:
        data = json.load(file)
    processed_data = [(example["question"], example["action_name"], example["parameters"]) for example in data]
    return processed_data


@pytest.fixture(scope="module")
def ros_setup():
    # Initialize ROS once for all tests in the module
    rclpy.init()
    
    # Create nodes that will be shared across tests
    input_node = rclpy.create_node('test_input_publisher')
    publisher = input_node.create_publisher(String, '/user_input', 10)
    
    # Create the listener node
    listener = IntentOutputListener()
    
    # Yield the nodes for the tests to use
    yield publisher, listener
    
    # Clean up after all tests are done
    input_node.destroy_node()
    listener.destroy_node()
    rclpy.shutdown()


def get_examples():
    """Helper function to get examples for parameterized tests"""
    scenario = os.getenv("SCENARIO")
    example_filename = "examples.json" if scenario is None else f"examples_{scenario}.json"
    examples = extract_examples(filename=example_filename)
    print(f"Found {len(examples)} examples in the dataset.")
    random.shuffle(examples)
    return examples


@pytest.mark.parametrize("user_input,expected_action_name,expected_parameters", get_examples())
def test_single_intent_response(ros_setup, user_input, expected_action_name, expected_parameters):
    publisher, listener = ros_setup
    
    # Reset the listener's state before each test
    listener.reset()
    
    # Publish message
    msg = String()
    msg.data = user_input
    publisher.publish(msg)
    print(f"[TEST] Published:\n{msg.data} on topic /user_input")

    # Spin listener until we get a response or timeout
    timeout = time.time() + 15  # 10 seconds timeout
    while time.time() < timeout:
        rclpy.spin_once(listener, timeout_sec=0.1)
        if listener.action_name is not None and listener.user_input == user_input:
            break
    time.sleep(1)  # Give some time for the listener to process the message
    # Make sure we received something
    if listener.user_input != user_input:
        assert False, f"Expected input: {user_input}, but got: {listener.user_input}"

    assert listener.action_name is not None, f"No response received within timeout for input: {user_input}"

    # Assert message content
    if listener.action_name != expected_action_name:
        print(f"Expected intent: {expected_action_name}, but got: {listener.action_name}")
        assert False, f"Expected intent: {expected_action_name}, but got: {listener.action_name}"
    
    elif listener.parameters != expected_parameters:
        if expected_parameters == {}:
            print(f"Expected empty parameters, but got: {listener.parameters}")
            assert False, f"Expected empty parameters, but got: {listener.parameters}"
        else:
            mistakes = ""
            for key in expected_parameters.keys():
                if key not in listener.parameters:
                    print(f"Key '{key}' not found in received parameters.")
                    mistakes += f"{key} not found\n"
                elif listener.parameters[key] != expected_parameters[key]:
                    print(f"Value mismatch for key '{key}': '{listener.parameters[key]}' != '{expected_parameters[key]}'")
                    mistakes += f"{key}: '{listener.parameters[key]}' != '{expected_parameters[key]}'\n"
        
            error_msg = f"Mismatch between expected and actual parameters:\n\n{mistakes}"
                    
            assert False, error_msg

#  to run: python3 -m pytest -vv --no-header --tb=native /home/kimary/unipa/src/unipa_inner_speech/intent_recognition/test/test_intent_recognition.py