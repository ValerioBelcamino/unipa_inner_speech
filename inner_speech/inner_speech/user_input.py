import rclpy
import sys
import json
import time
from multiprocessing.shared_memory import SharedMemory
import subprocess
from rclpy.node import Node
from std_msgs.msg import Bool, String
import os 
import re
import ast


# Assuming your workspace structure is standard
ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_foxy_ws")  # Replace with your workspace path if needed
source_dir = os.path.join(ws_dir, 'src', 'inner_speech', 'inner_speech')

action_dict = {0: '/out_of_scope', 1: '/user_insertion', 2: '/dish_info', 3: '/meal_prep'}

class Action_Dispatcher(Node):

    def __init__(self):
        super().__init__('Action_Dispatcher')
        self.publisher_clingo_start = self.create_publisher(String, '/Module_activation', 10)
        # self.publisher_inner_speech_queries = self.create_publisher(String, '/IS_queries', 10)

        # Create a shared memory block
        shm_size = 2048  # Size in bytes
        shm = SharedMemory(create=True, size=shm_size)
        shared_memory_name = shm.name

        try:
            # Start the subprocess and pass the shared memory name
            res = subprocess.run(
                ["/home/belca/miniconda3/bin/python", os.path.join(source_dir, "llm_intent_recognition.py"), shared_memory_name],
                check=True
            )

            # Read from shared memory
            data = bytes(shm.buf[:]).decode("utf-8").strip("\x00")  # Remove null bytes
            print("Data from subprocess:")

            data = json.loads(data)
            print(data)
            print(type(data))

        except Exception as e:
            print(f"Error: {e}")
        finally:
            # Cleanup shared memory
            shm.close()
            shm.unlink()        
     
        print("Subprocess finito")
        self.send_msg_inner_speech(data)

    def send_msg_inner_speech(self, res):
        global action_dict

        msg = String()

        action_id, parameters = self.extract_info(res['Risposta'])

        msg_dict = {'question': res['Domanda'], 'parameters': parameters}
        msg.data = json.dumps(msg_dict)
        
        publisher_action_dispatch = self.create_publisher(String, action_dict[action_id], 10)
        publisher_action_dispatch.publish(msg)
        self.get_logger().info(f'Publishing {msg.data} on topic {action_dict[action_id]}')


    def extract_info(self, formatted_str):
        try:
            # Estrai il numero dell'azione
            action_match = re.search(r"#(\d+)", formatted_str)
            if not action_match:
                raise ValueError("Numero azione non trovato")
            action_id = int(action_match.group(1))

            # Estrai il dizionario
            dict_match = re.search(r"#\{(.*?)\}", formatted_str, re.DOTALL)
            if not dict_match:
                raise ValueError("Dizionario non trovato")
            
            # Converti il dizionario in un oggetto Python
            raw_dict = dict_match.group(1)
            # raw_dict = '{' + raw_dict + '}'
            # print(raw_dict)
            parameters = eval(f"{{{raw_dict}}}")

            return action_id, parameters
        except Exception as e:
            print(f"Errore durante l'estrazione: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)




    ad = Action_Dispatcher()

    rclpy.spin(ad)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ad.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()