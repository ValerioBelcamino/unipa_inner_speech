import os
import numpy as np
import speech_recognition as sr
import whisper
import torch
import difflib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timedelta
from queue import Queue
from time import sleep
import json
from dotenv import load_dotenv

# Load environment variables from .env file
BASE_DIR = "/home/belca/Desktop/ros2_humble_ws/src"
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)

class SpeechRecognitionNode(Node):
    def __init__(self, model, phrase_timeout, hallucinations):
        super().__init__('speech_recognition_node')
        self.publisher_ = self.create_publisher(String, '/user_input', 10)
        self.data_queue = Queue()
        self.recorder = sr.Recognizer()
        self.source = sr.Microphone(sample_rate=16000)
        self.audio_model = whisper.load_model(model)
        self.hallucinations = hallucinations
        self.transcription = ['']
        self.phrase_time = None
        self.phrase_timeout = phrase_timeout


    def process_audio(self):

        source = sr.Microphone(sample_rate=16000)
        print(f"Using microphone: {source}")
        with self.source:
            self.recorder.adjust_for_ambient_noise(self.source)
        self.recorder.energy_threshold = 1000
        self.recorder.dynamic_energy_threshold = True

        def record_callback(_, audio:sr.AudioData) -> None:
            """
            Threaded callback function to receive audio data when recordings finish.
            audio: An AudioData containing the recorded bytes.
            """
            # Grab the raw bytes and push it into the thread safe queue.
            data = audio.get_raw_data()
            self.data_queue.put(data)

        self.recorder.listen_in_background(source, record_callback, phrase_time_limit=0)


        while rclpy.ok():
            now = datetime.utcnow()
            if not self.data_queue.empty():
                phrase_complete = False
                if self.phrase_time and now - self.phrase_time > timedelta(seconds=self.phrase_timeout):
                    phrase_complete = True
                self.phrase_time = now

                audio_data = b''.join(self.data_queue.queue)
                self.data_queue.queue.clear()
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

                result = self.audio_model.transcribe(audio_np, temperature=0.0, language="it", fp16=torch.cuda.is_available())
                text = result['text'].strip()

                if difflib.get_close_matches(text, self.hallucinations, n=1, cutoff=0.8):
                    continue
                
                if phrase_complete:
                    self.transcription.append(text)
                else:
                    self.transcription[-1] = text

                os.system('cls' if os.name == 'nt' else 'clear')
                for line in self.transcription:
                    print(line)
                print('', end='', flush=True)
                
                if len(text) > 0:
                    msg = String()
                    msg.data = text
                    self.publisher_.publish(msg)
            else:
                sleep(0.25)


def main():
    rclpy.init()
    
    path = os.path.dirname(os.path.realpath(__file__))
    # Assuming your workspace structure is standard
    ws_dir = os.getenv("ROS2_WORKSPACE") 
    path = os.path.join(ws_dir, 'perception_nodes', 'perception_nodes')

    hallucinations = json.load(open(os.path.join(path, 'hallucinations.json')))['it']
    
    node = SpeechRecognitionNode('small', 3, hallucinations)
    node.process_audio()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
