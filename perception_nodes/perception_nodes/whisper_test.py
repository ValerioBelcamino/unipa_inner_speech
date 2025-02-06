import argparse
import os
import numpy as np
import speech_recognition as sr
import whisper
import torch
import threading
import difflib
from datetime import datetime, timedelta
from queue import Queue
from time import sleep
from sys import platform
import json 



def main():

    # Check if GPU is being used
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    # path of this file
    path = os.path.dirname(os.path.realpath(__file__))
    hallucinations = json.load(open(os.path.join(path,'hallucinations.json')))['it']


    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="small", help="Model to use",
                        choices=["tiny", "base", "small", "medium", "large"])
    parser.add_argument("--non_english", action='store_true',
                        help="Don't use the english model.")
    parser.add_argument("--energy_threshold", default=0,
                        help="Energy level for mic to detect.", type=int)
    parser.add_argument("--record_timeout", default=0,
                        help="How real time the recording is in seconds.", type=float)
    parser.add_argument("--phrase_timeout", default=3,
                        help="How much empty space between recordings before we "
                             "consider it a new line in the transcription.", type=float)

    args = parser.parse_args()

    # The last time a recording was retrieved from the queue.
    phrase_time = None
    # Thread-safe Queue for passing data from the threaded recording callback.
    data_queue = Queue()
    # We use SpeechRecognizer to record our audio because it has a nice feature where it can detect when speech ends.
    recorder = sr.Recognizer()

    source = sr.Microphone(sample_rate=16000)
    print(f"Using microphone: {source}")

    # Load Whisper model
    model = args.model
    audio_model = whisper.load_model(model)
    print(f'Model loaded: {model}')

    record_timeout = args.record_timeout
    phrase_timeout = args.phrase_timeout

    transcription = ['']


    def record_callback(_, audio:sr.AudioData) -> None:
        """
        Threaded callback function to receive audio data when recordings finish.
        audio: An AudioData containing the recorded bytes.
        """
        # Grab the raw bytes and push it into the thread safe queue.
        data = audio.get_raw_data()
        data_queue.put(data)

    with source:
        recorder.adjust_for_ambient_noise(source)
        # Create a background thread that will pass us raw audio bytes.
        # We could do this manually but SpeechRecognizer provides a nice helper.
    recorder.energy_threshold = 1000  # Lower value to detect softer sounds
    recorder.dynamic_energy_threshold = True  # Allow auto-adjustment

    recorder.listen_in_background(source, record_callback, phrase_time_limit=0)

    # Start the microphone recording in a separate thread
    # mic_thread = threading.Thread(target=microphone_thread, args=(data_queue, record_timeout))
    # mic_thread.daemon = True  # Make the thread exit when the main program exits
    # mic_thread.start()

    # Loop to process audio data and transcribe
    try:
        while True:
            now = datetime.utcnow()
            # print(f'Queue size: {data_queue.qsize()}')
            if not data_queue.empty():
                phrase_complete = False
                if phrase_time and now - phrase_time > timedelta(seconds=phrase_timeout):
                    phrase_complete = True
                phrase_time = now

                # Combine audio data from the queue
                audio_data = b''.join(data_queue.queue)
                data_queue.queue.clear()

                # Convert the raw bytes to numpy array and process
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

                # Transcribe the audio using Whisper
                result = audio_model.transcribe(audio_np, temperature=0.0, language="it", fp16=torch.cuda.is_available())
                text = result['text'].strip()

                matches = difflib.get_close_matches(text, hallucinations, n=1, cutoff=0.8)
                if matches:
                    # print('halluuuu')
                    # print(f'text: {text} __ {matches[0]}')
                    continue
                # Update transcription
                if phrase_complete:
                    transcription.append(text)
                else:
                    transcription[-1] = text

                # Print the updated transcription
                os.system('cls' if os.name == 'nt' else 'clear')
                for line in transcription:
                    print(line)

                print('', end='', flush=True)
            else:
                # Sleep to reduce CPU usage
                sleep(0.25)
    except KeyboardInterrupt:
        pass

    # Final transcription output
    print("\n\nFinal Transcription:")
    for line in transcription:
        print(line)


if __name__ == "__main__":
    main()
