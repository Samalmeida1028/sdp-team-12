import sounddevice as sd
import numpy as np
import wave
import threading
from datetime import datetime
import time

stream_started = True

def start_recording_audio(): # record the audio 
    print('Starting audio recording thread')
    while stream_started:
        data = sd.rec(frames_per_buffer, samplerate=rate, channels=channels, dtype='int16')
        sd.wait()
        waveFile.writeframes(data.tobytes())

recording_loc= f"recordings/RECORDING{datetime.now()}"

rate = 44100
frames_per_buffer = 1024
channels = 1
format = 'int16'
audio_filename = recording_loc + ".wav"

waveFile = wave.open(audio_filename, 'wb')
waveFile.setnchannels(channels)
waveFile.setsampwidth(2)
waveFile.setframerate(rate)

print('Created audio writer')

# start audio recording thread ONCE
audio_thread = threading.Thread(target=start_recording_audio)
audio_thread.daemon = True
audio_thread.start()

time.sleep(10)

print('Pausing audio recording thread')
stream_started = False