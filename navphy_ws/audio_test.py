import sounddevice as sd
import wave
import threading
from datetime import datetime
import time

def record_audio():
    print("In record_audio")
    while True:
        audio_data = sd.rec(44100, samplerate=rate, channels=channels, dtype='int16')
        sd.wait()
        yield audio_data

def write_to_wav(audio_data):
    waveFile.writeframes(audio_data.tobytes())

# audio_recorder = record_audio()

recording_loc= f"recordings/RECORDING{datetime.now()}"

rate = 44100
frames_per_buffer = 4096
channels = 1
format = 'int16'
audio_filename = recording_loc + ".wav"

waveFile = wave.open(audio_filename, 'wb')
waveFile.setnchannels(channels)
waveFile.setsampwidth(2)
waveFile.setframerate(rate)

print('Created audio writer')

# try:
#     gen = record_audio()
#     while True:
#         audio_data = next(gen)
#         write_thread = threading.Thread(target=write_to_wav, args=(audio_data,))
#         write_thread.start()
#         write_thread.join()
# except KeyboardInterrupt:
#     print("Recording stopped")

def audio_callback(indata, frames, time, status):
    waveFile.writeframes(indata.tobytes())

with sd.InputStream(samplerate=rate, channels=1, dtype=format, callback=audio_callback):
    while True: time.sleep(0.05)
