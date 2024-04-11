import sounddevice as sd
import wave
import threading
from datetime import datetime

def record_audio():
    while True:
        audio_data = sd.rec(44100, samplerate=rate, channels=channels, dtype='int16')
        sd.wait()
        yield audio_data

def write_to_wav(audio_data):
    waveFile.writeframes(audio_data.tobytes())

audio_recorder = record_audio()

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

try:
    while True:
        audio_data = next(record_audio())
        write_thread = threading.Thread(target=write_to_wav, args=(audio_data,))
        write_thread.start()
        write_thread.join()
except KeyboardInterrupt:
    print("Recording stopped")
