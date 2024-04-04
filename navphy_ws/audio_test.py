import pyaudio
import wave
import threading
from datetime import datetime
import time

def start_recording_audio(): # record the audio 
    print('Starting audio recording thread')
    while stream_started:
        data = stream.read(frames_per_buffer)
        waveFile.writeframes(data)

def stop_recording_audio():
    print('Pausing audio recording thread')
    if stream_started:
        stream_started = False

    #################### Optimized Option ####################
    # stream.stop_stream()
    # stream.close()
    ##########################################################

recording_loc= f"recordings/RECORDING{datetime.now()}"

rate = 20000
frames_per_buffer = 1024
channels = 1
format = pyaudio.paInt16
audio_filename = recording_loc + ".wav"
audio = pyaudio.PyAudio()

#################### Regular Option ####################
stream = audio.open(format=format,
                                channels=channels,
                                rate=rate,
                                input=True,
                                frames_per_buffer = frames_per_buffer)
stream.start_stream()
########################################################
stream_started = True

waveFile = wave.open(audio_filename, 'wb')
waveFile.setnchannels(channels)
waveFile.setsampwidth(audio.get_sample_size(format))
waveFile.setframerate(rate)

print('Created audio writer')

# start audio recording thread ONCE
audio_thread = threading.Thread(target=start_recording_audio)
audio_thread.start()
audio_thread.join()
#################### Optimized Option ####################
# stream = audio.open(format=format,
#                           channels=channels,
#                           rate=rate,
#                           input=True,
#                           frames_per_buffer = frames_per_buffer)
# stream.start_stream()
##########################################################

time.sleep(10)

stop_recording_audio()