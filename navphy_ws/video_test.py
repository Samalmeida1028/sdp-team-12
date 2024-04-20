import subprocess
import time

# Define the FFmpeg command
ffmpeg_command = [
    'ffmpeg',
    '-f', 'v4l2',
    '-input_format', 'mjpeg',
    '-r', '60',
    '-video_size', '1280x720',
    '-i', '/dev/video0',
    '-f', 'alsa',
    '-i', 'default',
    '-c:v', 'copy',
    '-c:a', 'aac',
    'recordings/ffmpeg_out.mp4'
]

# Start the FFmpeg process as a subprocess
process = subprocess.Popen(ffmpeg_command)

while True:
    time.sleep(5)

    print("Pausing FFmpeg process...")
    process.send_signal(subprocess.signal.SIGSTOP)

    time.sleep(5)

    print("Resuming FFmpeg process...")
    process.send_signal(subprocess.signal.SIGCONT)

    time.sleep(5)

    print("Stopping recording and saving file...")
    process.terminate()
    process.wait()

    break
