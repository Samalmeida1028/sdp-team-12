# pip install SpeechRecognition, soundfile, openai-whisper

import speech_recognition as sr

def speech_to_text():
    # Initialize recognizer
    recognizer = sr.Recognizer()

    # Use default microphone as source
    with sr.Microphone() as source:
        print("Listening...")

        # Adjust for ambient noise
        recognizer.adjust_for_ambient_noise(source)

        # Capture the audio input
        audio = recognizer.listen(source)

        print("Processing...")

        try:
            # Use Google Speech Recognition
            text = recognizer.recognize_whisper(audio)
            print("You said:", text)
        except sr.UnknownValueError:
            print("Could not understand audio")
        except sr.RequestError as e:
            print("Could not request results; {0}".format(e))

if __name__ == "__main__":
    speech_to_text()