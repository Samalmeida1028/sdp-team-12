from whisper_mic.whisper_mic import WhisperMic

mic = WhisperMic()

while 1: # keep listening for input until hears keyword "turn"
    result = ""
    result = mic.listen()
    result = result.lower() # make it lower case -> case insensitive
    print(result)