# Author: Arjun Viswanathan
# Credit: SDP Team 11, Alice Hung
# Date created: 3/22/24
# Date last modified: 3/23/24

'''
How to run:
1. Connect bluetooth device
    First time: connect HDMI in to NUC and manually connect from bluetooth menu. Note down Address of device
    Every other time: turn on bluetooth earpiece. In the terminal, type the following:
        bluetooth ctl connect ADDRESS
    To disconnect or if that command gives error:
        bluetooth ctl disconnect ADDRESS
        bluetooth ctl connect ADDRESS
    For our case, ADDRESS = 64:72:D8:49:95:D9
2. Verify connected device using bt-device -l 
3. Source the bashrc file
4. Run python3 nlp_s2t.py file and enjoy :D
'''

import whisper
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write

Model = 'small'     # Whisper model size (tiny, base, small, medium, large)
English = True      # Use English-only model?
Translate = False   # Translate non-English to English?
SampleRate = 44100  # Stream device recording frequency
BlockSize = 20      # Block size in milliseconds
Threshold = 0.01     # Minimum volume threshold to activate listening
Vocals = [50, 1000] # Frequency range to detect sounds that could be speech
EndBlocks = 10      # Number of blocks to wait before sending to Whisper

class StreamHandler:
   def __init__(self, assist=None):
       if assist == None:  # If not being run by my assistant, just run as terminal transcriber.
           class fakeAsst(): running, talking, analyze = True, False, None
           self.asst = fakeAsst()  # anyone know a better way to do this?
       else: self.asst = assist

       self.running = True
       self.padding = 0
       self.prevblock = self.buffer = np.zeros((0,1))
       self.fileready = False

       print("\033[96mLoading Whisper Model..\033[0m", end='', flush=True)
       self.model = whisper.load_model(f'{Model}{".en" if English else ""}')
       print("\033[90m Done.\033[0m")

   def callback(self, indata, frames, time, status):
       #if status: print(status) # for debugging, prints stream errors.
       if not any(indata):
           print('\033[31m.\033[0m', end='', flush=True) # if no input, prints red dots
           return

       freq = np.argmax(np.abs(np.fft.rfft(indata[:, 0]))) * SampleRate / frames

       if np.sqrt(np.mean(indata**2)) > Threshold and Vocals[0] <= freq <= Vocals[1] and not self.asst.talking:
           print('.', end='', flush=True)
           if self.padding < 1: self.buffer = self.prevblock.copy()

           self.buffer = np.concatenate((self.buffer, indata))
           self.padding = EndBlocks
       else:
           self.padding -= 1

           if self.padding > 1:
               self.buffer = np.concatenate((self.buffer, indata))
           elif self.padding < 1 < self.buffer.shape[0] > SampleRate: # if enough silence has passed, write to file.
               self.fileready = True
               write('dictate.wav', SampleRate, self.buffer) # I'd rather send data to Whisper directly..
               self.buffer = np.zeros((0,1))
           elif self.padding < 1 < self.buffer.shape[0] < SampleRate: # if recording not long enough, reset buffer.
               self.buffer = np.zeros((0,1))
               print("\033[2K\033[0G", end='', flush=True)
           else:
               self.prevblock = indata.copy() #np.concatenate((self.prevblock[-int(SampleRate/10):], indata)) # SLOW

   def process(self):
       if self.fileready:
           print("\n\033[90mTranscribing..\033[0m")
           result = self.model.transcribe('dictate.wav',fp16=False,language='en' if English else '',task='translate' if Translate else 'transcribe')
           print(result['text'])

           if self.asst.analyze != None: self.asst.analyze(result['text'])

           self.fileready = False
           
           message = result['text'].lower().replace(",","").replace(".","")
           return message
       else:
           return "no"

   def listen(self):
       print("\033[32mListening.. \033[37m(Ctrl+C to Quit)\033[0m")
       with sd.InputStream(channels=1, callback=self.callback, blocksize=int(SampleRate * BlockSize / 1000), samplerate=SampleRate):
           while self.running and self.asst.running:
               processedout = self.process()
               if(processedout != "no"):
                    break

       return processedout
   
def main():
    handler = StreamHandler()
    text = handler.listen().lower().split()

    if "holly" in text:
        hollyind = text.index('holly')
        if "target" in text:
            targetind = text.index('target')
            if targetind > hollyind and text[targetind + 1].isdigit():
                target = text[targetind + 1]
                print("Putting new target {} into text file".format(target))

                fd = open("targets.txt", "a")
                fd.write(str(target) + "\n")

if __name__ == "__main__":
    main()