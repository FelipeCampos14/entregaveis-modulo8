import whisper
import simpleaudio as sa
import translators as ts
from deep_translator import GoogleTranslator
import re
from gtts import gTTS 
import pygame



def select_language():
    actions = {
        r"(portugu[êe]s|[Pp][Tt])":"pt",
        r"(ingl[êe]s|[Ee][Nn])":"en",
        r"(japon[êe]s|[Jj][Nn])":"ja"
    }
    lan = input("Selecione uma língua de sua escolha: (pt/en/ja) \n")
    for key, value in actions.items():
            pattern = re.compile(key)
            groups = pattern.findall(lan)
            if groups and value=="pt" :
                return 'pt'
            elif groups and value =='en':
                return 'en'
            elif groups and value =='ja':
                return 'ja'

def select_audio():
    audio = input("Selecione um audio para transcrever e traduzir: (audio1.mp3/audio2.mp3/audio3.mp3) \n")
    model = whisper.load_model("base")
    print('ouvindo')
    return model.transcribe(audio)['text']

def tts(language, mytext):
    if language == 'ja':
        language = 'jn'
    myobj = gTTS(text=mytext, lang=language, slow=False)
    myobj.save("welcome.mp3") 
    pygame.init()
    pygame.mixer.music.load("welcome.mp3")
    pygame.mixer.music.play()
    pygame.event.wait()

 

def main():
    text = select_audio()
    lan = select_language()
    translator = GoogleTranslator(source='auto', target=lan).translate(text=text)
    print(translator)
    tts(lan, translator)

if __name__ == "__main__":
    main()