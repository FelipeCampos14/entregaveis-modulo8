import whisper
from deep_translator import GoogleTranslator
import re
from gtts import gTTS 
import vlc
import time



def select_language():
    actions = {
        r"(portugu[êe]s|[Pp][Tt])":"pt",
        r"(ingl[êe]s|[Ee][Nn])":"en",
        r"([Ff]ranc[eê]s|[Ff][Rr])":"fr"
    }
    lan = input("Selecione uma língua de sua escolha: (pt/en/fr) \n")
    for key, value in actions.items():
            pattern = re.compile(key)
            groups = pattern.findall(lan)
            if groups and value=="pt" :
                return 'pt'
            elif groups and value =='en':
                return 'en'
            elif groups and value =='fr':
                return 'fr'

def select_audio():
    audio = input("Selecione um audio para transcrever e traduzir: (audio1.mp3/audio2.ogg) \n")
    model = whisper.load_model("base")
    print('Ouvindo...')
    return model.transcribe(audio)['text']

def tts(language, mytext):
    myobj = gTTS(text=mytext, lang=language, slow=False)
    myobj.save("out.mp3") 
    p = vlc.MediaPlayer("out.mp3")
    p.play()
    time.sleep(4)


def main():
    while True:
        proceed = input("Deseja traduzir algum áudio? \n")
        if proceed.lower() == "sim":
            text = select_audio()
            lan = select_language()
            translator = GoogleTranslator(source='auto', target=lan).translate(text=text)
            print(f'Texto traduzido para {lan}: {translator}')
            tts(lan, translator)
        elif proceed.lower() == "não" or proceed.lower() == "nao":
            break
        else: 
            print(proceed.lower())
            print('Desculpa não entendi, repita por favor')

if __name__ == "__main__":
    main()