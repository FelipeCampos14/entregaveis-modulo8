#! /bin/env python3

import re
import time

intent_dict = {
    r"\b(?:(?:[Bb]o[am])\s(?:tarde|dia|noite))": "greetings",
    r"\b(?:[Vv][aá][i]*|[Dd]ir[i]*ja[-se]*|l[e]*v[e]*|ande).+(?:ao|[àa])?\s\b([\wáéíóúâêîôûãõç]+\b)": "command"
}

actions = {
    "biblioteca":"a posição da biblioteca fica em (2.0,3.0)",
    "secretaria":"a posição da secretaria fica em (1.0,4.0)",
    "laboratório":"a posição da laboratório fica em (0.0,4.0)"
}
def main():
    while True:
        command = input("Para onde deseja ir: ")
        if command == "quit()":
            break
        else:
            for key, value in intent_dict.items():
                pattern = re.compile(key)
                groups = pattern.findall(command)
                confirmacao = input(f"Você deseja ir à {groups[0]}? (yes|no) ")
                if groups and value=="command" and confirmacao=="yes":
                    for key_actions in actions.keys():
                        if key_actions == groups[0]:
                            print(actions[key_actions][0])
                            print('Andando...')
                            time.sleep(5)
                            print("Cheguei!")
                elif confirmacao =="no":
                    break
                


                
        