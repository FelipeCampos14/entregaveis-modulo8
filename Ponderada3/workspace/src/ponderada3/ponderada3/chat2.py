#! /bin/env python3

import re
import time

intent_dict = {
    r"\b(?:[Vv][aá][i]*|[Dd]ir[i]*ja[-se]*|l[e]*v[e]*|ande).+(?:[àa]?[o]|)?\s\b(lab[a-o-r]+t[óo]rio|lab|s[c-e-r]+[a-r-t]+[a-i-r]+|b[i-l]+b[i-l]+[e-o-t]+ca)\b": "command",
}

actions = {
    r"(b[i-l]+b[i-l]+[e-o-t]+ca)":"a posição da biblioteca fica em (2.0,3.0)",
    r"(s[c-e-r]+[a-r-t]+[a-i-r]+)":"a posição da secretaria fica em (1.0,4.0)",
    r"(lab[a-o-r]+t[óo]rio|lab)":"a posição do laboratório fica em (0.0,4.0)"
}
def main():
    while True:
        command = input("Como posso te ajudar? ")
        if command == "quit()":
            break
        else:
            for key, value in intent_dict.items():
                pattern = re.compile(key)
                groups = pattern.findall(command)
                if groups and value=="command" :
                    for key_actions in actions.keys():
                        pattern_key_action = re.compile(key_actions)
                        if pattern_key_action.findall(groups[0].lower()):
                            print(actions[key_actions])
                            print('Andando...')
                            time.sleep(5)
                            print("Cheguei!\n")
                else:
                    print("Não entendi, repita por favor.\n")


                
        