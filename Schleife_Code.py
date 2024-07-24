#!/usr/bin/env python3

# prerequisites: as described in https://alphacephei.com/vosk/install and also python module `sounddevice` (simply run command `pip install sounddevice`)
# Example usage using Dutch (nl) recognition model: `python test_microphone.py -m nl`
# For more help run: `python test_microphone.py -h`

#Installationsvorbereitung
#Visual Code 
#sudo apt-get install code (Visual Code)
#Add: Install Python-Extension in Visual Code
#
#Sprachkommando/-befehle
#pip3 install vosk 
#LÖSUNG BEI FEHLERMELDUNG: sudo rm /usr/lib/python3.11/EXTERNALLY-MANAGED)
#git clone https://github.com/alphacep/vosk-api.git (Repository)

#OpenCV
#pip install ultralytics
#pip install opencv-python

#Threading
#pip install threaded

#Sprachausgabe
#sudo apt-get install espeak-ng
#pip install py-espeak-ng

#Soundgeräteerkennung
#pip3 install sounddevice
#sudo apt-get install portaudio19-dev

#Audiowiedergabe (.mp3)
#pip install pydub
#pip install pyaudio

#----------------------------------------------------------------------------------------------
#Textdateien zur Erkennung der Sprachbefehle:
#name.txt
#neue_notiz.txt
#neuer_name.txt
#notiz_abspielen.txt
#notiz_erstellen.txt
#notiz_loeschen.txt
#objektsuche.txt
#objekteingabe.txt
#neues_objekt.txt
#vorwaerts.txt

#Textdateien auf dem Desktop ablegen. Im Code muss der Pfad angepasst werden.
#----------------------------------------------------------------------------------------------
#Musikdateien zur Tonausgabe:
#accepted.mp3
#declined.mp3
#notizsound.mp3

#Musikdateien im Ordner Musik ablegen. Im Code muss der Pfad angepasst werden.
#----------------------------------------------------------------------------------------------


#version
#1.0 Code für Sprachbefehle
#2.0 Code für Sprachbefehle + Objekterkennung via OpenCV
#2.2 Code für Sprachbefehle + Objekterkennung via OpenCV (mit Multithreading)
#3.0 Code für Sprachbefehle + Objekterkennung + Motorsteuerung via OpenCV (mit Multithreading)

import argparse
import queue
import sys
import sounddevice as sd
import json
import time
import threading
import math

#Spracheingabe, Sprachausgabe, Tonausgabe
from vosk import Model, KaldiRecognizer

from espeakng import ESpeakNG
from pydub import AudioSegment
from pydub.playback import play
from time import sleep
from gpiozero import DigitalOutputDevice, DistanceSensor
from time import time
from time import *
# Bibliotheken laden
from gpiozero import LED

# Initialisierung von GPIO3 als LED (Ausgang)
#led = LED(17)

# LED einschalten
#led.on()



sys.settrace
#Sprachausgabe (deutsch), Sprachtempo, MP3-TÃ¶ne und Textdateien
esng = ESpeakNG(voice='de')
esng.pitch = 32
esng.speed = 150
accepted = AudioSegment.from_mp3("/home/team42/Music/accepted.mp3")
declined = AudioSegment.from_mp3("/home/team42/Music/declined.mp3")
notizsound = AudioSegment.from_mp3("/home/team42/Music/notizsound.mp3")
starting = AudioSegment.from_mp3("/home/team42/Music/startup.mp3")
geschichteerzaehlen = AudioSegment.from_mp3("/home/team42/Music/geschichte.mp3")
#an = ["licht an", "an", "schalte das licht an", "schalte bitte das licht an", "einschalten", "licht ein", "ein"]
#aus = ["licht aus", "aus", "schalte das licht aus", "schalte bitte das licht aus", "ausschalten"]
helpfix = ["hallo hält fix","hallo halb sechs","hallo herr x", "hallo help fix", "hallo halbwegs", "hält fix", "hallo herr fix", "hallo fix", "hallo hell fix", "hallo hell freaks", "hallo helm fix"]


objekteingabe=open("/home/team42/Desktop/objekteingabe.txt","r").read()
objekteingabe_woerter= objekteingabe.splitlines()

notab=open("/home/team42/Desktop/notiz_abspielen.txt","r").read()
notab_woerter= notab.splitlines()

notloe=open("/home/team42/Desktop/notiz_loeschen.txt","r").read()
notloe_woerter= notloe.splitlines()

notauf=open("/home/team42/Desktop/notiz_erstellen.txt","r").read()
notauf_woerter= notauf.splitlines()

namneu=open("/home/team42/Desktop/neuer_name.txt","r").read()
namneu_woerter= namneu.splitlines()

leer=open('/home/team42/Desktop/name.txt',"r").read()
l= leer.splitlines()

geschichte=open('/home/team42/Desktop/geschichte.txt',"r").read()
geschichte_woerter= geschichte.splitlines()

vorwaerts=open('/home/team42/Desktop/vorwaerts.txt',"r").read()
vorwaertsfahren_woerter = vorwaerts.splitlines()

links=open('/home/team42/Desktop/links.txt',"r").read()
linksfahren_woerter = links.splitlines()

rechts=open('/home/team42/Desktop/rechts.txt',"r").read()
rechtsfahren_woerter = rechts.splitlines()

rueckwaerts=open('/home/team42/Desktop/rueckwaerts.txt',"r").read()
rueckwaertsfahren_woerter = rueckwaerts.splitlines()

schnell=open('/home/team42/Desktop/schnell.txt',"r").read()
schnell_vorwaertsfahren_woerter = schnell.splitlines()

route=open('/home/team42/Desktop/routing.txt',"r").read()
routing_woerter = route.splitlines()

random=open('/home/team42/Desktop/random.txt',"r").read()
random_woerter = random.splitlines()

begruessung='hallo'
begruessung2="hey"
ausgabe='wie kann ich dir helfen?'
ausgabe2="wobei kann ich dich unterstützen?"

q = queue.Queue()

##################Motorsteuerung###################
# Konfiguration der GPIO-Pins für den ersten Motor
EN_PIN1 = 16
DIR_PIN1 = 21
STEP_PIN1 = 20
M1_PIN1 = 5  # Beispiel GPIO-Pin für M1 des ersten Motors
M2_PIN1 = 6  # Beispiel GPIO-Pin für M2 des ersten Motors

# Konfiguration der GPIO-Pins für den zweiten Motor
EN_PIN2 = 13
DIR_PIN2 = 26
STEP_PIN2 = 19
M1_PIN2 = 27  # Beispiel GPIO-Pin für M1 des zweiten Motors
M2_PIN2 = 7   # Beispiel GPIO-Pin für M2 des zweiten Motors

# Initialisierung der GPIO-Pins für den ersten Motor
enable1 = DigitalOutputDevice(EN_PIN1)
direction1 = DigitalOutputDevice(DIR_PIN1)
step1 = DigitalOutputDevice(STEP_PIN1)
m1_1 = DigitalOutputDevice(M1_PIN1)
m2_1 = DigitalOutputDevice(M2_PIN1)

# Initialisierung der GPIO-Pins für den zweiten Motor
enable2 = DigitalOutputDevice(EN_PIN2)
direction2 = DigitalOutputDevice(DIR_PIN2)
step2 = DigitalOutputDevice(STEP_PIN2)
m1_2 = DigitalOutputDevice(M1_PIN2)
m2_2 = DigitalOutputDevice(M2_PIN2)

# Initialisierung des Ultraschallsensors
sensor = DistanceSensor(echo=23, trigger=24)
###############################################################




def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
    "-l", "--list-devices", action="store_true",
    help="show list of audio devices and exit")
args, remaining = parser.parse_known_args()
if args.list_devices:
    print(sd.query_devices())
    parser.exit(0)
parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
    parents=[parser])
parser.add_argument(
    "-f", "--filename", type=str, metavar="FILENAME",
    help="audio file to store recording to")
parser.add_argument(
    "-d", "--device", type=int_or_str,
    help="input device (numeric ID or substring)")
parser.add_argument(
    "-r", "--samplerate", type=int, help="sampling rate")
parser.add_argument(
    "-m", "--model", type=str, help="language model; e.g. en-us, fr, nl; default is en-us")
args = parser.parse_args(remaining)



# Funktion, um Sprachbefehle  durchzufÃƒÂ¼hren
def sprachbefehle():
    # Schleife benÃƒÂ¶tigt 4sek um durchzulaufen, nach 5 Minuten ins time out
    time_out = 75
    vorlesen = open('/home/team42/Desktop/name.txt','r').read()
    vor= vorlesen.splitlines()
    #thread1.join()
    while time_out > 0:
        notizvorhanden = open('/home/team42/Desktop/neue_notiz.txt','r').read()
        notizvor= notizvorhanden.splitlines()
        data = q.get()
       
        if rec.AcceptWaveform(data):
            jsonRec = json.loads(rec.Result())
            speech_cmd = jsonRec['text']
            
              
            print("Kommando: " + speech_cmd)
            
            #PrÃƒÂ¼fen, ob die Spracheingabe in der Textdatei "name" ist
            #Inhalt der name.txt wird geleert
            if speech_cmd in namneu_woerter:
                    play(accepted)
                    n = open('/home/team42/Desktop/name.txt','w')
                    n.write('')
                    n.close()
                    time.sleep(1)
                    esng.say("Hallo, wie kann ich dich nennen?")
                    name()
            #Starte mit der Sprachaufnahme(Sprachmemo)        
            if speech_cmd in notauf_woerter:
                    play(accepted)
                    notiz = open('/home/team42/Desktop/neue_notiz.txt','w')
                    notiz.write('')
                    notiz.close()
                    time.sleep(1)
                    esng.say("Starte mit der Notiz?")
                    notiz_fun()
            #Abspielen der Sprachaufnahme(Sprachmemo)
            if speech_cmd in notab_woerter and len(notizvor) != 0:
                esng.voice = 'german'
                nachricht="".join(notizvor)
                esng.say("Notiz wird vorgelesen")
                test=0
                time.sleep(3)
                esng.say(nachricht)
                time.sleep(3)
                if test == 0:
                    esng.say("Soll die Notiz gelöscht werden?")
                    time.sleep(1)
                    loeschen()
            #LÃ¶schen der Sprachaufnahme(Sprachmemo)
            if speech_cmd in notloe_woerter and len(notizvor) != 0:
                play(accepted)
                notiz = open('/home/team42/Desktop/neue_notiz.txt','w')
                notiz.write('')
                notiz.close()
                esng.say("Notiz wurde gelöscht")    

            if len(notizvor) != 0:
                    play(notizsound)
            
            if speech_cmd in objekteingabe_woerter:
                play(accepted)
                esng.say("Nach was soll gesucht werden?")
                obj_suche()
                
            if speech_cmd in geschichte_woerter:
                play(accepted)
                time.sleep(1)
                geschichte() 
                
            if speech_cmd in vorwaertsfahren_woerter:
                play(accepted)
                esng.say("Es wird vorwärts gefahren")
                enable_motors()
                vorwaertsfahren()
            
            if speech_cmd in linksfahren_woerter:
                play(accepted)
                esng.say("Es wird nach links gefahren")
                enable_motors()
                move_left(10)
                
            if speech_cmd in rechtsfahren_woerter:
                play(accepted)
                esng.say("Es wird nach rechts gefahren")
                enable_motors()
                move_right(10)
                
            if speech_cmd in rueckwaertsfahren_woerter:
                play(accepted)
                esng.say("Es wird rückwärts gefahren")
                enable_motors()
                move_backward(10)

            if speech_cmd in schnell_vorwaertsfahren_woerter:
                play(accepted)
                esng.say("Ich gebe Gas")
                enable_motors()
                schnellvorwaertsfahren()
                
            if speech_cmd in routing_woerter:
                play(accepted)
                esng.say("Ich komme zu ihnen")
                enable_motors()
                routing()
                
            if speech_cmd in random_woerter:
                play(accepted)
                esng.say("Ich fahre rum")
                enable_motors()
                random()

            if speech_cmd == "abbrechen":
                    play(accepted)
                    esng.say("Der Vorgang wurde gestoppt")
                    hard_stop()
                    disable_motors()
                    time.sleep(1.5)
                    esng.say(ausgabe)
                    cap.release()
                    cv2.destroyAllWindows()
                    try:
                        thread1.join()
                        distance_thread.join()
                        motor_thread.join()
                        motor1_thread.join()
                        motor2_thread.join()
                        #hard_stop()
                        disable_motors()
                        sprachbefehle()
                    except:
                        sprachbefehle()
                        

            # Timer zÃƒÂ¤hlt runter, wenn keine Aktion erfolgt
            if speech_cmd =="":
                time_out -= 1  
            #Wenn Timer auf 1 ist oder "beenden" ausgefÃƒÂ¼hrt wird, wird die Funktion beendet  
            if time_out == 1 or speech_cmd == 'beenden':
                    
                    esng.voice = 'german'
                    meinname="".join(vor)                       
                    esng.say('Auf Wiedersehen' + '' + meinname)
                    #time.sleep(1)
                    play(declined)
                    try:
                        thread1.join()
                        hard_stop()
                        disable_motors()
                        cap.release()
                        cv2.destroyAllWindows()
                        distance_thread.join()
                        motor_thread.join()
                        motor1_thread.join()
                        motor2_thread.join()
                        standard()
                    except:
                        #Aufrufen der Funktion standard fÃƒÂ¼r den Standard-Loop
                        standard()
                
        else:
            print(rec.PartialResult())
        if dump_fn is not None:
            dump_fn.write(data)
##########################################################################

def geschichte():
    esng.say("Geschichte wird jetzt vorgelesen")
    thread1 = threading.Thread(target=sprachbefehle)
    play(geschichteerzaehlen)
            
#########################Motorsteuerung####################################

# Modus auf SpreadCycle setzen für beide Motoren
def set_spreadcycle_mode():
    m1_1.on()  # Setzt M1 des ersten Motors auf HIGH
    m2_1.off()  # Setzt M2 des ersten Motors auf LOW
    m1_2.on()  # Setzt M1 des zweiten Motors auf HIGH
    m2_2.off()  # Setzt M2 des zweiten Motors auf LOW

# Motoren aktivieren
def enable_motors():
    enable1.off()  # Aktiviert den Treiber, wenn EN auf LOW ist (abhängig vom Treiber)
    enable2.off()  # Aktiviert den Treiber, wenn EN auf LOW ist (abhängig vom Treiber)

# Motoren deaktivieren
def disable_motors():
    enable1.on()  # Deaktiviert den Treiber, wenn EN auf HIGH ist (abhängig vom Treiber)
    enable2.on()  # Deaktiviert den Treiber, wenn EN auf HIGH ist (abhängig vom Treiber)
    sprachbefehle()

# Vorwärtsfahren
def move_forward(duration):

    direction1.off()  # Setzt die Richtung für den ersten Motor auf vorwärts

    direction2.on()  # Setzt die Richtung für den zweiten Motor auf vorwärts

    start_time = time()
    initial_sleep = 0.007  # Startwert für den sleep
    final_sleep = 0.001  # Endwert für den sleep
    acceleration_time = 7  # Zeit in Sekunden, um die maximale Geschwindigkeit zu erreichen

    while time() - start_time < duration:

        elapsed_time = time() - start_time

        # Berechne den aktuellen sleep-Wert basierend auf der verstrichenen Zeit
        if elapsed_time < acceleration_time:
            current_sleep = initial_sleep - (initial_sleep - final_sleep) * (elapsed_time / acceleration_time)
        else:
            current_sleep = final_sleep

        step1.on()
        step2.on()
        sleep(current_sleep)
        step1.off()
        step2.off()
        sleep(current_sleep)

# Vorwärtsfahren schnell
def move_forward_fast(duration):
    direction1.off()  # Setzt die Richtung für den ersten Motor auf vorwärts
    direction2.on()  # Setzt die Richtung für den zweiten Motor auf vorwärts
    
    start_time = time()
    initial_sleep = 0.003  # Startwert für den sleep
    final_sleep = 0.0001  # Endwert für den sleep
    acceleration_time = 5  # Zeit in Sekunden, um die maximale Geschwindigkeit zu erreichen

    while time() - start_time < duration:
        elapsed_time = time() - start_time
        # Berechne den aktuellen sleep-Wert basierend auf der verstrichenen Zeit
        if elapsed_time < acceleration_time:
            current_sleep = initial_sleep - ((initial_sleep - final_sleep) * (elapsed_time / acceleration_time))
        else:
            current_sleep = final_sleep
        
        step1.on()
        step2.on()
        sleep(current_sleep)
        step1.off()
        step2.off()
        sleep(current_sleep)

def control_routing():
    move_forward(5)
    sleep(1)
    move_right(10)
    sleep(1)
    move_forward(5)
    sleep(1)
    move_left(10)
    sleep(1)
    move_forward(5)
    hard_stop()

#routing
def routing():
    thread1 = threading.Thread(target=sprachbefehle)
    thread1.start()
    distance_thread = threading.Thread(target=monitor_distance)
    distance_thread.start()
    thread2 = threading.Thread(target=control_routing)
    thread2.start()
    distance_thread.join()
    thread2.join()

# Rückwärtsfahren
def move_backward(duration):
    thread1 = threading.Thread(target=sprachbefehle)
    thread1.start()
    direction1.on()  # Setzt die Richtung für den ersten Motor auf vorwärts
    direction2.off()  # Setzt die Richtung für den zweiten Motor auf vorwärts
    
    start_time = time()
    initial_sleep = 0.005  # Startwert für den sleep
    final_sleep = 0.0005  # Endwert für den sleep
    acceleration_time = 10  # Zeit in Sekunden, um die maximale Geschwindigkeit zu erreichen

    while time() - start_time < duration:
        elapsed_time = time() - start_time
        
        # Berechne den aktuellen sleep-Wert basierend auf der verstrichenen Zeit
        if elapsed_time < acceleration_time:
            current_sleep = initial_sleep - (initial_sleep - final_sleep) * (elapsed_time / acceleration_time)
        else:
            current_sleep = final_sleep
        
        step1.on()
        step2.on()
        sleep(current_sleep)
        step1.off()
        step2.off()
        sleep(current_sleep)

# Funktion zur Steuerung des ersten Motors für Linksbewegung
def control_motor1_left(duration):
    direction1.off()  # Setzt die Richtung für den ersten Motor auf vorwärts
    start_time = time()
    initial_sleep = 0.007  # Startwert für den sleep
    final_sleep = 0.001  # Endwert für den sleep
    acceleration_time = 7  # Zeit in Sekunden, um die maximale Geschwindigkeit zu erreichen

    while time() - start_time < duration:
        elapsed_time = time() - start_time
        if elapsed_time < acceleration_time:
            current_sleep_motor2 = initial_sleep - (initial_sleep - final_sleep) * (elapsed_time / acceleration_time)
        else:
            current_sleep_motor2 = final_sleep

        step1.on()
        sleep(current_sleep_motor2)
        step1.off()
        sleep(current_sleep_motor2)

# Funktion zur Steuerung des zweiten Motors für Linksbewegung
def control_motor2_left(duration):
    direction2.on()  # Setzt die Richtung für den zweiten Motor auf vorwärts
    start_time = time()
    current_sleep_motor1 = 0.007  # Konstanter Schrittintervall für den Motor 1

    while time() - start_time < duration:
        step2.on()
        sleep(current_sleep_motor1)
        step2.off()
        sleep(current_sleep_motor1)

# Funktion zur Steuerung des ersten Motors für Rechtsbewegung
def control_motor1_right(duration):
    direction1.off()  # Setzt die Richtung für den ersten Motor auf rückwärts
    start_time = time()
    current_sleep_motor1 = 0.007  # Konstanter Schrittintervall für den Motor 1

    while time() - start_time < duration:
        step1.on()
        sleep(current_sleep_motor1)
        step1.off()
        sleep(current_sleep_motor1)

# Funktion zur Steuerung des zweiten Motors für Rechtsbewegung
def control_motor2_right(duration):
    direction2.on()  # Setzt die Richtung für den zweiten Motor auf rückwärts
    start_time = time()
    initial_sleep = 0.007  # Startwert für den sleep
    final_sleep = 0.001  # Endwert für den sleep
    acceleration_time = 7  # Zeit in Sekunden, um die maximale Geschwindigkeit zu erreichen

    while time() - start_time < duration:
        elapsed_time = time() - start_time
        if elapsed_time < acceleration_time:
            current_sleep_motor2 = initial_sleep - (initial_sleep - final_sleep) * (elapsed_time / acceleration_time)
        else:
            current_sleep_motor2 = final_sleep

        step2.on()
        sleep(current_sleep_motor2)
        step2.off()
        sleep(current_sleep_motor2)

# Hauptfunktion zur Steuerung beider Motoren für Linksbewegung
def move_left(duration):
    thread1 = threading.Thread(target=sprachbefehle)
    thread1.start()
    motor1_thread = threading.Thread(target=control_motor1_left, args=(duration,))
    motor2_thread = threading.Thread(target=control_motor2_left, args=(duration,))

    motor1_thread.start()
    motor2_thread.start()

    motor1_thread.join()
    motor2_thread.join()

# Hauptfunktion zur Steuerung beider Motoren für Rechtsbewegung
def move_right(duration):
    thread1 = threading.Thread(target=sprachbefehle)
    thread1.start()
    motor1_thread = threading.Thread(target=control_motor1_right, args=(duration,))
    motor2_thread = threading.Thread(target=control_motor2_right, args=(duration,))

    motor1_thread.start()
    motor2_thread.start()

    motor1_thread.join()
    motor2_thread.join()

# Hardstopp
def hard_stop():
    thread1 = threading.Thread(target=sprachbefehle)
    thread1.start()
    step1.on()
    step2.on()
    sleep(0.1)
    step1.off()
    step2.off()
    sleep(0.1)

# Softstopp
def soft_stop():
    step1.off()
    step2.off()
    sleep(0.1)
    disable_motors()

def random(duration):
        thread1 = threading.Thread(target=sprachbefehle)
        thread1.start()
        enable_motors()

        while sensor.distance * 100 <= 50:
            # Vorwärts fahren und Abstand überwachen
            distance_thread = threading.Thread(target=monitor_distance)
            distance_thread.start()
            move_forward(10000)
            distance_thread.join()
            
        # Wenn ein Objekt erkannt wurde
        if sensor.distance * 100 <= 50:
            hard_stop()
            disabel_motors()
            sleep(1)
            enable_motors()
            # Rückwärts fahren
            move_backward(10)
            
            # Rechts drehen
            move_right(3)

# Funktion zur Überwachung der Distanz
def monitor_distance():
    while True:
        distance = sensor.distance * 100  # Distanz in cm
        print('Distance: {:.2f} cm'.format(distance))
        if distance <= 50:
            print('Object detected within 50 cm! Hard stop initiated.')
            hard_stop()
            disable_motors()
            break
        sleep(0.1)

# Funktion zur Steuerung der Motoren
def control_motors():
    enable_motors()
    move_forward(15)
    sleep(2)
    disable_motors()

# Funktion zur Steuerung der Motoren
def control_motors_fast():
    enable_motors()
    move_forward_fast(10)
    sleep(2)
    disable_motors()

def schnellvorwaertsfahren():
    set_spreadcycle_mode()
    thread1 = threading.Thread(target=sprachbefehle)
    thread1.start()
    # Erstellen und Starten der Threads
    distance_thread = threading.Thread(target=monitor_distance)
    motor_thread = threading.Thread(target=control_motors_fast)
    
    distance_thread.start()
    motor_thread.start()
    
    # Warten, bis beide Threads beendet sind
    distance_thread.join()
    motor_thread.join()   

def vorwaertsfahren():
    set_spreadcycle_mode()
    thread1 = threading.Thread(target=sprachbefehle)
    thread1.start()
    # Erstellen und Starten der Threads
    distance_thread = threading.Thread(target=monitor_distance)
    motor_thread = threading.Thread(target=control_motors)
    
    distance_thread.start()
    motor_thread.start()
    
    # Warten, bis beide Threads beendet sind
    distance_thread.join()
    motor_thread.join()
            
#########################OpenCV####################################
from ultralytics import YOLO
import cv2
import math 
# start opencvfunc
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# model
model1 = YOLO("yolo-Weights/yolov8n.pt")

# object classes
classNames = ["person", "fahrrad", "auto", "motorrad", "flugzeug", "bus", "zug", "lkw", "boot",
              "ampel", "hydrant", "stoppschild", "parking meter", "bank", "vogel", "katze",
              "hund", "pferd", "schaf", "kuh", "elefant", "bär", "zebra", "giraffe", "rucksack", "regenschirm",
              "handtasche", "krawatte", "koffer", "frisbee", "skier", "snowboard", "ball", "drachen", "baseballschläger",
              "baseballhandschuh", "skateboard", "surfboard", "tennisschläger", "flasche", "weinglas", "becher",
              "gabel", "messer", "löffel", "schüssel", "banane", "apfel", "sandwich", "orange", "brokkoli",
              "karotte", "hot dog", "pizza", "donut", "kuchen", "stuhl", "couch", "topfpflanze", "bett",
              "tisch", "toilette", "fernseher", "laptop", "maus", "fernbedienung", "tastatur", "smartphone",
              "mikrowelle", "ofen", "toaster", "waschbecken", "kühlschrank", "buch", "uhr", "vase", "schere",
              "teddybär", "haartrockner", "zahnbürste"
              ]
def obj_suche():
    
    objektsuche=''
    counter = 4
    try:
        thread1.join()
    except:
        print ("Alles in Ordnung")
        
    while True:
        data = q.get()    
        if rec.AcceptWaveform(data):
            jsonRec = json.loads(rec.Result())
            speech_cmd = jsonRec['text']
            counter += 1 
            if objektsuche =='' and counter > 5:
                    objekt1 = open('/home/team42/Desktop/objektsuche.txt','w')
                    objekt1.write(speech_cmd)
                    objekt1.close()
                    play(accepted)
                    objektsuche1 = open('/home/team42/Desktop/objektsuche.txt','r').read()
                    objektvor= objektsuche1.splitlines()
                    objektsuche="".join(objektvor)                  
                    esng.say('Es wird nach'+ objektsuche + "gesucht")
                    #time.sleep(2)
                    opencvfunc()

        else:
            print(rec.PartialResult())
        if dump_fn is not None:
            dump_fn.write(data)


    #while True:
        #thread1 = threading.Thread(target=stopfunktion)
        #thread2 = threading.Thread(target=opencvfunc)
        
        #thread1.start()
        #thread2.start()
        #thread1.join()
        #thread2.join()

#def stopfunktion():
#    while True:
#        data = q.get()    
#        if rec.AcceptWaveform(data):
#            jsonRec = json.loads(rec.Result())
#            speech_cmd = jsonRec['text']
#            if speech_cmd == "abbrechen":
#                    play(accepted)
#                    hard_stop()
#                    esng.say("Der Vorgang wurde gestoppt")
#                    time.sleep(2)
#                    esng.say(ausgabe)
#                    cap.release()
#                    cv2.destroyAllWindows()
#                    #thread1.join()
#                    #thread2.join()
#                    time.sleep(1)
#                    sprachbefehle()
 
        else:
            print(rec.PartialResult())


def opencvfunc():
    objektsuche1 = open('/home/team42/Desktop/objektsuche.txt','r').read()
    objektvor= objektsuche1.splitlines()
    objektsuche="".join(objektvor)
    thread1 = threading.Thread(target=sprachbefehle)
    thread1.start()
    #thread2 = threading.Thread(target=opencvfunc)
    #thread2.start()
    try:
        while True:
            #thread2 = threading.Thread(target=opencvfunc)
            #thread1 = threading.Thread(target=stopfunktion)
            
            success, img = cap.read()
            results = model1(img, stream=True)

            # coordinates
            for r in results:
                boxes = r.boxes

                for box in boxes:
                    # bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                    # put box in cam
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                    # confidence
                    confidence = math.ceil((box.conf[0]*100))/100
                    print("Confidence --->",confidence)

                    # class name
                    cls = int(box.cls[0])
                    print("Class name -->", classNames[cls])
                    objektsuche1 = open('/home/team42/Desktop/objektsuche.txt','r').read()
                    objektvor= objektsuche1.splitlines()
                    objektsuche="".join(objektvor)
                    print(objektvor)
                    if objektsuche == classNames[cls]:
                            play(accepted)
                            esng.say("Es wurde das Objekt" + objektsuche + "gefunden")
                            time.sleep(3)
                            esng.say(ausgabe2)
                            cap.release()
                            cv2.destroyAllWindows()
                            #thread1.join()
                            #thread2.join()
                            sprachbefehle()
                            # object details
                    org = [x1, y1]
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale = 1
                    color = (255, 0, 0)
                    thickness = 2
                    
                    cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)
                    

            cv2.imshow('Webcam', img)
            if cv2.waitKey(1) == ord('q'):
                break
    
        cap.release()
        cv2.destroyAllWindows()
        
    except:
        #sprachbefehle()
        print('interner Fehler')
        

#thread2 = threading.Thread(target=opencvfunc)
#thread1 = threading.Thread(target=stopfunktion)

##############################################################
#Notizfunktion wird aufgerufen
def notiz_fun():
    
    notiztext=''
    counter = 4
    while True:
        data = q.get()    
        if rec.AcceptWaveform(data):
            jsonRec = json.loads(rec.Result())
            speech_cmd = jsonRec['text']
            counter += 1 
            if notiztext =='' and counter > 5:
                    notiz1 = open('/home/team42/Desktop/neue_notiz.txt','w')
                    notiz1.write(speech_cmd)
                    notiz1.close()
                    #play(accepted)
                    notiztext = notizvorhanden = open('/home/team42/Desktop/neue_notiz.txt','r').read()
                    notizvor= notizvorhanden.splitlines()
                    notiztext = notizvor
                    play(accepted)
                    esng.say('Notiz erfolgreich gespeichert')
                    time.sleep(2)
                    esng.say(ausgabe)
                    sprachbefehle()  

        else:
            print(rec.PartialResult())
        if dump_fn is not None:
            dump_fn.write(data)
#Notiz lÃ¶schen-Funktion wird aufgerufen
def loeschen():
    counter = 4
    while True:
        data = q.get()    
        if rec.AcceptWaveform(data):
            jsonRec = json.loads(rec.Result())
            speech_cmd = jsonRec['text']
            counter += 1 
            if speech_cmd =='ja' and counter > 5:
                play(accepted)
                notiz = open('/home/team42/Desktop/neue_notiz.txt','w')
                notiz.write('')
                notiz.close()
                esng.say("Notiz wurde gelöscht")
                time.sleep(2)
                esng.say(ausgabe)
                time.sleep(0.5)  
                sprachbefehle()
            if speech_cmd =='nein' and counter > 5:
                play(accepted)
                esng.say(ausgabe)
                time.sleep(0.5)
                sprachbefehle() 

        else:
            print(rec.PartialResult())
        if dump_fn is not None:
            dump_fn.write(data)
#Neuer Name-Funktion wird aufgerufen
def name():
    meinname = ''
    
    counter = 4
    while True:
        data = q.get()    
        if rec.AcceptWaveform(data):
            jsonRec = json.loads(rec.Result())
            speech_cmd = jsonRec['text']
            counter += 1

            if meinname == '' and counter > 5:  
                    n = open('/home/team42/Desktop/name.txt','w')
                    n.write(speech_cmd)
                    n.close()
                    play(accepted)
                    vorlesen = open('/home/team42/Desktop/name.txt','r').read()
                    vor= vorlesen.splitlines()
                    esng.voice = 'german'
                    meinname="".join(vor)                         
                    esng.say(begruessung + meinname + ausgabe)
                    sprachbefehle()     

        else:
            print(rec.PartialResult())
        if dump_fn is not None:
            dump_fn.write(data)
#Standardschleife wird aufgerufen (Weitetleitung zur Sprachbefehle-Funktion)

def standard():
    
    vorlesen = open('/home/team42/Desktop/name.txt','r').read()
    vor= vorlesen.splitlines()
    while True:
            
        data = q.get()    
        if rec.AcceptWaveform(data):
            jsonRec = json.loads(rec.Result())
            speech_cmd = jsonRec['text']
            

            if speech_cmd in helpfix and len(vor) == 0:
                play(accepted)                    
                esng.say("Hallo, wie kann ich dich nennen?")
                #vorlesen.close()
                name()
                print("Kommando: " + speech_cmd)
                
            if speech_cmd in helpfix and len(vor) != 0:
                play(accepted)
                esng.voice = 'german'
                meinname="".join(vor)                       
                esng.say(begruessung2 + meinname + ausgabe2)
                #vorlesen.close()
                sprachbefehle()
            print("Kommando: " + speech_cmd)

        else:
            print(rec.PartialResult())
        if dump_fn is not None:
            dump_fn.write(data)

try:
    if args.samplerate is None:
        device_info = sd.query_devices(args.device, "input")
        # soundfile expects an int, sounddevice provides a float:
        args.samplerate = int(device_info["default_samplerate"])
        
    if args.model is None:
        model = Model(lang="de")
    else:
        model = Model(lang=args.model)

    if args.filename:
        dump_fn = open(args.filename, "wb")
    else:
        dump_fn = None
#Standardloop wird aufgerufen
    with sd.RawInputStream(samplerate=args.samplerate, blocksize = 8000, device=args.device,
            dtype="int16", channels=1, callback=callback):
        print("#" * 80)
        print("Press Ctrl+C to stop the recording")
        print("#" * 80)

        rec = KaldiRecognizer(model, args.samplerate)
        
        play(starting)
        while True:
            vorlesen = open('/home/team42/Desktop/name.txt','r').read()
            vor= vorlesen.splitlines()
            data = q.get()

            if rec.AcceptWaveform(data):
                jsonRec = json.loads(rec.Result())
                speech_cmd = jsonRec['text']

                if speech_cmd in helpfix and len(vor) == 0:
                    play(accepted)
                    esng.say("Hallo, wie kann ich dich nennen?")
                    name()
                print("Kommando: " + speech_cmd)
                
                if speech_cmd in helpfix and len(vor) != 0:
                    play(accepted)
                    esng.voice = 'german'
                    meinname="".join(vor)                       
                    esng.say(begruessung + meinname + ausgabe)
                    sprachbefehle()
                print("Kommando: " + speech_cmd)

            else:
                print(rec.PartialResult())
              

except KeyboardInterrupt:        
    print("\nDone")
    parser.exit(0)
except Exception as e:
    parser.exit(type(e).__name__ + ": " + str(e))





