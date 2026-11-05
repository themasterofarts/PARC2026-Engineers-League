#! /usr/bin/env python3

##### Demarrage vocal hors ligne (bonus +5 points).
#    Ecoute le micro ReSpeaker du robot (/speech_audio, audio_common_msgs/AudioData,
#    16 kHz, 16 bits signes, mono) et reconnait le mot de depart avec Vosk (modele francais).
#    Vocabulaire libre, et le depart n'est accepte que si la phrase entiere est le mot seul
#    ("allez"). Mesure du 05/11 sur 60 s de parole sans le mot : une grammaire limitee a
#    go/allez/vai donnait 10 faux departs ("allons", "alors"...), cette regle 0.
#    Publie le mot reconnu sur /ma64/start_vocal : le noeud de mission publie alors le
#    StartSignal avec method = METHOD_VOICE et heard = le mot. Le depart par commande
#    (/ma64/start) reste possible en secours.
#    /speech_audio arrive en un message par phrase (simulateur officiel) ou par blocs :
#    la phrase est terminee quand plus rien n'arrive pendant fin_phrase_s.
#    Test sans robot : parametre wav_file (WAV 16 bits mono 16 kHz).

import json
import os
import time
import wave

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

try:
    from vosk import KaldiRecognizer, Model, SetLogLevel
except ImportError:
    Model = None


class DemarrageVocal(Node):

    def __init__(self):
        super().__init__('demarrage_vocal')
        p = self.declare_parameter
        self.words = [w.lower() for w in p('words', ['allez']).value]
        model_path = os.path.expanduser(p('model_path', '~/MA64/vosk-model').value)
        self.rate = p('sample_rate', 16000).value
        self.channels = p('channels', 1).value
        self.channel = p('channel', 0).value       # canal garde si l'audio est multicanal
        audio_topic = p('audio_topic', '/speech_audio').value
        wav_file = p('wav_file', '').value
        self.min_conf = p('min_confidence', 0.6).value
        self.fin_phrase = p('fin_phrase_s', 0.4).value

        self.pub = self.create_publisher(String, '/ma64/start_vocal', 10)
        self.en_cours = False      # audio recu depuis la derniere fin de phrase
        self.dernier = 0.0
        if Model is None:
            self.get_logger().error('module vosk absent : pip install vosk (demarrage vocal desactive)')
            return
        if not os.path.isdir(model_path):
            self.get_logger().error(f'modele Vosk introuvable : {model_path}')
            return
        SetLogLevel(-1)
        self.model = Model(model_path)
        self.rec = self.nouveau_recognizer()
        self.get_logger().info(f'ecoute de {audio_topic} : mots {self.words}, {self.rate} Hz, '
                               f'{self.channels} canal(aux), confiance min {self.min_conf}')
        if wav_file:
            self.lire_wav(wav_file)
            return
        # best effort : compatible avec un publieur fiable ou non
        self.create_subscription(get_message('audio_common_msgs/msg/AudioData'), audio_topic,
                                 self.on_audio, qos_profile_sensor_data)
        self.create_timer(0.1, self.verifie_fin_phrase)

    def nouveau_recognizer(self):
        # vocabulaire libre : une grammaire limitee force les mots proches sur le mot de depart
        rec = KaldiRecognizer(self.model, self.rate)
        rec.SetWords(True)
        return rec

    def on_audio(self, msg):
        data = bytes(msg.data)
        if self.channels > 1:
            s = np.frombuffer(data[:len(data) // (2 * self.channels) * 2 * self.channels], dtype=np.int16)
            data = s.reshape(-1, self.channels)[:, self.channel].copy().tobytes()
        if self.rec.AcceptWaveform(data):
            self.resultat(json.loads(self.rec.Result()))
        self.en_cours = True
        self.dernier = time.monotonic()

    def verifie_fin_phrase(self):
        if self.en_cours and time.monotonic() - self.dernier > self.fin_phrase:
            self.en_cours = False
            self.resultat(json.loads(self.rec.FinalResult()))
            self.rec = self.nouveau_recognizer()

    def lire_wav(self, chemin):
        with wave.open(chemin, 'rb') as w:
            while True:
                bloc = w.readframes(4000)
                if not bloc:
                    break
                if self.rec.AcceptWaveform(bloc):
                    self.resultat(json.loads(self.rec.Result()))
        self.resultat(json.loads(self.rec.FinalResult()))

    def resultat(self, res):
        texte = res.get('text', '')
        if not texte:
            return
        self.get_logger().info(f'entendu : "{texte}"')
        # la phrase entiere doit etre le mot seul ("allons-y", "alors allez" ne comptent pas)
        if texte not in self.words:
            return
        conf = min((m.get('conf', 0.0) for m in res.get('result', [])), default=0.0)
        if conf >= self.min_conf:
            self.get_logger().info(f'DEPART VOCAL : "{texte}" (confiance {conf:.2f})')
            self.pub.publish(String(data=texte))
        else:
            self.get_logger().info(f'"{texte}" ignore : confiance {conf:.2f} < {self.min_conf}')


def main():
    rclpy.init()
    node = DemarrageVocal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
