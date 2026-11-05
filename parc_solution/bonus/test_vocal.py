import os
import time
import sounddevice as sd
import openwakeword
from openwakeword.model import Model

dossier = os.path.join(os.path.dirname(openwakeword.__file__), "resources", "models")
chemin = os.path.join(dossier, "hey_jarvis_v0.1.onnx")

model = Model(wakeword_model_paths=[chemin])

SEUIL = 0.4
dernier = 0

print("Dis 'hey jarvis' (Ctrl+C pour arrêter)")
with sd.InputStream(samplerate=16000, channels=1, dtype="int16", blocksize=1280) as stream:
    while True:
        audio, _ = stream.read(1280)
        scores = model.predict(audio.flatten())
        nom = list(scores.keys())[0]
        score = scores[nom]
        print(f"{nom}: {score:.2f}", end="\r")
        if score > SEUIL and time.time() - dernier > 2:
            print(f"\nDétecté ! (score {score:.2f})")
            dernier = time.time()
