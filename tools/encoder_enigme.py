#!/usr/bin/env python3
"""
Encodeur de l'énigme audio : 3 couches de brouillage.

  1. TONALITÉ   : aigus montés de TONALITE_DB dB, graves baissés d'autant
                  → le joueur corrige avec le joystick X
  2. INVERSION  : chaque fréquence f devient F_CLE - f (graves ↔ aigus)
                  → le joueur trouve la clé avec le potentiomètre
  3. VITESSE    : le son est accéléré ×VITESSE (plus rapide ET plus aigu)
                  → le joueur ralentit avec le joystick Y

Le décodeur (ESP32) doit défaire les couches dans l'ordre inverse :
  vitesse (Y) → inversion (potentiomètre) → tonalité (X)

Bonnes valeurs côté décodeur :
  - vitesse de lecture = 1 / VITESSE   (ex. 1 / 1.3 ≈ 0.77)
  - porteuse           = F_CLE         (ex. 3300 Hz)
  - tonalité           = -TONALITE_DB  (ex. -12 dB : graves +12, aigus -12)

Utilisation :
  pip install numpy scipy
  (ffmpeg doit être installé et dans le PATH pour les formats autres que WAV)
  python encoder_enigme.py ma_voix.mp3
  python encoder_enigme.py ma_voix.mp3 3300 1.3 12      (clé, vitesse, tonalité)
  → crée 0001.wav à copier à la racine de la carte SD du DFPlayer.

N'importe quel format lisible par ffmpeg fonctionne (mp3, m4a, ogg, flac, wav, ...).
"""

import os
import subprocess
import sys
import tempfile
from fractions import Fraction
from math import gcd
import numpy as np
from scipy.io import wavfile
from scipy.signal import butter, sosfiltfilt, sosfilt, resample_poly

# ---------- Réglages de l'énigme ----------
F_CLE       = 3300.0   # fréquence secrète (Hz), > 3100
VITESSE     = 1.3      # le message est joué 1,3× trop vite (évite 1.0 : ce serait la position de départ)
TONALITE_DB = 12.0     # déséquilibre aigus/graves (dB)

F_BAS, F_HAUT = 300, 3000   # bande de la voix conservée
F_GRAVES = 600              # fréquence du filtre des graves (shelf)
F_AIGUS  = 1500             # fréquence du filtre des aigus (shelf)
FS = 44100                  # fréquence du fichier pour le DFPlayer


def convertir_en_wav_temp(chemin, fs):
    """Convertit n'importe quel format lisible par ffmpeg (mp3, m4a, ogg, flac, ...) en WAV mono temporaire."""
    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    tmp.close()
    try:
        subprocess.run(
            ["ffmpeg", "-y", "-i", chemin, "-ac", "1", "-ar", str(fs), tmp.name],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        os.remove(tmp.name)
        raise RuntimeError(
            "ffmpeg n'est pas installé ou pas dans le PATH (nécessaire pour lire les formats autres que WAV)."
        )
    except subprocess.CalledProcessError:
        os.remove(tmp.name)
        raise RuntimeError(f"ffmpeg n'a pas pu convertir '{chemin}' — fichier corrompu ou format non supporté.")
    return tmp.name


def charger(chemin):
    ext = os.path.splitext(chemin)[1].lower()
    tmp_wav = None
    if ext != ".wav":
        tmp_wav = convertir_en_wav_temp(chemin, FS)
        chemin_lecture = tmp_wav
    else:
        chemin_lecture = chemin

    try:
        fs, x = wavfile.read(chemin_lecture)
        if x.dtype == np.uint8:
            x = (x.astype(np.float64) - 128) / 128
        elif np.issubdtype(x.dtype, np.integer):
            x = x.astype(np.float64) / np.iinfo(x.dtype).max
        else:
            x = x.astype(np.float64)
        if x.ndim == 2:
            x = x.mean(axis=1)                      # stéréo → mono
        if fs != FS:
            g = gcd(fs, FS)
            x = resample_poly(x, FS // g, fs // g)  # → 44 100 Hz
    finally:
        if tmp_wav:
            os.remove(tmp_wav)

    return x


def shelf(type_, f0, gain_db, fs):
    """Filtre en plateau (formules RBJ), en format SOS. type_ = 'bas' ou 'haut'."""
    A = 10 ** (gain_db / 40)
    w = 2 * np.pi * f0 / fs
    c, s = np.cos(w), np.sin(w)
    alpha = s / 2 * np.sqrt(2)                  # pente S = 1
    k = 2 * np.sqrt(A) * alpha
    if type_ == "bas":
        b = [A * ((A + 1) - (A - 1) * c + k), 2 * A * ((A - 1) - (A + 1) * c), A * ((A + 1) - (A - 1) * c - k)]
        a = [(A + 1) + (A - 1) * c + k, -2 * ((A - 1) + (A + 1) * c), (A + 1) + (A - 1) * c - k]
    else:
        b = [A * ((A + 1) + (A - 1) * c + k), -2 * A * ((A - 1) + (A + 1) * c), A * ((A + 1) + (A - 1) * c - k)]
        a = [(A + 1) - (A - 1) * c + k, 2 * ((A - 1) - (A + 1) * c), (A + 1) - (A - 1) * c - k]
    b, a = np.array(b) / a[0], np.array(a) / a[0]
    return np.concatenate([b, a])[None, :]


def tonalite(x, db, fs):
    """db > 0 : aigus +db et graves -db.  db < 0 : l'inverse (c'est ce que fait le joystick X)."""
    x = sosfilt(shelf("bas", F_GRAVES, -db, fs), x)
    x = sosfilt(shelf("haut", F_AIGUS, +db, fs), x)
    return x


def inversion(x, f_cle, fs):
    """Graves ↔ aigus : f devient f_cle - f. Même opération pour encoder et décoder."""
    n = np.arange(len(x))
    y = 2 * x * np.cos(2 * np.pi * f_cle * n / fs)
    lo, hi = max(f_cle - F_HAUT, 50), f_cle - F_BAS
    return sosfiltfilt(butter(8, [lo, hi], btype="bandpass", fs=fs, output="sos"), y)


def changer_vitesse(x, vitesse):
    """Comme une cassette : plus rapide = plus court et plus aigu."""
    r = Fraction(1 / vitesse).limit_denominator(200)
    return resample_poly(x, r.numerator, r.denominator)


def encoder(x, fs, f_cle, vitesse, ton_db):
    if f_cle - F_HAUT < 100:
        raise ValueError("La clé doit être au moins 100 Hz au-dessus de 3000 Hz")
    x = sosfiltfilt(butter(8, [F_BAS, F_HAUT], btype="bandpass", fs=fs, output="sos"), x)
    x = x / (np.max(np.abs(x)) + 1e-12)
    x = tonalite(x, ton_db, fs)           # 1. tonalité
    x = inversion(x, f_cle, fs)           # 2. inversion
    x = changer_vitesse(x, vitesse)       # 3. vitesse
    return 0.89 * x / (np.max(np.abs(x)) + 1e-12)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    entree = sys.argv[1]
    f_cle  = float(sys.argv[2]) if len(sys.argv) > 2 else F_CLE
    vit    = float(sys.argv[3]) if len(sys.argv) > 3 else VITESSE
    ton    = float(sys.argv[4]) if len(sys.argv) > 4 else TONALITE_DB

    y = encoder(charger(entree), FS, f_cle, vit, ton)
    wavfile.write("0001.wav", FS, (y * 32767).astype(np.int16))
    print(f"0001.wav créé ({len(y) / FS:.1f} s)")
    print("Réponses de l'énigme (côté décodeur) :")
    print(f"  vitesse   (joystick Y)     : {1 / vit:.3f}x")
    print(f"  clé       (potentiomètre)  : {f_cle:.0f} Hz")
    print(f"  tonalité  (joystick X)     : {-ton:+.0f} dB")


if __name__ == "__main__":
    main()
