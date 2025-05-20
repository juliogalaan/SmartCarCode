import os
import torch
import torchaudio
import sounddevice as sd
import numpy as np
import librosa
from scipy.io.wavfile import write
import tensorflow as tf
import time
import pickle

# === CONFIGURACIÓN ===
SAMPLE_RATE = 16000
DURATION = 5
AUDIO_FILENAME = "voz_detectada.wav"
EXPECTED_CLASSES = [np.str_('backward'), np.str_('faster'), np.str_('go'), np.str_('left'), np.str_('off'), np.str_('right'), np.str_('slower'), np.str_('stop')]

# === CARGAR LABEL ENCODER desde misma carpeta del script ===
script_dir = os.path.dirname(os.path.abspath(__file__))
encoder_path = os.path.join(script_dir, "label_encoder.pkl")

if not os.path.exists(encoder_path):
    raise FileNotFoundError(f"❌ No se encontró label_encoder.pkl en {script_dir}")

with open(encoder_path, "rb") as f:
    le = pickle.load(f)

if list(le.classes_) != EXPECTED_CLASSES:
    raise ValueError(f"❌ El label encoder no coincide con las clases esperadas.\n"
                     f"Esperado: {EXPECTED_CLASSES}\n"
                     f"Encontrado: {list(le.classes_)}")

CLASSES = EXPECTED_CLASSES

# === CARGAR MODELO VAD ===
model_vad, utils = torch.hub.load('snakers4/silero-vad', 'silero_vad', force_reload=False)
(get_speech_timestamps, _, _, _, _) = utils

def grabar_con_vad():
    if os.path.exists(AUDIO_FILENAME):
        os.remove(AUDIO_FILENAME)

    print("🎙️ Grabando...")
    audio = sd.rec(int(DURATION * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
    sd.wait()
    print("✅ Grabación finalizada.")

    audio_tensor = torch.from_numpy(audio.squeeze()).float()
    if audio_tensor.ndim == 1:
        audio_tensor = audio_tensor.unsqueeze(0)

    timestamps = get_speech_timestamps(audio_tensor, model_vad, sampling_rate=SAMPLE_RATE)

    if not timestamps:
        print("❌ No se detectó voz.")
        return None

    segmentos = [audio_tensor[0, ts['start']:ts['end']] for ts in timestamps]
    voz_final = torch.cat(segmentos)
    np_audio = voz_final.numpy()

    if len(np_audio) > SAMPLE_RATE:
        centro = len(np_audio) // 2
        inicio = max(0, centro - SAMPLE_RATE // 2)
        final = inicio + SAMPLE_RATE
        np_audio = np_audio[inicio:final]
    elif len(np_audio) < SAMPLE_RATE:
        np_audio = np.pad(np_audio, (0, SAMPLE_RATE - len(np_audio)))

    write(AUDIO_FILENAME, SAMPLE_RATE, np_audio)
    time.sleep(0.1)
    return AUDIO_FILENAME

def extraer_mel_spectrogram(ruta_audio):
    y, sr = librosa.load(ruta_audio, sr=SAMPLE_RATE)
    if len(y) < SAMPLE_RATE:
        y = np.pad(y, (0, SAMPLE_RATE - len(y)))
    else:
        y = y[:SAMPLE_RATE]
    spectrogram = librosa.feature.melspectrogram(y=y, sr=sr, n_mels=128, fmax=8000)
    log_spectrogram = librosa.power_to_db(spectrogram, ref=np.max)
    log_spectrogram = (log_spectrogram - np.mean(log_spectrogram)) / np.std(log_spectrogram)
    log_spectrogram = log_spectrogram[:, :44]
    log_spectrogram = np.pad(log_spectrogram, ((0, 0), (0, 44 - log_spectrogram.shape[1])), mode='constant')
    log_spectrogram = np.stack([log_spectrogram] * 3, axis=-1)
    return log_spectrogram[np.newaxis, ...].astype(np.float32)

def cargar_modelo(path="best_model.keras"):
    if not os.path.exists(path):
        raise FileNotFoundError("❌ No se encontró el modelo '.keras'.")
    return tf.keras.models.load_model(path)

def predecir(audio_path, modelo):
    features = extraer_mel_spectrogram(audio_path)
    predicciones = modelo.predict(features, verbose=0)

    print("🔍 Predicciones detalladas:")
    for clase, prob in zip(CLASSES, predicciones[0]):
        print(f"{clase:>10}: {prob:.2%}")

    clase_idx = np.argmax(predicciones)
    return CLASSES[clase_idx]

if __name__ == "__main__":
    modelo = cargar_modelo("best_model.keras")
    comando = predecir("../voiceRecognition/uploads/audio.wav", modelo)
    print(f"🔊 Comando detectado: *{comando}*")
