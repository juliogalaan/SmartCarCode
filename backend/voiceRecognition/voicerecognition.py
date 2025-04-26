import pyaudio
import socket
import time

# Configuración del Socket
HOST = 'localhost'  # Dirección IP del servidor (coche o Node-RED)
PORT = 50007        # Puerto para la comunicación

# Configuración de Audio (captura desde el micrófono)
FORMAT = pyaudio.paInt16  # Formato de audio (16-bit PCM)
CHANNELS = 1             # Mono (1 canal)
RATE = 44100             # Frecuencia de muestreo
CHUNK = 1024             # Tamaño del buffer de audio (en muestras)

# Inicialización de PyAudio
p = pyaudio.PyAudio()

def create_socket():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            client_socket.connect((HOST, PORT))
            print("Conectado al servidor")
            return client_socket
        except socket.error:
            print("Error de conexión, intentando nuevamente...")
            time.sleep(2)  # Espera 2 segundos antes de volver a intentar

# Crear un socket de cliente
client_socket = create_socket()

# Función para capturar y enviar audio
def capture_and_send_audio():
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    print("Capturando audio...")

    try:
        while True:
            audio_data = stream.read(CHUNK, exception_on_overflow=False)
