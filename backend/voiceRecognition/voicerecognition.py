import socket
import wave

# Configuración del servidor
HOST = 'localhost'  # Escuchar en localhost (puedes cambiarlo a la IP de tu coche o PC)
PORT = 50007        # El mismo puerto que en el cliente

# Configuración del archivo de audio
OUTPUT_WAV_FILE = 'audio_recibido.wav'
FORMAT = 'h'  # Formato de dato (16 bits PCM)
CHANNELS = 1
RATE = 44100

# Crear el socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print("Esperando conexión...")
conn, addr = server_socket.accept()
print(f"Conectado por {addr}")

# Crear un archivo WAV para guardar el audio
wf = wave.open(OUTPUT_WAV_FILE, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(2)  # 2 bytes (16 bits) por muestra
wf.setframerate(RATE)

try:
    while True:
        data = conn.recv(4096)
        if not data:
            break
        wf.writeframes(data)
except KeyboardInterrupt:
    print("Interrumpido por el usuario.")
finally:
    print("Cerrando conexión y archivo.")
    conn.close()
    server_socket.close()
    wf.close()
