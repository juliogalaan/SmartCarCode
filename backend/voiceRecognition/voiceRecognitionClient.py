import socket

HOST = '192.168.1.100'  # IP del coche/servidor
PORT = 50007

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    with open('audio_a_enviar.wav', 'rb') as f:
        audio_data = f.read()
        s.sendall(audio_data)
