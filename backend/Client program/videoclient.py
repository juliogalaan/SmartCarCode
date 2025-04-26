import cv2
import socket
import struct
import pickle
import tkinter as tk
import threading

class VideoClient:
    def __init__(self, root):
        self.root = root
        self.root.title("Live Video Feed")

        # UI Elements
        self.ip_entry = tk.Entry(root, width=20)
        self.ip_entry.insert(0, "192.168.137.37")  # Default IP
        self.ip_entry.pack()

        self.connect_button = tk.Button(root, text="Connect", command=self.connect_to_server)
        self.connect_button.pack()

        self.disconnect_button = tk.Button(root, text="Disconnect", command=self.disconnect)
        self.disconnect_button.pack()

        self.client_socket = None
        self.connected = False
        self.thread = None

    def connect_to_server(self):
        """Establishes connection to the server and starts the video thread."""
        if self.connected:
            self.disconnect()  # Ensure a clean start

        ip = self.ip_entry.get()
        port = 5001

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.settimeout(5)  # Set timeout

        try:
            self.client_socket.connect((ip, port))
            self.connected = True
            self.thread = threading.Thread(target=self.receive_video, daemon=True)
            self.thread.start()
        except Exception as e:
            print(f"Connection failed: {e}")
            self.client_socket = None  # Prevent using an invalid socket
            self.connected = False

    def disconnect_alt(self):
        self.connected = False
        self.thread.join()
        # Wait until transfer is finished
        while not self.transfer_stopped:
            pass
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
        self.thread = None
        cv2.destroyAllWindows()

    def disconnect(self):
        """Stops video thread, closes socket, and cleans up."""
        if not self.connected:
            return  # Prevent multiple disconnect calls

        self.connected = False  # Stop the receiving loop

        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)  # Gracefully close socket
            except Exception:
                pass  # Ignore errors if socket already closed
            self.client_socket.close()
            self.client_socket = None  # Ensure socket is never used again

        # Ensure OpenCV window is closed properly
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Force window to close

    def receive_video(self):
        """Receives video frames from server and displays them."""
        try:
            while self.connected:
                if not self.client_socket:
                    break  # Prevent using a closed socket

                # Receive frame size
                size_data = self.receive_bytes(4)
                if not size_data:
                    break

                data_size = struct.unpack(">L", size_data)[0]

                # Receive frame data
                data = self.receive_bytes(data_size)
                if not data:
                    break

                frame_data = pickle.loads(data)
                frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)

                if frame is not None:
                    cv2.imshow("Live Video", frame)
                if self.client_socket and self.connected:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):  # Quit
                        self.client_socket.sendall(b'q')
                        self.disconnect()
                        break
                    elif key == ord('g'):  # Send 'g' key
                        self.client_socket.sendall(b'g')
                    elif key == ord('w'):  # Send 'w' key
                        self.client_socket.sendall(b'w')
                    elif key == ord('s'):  # Send 's' key
                        self.client_socket.sendall(b's')
                    elif key == ord('a'):  # Send 'a' key
                        self.client_socket.sendall(b'a')
                    elif key == ord('d'):  # Send 'd' key
                        self.client_socket.sendall(b'd')
                    elif key == ord('h'):  # Send 'h' key
                        self.client_socket.sendall(b'h')
                    elif key == ord('f'):  # Send 'f' key
                        self.client_socket.sendall(b'f')
                    elif key == ord('g'):  # Send 'g' key
                        self.client_socket.sendall(b'g')
                    elif key == ord('t'):  # Send 't' key
                        self.client_socket.sendall(b't')
                    elif key == ord('x'):  # Send 'x' key
                        self.client_socket.sendall(b'x')
                    elif key == ord('?'):  # Send '?' key
                        self.client_socket.sendall(b'?')
                    elif key == ord('c'):  # Send 'c' key
                        self.client_socket.sendall(b'c')
                    elif key == ord('7'):  # Send '7' key
                        self.client_socket.sendall(b'7')
                    elif key == ord('u'):  # Send 'u' key
                        self.client_socket.sendall(b'u')
                    elif key == ord('j'):  # Send 'j' key
                        self.client_socket.sendall(b'j')
                    elif key == ord('m'):  # Send 'm' key
                        self.client_socket.sendall(b'm')
                    elif key == ord('8'):  # Send '8' key
                        self.client_socket.sendall(b'8')
                    elif key == ord('i'):  # Send 'i' key
                        self.client_socket.sendall(b'i')
                    elif key == ord('k'):  # Send 'k' key
                        self.client_socket.sendall(b'k')
                    elif key == ord(','):  # Send ',' key
                        self.client_socket.sendall(b',')
                    elif key == ord('9'):  # Send '9' key
                        self.client_socket.sendall(b'9')
                    elif key == ord('o'):  # Send 'o' key
                        self.client_socket.sendall(b'o')
                    elif key == ord('l'):  # Send 'l' key
                        self.client_socket.sendall(b'l')
                    elif key == ord('.'):  # Send '.' key
                        self.client_socket.sendall(b'.')
                    elif key == ord('y'):  # Send 'y' key
                        self.client_socket.sendall(b'y')
        except Exception as e:
            print(f"Error receiving video: {e}")
        finally:
            self.disconnect()

    def receive_bytes(self, num_bytes):
        """Ensures full data reception."""
        data = b""
        while len(data) < num_bytes:
            try:
                if not self.client_socket or not self.connected:
                    return None  # Stop receiving if socket is closed

                packet = self.client_socket.recv(num_bytes - len(data))
                if not packet:
                    return None
                data += packet
            except socket.timeout:
                print("Socket timeout, disconnecting...")
                self.disconnect()
                return None
            except OSError as e:
                print(f"Socket error: {e}")
                self.disconnect()
                return None
        return data

if __name__ == "__main__":
    root = tk.Tk()
    app = VideoClient(root)
    root.mainloop()
