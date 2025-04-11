import socket
import json
import threading
import time
import queue
#import websockets

class SocketServer:
    def __init__(self, host='0.0.0.0', tcp_port=12345):
       # 기존 TCP 소켓 설정
       self.host = host
       self.tcp_port = tcp_port

       self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
       self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

       self.running = False
       self.connected = False
       self.client_socket = None
       self.jetson_address = None

       # 이벤트 기반 메시지 큐
       self.send_queue = queue.Queue()
       self.message_available = threading.Event()
       self.client_connected = threading.Event()

       # 소켓 타임아웃 설정
       self.socket_timeout = 5.0

    def start(self):
       try:
           # TCP 서버 설정
           self.socket.bind((self.host, self.tcp_port))
           self.socket.listen(1)
           self.socket.settimeout(1.0)
           self.running = True
           print(f"TCP server waiting for connection on {self.host}:{self.tcp_port}")

           # 기존 TCP 스레드 시작
           for target in [self._accept_connection, self._send_message_loop]:
               thread = threading.Thread(target=target)
               thread.daemon = True
               thread.start()

       except Exception as e:
           print(f"Failed to start server: {e}")
           self.stop()
           raise

    def _accept_connection(self):
       while self.running:
           try:
               try:
                   client_socket, addr = self.socket.accept()
               except socket.timeout:
                   continue

               if self.connected:
                   print("Rejecting new connection: Already connected")
                   client_socket.close()
                   continue

               # 클라이언트 소켓 설정
               client_socket.settimeout(self.socket_timeout)
               self.client_socket = client_socket
               self.jetson_address = addr
               self.connected = True

               print(f"Connected with Jetson at {addr}")
               self.client_connected.set()

               # 수신 스레드 시작
               receive_thread = threading.Thread(target=self._receive_messages)
               receive_thread.daemon = True
               receive_thread.start()
               try: 
                    with open('user_data.json', 'r', encoding='utf-8') as file:
                        user_data = json.load(file)
                
                    socketServer.send_message(user_data)
               except Exception as e:
                    print("Error in sending user_data: ", e)
                    pass 
            
           except Exception as e:
               if isinstance(e, socket.timeout):
                   continue
               print(f"Error in connection acceptance: {e}")
               time.sleep(1)

    def _receive_messages(self):
       while self.running and self.connected:
           try:
               data = self.client_socket.recv(1024)
               if not data:
                   raise ConnectionError("Connection closed by client")

               message = json.loads(data.decode('utf-8'))
               self._handle_message(message)

           except socket.timeout:
               continue
           except json.JSONDecodeError as e:
               print(f"Invalid JSON received: {e}")
           except ConnectionError as e:
               print(f"Connection error: {e}")
               break
           except Exception as e:
               print(f"Error in message reception: {e}")
               break

       self._cleanup_connection()

    def _send_message_loop(self):
       while self.running:
           try:
               if not self.connected:
                   self.client_connected.wait(timeout=1.0)
                   continue

               try:
                   message = self.send_queue.get(timeout=1.0)
                   self._send_message(message)
               except queue.Empty:
                   continue

           except Exception as e:
               print(f"Error in send loop: {e}")
               time.sleep(1)

    def _send_message(self, message):
       if not self.connected or not self.client_socket:
           return False

       try:
           data = json.dumps(message).encode('utf-8')
           self.client_socket.send(data)
           return True

       except Exception as e:
           print(f"Error sending message: {e}")
           self._cleanup_connection()
           return False

    def _cleanup_connection(self):
       if self.client_socket:
           try:
               self.client_socket.close()
           except:
               pass
           self.client_socket = None

       self.connected = False
       self.jetson_address = None
       self.client_connected.clear()
       print("Connection with Jetson closed")

    def _handle_message(self, message):
       print(f"Received from Jetson: {message}")

    def send_message(self, message):
       self.send_queue.put(message)
       self.message_available.set()

    def stop(self):
       self.running = False
       self._cleanup_connection()

       # 대기 중인 스레드들을 깨움
       self.message_available.set()
       self.client_connected.set()

       try:
           self.socket.close()
       except:
           pass

       print("Server stopped")

raspberry_instance = None

def get_raspberry_instance():
    global raspberry_instance
    if raspberry_instance is None:
        raspberry_instance = SocketServer()
        raspberry_instance.start()
    return raspberry_instance

if __name__ == "__main__":
    socketServer = get_raspberry_instance()
    try:
        flag = False
        while True:
            time.sleep(1)
            if socketServer.connected:
                if not flag:
                    try: 
                            with open('user_data.json', 'r', encoding='utf-8') as file:
                                user_data = json.load(file)
                    
                            socketServer.send_message(user_data)
                            flag = True
                    except:
                            pass 
                # print("input: ", end='')
                # text = input()
                # socketServer.send_message(text)
    except KeyboardInterrupt:
        print("Shutting down...")
        socketServer.stop()  

