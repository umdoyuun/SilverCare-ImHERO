#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import json
import time

HOST = '0.0.0.0'  # 서버가 모든 인터페이스에서 연결을 수신
PORT = 12345      # 클라이언트와 동일한 포트 사용

def start_test_server():
    # 소켓 생성
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        # TIME_WAIT 충돌 방지를 위해 SO_REUSEADDR 옵션 설정
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"[Server] Listening on {HOST}:{PORT}")

        # 클라이언트(위의 SocketClient)가 연결할 때까지 대기
        conn, addr = server_socket.accept()
        with conn:
            print(f"[Server] Connected by {addr}")

            # 무한 루프: 일정 간격으로 JSON 메시지를 보내기
            while True:
                # 예시로 보낼 JSON (client.process_message가 파싱 가능한 구조)
                data = {
                    "user_id": "test_user_01",
                    "session_id": "6a286f5a6f5f90f20636d653c4d806f0",
                    "family_id": "FlcuDLxVC9SolW70",
                    "is_camera_enabled": True,
                    "is_driving_enabled": False,
                }

                # JSON 문자열로 직렬화하여 전송
                message = json.dumps(data)
                conn.sendall(message.encode('utf-8'))
                print(f"[Server] Sent message: {message}")

                # 5초 간격으로 전송(테스트 목적)
                time.sleep(5)

if __name__ == '__main__':
    start_test_server()
