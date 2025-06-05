import socket

HOST = '127.0.0.1'  # 혹은 실제 IP
PORT = 12345

# 서버 소켓 생성
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print(f"서버 대기 중: {HOST}:{PORT}")

conn, addr = server_socket.accept()
print(f"클라이언트 연결: {addr}")

while True:
    data = conn.recv(1024).decode('utf-8')
    if not data:
        break
    print(f"받은 메시지: {data}")
    
    # 응답 전송
    conn.sendall("메시지 수신 완료".encode('utf-8'))

conn.close()
server_socket.close()
