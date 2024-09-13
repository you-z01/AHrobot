import socket


class TCPClient:
    def __init__(self, server_ip, server_port):
        """ 初始化客户端 """
        self.server_ip = server_ip
        self.server_port = server_port
        self.client_socket = None

    def connect(self):
        """ 连接到服务器 """
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect((self.server_ip, self.server_port))
            #print(f"已连接到 {self.server_ip}:{self.server_port}")
        except Exception as e:
            print(f"连接错误: {e}")
            self.client_socket = None

    def send_message(self, message):
        """ 发送消息 """
        if self.client_socket:
            try:
                self.client_socket.sendall(message.encode('utf-8'))
                print(f"已发送数据: {message}")
            except Exception as e:
                print(f"发送错误: {e}")
        #else:
            # print("客户端未连接")

    def receive_message(self):
        """ 接收消息 """
        if self.client_socket:
            try:
                data = self.client_socket.recv(1024)
                print(f"接收到数据: {data.decode('utf-8')}")
                return data
            except Exception as e:
                print(f"接收错误: {e}")
                return None
        else:
            print("客户端未连接")
            return None

    def close(self):
        """ 关闭连接 """
        if self.client_socket:
            self.client_socket.close()
            print("断开连接")
        else:
            print("客户端未连接")



def main():
    client = TCPClient('192.168.137.238', 9000)
    client.connect()

    while True:
        try:
            a = input("输入选项: ")
            if a == "1":
                client.send_message("[0.01877, 0.01158, 0.195, 103.2, 99.2, 47.2]")
            elif a == "4":
                client.send_message("[1]")
                client.receive_message()
            elif a == "2":
                client.send_message("[0.02398, -0.02622, 0.181, -171.06, -3.58, 2.69]")
            elif a == "3":
                client.close()
                break
        except KeyboardInterrupt:
            print("\n已退出")
            client.close()
            break
        except Exception as e:
            print(f"发生错误: {e}")
            continue

if __name__ == "__main__":
    main()











# import socket

# def main(a):
#     # 服务器的 IP 地址和端口号
#     SERVER_IP = '192.168.137.238'
#     SERVER_PORT = 9000

#     # 创建一个 TCP/IP 套接字
#     client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#     try:
#         # 连接到服务器
#         client_socket.connect((SERVER_IP, SERVER_PORT))
#         print(f"已连接到 {SERVER_IP}:{SERVER_PORT}")
#         if a == "1":
#             # 发送数据
#             message = "[0.01877, 0.01158, 0.195, 103.2, 99.2, 47.2]"
#             client_socket.sendall(message.encode('utf-8'))
#             print(f"已发送数据: {message}")

#             # data = client_socket.recv(1024)
#             # print(f"收到回显数据: {data.decode('utf-8')}")
#         if a == "4":
#             # 发送数据
#             message = "[1]"
#             client_socket.sendall(message.encode('utf-8'))
#             print(f"已发送数据: {message}")
#             data = client_socket.recv(1024)
#             print(f"收到回显数据: {data.decode('utf-8')}")
#             print(type(data))
#         if a == "2":
#             # 发送数据
#             message = "[0.02398, -0.02622, 0.181, -171.06, -3.58, 2.69]"
#             client_socket.sendall(message.encode('utf-8'))
#             print(f"已发送数据: {message}")
#         if a == "3":
#             client_socket.close()
#             print("断开连接")
#         # 接收回显数据
       

#     except Exception as e:
#         print(f"发生错误: {e}")



# if __name__ == "__main__":

#     # main()
#     while True:
#         try:
#             a = input("输入选项: ")
#             main(a)
#         except KeyboardInterrupt:
#             print("\n已退出")
#             break
#         except Exception as e:
#             print(f"发生错误: {e}")
#             continue
