import socket

def main(a):
    # 服务器的 IP 地址和端口号
    SERVER_IP = '192.168.137.238'
    SERVER_PORT = 9000

    # 创建一个 TCP/IP 套接字
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # 连接到服务器
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print(f"已连接到 {SERVER_IP}:{SERVER_PORT}")
        if a == "1":
            # 发送数据
            message = "[1]"
            client_socket.sendall(message.encode('utf-8'))
            print(f"已发送数据: {message}")

            # data = client_socket.recv(1024)
            # print(f"收到回显数据: {data.decode('utf-8')}")

        if a == "2":
            # 发送数据
            message = "[0.02398, -0.02622, 0.181, -171.06, -3.58, 2.69]"
            client_socket.sendall(message.encode('utf-8'))
            print(f"已发送数据: {message}")
        if a == "3":
            client_socket.close()
            print("断开连接")

        if a == "4":
            # 发送数据
            message = "[1]"
            client_socket.sendall(message.encode('utf-8'))
            print(f"已发送数据: {message}")
            data = client_socket.recv(1024)
            print(f"收到回显数据: {data.decode('utf-8')}")
            print(type(data))
        # 接收回显数据
       

    except Exception as e:
        print(f"发生错误: {e}")



if __name__ == "__main__":

    # main()
    while True:
        try:
            a = input("输入选项: ")
            main(a)
        except KeyboardInterrupt:
            print("\n已退出")
            break
        except Exception as e:
            print(f"发生错误: {e}")
            continue