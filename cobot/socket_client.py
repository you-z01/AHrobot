import socket
import ast
import json


class TCPClient:
    def __init__(self, server_ip, server_port):
        """初始化客户端并设置服务器 IP 和端口"""
        self.server_ip = server_ip
        self.server_port = server_port
        self.client_socket = None

    def connect(self):
        """连接到服务器"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_ip, self.server_port))
            # print(f"成功连接到服务器 {self.server_ip}:{self.server_port}")
        except Exception as e:
            print(f"连接错误: {e}")
            self.client_socket = None

    def send_message(self, data):
        """发送数据到服务器"""
        if self.client_socket:
            try:
                message = data.encode('utf-8')  # 转换为字节格式
                self.client_socket.sendall(message)
                print(f"已发送数据: {data}")
            except Exception as e:
                print(f"发送数据时发生错误: {e}")
        else:
            print("尚未连接到服务器")

    def receive_message(self, buffer_size=1024):
        """从服务器接收数据"""
        if self.client_socket:
            try:
                data = self.client_socket.recv(buffer_size)
                decoded_data = data.decode('utf-8')  # 将接收到的字节数据解码为字符串
                print(f"收到数据: {decoded_data}")
                return decoded_data
            except Exception as e:
                print(f"接收数据时发生错误: {e}")
                return None
        else:
            print("尚未连接到服务器")
            return None

    def close(self):
        """关闭客户端连接"""
        if self.client_socket:
            self.client_socket.close()
            print("已关闭与服务器的连接")
        else:
            print("客户端尚未连接")


def parse_data(data_str):
    """
    通用数据解析函数, 能够处理JSON、列表格式的数据。
    
    参数:
    data_str (str): 待解析的数据字符串。
    
    返回:
    list: 解析后的数据列表。如果解析失败, 返回None。
    """
    # 去除前后的空白符或无效字符
    data_str = data_str.strip()
    
    # 如果数据字符串为空，返回None
    if not data_str:
        print("收到的数据为空")
        return None

    # 尝试解析为JSON格式
    try:
        return json.loads(data_str)
    except json.JSONDecodeError:
        pass  # 如果失败，继续尝试其他方式

    # 尝试解析为Python列表
    try:
        return ast.literal_eval(data_str)
    except (ValueError, SyntaxError):
        pass  # 如果失败，继续

    # 如果所有尝试都失败，返回None并提示错误
    print(f"无法解析数据：{data_str}")
    return None



# 示例用法
if __name__ == "__main__":
    # 配置服务器 IP 和端口
    server_ip = '192.168.137.238'
    server_port = 9000

    # 创建客户端实例
    client = TCPClient(server_ip, server_port)

    # 连接服务器
    client.connect()

    # 发送数据示例
    client.send_message("[0.01877, 0.01158, 0.195, 103.2, 99.2, 47.2]")

    # 接收服务器返回的数据
    response = client.receive_message()

    # 关闭连接
    client.close()
