'''
Author: lihang lihang@kilox.cn
Date: 2025-09-26 17:49:10
LastEditors: lihang lihang@kilox.cn
LastEditTime: 2025-09-26 17:49:13
FilePath: /fast_lvio_ws/src/open_slam/scripts/test_base_station.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import socket

def tcp_client(ip, port):
    # 创建 TCP 套接字
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            # 连接目标 IP 和端口
            s.connect((ip, port))
            print(f"已连接到 {ip}:{port}")
            
            # 循环接收数据（每次最多接收 1024 字节）
            while True:
                data = s.recv(1024)
                if not data:
                    # 数据接收完毕（连接关闭）
                    print("\n连接已关闭")
                    break
                # 解码字节数据为字符串并打印（根据实际编码调整，如 utf-8、gbk 等）
                print(data)
                
        except ConnectionRefusedError:
            print(f"连接失败：{ip}:{port} 拒绝连接（端口未开放或服务未启动）")
        except Exception as e:
            print(f"发生错误：{e}")

# 调用函数：替换为你的 IP 和端口
tcp_client("192.168.10.222", 12000)