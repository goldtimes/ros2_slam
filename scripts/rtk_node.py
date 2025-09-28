import asyncio
import serial
import socket
import rospy
import subprocess
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix


class RTKNode(object):
    def __init__(self) -> None:
        # 初始化ROS节点
        rospy.init_node('rtk_node', anonymous=True)
        
        # 配置参数
        self.base_station_ip = "192.168.10.222"  # 目标IP修正
        self.base_station_port = 12000           # 端口应为整数
        self.usb_port = "/dev/ttyUSB0"           # 串口设备
        
        # 初始化串口
        self.setup_serial()
        
        # 初始化TCP socket
        self.server_socket = None
        
        # 用于存储GNSS数据的变量
        self.gnss_data = ""
        
        # ROS发布器（可选）
        self.rtk_pub = rospy.Publisher('/rtcm_data', String, queue_size=10)
        self.gnss_pub = rospy.Publisher("/gnss", NavSatFix, queue_size=20)
        
        # 保持程序运行的标志
        self.running = True

    def setup_serial(self):
        # 赋予串口权限
        subprocess.run(
            f'echo 123456 | sudo -S chmod +777 {self.usb_port}',
            shell=True,
            check=True
        )
        
        # 初始化串口连接
        try:
            self.serial = serial.Serial(
                port=self.usb_port,
                baudrate=115200,
                timeout=0.5
            )
            rospy.loginfo(f"串口 {self.usb_port} 初始化成功")
        except Exception as e:
            rospy.logerr(f"串口初始化失败: {e}")
            raise

    async def connect_to_server(self):
        """建立TCP连接（兼容Python 3.8及以下）"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(
                None,
                self.server_socket.connect,
                (self.base_station_ip, self.base_station_port)
            )
            rospy.loginfo(f"已连接到 {self.base_station_ip}:{self.base_station_port}")
            return True
        except Exception as e:
            rospy.logerr(f"TCP连接失败: {e}")
            return False

    async def receive_rtcm_data(self):
        """接收RTCM数据并写入串口（兼容低版本Python）"""
        if not self.server_socket:
            rospy.logerr("未建立TCP连接")
            return
        with open("rtcm_data.bin", "ab") as rtcm_file:
            try:
                loop = asyncio.get_event_loop()
                while self.running and not rospy.is_shutdown():
                    # 接收TCP数据（阻塞操作，用线程池执行）
                    print("等待RTCM数据...")
                    data = await loop.run_in_executor(
                        None,
                        self.server_socket.recv,
                        1024
                    )
                    # print(data.hex())
                    if not data:
                        rospy.logwarn("TCP连接已关闭")
                        break
                    
                    # 写入串口（阻塞操作，用线程池执行）
                    await loop.run_in_executor(
                        None,
                        self.serial.write,
                        data
                    )

                     # 2. 写入文件（二进制模式）
                    rtcm_file.write(data)
                    rtcm_file.flush()  # 立即刷新到磁盘，避免数据滞留内存
                    # # 发布调试信息
                    self.rtk_pub.publish(f"收到 {len(data)} 字节RTCM数据")

            except Exception as e:
                rospy.logerr(f"数据处理错误: {e}")

    async def read_gnss_data(self):
        """读取串口的GNSS数据（如GGA）并处理"""
        while self.running and not rospy.is_shutdown():
            try:
                # 读取串口数据（GNSS模块输出）
                line_bytes = self.serial.readline()
                if line_bytes:
                    # 尝试解码（根据实际编码调整）
                    # print("line_bytes:", line_bytes)
                    # try:
                    self.gnss_data = line_bytes.decode('ascii')
                    # except UnicodeDecodeError:
                    #     self.gnss_data = line_bytes.decode('gbk', errors='ignore')
                    # 处理GGA数据并发布到ROS
                    if 'GNGGA' in self.gnss_data or 'GPGGA' in self.gnss_data:
                        print("gnss_data:", self.gnss_data)

                        self.pub_gnss_to_ros(self.gnss_data)
                await asyncio.sleep(0.01)

            except Exception as e:
                rospy.logerr(f"GNSS数据读取错误: {e}")
                await asyncio.sleep(1)

    def pub_gnss_to_ros(self, data):
        """解析GGA数据并发布NavSatFix消息"""
        try:
            gps_data = data.split(',')
            if len(gps_data) < 10:
                return

            # 解析经纬度（NMEA格式转十进制）
            def nmea_to_decimal(nmea_str, is_latitude):
                if not nmea_str:
                    return 0.0
                degrees = int(float(nmea_str) / 100)
                minutes = float(nmea_str) - degrees * 100
                decimal = degrees + minutes / 60
                return decimal if is_latitude else decimal

            # 构造NavSatFix消息
            gps_msg = NavSatFix()
            gps_msg.header.frame_id = "gps"
            gps_msg.header.stamp = rospy.Time.now()
            
            # 纬度 (GGA第2字段)
            gps_msg.latitude = nmea_to_decimal(gps_data[2], is_latitude=True)
            # 经度 (GGA第4字段)
            gps_msg.longitude = nmea_to_decimal(gps_data[4], is_latitude=False)
            # 海拔 (GGA第9字段)
            gps_msg.altitude = float(gps_data[9]) if gps_data[9] else 0.0
            # 定位状态 (GGA第6字段)
            gps_msg.status.status = int(gps_data[6]) if gps_data[6] else -1

            self.gnss_pub.publish(gps_msg)
            rospy.logdebug(f"发布GNSS数据: {gps_data[2]}, {gps_data[4]}")

        except Exception as e:
            rospy.logerr(f"GNSS数据解析错误: {e}")

    async def run(self):
        """主运行函数"""
        # 建立TCP连接
        if not await self.connect_to_server():
            return

        # 并发运行两个任务：接收RTCM数据 + 读取GNSS数据
        try:
            await asyncio.gather(
                self.receive_rtcm_data(),
                self.read_gnss_data()
            )
        finally:
            self.close()

    def close(self):
        """资源清理"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            rospy.loginfo("串口已关闭")
        if self.server_socket:
            self.server_socket.close()
            rospy.loginfo("TCP连接已关闭")


if __name__ == "__main__":
    try:
        rtk_node = RTKNode()
        asyncio.run(rtk_node.run())
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"程序异常退出: {e}")