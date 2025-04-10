import serial
import struct
from vpython import *
import numpy as np
import threading
import time

from Posture import *
from DynamicPlot import *

SERIAL_PORT = '/dev/rfcomm0'  # Linux 
BAUD_RATE = 115200
START_BYTE = b'\xAB' 
DATA_LENGTH = 44
CRC_LENGTH = 4
FRAME_LENGTH = 1 + DATA_LENGTH + CRC_LENGTH

last_data_time = None
data_intervals = []  


pitch = 0.0  
roll = 0.0     
yaw = 0.0     

pressure = [0] * 16  


def parse_data_frame(data_bytes):

    if len(data_bytes) != 44:
        raise ValueError(f"数据长度必须为44字节，当前长度: {len(data_bytes)}")
    
    
    pitch_raw = struct.unpack('<i', data_bytes[0:4])[0]
    roll_raw = struct.unpack('<i', data_bytes[4:8])[0]
    yaw_raw = struct.unpack('<i', data_bytes[8:12])[0]
    
   
    pitch = pitch_raw / 65536.0
    roll = roll_raw / 65536.0
    yaw = yaw_raw / 65536.0

    pressure_values = []
    for i in range(16):

        start_pos = 12 + i * 2
        value = struct.unpack('<H', data_bytes[start_pos:start_pos+2])[0]
        pressure_values.append(value)

    new_order = [0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15]
    reordered_pressure_values = [pressure_values[i] for i in new_order]
    
    return pitch, roll, yaw, reordered_pressure_values

def crc32mpeg2(buf, crc=0xffffffff):
    for val in buf:
        crc ^= val << 24
        for _ in range(8):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
    return crc



def calculate_crc(data_bytes):
    if len(data_bytes) % 4 != 0:
        
        padding = 4 - (len(data_bytes) % 4)
        data_bytes = data_bytes + bytes([0] * padding)
    
    
    big_endian_bytes = bytearray()
    for i in range(0, len(data_bytes), 4):
        
        chunk = data_bytes[i:i+4]
        
        reversed_chunk = chunk[::-1]
        big_endian_bytes.extend(reversed_chunk)
    return crc32mpeg2(big_endian_bytes) & 0xFFFFFFFF  


def process_serial_data(posture_cuboid, posture_scene, points,valid_grid_points, grid_x, grid_z, ax, cax, mask, scatter, fig):
    global last_data_time, data_intervals
    ser = None 
    try:
        
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0  
        )
        print(f"串口 {SERIAL_PORT} 已打开 @ {BAUD_RATE} bps")
        ser.write(b"start\r\n")
        print("发送开始命令")

        read_buffer = b'' 

        while True:
           
            byte = ser.read(1)
            if not byte:
                
                print("等待数据...")
                ser.write(b"start\r\n")
                continue

            if byte == START_BYTE:
                
                read_buffer = byte 
                
                
                remaining_bytes_to_read = DATA_LENGTH + CRC_LENGTH
                frame_data = ser.read(remaining_bytes_to_read)

                if len(frame_data) == remaining_bytes_to_read:
                    print(f"数量: {len(frame_data)}")
                    
                    read_buffer += frame_data 
                    
                    
                    data_payload = read_buffer[1:1 + DATA_LENGTH] 
                    received_crc_bytes = read_buffer[1 + DATA_LENGTH:] 

                   
                    
                    received_crc = struct.unpack('<I', received_crc_bytes)[0]

                    
                    calculated_crc = calculate_crc(data_payload)
                   
                    if calculated_crc == received_crc:
                        
                        pitch, roll, yaw, pressure = parse_data_frame(data_payload)

                        posture( posture_cuboid, posture_scene, pitch, roll, yaw)
                        update_dynamic_plot(points, pressure[:15], valid_grid_points, grid_x, grid_z, ax, cax, mask, scatter, fig)

                        print(f"pitch={pitch:.2f}, roll={roll:.2f}, yaw={yaw:.2f}")
                        print(f"pressure={pressure}")

                        # current_time = time.time()
                        # # 计算两次数据接收之间的时间间隔
                        # if last_data_time is not None:
                        #     interval = current_time - last_data_time
                        #     data_intervals.append(interval)
                            
                        #     # 打印时间间隔
                        #     print(f"数据接收间隔: {interval*1000:.2f} ms")
                            
                        #     # 计算和显示统计信息(每接收20个数据包)
                        #     if len(data_intervals) % 20 == 0:
                        #         avg_interval = sum(data_intervals[-20:]) / 20
                        #         min_interval = min(data_intervals[-20:])
                        #         max_interval = max(data_intervals[-20:])
                        #         freq = 1.0 / avg_interval if avg_interval > 0 else 0
                                
                        #         print("\n=== 数据接收统计 ===")
                        #         print(f"平均间隔: {avg_interval*1000:.2f} ms")
                        #         print(f"最小间隔: {min_interval*1000:.2f} ms")
                        #         print(f"最大间隔: {max_interval*1000:.2f} ms")
                        #         print(f"平均频率: {freq:.2f} Hz")
                        #         print("==================\n")
                        
                        # # 更新上次数据接收时间
                        # last_data_time = current_time
                    else:
                        
                        print(f"CRC 校验失败! 接收到的 CRC: {received_crc:#06x}, 计算得到的 CRC: {calculated_crc:#06x}. 丢弃数据帧。")
                       

                    
                    read_buffer = b''

                else:
                    print(f"读取数据帧不完整 (期望 {remaining_bytes_to_read} 字节，实际 {len(frame_data)} 字节)。可能丢帧或超时。")
                    read_buffer = b''
                    

    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print(f"无法打开或读取串口 {SERIAL_PORT}。请检查端口名、权限以及设备连接。")
    except KeyboardInterrupt:
        print("接收被中断。正在关闭串口...")
    except Exception as e:
        print(f"发生未预料的错误: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print(f"串口 {SERIAL_PORT} 已关闭。")


# --- 启动程序 ---
if __name__ == "__main__":
    # main()
    posture_cuboid, posture_scene = posture_init()
    points,valid_grid_points, grid_x, grid_z, ax, cax, mask, scatter, fig = init_dynamic_plot()
    process_serial_data(posture_cuboid, posture_scene, points, valid_grid_points, grid_x, grid_z, ax, cax, mask, scatter, fig)