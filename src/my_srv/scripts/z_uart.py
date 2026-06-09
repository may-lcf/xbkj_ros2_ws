# 导包
import serial
import time
import threading
import serial.tools.list_ports
import subprocess  # 新增：用于执行权限修复命令
import os  # 已导入，无需重复

# 全局变量定义
ser = ''
uart_baud = 115200
uart_get_ok = 0
uart_receive_buf = ""
uart_receive_buf_index = 0
_uart_running = False

def uart_send_str(string):
    global ser
    try:
        if not ser or not ser.is_open:
            print("uart_send_str 错误：串口未打开！")
            return False
        data = string.encode("utf-8")
        bytes_written = ser.write(data)  # 关键步骤：实际发送
        time.sleep(0.005)
        ser.flush()
        return True
    except Exception as e:
        print(f"uart_send_str 失败：{str(e)}")
        return False

# 线程调用函数，主要处理数据接受格式，主要格式为 $...!   #...! {...}三种格式，...内容长度不限
def serialEvent():
    global ser, uart_receive_buf_index, uart_receive_buf, uart_get_ok
    mode = 0
    try:
        while _uart_running:
            if uart_get_ok == 0:
                if not ser or not ser.is_open:
                    break
                uart_receive_buf_index = ser.inWaiting()
                if uart_receive_buf_index > 0:
                    uart_receive_buf = uart_receive_buf + ser.read(uart_receive_buf_index).decode(errors='ignore')  # 新增 errors='ignore' 避免解码错误
                    # print('get1:',uart_receive_buf, " len:", len(uart_receive_buf), " mode:", mode)
                    if mode == 0:
                        if uart_receive_buf.find('{') >= 0:
                            mode = 1
                            # print('mode1 start')
                        elif uart_receive_buf.find('$') >= 0:
                            mode = 2
                            # print('mode2 start')
                        elif uart_receive_buf.find('#') >= 0:
                            mode = 3
                            # print('mode3 start')

                    if mode == 1:
                        if uart_receive_buf.find('}') >= 0:
                            uart_get_ok = 1
                            mode = 0
                            ser.flushInput()
                            # print('{}:',uart_receive_buf, " len:", len(uart_receive_buf))
                            # print('mode1 end')
                    elif mode == 2:
                        if uart_receive_buf.find('!') >= 0:
                            uart_get_ok = 2
                            mode = 0
                            ser.flushInput()
                            # print('$!:',uart_receive_buf, " len:", len(uart_receive_buf))
                            # print('mode2 end')
                    elif mode == 3:
                        if uart_receive_buf.find('!') >= 0:
                            uart_get_ok = 3
                            mode = 0
                            ser.flushInput()
                            # print('#!:', uart_receive_buf, " len:", len(uart_receive_buf))
                            # print('mode3 end')

                    # print('get2:',uart_receive_buf, " len:", len(uart_receive_buf), " mode:", mode, " getok:", uart_get_ok)

    except IOError:
        pass

# 串口接收线程
uart_thread = threading.Thread(target=serialEvent, daemon=True)  # 新增 daemon=True：主线程退出时自动关闭子线程

# 修复串口权限
def fix_ttyS0_permission():
    print("正在修复串口权限...")
    password = "1234"
    try:
        # 使用 echo 管道传递密码给 sudo -S
        cmd = f'echo "{password}" | sudo -S chmod 660 /dev/ttyAMA0'
        result = subprocess.run(
            cmd,
            shell=True,          # 启用shell解析
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5
        )
        # 检查是否有密码错误提示
        if "incorrect password" in result.stderr.lower():
            print("错误：密码不正确！")
            return False
            
        print("串口权限修复成功！")
        return True
    except subprocess.CalledProcessError as e:
        print(f"权限修复失败（命令执行错误）：{e.stderr.strip()}")
        return False
    except subprocess.TimeoutExpired:
        print("权限修复失败（超时）")
        return False
    except Exception as e:
        print(f"权限修复失败（未知错误）：{str(e)}")
        return False

# 串口初始化（整合权限修复）
def setup_uart(baud):
    global ser, uart_thread, uart_receive_buf, uart_baud, _uart_running
    uart_baud = baud
    exclusive = False
    try:
        # 1. 先修复串口权限（关键步骤）
        fix_ttyS0_permission()
        time.sleep(0.3)  # 延迟 0.3 秒，确保权限生效

        # 2. 检查设备是否存在
        if not os.path.exists("/dev/ttyAMA0"):
            print(f"错误：串口设备 /dev/ttyAMA0 不存在！")
            return False

        # 3. 尝试初始化串口
        ser = serial.Serial(
            port="/dev/ttyAMA0",
            baudrate=uart_baud,
            timeout=1,  # 超时设置，避免无限阻塞
        )

        # 4. 检查串口是否真的打开
        if not ser.is_open:
            print("错误：串口初始化成功，但未打开！")
            return False

        ser.flushInput()
        # 5. 启动线程前先检查线程状态
        _uart_running = True
        if not uart_thread.is_alive():
            uart_thread = threading.Thread(target=serialEvent, daemon=True)
            uart_thread.start()
            print("接收线程已启动")
        else:
            print("接收线程已在运行")

        # 6. 延迟一小段时间，确保串口就绪
        time.sleep(0.2)
        # 7. 尝试发送初始化消息
        uart_send_str("#255P1500T1000!")
        print("uart init ok!\r\n")  # 打印到控制台，确认执行到这一步
        uart_receive_buf = ''
        return True

    except Exception as e:
        print(f"串口初始化失败：{str(e)}")
        return False

def close_uart():
    global ser, _uart_running
    _uart_running = False
    if ser and ser.is_open:
        ser.close()
        time.sleep(0.5)  # 等待 OS 释放串口资源
        ser = ''
        print("串口已关闭")

# 循环执行串口
def loop_uart():
    global uart_get_ok, uart_receive_buf
    if uart_get_ok:
        print(f"接收数据：{uart_receive_buf}")  # 优化输出，更直观
        uart_send_str(uart_receive_buf)  # 回显接收的数据
        uart_receive_buf = ''
        uart_get_ok = 0

# 大循环
if __name__ == '__main__':
    # 初始化串口（波特率 115200）
    if not setup_uart(115200):
        print("串口初始化失败，程序退出！")
        exit(1)  # 初始化失败直接退出

    try:
        while True:
            loop_uart()
            uart_send_str("#255P1500T1000!")
            time.sleep(1)
            # print(0)

    except KeyboardInterrupt:
        print("\n收到退出信号，正在关闭程序...")
        close_uart()  # 退出时关闭串口
        exit(0)