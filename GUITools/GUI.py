import customtkinter as ctk
import tkinter as tk
from tkinter import ttk
import threading
import time
import os
import sys
from typing import Dict, Any, Optional

# 設定CTK外觀
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# 嘗試導入專案中的GUIClass
try:
    # 假設GUITools在專案根目錄
    gui_tools_path = os.path.join(os.path.dirname(__file__), 'GUITools')
    if gui_tools_path not in sys.path:
        sys.path.append(gui_tools_path)
    
    # 導入已實現的GUIClass
    from CCD1GUIClass import CCD1GUIClass
    from CCD2GUIClass import CCD2GUIClass  
    from CCD3GUIClass import CCD3GUIClass
    
    # VPGUIClass和LightGUIClass可能還沒實現，我們自己實現簡化版本
    GUICLASS_AVAILABLE = True
    print("成功載入GUIClass模組")
except ImportError as e:
    print(f"GUIClass模組載入失敗: {e}")
    GUICLASS_AVAILABLE = False

# 導入PyModbus用於VP和Light的簡化控制
try:
    from pymodbus.client import ModbusTcpClient
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False

class SimpleVPController:
    """簡化的VP控制器 - 基於實際VP_main.py的寄存器映射"""
    
    def __init__(self, host="127.0.0.1", port=502):
        self.host = host
        self.port = port
        self.client = None
        self.connected = False
        self.base_address = 300
        self.command_id = 1
        
        # VP動作映射 (基於VP_main.py)
        self.action_map = {
            'stop': 0, 'up': 1, 'down': 2, 'left': 3, 'right': 4,
            'upleft': 5, 'downleft': 6, 'upright': 7, 'downright': 8,
            'horizontal': 9, 'vertical': 10, 'spread': 11
        }
        
        self.status_callbacks = []
    
    def add_status_callback(self, callback):
        """添加狀態回調"""
        self.status_callbacks.append(callback)
    
    def connect(self):
        """連接到Modbus服務器"""
        if not MODBUS_AVAILABLE:
            return False
            
        try:
            self.client = ModbusTcpClient(host=self.host, port=self.port, timeout=1.0)
            self.connected = self.client.connect()
            return self.connected
        except Exception as e:
            print(f"VP連接失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.client:
            self.client.close()
            self.connected = False
    
    def send_command(self, cmd_code, param1=0, param2=0, param3=0):
        """發送指令到VP模組"""
        if not self.connected:
            return False
            
        try:
            # 基於VP_main.py的指令寄存器映射
            self.client.write_register(address=self.base_address + 20, value=cmd_code, slave=1)
            self.client.write_register(address=self.base_address + 21, value=param1, slave=1)
            self.client.write_register(address=self.base_address + 22, value=param2, slave=1)
            self.client.write_register(address=self.base_address + 23, value=param3, slave=1)
            self.client.write_register(address=self.base_address + 24, value=self.command_id, slave=1)
            
            self.command_id += 1
            if self.command_id > 65535:
                self.command_id = 1
                
            return True
        except Exception as e:
            print(f"VP指令發送失敗: {e}")
            return False
    
    def get_status(self):
        """獲取VP狀態"""
        if not self.connected:
            return {}
            
        try:
            result = self.client.read_holding_registers(address=self.base_address, count=15, slave=1)
            if hasattr(result, 'registers') and not result.isError():
                status_data = result.registers
                return {
                    'module_status': status_data[0],
                    'device_connection': status_data[1],
                    'device_status': status_data[2],
                    'error_code': status_data[3],
                    'command_status': status_data[8],
                    'brightness_status': status_data[10],
                    'backlight_status': status_data[11],
                    'vibration_status': status_data[12],
                    'frequency_status': status_data[13],
                }
        except Exception:
            pass
        return {}

class SimpleLightController:
    """簡化的Light控制器 - 基於實際LED_main.py的寄存器映射"""
    
    def __init__(self, host="127.0.0.1", port=502):
        self.host = host
        self.port = port
        self.client = None
        self.connected = False
        self.base_address = 600
        self.command_id = 1
        self.status_callbacks = []
    
    def add_status_callback(self, callback):
        """添加狀態回調"""
        self.status_callbacks.append(callback)
    
    def connect(self):
        """連接到Modbus服務器"""
        if not MODBUS_AVAILABLE:
            return False
            
        try:
            self.client = ModbusTcpClient(host=self.host, port=self.port, timeout=1.0)
            self.connected = self.client.connect()
            return self.connected
        except Exception as e:
            print(f"Light連接失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.client:
            self.client.close()
            self.connected = False
    
    def send_command(self, cmd_code, param1=0, param2=0):
        """發送指令到Light模組"""
        if not self.connected:
            return False
            
        try:
            # 基於LED_main.py的指令寄存器映射  
            self.client.write_register(address=self.base_address + 20, value=cmd_code, slave=1)
            self.client.write_register(address=self.base_address + 21, value=param1, slave=1)
            self.client.write_register(address=self.base_address + 22, value=param2, slave=1)
            self.client.write_register(address=self.base_address + 23, value=self.command_id, slave=1)
            
            self.command_id += 1
            if self.command_id > 65535:
                self.command_id = 1
                
            return True
        except Exception as e:
            print(f"Light指令發送失敗: {e}")
            return False
    
    def get_status(self):
        """獲取Light狀態"""
        if not self.connected:
            return {}
            
        try:
            result = self.client.read_holding_registers(address=self.base_address, count=16, slave=1)
            if hasattr(result, 'registers') and not result.isError():
                status_data = result.registers
                return {
                    'module_status': status_data[0],
                    'device_connection': status_data[1],
                    'active_channels': status_data[2],
                    'error_code': status_data[3],
                    'l1_state': status_data[4],
                    'l2_state': status_data[5],
                    'l3_state': status_data[6],
                    'l4_state': status_data[7],
                    'l1_brightness': status_data[8],
                    'l2_brightness': status_data[9],
                    'l3_brightness': status_data[10],
                    'l4_brightness': status_data[11],
                    'operation_count': status_data[12],
                    'error_count': status_data[13],
                }
        except Exception:
            pass
        return {}

class VPControlFrame(ctk.CTkFrame):
    """VP震動盤控制框架"""
    
    def __init__(self, master, vp_controller):
        super().__init__(master)
        self.vp_controller = vp_controller
        self.setup_ui()
    
    def setup_ui(self):
        """設置UI介面"""
        # 標題
        title_label = ctk.CTkLabel(self, text="VP震動盤控制", font=ctk.CTkFont(size=20, weight="bold"))
        title_label.pack(pady=(10, 20))
        
        # 狀態顯示區域
        status_frame = ctk.CTkFrame(self)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(status_frame, text="狀態資訊", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        # 狀態網格
        status_grid = ctk.CTkFrame(status_frame)
        status_grid.pack(pady=5)
        
        # 狀態標籤
        row1 = ctk.CTkFrame(status_grid)
        row1.pack(fill="x", pady=2)
        
        ctk.CTkLabel(row1, text="連接:").pack(side="left", padx=5)
        self.connection_status = ctk.CTkLabel(row1, text="離線", text_color="red")
        self.connection_status.pack(side="left", padx=5)
        
        ctk.CTkLabel(row1, text="設備:").pack(side="left", padx=20)
        self.device_status = ctk.CTkLabel(row1, text="未知")
        self.device_status.pack(side="left", padx=5)
        
        ctk.CTkLabel(row1, text="震動:").pack(side="left", padx=20)
        self.vibration_status = ctk.CTkLabel(row1, text="停止")
        self.vibration_status.pack(side="left", padx=5)
        
        # 背光控制區域
        backlight_frame = ctk.CTkFrame(self)
        backlight_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(backlight_frame, text="背光控制", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        # 背光按鈕
        backlight_buttons = ctk.CTkFrame(backlight_frame)
        backlight_buttons.pack(pady=5)
        
        ctk.CTkButton(backlight_buttons, text="開啟", fg_color="green", width=80, command=self.enable_backlight).pack(side="left", padx=5)
        ctk.CTkButton(backlight_buttons, text="關閉", fg_color="red", width=80, command=self.disable_backlight).pack(side="left", padx=5)
        
        # 亮度控制
        brightness_frame = ctk.CTkFrame(backlight_frame)
        brightness_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(brightness_frame, text="亮度:").pack(side="left")
        self.brightness_slider = ctk.CTkSlider(brightness_frame, from_=0, to=255, command=self.set_brightness)
        self.brightness_slider.set(28)
        self.brightness_slider.pack(side="left", fill="x", expand=True, padx=10)
        
        self.brightness_label = ctk.CTkLabel(brightness_frame, text="28", width=40)
        self.brightness_label.pack(side="right")
        
        # 震動控制區域
        vibration_frame = ctk.CTkFrame(self)
        vibration_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(vibration_frame, text="震動模式", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        # 9宮格方向控制
        direction_grid = ctk.CTkFrame(vibration_frame)
        direction_grid.pack(pady=10)
        
        # 第一行
        row1 = ctk.CTkFrame(direction_grid)
        row1.pack(pady=2)
        ctk.CTkButton(row1, text="↖", width=50, height=35, command=lambda: self.execute_action('upleft')).pack(side="left", padx=2)
        ctk.CTkButton(row1, text="↑", width=50, height=35, command=lambda: self.execute_action('up')).pack(side="left", padx=2)
        ctk.CTkButton(row1, text="↗", width=50, height=35, command=lambda: self.execute_action('upright')).pack(side="left", padx=2)
        
        # 第二行
        row2 = ctk.CTkFrame(direction_grid)
        row2.pack(pady=2)
        ctk.CTkButton(row2, text="←", width=50, height=35, command=lambda: self.execute_action('left')).pack(side="left", padx=2)
        ctk.CTkButton(row2, text="停", width=50, height=35, fg_color="red", command=lambda: self.execute_action('stop')).pack(side="left", padx=2)
        ctk.CTkButton(row2, text="→", width=50, height=35, command=lambda: self.execute_action('right')).pack(side="left", padx=2)
        
        # 第三行
        row3 = ctk.CTkFrame(direction_grid)
        row3.pack(pady=2)
        ctk.CTkButton(row3, text="↙", width=50, height=35, command=lambda: self.execute_action('downleft')).pack(side="left", padx=2)
        ctk.CTkButton(row3, text="↓", width=50, height=35, command=lambda: self.execute_action('down')).pack(side="left", padx=2)
        ctk.CTkButton(row3, text="↘", width=50, height=35, command=lambda: self.execute_action('downright')).pack(side="left", padx=2)
        
        # 特殊模式
        special_modes = ctk.CTkFrame(vibration_frame)
        special_modes.pack(pady=10)
        
        ctk.CTkButton(special_modes, text="水平", command=lambda: self.execute_action('horizontal')).pack(side="left", padx=5)
        ctk.CTkButton(special_modes, text="垂直", command=lambda: self.execute_action('vertical')).pack(side="left", padx=5)
        ctk.CTkButton(special_modes, text="散開", command=lambda: self.execute_action('spread')).pack(side="left", padx=5)
        
        # 參數控制
        params_frame = ctk.CTkFrame(vibration_frame)
        params_frame.pack(fill="x", padx=10, pady=10)
        
        # 強度
        strength_frame = ctk.CTkFrame(params_frame)
        strength_frame.pack(fill="x", pady=2)
        
        ctk.CTkLabel(strength_frame, text="強度:").pack(side="left")
        self.strength_slider = ctk.CTkSlider(strength_frame, from_=0, to=100, command=self.update_strength_label)
        self.strength_slider.set(50)
        self.strength_slider.pack(side="left", fill="x", expand=True, padx=10)
        
        self.strength_label = ctk.CTkLabel(strength_frame, text="50", width=40)
        self.strength_label.pack(side="right")
        
        # 頻率
        frequency_frame = ctk.CTkFrame(params_frame)
        frequency_frame.pack(fill="x", pady=2)
        
        ctk.CTkLabel(frequency_frame, text="頻率:").pack(side="left")
        self.frequency_slider = ctk.CTkSlider(frequency_frame, from_=1, to=255, command=self.update_frequency_label)
        self.frequency_slider.set(128)
        self.frequency_slider.pack(side="left", fill="x", expand=True, padx=10)
        
        self.frequency_label = ctk.CTkLabel(frequency_frame, text="128", width=40)
        self.frequency_label.pack(side="right")
    
    def update_strength_label(self, value):
        """更新強度標籤"""
        self.strength_label.configure(text=str(int(value)))
    
    def update_frequency_label(self, value):
        """更新頻率標籤"""
        self.frequency_label.configure(text=str(int(value)))
    
    def enable_backlight(self):
        """啟用背光"""
        self.vp_controller.send_command(1)  # enable_device
    
    def disable_backlight(self):
        """停用背光"""
        self.vp_controller.send_command(2)  # disable_device
    
    def set_brightness(self, value):
        """設置亮度"""
        brightness = int(value)
        self.brightness_label.configure(text=str(brightness))
        self.vp_controller.send_command(4, brightness)  # set_brightness
    
    def execute_action(self, action):
        """執行震動動作"""
        action_code = self.vp_controller.action_map.get(action, 0)
        strength = int(self.strength_slider.get())
        frequency = int(self.frequency_slider.get())
        
        # 發送執行動作指令 (指令5: execute_action)
        self.vp_controller.send_command(5, action_code, strength, frequency)
    
    def update_status(self):
        """更新狀態顯示"""
        if not self.vp_controller.connected:
            self.connection_status.configure(text="離線", text_color="red")
            return
        
        try:
            status = self.vp_controller.get_status()
            if status:
                # 連接狀態
                if status.get('device_connection', 0) == 1:
                    self.connection_status.configure(text="已連接", text_color="green")
                else:
                    self.connection_status.configure(text="斷開", text_color="orange")
                
                # 設備狀態
                module_status = status.get('module_status', 0)
                status_text = ["離線", "閒置", "執行中", "初始化", "錯誤"][min(module_status, 4)]
                self.device_status.configure(text=status_text)
                
                # 震動狀態
                vibration_active = status.get('vibration_status', 0)
                self.vibration_status.configure(text="震動中" if vibration_active else "停止")
                
        except Exception as e:
            print(f"更新VP狀態失敗: {e}")

class LightControlFrame(ctk.CTkFrame):
    """Light LED控制框架"""
    
    def __init__(self, master, light_controller):
        super().__init__(master)
        self.light_controller = light_controller
        self.setup_ui()
    
    def setup_ui(self):
        """設置UI介面"""
        # 標題
        title_label = ctk.CTkLabel(self, text="LED照明控制", font=ctk.CTkFont(size=20, weight="bold"))
        title_label.pack(pady=(10, 20))
        
        # 狀態顯示
        status_frame = ctk.CTkFrame(self)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(status_frame, text="系統狀態", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        # 狀態網格
        status_grid = ctk.CTkFrame(status_frame)
        status_grid.pack(pady=5)
        
        row1 = ctk.CTkFrame(status_grid)
        row1.pack(fill="x", pady=2)
        
        ctk.CTkLabel(row1, text="連接:").pack(side="left", padx=5)
        self.connection_status = ctk.CTkLabel(row1, text="離線", text_color="red")
        self.connection_status.pack(side="left", padx=5)
        
        ctk.CTkLabel(row1, text="開啟通道:").pack(side="left", padx=20)
        self.active_channels_label = ctk.CTkLabel(row1, text="0")
        self.active_channels_label.pack(side="left", padx=5)
        
        # 全域控制
        global_frame = ctk.CTkFrame(self)
        global_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(global_frame, text="全域控制", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        global_buttons = ctk.CTkFrame(global_frame)
        global_buttons.pack(pady=5)
        
        ctk.CTkButton(global_buttons, text="全開", fg_color="green", width=80, command=self.all_on).pack(side="left", padx=5)
        ctk.CTkButton(global_buttons, text="全關", fg_color="red", width=80, command=self.all_off).pack(side="left", padx=5)
        ctk.CTkButton(global_buttons, text="重置", fg_color="orange", width=80, command=self.reset_device).pack(side="left", padx=5)
        
        # 通道控制
        channels_frame = ctk.CTkFrame(self)
        channels_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ctk.CTkLabel(channels_frame, text="通道控制", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        # 創建4個通道
        self.channel_controls = []
        for i in range(4):
            channel_frame = self.create_channel_control(channels_frame, i + 1)
            channel_frame.pack(fill="x", padx=5, pady=3)
    
    def create_channel_control(self, parent, channel_num):
        """創建單一通道控制"""
        frame = ctk.CTkFrame(parent)
        
        # 通道標題和狀態
        header = ctk.CTkFrame(frame)
        header.pack(fill="x", padx=5, pady=3)
        
        ctk.CTkLabel(header, text=f"L{channel_num}", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left")
        
        status_label = ctk.CTkLabel(header, text="●", font=ctk.CTkFont(size=16), text_color="gray")
        status_label.pack(side="right")
        
        # 控制區域
        control = ctk.CTkFrame(frame)
        control.pack(fill="x", padx=5, pady=3)
        
        # 開關按鈕
        ctk.CTkButton(control, text="開", width=50, fg_color="green", 
                      command=lambda: self.channel_on(channel_num)).pack(side="left", padx=2)
        ctk.CTkButton(control, text="關", width=50, fg_color="red",
                      command=lambda: self.channel_off(channel_num)).pack(side="left", padx=2)
        
        # 亮度控制
        brightness_frame = ctk.CTkFrame(control)
        brightness_frame.pack(side="right", fill="x", expand=True, padx=(10, 0))
        
        ctk.CTkLabel(brightness_frame, text="亮度:").pack(side="left")
        
        brightness_slider = ctk.CTkSlider(brightness_frame, from_=0, to=511,
                                          command=lambda v, ch=channel_num: self.set_channel_brightness(ch, v))
        brightness_slider.set(255)
        brightness_slider.pack(side="left", fill="x", expand=True, padx=5)
        
        brightness_label = ctk.CTkLabel(brightness_frame, text="255", width=40)
        brightness_label.pack(side="left")
        
        # 保存控制元件
        control_dict = {
            'frame': frame,
            'status_label': status_label,
            'brightness_slider': brightness_slider,
            'brightness_label': brightness_label
        }
        self.channel_controls.append(control_dict)
        
        return frame
    
    def all_on(self):
        """全部開啟"""
        self.light_controller.send_command(1)
    
    def all_off(self):
        """全部關閉"""
        self.light_controller.send_command(2)
    
    def reset_device(self):
        """重置設備"""
        self.light_controller.send_command(3)
    
    def channel_on(self, channel):
        """開啟通道"""
        self.light_controller.send_command(5, channel)
    
    def channel_off(self, channel):
        """關閉通道"""
        self.light_controller.send_command(6, channel)
    
    def set_channel_brightness(self, channel, value):
        """設置通道亮度"""
        brightness = int(value)
        self.channel_controls[channel-1]['brightness_label'].configure(text=str(brightness))
        self.light_controller.send_command(4, channel, brightness)
    
    def update_status(self):
        """更新狀態顯示"""
        if not self.light_controller.connected:
            self.connection_status.configure(text="離線", text_color="red")
            return
        
        try:
            status = self.light_controller.get_status()
            if status:
                # 連接狀態
                if status.get('device_connection', 0) == 1:
                    self.connection_status.configure(text="已連接", text_color="green")
                else:
                    self.connection_status.configure(text="斷開", text_color="orange")
                
                # 開啟通道數
                active_channels = status.get('active_channels', 0)
                self.active_channels_label.configure(text=str(active_channels))
                
                # 更新通道狀態指示燈
                for i in range(4):
                    channel_state = status.get(f'l{i+1}_state', 0)
                    if channel_state == 1:
                        self.channel_controls[i]['status_label'].configure(text_color="green")
                    else:
                        self.channel_controls[i]['status_label'].configure(text_color="gray")
                        
        except Exception as e:
            print(f"更新LED狀態失敗: {e}")

class MainApplication(ctk.CTk):
    """主應用程式 - 基於實際GUIClass架構的調機介面"""
    
    def __init__(self):
        super().__init__()
        
        self.title("工業自動化調機工具 - VP震動盤 & LED照明")
        self.geometry("1400x900")
        
        # 初始化控制器
        self.vp_controller = SimpleVPController()
        self.light_controller = SimpleLightController()
        
        # 可擴展的GUIClass控制器 (如果可用)
        self.ccd1_controller = None
        self.ccd2_controller = None
        self.ccd3_controller = None
        
        if GUICLASS_AVAILABLE:
            try:
                self.ccd1_controller = CCD1GUIClass()
                self.ccd2_controller = CCD2GUIClass()
                self.ccd3_controller = CCD3GUIClass()
                print("視覺模組控制器初始化成功")
            except Exception as e:
                print(f"視覺模組控制器初始化失敗: {e}")
        
        self.setup_ui()
        self.start_status_update_thread()
    
    def setup_ui(self):
        """設置主介面"""
        # 主佈局
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        
        # 側邊欄
        self.sidebar = ctk.CTkFrame(self, width=250)
        self.sidebar.grid(row=0, column=0, sticky="nsew", padx=(10, 5), pady=10)
        self.sidebar.grid_propagate(False)
        
        # 側邊欄內容
        self.setup_sidebar()
        
        # 主內容區域
        self.main_content = ctk.CTkFrame(self)
        self.main_content.grid(row=0, column=1, sticky="nsew", padx=(5, 10), pady=10)
        
        # 創建頁面
        self.vp_frame = VPControlFrame(self.main_content, self.vp_controller)
        self.light_frame = LightControlFrame(self.main_content, self.light_controller)
        
        # 默認顯示VP頁面
        self.show_vp_control()
    
    def setup_sidebar(self):
        """設置側邊欄"""
        # 標題
        title = ctk.CTkLabel(self.sidebar, text="調機工具", font=ctk.CTkFont(size=20, weight="bold"))
        title.pack(pady=(20, 30))
        
        # 連接設定
        connection_frame = ctk.CTkFrame(self.sidebar)
        connection_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(connection_frame, text="Modbus連接", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=5)
        
        self.ip_entry = ctk.CTkEntry(connection_frame, placeholder_text="IP地址")
        self.ip_entry.pack(padx=5, pady=2, fill="x")
        self.ip_entry.insert(0, "127.0.0.1")
        
        self.port_entry = ctk.CTkEntry(connection_frame, placeholder_text="端口")
        self.port_entry.pack(padx=5, pady=2, fill="x")
        self.port_entry.insert(0, "502")
        
        self.connect_button = ctk.CTkButton(connection_frame, text="連接", command=self.toggle_connection)
        self.connect_button.pack(padx=5, pady=5)
        
        # 連接狀態
        status_frame = ctk.CTkFrame(connection_frame)
        status_frame.pack(fill="x", padx=5, pady=5)
        
        vp_row = ctk.CTkFrame(status_frame)
        vp_row.pack(fill="x", pady=1)
        ctk.CTkLabel(vp_row, text="VP:").pack(side="left")
        self.vp_status_label = ctk.CTkLabel(vp_row, text="未連接", text_color="red")
        self.vp_status_label.pack(side="right")
        
        light_row = ctk.CTkFrame(status_frame)
        light_row.pack(fill="x", pady=1)
        ctk.CTkLabel(light_row, text="Light:").pack(side="left")
        self.light_status_label = ctk.CTkLabel(light_row, text="未連接", text_color="red")
        self.light_status_label.pack(side="right")
        
        # 模組選擇
        modules_frame = ctk.CTkFrame(self.sidebar)
        modules_frame.pack(fill="x", padx=10, pady=20)
        
        ctk.CTkLabel(modules_frame, text="模組選擇", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=5)
        
        self.vp_button = ctk.CTkButton(modules_frame, text="VP震動盤", command=self.show_vp_control)
        self.vp_button.pack(padx=5, pady=3, fill="x")
        
        self.light_button = ctk.CTkButton(modules_frame, text="LED照明", command=self.show_light_control)
        self.light_button.pack(padx=5, pady=3, fill="x")
        
        # 如果有GUIClass可用，添加視覺模組按鈕
        if GUICLASS_AVAILABLE:
            self.ccd1_button = ctk.CTkButton(modules_frame, text="CCD1視覺", state="disabled")
            self.ccd1_button.pack(padx=5, pady=3, fill="x")
            
            self.ccd2_button = ctk.CTkButton(modules_frame, text="CCD2分類", state="disabled") 
            self.ccd2_button.pack(padx=5, pady=3, fill="x")
        
        # 系統資訊
        info_frame = ctk.CTkFrame(self.sidebar)
        info_frame.pack(fill="x", padx=10, pady=20)
        
        ctk.CTkLabel(info_frame, text="系統資訊", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=5)
        
        info_text = ctk.CTkTextbox(info_frame, height=100)
        info_text.pack(padx=5, pady=5, fill="x")
        info_text.insert("1.0", f"VP基地址: 300-349\nLED基地址: 600-649\n\nGUIClass: {'可用' if GUICLASS_AVAILABLE else '不可用'}\nModbus: {'可用' if MODBUS_AVAILABLE else '不可用'}")
        info_text.configure(state="disabled")
        
        # 操作日誌
        log_frame = ctk.CTkFrame(self.sidebar)
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ctk.CTkLabel(log_frame, text="操作日誌", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=5)
        
        self.log_textbox = ctk.CTkTextbox(log_frame, height=200)
        self.log_textbox.pack(padx=5, pady=5, fill="both", expand=True)
        
        self.add_log("系統啟動完成")
        if GUICLASS_AVAILABLE:
            self.add_log("GUIClass模組已載入")
        if MODBUS_AVAILABLE:
            self.add_log("Modbus模組已載入")
    
    def toggle_connection(self):
        """切換連接狀態"""
        if not (self.vp_controller.connected and self.light_controller.connected):
            host = self.ip_entry.get() or "127.0.0.1"
            port = int(self.port_entry.get() or "502")
            
            # 更新控制器配置
            self.vp_controller.host = host
            self.vp_controller.port = port
            self.light_controller.host = host
            self.light_controller.port = port
            
            # 嘗試連接
            vp_ok = self.vp_controller.connect()
            light_ok = self.light_controller.connect()
            
            # 更新狀態
            self.vp_status_label.configure(
                text="已連接" if vp_ok else "失敗", 
                text_color="green" if vp_ok else "red"
            )
            self.light_status_label.configure(
                text="已連接" if light_ok else "失敗",
                text_color="green" if light_ok else "red" 
            )
            
            if vp_ok or light_ok:
                self.connect_button.configure(text="斷開", fg_color="red")
                self.add_log(f"連接成功 - VP:{'✓' if vp_ok else '✗'} Light:{'✓' if light_ok else '✗'}")
            else:
                self.add_log("所有模組連接失敗")
        else:
            # 斷開連接
            self.vp_controller.disconnect()
            self.light_controller.disconnect()
            
            self.connect_button.configure(text="連接", fg_color="#1f538d")
            self.vp_status_label.configure(text="未連接", text_color="red")
            self.light_status_label.configure(text="未連接", text_color="red")
            
            self.add_log("已斷開所有連接")
    
    def show_vp_control(self):
        """顯示VP控制頁面"""
        self.light_frame.pack_forget()
        self.vp_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.vp_button.configure(fg_color="#1f538d")
        self.light_button.configure(fg_color="gray")
        
        self.add_log("切換到VP震動盤控制")
    
    def show_light_control(self):
        """顯示LED控制頁面"""
        self.vp_frame.pack_forget()
        self.light_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.light_button.configure(fg_color="#1f538d")
        self.vp_button.configure(fg_color="gray")
        
        self.add_log("切換到LED照明控制")
    
    def add_log(self, message):
        """添加日誌"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        self.log_textbox.insert("end", log_entry)
        self.log_textbox.see("end")
    
    def start_status_update_thread(self):
        """啟動狀態更新線程"""
        def update_loop():
            while True:
                try:
                    if self.vp_controller.connected:
                        self.vp_frame.update_status()
                    
                    if self.light_controller.connected:
                        self.light_frame.update_status()
                        
                except Exception as e:
                    print(f"狀態更新異常: {e}")
                    
                time.sleep(2)
        
        update_thread = threading.Thread(target=update_loop, daemon=True)
        update_thread.start()
        
        self.add_log("狀態監控已啟動")
    
    def on_closing(self):
        """程式關閉處理"""
        self.vp_controller.disconnect()
        self.light_controller.disconnect()
        
        if self.ccd1_controller:
            self.ccd1_controller.disconnect()
        if self.ccd2_controller:
            self.ccd2_controller.disconnect()
        if self.ccd3_controller:
            self.ccd3_controller.disconnect()
            
        self.add_log("系統正在關閉...")
        self.destroy()

if __name__ == "__main__":
    try:
        app = MainApplication()
        app.protocol("WM_DELETE_WINDOW", app.on_closing)
        app.mainloop()
    except Exception as e:
        print(f"應用程式啟動失敗: {e}")
        import traceback
        traceback.print_exc()