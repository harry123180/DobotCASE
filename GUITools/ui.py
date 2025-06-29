import customtkinter as ctk
import tkinter as tk
from typing import Callable, Dict, Any, Optional
import time

# 設定CTK外觀
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class VPControlPanel(ctk.CTkFrame):
    """VP震動盤控制面板 - 純UI層"""
    
    def __init__(self, master, callbacks: Dict[str, Callable] = None):
        super().__init__(master)
        self.callbacks = callbacks or {}
        self.setup_ui()
    
    def setup_ui(self):
        """設置VP控制UI"""
        # 標題
        title_label = ctk.CTkLabel(self, text="VP震動盤控制", font=ctk.CTkFont(size=20, weight="bold"))
        title_label.pack(pady=(10, 20))
        
        # 狀態顯示區
        self.status_frame = ctk.CTkFrame(self)
        self.status_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(self.status_frame, text="狀態資訊", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        status_grid = ctk.CTkFrame(self.status_frame)
        status_grid.pack(pady=5)
        
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
        
        # 背光控制區
        backlight_frame = ctk.CTkFrame(self)
        backlight_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(backlight_frame, text="背光控制", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        backlight_buttons = ctk.CTkFrame(backlight_frame)
        backlight_buttons.pack(pady=5)
        
        self.backlight_on_btn = ctk.CTkButton(backlight_buttons, text="開啟", fg_color="green", width=80,
                                              command=lambda: self._call_callback('backlight_on'))
        self.backlight_on_btn.pack(side="left", padx=5)
        
        self.backlight_off_btn = ctk.CTkButton(backlight_buttons, text="關閉", fg_color="red", width=80,
                                               command=lambda: self._call_callback('backlight_off'))
        self.backlight_off_btn.pack(side="left", padx=5)
        
        # 亮度控制
        brightness_frame = ctk.CTkFrame(backlight_frame)
        brightness_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(brightness_frame, text="亮度:").pack(side="left")
        self.brightness_slider = ctk.CTkSlider(brightness_frame, from_=0, to=255, 
                                               command=self._on_brightness_change)
        self.brightness_slider.set(28)
        self.brightness_slider.pack(side="left", fill="x", expand=True, padx=10)
        
        self.brightness_label = ctk.CTkLabel(brightness_frame, text="28", width=40)
        self.brightness_label.pack(side="right")
        
        # 震動控制區
        vibration_frame = ctk.CTkFrame(self)
        vibration_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(vibration_frame, text="震動模式", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        # 9宮格方向控制
        direction_grid = ctk.CTkFrame(vibration_frame)
        direction_grid.pack(pady=10)
        
        # 方向按鈕配置
        directions = [
            ['upleft', 'up', 'upright'],
            ['left', 'stop', 'right'],
            ['downleft', 'down', 'downright']
        ]
        
        direction_symbols = [
            ['↖', '↑', '↗'],
            ['←', '停', '→'],
            ['↙', '↓', '↘']
        ]
        
        for i, (dir_row, symbol_row) in enumerate(zip(directions, direction_symbols)):
            row_frame = ctk.CTkFrame(direction_grid)
            row_frame.pack(pady=2)
            
            for j, (direction, symbol) in enumerate(zip(dir_row, symbol_row)):
                color = "red" if direction == 'stop' else None
                btn = ctk.CTkButton(row_frame, text=symbol, width=50, height=35, fg_color=color,
                                    command=lambda d=direction: self._call_callback('execute_action', d))
                btn.pack(side="left", padx=2)
        
        # 特殊模式按鈕
        special_modes = ctk.CTkFrame(vibration_frame)
        special_modes.pack(pady=10)
        
        special_actions = [('horizontal', '水平'), ('vertical', '垂直'), ('spread', '散開')]
        for action, text in special_actions:
            btn = ctk.CTkButton(special_modes, text=text,
                                command=lambda a=action: self._call_callback('execute_action', a))
            btn.pack(side="left", padx=5)
        
        # 參數控制區
        params_frame = ctk.CTkFrame(vibration_frame)
        params_frame.pack(fill="x", padx=10, pady=10)
        
        # 強度控制
        strength_frame = ctk.CTkFrame(params_frame)
        strength_frame.pack(fill="x", pady=2)
        
        ctk.CTkLabel(strength_frame, text="強度:").pack(side="left")
        self.strength_slider = ctk.CTkSlider(strength_frame, from_=0, to=100, 
                                             command=self._on_strength_change)
        self.strength_slider.set(50)
        self.strength_slider.pack(side="left", fill="x", expand=True, padx=10)
        
        self.strength_label = ctk.CTkLabel(strength_frame, text="50", width=40)
        self.strength_label.pack(side="right")
        
        # 頻率控制
        frequency_frame = ctk.CTkFrame(params_frame)
        frequency_frame.pack(fill="x", pady=2)
        
        ctk.CTkLabel(frequency_frame, text="頻率:").pack(side="left")
        self.frequency_slider = ctk.CTkSlider(frequency_frame, from_=1, to=255, 
                                              command=self._on_frequency_change)
        self.frequency_slider.set(128)
        self.frequency_slider.pack(side="left", fill="x", expand=True, padx=10)
        
        self.frequency_label = ctk.CTkLabel(frequency_frame, text="128", width=40)
        self.frequency_label.pack(side="right")
    
    def _call_callback(self, callback_name: str, *args):
        """調用回調函數"""
        if callback_name in self.callbacks:
            self.callbacks[callback_name](*args)
    
    def _on_brightness_change(self, value):
        """亮度滑桿變化"""
        brightness = int(value)
        self.brightness_label.configure(text=str(brightness))
        self._call_callback('set_brightness', brightness)
    
    def _on_strength_change(self, value):
        """強度滑桿變化"""
        strength = int(value)
        self.strength_label.configure(text=str(strength))
    
    def _on_frequency_change(self, value):
        """頻率滑桿變化"""
        frequency = int(value)
        self.frequency_label.configure(text=str(frequency))
    
    def get_vibration_params(self):
        """獲取震動參數"""
        return {
            'strength': int(self.strength_slider.get()),
            'frequency': int(self.frequency_slider.get())
        }
    
    def update_status(self, status_data: Dict[str, Any]):
        """更新狀態顯示"""
        if status_data.get('device_connection') == 1:
            self.connection_status.configure(text="已連接", text_color="green")
        else:
            self.connection_status.configure(text="斷開", text_color="orange")
        
        module_status = status_data.get('module_status', 0)
        status_text = ["離線", "閒置", "執行中", "初始化", "錯誤"][min(module_status, 4)]
        self.device_status.configure(text=status_text)
        
        vibration_active = status_data.get('vibration_status', 0)
        self.vibration_status.configure(text="震動中" if vibration_active else "停止")

class LightControlPanel(ctk.CTkFrame):
    """LED照明控制面板 - 純UI層"""
    
    def __init__(self, master, callbacks: Dict[str, Callable] = None):
        super().__init__(master)
        self.callbacks = callbacks or {}
        self.setup_ui()
    
    def setup_ui(self):
        """設置LED控制UI"""
        # 標題
        title_label = ctk.CTkLabel(self, text="LED照明控制", font=ctk.CTkFont(size=20, weight="bold"))
        title_label.pack(pady=(10, 20))
        
        # 狀態顯示區
        status_frame = ctk.CTkFrame(self)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(status_frame, text="系統狀態", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
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
        
        # 全域控制區
        global_frame = ctk.CTkFrame(self)
        global_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(global_frame, text="全域控制", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        global_buttons = ctk.CTkFrame(global_frame)
        global_buttons.pack(pady=5)
        
        self.all_on_btn = ctk.CTkButton(global_buttons, text="全開", fg_color="green", width=80,
                                        command=lambda: self._call_callback('all_on'))
        self.all_on_btn.pack(side="left", padx=5)
        
        self.all_off_btn = ctk.CTkButton(global_buttons, text="全關", fg_color="red", width=80,
                                         command=lambda: self._call_callback('all_off'))
        self.all_off_btn.pack(side="left", padx=5)
        
        self.reset_btn = ctk.CTkButton(global_buttons, text="重置", fg_color="orange", width=80,
                                       command=lambda: self._call_callback('reset_device'))
        self.reset_btn.pack(side="left", padx=5)
        
        # 通道控制區
        channels_frame = ctk.CTkFrame(self)
        channels_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ctk.CTkLabel(channels_frame, text="通道控制", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        # 創建4個通道控制
        self.channel_controls = []
        for i in range(4):
            channel_frame = self._create_channel_control(channels_frame, i + 1)
            channel_frame.pack(fill="x", padx=5, pady=3)
    
    def _create_channel_control(self, parent, channel_num):
        """創建單個通道控制"""
        frame = ctk.CTkFrame(parent)
        
        # 通道標題
        header = ctk.CTkFrame(frame)
        header.pack(fill="x", padx=5, pady=3)
        
        ctk.CTkLabel(header, text=f"L{channel_num}", font=ctk.CTkFont(size=14, weight="bold")).pack(side="left")
        
        status_label = ctk.CTkLabel(header, text="●", font=ctk.CTkFont(size=16), text_color="gray")
        status_label.pack(side="right")
        
        # 通道控制
        control = ctk.CTkFrame(frame)
        control.pack(fill="x", padx=5, pady=3)
        
        on_btn = ctk.CTkButton(control, text="開", width=50, fg_color="green",
                               command=lambda: self._call_callback('channel_on', channel_num))
        on_btn.pack(side="left", padx=2)
        
        off_btn = ctk.CTkButton(control, text="關", width=50, fg_color="red",
                                command=lambda: self._call_callback('channel_off', channel_num))
        off_btn.pack(side="left", padx=2)
        
        # 亮度控制
        brightness_frame = ctk.CTkFrame(control)
        brightness_frame.pack(side="right", fill="x", expand=True, padx=(10, 0))
        
        ctk.CTkLabel(brightness_frame, text="亮度:").pack(side="left")
        
        brightness_slider = ctk.CTkSlider(brightness_frame, from_=0, to=511,
                                          command=lambda v, ch=channel_num: self._on_brightness_change(ch, v))
        brightness_slider.set(255)
        brightness_slider.pack(side="left", fill="x", expand=True, padx=5)
        
        brightness_label = ctk.CTkLabel(brightness_frame, text="255", width=40)
        brightness_label.pack(side="left")
        
        control_dict = {
            'frame': frame,
            'status_label': status_label,
            'brightness_slider': brightness_slider,
            'brightness_label': brightness_label,
            'on_btn': on_btn,
            'off_btn': off_btn
        }
        self.channel_controls.append(control_dict)
        
        return frame
    
    def _call_callback(self, callback_name: str, *args):
        """調用回調函數"""
        if callback_name in self.callbacks:
            self.callbacks[callback_name](*args)
    
    def _on_brightness_change(self, channel, value):
        """通道亮度變化"""
        brightness = int(value)
        self.channel_controls[channel-1]['brightness_label'].configure(text=str(brightness))
        self._call_callback('set_channel_brightness', channel, brightness)
    
    def update_status(self, status_data: Dict[str, Any]):
        """更新狀態顯示"""
        if status_data.get('device_connection') == 1:
            self.connection_status.configure(text="已連接", text_color="green")
        else:
            self.connection_status.configure(text="斷開", text_color="orange")
        
        active_channels = status_data.get('active_channels', 0)
        self.active_channels_label.configure(text=str(active_channels))
        
        # 更新各通道狀態
        for i in range(4):
            channel_state = status_data.get(f'l{i+1}_state', 0)
            if channel_state == 1:
                self.channel_controls[i]['status_label'].configure(text_color="green")
            else:
                self.channel_controls[i]['status_label'].configure(text_color="gray")

class GUIClassControlPanel(ctk.CTkFrame):
    """通用GUIClass控制面板 - 純UI層"""
    
    def __init__(self, master, module_name: str, callbacks: Dict[str, Callable] = None):
        super().__init__(master)
        self.module_name = module_name
        self.callbacks = callbacks or {}
        self.setup_ui()
    
    def setup_ui(self):
        """設置通用GUIClass控制UI"""
        # 標題
        title_label = ctk.CTkLabel(self, text=f"{self.module_name}控制", font=ctk.CTkFont(size=20, weight="bold"))
        title_label.pack(pady=(10, 20))
        
        # 連接狀態區
        status_frame = ctk.CTkFrame(self)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(status_frame, text="連接狀態", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        self.connection_status = ctk.CTkLabel(status_frame, text="未連接", text_color="red")
        self.connection_status.pack(pady=5)
        
        # 控制按鈕區
        control_frame = ctk.CTkFrame(self)
        control_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(control_frame, text="基本控制", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        self.buttons_frame = ctk.CTkFrame(control_frame)
        self.buttons_frame.pack(pady=5)
        
        # 根據模組類型創建按鈕
        self._create_module_buttons()
        
        # 狀態資訊區
        info_frame = ctk.CTkFrame(self)
        info_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ctk.CTkLabel(info_frame, text="狀態資訊", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        self.status_textbox = ctk.CTkTextbox(info_frame, height=200)
        self.status_textbox.pack(padx=5, pady=5, fill="both", expand=True)
    
    def _create_module_buttons(self):
        """根據模組類型創建控制按鈕"""
        if "CCD" in self.module_name:
            self._create_ccd_buttons()
        elif "Gripper" in self.module_name:
            self._create_gripper_buttons()
        elif "Dobot" in self.module_name:
            self._create_dobot_buttons()
    
    def _create_ccd_buttons(self):
        """創建CCD模組按鈕"""
        buttons = [
            ("初始化相機", "initialize_camera"),
            ("拍照", "capture_image"),
            ("拍照+檢測", "capture_and_detect"),
            ("清除指令", "clear_command", "orange")
        ]
        
        for button_data in buttons:
            text, callback = button_data[:2]
            color = button_data[2] if len(button_data) > 2 else None
            btn = ctk.CTkButton(self.buttons_frame, text=text, fg_color=color,
                                command=lambda cb=callback: self._call_callback(cb))
            btn.pack(side="left", padx=5)
    
    def _create_gripper_buttons(self):
        """創建夾爪模組按鈕"""
        buttons = [
            ("初始化所有夾爪", "initialize_all_grippers"),
            ("停止所有", "stop_all_grippers", "red"),
            ("全部開啟", "open_all_grippers", "green"),
            ("全部關閉", "close_all_grippers", "blue")
        ]
        
        for button_data in buttons:
            text, callback = button_data[:2]
            color = button_data[2] if len(button_data) > 2 else None
            btn = ctk.CTkButton(self.buttons_frame, text=text, fg_color=color,
                                command=lambda cb=callback: self._call_callback(cb))
            btn.pack(side="left", padx=5)
    
    def _create_dobot_buttons(self):
        """創建Dobot模組按鈕"""
        buttons = [
            ("連接機械臂", "connect_robot"),
            ("使能機械臂", "enable_robot", "green"),
            ("清除錯誤", "clear_error", "orange"),
            ("緊急停止", "emergency_stop", "red")
        ]
        
        for button_data in buttons:
            text, callback = button_data[:2]
            color = button_data[2] if len(button_data) > 2 else None
            btn = ctk.CTkButton(self.buttons_frame, text=text, fg_color=color,
                                command=lambda cb=callback: self._call_callback(cb))
            btn.pack(side="left", padx=5)
    
    def _call_callback(self, callback_name: str, *args):
        """調用回調函數"""
        if callback_name in self.callbacks:
            self.callbacks[callback_name](*args)
    
    def update_connection_status(self, connected: bool):
        """更新連接狀態"""
        if connected:
            self.connection_status.configure(text="已連接", text_color="green")
        else:
            self.connection_status.configure(text="未連接", text_color="red")
    
    def add_status_log(self, message: str):
        """添加狀態日誌"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        current_text = self.status_textbox.get("1.0", "end")
        self.status_textbox.delete("1.0", "end")
        self.status_textbox.insert("1.0", log_entry + current_text)
    
    def update_status_display(self, status_text: str):
        """更新狀態顯示"""
        self.status_textbox.delete("1.0", "end")
        self.status_textbox.insert("1.0", status_text)

class MainUI(ctk.CTk):
    """主介面UI層 - 純UI實現"""
    
    def __init__(self, callbacks: Dict[str, Callable] = None):
        super().__init__()
        
        self.callbacks = callbacks or {}
        self.current_panel = None
        
        self.title("統一機台調適工具 - 工業自動化控制系統")
        self.geometry("1600x1000")
        
        self.setup_ui()
    
    def setup_ui(self):
        """設置主UI界面"""
        # 主佈局
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        
        # 側邊欄
        self.sidebar = ctk.CTkScrollableFrame(self, width=280)
        self.sidebar.grid(row=0, column=0, sticky="nsew", padx=(10, 5), pady=10)
        
        # 主內容區域
        self.main_content = ctk.CTkFrame(self)
        self.main_content.grid(row=0, column=1, sticky="nsew", padx=(5, 10), pady=10)
        self.main_content.grid_columnconfigure(0, weight=1)
        self.main_content.grid_rowconfigure(0, weight=1)
        
        # 設置側邊欄
        self.setup_sidebar()
        
        # 初始化面板容器
        self.panels = {}
    
    def setup_sidebar(self):
        """設置側邊欄UI"""
        # 標題
        title = ctk.CTkLabel(self.sidebar, text="統一調機工具", font=ctk.CTkFont(size=22, weight="bold"))
        title.pack(pady=(20, 30))
        
        # 連接設定區
        connection_frame = ctk.CTkFrame(self.sidebar)
        connection_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(connection_frame, text="Modbus連接", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        self.ip_entry = ctk.CTkEntry(connection_frame, placeholder_text="IP地址")
        self.ip_entry.pack(padx=5, pady=2, fill="x")
        self.ip_entry.insert(0, "127.0.0.1")
        
        self.port_entry = ctk.CTkEntry(connection_frame, placeholder_text="端口")
        self.port_entry.pack(padx=5, pady=2, fill="x")
        self.port_entry.insert(0, "502")
        
        self.connect_button = ctk.CTkButton(connection_frame, text="連接",
                                            command=lambda: self._call_callback('toggle_connection'))
        self.connect_button.pack(padx=5, pady=5)
        
        # 基礎模組狀態顯示
        basic_status_frame = ctk.CTkFrame(connection_frame)
        basic_status_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(basic_status_frame, text="基礎模組狀態", font=ctk.CTkFont(size=12, weight="bold")).pack()
        
        vp_row = ctk.CTkFrame(basic_status_frame)
        vp_row.pack(fill="x", pady=1)
        ctk.CTkLabel(vp_row, text="VP:", font=ctk.CTkFont(size=11)).pack(side="left")
        self.vp_status_label = ctk.CTkLabel(vp_row, text="未連接", text_color="red", font=ctk.CTkFont(size=11))
        self.vp_status_label.pack(side="right")
        
        light_row = ctk.CTkFrame(basic_status_frame)
        light_row.pack(fill="x", pady=1)
        ctk.CTkLabel(light_row, text="Light:", font=ctk.CTkFont(size=11)).pack(side="left")
        self.light_status_label = ctk.CTkLabel(light_row, text="未連接", text_color="red", font=ctk.CTkFont(size=11))
        self.light_status_label.pack(side="right")
        
        # 模組選擇區
        modules_frame = ctk.CTkFrame(self.sidebar)
        modules_frame.pack(fill="x", padx=10, pady=20)
        
        ctk.CTkLabel(modules_frame, text="模組選擇", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        # 基礎模組按鈕
        ctk.CTkLabel(modules_frame, text="基礎模組", font=ctk.CTkFont(size=12, weight="bold")).pack(pady=(10, 5))
        
        self.vp_button = ctk.CTkButton(modules_frame, text="VP震動盤",
                                       command=lambda: self._call_callback('show_module', 'VP'))
        self.vp_button.pack(padx=5, pady=2, fill="x")
        
        self.light_button = ctk.CTkButton(modules_frame, text="LED照明",
                                          command=lambda: self._call_callback('show_module', 'Light'))
        self.light_button.pack(padx=5, pady=2, fill="x")
        
        # 進階模組按鈕 - 動態添加
        self.advanced_label = ctk.CTkLabel(modules_frame, text="進階模組", font=ctk.CTkFont(size=12, weight="bold"))
        self.module_buttons = {}
        
        # 系統資訊區
        info_frame = ctk.CTkFrame(self.sidebar)
        info_frame.pack(fill="x", padx=10, pady=20)
        
        ctk.CTkLabel(info_frame, text="系統資訊", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        self.info_textbox = ctk.CTkTextbox(info_frame, height=120)
        self.info_textbox.pack(padx=5, pady=5, fill="x")
        self.info_textbox.configure(state="disabled")
        
        # 操作日誌區
        log_frame = ctk.CTkFrame(self.sidebar)
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ctk.CTkLabel(log_frame, text="操作日誌", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=5)
        
        self.log_textbox = ctk.CTkTextbox(log_frame, height=250)
        self.log_textbox.pack(padx=5, pady=5, fill="both", expand=True)
    
    def _call_callback(self, callback_name: str, *args):
        """調用回調函數"""
        if callback_name in self.callbacks:
            self.callbacks[callback_name](*args)
    
    def get_connection_info(self) -> Dict[str, str]:
        """獲取連接資訊"""
        return {
            'host': self.ip_entry.get() or "127.0.0.1",
            'port': self.port_entry.get() or "502"
        }
    
    def add_advanced_module_button(self, module_name: str):
        """添加進階模組按鈕"""
        if module_name not in self.module_buttons:
            # 第一次添加時顯示標籤
            if not self.module_buttons:
                self.advanced_label.pack(pady=(15, 5))
            
            button = ctk.CTkButton(self.advanced_label.master, text=module_name,
                                   command=lambda: self._call_callback('show_module', module_name))
            button.pack(padx=5, pady=2, fill="x")
            self.module_buttons[module_name] = button
    
    def add_panel(self, panel_name: str, panel_widget):
        """添加控制面板"""
        self.panels[panel_name] = panel_widget
    
    def show_panel(self, panel_name: str):
        """顯示指定面板"""
        # 隱藏當前面板
        if self.current_panel:
            self.current_panel.grid_forget()
        
        # 顯示新面板
        if panel_name in self.panels:
            self.panels[panel_name].grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
            self.current_panel = self.panels[panel_name]
            
            # 重置按鈕顏色
            self.reset_button_colors()
            
            # 高亮當前按鈕
            if panel_name == 'VP':
                self.vp_button.configure(fg_color="#1f538d")
            elif panel_name == 'Light':
                self.light_button.configure(fg_color="#1f538d")
            elif panel_name in self.module_buttons:
                self.module_buttons[panel_name].configure(fg_color="#1f538d")
    
    def reset_button_colors(self):
        """重置所有按鈕顏色"""
        self.vp_button.configure(fg_color="gray")
        self.light_button.configure(fg_color="gray")
        for button in self.module_buttons.values():
            button.configure(fg_color="gray")
    
    def update_connection_status(self, vp_connected: bool, light_connected: bool):
        """更新連接狀態"""
        self.vp_status_label.configure(
            text="已連接" if vp_connected else "失敗",
            text_color="green" if vp_connected else "red"
        )
        self.light_status_label.configure(
            text="已連接" if light_connected else "失敗",
            text_color="green" if light_connected else "red"
        )
    
    def update_connect_button(self, connected: bool):
        """更新連接按鈕"""
        if connected:
            self.connect_button.configure(text="斷開", fg_color="red")
        else:
            self.connect_button.configure(text="連接", fg_color="gray")
    
    def update_system_info(self, info_text: str):
        """更新系統資訊"""
        self.info_textbox.configure(state="normal")
        self.info_textbox.delete("1.0", "end")
        self.info_textbox.insert("1.0", info_text)
        self.info_textbox.configure(state="disabled")
    
    def add_log(self, message: str):
        """添加日誌"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        self.log_textbox.insert("end", log_entry)
        self.log_textbox.see("end")
    
    def set_close_callback(self, callback: Callable):
        """設置關閉回調"""
        self.protocol("WM_DELETE_WINDOW", callback)