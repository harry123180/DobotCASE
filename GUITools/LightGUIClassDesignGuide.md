# LED控制器GUI類別設計指引

## 概述

`LightGUIClass.py`是LED控制器模組的GUI橋接層，提供統一的介面供主GUI工具整合。該類別封裝了所有LED控制功能，並提供狀態監控與回調機制。

## 架構設計

```
統一GUI工具
    |
    |-- import LightGUIClass
    |     |
    |     |-- ModbusTCP Client (127.0.0.1:502)
    |           |
    |           |-- LED_main.py (基地址600)
    |                 |
    |                 |-- RS232 --> LED控制器硬體
```

## 核心特性

### 1. 連接管理
- **自動重連**: 支援連接斷線檢測與重連
- **連接狀態監控**: 即時監控ModbusTCP連接狀態
- **資源清理**: 上下文管理器自動清理資源

### 2. 狀態監控
- **即時狀態讀取**: 自動讀取LED模組所有狀態寄存器
- **回調機制**: 支援多個狀態更新回調函數
- **自動更新**: 可配置的自動狀態更新循環

### 3. LED控制功能
- **通道控制**: 4通道獨立開關控制
- **亮度調整**: 0-511範圍亮度設定
- **批量操作**: 支援全部開啟/關閉、多通道批量設定
- **設備管理**: 設備重置、錯誤清除

## 類別介面規範

### 初始化參數
```python
LightGUIClass(
    host="127.0.0.1",    # ModbusTCP Server位址
    port=502,             # ModbusTCP Server端口
    unit_id=1,            # Modbus單元ID
    timeout=3.0           # 連接逾時時間
)
```

### 連接控制
- `connect() -> bool`: 連接到ModbusTCP Server
- `disconnect()`: 斷開連接
- `is_connected() -> bool`: 檢查連接狀態

### 狀態查詢
- `get_status() -> Dict[str, Any]`: 獲取完整狀態數據
- `get_module_status_text() -> str`: 獲取模組狀態文字描述
- `get_channel_states() -> Dict[int, bool]`: 獲取通道開關狀態
- `get_channel_brightness() -> Dict[int, int]`: 獲取通道亮度
- `is_device_connected() -> bool`: 檢查LED設備連接狀態
- `has_error() -> bool`: 檢查錯誤狀態
- `get_error_code() -> int`: 獲取錯誤代碼

### LED控制命令
- `turn_on_all() -> bool`: 全部開啟
- `turn_off_all() -> bool`: 全部關閉
- `reset_device() -> bool`: 重置設備
- `set_channel_brightness(channel: int, brightness: int) -> bool`: 設定通道亮度
- `turn_on_channel(channel: int) -> bool`: 開啟通道
- `turn_off_channel(channel: int) -> bool`: 關閉通道
- `clear_error() -> bool`: 清除錯誤

### 批量操作
- `set_all_brightness(brightness: int) -> bool`: 設定所有通道亮度
- `set_multiple_brightness(brightness_dict: Dict[int, int]) -> bool`: 批量設定亮度

### 狀態更新機制
- `add_status_callback(callback)`: 新增狀態回調函數
- `remove_status_callback(callback)`: 移除狀態回調函數
- `start_auto_update(interval: float)`: 開始自動狀態更新
- `stop_auto_update()`: 停止自動狀態更新
- `manual_update() -> bool`: 手動更新狀態

## GUI整合實作指引

### 1. 基本整合範例

```python
from LightGUIClass import LightGUIClass
import tkinter as tk
from tkinter import ttk

class MainGUITool:
    def __init__(self):
        self.root = tk.Tk()
        self.light_gui = LightGUIClass()
        self.setup_light_controls()
        
    def setup_light_controls(self):
        # LED控制面板
        light_frame = ttk.LabelFrame(self.root, text="LED控制器")
        light_frame.pack(padx=10, pady=5, fill="x")
        
        # 連接按鈕
        self.connect_btn = ttk.Button(
            light_frame, 
            text="連接", 
            command=self.connect_led
        )
        self.connect_btn.pack(side="left", padx=5)
        
        # 狀態顯示
        self.status_label = ttk.Label(light_frame, text="未連接")
        self.status_label.pack(side="left", padx=10)
        
        # 通道控制
        for channel in [1, 2, 3, 4]:
            self.create_channel_control(light_frame, channel)
    
    def create_channel_control(self, parent, channel):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=5, pady=2)
        
        # 通道標籤
        ttk.Label(frame, text=f"L{channel}:").pack(side="left")
        
        # 開關按鈕
        on_btn = ttk.Button(
            frame, 
            text="ON", 
            command=lambda: self.light_gui.turn_on_channel(channel)
        )
        on_btn.pack(side="left", padx=2)
        
        off_btn = ttk.Button(
            frame, 
            text="OFF", 
            command=lambda: self.light_gui.turn_off_channel(channel)
        )
        off_btn.pack(side="left", padx=2)
        
        # 亮度滑桿
        brightness_scale = ttk.Scale(
            frame, 
            from_=0, 
            to=511, 
            orient="horizontal",
            command=lambda v: self.light_gui.set_channel_brightness(
                channel, int(float(v))
            )
        )
        brightness_scale.pack(side="left", fill="x", expand=True, padx=5)
    
    def connect_led(self):
        if self.light_gui.connect():
            self.light_gui.add_status_callback(self.update_status_display)
            self.light_gui.start_auto_update(1.0)
            self.status_label.config(text="已連接")
        else:
            self.status_label.config(text="連接失敗")
    
    def update_status_display(self, status_data):
        # 更新GUI狀態顯示
        module_status = status_data['module_status']
        if module_status == 1:
            self.status_label.config(text="已連接 - 閒置")
        elif module_status == 2:
            self.status_label.config(text="已連接 - 執行中")
        elif module_status == 4:
            self.status_label.config(text="已連接 - 錯誤")
```

### 2. 狀態監控面板實作

```python
class LEDStatusPanel:
    def __init__(self, parent_frame, light_gui):
        self.light_gui = light_gui
        self.create_status_panel(parent_frame)
        
    def create_status_panel(self, parent):
        # 狀態面板
        status_frame = ttk.LabelFrame(parent, text="LED狀態監控")
        status_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # 模組狀態
        self.module_status_var = tk.StringVar(value="未知")
        ttk.Label(status_frame, text="模組狀態:").grid(row=0, column=0, sticky="w")
        ttk.Label(status_frame, textvariable=self.module_status_var).grid(row=0, column=1, sticky="w")
        
        # 設備連接狀態
        self.device_status_var = tk.StringVar(value="未知")
        ttk.Label(status_frame, text="設備連接:").grid(row=1, column=0, sticky="w")
        ttk.Label(status_frame, textvariable=self.device_status_var).grid(row=1, column=1, sticky="w")
        
        # 通道狀態表格
        self.create_channel_table(status_frame)
        
        # 註冊狀態回調
        self.light_gui.add_status_callback(self.update_display)
    
    def create_channel_table(self, parent):
        # 通道狀態表格
        columns = ("通道", "狀態", "亮度")
        self.tree = ttk.Treeview(parent, columns=columns, show="headings", height=4)
        
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=80)
        
        # 插入通道項目
        for channel in [1, 2, 3, 4]:
            self.tree.insert("", "end", iid=f"L{channel}", 
                           values=(f"L{channel}", "OFF", "0"))
        
        self.tree.grid(row=2, column=0, columnspan=2, pady=10)
    
    def update_display(self, status_data):
        # 更新模組狀態
        status_text = self.get_status_text(status_data['module_status'])
        self.module_status_var.set(status_text)
        
        # 更新設備狀態
        device_text = "已連接" if status_data['device_connection'] else "斷開"
        self.device_status_var.set(device_text)
        
        # 更新通道狀態
        channels_data = [
            ('L1', status_data['l1_state'], status_data['l1_brightness']),
            ('L2', status_data['l2_state'], status_data['l2_brightness']),
            ('L3', status_data['l3_state'], status_data['l3_brightness']),
            ('L4', status_data['l4_state'], status_data['l4_brightness'])
        ]
        
        for channel, state, brightness in channels_data:
            state_text = "ON" if state else "OFF"
            self.tree.set(channel, "狀態", state_text)
            self.tree.set(channel, "亮度", str(brightness))
    
    def get_status_text(self, status_code):
        status_map = {0: "離線", 1: "閒置", 2: "執行中", 3: "初始化", 4: "錯誤"}
        return status_map.get(status_code, "未知")
```

### 3. 進階控制面板實作

```python
class LEDAdvancedPanel:
    def __init__(self, parent_frame, light_gui):
        self.light_gui = light_gui
        self.create_advanced_panel(parent_frame)
        
    def create_advanced_panel(self, parent):
        # 進階控制面板
        adv_frame = ttk.LabelFrame(parent, text="LED進階控制")
        adv_frame.pack(fill="x", padx=5, pady=5)
        
        # 全域控制
        global_frame = ttk.Frame(adv_frame)
        global_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Button(global_frame, text="全部開啟", 
                  command=self.light_gui.turn_on_all).pack(side="left", padx=2)
        ttk.Button(global_frame, text="全部關閉", 
                  command=self.light_gui.turn_off_all).pack(side="left", padx=2)
        ttk.Button(global_frame, text="重置設備", 
                  command=self.light_gui.reset_device).pack(side="left", padx=2)
        ttk.Button(global_frame, text="清除錯誤", 
                  command=self.light_gui.clear_error).pack(side="left", padx=2)
        
        # 批量亮度設定
        batch_frame = ttk.Frame(adv_frame)
        batch_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(batch_frame, text="批量亮度:").pack(side="left")
        self.batch_brightness = tk.IntVar(value=255)
        batch_scale = ttk.Scale(batch_frame, from_=0, to=511, 
                               variable=self.batch_brightness, 
                               orient="horizontal")
        batch_scale.pack(side="left", fill="x", expand=True, padx=5)
        
        ttk.Button(batch_frame, text="套用", 
                  command=self.apply_batch_brightness).pack(side="right", padx=2)
    
    def apply_batch_brightness(self):
        brightness = self.batch_brightness.get()
        self.light_gui.set_all_brightness(brightness)
```

## 錯誤處理指引

### 1. 連接錯誤處理
```python
def safe_connect(light_gui):
    try:
        if light_gui.connect():
            return True
        else:
            # 顯示錯誤訊息
            messagebox.showerror("連接錯誤", "無法連接到LED控制器模組")
            return False
    except Exception as e:
        messagebox.showerror("連接異常", f"連接過程發生異常: {e}")
        return False
```

### 2. 指令執行錯誤處理
```python
def safe_command(command_func, *args, **kwargs):
    try:
        result = command_func(*args, **kwargs)
        if not result:
            messagebox.showwarning("指令失敗", "LED指令執行失敗")
        return result
    except Exception as e:
        messagebox.showerror("指令異常", f"指令執行異常: {e}")
        return False
```

### 3. 狀態更新錯誤處理
```python
def robust_status_callback(status_data):
    try:
        # 檢查錯誤狀態
        if status_data['error_code'] != 0:
            error_msg = f"LED控制器錯誤: {status_data['error_code']}"
            messagebox.showwarning("設備錯誤", error_msg)
        
        # 更新GUI顯示
        update_gui_display(status_data)
        
    except Exception as e:
        logging.error(f"狀態回調異常: {e}")
```

## 最佳實踐建議

### 1. 資源管理
- 使用上下文管理器確保資源正確釋放
- 程式退出前呼叫`cleanup()`方法
- 避免重複建立LightGUIClass實例

### 2. 效能優化
- 合理設定狀態更新間隔(建議1-2秒)
- 避免在狀態回調中執行耗時操作
- 必要時使用執行緒處理GUI更新

### 3. 使用者體驗
- 提供清晰的連接狀態指示
- 實作亮度調整的即時回饋
- 提供錯誤狀態的明確提示

### 4. 程式碼組織
- 將LED控制功能封裝為獨立模組
- 使用設計模式分離GUI邏輯與控制邏輯
- 實作配置檔案管理

## 完整整合範例

```python
# main_gui.py
import tkinter as tk
from tkinter import ttk, messagebox
from LightGUIClass import LightGUIClass

class MainGUIApplication:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("機台調適工具")
        self.root.geometry("800x600")
        
        # 初始化LED控制器
        self.light_gui = LightGUIClass()
        
        # 建立GUI介面
        self.create_interface()
        
        # 註冊程式退出處理
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_interface(self):
        # 主選單
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # LED選單
        led_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="LED控制器", menu=led_menu)
        led_menu.add_command(label="連接", command=self.connect_led)
        led_menu.add_command(label="斷開", command=self.disconnect_led)
        led_menu.add_separator()
        led_menu.add_command(label="全部開啟", command=self.light_gui.turn_on_all)
        led_menu.add_command(label="全部關閉", command=self.light_gui.turn_off_all)
        
        # 主容器
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # LED控制面板
        self.led_panel = LEDControlPanel(main_frame, self.light_gui)
        self.led_status_panel = LEDStatusPanel(main_frame, self.light_gui)
        self.led_advanced_panel = LEDAdvancedPanel(main_frame, self.light_gui)
    
    def connect_led(self):
        if self.light_gui.connect():
            self.light_gui.start_auto_update(1.0)
            messagebox.showinfo("連接成功", "LED控制器已連接")
        else:
            messagebox.showerror("連接失敗", "無法連接LED控制器")
    
    def disconnect_led(self):
        self.light_gui.stop_auto_update()
        self.light_gui.disconnect()
        messagebox.showinfo("已斷開", "LED控制器已斷開")
    
    def on_closing(self):
        # 清理資源
        self.light_gui.cleanup()
        self.root.destroy()
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = MainGUIApplication()
    app.run()
```

## 擴展性考慮

### 1. 多模組整合
- 設計統一的模組介面規範
- 實作模組管理器統一控制
- 支援模組熱插拔

### 2. 配置管理
- 實作GUI配置檔案管理
- 支援使用者自定義介面布局
- 提供模組參數配置介面

### 3. 日誌與除錯
- 整合統一的日誌系統
- 提供除錯模式與詳細日誌
- 實作操作歷史記錄

通過以上設計指引，統一GUI工具可以輕鬆整合LED控制器功能，並提供完整的使用者介面與控制能力。