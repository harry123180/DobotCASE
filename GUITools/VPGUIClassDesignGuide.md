# VPGUIClass設計指南

## 概述

VPGUIClass是為統一機台調適工具設計的VP震動盤模組控制類別，提供完整的Modbus TCP通訊介面和GUI操作功能。支援12種震動模式、背光控制、參數設定及即時狀態監控。

## 架構設計

### 設計原則
- 單一職責原則: 專門處理VP震動盤模組的GUI控制
- 封裝Modbus通訊複雜性
- 提供簡潔的API介面
- 支援回調機制實現即時更新
- 線程安全的狀態管理

### 通訊架構
```
統一GUI工具
    |
    |-- import --> VPGUIClass
                       |
                       |-- ModbusTCP Client --> 主服務器(502)
                                                    |
                                                    |-- VP模組(基地址300)
                                                          |
                                                          |-- TCP --> 震動盤設備(192.168.1.7:1000)
```

## 主要功能模組

### 1. 連接管理
- `connect()`: 建立Modbus TCP連接
- `disconnect()`: 斷開連接
- `is_connected()`: 檢查連接狀態
- `get_connection_info()`: 獲取連接資訊

### 2. 狀態監控
- 200ms輪詢頻率自動更新系統狀態
- 支援狀態變化回調通知
- 即時監控模組狀態、設備狀態、震動狀態

### 3. 基本控制操作
- `enable_device()`: 啟用設備 (開啟背光)
- `disable_device()`: 停用設備 (關閉背光)
- `stop_all_actions()`: 停止所有動作
- `emergency_stop()`: 緊急停止
- `reset_error()`: 錯誤重置

### 4. 背光控制
- `set_brightness(brightness)`: 設定背光亮度 (0-255)
- `toggle_backlight(state)`: 切換背光開關

### 5. 震動控制
- `execute_action(action, strength, frequency)`: 執行震動動作
- `execute_action_with_params(params)`: 使用參數結構執行動作
- `set_action_parameters(action, strength, frequency)`: 設定動作參數
- `set_frequency(frequency)`: 設定全域頻率
- `quick_action(action)`: 快速執行動作 (預設參數)

### 6. 進階功能
- `wait_for_idle(timeout)`: 等待系統閒置
- `execute_action_sequence(actions, interval)`: 執行動作序列
- `get_status_description()`: 獲取狀態描述

## 數據結構定義

### VPStatus - 系統狀態
```python
@dataclass
class VPStatus:
    module_status: int = 0          # 0=離線, 1=閒置, 2=執行中, 3=初始化, 4=錯誤
    device_connected: bool = False  # 設備連接狀態
    device_status: bool = False     # 設備狀態 0=OFF, 1=ON
    error_code: int = 0             # 錯誤代碼
    executing_command: bool = False # 指令執行中
    brightness_status: int = 0      # 背光亮度 0-255
    backlight_status: bool = False  # 背光開關
    vibration_status: bool = False  # 震動狀態
    frequency_status: int = 0       # 頻率狀態
    current_action: int = 0         # 當前動作編碼 (32位)
    target_action: int = 0          # 目標動作編碼 (32位)
    comm_error_count: int = 0       # 通訊錯誤計數
    connected_to_server: bool = False  # Modbus連接狀態
```

### VPOperationResult - 操作結果
```python
@dataclass
class VPOperationResult:
    success: bool = False           # 操作成功標誌
    error_message: str = ""         # 錯誤訊息
    timestamp: str = ""             # 操作時間戳
    operation_type: str = ""        # 操作類型
```

### VPActionParams - 動作參數
```python
@dataclass
class VPActionParams:
    action: str = "stop"            # 動作名稱
    strength: int = 100             # 強度 0-255
    frequency: int = 100            # 頻率 0-255
    duration: float = 0.0           # 持續時間 (秒, 保留欄位)
```

## GUI實現指南

### 基本使用模式

#### 1. 導入和初始化
```python
from VPGUIClass import VPGUIClass, VPStatus, VPOperationResult, VPActionParams

# 初始化VP控制器
vp = VPGUIClass(modbus_host="127.0.0.1", modbus_port=502)

# 連接到Modbus服務器
if vp.connect():
    print("VP連接成功")
else:
    print("VP連接失敗")
```

#### 2. 狀態監控實現
```python
def on_vp_status_changed(status: VPStatus):
    """VP狀態變化回調函數"""
    print(f"模組狀態: {status.module_status}")
    print(f"設備連接: {status.device_connected}")
    print(f"震動狀態: {status.vibration_status}")
    print(f"背光狀態: {status.backlight_status}")
    print(f"亮度: {status.brightness_status}")
    print(f"頻率: {status.frequency_status}")

def on_vp_result_updated(result: VPOperationResult):
    """VP操作結果回調函數"""
    if result.success:
        print(f"操作成功: {result.operation_type} at {result.timestamp}")
    else:
        print(f"操作失敗: {result.error_message}")

# 註冊回調函數
vp.add_status_callback(on_vp_status_changed)
vp.add_result_callback(on_vp_result_updated)
```

#### 3. 基本控制實現
```python
# 啟用設備
if vp.enable_device():
    print("設備啟用成功")

# 設定背光亮度
if vp.set_brightness(200):
    print("亮度設定成功")

# 執行震動動作
if vp.execute_action("up", strength=150, frequency=120):
    print("震動動作執行成功")

# 緊急停止
if vp.emergency_stop():
    print("緊急停止成功")
```

#### 4. 進階控制實現
```python
# 等待系統閒置
if vp.wait_for_idle(10.0):
    print("系統已閒置")

# 動作序列執行
action_sequence = [
    VPActionParams(action="up", strength=100, frequency=100),
    VPActionParams(action="down", strength=120, frequency=110),
    VPActionParams(action="stop", strength=0, frequency=0)
]

if vp.execute_action_sequence(action_sequence, interval=2.0):
    print("動作序列執行完成")
```

### 進階使用技巧

#### 1. 安全操作包裝器
```python
def safe_vp_operation(operation_func):
    """VP安全操作包裝器"""
    try:
        # 檢查連接狀態
        if not vp.is_connected():
            if not vp.connect():
                return False
        
        # 檢查系統狀態
        status = vp.get_status()
        if status.error_code > 0:
            vp.reset_error()
            time.sleep(0.5)
            return False
        
        # 執行操作
        return operation_func()
        
    except Exception as e:
        print(f"VP操作異常: {e}")
        return False
```

#### 2. 動作控制模式
```python
class VPActionController:
    """VP動作控制器"""
    
    def __init__(self, vp_controller: VPGUIClass):
        self.vp = vp_controller
        self.current_params = VPActionParams()
    
    def set_default_params(self, strength: int = 120, frequency: int = 100):
        """設定預設參數"""
        self.current_params.strength = strength
        self.current_params.frequency = frequency
    
    def execute_direction(self, direction: str) -> bool:
        """執行方向動作"""
        directions = ['up', 'down', 'left', 'right', 'upleft', 'downleft', 'upright', 'downright']
        if direction in directions:
            self.current_params.action = direction
            return self.vp.execute_action_with_params(self.current_params)
        return False
    
    def execute_pattern(self, pattern: str) -> bool:
        """執行模式動作"""
        patterns = ['horizontal', 'vertical', 'spread']
        if pattern in patterns:
            self.current_params.action = pattern
            return self.vp.execute_action_with_params(self.current_params)
        return False
    
    def stop(self) -> bool:
        """停止動作"""
        return self.vp.stop_all_actions()
```

#### 3. 狀態監控器
```python
class VPStatusMonitor:
    """VP狀態監控器"""
    
    def __init__(self, vp_controller: VPGUIClass):
        self.vp = vp_controller
        self.alarm_callbacks = []
        self.vp.add_status_callback(self.on_status_changed)
    
    def on_status_changed(self, status: VPStatus):
        """狀態變化處理"""
        # 檢查錯誤狀態
        if status.error_code > 0:
            self.handle_error(status.error_code)
        
        # 檢查連接狀態
        if not status.device_connected:
            self.handle_disconnection()
    
    def handle_error(self, error_code: int):
        """處理錯誤狀態"""
        print(f"VP錯誤: {error_code}")
        for callback in self.alarm_callbacks:
            callback("error", error_code)
    
    def handle_disconnection(self):
        """處理斷線狀態"""
        print("VP設備斷線")
        for callback in self.alarm_callbacks:
            callback("disconnection", 0)
    
    def add_alarm_callback(self, callback):
        """新增警報回調"""
        self.alarm_callbacks.append(callback)
```

## GUI介面設計建議

### 1. 狀態顯示區域
```python
# 狀態指示燈設計
class VPStatusIndicators:
    def __init__(self, vp_controller):
        self.vp = vp_controller
        self.vp.add_status_callback(self.update_indicators)
    
    def update_indicators(self, status: VPStatus):
        """更新狀態指示燈"""
        # 模組狀態指示
        if status.module_status == 1:
            self.module_led.color = "green"  # 閒置
        elif status.module_status == 2:
            self.module_led.color = "yellow"  # 執行中
        elif status.module_status == 4:
            self.module_led.color = "red"  # 錯誤
        else:
            self.module_led.color = "gray"  # 離線/初始化
        
        # 設備連接指示
        self.device_led.color = "green" if status.device_connected else "red"
        
        # 震動狀態指示
        self.vibration_led.color = "blue" if status.vibration_status else "gray"
        
        # 背光狀態指示
        self.backlight_led.color = "orange" if status.backlight_status else "gray"
```

### 2. 控制按鈕區域
```python
class VPControlPanel:
    def __init__(self, vp_controller):
        self.vp = vp_controller
        self.setup_buttons()
    
    def setup_buttons(self):
        """設置控制按鈕"""
        # 基本控制按鈕
        self.enable_btn.on_click = self.on_enable_clicked
        self.disable_btn.on_click = self.on_disable_clicked
        self.stop_btn.on_click = self.on_stop_clicked
        self.emergency_btn.on_click = self.on_emergency_clicked
        
        # 方向控制按鈕
        direction_buttons = {
            'up': self.up_btn, 'down': self.down_btn,
            'left': self.left_btn, 'right': self.right_btn,
            'upleft': self.upleft_btn, 'downleft': self.downleft_btn,
            'upright': self.upright_btn, 'downright': self.downright_btn
        }
        
        for direction, button in direction_buttons.items():
            button.on_click = lambda d=direction: self.on_direction_clicked(d)
        
        # 模式按鈕
        self.horizontal_btn.on_click = lambda: self.on_pattern_clicked('horizontal')
        self.vertical_btn.on_click = lambda: self.on_pattern_clicked('vertical')
        self.spread_btn.on_click = lambda: self.on_pattern_clicked('spread')
    
    def on_enable_clicked(self):
        """啟用按鈕點擊"""
        if self.check_ready():
            self.vp.enable_device()
    
    def on_direction_clicked(self, direction: str):
        """方向按鈕點擊"""
        if self.check_ready():
            strength = int(self.strength_slider.value)
            frequency = int(self.frequency_slider.value)
            self.vp.execute_action(direction, strength, frequency)
    
    def on_pattern_clicked(self, pattern: str):
        """模式按鈕點擊"""
        if self.check_ready():
            strength = int(self.strength_slider.value)
            frequency = int(self.frequency_slider.value)
            self.vp.execute_action(pattern, strength, frequency)
    
    def check_ready(self) -> bool:
        """檢查是否準備好執行操作"""
        status = self.vp.get_status()
        return (status.connected_to_server and 
                status.device_connected and 
                not status.executing_command and 
                status.error_code == 0)
```

### 3. 參數調整區域
```python
class VPParameterPanel:
    def __init__(self, vp_controller):
        self.vp = vp_controller
        self.setup_parameter_controls()
    
    def setup_parameter_controls(self):
        """設置參數控制元件"""
        # 亮度滑桿 (0-255)
        self.brightness_slider.min_value = 0
        self.brightness_slider.max_value = 255
        self.brightness_slider.value = 100
        self.brightness_slider.on_change = self.on_brightness_changed
        
        # 強度滑桿 (0-255)
        self.strength_slider.min_value = 0
        self.strength_slider.max_value = 255
        self.strength_slider.value = 120
        
        # 頻率滑桿 (0-255)
        self.frequency_slider.min_value = 0
        self.frequency_slider.max_value = 255
        self.frequency_slider.value = 100
        self.frequency_slider.on_change = self.on_frequency_changed
    
    def on_brightness_changed(self, value: int):
        """亮度變化處理"""
        self.vp.set_brightness(value)
        self.brightness_label.text = f"亮度: {value}"
    
    def on_frequency_changed(self, value: int):
        """頻率變化處理"""
        self.vp.set_frequency(value)
        self.frequency_label.text = f"頻率: {value}"
    
    def get_current_params(self) -> VPActionParams:
        """獲取當前參數"""
        return VPActionParams(
            action="stop",
            strength=int(self.strength_slider.value),
            frequency=int(self.frequency_slider.value)
        )
```

### 4. 狀態顯示區域
```python
class VPStatusDisplay:
    def __init__(self, vp_controller):
        self.vp = vp_controller
        self.vp.add_status_callback(self.update_status_display)
    
    def update_status_display(self, status: VPStatus):
        """更新狀態顯示"""
        status_desc = self.vp.get_status_description()
        
        # 更新狀態文字
        self.module_status_label.text = f"模組狀態: {status_desc['模組狀態']}"
        self.device_status_label.text = f"設備狀態: {status_desc['設備狀態']}"
        self.brightness_label.text = f"背光亮度: {status_desc['背光亮度']}"
        self.frequency_label.text = f"頻率: {status_desc['頻率']}"
        self.error_label.text = f"錯誤: {status_desc['錯誤代碼']}"
        
        # 更新連接狀態
        self.connection_status.text = "已連接" if status.connected_to_server else "未連接"
        self.connection_status.color = "green" if status.connected_to_server else "red"
        
        # 更新執行狀態
        self.execution_status.text = "執行中" if status.executing_command else "空閒"
        self.execution_status.color = "orange" if status.executing_command else "blue"
```

## 動作映射參考

### 基本動作 (0-11)
```python
ACTION_MAP = {
    'stop': 0,          # 停止
    'up': 1,            # 向上
    'down': 2,          # 向下
    'left': 3,          # 向左
    'right': 4,         # 向右
    'upleft': 5,        # 左上
    'downleft': 6,      # 左下
    'upright': 7,       # 右上
    'downright': 8,     # 右下
    'horizontal': 9,    # 水平震動
    'vertical': 10,     # 垂直震動
    'spread': 11        # 散布震動
}
```

### 指令映射 (0-14)
```python
COMMAND_MAP = {
    'nop': 0,               # 無操作
    'enable_device': 1,     # 設備啟用 (背光開啟)
    'disable_device': 2,    # 設備停用 (背光關閉)
    'stop_all': 3,          # 停止所有動作
    'set_brightness': 4,    # 設定背光亮度
    'execute_action': 5,    # 執行動作
    'emergency_stop': 6,    # 緊急停止
    'reset_error': 7,       # 錯誤重置
    'set_action_params': 11,    # 設定動作參數
    'toggle_backlight': 12,     # 背光切換
    'execute_with_params': 13,  # 執行動作並設定參數
    'set_frequency': 14,        # 設定頻率
}
```

## 寄存器映射參考

### 狀態寄存器 (只讀 300-314)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 300 | 模組狀態 | 0=離線, 1=閒置, 2=執行中, 3=初始化, 4=錯誤 |
| 301 | 設備連接狀態 | 0=斷開, 1=已連接 |
| 302 | 設備狀態 | 0=OFF, 1=ON |
| 303 | 錯誤代碼 | 錯誤編號，0=無錯誤 |
| 304-305 | 當前動作編碼 | 32位 (高位+低位) |
| 306-307 | 目標動作編碼 | 32位 (高位+低位) |
| 308 | 指令執行狀態 | 0=空閒, 1=執行中 |
| 309 | 通訊錯誤計數 | 累積錯誤次數 |
| 310 | 背光亮度狀態 | 當前背光亮度 (0-255) |
| 311 | 背光開關狀態 | 0=關閉, 1=開啟 |
| 312 | 震動狀態 | 0=停止, 1=運行 |
| 313 | 頻率狀態 | 當前頻率值 |
| 314 | 時間戳 | 最後更新時間戳 |

### 指令寄存器 (讀寫 320-324)
| 地址 | 功能 | 說明 |
|------|------|------|
| 320 | 指令代碼 | 執行的指令類型 |
| 321 | 參數1 | 強度/亮度/動作碼 |
| 322 | 參數2 | 頻率 |
| 323 | 參數3 | 保留欄位 |
| 324 | 指令ID | 唯一識別碼，防重複執行 |

## 集成注意事項

### 1. 線程安全
- VPGUIClass內部使用線程鎖保護共享狀態
- 回調函數在監控線程中執行，GUI更新需要切換到主線程
- 建議使用訊號-槽機制或事件佇列進行跨線程通訊

### 2. 資源管理
```python
# 應用程式退出時的清理
def on_application_exit():
    """應用程式退出清理"""
    if vp:
        vp.stop_all_actions()  # 停止所有動作
        vp.disconnect()        # 斷開連接
```

### 3. 異常處理
- 所有Modbus操作都包含異常處理
- 網路斷線時會自動停止監控
- 建議實現自動重連機制

### 4. 效能考慮
- 狀態監控頻率為200ms，適合即時顯示
- 避免在回調函數中執行耗時操作
- 動作序列執行時注意間隔時間設定

## 使用範例

完整的GUI整合範例:

```python
from VPGUIClass import VPGUIClass, VPStatus, VPOperationResult, VPActionParams

class UnifiedMachineGUI:
    def __init__(self):
        # 初始化VP控制器
        self.vp = VPGUIClass()
        
        # 註冊回調
        self.vp.add_status_callback(self.on_vp_status_changed)
        self.vp.add_result_callback(self.on_vp_result_updated)
        
        # 建立GUI界面
        self.setup_gui()
        
        # 連接VP
        self.connect_vp()
    
    def setup_gui(self):
        """建立GUI界面"""
        # 實現GUI界面創建
        pass
    
    def connect_vp(self):
        """連接VP模組"""
        if self.vp.connect():
            self.update_status("VP連接成功")
        else:
            self.update_status("VP連接失敗")
    
    def on_vp_status_changed(self, status: VPStatus):
        """VP狀態變化處理"""
        # 更新GUI狀態顯示
        self.update_vp_status_display(status)
    
    def on_vp_result_updated(self, result: VPOperationResult):
        """VP操作結果更新處理"""
        # 更新GUI操作結果顯示
        self.update_vp_result_display(result)
    
    def on_direction_button_clicked(self, direction: str):
        """方向按鈕點擊處理"""
        params = VPActionParams(
            action=direction,
            strength=self.get_strength_setting(),
            frequency=self.get_frequency_setting()
        )
        self.vp.execute_action_with_params(params)
    
    def on_emergency_stop_clicked(self):
        """緊急停止按鈕處理"""
        self.vp.emergency_stop()
```

這個設計指南提供了完整的VPGUIClass使用框架，支援統一機台調適工具的開發需求，確保震動盤模組的所有功能都能透過簡潔的API介面進行控制。