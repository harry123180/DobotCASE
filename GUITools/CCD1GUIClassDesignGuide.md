# CCD1GUIClass設計指南

## 概述

CCD1GUIClass是為統一機台調適工具設計的CCD1視覺檢測模組控制類別，提供完整的Modbus TCP通訊介面和GUI操作功能。

## 架構設計

### 設計原則
- 單一職責原則: 專門處理CCD1模組的GUI控制
- 封裝Modbus通訊複雜性
- 提供簡潔的API介面
- 支援回調機制實現即時更新
- 線程安全的狀態管理

### 通訊架構
```
統一GUI工具
    |
    |-- import --> CCD1GUIClass
                       |
                       |-- ModbusTCP Client --> 主服務器(502)
                                                    |
                                                    |-- CCD1模組(基地址200)
```

## 主要功能模組

### 1. 連接管理
- `connect()`: 建立Modbus TCP連接
- `disconnect()`: 斷開連接
- `is_connected()`: 檢查連接狀態

### 2. 狀態監控
- 200ms輪詢頻率自動更新系統狀態
- 支援狀態變化回調通知
- 即時監控握手協議狀態

### 3. 操作控制
- `capture_image()`: 執行拍照
- `capture_and_detect()`: 拍照+檢測
- `initialize_camera()`: 相機重新初始化
- `clear_command()`: 清空指令

### 4. 參數管理
- `read_detection_params()`: 讀取檢測參數
- `write_detection_params()`: 寫入檢測參數
- 支援32位面積值處理

### 5. 結果獲取
- 自動解析像素座標和世界座標
- 支援最多5個圓形檢測結果
- 提供統計資訊(耗時、計數等)

## 數據結構定義

### CCD1Status - 系統狀態
```python
@dataclass
class CCD1Status:
    ready: bool = False          # Ready狀態
    running: bool = False        # Running狀態
    alarm: bool = False          # Alarm狀態
    initialized: bool = False    # Initialized狀態
    connected: bool = False      # Modbus連接狀態
    world_coord_valid: bool = False  # 世界座標有效性
```

### CircleData - 圓形檢測結果
```python
@dataclass
class CircleData:
    pixel_x: int                 # 像素座標X
    pixel_y: int                 # 像素座標Y
    radius: int                  # 半徑
    world_x: Optional[float] = None  # 世界座標X (mm)
    world_y: Optional[float] = None  # 世界座標Y (mm)
```

### DetectionResult - 完整檢測結果
```python
@dataclass
class DetectionResult:
    circle_count: int            # 檢測到的圓形數量
    circles: List[CircleData]    # 圓形數據列表
    capture_time: int            # 拍照耗時(ms)
    process_time: int            # 處理耗時(ms)
    total_time: int              # 總耗時(ms)
    timestamp: str               # 時間戳
```

### DetectionParams - 檢測參數
```python
@dataclass
class DetectionParams:
    min_area: int = 50000        # 最小面積
    min_roundness: int = 800     # 最小圓度(×1000)
    gaussian_kernel: int = 9     # 高斯核大小
    canny_low: int = 20          # Canny低閾值
    canny_high: int = 60         # Canny高閾值
```

## GUI實現指南

### 基本使用模式

#### 1. 導入和初始化
```python
from CCD1GUIClass import CCD1GUIClass, CCD1Status, DetectionResult

# 初始化CCD1控制器
ccd1 = CCD1GUIClass(modbus_host="127.0.0.1", modbus_port=502)

# 連接到Modbus服務器
if ccd1.connect():
    print("CCD1連接成功")
else:
    print("CCD1連接失敗")
```

#### 2. 狀態監控實現
```python
def on_status_changed(status: CCD1Status):
    """狀態變化回調函數"""
    print(f"Ready: {status.ready}")
    print(f"Running: {status.running}")
    print(f"Alarm: {status.alarm}")
    print(f"Initialized: {status.initialized}")
    print(f"World Coord Valid: {status.world_coord_valid}")

def on_result_updated(result: DetectionResult):
    """檢測結果回調函數"""
    print(f"檢測到 {result.circle_count} 個圓形")
    for i, circle in enumerate(result.circles):
        print(f"圓形 {i+1}: 像素({circle.pixel_x}, {circle.pixel_y})")
        if circle.world_x is not None:
            print(f"         世界座標({circle.world_x:.2f}, {circle.world_y:.2f})")

# 註冊回調函數
ccd1.add_status_callback(on_status_changed)
ccd1.add_result_callback(on_result_updated)
```

#### 3. 操作控制實現
```python
# 執行拍照+檢測
if ccd1.get_status().ready:
    if ccd1.capture_and_detect():
        print("拍照+檢測指令發送成功")
    else:
        print("指令發送失敗")
else:
    print("系統未準備好")

# 重新初始化相機
if ccd1.initialize_camera():
    print("初始化指令發送成功")
```

#### 4. 參數調整實現
```python
# 讀取當前參數
current_params = ccd1.read_detection_params()
print(f"當前最小面積: {current_params.min_area}")

# 修改參數
new_params = DetectionParams(
    min_area=60000,
    min_roundness=750,
    gaussian_kernel=11,
    canny_low=25,
    canny_high=65
)

# 寫入新參數
if ccd1.write_detection_params(new_params):
    print("參數更新成功")
```

### 進階使用技巧

#### 1. 狀態機控制
```python
def wait_for_ready(timeout_seconds: int = 10) -> bool:
    """等待系統進入Ready狀態"""
    start_time = time.time()
    while time.time() - start_time < timeout_seconds:
        status = ccd1.get_status()
        if status.ready and not status.running:
            return True
        time.sleep(0.1)
    return False

def execute_detection_with_wait():
    """執行檢測並等待完成"""
    if not wait_for_ready():
        print("系統未準備好")
        return False
    
    if not ccd1.capture_and_detect():
        print("指令發送失敗")
        return False
    
    # 等待執行完成
    start_time = time.time()
    while time.time() - start_time < 30:  # 30秒超時
        status = ccd1.get_status()
        if not status.running and status.ready:
            print("檢測完成")
            return True
        time.sleep(0.1)
    
    print("檢測超時")
    return False
```

#### 2. 錯誤處理模式
```python
def handle_alarm_state():
    """處理Alarm狀態"""
    status = ccd1.get_status()
    if status.alarm:
        print("系統處於Alarm狀態，嘗試重新初始化")
        ccd1.initialize_camera()
        
        # 等待初始化完成
        time.sleep(2)
        
        # 清空指令恢復正常狀態
        ccd1.clear_command()

def safe_operation_wrapper(operation_func):
    """安全操作包裝器"""
    try:
        # 檢查連接狀態
        if not ccd1.is_connected():
            if not ccd1.connect():
                return False
        
        # 檢查系統狀態
        status = ccd1.get_status()
        if status.alarm:
            handle_alarm_state()
            return False
        
        # 執行操作
        return operation_func()
        
    except Exception as e:
        print(f"操作異常: {e}")
        return False
```

#### 3. 批量檢測模式
```python
def batch_detection(count: int, interval: float = 1.0):
    """批量檢測"""
    results = []
    
    for i in range(count):
        print(f"執行第 {i+1} 次檢測")
        
        if execute_detection_with_wait():
            result = ccd1.get_detection_result()
            if result:
                results.append(result)
        
        if i < count - 1:  # 最後一次不需要等待
            time.sleep(interval)
    
    return results
```

## GUI介面設計建議

### 1. 狀態顯示區域
- Ready狀態指示燈 (綠色=Ready, 灰色=NotReady)
- Running狀態指示燈 (黃色=Running, 灰色=Idle)
- Alarm狀態指示燈 (紅色=Alarm, 灰色=Normal)
- Initialized狀態指示燈 (藍色=Initialized, 灰色=NotInitialized)
- 連接狀態指示燈 (綠色=Connected, 紅色=Disconnected)
- 世界座標有效指示燈 (藍色=Valid, 灰色=Invalid)

### 2. 操作控制區域
```python
# 按鈕控制範例
def on_capture_button_clicked():
    if ccd1.get_status().ready:
        ccd1.capture_image()
    else:
        show_message("系統未準備好")

def on_detect_button_clicked():
    if ccd1.get_status().ready:
        ccd1.capture_and_detect()
    else:
        show_message("系統未準備好")

def on_init_button_clicked():
    ccd1.initialize_camera()

def on_clear_button_clicked():
    ccd1.clear_command()
```

### 3. 參數調整區域
```python
# 參數調整UI範例
class ParameterPanel:
    def __init__(self, ccd1_controller):
        self.ccd1 = ccd1_controller
        self.load_current_params()
    
    def load_current_params(self):
        """載入當前參數到UI"""
        params = self.ccd1.read_detection_params()
        self.min_area_input.value = params.min_area
        self.min_roundness_input.value = params.min_roundness / 1000.0
        self.gaussian_kernel_input.value = params.gaussian_kernel
        self.canny_low_input.value = params.canny_low
        self.canny_high_input.value = params.canny_high
    
    def save_params(self):
        """儲存參數到CCD1"""
        params = DetectionParams(
            min_area=int(self.min_area_input.value),
            min_roundness=int(self.min_roundness_input.value * 1000),
            gaussian_kernel=int(self.gaussian_kernel_input.value),
            canny_low=int(self.canny_low_input.value),
            canny_high=int(self.canny_high_input.value)
        )
        
        if self.ccd1.write_detection_params(params):
            show_message("參數更新成功")
        else:
            show_message("參數更新失敗")
```

### 4. 檢測結果顯示區域
```python
# 結果顯示範例
class ResultDisplayPanel:
    def __init__(self, ccd1_controller):
        self.ccd1 = ccd1_controller
        self.ccd1.add_result_callback(self.on_result_updated)
    
    def on_result_updated(self, result: DetectionResult):
        """更新檢測結果顯示"""
        # 更新統計資訊
        self.circle_count_label.text = f"檢測數量: {result.circle_count}"
        self.capture_time_label.text = f"拍照耗時: {result.capture_time}ms"
        self.process_time_label.text = f"處理耗時: {result.process_time}ms"
        self.total_time_label.text = f"總耗時: {result.total_time}ms"
        
        # 更新圓形數據表格
        self.update_circle_table(result.circles)
    
    def update_circle_table(self, circles: List[CircleData]):
        """更新圓形數據表格"""
        self.circle_table.clear()
        
        headers = ["ID", "像素X", "像素Y", "半徑", "世界X", "世界Y"]
        self.circle_table.set_headers(headers)
        
        for i, circle in enumerate(circles):
            row = [
                str(i + 1),
                str(circle.pixel_x),
                str(circle.pixel_y),
                str(circle.radius),
                f"{circle.world_x:.2f}" if circle.world_x is not None else "N/A",
                f"{circle.world_y:.2f}" if circle.world_y is not None else "N/A"
            ]
            self.circle_table.add_row(row)
```

## 集成注意事項

### 1. 線程安全
- CCD1GUIClass內部使用線程鎖保護共享狀態
- 回調函數在監控線程中執行，GUI更新需要切換到主線程
- 建議使用訊號-槽機制或事件佇列進行跨線程通訊

### 2. 資源管理
```python
# 應用程式退出時的清理
def on_application_exit():
    """應用程式退出清理"""
    if ccd1:
        ccd1.disconnect()
```

### 3. 異常處理
- 所有Modbus操作都包含異常處理
- 網路斷線時會自動停止監控
- 建議實現重連機制

### 4. 效能考慮
- 狀態監控頻率為200ms，適合即時顯示
- 避免在回調函數中執行耗時操作
- 大量歷史數據建議異步處理

## 寄存器映射參考

### 控制寄存器 (200-201)
- 200: 控制指令 (0=清空, 8=拍照, 16=拍照+檢測, 32=重新初始化)
- 201: 狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)

### 檢測參數寄存器 (210-215)
- 210-211: 最小面積 (32位, 高位+低位)
- 212: 最小圓度 (×1000存儲)
- 213: 高斯核大小
- 214: Canny低閾值
- 215: Canny高閾值

### 檢測結果寄存器 (240-276)
- 240: 檢測圓形數量
- 241-255: 像素座標結果 (每個圓形3個寄存器: X, Y, Radius)
- 256: 世界座標有效標誌
- 257-276: 世界座標結果 (每個圓形4個寄存器: X高位, X低位, Y高位, Y低位)

### 統計資訊寄存器 (280-284)
- 280: 最後拍照耗時
- 281: 最後處理耗時
- 282: 最後總耗時
- 283: 操作計數器
- 284: 錯誤計數器

## 標定功能擴展

由於標定檔案管理需要與CCD1模組的Web API通訊，建議在統一GUI工具中實現以下功能:

### 1. HTTP API通訊
```python
import requests

def scan_calibration_files_via_http():
    """透過HTTP API掃描標定檔案"""
    try:
        response = requests.get("http://localhost:5051/api/calibration/scan")
        return response.json()
    except Exception as e:
        return {"success": False, "error": str(e)}

def load_calibration_via_http(intrinsic_file="", extrinsic_file=""):
    """透過HTTP API載入標定數據"""
    try:
        data = {
            "intrinsic_file": intrinsic_file,
            "extrinsic_file": extrinsic_file
        }
        response = requests.post("http://localhost:5051/api/calibration/load", json=data)
        return response.json()
    except Exception as e:
        return {"success": False, "error": str(e)}
```

### 2. 標定狀態監控
```python
def get_calibration_status_via_http():
    """透過HTTP API獲取標定狀態"""
    try:
        response = requests.get("http://localhost:5051/api/calibration/status")
        return response.json()
    except Exception as e:
        return {"success": False, "error": str(e)}
```

## 使用範例

完整的GUI整合範例請參考以下結構:

```python
from CCD1GUIClass import CCD1GUIClass, CCD1Status, DetectionResult, DetectionParams

class UnifiedMachineGUI:
    def __init__(self):
        # 初始化CCD1控制器
        self.ccd1 = CCD1GUIClass()
        
        # 註冊回調
        self.ccd1.add_status_callback(self.on_ccd1_status_changed)
        self.ccd1.add_result_callback(self.on_ccd1_result_updated)
        
        # 建立GUI界面
        self.setup_gui()
        
        # 連接CCD1
        self.connect_ccd1()
    
    def setup_gui(self):
        """建立GUI界面"""
        # 實現GUI界面創建
        pass
    
    def connect_ccd1(self):
        """連接CCD1模組"""
        if self.ccd1.connect():
            self.update_status("CCD1連接成功")
        else:
            self.update_status("CCD1連接失敗")
    
    def on_ccd1_status_changed(self, status: CCD1Status):
        """CCD1狀態變化處理"""
        # 更新GUI狀態顯示
        self.update_ccd1_status_display(status)
    
    def on_ccd1_result_updated(self, result: DetectionResult):
        """CCD1檢測結果更新處理"""
        # 更新GUI結果顯示
        self.update_ccd1_result_display(result)
```

這個設計指南提供了完整的CCD1GUIClass使用框架，支援統一機台調適工具的開發需求。