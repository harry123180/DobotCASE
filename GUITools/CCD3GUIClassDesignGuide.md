# CCD3GUIClass設計指南

## 概述

CCD3GUIClass是為統一機台調適工具設計的CCD3角度辨識模組控制類別，提供完整的Modbus TCP通訊介面和角度檢測功能。

## 架構設計

### 設計原則
- 專門處理CCD3模組的角度檢測控制
- 封裝雙模式檢測算法複雜性
- 提供簡潔的角度檢測API
- 支援回調機制實現即時更新
- 線程安全的狀態管理

### 通訊架構
```
統一GUI工具
    |
    |-- import --> CCD3GUIClass
                       |
                       |-- ModbusTCP Client --> 主服務器(502)
                       |                           |
                       |                           |-- CCD3模組(基地址800)
                       |
                       |-- 角度檢測算法 --> opencv_detect_module.py
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
- `capture_and_detect_angle()`: 拍照+角度檢測
- `initialize_camera()`: 相機重新初始化
- `clear_command()`: 清空指令

### 4. 參數管理
- `read_detection_params()`: 讀取檢測參數
- `write_detection_params()`: 寫入檢測參數
- 支援雙模式檢測參數配置

### 5. 結果獲取
- 自動解析角度檢測結果
- 支援32位角度精度處理
- 提供橢圓和矩形模式結果
- 預留內外徑檢測功能

## 數據結構定義

### CCD3Status - 系統狀態
```python
@dataclass
class CCD3Status:
    ready: bool = False          # Ready狀態
    running: bool = False        # Running狀態
    alarm: bool = False          # Alarm狀態
    initialized: bool = False    # Initialized狀態
    connected: bool = False      # Modbus連接狀態
```

### AngleDetectionParams - 檢測參數
```python
@dataclass
class AngleDetectionParams:
    detection_mode: int = 0      # 檢測模式(0=橢圓擬合, 1=矩形)
    min_area_rate: int = 50      # 最小面積比例(×1000存儲)
    sequence_mode: int = 0       # 輪廓選擇模式(0=最大, 1=序列)
    gaussian_kernel: int = 3     # 高斯模糊核大小
    threshold_mode: int = 0      # 閾值模式(0=OTSU, 1=手動)
    manual_threshold: int = 127  # 手動閾值
```

### AngleDetectionResult - 檢測結果
```python
@dataclass
class AngleDetectionResult:
    success: bool                # 檢測是否成功
    center_x: int                # 物體中心X座標
    center_y: int                # 物體中心Y座標
    angle: float                 # 檢測角度(-180~180度)
    major_axis: int              # 長軸長度(橢圓模式)
    minor_axis: int              # 短軸長度(橢圓模式)
    rect_width: int              # 矩形寬度(矩形模式)
    rect_height: int             # 矩形高度(矩形模式)
    contour_area: int            # 輪廓面積
    inner_diameter: Optional[float]  # 內徑(預留功能)
    outer_diameter: Optional[float]  # 外徑(預留功能)
    processing_time: int         # 處理耗時(ms)
    timestamp: str               # 時間戳
```

### StatisticsInfo - 統計資訊
```python
@dataclass
class StatisticsInfo:
    capture_time: int = 0        # 拍照耗時
    process_time: int = 0        # 處理耗時
    total_time: int = 0          # 總耗時
    operation_count: int = 0     # 操作計數
    error_count: int = 0         # 錯誤計數
    connection_count: int = 0    # 連接計數
    software_version_major: int = 3  # 軟體主版本號
    software_version_minor: int = 0  # 軟體次版本號
    run_time_hours: int = 0      # 運行時間(小時)
    run_time_minutes: int = 0    # 運行時間(分鐘)
```

## GUI實現指南

### 基本使用模式

#### 1. 導入和初始化
```python
from CCD3GUIClass import CCD3GUIClass, CCD3Status, AngleDetectionResult

# 初始化CCD3控制器
ccd3 = CCD3GUIClass(modbus_host="127.0.0.1", modbus_port=502)

# 連接到Modbus服務器
if ccd3.connect():
    print("CCD3連接成功")
else:
    print("CCD3連接失敗")
```

#### 2. 狀態監控實現
```python
def on_status_changed(status: CCD3Status):
    """狀態變化回調函數"""
    print(f"Ready: {status.ready}")
    print(f"Running: {status.running}")
    print(f"Alarm: {status.alarm}")
    print(f"Initialized: {status.initialized}")

def on_result_updated(result: AngleDetectionResult):
    """角度檢測結果回調函數"""
    print(f"檢測成功: {result.success}")
    print(f"角度: {result.angle:.2f}°")
    print(f"中心座標: ({result.center_x}, {result.center_y})")
    print(f"處理時間: {result.processing_time}ms")
    
    # 根據檢測模式顯示不同結果
    params = ccd3.get_detection_params()
    if params.detection_mode == 0:  # 橢圓模式
        print(f"長軸: {result.major_axis}, 短軸: {result.minor_axis}")
    else:  # 矩形模式
        print(f"寬度: {result.rect_width}, 高度: {result.rect_height}")

# 註冊回調函數
ccd3.add_status_callback(on_status_changed)
ccd3.add_result_callback(on_result_updated)
```

#### 3. 檢測參數管理實現
```python
# 讀取當前參數
current_params = ccd3.read_detection_params()
print(f"檢測模式: {ccd3.get_mode_description(current_params.detection_mode)}")
print(f"最小面積比例: {current_params.min_area_rate/1000:.3f}")
print(f"閾值模式: {ccd3.get_threshold_mode_description(current_params.threshold_mode)}")

# 修改參數
new_params = AngleDetectionParams(
    detection_mode=1,           # 切換到矩形模式
    min_area_rate=60,           # 調整面積比例
    sequence_mode=0,            # 最大輪廓模式
    gaussian_kernel=5,          # 增大高斯核
    threshold_mode=1,           # 手動閾值
    manual_threshold=100        # 設定閾值
)

# 寫入新參數
if ccd3.write_detection_params(new_params):
    print("參數更新成功")
    print(f"新檢測模式: {ccd3.get_mode_description(new_params.detection_mode)}")
```

#### 4. 角度檢測操作實現
```python
# 執行角度檢測
if ccd3.get_status().ready:
    if ccd3.capture_and_detect_angle():
        print("角度檢測指令發送成功")
    else:
        print("指令發送失敗")
else:
    print("系統未準備好")

# 重新初始化相機
if ccd3.initialize_camera():
    print("初始化指令發送成功")
```

#### 5. 統計資訊監控實現
```python
def display_statistics():
    """顯示統計資訊"""
    stats = ccd3.get_statistics()
    print(f"軟體版本: {ccd3.get_software_version()}")
    print(f"運行時間: {ccd3.get_run_time_string()}")
    print(f"操作次數: {stats.operation_count}")
    print(f"錯誤次數: {stats.error_count}")
    print(f"最後處理時間: {ccd3.format_processing_time(stats.total_time)}")

# 定期更新統計顯示
import threading
import time

def stats_monitor_loop():
    while True:
        display_statistics()
        time.sleep(5.0)

stats_thread = threading.Thread(target=stats_monitor_loop, daemon=True)
stats_thread.start()
```

### 進階使用技巧

#### 1. 模式切換控制
```python
def switch_detection_mode(target_mode: int) -> bool:
    """切換檢測模式"""
    current_params = ccd3.read_detection_params()
    
    if current_params.detection_mode == target_mode:
        print(f"已經是{ccd3.get_mode_description(target_mode)}")
        return True
    
    # 根據模式調整相關參數
    if target_mode == 0:  # 橢圓模式
        current_params.gaussian_kernel = 3
        current_params.min_area_rate = 50
    else:  # 矩形模式
        current_params.gaussian_kernel = 5
        current_params.min_area_rate = 40
    
    current_params.detection_mode = target_mode
    
    if ccd3.write_detection_params(current_params):
        print(f"切換到{ccd3.get_mode_description(target_mode)}成功")
        return True
    else:
        print("模式切換失敗")
        return False

# 使用範例
switch_detection_mode(DetectionMode.ELLIPSE_FITTING)  # 切換到橢圓模式
switch_detection_mode(DetectionMode.MIN_AREA_RECT)    # 切換到矩形模式
```

#### 2. 自適應參數調整
```python
def auto_adjust_parameters(image_quality: str = "normal") -> bool:
    """根據圖像品質自動調整參數"""
    current_params = ccd3.read_detection_params()
    
    if image_quality == "high":
        # 高品質圖像 - 使用更精細的參數
        current_params.gaussian_kernel = 3
        current_params.threshold_mode = ThresholdMode.OTSU_AUTO
        current_params.min_area_rate = 30
    elif image_quality == "low":
        # 低品質圖像 - 使用更寬鬆的參數
        current_params.gaussian_kernel = 7
        current_params.threshold_mode = ThresholdMode.MANUAL
        current_params.manual_threshold = 80
        current_params.min_area_rate = 60
    else:  # normal
        # 標準參數
        current_params.gaussian_kernel = 5
        current_params.threshold_mode = ThresholdMode.OTSU_AUTO
        current_params.min_area_rate = 50
    
    return ccd3.write_detection_params(current_params)

# 使用範例
auto_adjust_parameters("high")   # 高品質模式
auto_adjust_parameters("low")    # 低品質模式
auto_adjust_parameters("normal") # 標準模式
```

#### 3. 狀態機控制
```python
def wait_for_ready(timeout_seconds: int = 10) -> bool:
    """等待系統進入Ready狀態"""
    start_time = time.time()
    while time.time() - start_time < timeout_seconds:
        status = ccd3.get_status()
        if status.ready and not status.running:
            return True
        time.sleep(0.1)
    return False

def execute_angle_detection_with_wait():
    """執行角度檢測並等待完成"""
    if not wait_for_ready():
        print("系統未準備好")
        return False
    
    if not ccd3.capture_and_detect_angle():
        print("指令發送失敗")
        return False
    
    # 等待執行完成
    start_time = time.time()
    while time.time() - start_time < 30:  # 30秒超時
        status = ccd3.get_status()
        if not status.running and status.ready:
            print("角度檢測完成")
            result = ccd3.get_detection_result()
            if result and result.success:
                print(f"檢測角度: {ccd3.format_angle(result.angle)}")
                return True
            else:
                print("角度檢測失敗")
                return False
        time.sleep(0.1)
    
    print("角度檢測超時")
    return False
```

#### 4. 錯誤處理模式
```python
def handle_alarm_state():
    """處理Alarm狀態"""
    status = ccd3.get_status()
    if status.alarm:
        print("系統處於Alarm狀態，嘗試重新初始化")
        ccd3.initialize_camera()
        
        # 等待初始化完成
        time.sleep(3)
        
        # 清空指令恢復正常狀態
        ccd3.clear_command()

def safe_detection_wrapper(detection_func):
    """安全檢測包裝器"""
    try:
        # 檢查連接狀態
        if not ccd3.is_connected():
            if not ccd3.connect():
                return False
        
        # 檢查系統狀態
        status = ccd3.get_status()
        if status.alarm:
            handle_alarm_state()
            return False
        
        # 執行檢測
        return detection_func()
        
    except Exception as e:
        print(f"檢測異常: {e}")
        return False
```

#### 5. 批量角度檢測模式
```python
def batch_angle_detection(count: int, interval: float = 2.0):
    """批量角度檢測"""
    results = []
    angles = []
    
    for i in range(count):
        print(f"執行第 {i+1} 次角度檢測")
        
        if execute_angle_detection_with_wait():
            result = ccd3.get_detection_result()
            if result and result.success:
                results.append(result)
                angles.append(result.angle)
        
        if i < count - 1:  # 最後一次不需要等待
            time.sleep(interval)
    
    # 統計分析
    if angles:
        import statistics
        mean_angle = statistics.mean(angles)
        std_dev = statistics.stdev(angles) if len(angles) > 1 else 0
        print(f"批量檢測完成: {len(results)}/{count} 成功")
        print(f"平均角度: {ccd3.format_angle(mean_angle)}")
        print(f"角度標準差: {std_dev:.3f}°")
    
    return results, angles
```

## GUI介面設計建議

### 1. 狀態顯示區域
- Ready狀態指示燈 (綠色=Ready, 灰色=NotReady)
- Running狀態指示燈 (黃色=Running, 灰色=Idle)
- Alarm狀態指示燈 (紅色=Alarm, 灰色=Normal)
- Initialized狀態指示燈 (藍色=Initialized, 灰色=NotInitialized)
- 連接狀態指示燈 (綠色=Connected, 紅色=Disconnected)

### 2. 檢測模式控制區域
```python
# 模式選擇UI範例
class DetectionModePanel:
    def __init__(self, ccd3_controller):
        self.ccd3 = ccd3_controller
        self.mode_options = [
            (0, "橢圓擬合模式"),
            (1, "最小外接矩形模式")
        ]
    
    def on_mode_changed(self, selected_mode):
        """檢測模式變化回調"""
        current_params = self.ccd3.read_detection_params()
        current_params.detection_mode = selected_mode
        
        if self.ccd3.write_detection_params(current_params):
            self.update_mode_description(selected_mode)
            show_message(f"切換到{self.ccd3.get_mode_description(selected_mode)}")
        else:
            show_message("模式切換失敗")
    
    def update_mode_description(self, mode):
        """更新模式描述"""
        description = self.ccd3.get_mode_description(mode)
        self.mode_description_label.text = description
        
        # 根據模式顯示不同的結果欄位
        if mode == 0:  # 橢圓模式
            self.major_axis_widget.visible = True
            self.minor_axis_widget.visible = True
            self.rect_width_widget.visible = False
            self.rect_height_widget.visible = False
        else:  # 矩形模式
            self.major_axis_widget.visible = False
            self.minor_axis_widget.visible = False
            self.rect_width_widget.visible = True
            self.rect_height_widget.visible = True
```

### 3. 參數調整區域
```python
# 參數調整UI範例
class ParameterAdjustmentPanel:
    def __init__(self, ccd3_controller):
        self.ccd3 = ccd3_controller
        self.load_current_params()
    
    def load_current_params(self):
        """載入當前參數到UI"""
        params = self.ccd3.read_detection_params()
        
        self.detection_mode_combo.selected_value = params.detection_mode
        self.min_area_rate_slider.value = params.min_area_rate
        self.sequence_mode_combo.selected_value = params.sequence_mode
        self.gaussian_kernel_spinner.value = params.gaussian_kernel
        self.threshold_mode_combo.selected_value = params.threshold_mode
        self.manual_threshold_slider.value = params.manual_threshold
        
        # 根據閾值模式啟用/停用手動閾值控制
        self.manual_threshold_slider.enabled = (params.threshold_mode == 1)
    
    def save_params(self):
        """儲存參數到CCD3"""
        params = AngleDetectionParams(
            detection_mode=self.detection_mode_combo.selected_value,
            min_area_rate=self.min_area_rate_slider.value,
            sequence_mode=self.sequence_mode_combo.selected_value,
            gaussian_kernel=self.gaussian_kernel_spinner.value,
            threshold_mode=self.threshold_mode_combo.selected_value,
            manual_threshold=self.manual_threshold_slider.value
        )
        
        if self.ccd3.write_detection_params(params):
            show_message("參數更新成功")
        else:
            show_message("參數更新失敗")
    
    def on_threshold_mode_changed(self, mode):
        """閾值模式變化回調"""
        self.manual_threshold_slider.enabled = (mode == 1)
        if mode == 0:
            self.manual_threshold_label.text = "OTSU自動閾值"
        else:
            self.manual_threshold_label.text = f"手動閾值: {self.manual_threshold_slider.value}"
```

### 4. 操作控制區域
```python
# 操作控制UI範例
class OperationControlPanel:
    def __init__(self, ccd3_controller):
        self.ccd3 = ccd3_controller
    
    def on_capture_button_clicked(self):
        if self.ccd3.get_status().ready:
            self.ccd3.capture_image()
        else:
            show_message("系統未準備好")
    
    def on_detect_button_clicked(self):
        if self.ccd3.get_status().ready:
            self.ccd3.capture_and_detect_angle()
        else:
            show_message("系統未準備好")
    
    def on_init_button_clicked(self):
        self.ccd3.initialize_camera()
    
    def on_clear_button_clicked(self):
        self.ccd3.clear_command()
    
    def on_auto_adjust_button_clicked(self):
        """自動調整參數按鈕"""
        quality = self.image_quality_combo.selected_value
        if auto_adjust_parameters(quality):
            show_message(f"已調整為{quality}品質參數")
        else:
            show_message("參數調整失敗")
```

### 5. 檢測結果顯示區域
```python
# 結果顯示UI範例
class ResultDisplayPanel:
    def __init__(self, ccd3_controller):
        self.ccd3 = ccd3_controller
        self.ccd3.add_result_callback(self.on_result_updated)
    
    def on_result_updated(self, result: AngleDetectionResult):
        """更新檢測結果顯示"""
        # 基本結果
        self.success_label.text = "成功" if result.success else "失敗"
        self.success_label.color = "green" if result.success else "red"
        
        if result.success:
            # 角度顯示
            self.angle_label.text = self.ccd3.format_angle(result.angle)
            
            # 中心座標
            self.center_label.text = f"({result.center_x}, {result.center_y})"
            
            # 輪廓面積
            self.area_label.text = f"{result.contour_area} px²"
            
            # 根據檢測模式顯示相應結果
            params = self.ccd3.get_detection_params()
            if params.detection_mode == 0:  # 橢圓模式
                self.ellipse_result_panel.visible = True
                self.rect_result_panel.visible = False
                self.major_axis_label.text = f"{result.major_axis} px"
                self.minor_axis_label.text = f"{result.minor_axis} px"
            else:  # 矩形模式
                self.ellipse_result_panel.visible = False
                self.rect_result_panel.visible = True
                self.rect_width_label.text = f"{result.rect_width} px"
                self.rect_height_label.text = f"{result.rect_height} px"
            
            # 處理時間
            self.processing_time_label.text = self.ccd3.format_processing_time(result.processing_time)
            
            # 內外徑(如果有)
            if result.inner_diameter is not None:
                self.inner_diameter_label.text = f"{result.inner_diameter:.2f} mm"
                self.inner_diameter_panel.visible = True
            if result.outer_diameter is not None:
                self.outer_diameter_label.text = f"{result.outer_diameter:.2f} mm"
                self.outer_diameter_panel.visible = True
        
        # 更新統計資訊
        self.update_statistics_display()
    
    def update_statistics_display(self):
        """更新統計資訊顯示"""
        stats = self.ccd3.get_statistics()
        self.operation_count_label.text = str(stats.operation_count)
        self.error_count_label.text = str(stats.error_count)
        self.version_label.text = self.ccd3.get_software_version()
        self.runtime_label.text = self.ccd3.get_run_time_string()
```

### 6. 角度視覺化顯示
```python
# 角度視覺化UI範例
class AngleVisualizationPanel:
    def __init__(self, ccd3_controller):
        self.ccd3 = ccd3_controller
        self.ccd3.add_result_callback(self.on_result_updated)
        self.angle_history = []
    
    def on_result_updated(self, result: AngleDetectionResult):
        """更新角度視覺化"""
        if result.success:
            # 添加到歷史記錄
            self.angle_history.append(result.angle)
            if len(self.angle_history) > 50:  # 保持最近50次記錄
                self.angle_history.pop(0)
            
            # 更新角度錶盤
            self.update_angle_dial(result.angle)
            
            # 更新角度趨勢圖
            self.update_angle_trend_chart()
    
    def update_angle_dial(self, angle: float):
        """更新角度錶盤顯示"""
        # 將角度轉換為錶盤指針位置
        # 實現具體的錶盤繪製邏輯
        pass
    
    def update_angle_trend_chart(self):
        """更新角度趨勢圖表"""
        # 繪製最近的角度變化趨勢
        # 實現具體的圖表更新邏輯
        pass
```

## 集成注意事項

### 1. 線程安全
- CCD3GUIClass內部使用線程鎖保護共享狀態
- 回調函數在監控線程中執行，GUI更新需要切換到主線程
- 建議使用訊號-槽機制或事件佇列進行跨線程通訊

### 2. 角度精度處理
```python
# 角度精度處理範例
def handle_angle_precision():
    """處理角度精度"""
    result = ccd3.get_detection_result()
    if result:
        # CCD3提供0.01度精度
        precise_angle = result.angle  # 已經是浮點數
        
        # 角度範圍處理
        normalized_angle = ccd3.get_angle_in_range(precise_angle, -180, 180)
        
        # 格式化顯示
        angle_str = ccd3.format_angle(normalized_angle, precision=2)
        print(f"檢測角度: {angle_str}")
```

### 3. 資源管理
```python
# 應用程式退出時的清理
def on_application_exit():
    """應用程式退出清理"""
    if ccd3:
        ccd3.disconnect()
```

### 4. 異常處理
- 所有Modbus操作都包含異常處理
- 網路斷線時會自動停止監控
- 建議實現重連機制

## 寄存器映射參考

### 控制寄存器 (800-801)
- 800: 控制指令 (0=清空, 8=拍照, 16=拍照+角度檢測, 32=重新初始化)
- 801: 狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)

### 檢測參數寄存器 (810-819)
- 810: 檢測模式 (0=橢圓擬合, 1=最小外接矩形)
- 811: 最小面積比例 (×1000存儲)
- 812: 序列模式 (0=最大輪廓, 1=序列輪廓)
- 813: 高斯模糊核大小
- 814: 閾值處理模式 (0=OTSU自動, 1=手動)
- 815: 手動閾值 (0-255)

### 角度檢測結果寄存器 (840-859)
- 840: 檢測成功標誌
- 841-842: 物體中心座標 (X, Y)
- 843-844: 角度 (32位, ×100存儲)
- 845-846: 長軸、短軸長度 (橢圓模式)
- 847-848: 矩形寬度、高度 (矩形模式)
- 849: 檢測輪廓面積
- 850-853: 內外徑 (預留功能, 32位存儲)

### 統計資訊寄存器 (880-899)
- 880-882: 拍照、處理、總耗時
- 883-885: 操作、錯誤、連接計數
- 890-891: 軟體版本
- 892-893: 運行時間

## 特殊功能說明

### 1. 雙模式檢測
- **橢圓擬合模式**: 適用於圓形、橢圓形Ring物件，提供長軸短軸資訊
- **矩形模式**: 適用於矩形、不規則Ring物件，提供寬度高度資訊

### 2. 32位角度精度
- 角度範圍: ±180度
- 精度: 0.01度
- 存儲方式: 32位有符號整數(×100)

### 3. 預留功能
- 內外徑檢測寄存器已預留(850-853)
- 支援未來功能擴展

### 4. 工具方法
- `get_angle_in_range()`: 角度範圍限制
- `format_angle()`: 角度格式化顯示
- `get_mode_description()`: 模式描述獲取

## 使用範例

完整的GUI整合範例:

```python
from CCD3GUIClass import CCD3GUIClass, CCD3Status, AngleDetectionResult

class UnifiedMachineGUI:
    def __init__(self):
        # 初始化CCD3控制器
        self.ccd3 = CCD3GUIClass()
        
        # 註冊回調
        self.ccd3.add_status_callback(self.on_ccd3_status_changed)
        self.ccd3.add_result_callback(self.on_ccd3_result_updated)
        
        # 建立GUI界面
        self.setup_gui()
        
        # 連接CCD3
        self.connect_ccd3()
        
        # 載入預設參數
        self.load_default_params()
    
    def setup_gui(self):
        """建立GUI界面"""
        # 實現GUI界面創建
        pass
    
    def connect_ccd3(self):
        """連接CCD3模組"""
        if self.ccd3.connect():
            self.update_status("CCD3連接成功")
        else:
            self.update_status("CCD3連接失敗")
    
    def load_default_params(self):
        """載入預設檢測參數"""
        default_params = AngleDetectionParams(
            detection_mode=0,       # 橢圓模式
            min_area_rate=50,       # 5%面積比例
            sequence_mode=0,        # 最大輪廓
            gaussian_kernel=5,      # 5×5高斯核
            threshold_mode=0,       # OTSU自動閾值
            manual_threshold=127    # 手動閾值備用
        )
        
        if self.ccd3.write_detection_params(default_params):
            self.update_status("預設參數載入成功")
        else:
            self.update_status("預設參數載入失敗")
    
    def on_ccd3_status_changed(self, status: CCD3Status):
        """CCD3狀態變化處理"""
        # 更新GUI狀態顯示
        self.update_ccd3_status_display(status)
    
    def on_ccd3_result_updated(self, result: AngleDetectionResult):
        """CCD3檢測結果更新處理"""
        # 更新GUI結果顯示
        self.update_ccd3_result_display(result)
```

這個設計指南提供了完整的CCD3GUIClass使用框架，支援統一機台調適工具的開發需求，特別針對角度檢測功能和雙模式檢測進行了優化。