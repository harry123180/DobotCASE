# CCD2GUIClass設計指南

## 概述

CCD2GUIClass是為統一機台調適工具設計的CCD2圖像分類模組控制類別，提供完整的Modbus TCP通訊介面和JSON配置管理功能。

## 架構設計

### 設計原則
- 專門處理CCD2模組的圖像分類控制
- 封裝JSON配置檔案管理複雜性
- 提供簡潔的分類操作API
- 支援回調機制實現即時更新
- 線程安全的狀態管理

### 通訊架構
```
統一GUI工具
    |
    |-- import --> CCD2GUIClass
                       |
                       |-- ModbusTCP Client --> 主服務器(502)
                       |                           |
                       |                           |-- CCD2模組(基地址1100)
                       |
                       |-- JSON配置管理 --> condition/*.json
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
- `capture_and_classify()`: 拍照+分類
- `initialize_camera()`: 相機重新初始化
- `reload_configuration()`: 重新載入JSON配置
- `clear_command()`: 清空指令

### 4. 配置檔案管理
- `scan_configuration_files()`: 掃描JSON配置檔案
- `load_configuration_file()`: 載入指定配置檔案
- 自動驗證配置檔案格式
- 解析分類類別和處理參數

### 5. 結果獲取
- 自動解析分類結果和特徵值
- 支援32位浮點數精度處理
- 提供統計資訊(分類次數、錯誤次數等)

## 數據結構定義

### CCD2Status - 系統狀態
```python
@dataclass
class CCD2Status:
    ready: bool = False          # Ready狀態
    running: bool = False        # Running狀態
    alarm: bool = False          # Alarm狀態
    initialized: bool = False    # Initialized狀態
    connected: bool = False      # Modbus連接狀態
    config_loaded: bool = False  # 配置載入狀態
```

### ClassificationResult - 分類結果
```python
@dataclass
class ClassificationResult:
    success: bool                # 分類是否成功
    category_id: int             # 分類類別ID
    confidence: float            # 信心度 (0.0-100.0)
    matched_conditions: int      # 匹配條件數
    features: Dict[str, float]   # 特徵值字典
    processing_time: int         # 處理耗時(ms)
    timestamp: str               # 時間戳
```

### FeatureValues - 特徵值數據
```python
@dataclass
class FeatureValues:
    mean_value: float = 0.0      # 平均值
    std_dev: float = 0.0         # 標準差
    skewness: float = 0.0        # 偏度
    kurtosis: float = 0.0        # 峰度
```

### ConfigurationInfo - 配置檔案資訊
```python
@dataclass
class ConfigurationInfo:
    module_name: str = ""        # 模組名稱
    version: str = ""            # 版本號
    created_date: str = ""       # 創建日期
    description: str = ""        # 描述
    categories_count: int = 0    # 分類類別數量
    loaded_file: str = ""        # 載入的檔案名
```

### ProcessingParams - 處理參數
```python
@dataclass
class ProcessingParams:
    gaussian_kernel: int = 5     # 高斯核大小
    use_otsu: bool = True        # 是否使用OTSU閾值
    manual_threshold: int = 127  # 手動閾值
    canny_low: int = 50          # Canny低閾值
    canny_high: int = 150        # Canny高閾值
    lbp_radius: int = 3          # LBP半徑
    lbp_points: int = 24         # LBP點數
    roi_enabled: bool = True     # ROI啟用
    roi_x: int = 100             # ROI起始X座標
    roi_y: int = 100             # ROI起始Y座標
    roi_width: int = 200         # ROI寬度
    roi_height: int = 200        # ROI高度
```

### ClassificationCategory - 分類類別
```python
@dataclass
class ClassificationCategory:
    id: int                      # 類別ID
    name: str                    # 類別名稱
    enabled: bool                # 是否啟用
    logic: str                   # 條件邏輯 ("AND"/"OR")
    conditions: List[Dict]       # 分類條件列表
```

## GUI實現指南

### 基本使用模式

#### 1. 導入和初始化
```python
from CCD2GUIClass import CCD2GUIClass, CCD2Status, ClassificationResult

# 初始化CCD2控制器
ccd2 = CCD2GUIClass(modbus_host="127.0.0.1", modbus_port=502)

# 連接到Modbus服務器
if ccd2.connect():
    print("CCD2連接成功")
else:
    print("CCD2連接失敗")
```

#### 2. 狀態監控實現
```python
def on_status_changed(status: CCD2Status):
    """狀態變化回調函數"""
    print(f"Ready: {status.ready}")
    print(f"Running: {status.running}")
    print(f"Alarm: {status.alarm}")
    print(f"Initialized: {status.initialized}")
    print(f"Config Loaded: {status.config_loaded}")

def on_result_updated(result: ClassificationResult):
    """分類結果回調函數"""
    print(f"分類結果: 類別ID {result.category_id}")
    print(f"信心度: {result.confidence:.1f}%")
    print(f"匹配條件: {result.matched_conditions}")
    print(f"處理時間: {result.processing_time}ms")
    print(f"特徵值: {result.features}")

# 註冊回調函數
ccd2.add_status_callback(on_status_changed)
ccd2.add_result_callback(on_result_updated)
```

#### 3. 配置檔案管理實現
```python
# 掃描可用的配置檔案
config_files = ccd2.scan_configuration_files()
print(f"可用配置檔案: {config_files}")

# 載入指定配置檔案
if config_files:
    selected_file = config_files[0]  # 選擇第一個檔案
    if ccd2.load_configuration_file(selected_file):
        print(f"配置檔案載入成功: {selected_file}")
        
        # 獲取配置資訊
        config_info = ccd2.get_configuration_info()
        print(f"模組名稱: {config_info.module_name}")
        print(f"版本: {config_info.version}")
        print(f"分類類別數: {config_info.categories_count}")
        
        # 獲取分類類別
        categories = ccd2.get_categories()
        for cat in categories:
            print(f"類別 {cat.id}: {cat.name} ({'啟用' if cat.enabled else '停用'})")
    else:
        print("配置檔案載入失敗")
```

#### 4. 分類操作實現
```python
# 執行拍照+分類
if ccd2.get_status().ready:
    if ccd2.capture_and_classify():
        print("拍照+分類指令發送成功")
    else:
        print("指令發送失敗")
else:
    print("系統未準備好")

# 重新載入配置
if ccd2.reload_configuration():
    print("配置重新載入指令發送成功")
```

#### 5. 特徵值監控實現
```python
def monitor_features():
    """監控特徵值變化"""
    features = ccd2.get_feature_values()
    print(f"平均值: {features.mean_value:.2f}")
    print(f"標準差: {features.std_dev:.2f}")
    print(f"偏度: {features.skewness:.3f}")
    print(f"峰度: {features.kurtosis:.3f}")

# 定期更新特徵值顯示
import threading
import time

def feature_monitor_loop():
    while True:
        monitor_features()
        time.sleep(1.0)

monitor_thread = threading.Thread(target=feature_monitor_loop, daemon=True)
monitor_thread.start()
```

### 進階使用技巧

#### 1. 配置檔案驗證
```python
def validate_and_load_config(filename: str) -> bool:
    """驗證並載入配置檔案"""
    try:
        # 掃描可用檔案
        available_files = ccd2.scan_configuration_files()
        
        if filename not in available_files:
            print(f"配置檔案不存在或格式錯誤: {filename}")
            return False
        
        # 載入配置
        if ccd2.load_configuration_file(filename):
            config_info = ccd2.get_configuration_info()
            
            # 驗證配置完整性
            if config_info.categories_count == 0:
                print("配置檔案中沒有分類類別")
                return False
            
            print(f"配置載入成功: {config_info.module_name} v{config_info.version}")
            return True
        else:
            print("配置檔案載入失敗")
            return False
            
    except Exception as e:
        print(f"配置載入異常: {e}")
        return False
```

#### 2. 狀態機控制
```python
def wait_for_ready(timeout_seconds: int = 10) -> bool:
    """等待系統進入Ready狀態"""
    start_time = time.time()
    while time.time() - start_time < timeout_seconds:
        status = ccd2.get_status()
        if status.ready and not status.running:
            return True
        time.sleep(0.1)
    return False

def execute_classification_with_wait():
    """執行分類並等待完成"""
    if not wait_for_ready():
        print("系統未準備好")
        return False
    
    if not ccd2.capture_and_classify():
        print("指令發送失敗")
        return False
    
    # 等待執行完成
    start_time = time.time()
    while time.time() - start_time < 30:  # 30秒超時
        status = ccd2.get_status()
        if not status.running and status.ready:
            print("分類完成")
            result = ccd2.get_classification_result()
            if result and result.success:
                print(f"分類結果: 類別{result.category_id}, 信心度{result.confidence:.1f}%")
                return True
            else:
                print("分類失敗")
                return False
        time.sleep(0.1)
    
    print("分類超時")
    return False
```

#### 3. 錯誤處理模式
```python
def handle_alarm_state():
    """處理Alarm狀態"""
    status = ccd2.get_status()
    if status.alarm:
        print("系統處於Alarm狀態，嘗試重新初始化")
        ccd2.initialize_camera()
        
        # 等待初始化完成
        time.sleep(2)
        
        # 清空指令恢復正常狀態
        ccd2.clear_command()

def safe_operation_wrapper(operation_func):
    """安全操作包裝器"""
    try:
        # 檢查連接狀態
        if not ccd2.is_connected():
            if not ccd2.connect():
                return False
        
        # 檢查系統狀態
        status = ccd2.get_status()
        if status.alarm:
            handle_alarm_state()
            return False
        
        # 檢查配置載入狀態
        if not status.config_loaded:
            print("配置未載入，請先載入分類配置")
            return False
        
        # 執行操作
        return operation_func()
        
    except Exception as e:
        print(f"操作異常: {e}")
        return False
```

#### 4. 批量分類模式
```python
def batch_classification(count: int, interval: float = 2.0):
    """批量分類檢測"""
    results = []
    
    for i in range(count):
        print(f"執行第 {i+1} 次分類")
        
        if execute_classification_with_wait():
            result = ccd2.get_classification_result()
            if result:
                results.append(result)
        
        if i < count - 1:  # 最後一次不需要等待
            time.sleep(interval)
    
    # 統計結果
    success_count = sum(1 for r in results if r.success)
    print(f"批量分類完成: {success_count}/{len(results)} 成功")
    
    return results
```

## GUI介面設計建議

### 1. 狀態顯示區域
- Ready狀態指示燈 (綠色=Ready, 灰色=NotReady)
- Running狀態指示燈 (黃色=Running, 灰色=Idle)
- Alarm狀態指示燈 (紅色=Alarm, 灰色=Normal)
- Initialized狀態指示燈 (藍色=Initialized, 灰色=NotInitialized)
- 連接狀態指示燈 (綠色=Connected, 紅色=Disconnected)
- 配置載入指示燈 (藍色=Loaded, 灰色=NotLoaded)

### 2. 配置管理區域
```python
# 配置管理UI範例
class ConfigurationPanel:
    def __init__(self, ccd2_controller):
        self.ccd2 = ccd2_controller
        self.config_files = []
        self.selected_file = ""
    
    def refresh_config_list(self):
        """刷新配置檔案列表"""
        self.config_files = self.ccd2.scan_configuration_files()
        self.config_list_widget.clear()
        self.config_list_widget.add_items(self.config_files)
    
    def on_config_selected(self, filename):
        """配置檔案選擇回調"""
        self.selected_file = filename
        self.load_button.enabled = True
    
    def load_selected_config(self):
        """載入選中的配置檔案"""
        if self.selected_file:
            if self.ccd2.load_configuration_file(self.selected_file):
                self.update_config_display()
                show_message("配置載入成功")
            else:
                show_message("配置載入失敗")
    
    def update_config_display(self):
        """更新配置顯示"""
        config_info = self.ccd2.get_configuration_info()
        self.module_name_label.text = config_info.module_name
        self.version_label.text = config_info.version
        self.categories_count_label.text = str(config_info.categories_count)
        
        # 顯示分類類別
        categories = self.ccd2.get_categories()
        self.categories_table.clear()
        for cat in categories:
            row = [
                str(cat.id),
                cat.name,
                "啟用" if cat.enabled else "停用",
                cat.logic,
                str(len(cat.conditions))
            ]
            self.categories_table.add_row(row)
```

### 3. 操作控制區域
```python
# 操作控制UI範例
class OperationPanel:
    def __init__(self, ccd2_controller):
        self.ccd2 = ccd2_controller
    
    def on_capture_button_clicked(self):
        if self.ccd2.get_status().ready:
            self.ccd2.capture_image()
        else:
            show_message("系統未準備好")
    
    def on_classify_button_clicked(self):
        status = self.ccd2.get_status()
        if not status.ready:
            show_message("系統未準備好")
            return
        
        if not status.config_loaded:
            show_message("請先載入分類配置")
            return
        
        self.ccd2.capture_and_classify()
    
    def on_init_button_clicked(self):
        self.ccd2.initialize_camera()
    
    def on_reload_config_button_clicked(self):
        self.ccd2.reload_configuration()
    
    def on_clear_button_clicked(self):
        self.ccd2.clear_command()
```

### 4. 分類結果顯示區域
```python
# 結果顯示UI範例
class ResultDisplayPanel:
    def __init__(self, ccd2_controller):
        self.ccd2 = ccd2_controller
        self.ccd2.add_result_callback(self.on_result_updated)
    
    def on_result_updated(self, result: ClassificationResult):
        """更新分類結果顯示"""
        # 更新基本結果
        self.success_label.text = "成功" if result.success else "失敗"
        self.category_id_label.text = str(result.category_id)
        self.confidence_label.text = f"{result.confidence:.1f}%"
        self.matched_conditions_label.text = str(result.matched_conditions)
        self.processing_time_label.text = f"{result.processing_time}ms"
        
        # 更新特徵值顯示
        self.update_features_display(result.features)
        
        # 更新統計資訊
        stats = self.ccd2.get_statistics()
        self.classification_count_label.text = str(stats.get("分類次數", 0))
        self.error_count_label.text = str(stats.get("錯誤次數", 0))
    
    def update_features_display(self, features: Dict[str, float]):
        """更新特徵值顯示"""
        self.mean_value_label.text = f"{features.get('平均值', 0):.2f}"
        self.std_dev_label.text = f"{features.get('標準差', 0):.2f}"
        self.skewness_label.text = f"{features.get('偏度', 0):.3f}"
        self.kurtosis_label.text = f"{features.get('峰度', 0):.3f}"
```

### 5. 處理參數顯示區域
```python
# 參數顯示UI範例
class ParametersDisplayPanel:
    def __init__(self, ccd2_controller):
        self.ccd2 = ccd2_controller
    
    def update_parameters_display(self):
        """更新處理參數顯示"""
        params = self.ccd2.get_processing_params()
        
        # 圖像處理參數
        self.gaussian_kernel_label.text = str(params.gaussian_kernel)
        self.use_otsu_label.text = "是" if params.use_otsu else "否"
        self.manual_threshold_label.text = str(params.manual_threshold)
        self.canny_low_label.text = str(params.canny_low)
        self.canny_high_label.text = str(params.canny_high)
        
        # LBP參數
        self.lbp_radius_label.text = str(params.lbp_radius)
        self.lbp_points_label.text = str(params.lbp_points)
        
        # ROI參數
        self.roi_enabled_label.text = "啟用" if params.roi_enabled else "停用"
        if params.roi_enabled:
            self.roi_info_label.text = f"({params.roi_x}, {params.roi_y}) {params.roi_width}×{params.roi_height}"
        else:
            self.roi_info_label.text = "未啟用"
```

## 集成注意事項

### 1. 線程安全
- CCD2GUIClass內部使用線程鎖保護共享狀態
- 回調函數在監控線程中執行，GUI更新需要切換到主線程
- 建議使用訊號-槽機制或事件佇列進行跨線程通訊

### 2. 配置檔案管理
```python
# 配置檔案路徑設置
def setup_config_folder():
    """設置配置檔案資料夾"""
    config_folder = "condition"
    if not os.path.exists(config_folder):
        os.makedirs(config_folder)
        print(f"創建配置資料夾: {config_folder}")
    
    return config_folder
```

### 3. 資源管理
```python
# 應用程式退出時的清理
def on_application_exit():
    """應用程式退出清理"""
    if ccd2:
        ccd2.disconnect()
```

### 4. 異常處理
- 所有Modbus操作都包含異常處理
- 配置檔案載入包含格式驗證
- 建議實現重連機制

## 寄存器映射參考

### 控制寄存器 (1100-1101)
- 1100: 控制指令 (0=清空, 8=拍照, 16=拍照+分類, 32=初始化, 64=重新載入配置)
- 1101: 狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)

### 分類結果寄存器 (1140-1159)
- 1140: 分類成功標誌
- 1141: 類別ID
- 1142-1143: 信心度 (32位, ×100存儲)
- 1144: 匹配條件數

### 特徵值寄存器 (1150-1159)
- 1150-1151: 平均值 (32位, ×100存儲)
- 1152-1153: 標準差 (32位, ×100存儲)
- 1154-1155: 偏度 (32位, ×1000存儲)
- 1156-1157: 峰度 (32位, ×1000存儲)

### 統計資訊寄存器 (1180-1199)
- 1180: 處理耗時
- 1181: 分類計數
- 1182: 錯誤計數
- 1183: 配置載入計數
- 1190-1191: 軟體版本
- 1192-1193: 運行時間

## JSON配置檔案格式

### 標準配置檔案結構
```json
{
  "module_info": {
    "module_name": "CCD2分類模組",
    "version": "2.0",
    "created_date": "2024-06-24",
    "description": "圖像分類配置"
  },
  "categories": [
    {
      "id": 1,
      "name": "類型A",
      "enabled": true,
      "logic": "AND",
      "conditions": [
        {
          "feature": "平均值",
          "operator": ">",
          "threshold": 100.0,
          "enabled": true
        }
      ]
    }
  ],
  "processing_parameters": {
    "gaussian_kernel": 5,
    "use_otsu": true,
    "manual_threshold": 127,
    "canny_low": 50,
    "canny_high": 150,
    "lbp_radius": 3,
    "lbp_points": 24,
    "roi": {
      "enabled": true,
      "x": 100,
      "y": 100,
      "width": 200,
      "height": 200
    }
  }
}
```

### 配置檔案驗證規則
1. 必須包含module_info、categories、processing_parameters三個主要區塊
2. module_info必須包含module_name和version欄位
3. categories必須是非空列表
4. 每個category必須包含id、name、enabled、logic、conditions欄位
5. processing_parameters必須包含所有必要的圖像處理參數

## 使用範例

完整的GUI整合範例請參考以下結構:

```python
from CCD2GUIClass import CCD2GUIClass, CCD2Status, ClassificationResult

class UnifiedMachineGUI:
    def __init__(self):
        # 初始化CCD2控制器
        self.ccd2 = CCD2GUIClass()
        
        # 註冊回調
        self.ccd2.add_status_callback(self.on_ccd2_status_changed)
        self.ccd2.add_result_callback(self.on_ccd2_result_updated)
        
        # 建立GUI界面
        self.setup_gui()
        
        # 連接CCD2
        self.connect_ccd2()
        
        # 載入預設配置
        self.load_default_config()
    
    def setup_gui(self):
        """建立GUI界面"""
        # 實現GUI界面創建
        pass
    
    def connect_ccd2(self):
        """連接CCD2模組"""
        if self.ccd2.connect():
            self.update_status("CCD2連接成功")
        else:
            self.update_status("CCD2連接失敗")
    
    def load_default_config(self):
        """載入預設配置"""
        config_files = self.ccd2.scan_configuration_files()
        if config_files:
            # 載入第一個可用配置
            if self.ccd2.load_configuration_file(config_files[0]):
                self.update_status(f"預設配置載入成功: {config_files[0]}")
            else:
                self.update_status("預設配置載入失敗")
    
    def on_ccd2_status_changed(self, status: CCD2Status):
        """CCD2狀態變化處理"""
        # 更新GUI狀態顯示
        self.update_ccd2_status_display(status)
    
    def on_ccd2_result_updated(self, result: ClassificationResult):
        """CCD2分類結果更新處理"""
        # 更新GUI結果顯示
        self.update_ccd2_result_display(result)
```

這個設計指南提供了完整的CCD2GUIClass使用框架，支援統一機台調適工具的開發需求，特別針對圖像分類功能和JSON配置管理進行了優化。