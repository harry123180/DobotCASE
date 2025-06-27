# DobotM1ProGUIClass設計指南

## 概述

DobotM1ProGUIClass是為統一機台調適工具設計的Dobot M1Pro機械臂控制類別，基於新架構混合交握協議，支援運動類Flow和IO類Flow的完整控制功能。

## 架構設計

### 設計原則
- **混合交握協議**: 運動類Flow採用狀態機交握，IO類Flow採用專用佇列併行
- **安全性優先**: 運動類Flow嚴格序列化執行，確保機械臂安全
- **效率兼顧**: IO類Flow支援併行執行，提高系統整體效率
- **狀態透明**: 完整的狀態監控和回調機制
- **地址安全**: 解決地址衝突，使用新地址範圍1200-1249

### 通訊架構
```
統一GUI工具
    |
    |-- import --> DobotM1ProGUIClass
                       |
                       |-- ModbusTCP Client --> 主服務器(502)
                                                    |
                                                    |-- 運動類Flow(基地址1200-1249)
                                                    |     |-- Flow1 VP視覺抓取
                                                    |     |-- Flow2 CV出料流程
                                                    |     |-- Flow5 機械臂運轉
                                                    |
                                                    |-- IO類Flow(地址447-449)
                                                          |-- Flow3 翻轉站控制
                                                          |-- Flow4 震動投料控制
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
- 分離運動狀態和IO狀態監控

### 3. 運動類Flow控制 (狀態機交握)
- `start_flow1_vp_vision_pick()`: 啟動Flow1 VP視覺抓取流程
- `start_flow2_cv_unload()`: 啟動Flow2 CV出料流程
- `start_flow5_assembly()`: 啟動Flow5 機械臂運轉流程
- `clear_motion_flow(flow_id)`: 清空運動類Flow控制寄存器
- `clear_motion_alarm()`: 清除運動類警報
- `motion_emergency_stop()`: 運動類緊急停止

### 4. IO類Flow控制 (併行執行)
- `start_flow3_flip_station()`: 啟動Flow3 翻轉站控制
- `start_flow4_vibration_feed()`: 啟動Flow4 震動投料控制

### 5. 狀態查詢方法
- `is_motion_ready()`: 檢查運動系統是否Ready
- `is_motion_running()`: 檢查運動系統是否執行中
- `is_motion_alarm()`: 檢查運動系統是否警報
- `get_current_motion_flow()`: 獲取當前運動Flow
- `get_motion_progress()`: 獲取運動進度
- `get_flow_completion_status()`: 獲取Flow完成狀態
- `get_io_flow_status()`: 獲取IO類Flow狀態

### 6. 進階功能
- `wait_for_motion_ready(timeout)`: 等待運動系統Ready
- `wait_for_motion_complete(timeout)`: 等待運動執行完成
- `wait_for_flow_complete(flow_id, timeout)`: 等待特定Flow完成
- `execute_flow_sequence(flow_sequence)`: 執行Flow序列

## 數據結構定義

### DobotMotionStatus - 運動類狀態
```python
@dataclass
class DobotMotionStatus:
    ready: bool = False             # Ready狀態 (bit0)
    running: bool = False           # Running狀態 (bit1)
    alarm: bool = False             # Alarm狀態 (bit2)
    initialized: bool = False       # Initialized狀態 (bit3)
    current_motion_flow: int = 0    # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
    motion_progress: int = 0        # 運動進度 (0-100%)
    motion_error_code: int = 0      # 運動錯誤碼
    flow1_complete: bool = False    # Flow1完成且角度校正成功
    flow2_complete: bool = False    # Flow2完成
    flow5_complete: bool = False    # Flow5完成
    motion_op_count: int = 0        # 運動操作計數
    motion_err_count: int = 0       # 運動錯誤計數
    motion_run_time: int = 0        # 運動系統運行時間(分鐘)
```

### DobotIOStatus - IO類狀態
```python
@dataclass
class DobotIOStatus:
    flow3_active: bool = False      # Flow3翻轉站執行中
    flow4_active: bool = False      # Flow4震動投料執行中
    flow3_control: int = 0          # Flow3控制寄存器值
    flow4_control: int = 0          # Flow4控制寄存器值
```

### DobotSystemStatus - 系統整體狀態
```python
@dataclass
class DobotSystemStatus:
    motion_status: DobotMotionStatus = None
    io_status: DobotIOStatus = None
    connected_to_server: bool = False
    timestamp: str = ""
```

### DobotOperationResult - 操作結果
```python
@dataclass
class DobotOperationResult:
    success: bool = False           # 操作成功標誌
    flow_type: FlowType = None      # Flow類型 (MOTION/IO)
    flow_id: int = 0               # Flow ID
    error_message: str = ""         # 錯誤訊息
    timestamp: str = ""             # 操作時間戳
    operation_type: str = ""        # 操作類型
```

## GUI實現指南

### 基本使用模式

#### 1. 導入和初始化
```python
from DobotM1ProGUIClass import DobotM1ProGUIClass, DobotSystemStatus, DobotOperationResult, FlowType

# 初始化Dobot控制器
dobot = DobotM1ProGUIClass(modbus_host="127.0.0.1", modbus_port=502)

# 連接到Modbus服務器
if dobot.connect():
    print("Dobot連接成功")
    print(f"架構: {dobot.get_connection_info()['architecture']}")
else:
    print("Dobot連接失敗")
```

#### 2. 狀態監控實現
```python
def on_dobot_status_changed(status: DobotSystemStatus):
    """Dobot狀態變化回調函數"""
    print(f"運動狀態: Ready={status.motion_status.ready}, "
          f"Running={status.motion_status.running}, "
          f"Alarm={status.motion_status.alarm}")
    
    if status.motion_status.current_motion_flow > 0:
        print(f"當前運動Flow: {status.motion_status.current_motion_flow}")
        print(f"運動進度: {status.motion_status.motion_progress}%")
    
    print(f"IO狀態: Flow3={status.io_status.flow3_active}, "
          f"Flow4={status.io_status.flow4_active}")

def on_dobot_result_updated(result: DobotOperationResult):
    """Dobot操作結果回調函數"""
    flow_type_str = "運動類" if result.flow_type == FlowType.MOTION else "IO類"
    
    if result.success:
        print(f"{flow_type_str}Flow{result.flow_id}啟動成功: {result.operation_type}")
    else:
        print(f"{flow_type_str}Flow{result.flow_id}啟動失敗: {result.error_message}")

# 註冊回調函數
dobot.add_status_callback(on_dobot_status_changed)
dobot.add_result_callback(on_dobot_result_updated)
```

#### 3. 運動類Flow控制實現
```python
# 檢查運動系統狀態
if dobot.is_motion_ready():
    print("運動系統準備就緒")
    
    # 執行Flow1 VP視覺抓取
    if dobot.start_flow1_vp_vision_pick():
        print("Flow1啟動成功")
        
        # 等待完成
        if dobot.wait_for_flow_complete(1, timeout=300.0):
            print("Flow1執行完成")
            # 清空控制寄存器
            dobot.clear_motion_flow(1)
        else:
            print("Flow1執行超時")
    else:
        print("Flow1啟動失敗")

elif dobot.is_motion_alarm():
    print("運動系統警報，嘗試清除")
    dobot.clear_motion_alarm()
else:
    print("運動系統未準備好")
```

#### 4. IO類Flow控制實現
```python
# IO類Flow可以併行執行，不需要檢查運動狀態
if dobot.start_flow3_flip_station():
    print("Flow3翻轉站啟動成功")

if dobot.start_flow4_vibration_feed():
    print("Flow4震動投料啟動成功")

# 檢查IO狀態
io_status = dobot.get_io_flow_status()
print(f"Flow3執行中: {io_status['flow3_active']}")
print(f"Flow4執行中: {io_status['flow4_active']}")
```

#### 5. Flow序列執行
```python
# 執行運動類Flow序列
flow_sequence = [1, 2, 5]  # Flow1 -> Flow2 -> Flow5
if dobot.execute_flow_sequence(flow_sequence):
    print("Flow序列執行完成")
else:
    print("Flow序列執行失敗")
```

### 進階使用技巧

#### 1. 安全操作包裝器
```python
def safe_dobot_motion_operation(operation_func):
    """Dobot運動類安全操作包裝器"""
    try:
        # 檢查連接狀態
        if not dobot.is_connected():
            if not dobot.connect():
                return False
        
        # 檢查運動系統狀態
        if dobot.is_motion_alarm():
            print("清除運動警報")
            dobot.clear_motion_alarm()
            time.sleep(1.0)
            return False
        
        if not dobot.is_motion_ready():
            print("等待運動系統準備就緒")
            if not dobot.wait_for_motion_ready(10.0):
                return False
        
        # 執行操作
        return operation_func()
        
    except Exception as e:
        print(f"Dobot運動操作異常: {e}")
        return False

def safe_dobot_io_operation(operation_func):
    """Dobot IO類安全操作包裝器"""
    try:
        # 檢查連接狀態
        if not dobot.is_connected():
            if not dobot.connect():
                return False
        
        # IO類操作無需檢查運動狀態
        return operation_func()
        
    except Exception as e:
        print(f"Dobot IO操作異常: {e}")
        return False
```

#### 2. Flow執行監控器
```python
class DobotFlowMonitor:
    """Dobot Flow執行監控器"""
    
    def __init__(self, dobot_controller: DobotM1ProGUIClass):
        self.dobot = dobot_controller
        self.execution_history = []
        self.dobot.add_status_callback(self.on_status_changed)
        self.dobot.add_result_callback(self.on_result_updated)
    
    def on_status_changed(self, status: DobotSystemStatus):
        """狀態變化處理"""
        # 記錄運動狀態變化
        if status.motion_status.running:
            current_flow, flow_name = self.dobot.get_current_motion_flow()
            if current_flow > 0:
                print(f"運動Flow{current_flow}執行中: {flow_name}")
                print(f"進度: {status.motion_status.motion_progress}%")
        
        # 記錄Flow完成
        completion_status = self.dobot.get_flow_completion_status()
        for flow, completed in completion_status.items():
            if completed:
                flow_id = flow.replace('flow', '').replace('_complete', '')
                print(f"Flow{flow_id}執行完成")
    
    def on_result_updated(self, result: DobotOperationResult):
        """結果更新處理"""
        self.execution_history.append({
            'timestamp': result.timestamp,
            'flow_type': result.flow_type.value,
            'flow_id': result.flow_id,
            'success': result.success,
            'operation': result.operation_type,
            'error': result.error_message if not result.success else None
        })
        
        # 保持最近100筆記錄
        if len(self.execution_history) > 100:
            self.execution_history.pop(0)
    
    def get_execution_summary(self) -> Dict[str, Any]:
        """獲取執行摘要"""
        motion_stats = self.dobot.get_motion_statistics()
        return {
            'motion_operations': motion_stats['operation_count'],
            'motion_errors': motion_stats['error_count'],
            'run_time_minutes': motion_stats['run_time_minutes'],
            'recent_executions': len(self.execution_history),
            'last_10_operations': self.execution_history[-10:] if self.execution_history else []
        }
```

#### 3. 混合Flow控制器
```python
class DobotMixedFlowController:
    """Dobot混合Flow控制器 - 支援運動+IO併行"""
    
    def __init__(self, dobot_controller: DobotM1ProGUIClass):
        self.dobot = dobot_controller
    
    def execute_mixed_operation(self, motion_flow_id: int, io_flows: List[int]) -> bool:
        """執行混合操作: 運動Flow + IO Flow併行"""
        try:
            # 1. 啟動IO類Flow (併行執行)
            for io_flow in io_flows:
                if io_flow == 3:
                    self.dobot.start_flow3_flip_station()
                elif io_flow == 4:
                    self.dobot.start_flow4_vibration_feed()
            
            # 2. 啟動運動類Flow
            if motion_flow_id == 1:
                success = self.dobot.start_flow1_vp_vision_pick()
            elif motion_flow_id == 2:
                success = self.dobot.start_flow2_cv_unload()
            elif motion_flow_id == 5:
                success = self.dobot.start_flow5_assembly()
            else:
                return False
            
            if not success:
                return False
            
            # 3. 等待運動Flow完成
            if self.dobot.wait_for_flow_complete(motion_flow_id, 300.0):
                # 4. 清空運動控制寄存器
                self.dobot.clear_motion_flow(motion_flow_id)
                return True
            else:
                return False
                
        except Exception as e:
            print(f"混合操作異常: {e}")
            return False
    
    def execute_production_cycle(self) -> bool:
        """執行生產週期: Flow1+Flow3 -> Flow2+Flow4"""
        # 第一階段: VP視覺抓取 + 翻轉站併行
        if not self.execute_mixed_operation(1, [3]):
            return False
        
        # 等待運動系統準備
        if not self.dobot.wait_for_motion_ready(10.0):
            return False
        
        # 第二階段: CV出料 + 震動投料併行
        if not self.execute_mixed_operation(2, [4]):
            return False
        
        return True
```

## GUI介面設計建議

### 1. 狀態顯示區域
```python
class DobotStatusDisplay:
    def __init__(self, dobot_controller):
        self.dobot = dobot_controller
        self.dobot.add_status_callback(self.update_status_display)
    
    def update_status_display(self, status: DobotSystemStatus):
        """更新狀態顯示"""
        # 運動狀態指示燈
        self.motion_ready_led.color = "green" if status.motion_status.ready else "gray"
        self.motion_running_led.color = "yellow" if status.motion_status.running else "gray"
        self.motion_alarm_led.color = "red" if status.motion_status.alarm else "gray"
        
        # 當前運動Flow顯示
        if status.motion_status.current_motion_flow > 0:
            flow_name = self.dobot.FLOW_DESCRIPTIONS[status.motion_status.current_motion_flow]
            self.current_flow_label.text = f"當前Flow: {flow_name}"
            self.progress_bar.value = status.motion_status.motion_progress
        else:
            self.current_flow_label.text = "當前Flow: 無"
            self.progress_bar.value = 0
        
        # Flow完成狀態
        self.flow1_complete_led.color = "blue" if status.motion_status.flow1_complete else "gray"
        self.flow2_complete_led.color = "blue" if status.motion_status.flow2_complete else "gray"
        self.flow5_complete_led.color = "blue" if status.motion_status.flow5_complete else "gray"
        
        # IO狀態指示
        self.flow3_active_led.color = "orange" if status.io_status.flow3_active else "gray"
        self.flow4_active_led.color = "orange" if status.io_status.flow4_active else "gray"
        
        # 統計資訊
        self.operation_count_label.text = f"運動操作: {status.motion_status.motion_op_count}"
        self.error_count_label.text = f"運動錯誤: {status.motion_status.motion_err_count}"
        self.run_time_label.text = f"運行時間: {status.motion_status.motion_run_time}分鐘"
```

### 2. 控制按鈕區域
```python
class DobotControlPanel:
    def __init__(self, dobot_controller):
        self.dobot = dobot_controller
        self.setup_buttons()
    
    def setup_buttons(self):
        """設置控制按鈕"""
        # 運動類Flow按鈕 (需要檢查Ready狀態)
        self.flow1_btn.on_click = self.on_flow1_clicked
        self.flow2_btn.on_click = self.on_flow2_clicked
        self.flow5_btn.on_click = self.on_flow5_clicked
        
        # IO類Flow按鈕 (可直接執行)
        self.flow3_btn.on_click = self.on_flow3_clicked
        self.flow4_btn.on_click = self.on_flow4_clicked
        
        # 控制按鈕
        self.clear_alarm_btn.on_click = self.on_clear_alarm_clicked
        self.emergency_stop_btn.on_click = self.on_emergency_stop_clicked
        
        # 序列執行按鈕
        self.sequence_btn.on_click = self.on_sequence_clicked
        self.mixed_operation_btn.on_click = self.on_mixed_operation_clicked
    
    def on_flow1_clicked(self):
        """Flow1按鈕點擊"""
        if self.check_motion_ready("Flow1 VP視覺抓取"):
            self.dobot.start_flow1_vp_vision_pick()
    
    def on_flow2_clicked(self):
        """Flow2按鈕點擊"""
        if self.check_motion_ready("Flow2 CV出料"):
            self.dobot.start_flow2_cv_unload()
    
    def on_flow5_clicked(self):
        """Flow5按鈕點擊"""
        if self.check_motion_ready("Flow5 機械臂運轉"):
            self.dobot.start_flow5_assembly()
    
    def on_flow3_clicked(self):
        """Flow3按鈕點擊 (IO類，可直接執行)"""
        self.dobot.start_flow3_flip_station()
    
    def on_flow4_clicked(self):
        """Flow4按鈕點擊 (IO類，可直接執行)"""
        self.dobot.start_flow4_vibration_feed()
    
    def check_motion_ready(self, operation_name: str) -> bool:
        """檢查運動系統是否準備好"""
        if not self.dobot.is_connected():
            show_message(f"{operation_name}: 未連接到服務器")
            return False
        
        if self.dobot.is_motion_alarm():
            show_message(f"{operation_name}: 運動系統警報，請先清除")
            return False
        
        if not self.dobot.is_motion_ready():
            show_message(f"{operation_name}: 運動系統未準備好或執行中")
            return False
        
        return True
    
    def on_sequence_clicked(self):
        """序列執行按鈕"""
        if self.check_motion_ready("Flow序列執行"):
            # 執行Flow1 -> Flow2 -> Flow5
            threading.Thread(target=self.execute_sequence, daemon=True).start()
    
    def execute_sequence(self):
        """執行序列"""
        sequence = [1, 2, 5]
        if self.dobot.execute_flow_sequence(sequence):
            show_message("Flow序列執行完成")
        else:
            show_message("Flow序列執行失敗")
    
    def on_mixed_operation_clicked(self):
        """混合操作按鈕"""
        if self.check_motion_ready("混合操作"):
            # Flow1 + Flow3併行
            threading.Thread(target=self.execute_mixed_operation, daemon=True).start()
    
    def execute_mixed_operation(self):
        """執行混合操作"""
        # 啟動Flow3 (IO類併行)
        self.dobot.start_flow3_flip_station()
        # 啟動Flow1 (運動類)
        if self.dobot.start_flow1_vp_vision_pick():
            if self.dobot.wait_for_flow_complete(1, 300.0):
                self.dobot.clear_motion_flow(1)
                show_message("混合操作完成")
            else:
                show_message("混合操作超時")
        else:
            show_message("混合操作啟動失敗")
```

### 3. 進度監控區域
```python
class DobotProgressMonitor:
    def __init__(self, dobot_controller):
        self.dobot = dobot_controller
        self.dobot.add_status_callback(self.update_progress)
    
    def update_progress(self, status: DobotSystemStatus):
        """更新進度顯示"""
        if status.motion_status.running and status.motion_status.current_motion_flow > 0:
            flow_name = self.dobot.FLOW_DESCRIPTIONS[status.motion_status.current_motion_flow]
            
            # 更新進度條
            self.progress_bar.value = status.motion_status.motion_progress
            self.progress_label.text = f"{flow_name}: {status.motion_status.motion_progress}%"
            
            # 更新估計剩餘時間 (如果有的話)
            if status.motion_status.motion_progress > 0:
                # 簡單估算剩餘時間 (實際應用中需要更複雜的算法)
                estimated_total_time = 180  # 假設180秒
                elapsed_ratio = status.motion_status.motion_progress / 100.0
                remaining_time = estimated_total_time * (1 - elapsed_ratio)
                self.remaining_time_label.text = f"預估剩餘: {int(remaining_time)}秒"
        else:
            self.progress_bar.value = 0
            self.progress_label.text = "無運動執行中"
            self.remaining_time_label.text = ""
```

## 寄存器映射參考

### 運動類寄存器 (基地址1200-1249) - 修正地址衝突版本

#### 運動狀態寄存器 (1200-1219) - 只讀
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 1200 | 運動狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized |
| 1201 | 當前運動Flow | 0=無, 1=Flow1, 2=Flow2, 5=Flow5 |
| 1202 | 運動進度 | 0-100百分比 |
| 1203 | 運動錯誤碼 | 0=無錯誤, >0=錯誤類型 |
| 1204 | Flow1完成狀態 | 0=未完成, 1=完成且角度校正成功 |
| 1205 | Flow2完成狀態 | 0=未完成, 1=完成 |
| 1206 | Flow5完成狀態 | 0=未完成, 1=完成 |
| 1207 | 運動操作計數 | 累積成功次數 |
| 1208 | 運動錯誤計數 | 累積錯誤次數 |
| 1209 | 運動系統運行時間 | 分鐘數 |

#### 運動控制寄存器 (1240-1249) - 讀寫
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 1240 | Flow1控制 | 0=清空, 1=啟動VP視覺抓取 |
| 1241 | Flow2控制 | 0=清空, 1=啟動出料流程 |
| 1242 | Flow5控制 | 0=清空, 1=啟動機械臂運轉 |
| 1243 | 運動清除警報 | 0=無動作, 1=清除Alarm |
| 1244 | 運動緊急停止 | 0=正常, 1=緊急停止 |

### IO類寄存器 (447-449) - 保持不變
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 447 | Flow3控制 | 0=清空, 1=啟動翻轉站 |
| 448 | Flow4控制 | 0=清空, 1=啟動震動投料 |
| 449 | 保留IO控制 | 未來IO Flow擴展 |

## 交握協議說明

### 運動類Flow交握協議 (狀態機)
```
1. 檢查準備: 讀取1200寄存器bit0=1 (Ready=1)
2. 啟動Flow: 寫入對應控制寄存器=1 (1240/1241/1242)
3. 系統響應: 1200寄存器bit1=1 (Running=1), bit0=0 (Ready=0)
4. 執行監控: 讀取1201(當前Flow), 1202(進度)
5. 等待完成: 1200寄存器bit1=0 (Running=0), 對應完成標誌=1
6. 清零指令: 寫入控制寄存器=0
7. 系統恢復: 1200寄存器bit0=1 (Ready=1)
```

### IO類Flow併行協議 (佇列)
```
1. 直接觸發: 寫入447或448=1
2. 系統處理: 自動加入對應佇列
3. 異步執行: 不阻塞運動Flow
4. 自動清零: 寄存器自動變為0
5. 可重複: 隨時可再次觸發
```

## 集成注意事項

### 1. 線程安全
- DobotM1ProGUIClass內部使用線程鎖保護共享狀態
- 回調函數在監控線程中執行，GUI更新需要切換到主線程
- 建議使用訊號-槽機制或事件佇列進行跨線程通訊

### 2. 資源管理
```python
# 應用程式退出時的清理
def on_application_exit():
    """應用程式退出清理"""
    if dobot:
        # 緊急停止所有運動
        dobot.motion_emergency_stop()
        time.sleep(1.0)
        # 斷開連接
        dobot.disconnect()
```

### 3. 異常處理
- 所有Modbus操作都包含異常處理
- 網路斷線時會自動停止監控
- 建議實現自動重連機制
- 運動類和IO類錯誤分別處理

### 4. 效能考慮
- 狀態監控頻率為200ms，適合即時顯示
- 避免在回調函數中執行耗時操作
- Flow序列執行時注意超時設定
- 混合操作時注意資源協調

### 5. 地址衝突解決
- **重要**: 使用新地址範圍1200-1249，避開CCD2模組1000-1099
- 原地址1100-1149已廢棄，不可使用
- IO類地址447-449保持不變
- 確保與其他模組無地址衝突

## 使用範例

完整的GUI整合範例:

```python
from DobotM1ProGUIClass import DobotM1ProGUIClass, DobotSystemStatus, DobotOperationResult, FlowType

class UnifiedMachineGUI:
    def __init__(self):
        # 初始化Dobot控制器
        self.dobot = DobotM1ProGUIClass()
        
        # 註冊回調
        self.dobot.add_status_callback(self.on_dobot_status_changed)
        self.dobot.add_result_callback(self.on_dobot_result_updated)
        
        # 建立GUI界面
        self.setup_gui()
        
        # 連接Dobot
        self.connect_dobot()
    
    def setup_gui(self):
        """建立GUI界面"""
        # 狀態顯示區域
        self.setup_status_display()
        
        # 運動類Flow控制區域
        self.setup_motion_controls()
        
        # IO類Flow控制區域
        self.setup_io_controls()
        
        # 進階操作區域
        self.setup_advanced_controls()
    
    def setup_status_display(self):
        """設置狀態顯示"""
        # 運動狀態指示燈組
        self.motion_status_group = StatusLEDGroup([
            ("Ready", "green"),
            ("Running", "yellow"), 
            ("Alarm", "red"),
            ("Initialized", "blue")
        ])
        
        # Flow完成狀態指示燈組
        self.flow_complete_group = StatusLEDGroup([
            ("Flow1完成", "blue"),
            ("Flow2完成", "blue"),
            ("Flow5完成", "blue")
        ])
        
        # IO狀態指示燈組
        self.io_status_group = StatusLEDGroup([
            ("Flow3執行中", "orange"),
            ("Flow4執行中", "orange")
        ])
    
    def setup_motion_controls(self):
        """設置運動控制"""
        # 運動類Flow按鈕
        self.flow1_btn = Button("Flow1 VP視覺抓取", self.on_flow1_clicked)
        self.flow2_btn = Button("Flow2 CV出料", self.on_flow2_clicked)
        self.flow5_btn = Button("Flow5 機械臂運轉", self.on_flow5_clicked)
        
        # 運動控制按鈕
        self.clear_alarm_btn = Button("清除警報", self.on_clear_alarm_clicked)
        self.emergency_stop_btn = Button("緊急停止", self.on_emergency_stop_clicked)
    
    def setup_io_controls(self):
        """設置IO控制"""
        # IO類Flow按鈕 (可併行執行)
        self.flow3_btn = Button("Flow3 翻轉站", self.on_flow3_clicked)
        self.flow4_btn = Button("Flow4 震動投料", self.on_flow4_clicked)
    
    def setup_advanced_controls(self):
        """設置進階控制"""
        self.sequence_btn = Button("執行序列 (1→2→5)", self.on_sequence_clicked)
        self.mixed_btn = Button("混合操作 (1+3併行)", self.on_mixed_clicked)
        self.production_cycle_btn = Button("生產週期", self.on_production_cycle_clicked)
    
    def connect_dobot(self):
        """連接Dobot模組"""
        if self.dobot.connect():
            self.update_status("Dobot連接成功 - 新架構混合交握協議")
            
            # 顯示地址資訊
            connection_info = self.dobot.get_connection_info()
            self.update_status(f"運動類基地址: {connection_info['motion_base_address']}")
            self.update_status(f"IO類基地址: {connection_info['io_base_address']}")
        else:
            self.update_status("Dobot連接失敗")
    
    def on_dobot_status_changed(self, status: DobotSystemStatus):
        """Dobot狀態變化處理"""
        # 更新運動狀態指示燈
        self.motion_status_group.update({
            "Ready": status.motion_status.ready,
            "Running": status.motion_status.running,
            "Alarm": status.motion_status.alarm,
            "Initialized": status.motion_status.initialized
        })
        
        # 更新Flow完成指示燈
        self.flow_complete_group.update({
            "Flow1完成": status.motion_status.flow1_complete,
            "Flow2完成": status.motion_status.flow2_complete,
            "Flow5完成": status.motion_status.flow5_complete
        })
        
        # 更新IO狀態指示燈
        self.io_status_group.update({
            "Flow3執行中": status.io_status.flow3_active,
            "Flow4執行中": status.io_status.flow4_active
        })
        
        # 更新當前Flow和進度
        if status.motion_status.current_motion_flow > 0:
            flow_id, flow_name = self.dobot.get_current_motion_flow()
            self.current_flow_label.text = f"當前: {flow_name}"
            self.progress_bar.value = status.motion_status.motion_progress
        else:
            self.current_flow_label.text = "當前: 無"
            self.progress_bar.value = 0
        
        # 更新按鈕狀態
        motion_ready = status.motion_status.ready and not status.motion_status.running
        self.flow1_btn.enabled = motion_ready
        self.flow2_btn.enabled = motion_ready
        self.flow5_btn.enabled = motion_ready
        
        # IO按鈕始終可用 (併行執行)
        self.flow3_btn.enabled = True
        self.flow4_btn.enabled = True
    
    def on_dobot_result_updated(self, result: DobotOperationResult):
        """Dobot操作結果更新處理"""
        flow_type_str = "運動類" if result.flow_type == FlowType.MOTION else "IO類"
        
        if result.success:
            self.update_status(f"✓ {flow_type_str}Flow{result.flow_id}啟動成功: {result.operation_type}")
        else:
            self.update_status(f"✗ {flow_type_str}Flow{result.flow_id}啟動失敗: {result.error_message}")
    
    def on_flow1_clicked(self):
        """Flow1按鈕點擊"""
        self.dobot.start_flow1_vp_vision_pick()
    
    def on_flow2_clicked(self):
        """Flow2按鈕點擊"""
        self.dobot.start_flow2_cv_unload()
    
    def on_flow5_clicked(self):
        """Flow5按鈕點擊"""
        self.dobot.start_flow5_assembly()
    
    def on_flow3_clicked(self):
        """Flow3按鈕點擊 (IO類併行)"""
        self.dobot.start_flow3_flip_station()
    
    def on_flow4_clicked(self):
        """Flow4按鈕點擊 (IO類併行)"""
        self.dobot.start_flow4_vibration_feed()
    
    def on_clear_alarm_clicked(self):
        """清除警報按鈕"""
        if self.dobot.clear_motion_alarm():
            self.update_status("運動警報清除指令已發送")
    
    def on_emergency_stop_clicked(self):
        """緊急停止按鈕"""
        if self.dobot.motion_emergency_stop():
            self.update_status("運動緊急停止指令已發送")
    
    def on_sequence_clicked(self):
        """序列執行按鈕"""
        def execute_sequence():
            self.update_status("開始執行Flow序列: 1→2→5")
            if self.dobot.execute_flow_sequence([1, 2, 5]):
                self.update_status("✓ Flow序列執行完成")
            else:
                self.update_status("✗ Flow序列執行失敗")
        
        # 在背景線程執行
        threading.Thread(target=execute_sequence, daemon=True).start()
    
    def on_mixed_clicked(self):
        """混合操作按鈕"""
        def execute_mixed():
            self.update_status("開始混合操作: Flow1運動 + Flow3併行")
            
            # 啟動Flow3 (IO類併行)
            self.dobot.start_flow3_flip_station()
            
            # 啟動Flow1 (運動類)
            if self.dobot.start_flow1_vp_vision_pick():
                if self.dobot.wait_for_flow_complete(1, 300.0):
                    self.dobot.clear_motion_flow(1)
                    self.update_status("✓ 混合操作完成")
                else:
                    self.update_status("✗ 混合操作超時")
            else:
                self.update_status("✗ 混合操作啟動失敗")
        
        # 在背景線程執行
        threading.Thread(target=execute_mixed, daemon=True).start()
    
    def on_production_cycle_clicked(self):
        """生產週期按鈕"""
        def execute_production_cycle():
            self.update_status("開始生產週期")
            
            # 第一階段: Flow1 + Flow3併行
            self.dobot.start_flow3_flip_station()
            if self.dobot.start_flow1_vp_vision_pick():
                if self.dobot.wait_for_flow_complete(1, 300.0):
                    self.dobot.clear_motion_flow(1)
                    
                    # 等待運動系統準備
                    if self.dobot.wait_for_motion_ready(10.0):
                        # 第二階段: Flow2 + Flow4併行
                        self.dobot.start_flow4_vibration_feed()
                        if self.dobot.start_flow2_cv_unload():
                            if self.dobot.wait_for_flow_complete(2, 300.0):
                                self.dobot.clear_motion_flow(2)
                                self.update_status("✓ 生產週期完成")
                            else:
                                self.update_status("✗ Flow2執行超時")
                        else:
                            self.update_status("✗ Flow2啟動失敗")
                    else:
                        self.update_status("✗ 運動系統準備超時")
                else:
                    self.update_status("✗ Flow1執行超時")
            else:
                self.update_status("✗ Flow1啟動失敗")
        
        # 在背景線程執行
        threading.Thread(target=execute_production_cycle, daemon=True).start()

# 使用範例
if __name__ == "__main__":
    app = UnifiedMachineGUI()
    app.run()
```

## 架構優勢總結

### 1. 安全性
- **運動類Flow**: 嚴格的狀態機交握，確保機械臂安全
- **序列化執行**: 一次只能執行一個運動Flow，避免運動衝突
- **警報處理**: 完整的錯誤檢測和警報清除機制

### 2. 效率性
- **IO類Flow**: 採用佇列併行執行，不影響運動Flow
- **混合操作**: 支援運動+IO同時進行，最大化系統利用率
- **快速響應**: 200ms狀態監控，50ms系統交握循環

### 3. 靈活性
- **模組化設計**: 運動類和IO類完全獨立，可分別擴展
- **多種操作模式**: 單一Flow、序列執行、混合操作、生產週期
- **回調機制**: 靈活的狀態變化和結果通知

### 4. 相容性
- **地址安全**: 新地址1200-1249避開所有現有模組
- **向後相容**: IO類地址447-449保持不變
- **標準協議**: 基於Modbus TCP標準通訊協議

### 5. 可維護性
- **清晰架構**: 運動類和IO類職責分離
- **完整監控**: 詳細的狀態資訊和統計數據
- **調試友好**: 豐富的狀態描述和錯誤資訊

這個設計指南提供了完整的DobotM1ProGUIClass使用框架，支援統一機台調適工具的開發需求，確保新架構混合交握協議的所有功能都能透過簡潔的API介面進行控制。