# GripperGUIClass設計指南

## 概述

GripperGUIClass是為統一機台調適工具設計的Gripper夾爪模組控制類別，提供完整的三款夾爪(PGC、PGHL、PGE)統一操作介面。

## 架構設計

### 設計原則
- 統一三款夾爪的操作介面
- 封裝不同夾爪的參數差異
- 提供型號特定功能支援
- 支援批量操作和單獨控制
- 線程安全的狀態管理

### 通訊架構
```
統一GUI工具
    |
    |-- import --> GripperGUIClass
                       |
                       |-- ModbusTCP Client --> 主服務器(502)
                       |                           |
                       |                           |-- Gripper模組(基地址500-589)
                       |                                     |
                       |                                     |-- PGC(500-529)
                       |                                     |-- PGHL(530-559)
                       |                                     |-- PGE(560-579)
                       |
                       |-- RTU橋接 --> COM5 --> 三款夾爪硬體
```

## 主要功能模組

### 1. 連接管理
- `connect()`: 建立Modbus TCP連接
- `disconnect()`: 斷開連接
- `is_connected()`: 檢查連接狀態

### 2. 狀態監控
- 200ms輪詢頻率自動更新三款夾爪狀態
- 支援狀態變化回調通知
- 統一的狀態結構處理

### 3. 指令控制
- `initialize_gripper()`: 初始化/回零
- `move_to_position()`: 絕對位置移動
- `move_relative()`: 相對位置移動(僅PGHL)
- `set_force()`: 設定夾持力道
- `set_speed()`: 設定移動速度
- `quick_open()` / `quick_close()`: 快速開關
- `stop_gripper()`: 停止運動

### 4. 批量操作
- `initialize_all_grippers()`: 初始化所有夾爪
- `stop_all_grippers()`: 停止所有夾爪
- `open_all_grippers()` / `close_all_grippers()`: 批量開關

### 5. 狀態獲取
- `get_gripper_status()`: 獲取單個夾爪狀態
- `get_all_gripper_status()`: 獲取所有夾爪狀態
- `is_gripper_ready()`: 檢查夾爪就緒狀態

## 數據結構定義

### GripperType - 夾爪類型枚舉
```python
class GripperType(IntEnum):
    PGC = 6      # unit_id=6, 寄存器500-529
    PGHL = 5     # unit_id=5, 寄存器530-559
    PGE = 4      # unit_id=4, 寄存器560-579
```

### GripperStatus - 夾爪狀態
```python
@dataclass
class GripperStatus:
    gripper_type: GripperType
    module_state: int = 0        # 模組狀態(0=離線, 1=在線)
    connection_state: int = 0    # 連接狀態(0=斷開, 1=已連接)
    device_state: int = 0        # 設備狀態(初始化狀態)
    error_count: int = 0         # 錯誤計數
    motion_state: int = 0        # 夾持/運動狀態
    current_position: int = 0    # 當前位置
    current_value: int = 0       # 電流值(僅PGHL)
    timestamp: int = 0           # 時間戳
    
    # 便利屬性
    @property
    def is_online(self) -> bool
    def is_connected(self) -> bool
    def is_initialized(self) -> bool
    def is_moving(self) -> bool
```

### GripperConfig - 夾爪配置
```python
@dataclass
class GripperConfig:
    gripper_type: GripperType
    unit_id: int
    enabled: bool = True
    position_range: tuple = (0, 1000)    # 位置範圍
    force_range: tuple = (20, 100)       # 力道範圍
    speed_range: tuple = (1, 100)        # 速度範圍
    has_current_feedback: bool = False   # 是否有電流反饋
    supports_relative_move: bool = False # 是否支援相對移動
    position_precision: float = 1.0     # 位置精度
```

### GripperCommand - 控制指令枚舉
```python
class GripperCommand(IntEnum):
    NOP = 0              # 無操作
    INITIALIZE = 1       # 初始化/回零
    STOP = 2             # 停止當前動作
    ABSOLUTE_POSITION = 3 # 絕對位置移動
    RELATIVE_POSITION = 4 # 相對位置移動(僅PGHL)
    SET_FORCE = 5        # 設定力道
    SET_SPEED = 6        # 設定速度
    QUICK_OPEN = 7       # 快速開啟
    QUICK_CLOSE = 8      # 快速關閉
```

## GUI實現指南

### 基本使用模式

#### 1. 導入和初始化
```python
from GripperGUIClass import GripperGUIClass, GripperType, GripperStatus

# 初始化Gripper控制器
gripper = GripperGUIClass(modbus_host="127.0.0.1", modbus_port=502)

# 連接到Modbus服務器
if gripper.connect():
    print("Gripper連接成功")
else:
    print("Gripper連接失敗")
```

#### 2. 狀態監控實現
```python
def on_gripper_status_changed(statuses: Dict[GripperType, GripperStatus]):
    """夾爪狀態變化回調函數"""
    for gripper_type, status in statuses.items():
        name = gripper.get_gripper_type_name(gripper_type)
        print(f"{name}:")
        print(f"  在線: {status.is_online}")
        print(f"  已連接: {status.is_connected}")
        print(f"  已初始化: {status.is_initialized}")
        print(f"  運動中: {status.is_moving}")
        print(f"  當前位置: {gripper.format_position(gripper_type, status.current_position)}")
        print(f"  錯誤計數: {status.error_count}")
        
        # PGHL特有的電流反饋
        config = gripper.get_gripper_config(gripper_type)
        if config.has_current_feedback:
            print(f"  電流值: {gripper.format_current(status.current_value)}")

# 註冊回調函數
gripper.add_status_callback(on_gripper_status_changed)
```

#### 3. 單個夾爪控制實現
```python
# 選擇夾爪類型
target_gripper = GripperType.PGC

# 檢查夾爪是否準備好
if gripper.is_gripper_ready(target_gripper):
    print(f"{gripper.get_gripper_type_name(target_gripper)}已準備好")
    
    # 移動到指定位置
    if gripper.move_to_position(target_gripper, 500):
        print("位置指令發送成功")
    
    # 設定力道
    if gripper.set_force(target_gripper, 80):
        print("力道設定成功")
    
    # 設定速度
    if gripper.set_speed(target_gripper, 50):
        print("速度設定成功")
    
    # 快速開關操作
    if gripper.quick_open(target_gripper):
        print("快速開啟指令發送成功")
        
else:
    status = gripper.get_gripper_status(target_gripper)
    print(f"夾爪未準備好: 在線={status.is_online}, 初始化={status.is_initialized}")
```

#### 4. PGHL特殊功能使用
```python
# PGHL支援相對移動
pghl_gripper = GripperType.PGHL

if gripper.is_gripper_ready(pghl_gripper):
    # 相對移動 +100 (0.01mm精度，實際移動1mm)
    if gripper.move_relative(pghl_gripper, 100):
        print("PGHL相對移動指令發送成功")
    
    # 檢查電流反饋
    status = gripper.get_gripper_status(pghl_gripper)
    print(f"PGHL電流值: {gripper.format_current(status.current_value)}")
```

#### 5. 批量操作實現
```python
# 初始化所有夾爪
init_results = gripper.initialize_all_grippers()
for gripper_type, success in init_results.items():
    name = gripper.get_gripper_type_name(gripper_type)
    print(f"{name}初始化: {'成功' if success else '失敗'}")

# 開啟所有夾爪
open_results = gripper.open_all_grippers()
success_count = sum(1 for success in open_results.values() if success)
print(f"批量開啟: {success_count}/{len(open_results)} 成功")

# 停止所有夾爪
stop_results = gripper.stop_all_grippers()
print("緊急停止所有夾爪")
```

### 進階使用技巧

#### 1. 夾爪配置管理
```python
def display_gripper_configurations():
    """顯示所有夾爪配置"""
    configs = gripper.get_all_gripper_configs()
    
    for gripper_type, config in configs.items():
        name = gripper.get_gripper_type_name(gripper_type)
        print(f"\n{name}配置:")
        print(f"  位置範圍: {config.position_range}")
        print(f"  力道範圍: {config.force_range}")
        print(f"  速度範圍: {config.speed_range}")
        print(f"  位置精度: {config.position_precision}")
        print(f"  電流反饋: {'是' if config.has_current_feedback else '否'}")
        print(f"  相對移動: {'是' if config.supports_relative_move else '否'}")

# 使用配置進行參數驗證
def safe_move_to_position(gripper_type: GripperType, position: int) -> bool:
    """安全的位置移動"""
    config = gripper.get_gripper_config(gripper_type)
    
    # 檢查位置範圍
    if not (config.position_range[0] <= position <= config.position_range[1]):
        print(f"位置{position}超出{gripper.get_gripper_type_name(gripper_type)}範圍{config.position_range}")
        return False
    
    # 檢查夾爪狀態
    if not gripper.is_gripper_ready(gripper_type):
        print(f"{gripper.get_gripper_type_name(gripper_type)}未準備好")
        return False
    
    return gripper.move_to_position(gripper_type, position)
```

#### 2. 狀態機控制
```python
def wait_for_gripper_ready(gripper_type: GripperType, timeout: float = 10.0) -> bool:
    """等待夾爪準備好"""
    import time
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        if gripper.is_gripper_ready(gripper_type):
            return True
        time.sleep(0.1)
    
    return False

def execute_gripper_sequence(gripper_type: GripperType, positions: List[int]) -> bool:
    """執行夾爪位置序列"""
    name = gripper.get_gripper_type_name(gripper_type)
    
    for i, position in enumerate(positions):
        print(f"{name}移動到位置{i+1}: {position}")
        
        if not safe_move_to_position(gripper_type, position):
            print(f"位置{i+1}移動失敗")
            return False
        
        # 等待到達
        if not wait_for_movement_complete(gripper_type, timeout=5.0):
            print(f"位置{i+1}移動超時")
            return False
    
    print(f"{name}序列執行完成")
    return True

def wait_for_movement_complete(gripper_type: GripperType, timeout: float = 5.0) -> bool:
    """等待移動完成"""
    import time
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        status = gripper.get_gripper_status(gripper_type)
        if not status.is_moving:
            return True
        time.sleep(0.1)
    
    return False
```

#### 3. 錯誤處理和恢復
```python
def handle_gripper_errors():
    """處理夾爪錯誤"""
    all_status = gripper.get_all_gripper_status()
    
    for gripper_type, status in all_status.items():
        name = gripper.get_gripper_type_name(gripper_type)
        
        # 檢查錯誤計數
        if status.error_count > 0:
            print(f"{name}有{status.error_count}個錯誤")
            
            # 嘗試重新初始化
            if not status.is_initialized:
                print(f"重新初始化{name}")
                gripper.initialize_gripper(gripper_type)
        
        # 檢查連接狀態
        if not status.is_connected:
            print(f"{name}連接斷開")
            # 可以觸發重連或告警

def emergency_stop_all():
    """緊急停止所有夾爪"""
    print("執行緊急停止")
    stop_results = gripper.stop_all_grippers()
    
    for gripper_type, success in stop_results.items():
        name = gripper.get_gripper_type_name(gripper_type)
        if success:
            print(f"{name}已停止")
        else:
            print(f"{name}停止指令發送失敗")
```

#### 4. 多夾爪協調操作
```python
def synchronized_gripper_operation():
    """同步夾爪操作"""
    ready_grippers = gripper.get_ready_grippers()
    
    if len(ready_grippers) == 0:
        print("沒有準備好的夾爪")
        return False
    
    print(f"準備好的夾爪: {[gripper.get_gripper_type_name(gt) for gt in ready_grippers]}")
    
    # 階段1: 全部開啟
    print("階段1: 開啟所有準備好的夾爪")
    for gripper_type in ready_grippers:
        gripper.quick_open(gripper_type)
    
    # 等待所有夾爪完成
    for gripper_type in ready_grippers:
        wait_for_movement_complete(gripper_type)
    
    # 階段2: 移動到中間位置
    print("階段2: 移動到中間位置")
    for gripper_type in ready_grippers:
        config = gripper.get_gripper_config(gripper_type)
        mid_position = (config.position_range[0] + config.position_range[1]) // 2
        gripper.move_to_position(gripper_type, mid_position)
    
    # 等待所有夾爪完成
    for gripper_type in ready_grippers:
        wait_for_movement_complete(gripper_type)
    
    # 階段3: 全部關閉
    print("階段3: 關閉所有夾爪")
    for gripper_type in ready_grippers:
        gripper.quick_close(gripper_type)
    
    print("同步操作完成")
    return True
```

## GUI介面設計建議

### 1. 夾爪狀態顯示區域
```python
# 夾爪狀態面板UI範例
class GripperStatusPanel:
    def __init__(self, gripper_controller):
        self.gripper = gripper_controller
        self.gripper.add_status_callback(self.on_status_updated)
        self.status_widgets = {}
        
        # 為每個夾爪創建狀態顯示區域
        for gripper_type in GripperType:
            self.create_gripper_status_widget(gripper_type)
    
    def create_gripper_status_widget(self, gripper_type: GripperType):
        """創建單個夾爪狀態顯示"""
        name = self.gripper.get_gripper_type_name(gripper_type)
        config = self.gripper.get_gripper_config(gripper_type)
        
        widget = {
            'name_label': create_label(name),
            'online_indicator': create_status_indicator("在線"),
            'connected_indicator': create_status_indicator("已連接"),
            'initialized_indicator': create_status_indicator("已初始化"),
            'moving_indicator': create_status_indicator("運動中"),
            'position_label': create_label("位置: --"),
            'error_count_label': create_label("錯誤: 0"),
        }
        
        # PGHL特有的電流顯示
        if config.has_current_feedback:
            widget['current_label'] = create_label("電流: --")
        
        self.status_widgets[gripper_type] = widget
    
    def on_status_updated(self, statuses: Dict[GripperType, GripperStatus]):
        """更新狀態顯示"""
        for gripper_type, status in statuses.items():
            widget = self.status_widgets[gripper_type]
            config = self.gripper.get_gripper_config(gripper_type)
            
            # 更新狀態指示燈
            widget['online_indicator'].set_state(status.is_online)
            widget['connected_indicator'].set_state(status.is_connected)
            widget['initialized_indicator'].set_state(status.is_initialized)
            widget['moving_indicator'].set_state(status.is_moving)
            
            # 更新位置顯示
            position_text = self.gripper.format_position(gripper_type, status.current_position)
            widget['position_label'].text = f"位置: {position_text}"
            
            # 更新錯誤計數
            widget['error_count_label'].text = f"錯誤: {status.error_count}"
            if status.error_count > 0:
                widget['error_count_label'].color = "red"
            else:
                widget['error_count_label'].color = "black"
            
            # 更新電流顯示(PGHL)
            if config.has_current_feedback and 'current_label' in widget:
                current_text = self.gripper.format_current(status.current_value)
                widget['current_label'].text = f"電流: {current_text}"
```

### 2. 夾爪控制操作區域
```python
# 夾爪控制面板UI範例
class GripperControlPanel:
    def __init__(self, gripper_controller):
        self.gripper = gripper_controller
        self.control_widgets = {}
        
        # 為每個夾爪創建控制區域
        for gripper_type in GripperType:
            self.create_gripper_control_widget(gripper_type)
    
    def create_gripper_control_widget(self, gripper_type: GripperType):
        """創建單個夾爪控制區域"""
        name = self.gripper.get_gripper_type_name(gripper_type)
        config = self.gripper.get_gripper_config(gripper_type)
        
        widget = {
            'name_label': create_label(name),
            'initialize_button': create_button("初始化", 
                lambda: self.on_initialize_clicked(gripper_type)),
            'stop_button': create_button("停止", 
                lambda: self.on_stop_clicked(gripper_type)),
            'open_button': create_button("開啟", 
                lambda: self.on_open_clicked(gripper_type)),
            'close_button': create_button("關閉", 
                lambda: self.on_close_clicked(gripper_type)),
            
            # 位置控制
            'position_slider': create_slider(
                min_value=config.position_range[0],
                max_value=config.position_range[1],
                callback=lambda pos: self.on_position_changed(gripper_type, pos)
            ),
            'position_input': create_number_input(
                min_value=config.position_range[0],
                max_value=config.position_range[1]
            ),
            'move_button': create_button("移動", 
                lambda: self.on_move_clicked(gripper_type)),
            
            # 力道控制
            'force_slider': create_slider(
                min_value=config.force_range[0],
                max_value=config.force_range[1],
                callback=lambda force: self.on_force_changed(gripper_type, force)
            ),
            
            # 速度控制
            'speed_slider': create_slider(
                min_value=config.speed_range[0],
                max_value=config.speed_range[1],
                callback=lambda speed: self.on_speed_changed(gripper_type, speed)
            ),
        }
        
        # PGHL特有的相對移動控制
        if config.supports_relative_move:
            widget['relative_input'] = create_number_input(
                min_value=-32767, max_value=32767
            )
            widget['relative_button'] = create_button("相對移動", 
                lambda: self.on_relative_move_clicked(gripper_type))
        
        self.control_widgets[gripper_type] = widget
        
        # 初始化控制狀態
        self.update_control_state(gripper_type)
    
    def update_control_state(self, gripper_type: GripperType):
        """更新控制狀態"""
        is_ready = self.gripper.is_gripper_ready(gripper_type)
        widget = self.control_widgets[gripper_type]
        
        # 啟用/停用控制按鈕
        widget['move_button'].enabled = is_ready
        widget['open_button'].enabled = is_ready
        widget['close_button'].enabled = is_ready
        widget['position_slider'].enabled = is_ready
        widget['force_slider'].enabled = is_ready
        widget['speed_slider'].enabled = is_ready
        
        if 'relative_button' in widget:
            widget['relative_button'].enabled = is_ready
    
    def on_initialize_clicked(self, gripper_type: GripperType):
        self.gripper.initialize_gripper(gripper_type)
    
    def on_stop_clicked(self, gripper_type: GripperType):
        self.gripper.stop_gripper(gripper_type)
    
    def on_open_clicked(self, gripper_type: GripperType):
        self.gripper.quick_open(gripper_type)
    
    def on_close_clicked(self, gripper_type: GripperType):
        self.gripper.quick_close(gripper_type)
    
    def on_move_clicked(self, gripper_type: GripperType):
        widget = self.control_widgets[gripper_type]
        position = widget['position_input'].value
        self.gripper.move_to_position(gripper_type, position)
    
    def on_relative_move_clicked(self, gripper_type: GripperType):
        widget = self.control_widgets[gripper_type]
        distance = widget['relative_input'].value
        self.gripper.move_relative(gripper_type, distance)
    
    def on_position_changed(self, gripper_type: GripperType, position: int):
        widget = self.control_widgets[gripper_type]
        widget['position_input'].value = position
    
    def on_force_changed(self, gripper_type: GripperType, force: int):
        self.gripper.set_force(gripper_type, force)
    
    def on_speed_changed(self, gripper_type: GripperType, speed: int):
        self.gripper.set_speed(gripper_type, speed)
```

### 3. 批量操作區域
```python
# 批量操作面板UI範例
class BatchOperationPanel:
    def __init__(self, gripper_controller):
        self.gripper = gripper_controller
        self.create_batch_controls()
    
    def create_batch_controls(self):
        """創建批量操作控制"""
        self.widgets = {
            'init_all_button': create_button("初始化所有夾爪", self.on_init_all_clicked),
            'stop_all_button': create_button("停止所有夾爪", self.on_stop_all_clicked),
            'open_all_button': create_button("開啟所有夾爪", self.on_open_all_clicked),
            'close_all_button': create_button("關閉所有夾爪", self.on_close_all_clicked),
            'emergency_button': create_button("緊急停止", self.on_emergency_clicked),
            
            # 在線夾爪顯示
            'online_grippers_label': create_label("在線夾爪: 無"),
            'ready_grippers_label': create_label("準備好夾爪: 無"),
            
            # 批量設定
            'batch_force_slider': create_slider(20, 100, self.on_batch_force_changed),
            'batch_speed_slider': create_slider(1, 100, self.on_batch_speed_changed),
            'apply_force_button': create_button("應用力道到所有夾爪", self.on_apply_force_clicked),
            'apply_speed_button': create_button("應用速度到所有夾爪", self.on_apply_speed_clicked),
        }
        
        # 定期更新在線夾爪顯示
        self.update_gripper_lists()
    
    def update_gripper_lists(self):
        """更新夾爪列表顯示"""
        online_grippers = self.gripper.get_online_grippers()
        ready_grippers = self.gripper.get_ready_grippers()
        
        online_names = [self.gripper.get_gripper_type_name(gt) for gt in online_grippers]
        ready_names = [self.gripper.get_gripper_type_name(gt) for gt in ready_grippers]
        
        self.widgets['online_grippers_label'].text = f"在線夾爪: {', '.join(online_names) if online_names else '無'}"
        self.widgets['ready_grippers_label'].text = f"準備好夾爪: {', '.join(ready_names) if ready_names else '無'}"
    
    def on_init_all_clicked(self):
        results = self.gripper.initialize_all_grippers()
        self.show_batch_result("初始化", results)
    
    def on_stop_all_clicked(self):
        results = self.gripper.stop_all_grippers()
        self.show_batch_result("停止", results)
    
    def on_open_all_clicked(self):
        results = self.gripper.open_all_grippers()
        self.show_batch_result("開啟", results)
    
    def on_close_all_clicked(self):
        results = self.gripper.close_all_grippers()
        self.show_batch_result("關閉", results)
    
    def on_emergency_clicked(self):
        emergency_stop_all()
    
    def on_batch_force_changed(self, force: int):
        self.current_batch_force = force
    
    def on_batch_speed_changed(self, speed: int):
        self.current_batch_speed = speed
    
    def on_apply_force_clicked(self):
        results = {}
        for gripper_type in self.gripper.get_ready_grippers():
            results[gripper_type] = self.gripper.set_force(gripper_type, self.current_batch_force)
        self.show_batch_result("力道設定", results)
    
    def on_apply_speed_clicked(self):
        results = {}
        for gripper_type in self.gripper.get_ready_grippers():
            results[gripper_type] = self.gripper.set_speed(gripper_type, self.current_batch_speed)
        self.show_batch_result("速度設定", results)
    
    def show_batch_result(self, operation: str, results: Dict[GripperType, bool]):
        """顯示批量操作結果"""
        success_count = sum(1 for success in results.values() if success)
        total_count = len(results)
        
        message = f"{operation}結果: {success_count}/{total_count} 成功"
        
        # 顯示詳細結果
        for gripper_type, success in results.items():
            name = self.gripper.get_gripper_type_name(gripper_type)
            status = "成功" if success else "失敗"
            message += f"\n{name}: {status}"
        
        show_message(message)
```

### 4. 夾爪配置顯示區域
```python
# 夾爪配置面板UI範例
class GripperConfigPanel:
    def __init__(self, gripper_controller):
        self.gripper = gripper_controller
        self.create_config_display()
    
    def create_config_display(self):
        """創建配置顯示"""
        configs = self.gripper.get_all_gripper_configs()
        
        for gripper_type, config in configs.items():
            name = self.gripper.get_gripper_type_name(gripper_type)
            
            # 創建配置顯示表格
            config_table = create_table([
                ["位置範圍", f"{config.position_range[0]} - {config.position_range[1]}"],
                ["力道範圍", f"{config.force_range[0]} - {config.force_range[1]}"],
                ["速度範圍", f"{config.speed_range[0]} - {config.speed_range[1]}"],
                ["位置精度", f"{config.position_precision}"],
                ["電流反饋", "是" if config.has_current_feedback else "否"],
                ["相對移動", "是" if config.supports_relative_move else "否"],
                ["Unit ID", str(config.unit_id)],
                ["寄存器範圍", self.get_register_range(gripper_type)]
            ])
            
            self.add_config_section(name, config_table)
    
    def get_register_range(self, gripper_type: GripperType) -> str:
        """獲取寄存器範圍描述"""
        reg_info = self.gripper.GRIPPER_REGISTERS[gripper_type]
        status_range = f"{reg_info['status_base']}-{reg_info['status_base']+19}"
        command_range = f"{reg_info['command_base']}-{reg_info['command_base']+9}"
        return f"狀態:{status_range}, 指令:{command_range}"
```

## 集成注意事項

### 1. 線程安全
- GripperGUIClass內部使用線程鎖保護共享狀態
- 回調函數在監控線程中執行，GUI更新需要切換到主線程
- 建議使用訊號-槽機制或事件佇列進行跨線程通訊

### 2. 夾爪差異處理
```python
# 夾爪差異適配範例
# 夾爪差異適配範例
def adaptive_gripper_operation(gripper_type: GripperType, operation: str, **kwargs):
   """自適應夾爪操作"""
   config = gripper.get_gripper_config(gripper_type)
   
   if operation == "move_to_center":
       # 移動到中心位置，根據夾爪型號自動計算
       center_pos = (config.position_range[0] + config.position_range[1]) // 2
       return gripper.move_to_position(gripper_type, center_pos)
   
   elif operation == "set_moderate_force":
       # 設定中等力道
       moderate_force = (config.force_range[0] + config.force_range[1]) // 2
       return gripper.set_force(gripper_type, moderate_force)
   
   elif operation == "relative_move" and config.supports_relative_move:
       # 僅PGHL支援相對移動
       distance = kwargs.get('distance', 0)
       return gripper.move_relative(gripper_type, distance)
   
   else:
       print(f"{gripper.get_gripper_type_name(gripper_type)}不支援操作: {operation}")
       return False
3. 資源管理
python# 應用程式退出時的清理
def on_application_exit():
    """應用程式退出清理"""
    if gripper:
        # 停止所有夾爪
        gripper.stop_all_grippers()
        # 斷開連接
        gripper.disconnect()
4. 異常處理

所有Modbus操作都包含異常處理
網路斷線時會自動停止監控
建議實現重連機制

5. 效能考慮

狀態監控頻率為200ms，適合即時顯示
避免在回調函數中執行耗時操作
批量操作時考慮序列埠通訊負載

寄存器映射參考
夾爪寄存器分配
PGC夾爪  (unit_id=6): 500-529 (狀態500-519, 指令520-529)
PGHL夾爪 (unit_id=5): 530-559 (狀態530-549, 指令550-559)
PGE夾爪  (unit_id=4): 560-579 (狀態560-579, 指令580-589)
狀態寄存器映射 (每個夾爪20個寄存器)

+0: 模組狀態 (0=離線, 1=在線)
+1: 連接狀態 (0=斷開, 1=已連接)
+2: 設備狀態 (初始化狀態)
+3: 錯誤計數
+4: 夾持狀態 (運動狀態)
+5: 當前位置
+6: 電流值 (僅PGHL)
+14: 時間戳

指令寄存器映射 (每個夾爪10個寄存器)

+0: 指令代碼
+1: 參數1
+2: 參數2
+3: 指令ID

三款夾爪特性對比
特性PGCPGHLPGEUnit ID654位置範圍0-10000-655350-1000位置精度1.00.01mm1.0力道範圍20-10020-10020-100速度範圍1-10050-1001-100電流反饋無有無相對移動無有無寄存器基地址500530560
使用範例
完整的GUI整合範例:
pythonfrom GripperGUIClass import GripperGUIClass, GripperType, GripperStatus

class UnifiedMachineGUI:
    def __init__(self):
        # 初始化Gripper控制器
        self.gripper = GripperGUIClass()
        
        # 註冊回調
        self.gripper.add_status_callback(self.on_gripper_status_changed)
        
        # 建立GUI界面
        self.setup_gui()
        
        # 連接Gripper
        self.connect_gripper()
        
        # 初始化所有夾爪
        self.initialize_all_grippers()
    
    def setup_gui(self):
        """建立GUI界面"""
        # 創建夾爪狀態面板
        self.status_panel = GripperStatusPanel(self.gripper)
        
        # 創建夾爪控制面板
        self.control_panel = GripperControlPanel(self.gripper)
        
        # 創建批量操作面板
        self.batch_panel = BatchOperationPanel(self.gripper)
        
        # 創建配置顯示面板
        self.config_panel = GripperConfigPanel(self.gripper)
    
    def connect_gripper(self):
        """連接Gripper模組"""
        if self.gripper.connect():
            self.update_status("Gripper連接成功")
        else:
            self.update_status("Gripper連接失敗")
    
    def initialize_all_grippers(self):
        """初始化所有夾爪"""
        results = self.gripper.initialize_all_grippers()
        success_count = sum(1 for success in results.values() if success)
        self.update_status(f"夾爪初始化: {success_count}/{len(results)} 成功")
    
    def on_gripper_status_changed(self, statuses: Dict[GripperType, GripperStatus]):
        """Gripper狀態變化處理"""
        # 更新GUI狀態顯示
        self.update_gripper_status_display(statuses)
        
        # 更新控制按鈕狀態
        self.update_control_button_states()
        
        # 更新批量操作顯示
        self.batch_panel.update_gripper_lists()
    
    def update_gripper_status_display(self, statuses: Dict[GripperType, GripperStatus]):
        """更新夾爪狀態顯示"""
        for gripper_type, status in statuses.items():
            # 更新狀態面板
            self.status_panel.on_status_updated(statuses)
    
    def update_control_button_states(self):
        """更新控制按鈕狀態"""
        for gripper_type in GripperType:
            self.control_panel.update_control_state(gripper_type)
開發注意事項
1. PyModbus版本兼容性

確保使用PyModbus 3.x版本
使用關鍵字參數調用API
正確處理response.isError()檢查

2. 夾爪通訊特性

所有夾爪共用COM5端口，依靠unit_id區分
RTU通訊有序列埠競爭，需要適當的輪詢間隔
錯誤計數累積，需要定期清零或重置

3. 位置精度處理
pythondef convert_pghl_position(position_raw: int) -> float:
    """PGHL位置轉換 (0.01mm精度)"""
    return position_raw * 0.01

def convert_to_pghl_position(position_mm: float) -> int:
    """轉換為PGHL原始位置值"""
    return int(position_mm * 100)
4. 指令ID管理

使用遞增的指令ID防止重複執行
ID範圍1-65535，超出後重置為1
每個夾爪獨立的指令ID追蹤

故障排除
常見問題

COM5權限問題: 確保串口可用且無其他程序占用
夾爪無回應: 檢查unit_id設定和電源連接
TCP連接失敗: 確認主服務器運行狀態
指令重複執行: 檢查指令ID機制

除錯方法

檢查夾爪在線狀態
監控錯誤計數器
驗證寄存器讀寫
測試單獨夾爪通訊

效能優化

調整狀態監控頻率
優化批量操作順序
監控RTU通訊負載
合理設置超時時間

這個設計指南提供了完整的GripperGUIClass使用框架，支援統一機台調適工具的開發需求，特別針對三款夾爪的差異性和統一性進行了優化設計。