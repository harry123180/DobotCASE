# Dobot_main機械手臂併行架構文件

## 系統概述

Dobot_main採用三執行緒併行架構，支援運動控制、DIO控制、外部模組交握的同時執行。使用統一指令佇列和優先權機制，確保指令按序執行且運動指令具有優先權。

## 核心架構

### 1. 三執行緒架構

```
┌─────────────────────────────────────────────────────────┐
│                 Dobot_main主控制器                        │
├─────────────────┬─────────────────┬─────────────────────┤
│  運動控制執行緒    │   DIO控制執行緒   │  外部模組交握執行緒   │
│ MotionFlowThread │  DIOFlowThread  │ ExternalModuleThread│
├─────────────────┼─────────────────┼─────────────────────┤
│ • Flow1Executor │ • DO設定         │ • CCD1交握          │
│ • Flow2Executor │ • DI讀取         │ • VP交握            │
│ • MovJ/MovL     │ • 脈衝輸出       │ • 夾爪交握          │
│ • 速度控制      │ • 序列輸出       │ • CCD3交握          │
└─────────────────┴─────────────────┴─────────────────────┘
                           ↓
              ┌─────────────────────────────┐
              │        統一指令佇列          │
              │    (優先權 + 先來後到)       │
              └─────────────────────────────┘
                           ↓
              ┌─────────────────────────────┐
              │    機械臂API (共用端口)      │
              │ DobotApiDashboard/Move      │
              └─────────────────────────────┘
```

### 2. 寄存器映射擴展 (400-499)

| 地址範圍 | 功能 | 說明 |
|---------|------|------|
| 400-439 | 原有狀態寄存器 | 位置、角度、IO狀態等 |
| 440-449 | 控制寄存器 | VP、出料、手動指令等 |
| 450-459 | 併行執行緒狀態 | 三執行緒運行狀態監控 |
| 460-469 | DIO專用寄存器 | DIO控制參數和狀態 |
| 470-479 | 外部模組交握 | 各模組交握狀態管控 |
| 480-499 | 預留擴展 | 未來功能擴展使用 |

### 3. 指令優先權機制

```python
# 優先權定義 (數值越小優先權越高)
EMERGENCY = 0      # 緊急停止
MOTION = 1         # 運動指令 (自動-0.1提升優先權)
DIO = 2           # DIO指令  
EXTERNAL = 3      # 外部模組指令

# 同優先權按時間戳排序 (先來後到)
```

## 執行緒詳細設計

### 1. 運動控制執行緒 (MotionFlowThread)

**職責**: 
- 執行所有機械臂運動相關指令
- 管理Flow1、Flow2等流程執行器
- 處理MovJ、MovL、速度設定等基本運動指令

**指令類型**:
```python
# Flow指令
{'type': 'flow1'}                    # 執行Flow1
{'type': 'flow2'}                    # 執行Flow2
{'type': 'flow3'}                    # 執行Flow3 (未來擴展)

# 基本運動指令
{'type': 'move_j', 'x': 300, 'y': 0, 'z': 200, 'r': 0}
{'type': 'move_l', 'x': 300, 'y': 0, 'z': 200, 'r': 0}
{'type': 'set_speed', 'speed': 50}

# 點位運動
{'type': 'move_to_point', 'point_name': 'standby'}
```

**狀態管理**:
- 450寄存器: 執行緒狀態 (0=停止, 1=運行, 2=執行中, 3=錯誤)
- 追蹤當前執行的Flow和步驟
- 維護操作計數和錯誤統計

### 2. DIO控制執行緒 (DIOFlowThread)

**職責**:
- 處理所有數位輸入輸出操作
- 通過dashboard_api.DO()和DI()控制機械臂DIO
- 支援脈衝輸出、序列輸出等複雜DIO操作

**指令類型**:
```python
# 基本DIO操作
{'type': 'set_do', 'pin': 1, 'value': 1}     # 設定數位輸出
{'type': 'get_di', 'pin': 1}                 # 讀取數位輸入

# 批量DIO操作
{'type': 'set_do_group', 'mask': 0x0F, 'value': 0x05}

# 脈衝輸出
{'type': 'pulse_do', 'pin': 1, 'pulse_width': 100}  # 100ms脈衝

# 序列輸出
{'type': 'sequence_do', 'sequence': [
    {'pin': 1, 'value': 1, 'delay': 100},
    {'pin': 2, 'value': 1, 'delay': 200},
    {'pin': 1, 'value': 0, 'delay': 100}
]}
```

**DIO專用寄存器 (460-469)**:
- 460: DIO輸出遮罩
- 461: DIO輸出值
- 462: DI輸入濾波設定
- 463: 脈衝寬度設定
- 464: 延遲時間設定
- 465: 序列步驟計數
- 466: DIO錯誤計數
- 467: DIO操作計數

### 3. 外部模組交握執行緒 (ExternalModuleThread)

**職責**:
- 管理與所有外部模組的交握通訊
- 實現標準Ready/Running/Alarm狀態機協議
- 支援動態擴展新的外部模組

**支援模組**:
- CCD1視覺檢測 (基地址200)
- VP震動盤 (基地址300)  
- 夾爪控制 (基地址500)
- CCD3角度檢測 (基地址800)
- 未來擴展模組...

**指令格式**:
```python
# CCD1指令
{'module': 'CCD1', 'operation': 'detect', 'params': {...}}
{'module': 'CCD1', 'operation': 'capture'}

# VP指令  
{'module': 'VP', 'operation': 'vibrate', 'action': 1, 'intensity': 50}
{'module': 'VP', 'operation': 'light', 'brightness': 100}
{'module': 'VP', 'operation': 'stop'}

# 夾爪指令
{'module': 'GRIPPER', 'operation': 'open'}
{'module': 'GRIPPER', 'operation': 'close'}
{'module': 'GRIPPER', 'operation': 'position', 'position': 1000}

# CCD3指令
{'module': 'CCD3', 'operation': 'angle_detect'}
```

**模組配置結構**:
```python
MODULE_CONFIG = {
    'CCD1': {
        'base_address': 200,
        'control_register': 0,    # 200
        'status_register': 1,     # 201
        'timeout': 10.0
    },
    'VP': {
        'base_address': 300,
        'control_register': 20,   # 320
        'status_register': 0,     # 300
        'timeout': 5.0
    },
    # ... 其他模組
}
```

## 狀態機交握協議

### 標準交握流程

1. **檢查Ready狀態**: 讀取狀態寄存器bit0=1
2. **發送控制指令**: 寫入控制寄存器
3. **等待執行完成**: 監控狀態寄存器Running=0
4. **讀取結果**: 讀取相關結果寄存器
5. **清零指令**: 寫入控制寄存器=0
6. **恢復Ready**: 確認Ready=1

### 交握狀態寄存器 (470-479)

- 470: CCD1交握狀態 (0=空閒, 1=交握中, 2=完成, 3=錯誤)
- 471: VP交握狀態
- 472: 夾爪交握狀態  
- 473: CCD3交握狀態
- 474: 外部模組錯誤計數
- 475: 交握超時設定 (秒)
- 476: 模組就緒狀態位 (bit0=CCD1, bit1=VP, bit2=夾爪, bit3=CCD3)
- 477: 最後交握錯誤代碼

## 主要類別設計

### 1. DobotConcurrentController (主控制器)

```python
class DobotConcurrentController:
    def __init__(self, config_file):
        self.command_queue = CommandQueue(max_size=100)
        self.motion_thread = None
        self.dio_thread = None  
        self.external_thread = None
        
    def initialize(self) -> bool:
        # 初始化Modbus、機械臂、執行緒
        
    def start(self) -> bool:
        # 啟動所有執行緒
        
    def _handshake_loop(self):
        # 主要狀態機交握循環
        # 監控控制寄存器，分派指令到佇列
```

### 2. CommandQueue (指令佇列)

```python
class CommandQueue:
    def __init__(self, max_size=100):
        self.queue = queue.PriorityQueue(max_size)
        
    def put_command(self, command: Command) -> bool:
        # 加入指令，自動處理優先權
        
    def get_command(self, timeout=None) -> Optional[Command]:
        # 取得最高優先權指令
```

### 3. Command (統一指令格式)

```python
@dataclass  
class Command:
    command_type: CommandType      # MOTION, DIO, EXTERNAL
    command_data: Dict[str, Any]   # 指令參數
    priority: CommandPriority      # 優先權等級
    timestamp: float               # 時間戳
    command_id: int               # 指令ID
    callback: Optional[callable]   # 回調函數
```

## 指令分派機制

### 寄存器監控 → 指令分派

```python
def _handshake_loop(self):
    while self.running:
        # 讀取控制寄存器
        vp_control = self.read_register(40)      # 440
        unload_control = self.read_register(41)   # 441  
        dio_command = self.read_register(46)      # 446
        external_command = self.read_register(47) # 447
        
        # 分派到相應佇列
        if vp_control == 1:
            self._dispatch_motion_command('flow1')
        if unload_control == 1:
            self._dispatch_motion_command('flow2')
        if dio_command != 0:
            self._dispatch_dio_command(dio_command)
        if external_command != 0:
            self._dispatch_external_command(external_command)
```

### 指令編碼規範

| 寄存器 | 指令碼 | 功能 | 目標執行緒 |
|-------|--------|------|-----------|
| 440 | 1 | Flow1執行 | 運動控制 |
| 441 | 1 | Flow2執行 | 運動控制 |
| 444 | 101-199 | 手動運動指令 | 運動控制 |
| 446 | 1-99 | DIO控制指令 | DIO控制 |
| 447 | 1-99 | 外部模組指令 | 外部模組 |

## 錯誤處理機制

### 1. 執行緒級錯誤處理

- 每個執行緒維護自己的錯誤狀態
- 錯誤時自動重試或降級處理
- 嚴重錯誤時停止執行緒並報警

### 2. 系統級錯誤處理

- 緊急停止: 清空佇列，停止所有運動
- 通訊錯誤: 自動重連機制
- 超時錯誤: 指令超時自動丟棄

### 3. 錯誤狀態寄存器

- 404: 系統錯誤代碼
- 456: 執行緒錯誤代碼  
- 466: DIO錯誤計數
- 474: 外部模組錯誤計數
- 477: 交握錯誤代碼

## 性能監控

### 統計寄存器

- 416: 操作計數器 (總)
- 453: 指令佇列大小
- 454: 活躍指令數量  
- 467: DIO操作計數
- 各執行緒維護獨立的操作統計

### 實時狀態監控

```python
def get_system_status(self):
    return {
        'command_queue_size': self.command_queue.size(),
        'motion_thread_status': self.motion_thread.status,
        'dio_thread_status': self.dio_thread.status, 
        'external_thread_status': self.external_thread.status,
        'total_commands': self.total_commands,
        'error_counts': {...}
    }
```

## 配置檔案結構

```json
{
    "robot": {
        "ip": "192.168.1.6",
        "dashboard_port": 29999,
        "move_port": 30003
    },
    "modbus": {
        "server_ip": "127.0.0.1", 
        "server_port": 502,
        "base_address": 400
    },
    "command_queue": {
        "max_size": 100,
        "timeout": 10.0
    },
    "threading": {
        "handshake_interval": 0.05,
        "status_update_interval": 1.0
    },
    "external_modules": {
        "CCD1": {"base_address": 200, "timeout": 10.0},
        "VP": {"base_address": 300, "timeout": 5.0},
        "GRIPPER": {"base_address": 500, "timeout": 3.0},
        "CCD3": {"base_address": 800, "timeout": 15.0}
    }
}
```

## 部署和使用

### 啟動順序

1. 啟動主Modbus TCP Server (端口502)
2. 啟動各外部模組程序
3. 啟動Dobot_main主程序
4. 檢查所有執行緒狀態
5. 開始接收PLC指令

### 監控和診斷

- 透過寄存器即時監控系統狀態
- 支援Web介面監控 (可選)
- 完整的日誌記錄機制
- 性能指標統計和報告

### 擴展性設計

- 新增外部模組只需配置基地址和超時
- 新增Flow只需實現FlowExecutor介面
- 新增DIO操作只需擴展指令類型
- 模組化設計便於維護和升級