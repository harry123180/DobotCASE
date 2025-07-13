# AutoFeeding獨立模組邏輯流程 - 完整版

## 核心檢測循環 (0.8秒週期)

```
啟動 → 模組檢查 → CCD1檢測 → 結果判斷 → 執行動作 → 交握處理 → 等待週期
  ↑                                                              ↓
  ←──────────────────── 錯誤處理/暫停控制 ←────────────────────────┘
```

## 詳細流程步驟

### 1. 前置檢查
```
檢查模組狀態:
├── CCD1模組: 201寄存器 → Ready=1 & Initialized=1 & Alarm=0
├── VP模組: 300=1 (閒置), 301=1 (已連接)
└── 暫停標誌: pause_for_robot=false (921寄存器控制)
```

### 2. CCD1檢測協議
```
觸發檢測:
1. 寫入200=16 (拍照+檢測指令)
2. 高頻輪詢等待: 203=1 & 204=1 & 205=1 (完成標誌，20ms間隔檢查)
3. 讀取結果: 240(CASE_F數量), 243(總檢測數量)
4. 提取座標: 261-264(第一個CASE_F世界座標)
5. 清空寄存器: 200,203,204,205=0
```

### 3. 保護區域判斷
```
硬編碼四點座標:
(10.71, -246.12), (10.71, -374.21), 
(-77.88, -374.22), (-111.25, -246.13)

射線法算法:
def is_point_in_quad(x, y) -> bool:
    # 中心點排序 + 射線交點計算
    return inside_polygon
```

### 4. 動作決策邏輯
```
if 找到保護區內CASE_F:
    ├── 複製座標到941-944寄存器 (32位世界座標)
    ├── 寫入946,947 (檢測總數,CASE_F數量)
    ├── 設置940=1 (入料完成標誌)
    ├── 等待945=1 (AutoProgram確認讀取，超時2秒)
    ├── 收到確認後清空940-947=0
    └── 進入暫停狀態等待AutoProgram處理

elif 總檢測數量 < 4 (料件不足):
    ├── 觸發Flow4送料: 448=1 → 延遲0.1秒 → 448=0
    ├── 累計連續直振次數
    └── 連續5次 → 進入VP清空流程

elif 總檢測數量 >= 4 (料件充足但無正面):
    ├── VP震動: 320=5,321=11,322=60,323=50
    ├── 等待0.3秒震動
    ├── 停止震動: 320=3,324=99
    ├── 等待0.15秒穩定
    └── 重新CCD1檢測
```

### 5. VP清空流程 (連續直振5次觸發)
```
清空模式:
1. VP震動翻正所有料件 (spread強度60頻率50)
2. 重複VP震動最多6次:
   ├── VP震動0.3秒
   ├── 等待穩定0.5秒
   ├── CCD1檢測是否為空
   └── 連續3次空檢測 → 停止AutoFeeding
3. 注意: 不操作機械臂相關寄存器(1200+)
```

## 🔄 AutoFeeding ↔ AutoProgram 交握協議

### 📤 AutoFeeding → AutoProgram (入料完成交握)
```
1. AutoFeeding找到保護區內CASE_F
2. 寫入座標到941-944寄存器:
   - 941: 世界座標X高16位
   - 942: 世界座標X低16位  
   - 943: 世界座標Y高16位
   - 944: 世界座標Y低16位
3. 寫入檢測統計946,947寄存器
4. 設置940=1 (入料完成標誌)
5. 等待AutoProgram確認945=1 (超時2秒)
6. 收到確認後清空940-947=0
7. 進入暫停狀態等待921控制

重要: 940寄存器不會自動清零，必須透過AutoProgram確認(945=1)或超時才清零
```

### 📥 AutoProgram → AutoFeeding (暫停控制交握)
```
1. AutoProgram寫入921=1 (暫停入料檢測)
2. AutoFeeding檢測到暫停信號 → 停止檢測週期
3. AutoProgram執行Flow1機械臂作業
4. Flow1完成後AutoProgram寫入921=0 (恢復入料檢測)
5. AutoFeeding恢復檢測週期

暫停機制用途: 防止Flow1機械臂執行時入料檢測干擾
```

### 🎯 AutoProgram 啟動控制
```
1. AutoProgram寫入920=1 → 啟動AutoFeeding檢測循環
2. AutoProgram寫入920=0 → 停止AutoFeeding檢測循環

啟動時機: AutoProgram系統就緒且機械臂Ready時啟動AutoFeeding
```

## 📋 AutoProgram 開發參考

### 核心狀態監控寄存器
| 寄存器 | 功能 | AutoProgram用途 |
|--------|------|-----------------|
| 900 | AutoFeeding狀態 | 監控AF運行狀態(0=停止,1=運行,2=暫停) |
| 940 | 入料完成標誌 | 輪詢檢查是否有新的正面料件 |
| 941-944 | 料件世界座標 | 讀取CASE_F目標座標給機械臂 |
| 946-947 | 檢測統計 | 了解當前VP上料件情況 |

### AutoProgram 控制寄存器
| 寄存器 | 功能 | AutoProgram操作 |
|--------|------|-----------------|
| 920 | 啟動/停止AF | 1=啟動AF, 0=停止AF |
| 921 | 暫停/恢復AF | 1=暫停AF(Flow1前), 0=恢復AF(Flow1後) |
| 945 | 確認讀取 | 讀取940後寫入1確認 |

### AutoProgram 協調邏輯建議
```python
class AutoProgramController:
    def __init__(self):
        self.prepare_done = False  # 是否已完成首次取料
        
    def main_loop(self):
        while True:
            # 檢查機械臂Ready狀態
            robot_ready = self.check_robot_ready()
            
            if robot_ready and not self.is_autofeeding_running():
                # 啟動AutoFeeding
                self.start_autofeeding()
            
            # 監控入料完成
            if self.check_feeding_complete():
                target_coords = self.read_target_coordinates()
                
                if not self.prepare_done:
                    # 首次入料完成，執行Flow1
                    self.pause_autofeeding()  # 921=1
                    success = self.execute_flow1(target_coords)
                    if success:
                        self.prepare_done = True
                    self.resume_autofeeding()  # 921=0
                else:
                    # 已準備好，等待外部Flow5觸發
                    pass
            
            # 監控Flow5完成
            if self.check_flow5_complete():
                self.prepare_done = False  # 重置狀態
                self.clear_flow5_status()
    
    def start_autofeeding(self):
        """啟動AutoFeeding"""
        self.write_register(920, 1)
    
    def pause_autofeeding(self):
        """暫停AutoFeeding(Flow1執行前)"""
        self.write_register(921, 1)
    
    def resume_autofeeding(self):
        """恢復AutoFeeding(Flow1執行後)"""
        self.write_register(921, 0)
    
    def check_feeding_complete(self) -> bool:
        """檢查入料完成"""
        return self.read_register(940) == 1
    
    def read_target_coordinates(self) -> tuple:
        """讀取目標座標並確認"""
        x = self.read_32bit_register(941, 942)
        y = self.read_32bit_register(943, 944)
        
        # 確認讀取
        self.write_register(945, 1)
        
        return (x, y)
    
    def execute_flow1(self, coords) -> bool:
        """執行Flow1取料"""
        # 觸發機械臂Flow1
        # 等待完成
        # 返回成功/失敗
        pass
```

### 時序協調圖
```
AutoProgram                 AutoFeeding              機械臂
    |                           |                      |
    |--- 920=1 啟動AF ---------->|                      |
    |                           |                      |
    |<------ 940=1 入料完成 -----|                      |
    |                           |                      |
    |--- 945=1 確認讀取 -------->|                      |
    |--- 921=1 暫停AF ---------->|                      |
    |                           |                      |
    |--- Flow1 執行 --------------------------->|      |
    |                           |              |      |
    |<------ Flow1完成 <------------------------|      |
    |                           |                      |
    |--- 921=0 恢復AF ---------->|                      |
    |                           |                      |
```

## 關鍵參數配置

### 時序參數 (已優化)
| 參數 | 預設值 | 說明 |
|------|--------|------|
| 檢測週期間隔 | 800ms | 主循環週期 (從2秒優化到0.8秒) |
| CCD1檢測超時 | 5000ms | 檢測等待超時 (從10秒優化到5秒) |
| VP震動持續 | 300ms | 震動時間 (從500ms優化到300ms) |
| VP停止延遲 | 100ms | 震動停止等待 (從200ms優化到100ms) |
| Flow4脈衝 | 100ms | 送料脈衝時間 |
| 交握確認超時 | 2000ms | 等待AutoProgram確認時間 |

### VP震動參數 (已優化)
| 參數 | 數值 | 說明 |
|------|------|------|
| 動作碼 | 11 | spread散開動作 |
| 強度 | 60 | 震動強度(從50提升到60) |
| 頻率 | 50Hz | 震動頻率(從43Hz提升到50Hz) |
| 停止指令 | 3 | stop_all停止 |

### Flow4參數配置
| 參數 | 數值 | 說明 |
|------|------|------|
| 脈衝持續時間 | 100ms | 448寄存器保持1的時間 |
| 脈衝間隔時間 | 50ms | 預留連續脈衝使用 |

### 保護機制
| 機制 | 觸發條件 | 處理方式 |
|------|----------|----------|
| 連續直振限制 | Flow4連續5次 | 進入VP清空流程 |
| VP空檢測限制 | 連續3次檢測為空 | 停止AutoFeeding |
| CCD1檢測超時 | 5秒無回應 | 跳過本週期 |
| 交握確認超時 | 2秒無AutoProgram確認 | 強制清空940=0 |
| 模組狀態異常 | Ready=0或連接斷開 | 跳過本週期 |

## 統計資訊追蹤

### 寄存器統計 (901-909)
- 901: 檢測週期計數
- 902: CASE_F找到次數  
- 903: Flow4觸發次數
- 904: VP震動次數
- 905: 連續直振次數
- 906: VP空檢測次數
- 907: 錯誤代碼
- 908: 當前操作狀態
- 909: VP清空模式標誌

### 效能指標
- CASE_F找到率 = 902 / 901 × 100%
- 送料效率 = 903 / 901 × 100% 
- 震動使用率 = 904 / 901 × 100%

## 異常處理機制

### 模組檢查失敗
```
CCD1異常:
├── Alarm=1 → 等待CCD1自行初始化完成
├── Ready=0 → 等待Ready狀態恢復
├── 連續無回應 → 記錄錯誤跳過週期
└── 檢測超時 → 記錄錯誤跳過週期

VP異常:
├── module_status≠1 → 記錄錯誤跳過週期
├── device_connection≠1 → 記錄錯誤跳過週期
└── 震動指令失敗 → 緊急停止震動
```

### 緊急停止機制
```
系統停止:
├── 924=1 → VP強制停止震動
├── 923=1 → 清除所有錯誤
├── 920=0 → 停止檢測循環
└── 緊急停止VP震動(320=3,324=99)
```

## 狀態機設計

### 主狀態 (900寄存器)
- 0: 停止
- 1: 運行中 (正常檢測循環)
- 2: 暫停 (等待Flow1完成)  
- 3: 檢測中 (CCD1檢測進行中)
- 4: VP震動中 (震動盤作業中)
- 5: 錯誤 (模組異常)

### 操作狀態 (908寄存器)  
- 0: 空閒
- 1: CCD檢測 (拍照+檢測)
- 2: VP控制 (震動/停止)
- 3: Flow4觸發 (送料脈衝)

## 實現優勢

### 獨立性
- 移除Threading，避免長時間運行問題
- 獨立進程，可單獨啟動/停止/重啟
- 模組異常不影響AutoProgram

### 可靠性  
- Modbus寄存器交握確保通訊穩定
- 完整的異常處理與恢復機制
- 狀態機設計確保邏輯清晰
- 超時保護防止死鎖

### 可維護性
- 清晰的責任分離
- 豐富的統計資訊便於調試
- 標準化寄存器映射便於整合
- 詳細的日誌輸出便於問題定位

### 擴展性
- 預留大量寄存器空間(960-999)
- 模組化設計易於功能擴展
- 標準化協議便於其他模組接入
- 可配置參數支援不同應用場景

## 🚀 速度優化總結

AutoFeeding已從原來的2秒週期優化到0.8秒週期，整體速度提升60%+：

- **CCD1檢測**: 20ms高頻輪詢，5倍響應速度
- **VP震動**: 300ms快速震動，強度60+頻率50Hz
- **Flow4送料**: 100ms敏捷脈衝
- **交握響應**: 2秒內完成確認
- **絲滑運行**: 無Threading穩定性問題

已實現**絲滑、不斷、快速、準確**的入料檢測效果。