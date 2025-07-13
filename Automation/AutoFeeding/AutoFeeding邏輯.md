# AutoFeeding獨立模組協調邏輯 - 修正版

## 🔄 核心設計改進

### ✅ **原有問題**
- AutoFeeding找到CASE_F後會卡住暫停，無法持續檢測
- Flow5執行時沒有自動恢復AutoFeeding運作
- 需要複雜的AutoProgram中介控制
- 交握協議複雜，容易出錯

### 🟢 **修正後優勢**
- **持續檢測**: 找到CASE_F後不暫停，繼續檢測確保保護區域隨時有料件
- **主動監控**: 直接監控Flow1控制地址(1240)，無需被動等待暫停指令
- **簡化交握**: Flow1直接讀取座標(940-944)，不需要透過AutoProgram中轉
- **自動運行**: 啟動後自動開始檢測，無需外部觸發

## 🎯 主動監控Flow1邏輯

### **Flow1狀態監控**
```python
def check_flow1_status(self) -> bool:
    """主動監控Flow1控制狀態"""
    flow1_control = self.read_register(1240)  # Dobot_main的Flow1控制寄存器
    flow1_now_active = (flow1_control == 1)
    
    if flow1_now_active != self.flow1_active:
        if flow1_now_active:
            print("🔴 檢測到Flow1啟動，暫停檢測")
            self.status = AutoFeedingStatus.FLOW1_PAUSED
        else:
            print("🟢 檢測到Flow1停止，恢復檢測")
            self.status = AutoFeedingStatus.RUNNING
```

### **監控時序**
```
AutoFeeding監控循環 (0.1秒間隔):
1. 檢查1240寄存器值
2. 1240=1 → 立即暫停檢測
3. 1240=0 → 立即恢復檢測
4. 全程無需外部控制
```

## 🔄 簡化交握協議

### **舊版複雜交握** (已廢棄)
```
AutoFeeding → AutoProgram → Flow1
1. AF設置940=1
2. 等待AP寫入945=1確認
3. AP暫停AF (921=1)
4. AP觸發Flow1
5. AP恢復AF (921=0)
```

### **新版直接交握** (現行版本)
```python
# AutoFeeding設置CASE_F可用
def set_case_f_available(self, coords: Tuple[float, float]):
    self.case_f_available = True
    self.case_f_coords = coords
    
    # 立即更新寄存器讓Flow1直接讀取
    self.write_register(940, 1)  # CASE_F可用標誌
    self.write_32bit_register(941, 942, coords[0])  # X座標
    self.write_32bit_register(943, 944, coords[1])  # Y座標

# Flow1直接讀取座標並確認
def check_coords_taken(self):
    coords_taken = self.read_register(945)  # Flow1設置此標誌
    if coords_taken == 1:
        # 清除CASE_F狀態，繼續檢測新的
        self.case_f_available = False
        self.write_register(940, 0)
        self.write_register(945, 0)
```

### **新版交握流程**
```
AutoFeeding ↔ Flow1 直接交握:
1. AF找到CASE_F → 設置940=1, 941-944=座標
2. Flow1檢查940=1 → 讀取941-944座標
3. Flow1讀取完成 → 設置945=1確認
4. AF檢測到945=1 → 清除狀態，繼續檢測
```

## 📋 寄存器映射 (簡化版)

### **CASE_F狀態寄存器 (940-959)**
| 地址 | 功能 | 讀寫方 | 說明 |
|------|------|--------|------|
| 940 | CASE_F可用標誌 | AF寫入, Flow1讀取 | 0=無CASE_F, 1=有CASE_F可取 |
| 941 | X座標高位 | AF寫入, Flow1讀取 | 32位世界座標X高16位 |
| 942 | X座標低位 | AF寫入, Flow1讀取 | 32位世界座標X低16位 |
| 943 | Y座標高位 | AF寫入, Flow1讀取 | 32位世界座標Y高16位 |
| 944 | Y座標低位 | AF寫入, Flow1讀取 | 32位世界座標Y低16位 |
| 945 | 座標已讀取標誌 | Flow1寫入, AF讀取 | Flow1確認已讀取座標 |

### **Flow1控制寄存器**
| 地址 | 功能 | 控制方 | 說明 |
|------|------|--------|------|
| 1240 | Flow1控制 | Dobot_main | AF主動監控此地址 |

## 🚀 使用方式

### **1. 啟動AutoFeeding**
```bash
# 進入AutoFeeding目錄
cd Automation/AutoFeeding/

# 啟動AutoFeeding獨立模組
python AutoFeeding_main.py
```

### **2. AutoFeeding自動行為**
```
啟動後自動執行:
✓ 連接Modbus服務器 (127.0.0.1:502)
✓ 初始化寄存器
✓ 自動開始檢測循環
✓ 主動監控Flow1狀態
✓ 持續確保保護區域有CASE_F
```

### **3. Flow1使用方式**
```python
# Flow1程序中讀取座標
def read_autofeeding_coordinates():
    # 檢查CASE_F是否可用
    case_f_available = read_register(940)
    if case_f_available == 1:
        # 讀取32位座標
        x = read_32bit_register(941, 942)
        y = read_32bit_register(943, 944)
        
        # 確認已讀取
        write_register(945, 1)
        
        return (x, y)
    return None
```

### **4. 不需要AutoProgram中介**
```
原來需要:
AutoProgram → 控制AutoFeeding → 協調Flow1

現在只需要:
AutoFeeding ↔ Flow1 直接交握
```

## ⚡ 執行時序圖

### **正常運作流程**
```
時間軸  AutoFeeding              Flow1              Dobot_main
  |         |                     |                     |
  t1    啟動檢測循環                |                     |
  |         |                     |                     |
  t2    找到CASE_F                |                     |
  |     設置940=1,941-944         |                     |
  |         |                     |                     |
  t3        |              檢查940=1                   |
  |         |              讀取941-944                 |
  |         |              設置945=1                   |
  |         |                     |                     |
  t4    檢測到945=1               |           寫入1240=1
  |     清除CASE_F狀態             |                     |
  |         |                     |                     |
  t5    檢測到1240=1              |                     |
  |     暫停檢測                    |              Flow1執行中
  |         |                     |                     |
  t6        |                     |           寫入1240=0
  |         |                     |                     |
  t7    檢測到1240=0              |                     |
  |     恢復檢測                    |                     |
  |         |                     |                     |
  t8    繼續檢測新CASE_F          |                     |
```

## 🎛️ 配置參數

### **時序優化配置**
```python
"autofeeding": {
    "cycle_interval": 1.0,        # 檢測週期1秒
    "ccd1_timeout": 5.0,          # CCD1超時5秒
    "auto_start": True            # 自動啟動檢測
},
"timing": {
    "flow1_check_interval": 0.1   # Flow1監控間隔0.1秒
}
```

### **VP震動參數**
```python
"vp_params": {
    "spread_strength": 60,        # 震動強度60
    "spread_frequency": 50,       # 震動頻率50Hz
    "spread_duration": 0.3        # 震動持續0.3秒
}
```

## 🔧 主要改進功能

### **1. 持續檢測策略**
```python
def feeding_cycle(self) -> bool:
    # 找到CASE_F後設置可用狀態，但不暫停
    if target_coords:
        self.set_case_f_available(target_coords)
        # 🟢 繼續檢測確保持續有料件，不暫停
        print("[AutoFeeding] CASE_F已就緒，繼續檢測確保料件充足")
```

### **2. 主動Flow1監控**
```python
def main_loop(self):
    while True:
        # 🟢 主動監控Flow1狀態
        self.check_flow1_status()
        
        # 🟢 檢查座標是否被讀取
        if self.case_f_available:
            self.check_coords_taken()
        
        # 只有在Flow1未執行時才檢測
        if self.running and not self.flow1_active:
            self.feeding_cycle()
```

### **3. 自動啟動機制**
```python
# 主循環中自動啟動
auto_start = self.config['autofeeding'].get('auto_start', True)
if auto_start:
    self.start_feeding()
```

## 📊 狀態監控

### **AutoFeeding狀態 (900寄存器)**
- 0: 停止
- 1: 運行中 (正常檢測)
- 2: Flow1暫停 (1240=1時)
- 3: 檢測中 (CCD1作業)
- 4: VP震動中
- 5: 錯誤

### **Flow1監控狀態 (909寄存器)**
- 0: Flow1未執行
- 1: Flow1執行中

### **CASE_F狀態追蹤**
```python
self.case_f_available     # 是否有CASE_F可用
self.case_f_coords        # 當前CASE_F座標
self.case_f_taken         # 座標是否已被讀取
```

## 🎉 使用優勢

### **對開發者**
- **簡化整合**: 不需要複雜的AutoProgram協調邏輯
- **直接交握**: Flow1直接讀取座標，邏輯清晰
- **自動運行**: 啟動即可工作，無需手動觸發
- **狀態透明**: 所有狀態都有寄存器對應

### **對系統運行**
- **持續供料**: 保護區域隨時有CASE_F可用
- **快速響應**: 0.1秒間隔監控Flow1狀態
- **無死鎖**: 主動監控機制避免等待卡住
- **高可靠**: 獨立進程，異常不影響其他模組

### **對維護調試**
- **清晰日誌**: 所有狀態變化都有日誌記錄
- **狀態可見**: 寄存器狀態可實時監控
- **獨立運行**: 可單獨啟動測試
- **簡單重啟**: 重啟不影響其他模組

## 🚨 注意事項

### **1. 啟動順序**
```
建議啟動順序:
1. 主Modbus TCP Server (端口502)
2. CCD1視覺檢測模組
3. VP震動盤模組  
4. AutoFeeding_main.py (自動啟動檢測)
5. Dobot_main (包含Flow1)
```

### **2. Flow1程序修改**
```
Flow1需要包含座標讀取邏輯:
- 檢查940=1
- 讀取941-944座標
- 設置945=1確認
- 執行後續動作
```

### **3. AutoProgram角色變化**
```
AutoProgram不再需要:
❌ 啟動/停止AutoFeeding (920控制)
❌ 暫停/恢復AutoFeeding (921控制)  
❌ 座標中轉處理
❌ 複雜交握協議

AutoProgram現在只需要:
✅ 監控系統整體狀態
✅ Flow5完成後的狀態重置
✅ 異常處理和恢復
```

## 🎯 總結

修正後的AutoFeeding實現了真正的**背景服務**模式：
- **持續不斷**的檢測確保料件供應
- **主動監控**Flow1避免衝突
- **直接交握**簡化整合難度
- **自動運行**降低操作複雜度

這個設計讓AutoFeeding變成一個可靠的料件供應服務，Flow1可以隨時取得所需的座標，整個系統更加穩定和高效。