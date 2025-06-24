## 流程與點位對應關係

### Flow1流程點位序列
```
standby → (JointMovJ)vp_topside → (MovL)CCD1檢測物件座標 → (JointMovJ)vp_topside → standby 
→ flip_pre → flip_top → flip_down → flip_top → flip_pre → standby
```

### Flow2流程點位序列  
```
standby → Goal_CV_top → Goal_CV_down → Goal_CV_top 
→ rotate_top → rotate_down → rotate_top → put_asm_pre
```

### Flow3流程點位序列
```
put_asm_pre → rotate_top → rotate_down → rotate_top → put_asm_pre 
→ put_asm_top → put_asm_down → put_asm_top → put_asm_pre → standby
```# Dobot M1Pro機械臂三流程規範文檔

## 流程概述

本文檔定義Dobot M1Pro機械臂的三個核心自動化流程，實現完整的視覺抓取→翻轉檢測→角度校正→組裝放置工作循環。

### 流程序列
1. **Flow1** - VP視覺抓取流程
2. **Flow2** - CV出料流程  
3. **Flow3** - 組裝作業流程

## 設計原則

### 結構簡明精簡
- 每個流程職責單一，介面清晰
- 使用高階API封裝底層複雜性
- 統一錯誤處理機制
- 最小化模組間耦合

### 易於維護
- 獨立配置檔案管理
- 標準化日誌記錄
- 模組化組件設計
- 清晰的狀態機管理

## 系統架構

```
同層資料夾/
├── Dobot_main.py          # 主控制器
├── Dobot_Flow1.py         # VP視覺抓取流程
├── Dobot_Flow2.py         # CV出料流程
├── Dobot_Flow3.py         # 組裝作業流程 [待實現]
├── CCD1HighLevel.py       # 視覺檢測API
├── GripperHighLevel.py    # 夾爪控制API
├── AngleHighLevel.py      # 角度校正API
└── dobot_api.py           # 機械臂底層API
```

## Flow1 - VP視覺抓取流程

### 業務需求
從VP震動盤抓取料件，執行翻轉檢測，為後續流程準備料件。

### 流程步驟
```
standby → 夾爪關閉 → vp_topside → CCD1檢測API → 移動到視覺檢測到的物件座標(Z軸高度與vp_topside等高)
→ 移動到檢測到的物件座標(Z軸到夾取位置) → 夾爪撐開至370 → 移動到vp_topside → 移動到standby
→ 移動到flip_pre → 移動到flip_top → 移動到flip_down → 夾爪關閉 → 移動到flip_top 
→ 移動到flip_pre → 移動到standby → 觸發CCD2(物件正反面辨識與輸送帶翻轉機構的IO控制)
```

### 關鍵特性
- **視覺檢測**: CCD1HighLevel API提供FIFO佇列管理
- **智能夾取**: GripperHighLevel API自動判斷夾取成功  
- **翻轉檢測**: 觸發CCD2模組進行物件正反面辨識與輸送帶翻轉

### IO操作設計
根據專案知識中的dobot_api，CCD2翻轉檢測模組IO控制：

```python
# dobot_api支援的IO操作
def DOExecute(self, offset1, offset2):  # 立即執行數位輸出設置
def DI(self, offset1):                  # 讀取數位輸入
```

**CCD2翻轉檢測模組架構**:
- 使用dashboard_api執行DOExecute/DI指令，與move_api(30003端口)隔離
- 觸發CCD2後，手臂可立即執行其他流程，不需等待CCD2完成
- CCD2動作相對耗時，採用異步IO操作設計

## Flow2 - CV出料流程

### 業務需求
從CV輸送帶撈取料件，移動到旋轉工位，結束流程。

### 流程步驟
```
standby → 夾爪關閉 → Goal_CV_top → Goal_CV_down → 夾爪撐開到370(智慧撐開)
→ Goal_CV_top → rotate_top → rotate_down → 夾爪快速關閉 → rotate_top → put_asm_pre
```

### 關鍵特性
- **智能撐開**: 在Goal_CV_down撐開料件至370位置
- **快速操作**: rotate_down使用快速關閉夾爪
- **流程結束**: put_asm_pre為Flow2結束點

## Flow3 - 組裝作業流程

### 業務需求
當物件旋轉到位後，拿去組裝區組裝。

### 流程步驟  
```
put_asm_pre → 夾爪快速關閉 → rotate_top → rotate_down → 夾爪撐開到370(智慧撐開)
→ rotate_top → put_asm_pre → put_asm_top → put_asm_down → 夾爪快速關閉
→ put_asm_top → put_asm_pre → standby
```

### 關鍵特性
- **角度調教**: 流程開始前使用AngleHighLevel API調教角度
- **精密定位**: put_asm系列點位確保組裝精度
- **料件釋放**: put_asm_down快速關閉完成組裝
- **循環完成**: 回到standby準備下一循環

### 流程觸發時機
Flow3在Flow2結束後，透過AngleHighLevel調教角度完成後觸發

## 高階API模組規範

### CCD1HighLevel - 視覺檢測API

```python
class CCD1HighLevelAPI:
    def get_next_circle_world_coord(self) -> Optional[CircleWorldCoord]:
        """獲取下一個物件世界座標 (FIFO佇列)"""
        
    def capture_and_detect(self) -> bool:
        """執行拍照+檢測 (自動握手協議)"""
        
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
```

### GripperHighLevel - 夾爪控制API

```python
class GripperHighLevelAPI:
    def smart_grip(self, target_position: int = 420) -> bool:
        """智能夾取 - 自動判斷成功"""
        
    def smart_release(self, release_position: int = 50) -> bool:
        """智能釋放 - 移動到釋放位置"""
        
    def quick_close(self) -> bool:
        """快速關閉 (發了就走)"""
        
    def quick_open(self) -> bool:
        """快速開啟 (發了就走)"""
```

### AngleHighLevel - 角度校正API

```python
class AngleHighLevel:
    def adjust_to_90_degrees(self) -> AngleCorrectionResult:
        """執行角度校正到90度"""
        
    def is_system_ready(self) -> bool:
        """檢查系統準備狀態"""
        
    def reset_errors(self) -> AngleOperationResult:
        """重置系統錯誤"""
```

## 狀態機交握協議

### 寄存器映射 (基地址400)
```
400: 主狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm)
420: Flow1完成狀態 (0=未完成, 1=完成且角度校正成功)
440: VP視覺取料控制 (0=清空, 1=啟動Flow1)
441: 出料控制 (0=清空, 1=啟動Flow2)
```

### 交握流程
```
1. PLC檢查Ready狀態 (400寄存器bit0=1)
2. PLC寫入流程指令 (440或441=1)
3. 系統執行流程，設置Running狀態 (400寄存器bit1=1)
4. 流程完成，設置完成標記 (如420=1)
5. PLC讀取完成狀態並清零指令
6. 系統恢復Ready狀態
```


