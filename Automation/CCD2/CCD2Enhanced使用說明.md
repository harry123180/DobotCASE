# CCD2圖像分類增強模組使用說明

## 概述

CCD2Classification_Enhanced.py是基於CCD1架構整合的圖像分類模組，提供Web介面與圖像可視化功能。

## 主要特性

### 核心功能
- **圖像分類**: 基於JSON配置的特徵分類引擎
- **Web介面**: 整合Flask + SocketIO的即時控制介面  
- **握手協議**: 50ms高頻輪詢的Modbus TCP控制
- **可視化**: 圖像處理結果即時顯示

### 技術架構
```
CCD2Enhanced → camera_manager API → 海康威視相機(192.168.1.9)
     ↓
Modbus TCP Client → 主服務器(127.0.0.1:502)
     ↓  
Web介面(localhost:5052) + SocketIO即時通訊
```

## 檔案結構

```
CCD2/
├── CCD2Classification_Enhanced.py    # 主程序
├── templates/
│   └── ccd2_classification.html      # Web介面
├── condition/                        # 分類配置資料夾
│   └── *.json                       # JSON配置檔案
├── ccd2_enhanced_config.json         # 主配置檔案(自動生成)
└── ../API/camera_manager.py          # 相機管理API
```

## 配置說明

### 主配置檔案 (ccd2_enhanced_config.json)
```json
{
  "module_id": "CCD2圖像分類增強模組",
  "camera": {
    "ip": "192.168.1.9"
  },
  "tcp_server": {
    "host": "127.0.0.1",
    "port": 502
  },
  "modbus_mapping": {
    "base_address": 1100
  },
  "web_server": {
    "host": "localhost", 
    "port": 5052
  }
}
```

### 分類配置檔案 (condition/*.json) - 完整版本
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

## 參數配置說明

### **重要**: 參數來源
- ✅ **圖像處理參數**: 僅從JSON配置檔案讀取
- ✅ **ROI設定**: 僅從JSON配置檔案讀取  
- ✅ **分類條件**: 僅從JSON配置檔案讀取
- ❌ **不從Modbus寄存器讀取參數**

### JSON配置結構

#### 1. **圖像處理參數**
```json
"processing_parameters": {
  "gaussian_kernel": 5,        // 高斯模糊核大小
  "use_otsu": true,           // 是否使用OTSU閾值
  "manual_threshold": 127,    // 手動閾值
  "canny_low": 50,           // Canny低閾值
  "canny_high": 150,         // Canny高閾值
  "lbp_radius": 3,           // LBP半徑
  "lbp_points": 24,          // LBP點數
  "roi": {
    "enabled": true,         // ROI啟用
    "x": 100,               // ROI起始X座標
    "y": 100,               // ROI起始Y座標  
    "width": 200,           // ROI寬度
    "height": 200           // ROI高度
  }
}
```

#### 2. **分類條件配置**
```json
"categories": [
  {
    "id": 1,
    "name": "類型A", 
    "enabled": true,
    "logic": "AND",           // 條件邏輯: AND/OR
    "conditions": [
      {
        "feature": "平均值",   // 特徵: 平均值/標準差/偏度/峰度
        "operator": ">",      // 運算子: >/>=/</<=/==
        "threshold": 100.0,   // 閾值
        "enabled": true
      }
    ]
  }
]
```

### 分類結果寄存器 (1140-1159)
| 地址 | 功能 | 說明 |
|------|------|------|
| 1140 | 成功標誌 | 0=失敗, 1=成功 |
| 1141 | 類別ID | 分類結果ID |
| 1142-1143 | 信心度 | 32位信心度(×100) |
| 1144 | 匹配條件數 | 符合條件數量 |

### 特徵值寄存器 (1150-1159)
| 地址 | 特徵 | 說明 |
|------|------|------|
| 1150-1151 | 平均值 | 32位亮度平均值(×100) |
| 1152-1153 | 標準差 | 32位亮度標準差(×100) |
| 1154-1155 | 偏度 | 32位偏度值(×1000) |
| 1156-1157 | 峰度 | 32位峰度值(×1000) |

### 統計資訊寄存器 (1180-1199)
| 地址 | 功能 | 說明 |
|------|------|------|
| 1180 | 處理耗時 | 毫秒單位 |
| 1181 | 分類計數 | 累積分類次數 |
| 1182 | 錯誤計數 | 累積錯誤次數 |
| 1183 | 配置載入次數 | JSON載入計數 |
| 1190-1191 | 軟體版本 | 主版本.次版本 |
| 1192-1193 | 運行時間 | 小時.分鐘 |

## 運行步驟

### 1. 環境準備
```bash
# 確保相機連接到192.168.1.9
# 確保主Modbus TCP Server運行在502端口
```

### 2. 啟動服務
```bash
python CCD2Classification_Enhanced.py
```

### 3. 訪問Web介面
```
http://localhost:5052
```

### 4. 操作流程
1. **連接Modbus**: 輸入主服務器地址並連接
2. **初始化相機**: 點擊初始化相機按鈕
3. **掃描配置**: 掃描condition資料夾中的JSON檔案
4. **載入配置**: 載入分類配置檔案
5. **執行分類**: 點擊「拍照+分類」執行檢測

## Web介面功能

### 連接狀態監控
- **Modbus TCP**: 顯示主服務器連接狀態
- **相機連接**: 顯示海康威視相機連接狀態  
- **配置載入**: 顯示JSON配置檔案載入狀態
- **系統狀態**: 顯示整體系統就緒狀態

### 控制功能
- **連接控制**: Modbus服務器連接設定
- **相機控制**: 相機初始化與連接管理
- **分類控制**: 拍照分類執行控制
- **配置管理**: JSON配置檔案掃描與載入

### 結果顯示
- **分類結果**: 類別ID、信心度、匹配條件數
- **特徵數值**: 平均值、標準差、偏度、峰度
- **統計資訊**: 分類次數、錯誤次數、運行時間
- **圖像可視化**: 處理後圖像與結果標註

## 控制指令說明

### PLC控制指令
- **指令8**: 單純拍照，不執行分類
- **指令16**: 拍照+執行圖像分類檢測
- **指令32**: 重新初始化相機連接
- **指令64**: 重新載入JSON分類配置

### 握手協議
1. 系統就緒 → 狀態寄存器Ready=1
2. PLC下達指令 → 檢查Ready狀態
3. 開始執行 → Ready=0, Running=1  
4. 執行完成 → Running=0
5. PLC清零指令 → 恢復Ready=1

## 錯誤處理

### 常見問題
1. **相機連接失敗**
   - 檢查IP地址192.168.1.9是否正確
   - 確認camera_manager.py模組可用
   - 檢查海康威視SDK安裝

2. **Modbus連接失敗**  
   - 確認主服務器運行在502端口
   - 檢查網路連接與防火牆設定
   - 驗證基地址1100未被占用

3. **配置載入失敗**
   - 檢查condition資料夾是否存在
   - 驗證JSON檔案格式正確性
   - 確認檔案編碼為UTF-8

4. **分類結果異常**
   - 檢查圖像品質與光照條件
   - 調整分類閾值參數
   - 驗證特徵提取邏輯

### 日誌輸出
```
CCD2圖像分類增強模組啟動中...
系統架構: Modbus TCP Client + Web介面  
基地址: 1100
相機IP: 192.168.1.9
Web介面: http://localhost:5052
✓ 分類配置載入成功
✓ CCD2圖像分類增強模組啟動完成
```

## 與其他模組的關係

### 基地址分配
- **XC100升降模組**: 1000-1049
- **VP震動盤模組**: 300-349  
- **CCD1視覺模組**: 200-299
- **CCD2分類模組**: 1100-1199 (新分配)
- **機械臂模組**: 400-449

### 共享資源
- **主Modbus TCP Server**: 502端口
- **camera_manager API**: 相機管理介面
- **系統日誌**: 統一錯誤處理機制

## 技術特點

### 相對於原版CCD2的改進
1. **Web介面整合**: 提供圖形化控制介面
2. **即時狀態監控**: SocketIO即時通訊
3. **圖像可視化**: 分類結果圖像標註
4. **增強錯誤處理**: 完善的異常管理機制
5. **模組化設計**: 參考CCD1成熟架構

### 技術依賴
- **PyModbus 3.9.2**: Modbus TCP通訊
- **Flask + SocketIO**: Web介面框架
- **OpenCV**: 圖像處理
- **NumPy**: 數值計算  
- **scikit-image**: 紋理分析(可選)

## 部署建議

### 生產環境配置
- 關閉Web介面debug模式
- 設定適當的日誌等級
- 配置服務自動重啟機制
- 監控系統資源使用

### 性能調優
- 調整圖像處理參數減少耗時
- 優化JSON配置檔案大小
- 合理設定輪詢間隔時間
- 定期清理臨時檔案