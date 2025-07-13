#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_autofeeding.py - AutoFeeding座標讀取獨立測試工具
診斷AutoFeeding模組交握協議和座標讀取功能
"""

import time
import traceback
from typing import Optional, Dict

try:
    from pymodbus.client import ModbusTcpClient
    MODBUS_AVAILABLE = True
except ImportError:
    print("pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


class AutoFeedingTester:
    """AutoFeeding測試器"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # AutoFeeding完整寄存器映射
        self.REGISTERS = {
            # 模組狀態 (900-919)
            'AF_MODULE_STATUS': 900,      # AutoFeeding模組狀態
            'AF_CYCLE_COUNT': 901,        # 檢測週期計數
            'AF_CASE_F_FOUND': 902,       # CASE_F找到次數
            'AF_FLOW4_TRIGGER': 903,      # Flow4觸發次數
            'AF_VP_VIBRATION': 904,       # VP震動次數
            'AF_CONSECUTIVE_FLOW4': 905,  # 連續直振次數
            'AF_VP_EMPTY_COUNT': 906,     # VP空檢測次數
            'AF_ERROR_CODE': 907,         # 錯誤代碼
            'AF_OPERATION_STATUS': 908,   # 當前操作狀態
            'AF_VP_CLEAR_MODE': 909,      # VP清空模式標誌
            
            # 控制寄存器 (920-929)
            'AF_RUN_CONTROL': 920,        # 啟動/停止控制
            'AF_PAUSE_CONTROL': 921,      # 暫停/恢復控制
            'AF_VP_FORCE_STOP': 924,      # VP強制停止
            
            # 入料交握寄存器 (940-959)
            'FEEDING_COMPLETE': 940,      # 入料完成標誌
            'TARGET_X_HIGH': 941,         # 料件座標X高位
            'TARGET_X_LOW': 942,          # 料件座標X低位
            'TARGET_Y_HIGH': 943,         # 料件座標Y高位
            'TARGET_Y_LOW': 944,          # 料件座標Y低位
            'AUTOPROGRAM_CONFIRM': 945,   # AutoProgram確認讀取
            'TOTAL_DETECTIONS': 946,      # 檢測結果總數
            'CASE_F_COUNT': 947,          # CASE_F總數
            
            # CCD1相關寄存器
            'CCD1_STATUS': 201,           # CCD1狀態
            'CCD1_HANDSHAKE': 200,        # CCD1握手寄存器
            
            # VP相關寄存器
            'VP_MODULE_STATUS': 300,      # VP模組狀態
            'VP_DEVICE_CONNECTION': 301,  # VP設備連接
            'VP_ACTION': 320,             # VP動作碼
            
            # Flow4控制
            'FLOW4_CONTROL': 448,         # Flow4控制
        }
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("✗ pymodbus不可用")
                return False
            
            print(f"正在連接AutoFeeding測試: {self.modbus_host}:{self.modbus_port}")
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=5.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                print("✓ AutoFeeding Modbus連接成功")
                return True
            else:
                print("✗ AutoFeeding Modbus連接失敗")
                return False
                
        except Exception as e:
            print(f"✗ AutoFeeding連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
                print("AutoFeeding連接已斷開")
            except:
                pass
        self.connected = False
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取單個寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                print(f"✗ 讀取寄存器{register_name}({address})失敗: {result}")
                return None
                
        except Exception as e:
            print(f"✗ 讀取寄存器{register_name}異常: {e}")
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入單個寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            success = not result.isError()
            
            if success:
                print(f"✓ 寫入寄存器{register_name}({address})={value}成功")
            else:
                print(f"✗ 寫入寄存器{register_name}({address})={value}失敗: {result}")
            
            return success
            
        except Exception as e:
            print(f"✗ 寫入寄存器{register_name}異常: {e}")
            return False
    
    def read_32bit_coordinate(self, high_reg: str, low_reg: str) -> float:
        """讀取32位世界座標"""
        try:
            high_val = self.read_register(high_reg) or 0
            low_val = self.read_register(low_reg) or 0
            
            print(f"    {high_reg}({self.REGISTERS[high_reg]}): {high_val}")
            print(f"    {low_reg}({self.REGISTERS[low_reg]}): {low_val}")
            
            # 32位合併
            combined = (high_val << 16) | low_val
            print(f"    合併32位值: {combined}")
            
            # 處理負數 (補碼轉換)
            if combined >= 2**31:
                combined -= 2**32
                print(f"    補碼轉換後: {combined}")
            
            # 恢復精度 (÷100)
            world_coord = combined / 100.0
            print(f"    最終座標: {world_coord:.2f} mm")
            
            return world_coord
            
        except Exception as e:
            print(f"✗ 讀取32位座標異常: {e}")
            return 0.0
    
    def test_system_status(self):
        """測試系統狀態"""
        print("\n" + "="*60)
        print("🔍 測試1: 系統狀態檢查")
        print("="*60)
        
        # AutoFeeding狀態
        af_status = self.read_register('AF_MODULE_STATUS')
        af_run_control = self.read_register('AF_RUN_CONTROL')
        af_pause_control = self.read_register('AF_PAUSE_CONTROL')
        
        print(f"AutoFeeding模組狀態(900): {af_status}")
        print(f"  - 0: 停止, 1: 運行中, 2: 暫停, 3: 檢測中, 4: VP震動中, 5: 錯誤")
        print(f"AutoFeeding啟動控制(920): {af_run_control}")
        print(f"  - 0: 停止, 1: 啟動")
        print(f"AutoFeeding暫停控制(921): {af_pause_control}")
        print(f"  - 0: 正常, 1: 暫停")
        
        # CCD1狀態
        ccd1_status = self.read_register('CCD1_STATUS')
        print(f"CCD1狀態(201): {ccd1_status}")
        if ccd1_status:
            ready = (ccd1_status & 0x01) != 0
            initialized = (ccd1_status & 0x02) != 0
            alarm = (ccd1_status & 0x04) != 0
            print(f"  - Ready: {ready}, Initialized: {initialized}, Alarm: {alarm}")
        
        # VP狀態
        vp_status = self.read_register('VP_MODULE_STATUS')
        vp_connection = self.read_register('VP_DEVICE_CONNECTION')
        print(f"VP模組狀態(300): {vp_status}")
        print(f"VP設備連接(301): {vp_connection}")
        
        return af_status, af_run_control, af_pause_control
    
    def test_statistics(self):
        """測試統計資訊"""
        print("\n" + "="*60)
        print("📊 測試2: 統計資訊")
        print("="*60)
        
        cycle_count = self.read_register('AF_CYCLE_COUNT')
        case_f_found = self.read_register('AF_CASE_F_FOUND')
        flow4_trigger = self.read_register('AF_FLOW4_TRIGGER')
        vp_vibration = self.read_register('AF_VP_VIBRATION')
        consecutive_flow4 = self.read_register('AF_CONSECUTIVE_FLOW4')
        vp_empty_count = self.read_register('AF_VP_EMPTY_COUNT')
        error_code = self.read_register('AF_ERROR_CODE')
        operation_status = self.read_register('AF_OPERATION_STATUS')
        
        print(f"檢測週期計數(901): {cycle_count}")
        print(f"CASE_F找到次數(902): {case_f_found}")
        print(f"Flow4觸發次數(903): {flow4_trigger}")
        print(f"VP震動次數(904): {vp_vibration}")
        print(f"連續直振次數(905): {consecutive_flow4}")
        print(f"VP空檢測次數(906): {vp_empty_count}")
        print(f"錯誤代碼(907): {error_code}")
        print(f"當前操作狀態(908): {operation_status}")
        
        if cycle_count > 0:
            case_f_rate = (case_f_found / cycle_count) * 100
            print(f"CASE_F找到率: {case_f_rate:.1f}%")
    
    def test_feeding_status(self):
        """測試入料狀態"""
        print("\n" + "="*60)
        print("🎯 測試3: 入料交握狀態")
        print("="*60)
        
        feeding_complete = self.read_register('FEEDING_COMPLETE')
        total_detections = self.read_register('TOTAL_DETECTIONS')
        case_f_count = self.read_register('CASE_F_COUNT')
        confirm_status = self.read_register('AUTOPROGRAM_CONFIRM')
        
        print(f"入料完成標誌(940): {feeding_complete}")
        print(f"檢測結果總數(946): {total_detections}")
        print(f"CASE_F總數(947): {case_f_count}")
        print(f"AutoProgram確認(945): {confirm_status}")
        
        return feeding_complete == 1
    
    def test_coordinate_reading(self):
        """測試座標讀取"""
        print("\n" + "="*60)
        print("📍 測試4: 座標讀取")
        print("="*60)
        
        # 檢查入料完成標誌
        feeding_complete = self.read_register('FEEDING_COMPLETE')
        if feeding_complete != 1:
            print(f"⚠️ 入料未完成(940={feeding_complete})，無法讀取座標")
            return None
        
        print("✓ 入料已完成，開始讀取座標...")
        
        # 讀取X座標
        print("讀取X座標:")
        target_x = self.read_32bit_coordinate('TARGET_X_HIGH', 'TARGET_X_LOW')
        
        # 讀取Y座標
        print("讀取Y座標:")
        target_y = self.read_32bit_coordinate('TARGET_Y_HIGH', 'TARGET_Y_LOW')
        
        # 讀取統計
        total_detections = self.read_register('TOTAL_DETECTIONS')
        case_f_count = self.read_register('CASE_F_COUNT')
        
        coordinates = {
            'x': target_x,
            'y': target_y,
            'total_detections': total_detections,
            'case_f_count': case_f_count
        }
        
        print(f"\n✓ 座標讀取完成:")
        print(f"  目標座標: ({target_x:.2f}, {target_y:.2f}) mm")
        print(f"  檢測統計: 總數={total_detections}, CASE_F={case_f_count}")
        
        return coordinates
    
    def test_handshake_protocol(self):
        """測試完整交握協議"""
        print("\n" + "="*60)
        print("🤝 測試5: 完整交握協議")
        print("="*60)
        
        # 1. 檢查入料完成
        feeding_complete = self.read_register('FEEDING_COMPLETE')
        if feeding_complete != 1:
            print(f"⚠️ 入料未完成(940={feeding_complete})，跳過交握測試")
            return False
        
        print("✓ 步驟1: 入料完成標誌確認")
        
        # 2. 讀取座標
        coordinates = self.test_coordinate_reading()
        if not coordinates:
            print("✗ 步驟2: 座標讀取失敗")
            return False
        
        print("✓ 步驟2: 座標讀取成功")
        
        # 3. 確認讀取
        print("步驟3: 發送確認讀取信號...")
        if self.write_register('AUTOPROGRAM_CONFIRM', 1):
            print("✓ 步驟3: 確認讀取信號發送成功")
        else:
            print("✗ 步驟3: 確認讀取信號發送失敗")
            return False
        
        # 4. 等待AutoFeeding清除入料完成標誌
        print("步驟4: 等待AutoFeeding清除入料完成標誌...")
        timeout = 3.0  # 3秒超時
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            current_feeding_complete = self.read_register('FEEDING_COMPLETE')
            if current_feeding_complete == 0:
                elapsed = time.time() - start_time
                print(f"✓ 步驟4: 入料完成標誌已清除 (耗時{elapsed:.2f}s)")
                return True
            time.sleep(0.1)
        
        print(f"✗ 步驟4: 等待入料完成標誌清除超時 (3秒)")
        return False
    
    def test_manual_control(self):
        """測試手動控制"""
        print("\n" + "="*60)
        print("🎮 測試6: 手動控制測試")
        print("="*60)
        
        print("測試AutoFeeding啟動/停止控制...")
        
        # 讀取當前狀態
        current_run = self.read_register('AF_RUN_CONTROL')
        current_status = self.read_register('AF_MODULE_STATUS')
        
        print(f"當前啟動控制(920): {current_run}")
        print(f"當前模組狀態(900): {current_status}")
        
        # 如果當前是停止狀態，嘗試啟動
        if current_run == 0:
            print("嘗試啟動AutoFeeding...")
            if self.write_register('AF_RUN_CONTROL', 1):
                time.sleep(2.0)
                new_status = self.read_register('AF_MODULE_STATUS')
                print(f"啟動後模組狀態(900): {new_status}")
                
                if new_status == 1:
                    print("✓ AutoFeeding啟動成功")
                else:
                    print(f"⚠️ AutoFeeding啟動後狀態異常: {new_status}")
        else:
            print("AutoFeeding已在運行中")
        
        # 測試Flow4觸發
        print("\n測試Flow4送料觸發...")
        if self.write_register('FLOW4_CONTROL', 1):
            time.sleep(0.1)
            self.write_register('FLOW4_CONTROL', 0)
            print("✓ Flow4脈衝觸發完成")
        else:
            print("✗ Flow4觸發失敗")
    
    def run_diagnostic(self):
        """運行完整診斷"""
        print("🔧 AutoFeeding座標讀取診斷工具")
        print("="*60)
        print(f"測試目標: {self.modbus_host}:{self.modbus_port}")
        print("="*60)
        
        if not self.connect():
            print("✗ 連接失敗，診斷中止")
            return
        
        try:
            # 測試1: 系統狀態
            af_status, af_run, af_pause = self.test_system_status()
            
            # 測試2: 統計資訊
            self.test_statistics()
            
            # 測試3: 入料狀態
            has_feeding = self.test_feeding_status()
            
            # 測試4: 座標讀取
            coordinates = None
            if has_feeding:
                coordinates = self.test_coordinate_reading()
            
            # 測試5: 完整交握協議
            if has_feeding and coordinates:
                handshake_success = self.test_handshake_protocol()
            else:
                print("\n⚠️ 跳過交握協議測試 (無入料完成)")
                handshake_success = False
            
            # 測試6: 手動控制
            self.test_manual_control()
            
            # 診斷總結
            print("\n" + "="*60)
            print("📋 診斷總結")
            print("="*60)
            
            print(f"✓ Modbus連接: 成功")
            print(f"AutoFeeding狀態: {af_status} ({'運行中' if af_status == 1 else '停止/異常'})")
            print(f"啟動控制: {af_run} ({'已啟動' if af_run == 1 else '未啟動'})")
            print(f"暫停控制: {af_pause} ({'暫停中' if af_pause == 1 else '正常'})")
            print(f"入料完成: {'是' if has_feeding else '否'}")
            print(f"座標讀取: {'成功' if coordinates else '失敗/無數據'}")
            print(f"交握協議: {'成功' if handshake_success else '失敗/跳過'}")
            
            if coordinates:
                print(f"最終座標: ({coordinates['x']:.2f}, {coordinates['y']:.2f}) mm")
            
            # 建議
            print("\n💡 建議:")
            if af_status != 1:
                print("- AutoFeeding模組未運行，檢查920寄存器或模組程序")
            if af_run != 1:
                print("- 啟動控制未設置，需要AutoProgram寫入920=1")
            if af_pause == 1:
                print("- AutoFeeding處於暫停狀態，檢查921寄存器")
            if not has_feeding:
                print("- 無入料完成，檢查VP上是否有料件或CCD1檢測")
            if has_feeding and not coordinates:
                print("- 入料完成但座標讀取失敗，檢查941-944寄存器")
                
        except Exception as e:
            print(f"\n✗ 診斷過程異常: {e}")
            traceback.print_exc()
        
        finally:
            self.disconnect()
    
    def interactive_test(self):
        """互動式測試"""
        if not self.connect():
            return
            
        try:
            while True:
                print("\n" + "="*50)
                print("🔧 AutoFeeding互動式測試")
                print("="*50)
                print("1. 查看系統狀態")
                print("2. 查看入料狀態")
                print("3. 讀取座標")
                print("4. 執行完整交握")
                print("5. 啟動AutoFeeding")
                print("6. 停止AutoFeeding")
                print("7. 觸發Flow4送料")
                print("8. 運行完整診斷")
                print("0. 退出")
                
                choice = input("\n請選擇操作 (0-8): ").strip()
                
                if choice == '0':
                    break
                elif choice == '1':
                    self.test_system_status()
                elif choice == '2':
                    self.test_feeding_status()
                elif choice == '3':
                    self.test_coordinate_reading()
                elif choice == '4':
                    self.test_handshake_protocol()
                elif choice == '5':
                    self.write_register('AF_RUN_CONTROL', 1)
                elif choice == '6':
                    self.write_register('AF_RUN_CONTROL', 0)
                elif choice == '7':
                    if self.write_register('FLOW4_CONTROL', 1):
                        time.sleep(0.1)
                        self.write_register('FLOW4_CONTROL', 0)
                elif choice == '8':
                    self.disconnect()
                    self.run_diagnostic()
                    self.connect()
                else:
                    print("無效選擇")
                    
        except KeyboardInterrupt:
            print("\n用戶中斷")
        except Exception as e:
            print(f"互動測試異常: {e}")
            traceback.print_exc()
        finally:
            self.disconnect()


def main():
    """主程序"""
    print("AutoFeeding座標讀取獨立測試工具")
    
    # 可以修改這裡的IP和端口
    tester = AutoFeedingTester(modbus_host="127.0.0.1", modbus_port=502)
    
    print("\n請選擇測試模式:")
    print("1. 運行完整診斷 (推薦)")
    print("2. 互動式測試")
    
    choice = input("請選擇 (1-2): ").strip()
    
    if choice == '1':
        tester.run_diagnostic()
    elif choice == '2':
        tester.interactive_test()
    else:
        print("無效選擇，運行完整診斷...")
        tester.run_diagnostic()


if __name__ == "__main__":
    main()