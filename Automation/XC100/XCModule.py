#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XC100 超高速動作模組 - 調試版本
增加連線參數打印和連接狀態詳細顯示
"""

import os
import time
import threading
import json
import logging
from datetime import datetime
from enum import Enum
from typing import Optional, Dict, Any
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient, ModbusTcpClient

# 確保XC100目錄存在
XC100_DIR = "XC100"
if not os.path.exists(XC100_DIR):
    os.makedirs(XC100_DIR)

class XCState(Enum):
    """XC100狀態枚舉"""
    OFFLINE = 0
    IDLE = 1
    MOVING = 2
    HOMING = 3
    ERROR = 4
    SERVO_OFF = 5
    EMERGENCY = 6

class XCCommand(Enum):
    """XC100指令枚舉"""
    NOP = 0
    SERVO_ON = 1
    SERVO_OFF = 2
    HOME = 3
    MOVE_ABS = 4
    MOVE_REL = 5
    EMERGENCY_STOP = 6
    RESET_ERROR = 7

class UltraFastXCModule:
    """超高速XC100模組 - 調試版本"""
    
    def __init__(self, config_file=None):
        if config_file is None:
            # 讀取同層目錄下的配置檔案
            config_file = "xc_ultrafast_config.json"
        self.config_file = config_file
        self.config = self.load_config()
        
        self.setup_logging()
        self.module_id = self.config.get("module_id", "XC100_FAST")
        
        # 連接客戶端
        self.xc_client: Optional[ModbusSerialClient] = None
        self.tcp_client: Optional[ModbusTcpClient] = None
        self.xc_connected = False
        self.tcp_connected = False
        
        # 狀態變量 - 使用鎖保護
        self._state_lock = threading.Lock()
        self.current_state = XCState.OFFLINE
        self.servo_status = False
        self.error_code = 0
        self.current_position = 0
        self.target_position = 0
        self.command_executing = False
        self.command_start_time = 0
        
        # 位置設定
        self.position_A = self.config.get("positions", {}).get("A", 400)
        self.position_B = self.config.get("positions", {}).get("B", 2682)
        
        # 超高速線程
        self.fast_loop_thread = None
        self.fast_loop_running = False
        
        # 指令管理
        self.last_command_id = 0
        self.modbus_base_address = self.config.get("modbus_mapping", {}).get("base_address", 1000)
        
        # 連接重試計數器
        self.tcp_retry_count = 0
        self.xc_retry_count = 0
        self.max_retry = 3
        
        print(f"[調試] 模組初始化完成: {self.module_id}")
        self.print_config_summary()
    
    def print_config_summary(self):
        """打印配置摘要"""
        print("\n" + "=" * 60)
        print("XC100模組配置摘要")
        print("=" * 60)
        
        # XC100連線參數
        xc_config = self.config.get("xc_connection", {})
        print(f"XC100連線參數:")
        print(f"  端口: {xc_config.get('port', 'N/A')}")
        print(f"  波特率: {xc_config.get('baudrate', 'N/A')}")
        print(f"  站號: {xc_config.get('unit_id', 'N/A')}")
        print(f"  超時: {xc_config.get('timeout', 'N/A')}秒")
        print(f"  數據位: 8")
        print(f"  停止位: 1")
        print(f"  奇偶校驗: None")
        print(f"  重試次數: {xc_config.get('retry_count', 'N/A')}")
        print(f"  重試延遲: {xc_config.get('retry_delay', 'N/A')}秒")
        
        # TCP服務器參數
        tcp_config = self.config.get("tcp_server", {})
        print(f"\nTCP服務器參數:")
        print(f"  主機: {tcp_config.get('host', 'N/A')}")
        print(f"  端口: {tcp_config.get('port', 'N/A')}")
        print(f"  站號: {tcp_config.get('unit_id', 'N/A')}")
        print(f"  超時: {tcp_config.get('timeout', 'N/A')}秒")
        
        # Modbus映射
        modbus_config = self.config.get("modbus_mapping", {})
        print(f"\nModbus寄存器映射:")
        print(f"  基地址: {modbus_config.get('base_address', 'N/A')}")
        print(f"  寄存器數量: {modbus_config.get('register_count', 'N/A')}")
        
        # 時序參數
        timing_config = self.config.get("timing", {})
        print(f"\n時序參數:")
        print(f"  主循環間隔: {timing_config.get('fast_loop_interval', 0.02)*1000}ms")
        print(f"  移動延遲: {timing_config.get('movement_delay', 0.1)*1000}ms")
        print(f"  指令延遲: {timing_config.get('command_delay', 0.02)*1000}ms")
        print(f"  寄存器延遲: {timing_config.get('register_delay', 0.01)*1000}ms")
        print(f"  無等待模式: {timing_config.get('no_wait_mode', True)}")
        
        # 位置設定
        print(f"\n位置設定:")
        print(f"  A點位置: {self.position_A}")
        print(f"  B點位置: {self.position_B}")
        
        print("=" * 60)
    
    def print_connection_attempt(self, connection_type, config, attempt_num=1):
        """打印連線嘗試信息"""
        print(f"\n[連線嘗試 #{attempt_num}] {connection_type}")
        print("-" * 40)
        if connection_type == "XC100 RTU":
            print(f"正在連線到XC100設備...")
            print(f"  串口: {config.get('port', 'N/A')}")
            print(f"  波特率: {config.get('baudrate', 'N/A')} bps")
            print(f"  站號: {config.get('unit_id', 'N/A')}")
            print(f"  超時: {config.get('timeout', 'N/A')}秒")
            print(f"  數據格式: 8-N-1 (8數據位, 無奇偶校驗, 1停止位)")
        elif connection_type == "TCP服務器":
            print(f"正在連線到主服務器...")
            print(f"  地址: {config.get('host', 'N/A')}:{config.get('port', 'N/A')}")
            print(f"  站號: {config.get('unit_id', 'N/A')}")
            print(f"  超時: {config.get('timeout', 'N/A')}秒")
    
    def check_com_port_status(self, port):
        """檢查COM端口狀態"""
        print(f"\n[端口檢查] {port}")
        print("-" * 30)
        
        # 檢查端口是否存在
        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        print(f"系統可用端口: {available_ports}")
        
        if port in available_ports:
            print(f"✅ {port} 端口存在")
            
            # 獲取端口詳細信息
            for p in serial.tools.list_ports.comports():
                if p.device == port:
                    print(f"  描述: {p.description}")
                    print(f"  硬件ID: {p.hwid}")
                    print(f"  VID:PID: {p.vid}:{p.pid}")
                    break
        else:
            print(f"❌ {port} 端口不存在")
            print("請檢查:")
            print("  1. 設備是否正確連接")
            print("  2. 驅動程序是否安裝")
            print("  3. 端口是否被其他程序佔用")
    
    def test_xc100_communication(self):
        """測試XC100通訊"""
        if not self.xc_connected or not self.xc_client:
            print("❌ XC100未連接，無法測試通訊")
            return False
        
        print(f"\n[通訊測試] XC100設備")
        print("-" * 30)
        
        unit_id = self.config["xc_connection"]["unit_id"]
        test_results = []
        
        # 測試1: 讀取Servo狀態
        try:
            result = self.xc_client.read_holding_registers(address=0x100C, count=1, slave=unit_id)
            if not result.isError():
                servo_status = result.registers[0]
                print(f"✅ Servo狀態讀取成功: {servo_status} ({'ON' if servo_status == 1 else 'OFF'})")
                test_results.append(True)
            else:
                print(f"❌ Servo狀態讀取失敗: {result}")
                test_results.append(False)
        except Exception as e:
            print(f"❌ Servo狀態讀取異常: {e}")
            test_results.append(False)
        
        # 測試2: 讀取錯誤代碼
        try:
            result = self.xc_client.read_holding_registers(address=0x100D, count=1, slave=unit_id)
            if not result.isError():
                error_code = result.registers[0]
                print(f"✅ 錯誤代碼讀取成功: {error_code}")
                test_results.append(True)
            else:
                print(f"❌ 錯誤代碼讀取失敗: {result}")
                test_results.append(False)
        except Exception as e:
            print(f"❌ 錯誤代碼讀取異常: {e}")
            test_results.append(False)
        
        # 測試3: 讀取位置
        try:
            result = self.xc_client.read_holding_registers(address=0x1000, count=2, slave=unit_id)
            if not result.isError():
                position = (result.registers[0] << 16) | result.registers[1]
                print(f"✅ 位置讀取成功: {position}")
                test_results.append(True)
            else:
                print(f"❌ 位置讀取失敗: {result}")
                test_results.append(False)
        except Exception as e:
            print(f"❌ 位置讀取異常: {e}")
            test_results.append(False)
        
        success_rate = sum(test_results) / len(test_results) * 100
        print(f"\n通訊測試結果: {sum(test_results)}/{len(test_results)} ({success_rate:.1f}%)")
        
        return success_rate > 50
    
    def load_config(self) -> Dict[str, Any]:
        """載入超高速配置"""
        default_config = {
            "module_id": "XC100_FAST",
            "description": "XC100超高速模組",
            "xc_connection": {
                "port": "COM5",
                "baudrate": 19200,  # 確定使用19200
                "unit_id": 1,       # 確定使用站號1
                "timeout": 0.2,     
                "retry_count": 2,
                "retry_delay": 0.01
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "modbus_mapping": {
                "base_address": 1000,
                "register_count": 50
            },
            "positions": {
                "A": 400,
                "B": 2682
            },
            "timing": {
                "fast_loop_interval": 0.02,  
                "movement_delay": 0.1,       
                "command_delay": 0.02,       
                "register_delay": 0.01,      
                "no_wait_mode": True
            }
        }
        
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                loaded_config = json.load(f)
                for key, value in default_config.items():
                    if key not in loaded_config:
                        loaded_config[key] = value
                    elif isinstance(value, dict):
                        for sub_key, sub_value in value.items():
                            if sub_key not in loaded_config[key]:
                                loaded_config[key][sub_key] = sub_value
                print(f"[調試] 配置載入成功: {self.config_file}")
                print(f"[調試] 實際使用的XC連線參數:")
                print(f"  波特率: {loaded_config['xc_connection']['baudrate']}")
                print(f"  站號: {loaded_config['xc_connection']['unit_id']}")
                print(f"  端口: {loaded_config['xc_connection']['port']}")
                return loaded_config
        except FileNotFoundError:
            print(f"[調試] 配置文件不存在，創建默認配置: {self.config_file}")
            self.save_config(default_config)
        except Exception as e:
            print(f"[調試] 載入配置失敗: {e}")
            
        return default_config
    
    def save_config(self, config=None):
        """保存配置"""
        try:
            config_to_save = config or self.config
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config_to_save, f, indent=2, ensure_ascii=False)
            print(f"[調試] 配置保存成功: {self.config_file}")
        except Exception as e:
            print(f"[調試] 保存配置失敗: {e}")
    
    def setup_logging(self):
        """設置精簡日誌"""
        log_file = os.path.join(XC100_DIR, f'xc_debug_{datetime.now().strftime("%Y%m%d")}.log')
        logging.basicConfig(
            level=logging.INFO,  # 調試版本使用INFO級別
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file, encoding='utf-8'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(f'DebugXC.{self.config.get("module_id", "XC100")}')
    
    def safe_modbus_operation(self, operation_func, *args, **kwargs):
        """安全的Modbus操作包裝器"""
        try:
            return operation_func(*args, **kwargs)
        except Exception as e:
            self.logger.error(f"Modbus操作失敗: {e}")
            return None
    
    def connect_main_server(self) -> bool:
        """連接主服務器 - 增加詳細打印"""
        tcp_config = self.config["tcp_server"]
        self.print_connection_attempt("TCP服務器", tcp_config, self.tcp_retry_count + 1)
        
        try:
            if self.tcp_client:
                try:
                    self.tcp_client.close()
                except:
                    pass
            
            self.tcp_client = ModbusTcpClient(
                host=tcp_config["host"],
                port=tcp_config["port"],
                timeout=tcp_config["timeout"]
            )
            
            if self.tcp_client.connect():
                self.tcp_connected = True
                self.tcp_retry_count = 0
                print(f"✅ TCP服務器連接成功")
                return True
            else:
                print(f"❌ TCP服務器連接失敗")
                return False
        except Exception as e:
            print(f"❌ TCP連接異常: {e}")
            self.tcp_connected = False
            return False
    
    def connect_xc100(self) -> bool:
        """連接XC100 - 增加詳細打印"""
        xc_config = self.config["xc_connection"]
        self.print_connection_attempt("XC100 RTU", xc_config, self.xc_retry_count + 1)
        
        # 檢查COM端口狀態
        self.check_com_port_status(xc_config["port"])
        
        try:
            if self.xc_client:
                try:
                    self.xc_client.close()
                except:
                    pass
            
            self.xc_client = ModbusSerialClient(
                port=xc_config["port"],
                baudrate=xc_config["baudrate"],
                stopbits=1,
                parity='N',
                timeout=xc_config["timeout"]
            )
            
            print(f"正在建立連接...")
            if self.xc_client.connect():
                self.xc_connected = True
                self.xc_retry_count = 0
                with self._state_lock:
                    self.current_state = XCState.IDLE
                print(f"✅ XC100設備連接成功")
                
                # 執行通訊測試
                self.test_xc100_communication()
                return True
            else:
                print(f"❌ XC100設備連接失敗")
                return False
        except Exception as e:
            print(f"❌ XC100連接異常: {e}")
            self.xc_connected = False
            return False
    
    def ultra_fast_rtu_write(self, address, values):
        """超高速RTU寫入 - 增加調試信息"""
        if not self.xc_connected or not self.xc_client:
            print(f"[RTU寫入] 連接狀態異常: xc_connected={self.xc_connected}")
            return False
        
        try:
            unit_id = self.config["xc_connection"]["unit_id"]
            
            if isinstance(values, list):
                print(f"[RTU寫入] 寄存器 0x{address:04X}, 值 {values}, 站號 {unit_id}")
                result = self.xc_client.write_registers(
                    address=address, 
                    values=values, 
                    slave=unit_id
                )
            else:
                print(f"[RTU寫入] 寄存器 0x{address:04X}, 值 {values}, 站號 {unit_id}")
                result = self.xc_client.write_register(
                    address=address, 
                    value=values, 
                    slave=unit_id
                )
            
            if result.isError():
                print(f"[RTU寫入] 失敗: {result}")
                return False
            else:
                print(f"[RTU寫入] 成功")
                return True
            
        except Exception as e:
            print(f"[RTU寫入] 異常: {e}")
            return False
    
    def execute_xc_command_fast(self, command: XCCommand, param1=0, param2=0) -> bool:
        """超高速指令執行 - 增加調試信息"""
        print(f"\n[指令執行] {command.name}, 參數1={param1}, 參數2={param2}")
        
        with self._state_lock:
            if not self.xc_connected:
                print(f"[指令執行] XC100未連接，取消執行")
                return False
            if self.command_executing:
                print(f"[指令執行] 有指令正在執行中，取消執行")
                return False
            
            # 設置執行狀態
            self.command_executing = True
            self.command_start_time = time.time()
            print(f"[指令執行] 開始執行，時間戳: {self.command_start_time}")
        
        try:
            success = False
            
            if command == XCCommand.SERVO_ON:
                print(f"[SERVO_ON] 寫入寄存器 0x2011 = 0")
                success = self.ultra_fast_rtu_write(0x2011, 0)
                if success:
                    with self._state_lock:
                        self.current_state = XCState.IDLE
                        self.servo_status = True
                        print(f"[SERVO_ON] 狀態更新: IDLE, Servo=ON")
                
            elif command == XCCommand.SERVO_OFF:
                print(f"[SERVO_OFF] 寫入寄存器 0x2011 = 1")
                success = self.ultra_fast_rtu_write(0x2011, 1)
                if success:
                    with self._state_lock:
                        self.current_state = XCState.SERVO_OFF
                        self.servo_status = False
                        print(f"[SERVO_OFF] 狀態更新: SERVO_OFF, Servo=OFF")
                
            elif command == XCCommand.HOME:
                print(f"[HOME] 寫入寄存器 0x201E = 3")
                success = self.ultra_fast_rtu_write(0x201E, 3)
                if success:
                    with self._state_lock:
                        self.current_state = XCState.HOMING
                        print(f"[HOME] 狀態更新: HOMING")
                
            elif command == XCCommand.MOVE_ABS:
                position = (param2 << 16) | param1
                print(f"[MOVE_ABS] 目標位置: {position} (高位:{param2}, 低位:{param1})")
                with self._state_lock:
                    self.target_position = position
                
                position_high = (position >> 16) & 0xFFFF
                position_low = position & 0xFFFF
                print(f"[MOVE_ABS] 分解位置: 高位=0x{position_high:04X}, 低位=0x{position_low:04X}")
                
                # 第一個指令：寫入位置
                print(f"[MOVE_ABS] 步驟1: 寫入目標位置到 0x2002")
                success1 = self.ultra_fast_rtu_write(0x2002, [position_high, position_low])
                
                if success1:
                    # 延遲控制
                    if not self.config["timing"].get("no_wait_mode", False):
                        delay = self.config["timing"]["command_delay"]
                        print(f"[MOVE_ABS] 延遲 {delay*1000}ms")
                        time.sleep(delay)
                    
                    # 第二個指令：執行移動
                    print(f"[MOVE_ABS] 步驟2: 執行移動指令到 0x201E")
                    success2 = self.ultra_fast_rtu_write(0x201E, 1)
                    if success2:
                        with self._state_lock:
                            self.current_state = XCState.MOVING
                            print(f"[MOVE_ABS] 狀態更新: MOVING")
                        success = True
                        # 移動指令需要等待完成，不在這裡清除執行狀態
                        return True
                
            elif command == XCCommand.EMERGENCY_STOP:
                print(f"[EMERGENCY_STOP] 寫入寄存器 0x2020 = 1")
                success = self.ultra_fast_rtu_write(0x2020, 1)
                if success:
                    with self._state_lock:
                        self.current_state = XCState.EMERGENCY
                        print(f"[EMERGENCY_STOP] 狀態更新: EMERGENCY")
            
            # 對於非移動指令，立即清除執行狀態
            if command != XCCommand.MOVE_ABS:
                with self._state_lock:
                    self.command_executing = False
                    self.command_start_time = 0
                    print(f"[指令執行] 執行狀態已清除")
            
            print(f"[指令執行] 結果: {'成功' if success else '失敗'}")
            return success
            
        except Exception as e:
            print(f"[指令執行] 異常: {e}")
            with self._state_lock:
                self.command_executing = False
                self.command_start_time = 0
            return False
    
    def ultra_fast_loop(self):
        """超高速主循環 - 增加詳細狀態打印"""
        print("超高速循環開始")
        
        loop_interval = self.config["timing"]["fast_loop_interval"]
        movement_delay = self.config["timing"]["movement_delay"]
        
        error_count = 0
        max_errors = 10
        loop_count = 0
        
        while self.fast_loop_running:
            try:
                start_time = time.time()
                loop_count += 1
                
                # 每100次循環打印一次狀態
                if loop_count % 100 == 0:
                    with self._state_lock:
                        print(f"\n[循環 #{loop_count}] 狀態: {self.current_state.name}, "
                              f"TCP: {'✅' if self.tcp_connected else '❌'}, "
                              f"XC100: {'✅' if self.xc_connected else '❌'}, "
                              f"執行中: {'是' if self.command_executing else '否'}")
                
                # 1. 檢查連接狀態
                if not self.tcp_connected and self.tcp_retry_count < self.max_retry:
                    if self.connect_main_server():
                        print("TCP重新連接成功")
                    else:
                        self.tcp_retry_count += 1
                
                # 2. 讀取指令 - 增加詳細打印
                if self.tcp_connected and self.tcp_client:
                    try:
                        base_addr = self.modbus_base_address + 20
                        unit_id = self.config["tcp_server"]["unit_id"]
                        
                        result = self.tcp_client.read_holding_registers(
                            address=base_addr, count=5, slave=unit_id
                        )
                        
                        if not result.isError() and len(result.registers) >= 4:
                            command_code = result.registers[0]
                            param1 = result.registers[1]
                            param2 = result.registers[2]
                            command_id = result.registers[3]
                            
                            # 檢查新指令
                            if command_code != 0 and command_id != self.last_command_id:
                                print(f"\n[新指令] 代碼:{command_code}, 參數:{param1},{param2}, ID:{command_id}")
                                self.last_command_id = command_id
                                
                                try:
                                    command = XCCommand(command_code)
                                    
                                    # 執行指令
                                    if self.execute_xc_command_fast(command, param1, param2):
                                        print(f"[指令清除] 清除指令寄存器")
                                        # 清除指令
                                        self.safe_modbus_operation(
                                            self.tcp_client.write_registers,
                                            address=base_addr,
                                            values=[0, 0, 0, 0, 0],
                                            slave=unit_id
                                        )
                                        
                                except ValueError:
                                    print(f"[指令錯誤] 無效指令代碼: {command_code}")
                                except Exception as e:
                                    print(f"[指令處理] 失敗: {e}")
                        else:
                            if result.isError():
                                error_count += 1
                                if error_count > max_errors:
                                    print(f"[TCP錯誤] 達到最大錯誤次數，斷開TCP連接")
                                    self.tcp_connected = False
                                    error_count = 0
                                    
                    except Exception as e:
                        error_count += 1
                        if error_count % 50 == 0:  # 每50次錯誤打印一次
                            print(f"[指令讀取] 失敗 (錯誤計數: {error_count}): {e}")
                        if error_count > max_errors:
                            self.tcp_connected = False
                            error_count = 0
                
                # 3. 檢查移動完成
                with self._state_lock:
                    if (self.command_executing and 
                        self.command_start_time > 0 and 
                        time.time() - self.command_start_time > movement_delay):
                        print(f"[移動完成] 移動延遲超時，清除執行狀態")
                        self.command_executing = False
                        if self.current_state == XCState.MOVING:
                            self.current_state = XCState.IDLE
                            print(f"[狀態更新] MOVING -> IDLE")
                        self.command_start_time = 0
                
                # 4. 更新狀態到主服務器
                if self.tcp_connected and self.tcp_client:
                    try:
                        base_addr = self.modbus_base_address
                        unit_id = self.config["tcp_server"]["unit_id"]
                        
                        with self._state_lock:
                            key_data = [
                                self.current_state.value,
                                1 if self.xc_connected else 0,
                                1 if self.servo_status else 0,
                                self.error_code,
                                self.current_position & 0xFFFF,
                                (self.current_position >> 16) & 0xFFFF,
                                1 if self.command_executing else 0
                            ]
                        
                        # 每50次循環打印一次狀態更新
                        if loop_count % 50 == 0:
                            print(f"[狀態更新] 到基地址 {base_addr}: {key_data}")
                        
                        self.safe_modbus_operation(
                            self.tcp_client.write_registers,
                            address=base_addr,
                            values=key_data,
                            slave=unit_id
                        )
                        
                    except Exception as e:
                        if loop_count % 100 == 0:  # 減少錯誤打印頻率
                            print(f"[狀態更新] 失敗: {e}")
                
                # 5. 精確計時
                elapsed = time.time() - start_time
                if elapsed < loop_interval:
                    time.sleep(loop_interval - elapsed)
                    
            except Exception as e:
                print(f"[主循環] 異常: {e}")
                time.sleep(loop_interval)
        
        print("超高速循環停止")
    
    def start_fast_loop(self):
        """啟動超高速循環"""
        if not self.fast_loop_running:
            self.fast_loop_running = True
            self.fast_loop_thread = threading.Thread(target=self.ultra_fast_loop, daemon=True)
            self.fast_loop_thread.start()
            print(f"[調試] 超高速循環線程已啟動")
    
    def stop_fast_loop(self):
        """停止超高速循環"""
        self.fast_loop_running = False
        if self.fast_loop_thread:
            self.fast_loop_thread.join(timeout=2)
            print(f"[調試] 超高速循環線程已停止")
    
    def start(self):
        """啟動超高速模組"""
        print(f"\n啟動超高速XC100模組: {self.module_id}")
        print("=" * 50)
        
        # 1. 先連接主服務器
        print(f"\n步驟1: 連接主服務器")
        if not self.connect_main_server():
            print("❌ 主服務器連接失敗")
            return False
        
        # 2. 再連接XC100設備
        print(f"\n步驟2: 連接XC100設備")
        if not self.connect_xc100():
            print("⚠️ XC100連接失敗，但繼續運行")
        
        # 3. 啟動主循環
        print(f"\n步驟3: 啟動超高速循環")
        self.start_fast_loop()
        
        print(f"\n✅ 超高速模組啟動成功")
        print(f"循環間隔: {self.config['timing']['fast_loop_interval']*1000}ms")
        print(f"移動延遲: {self.config['timing']['movement_delay']*1000}ms")
        print("=" * 50)
        return True
    
    def stop(self):
        """停止模組"""
        print(f"\n停止超高速模組: {self.module_id}")
        self.stop_fast_loop()
        
        try:
            if self.xc_client and self.xc_connected:
                self.xc_client.close()
                print(f"[調試] XC100連接已關閉")
        except:
            pass
            
        try:
            if self.tcp_client and self.tcp_connected:
                self.tcp_client.close()
                print(f"[調試] TCP連接已關閉")
        except:
            pass
    
    def get_status(self):
        """獲取狀態"""
        with self._state_lock:
            return {
                "module_id": self.module_id,
                "tcp_connected": self.tcp_connected,
                "xc_connected": self.xc_connected,
                "current_state": self.current_state.name,
                "current_position": self.current_position,
                "command_executing": self.command_executing,
                "fast_mode": True
            }

def main():
    """調試版主函數"""
    import argparse
    
    parser = argparse.ArgumentParser(description='XC100超高速動作模組 - 調試版本')
    parser.add_argument('--config', type=str, help='配置文件路徑')
    parser.add_argument('--port', type=str, help='XC100串口號')
    parser.add_argument('--baudrate', type=int, help='波特率')
    parser.add_argument('--unit-id', type=int, help='站號')
    args = parser.parse_args()
    
    print("XC100超高速動作模組 - 調試版本")
    print("=" * 50)
    
    module = UltraFastXCModule(args.config)
    
    # 命令行參數覆蓋
    if args.port:
        module.config["xc_connection"]["port"] = args.port
        print(f"[參數覆蓋] 串口: {args.port}")
    if args.baudrate:
        module.config["xc_connection"]["baudrate"] = args.baudrate
        print(f"[參數覆蓋] 波特率: {args.baudrate}")
    if args.unit_id:
        module.config["xc_connection"]["unit_id"] = args.unit_id
        print(f"[參數覆蓋] 站號: {args.unit_id}")
    
    if args.port or args.baudrate or args.unit_id:
        module.save_config()
        print(f"[參數覆蓋] 配置已保存")
    
    try:
        if module.start():
            print(f"\n模組運行中: {module.module_id}")
            print("按 Ctrl+C 停止")
            
            while True:
                status = module.get_status()
                print(f"\r[狀態] {status['current_state']} | "
                      f"TCP: {'✅' if status['tcp_connected'] else '❌'} | "
                      f"XC100: {'✅' if status['xc_connected'] else '❌'} | "
                      f"執行中: {'是' if status['command_executing'] else '否'} | "
                      f"位置: {status['current_position']}", end="")
                time.sleep(1)  # 1秒更新狀態顯示
        else:
            print("❌ 模組啟動失敗")
            
    except KeyboardInterrupt:
        print("\n\n正在停止超高速模組...")
        module.stop()
        print("✅ 已停止")

if __name__ == "__main__":
    main()