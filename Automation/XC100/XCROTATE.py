# -*- coding: utf-8 -*-
"""
XCRotateAPI.py - 整合XC100/旋轉平台的高級API
整合COM5上的兩個設備：XC100升降(slave=2)、旋轉平台(slave=3)
"""

import time
import logging
import threading
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
from pymodbus.client import ModbusSerialClient

# 設置logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 固定配置常數
COM_PORT = "COM5"
BAUDRATE = 19200
XC100_SLAVE_ID = 2
ROTATE_SLAVE_ID = 3

# XC100位置定義
XC100_POSITION_A = 400  # A點高度
XC100_POSITION_B = 3200  # B點高度

class OperationResult(Enum):
    """操作結果枚舉"""
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"
    NOT_READY = "NOT_READY"
    CONNECTION_ERROR = "CONNECTION_ERROR"
    SYSTEM_ERROR = "SYSTEM_ERROR"

@dataclass
class XCRotateResult:
    """操作結果數據類"""
    result: OperationResult
    message: str
    device: Optional[str] = None
    execution_time: Optional[float] = None
    error_details: Optional[str] = None
    data: Optional[Dict[str, Any]] = None

# 全局串口管理器
_serial_lock = threading.Lock()
_shared_client = None
_client_ref_count = 0

class XCRotateAPI:
    """XC100/旋轉平台整合高級API
    
    管理COM5上的兩個設備，提供統一的控制接口
    支援多處同時import使用，共享串口連接
    """
    
    def __init__(self, port: str = COM_PORT, baudrate: int = BAUDRATE, auto_connect: bool = False):
        """初始化XCRotateAPI
        
        Args:
            port: 串口號，默認COM5
            baudrate: 波特率，默認115200
            auto_connect: 是否自動連接，預設False
        """
        self.port = port
        self.baudrate = baudrate
        self.client = None
        self.connected = False
        self.using_shared_client = False
        
        # 設備狀態
        self.xc100_ready = False
        self.rotate_ready = False
        
        # 超時設定
        self.operation_timeout = 15.0
        self.status_check_interval = 0.5
        
        logger.info(f"XCRotateAPI初始化: {port}@{baudrate}")
        logger.info(f"設備配置: XC100(slave={XC100_SLAVE_ID}), 旋轉平台(slave={ROTATE_SLAVE_ID})")
        
        if auto_connect:
            self.connect()
    
    def connect(self, use_shared: bool = True) -> bool:
        """連接到COM5串口
        
        Args:
            use_shared: 是否使用共享連接，預設True
        
        Returns:
            bool: 連接成功返回True
        """
        global _shared_client, _client_ref_count
        
        try:
            logger.info(f"正在連接到 {self.port}...")
            
            if use_shared:
                with _serial_lock:
                    if _shared_client is None:
                        # 創建共享連接
                        _shared_client = ModbusSerialClient(
                            port=self.port,
                            baudrate=self.baudrate,
                            stopbits=1,
                            parity='N',
                            timeout=1.0
                        )
                        
                        if _shared_client.connect():
                            logger.info(f"創建共享串口連接: {self.port}@{self.baudrate}")
                        else:
                            _shared_client = None
                            logger.error(f"無法創建共享串口連接: {self.port}")
                            return False
                    
                    if _shared_client and _shared_client.connected:
                        self.client = _shared_client
                        self.using_shared_client = True
                        _client_ref_count += 1
                        self.connected = True
                        logger.info(f"使用共享串口連接 (引用計數: {_client_ref_count})")
                        
                        # 檢查各設備狀態
                        self._check_all_devices()
                        return True
            else:
                # 創建獨立連接
                if self.client:
                    self.client.close()
                
                self.client = ModbusSerialClient(
                    port=self.port,
                    baudrate=self.baudrate,
                    stopbits=1,
                    parity='N',
                    timeout=1.0
                )
                
                if self.client.connect():
                    self.connected = True
                    self.using_shared_client = False
                    logger.info(f"創建獨立串口連接: {self.port}@{self.baudrate}")
                    
                    # 檢查各設備狀態
                    self._check_all_devices()
                    return True
                else:
                    logger.error(f"無法創建獨立串口連接: {self.port}")
                    return False
                    
        except Exception as e:
            logger.error(f"連接串口失敗: {e}")
            return False
        
        return False
    
    def disconnect(self):
        """斷開串口連接"""
        global _shared_client, _client_ref_count
        
        if self.using_shared_client:
            with _serial_lock:
                _client_ref_count -= 1
                logger.info(f"釋放共享串口連接 (剩餘引用計數: {_client_ref_count})")
                
                if _client_ref_count <= 0:
                    # 最後一個使用者，關閉共享連接
                    if _shared_client:
                        _shared_client.close()
                        _shared_client = None
                    _client_ref_count = 0
                    logger.info("共享串口連接已關閉")
                
                self.client = None
                self.connected = False
                self.using_shared_client = False
        else:
            # 獨立連接
            if self.client:
                self.client.close()
                self.client = None
                self.connected = False
                logger.info("獨立串口連接已關閉")
    
    def is_connected(self) -> bool:
        """檢查連接狀態"""
        return self.connected and self.client is not None
    
    def safe_operation(self, operation_func, *args, **kwargs):
        """安全操作包裝器，確保線程安全
        
        Args:
            operation_func: 要執行的操作函數
            *args, **kwargs: 操作函數的參數
            
        Returns:
            操作函數的返回值
        """
        with _serial_lock:
            if not self.is_connected():
                return XCRotateResult(
                    result=OperationResult.CONNECTION_ERROR,
                    message="串口未連接"
                )
            
            try:
                return operation_func(*args, **kwargs)
            except Exception as e:
                logger.error(f"操作執行異常: {e}")
                return XCRotateResult(
                    result=OperationResult.SYSTEM_ERROR,
                    message="操作執行異常",
                    error_details=str(e)
                )
    
    @staticmethod
    def get_shared_connection_info() -> Dict[str, Any]:
        """獲取共享連接資訊
        
        Returns:
            Dict: 共享連接資訊
        """
        global _shared_client, _client_ref_count
        
        with _serial_lock:
            return {
                "has_shared_connection": _shared_client is not None,
                "shared_connected": _shared_client.connected if _shared_client else False,
                "reference_count": _client_ref_count,
                "port": COM_PORT,
                "baudrate": BAUDRATE
            }
    
    # =========================
    # XC100 升降控制方法
    # =========================
    
    def xc100_home(self) -> XCRotateResult:
        """XC100原點復歸
        
        Returns:
            XCRotateResult: 操作結果
        """
        return self.safe_operation(self._xc100_home_impl)
    
    def _xc100_home_impl(self) -> XCRotateResult:
        """XC100原點復歸實現
        
        Returns:
            XCRotateResult: 操作結果
        """
        start_time = time.time()
        
        try:
            logger.info("執行XC100原點復歸...")
            
            # 發送原點復歸指令 (201E=3)
            result = self.client.write_register(
                address=0x201E, value=3, slave=XC100_SLAVE_ID
            )
            
            if result.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="XC100原點復歸指令發送失敗",
                    device="XC100",
                    execution_time=time.time() - start_time
                )
            
            logger.info("XC100原點復歸指令已發送")
            return XCRotateResult(
                result=OperationResult.SUCCESS,
                message="XC100原點復歸指令發送成功",
                device="XC100",
                execution_time=time.time() - start_time
            )
            
        except Exception as e:
            return XCRotateResult(
                result=OperationResult.SYSTEM_ERROR,
                message="XC100原點復歸異常",
                device="XC100",
                execution_time=time.time() - start_time,
                error_details=str(e)
            )
    
    def xc100_goto_a(self) -> XCRotateResult:
        """XC100移動到A點 (高度4000)
        
        Returns:
            XCRotateResult: 操作結果
        """
        return self.safe_operation(self._xc100_move_to_position, XC100_POSITION_A, "A點")
    
    def xc100_goto_b(self) -> XCRotateResult:
        """XC100移動到B點 (高度3200)
        
        Returns:
            XCRotateResult: 操作結果
        """
        return self.safe_operation(self._xc100_move_to_position, XC100_POSITION_B, "B點")
    
    def _xc100_move_to_position(self, position: int, point_name: str) -> XCRotateResult:
        """XC100移動到指定位置 (私有方法)"""
        start_time = time.time()
        
        try:
            logger.info(f"執行XC100移動到{point_name} (位置: {position})...")
            
            # 32位位置分解
            position_high = (position >> 16) & 0xFFFF
            position_low = position & 0xFFFF
            
            # 寫入目標位置 (0x2002)
            result1 = self.client.write_registers(
                address=0x2002, 
                values=[position_high, position_low], 
                slave=XC100_SLAVE_ID
            )
            
            if result1.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message=f"XC100寫入{point_name}位置失敗",
                    device="XC100",
                    execution_time=time.time() - start_time
                )
            
            time.sleep(0.1)  # 短暫延遲
            
            # 發送移動指令 (201E=1)
            result2 = self.client.write_register(
                address=0x201E, value=1, slave=XC100_SLAVE_ID
            )
            
            if result2.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message=f"XC100移動到{point_name}指令發送失敗",
                    device="XC100",
                    execution_time=time.time() - start_time
                )
            
            logger.info(f"XC100移動到{point_name}指令已發送")
            return XCRotateResult(
                result=OperationResult.SUCCESS,
                message=f"XC100移動到{point_name}指令發送成功",
                device="XC100",
                execution_time=time.time() - start_time,
                data={"position": position, "point": point_name}
            )
            
        except Exception as e:
            return XCRotateResult(
                result=OperationResult.SYSTEM_ERROR,
                message=f"XC100移動到{point_name}異常",
                device="XC100",
                execution_time=time.time() - start_time,
                error_details=str(e)
            )
    
    # =========================
    # 旋轉平台控制方法 (兼容AngleHighLevel)
    # =========================
    
    def adjust_to_90_degrees(self) -> XCRotateResult:
        """角度校正到90度 (兼容AngleHighLevel.adjust_to_90_degrees)
        
        Returns:
            XCRotateResult: 校正結果
        """
        return self.safe_operation(self._adjust_to_90_degrees_impl)
    
    def _adjust_to_90_degrees_impl(self) -> XCRotateResult:
        """角度校正到90度實現"""
        start_time = time.time()
        
        try:
            logger.info("執行角度校正到90度...")
            
            # 1. 檢查並準備旋轉平台
            prepare_result = self._clear_rotate_alarm_and_prepare()
            if prepare_result.result != OperationResult.SUCCESS:
                return prepare_result
            
            # 2. 發送角度校正指令到旋轉平台
            # 根據Tool.py，發送移動指令 (寄存器125, 值8)
            result = self.client.write_register(
                address=125, value=8, slave=ROTATE_SLAVE_ID
            )
            
            if result.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="角度校正指令發送失敗",
                    device="旋轉平台",
                    execution_time=time.time() - start_time
                )
            
            logger.info("角度校正指令已發送")
            return XCRotateResult(
                result=OperationResult.SUCCESS,
                message="角度校正指令發送成功",
                device="旋轉平台",
                execution_time=time.time() - start_time
            )
            
        except Exception as e:
            return XCRotateResult(
                result=OperationResult.SYSTEM_ERROR,
                message="角度校正異常",
                device="旋轉平台",
                execution_time=time.time() - start_time,
                error_details=str(e)
            )
    
    def rotate_move_to_position(self, position: int) -> XCRotateResult:
        """旋轉平台移動到指定位置
        
        Args:
            position: 目標位置
            
        Returns:
            XCRotateResult: 操作結果
        """
        return self.safe_operation(self._rotate_move_to_position_impl, position)
    
    def _rotate_move_to_position_impl(self, position: int) -> XCRotateResult:
        """旋轉平台移動到指定位置實現"""
        start_time = time.time()
        
        try:
            logger.info(f"執行旋轉平台移動到位置: {position}")
            
            # 1. 檢查並準備旋轉平台
            prepare_result = self._clear_rotate_alarm_and_prepare()
            if prepare_result.result != OperationResult.SUCCESS:
                return prepare_result
            
            # 2. 設置目標位置 (參考Tool.py的寄存器6147)
            result1 = self.client.write_register(
                address=6147, value=position, slave=ROTATE_SLAVE_ID
            )
            
            if result1.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="旋轉平台設置位置失敗",
                    device="旋轉平台",
                    execution_time=time.time() - start_time
                )
            
            time.sleep(0.1)
            
            # 3. 發送移動指令 (寄存器125, 值8)
            result2 = self.client.write_register(
                address=125, value=8, slave=ROTATE_SLAVE_ID
            )
            
            if result2.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="旋轉平台移動指令發送失敗",
                    device="旋轉平台",
                    execution_time=time.time() - start_time
                )
            
            logger.info(f"旋轉平台移動到位置{position}指令已發送")
            return XCRotateResult(
                result=OperationResult.SUCCESS,
                message=f"旋轉平台移動到位置{position}指令發送成功",
                device="旋轉平台",
                execution_time=time.time() - start_time,
                data={"position": position}
            )
            
        except Exception as e:
            return XCRotateResult(
                result=OperationResult.SYSTEM_ERROR,
                message="旋轉平台移動異常",
                device="旋轉平台",
                execution_time=time.time() - start_time,
                error_details=str(e)
            )
    
    def rotate_home(self) -> XCRotateResult:
        """旋轉平台回原點
        
        Returns:
            XCRotateResult: 操作結果
        """
        return self.safe_operation(self._rotate_home_impl)
    
    def rotate_relative_steps(self, steps: int) -> XCRotateResult:
        """旋轉平台相對移動指定步數 (正轉)
        
        Args:
            steps: 相對步數 (正值為正轉)
            
        Returns:
            XCRotateResult: 操作結果
        """
        return self.safe_operation(self._rotate_relative_steps_impl, steps)
    
    def _rotate_home_impl(self) -> XCRotateResult:
        """旋轉平台回原點實現"""
        start_time = time.time()
        
        try:
            logger.info("執行旋轉平台回原點...")
            
            # 1. 檢查並準備旋轉平台
            prepare_result = self._clear_rotate_alarm_and_prepare()
            if prepare_result.result != OperationResult.SUCCESS:
                return prepare_result
            
            # 2. 發送回原點指令 (寄存器125, 值16)
            result = self.client.write_register(
                address=125, value=16, slave=ROTATE_SLAVE_ID
            )
            
            if result.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="旋轉平台回原點指令發送失敗",
                    device="旋轉平台",
                    execution_time=time.time() - start_time
                )
            
            logger.info("旋轉平台回原點指令已發送")
            return XCRotateResult(
                result=OperationResult.SUCCESS,
                message="旋轉平台回原點指令發送成功",
                device="旋轉平台",
                execution_time=time.time() - start_time
            )
            
        except Exception as e:
            return XCRotateResult(
                result=OperationResult.SYSTEM_ERROR,
                message="旋轉平台回原點異常",
                device="旋轉平台",
                execution_time=time.time() - start_time,
                error_details=str(e)
            )
    
    def _rotate_relative_steps_impl(self, steps: int) -> XCRotateResult:
        """旋轉平台相對移動指定步數實現"""
        start_time = time.time()
        
        try:
            logger.info(f"執行旋轉平台相對移動: +{steps} 步")
            
            # 1. 檢查並準備旋轉平台
            prepare_result = self._clear_rotate_alarm_and_prepare()
            if prepare_result.result != OperationResult.SUCCESS:
                return prepare_result
            
            # 2. 讀取當前位置 (假設從寄存器6147讀取，需要您確認正確的寄存器)
            current_pos_result = self.client.read_holding_registers(
                address=6147, count=1, slave=ROTATE_SLAVE_ID
            )
            
            if current_pos_result.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="無法讀取當前位置",
                    device="旋轉平台",
                    execution_time=time.time() - start_time
                )
            
            current_position = current_pos_result.registers[0]
            target_position = current_position + steps
            
            logger.info(f"當前位置: {current_position}, 目標位置: {target_position}")
            
            # 3. 設置目標位置
            result1 = self.client.write_register(
                address=6147, value=target_position, slave=ROTATE_SLAVE_ID
            )
            
            if result1.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="設置目標位置失敗",
                    device="旋轉平台",
                    execution_time=time.time() - start_time
                )
            
            time.sleep(0.1)
            
            # 4. 發送移動指令 (寄存器125, 值8)
            result2 = self.client.write_register(
                address=125, value=8, slave=ROTATE_SLAVE_ID
            )
            
            if result2.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="相對移動指令發送失敗",
                    device="旋轉平台",
                    execution_time=time.time() - start_time
                )
            
            logger.info(f"旋轉平台相對移動 +{steps} 步指令已發送")
            return XCRotateResult(
                result=OperationResult.SUCCESS,
                message=f"旋轉平台相對移動 +{steps} 步指令發送成功",
                device="旋轉平台",
                execution_time=time.time() - start_time,
                data={
                    "steps": steps,
                    "current_position": current_position,
                    "target_position": target_position
                }
            )
            
        except Exception as e:
            return XCRotateResult(
                result=OperationResult.SYSTEM_ERROR,
                message="旋轉平台相對移動異常",
                device="旋轉平台",
                execution_time=time.time() - start_time,
                error_details=str(e)
            )
    
    # =========================
    # 狀態檢查方法
    # =========================
    
    def _check_all_devices(self):
        """檢查所有設備狀態"""
        logger.info("檢查所有設備狀態...")
        self.xc100_ready = self._check_xc100_ready()
        self.rotate_ready = self._check_rotate_ready()
        
        logger.info(f"設備狀態 - XC100: {'就緒' if self.xc100_ready else '未就緒'}, "
                   f"旋轉平台: {'就緒' if self.rotate_ready else '未就緒'}")
    
    def _check_xc100_ready(self) -> bool:
        """檢查XC100狀態"""
        try:
            result = self.client.read_holding_registers(
                address=0x100C, count=1, slave=XC100_SLAVE_ID
            )
            return not result.isError()
        except:
            return False
    
    def _check_rotate_ready(self) -> bool:
        """檢查旋轉平台狀態"""
        try:
            result = self.client.read_holding_registers(
                address=127, count=1, slave=ROTATE_SLAVE_ID
            )
            if not result.isError():
                status_word = result.registers[0]
                ready = bool(status_word & (1 << 5))    # bit 5: 準備就緒
                alarm = bool(status_word & (1 << 7))    # bit 7: 警報狀態
                return ready and not alarm
            return False
        except:
            return False
    
    def _get_rotate_status(self) -> Dict[str, Any]:
        """獲取旋轉平台詳細狀態"""
        try:
            result = self.client.read_holding_registers(
                address=127, count=1, slave=ROTATE_SLAVE_ID
            )
            if not result.isError():
                status_word = result.registers[0]
                return {
                    "moving": bool(status_word & (1 << 13)),    # bit 13: 運動中
                    "home": bool(status_word & (1 << 4)),       # bit 4: 在原點  
                    "ready": bool(status_word & (1 << 5)),      # bit 5: 準備就緒
                    "alarm": bool(status_word & (1 << 7)),      # bit 7: 警報狀態
                    "status_word": status_word
                }
            return {"error": "讀取狀態失敗"}
        except Exception as e:
            return {"error": f"讀取狀態異常: {e}"}
    
    def _clear_rotate_alarm_and_prepare(self) -> XCRotateResult:
        """清除旋轉平台警報並準備設備"""
        try:
            logger.info("檢查旋轉平台狀態並清除警報...")
            
            # 1. 讀取當前狀態
            status = self._get_rotate_status()
            if "error" in status:
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message=f"無法讀取旋轉平台狀態: {status['error']}",
                    device="旋轉平台"
                )
            
            logger.info(f"旋轉平台狀態: Ready={status['ready']}, Alarm={status['alarm']}, Moving={status['moving']}")
            
            # 2. 如果有警報，先清除警報
            if status['alarm']:
                logger.info("檢測到警報狀態，執行警報重置...")
                
                # 發送ALM-RST指令 (bit7 = 1, 即值為 128)
                result = self.client.write_register(
                    address=125, value=128, slave=ROTATE_SLAVE_ID
                )
                
                if result.isError():
                    return XCRotateResult(
                        result=OperationResult.FAILED,
                        message="發送警報重置指令失敗",
                        device="旋轉平台"
                    )
                
                time.sleep(0.2)  # 等待警報重置處理
                
                # 清除指令寄存器
                self.client.write_register(
                    address=125, value=0, slave=ROTATE_SLAVE_ID
                )
                
                time.sleep(0.2)  # 等待指令清除
                logger.info("警報重置完成")
            
            # 3. 清空控制寄存器確保Ready狀態
            logger.info("清空控制寄存器...")
            result = self.client.write_register(
                address=125, value=0, slave=ROTATE_SLAVE_ID
            )
            
            if result.isError():
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="清空控制寄存器失敗",
                    device="旋轉平台"
                )
            
            time.sleep(0.2)  # 等待狀態更新
            
            # 4. 再次檢查狀態
            final_status = self._get_rotate_status()
            if "error" not in final_status:
                logger.info(f"最終狀態: Ready={final_status['ready']}, Alarm={final_status['alarm']}")
                
                if final_status['ready'] and not final_status['alarm']:
                    return XCRotateResult(
                        result=OperationResult.SUCCESS,
                        message="旋轉平台已準備就緒",
                        device="旋轉平台",
                        data=final_status
                    )
                else:
                    return XCRotateResult(
                        result=OperationResult.NOT_READY,
                        message=f"旋轉平台仍未就緒 (Ready={final_status['ready']}, Alarm={final_status['alarm']})",
                        device="旋轉平台",
                        data=final_status
                    )
            else:
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message="無法確認最終狀態",
                    device="旋轉平台"
                )
                
        except Exception as e:
            return XCRotateResult(
                result=OperationResult.SYSTEM_ERROR,
                message="清除警報過程異常",
                device="旋轉平台",
                error_details=str(e)
            )
    
    def get_rotate_status(self) -> XCRotateResult:
        """獲取旋轉平台狀態 (公開方法)
        
        Returns:
            XCRotateResult: 包含狀態資訊的結果
        """
        return self.safe_operation(self._get_rotate_status_public)
    
    def _get_rotate_status_public(self) -> XCRotateResult:
        """獲取旋轉平台狀態的公開實現"""
        try:
            status = self._get_rotate_status()
            if "error" in status:
                return XCRotateResult(
                    result=OperationResult.FAILED,
                    message=f"讀取旋轉平台狀態失敗: {status['error']}",
                    device="旋轉平台"
                )
            
            return XCRotateResult(
                result=OperationResult.SUCCESS,
                message="旋轉平台狀態讀取成功",
                device="旋轉平台",
                data=status
            )
        except Exception as e:
            return XCRotateResult(
                result=OperationResult.SYSTEM_ERROR,
                message="讀取旋轉平台狀態異常",
                device="旋轉平台",
                error_details=str(e)
            )
    
    def reset_rotate_alarm(self) -> XCRotateResult:
        """手動重置旋轉平台警報 (公開方法)
        
        Returns:
            XCRotateResult: 重置結果
        """
        return self.safe_operation(self._clear_rotate_alarm_and_prepare)
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態
        
        Returns:
            Dict: 系統狀態字典
        """
        if not self.is_connected():
            return {"connected": False, "error": "串口未連接"}
        
        self._check_all_devices()
        
        return {
            "connected": True,
            "port": self.port,
            "baudrate": self.baudrate,
            "devices": {
                "xc100": {
                    "slave_id": XC100_SLAVE_ID,
                    "ready": self.xc100_ready,
                    "position_a": XC100_POSITION_A,
                    "position_b": XC100_POSITION_B
                },
                "rotate_platform": {
                    "slave_id": ROTATE_SLAVE_ID,
                    "ready": self.rotate_ready
                }
            }
        }

# =========================
# 便利函數和單例模式支援
# =========================

# 全局單例實例
_global_api_instance = None
_instance_lock = threading.Lock()

def get_global_api() -> XCRotateAPI:
    """獲取全局API實例 (單例模式)
    
    多個地方import時可以安全使用同一個實例
    
    Returns:
        XCRotateAPI: 全局API實例
    """
    global _global_api_instance
    
    with _instance_lock:
        if _global_api_instance is None:
            _global_api_instance = XCRotateAPI(auto_connect=True)
            if not _global_api_instance.is_connected():
                logger.warning("全局API實例連接失敗")
        
        return _global_api_instance

def release_global_api():
    """釋放全局API實例"""
    global _global_api_instance
    
    with _instance_lock:
        if _global_api_instance:
            _global_api_instance.disconnect()
            _global_api_instance = None

def create_xc_rotate_api(port: str = COM_PORT, baudrate: int = BAUDRATE, auto_connect: bool = True) -> Optional[XCRotateAPI]:
    """創建XCRotateAPI實例
    
    Args:
        port: 串口號
        baudrate: 波特率
        auto_connect: 是否自動連接
        
    Returns:
        XCRotateAPI: API實例，連接失敗時返回None
    """
    api = XCRotateAPI(port, baudrate, auto_connect=auto_connect)
    if auto_connect and not api.is_connected():
        logger.error("創建XCRotateAPI失敗")
        return None
    return api

# 使用全局實例的快速操作函數
def quick_xc100_home_global() -> XCRotateResult:
    """使用全局實例的快速XC100原點復歸"""
    return get_global_api().xc100_home()

def quick_xc100_goto_a_global() -> XCRotateResult:
    """使用全局實例的快速XC100移動到A點"""
    return get_global_api().xc100_goto_a()

def quick_xc100_goto_b_global() -> XCRotateResult:
    """使用全局實例的快速XC100移動到B點"""
    return get_global_api().xc100_goto_b()

def quick_adjust_to_90_degrees_global() -> XCRotateResult:
    """使用全局實例的快速角度校正到90度"""
    return get_global_api().adjust_to_90_degrees()

def quick_rotate_relative_steps_global(steps: int) -> XCRotateResult:
    """使用全局實例的快速相對移動步數"""
    return get_global_api().rotate_relative_steps(steps)

# 獨立連接的快速操作函數 (兼容性保留)
def quick_xc100_home(port: str = COM_PORT) -> XCRotateResult:
    """快速XC100原點復歸"""
    api = create_xc_rotate_api(port)
    if api:
        try:
            return api.xc100_home()
        finally:
            api.disconnect()
    return XCRotateResult(OperationResult.CONNECTION_ERROR, "無法連接到設備")

def quick_xc100_goto_a(port: str = COM_PORT) -> XCRotateResult:
    """快速XC100移動到A點"""
    api = create_xc_rotate_api(port)
    if api:
        try:
            return api.xc100_goto_a()
        finally:
            api.disconnect()
    return XCRotateResult(OperationResult.CONNECTION_ERROR, "無法連接到設備")

def quick_xc100_goto_b(port: str = COM_PORT) -> XCRotateResult:
    """快速XC100移動到B點"""
    api = create_xc_rotate_api(port)
    if api:
        try:
            return api.xc100_goto_b()
        finally:
            api.disconnect()
    return XCRotateResult(OperationResult.CONNECTION_ERROR, "無法連接到設備")

def quick_adjust_to_90_degrees(port: str = COM_PORT) -> XCRotateResult:
    """快速角度校正到90度 (兼容AngleHighLevel)"""
    api = create_xc_rotate_api(port)
    if api:
        try:
            return api.adjust_to_90_degrees()
        finally:
            api.disconnect()
    return XCRotateResult(OperationResult.CONNECTION_ERROR, "無法連接到設備")

def quick_rotate_relative_steps(steps: int, port: str = COM_PORT) -> XCRotateResult:
    """快速相對移動步數"""
    api = create_xc_rotate_api(port)
    if api:
        try:
            return api.rotate_relative_steps(steps)
        finally:
            api.disconnect()
    return XCRotateResult(OperationResult.CONNECTION_ERROR, "無法連接到設備")

# =========================
# 使用範例
# =========================

# if __name__ == '__main__':
#     print("XCRotateAPI 測試程序")
#     print("=" * 50)
    
#     # 創建API實例
#     api = XCRotateAPI()
    
#     if api.connect():
#         print("✓ 串口連接成功")
        
#         # 顯示系統狀態
#         status = api.get_system_status()
#         print("\n系統狀態:")
#         print(f"  XC100: {'就緒' if status['devices']['xc100']['ready'] else '未就緒'}")
#         print(f"  旋轉平台: {'就緒' if status['devices']['rotate_platform']['ready'] else '未就緒'}")
        
#         # 測試各設備功能
#         print("\n=== 測試XC100功能 ===")
#         result = api.xc100_home()
#         print(f"原點復歸: {result.result.value} - {result.message}")
        
#         time.sleep(2)
#         result = api.xc100_goto_a()
#         print(f"移動到A點: {result.result.value} - {result.message}")
        
#         time.sleep(2)
#         result = api.xc100_goto_b()
#         print(f"移動到B點: {result.result.value} - {result.message}")
        
#         print("\n=== 測試旋轉平台功能 ===")
#         result = api.adjust_to_90_degrees()
#         print(f"角度校正: {result.result.value} - {result.message}")
        
#         time.sleep(2)
#         result = api.rotate_home()
#         print(f"旋轉平台回原點: {result.result.value} - {result.message}")
        
#         time.sleep(2)
#         print("\n=== 測試相對移動步數 ===")
#         result = api.rotate_relative_steps(9000)
#         print(f"相對移動 +100 步: {result.result.value} - {result.message}")
#         if result.data:
#             print(f"  詳細資訊: {result.data}")
        
#         time.sleep(2)
#         result = api.rotate_relative_steps(50)
#         print(f"相對移動 +50 步: {result.result.value} - {result.message}")
#         if result.data:
#             print(f"  詳細資訊: {result.data}")
        
#         api.disconnect()
#     else:
#         print("✗ 串口連接失敗")
    
#     print("\n=== 測試多實例使用 ===")
    
#     # 測試1: 使用全局實例
#     print("測試1: 使用全局實例")
#     api1 = get_global_api()
#     print(f"API1連接狀態: {api1.is_connected()}")
    
#     # 測試2: 創建第二個實例使用共享連接
#     print("\n測試2: 創建第二個實例")
#     api2 = XCRotateAPI(auto_connect=True)
#     print(f"API2連接狀態: {api2.is_connected()}")
    
#     # 顯示共享連接資訊
#     conn_info = XCRotateAPI.get_shared_connection_info()
#     print(f"共享連接資訊: {conn_info}")
    
#     # 測試3: 同時使用兩個實例
#     print("\n測試3: 同時操作")
#     result1 = api1.xc100_home()
#     print(f"API1操作: {result1.result.value}")
    
#     time.sleep(1)
#     result2 = api2.adjust_to_90_degrees()
#     print(f"API2操作: {result2.result.value}")
    
#     # 測試4: 使用快速函數
#     print("\n測試4: 快速函數")
#     result = quick_adjust_to_90_degrees_global()
#     print(f"快速角度校正: {result.result.value}")
    
#     time.sleep(1)
#     result = quick_rotate_relative_steps_global(30)
#     print(f"快速相對移動: {result.result.value}")
#     if result.data:
#         print(f"  詳細資訊: {result.data}")
    
#     # 清理
#     api2.disconnect()
#     release_global_api()
    
#     print("\n共享連接測試完成")