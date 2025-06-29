import time
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

# PyModbus imports
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# 設置logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AngleOperationResult(Enum):
    """角度操作結果枚舉"""
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"
    NOT_READY = "NOT_READY"
    CONNECTION_ERROR = "CONNECTION_ERROR"
    SYSTEM_ERROR = "SYSTEM_ERROR"
    NO_VALID_CONTOUR = "NO_VALID_CONTOUR"

@dataclass
class AngleDetectionResult:
    """角度檢測結果數據類"""
    result: AngleOperationResult
    message: str
    target_angle: Optional[float] = None
    detected_center: Optional[tuple] = None
    contour_area: Optional[float] = None
    execution_time: Optional[float] = None
    error_details: Optional[str] = None

class AngleHighLevel:
    """角度檢測系統高階API - CASE模式版本
    
    提供簡潔的方法供Flow流程調用，專注與CCD3模組交握獲取角度檢測結果
    參考DR版本修正交握狀態機邏輯，實現完整的狀態控制
    """
    
    def __init__(self, host: str = "127.0.0.1", port: int = 502):
        """初始化角度檢測高階API
        
        Args:
            host: Modbus服務器IP
            port: Modbus服務器端口
        """
        self.host = host
        self.port = port
        self.ccd3_base_address = 800  # CCD3模組基地址
        self.modbus_client = None
        self.timeout = 3.0
        
        # 操作超時設定
        self.detection_timeout = 10.0  # 角度檢測總超時10秒
        self.status_check_interval = 0.2  # 狀態檢查間隔200ms
        self.command_confirm_timeout = 3.0  # 指令確認超時3秒
        
        logger.info(f"AngleHighLevel初始化 (CASE模式): {host}:{port}, CCD3基地址:{self.ccd3_base_address}")
    
    def connect(self) -> bool:
        """連接到Modbus服務器
        
        Returns:
            bool: 連接成功返回True
        """
        try:
            logger.info("正在連接Modbus服務器...")
            
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.host,
                port=self.port,
                timeout=self.timeout
            )
            
            if self.modbus_client.connect():
                # 驗證CCD3模組回應
                ccd3_status = self._read_ccd3_status()
                if ccd3_status:
                    logger.info(f"CCD3模組連接成功 - Ready:{ccd3_status.get('ready')}, Initialized:{ccd3_status.get('initialized')}")
                    return True
                else:
                    logger.error("CCD3模組無回應")
                    return False
            else:
                logger.error(f"無法連接到Modbus服務器: {self.host}:{self.port}")
                return False
                
        except Exception as e:
            logger.error(f"連接Modbus服務器失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client:
            self.modbus_client.close()
            self.modbus_client = None
            logger.info("Modbus連接已斷開")
    
    def is_ccd3_ready(self) -> bool:
        """檢查CCD3系統是否準備就緒 - 修正版：檢查狀態值是否為9
        
        Returns:
            bool: CCD3系統狀態值=9時返回True
        """
        status = self._read_ccd3_status()
        if not status:
            return False
        
        status_register = status.get('status_register', 0)
        ready = status.get('ready', False)
        alarm = status.get('alarm', False)
        initialized = status.get('initialized', False)
        
        logger.debug(f"CCD3系統狀態檢查: status_register={status_register}, Ready={ready}, Alarm={alarm}, Initialized={initialized}")
        
        # 期望狀態值=9 (Ready=1 + Initialized=1)
        if status_register == 9:
            logger.debug("CCD3系統完全準備就緒 (狀態值=9)")
            return True
        elif ready and initialized and not alarm:
            logger.debug(f"CCD3系統基本準備就緒 (狀態值={status_register})")
            return True
        else:
            logger.debug(f"CCD3系統未準備就緒 (狀態值={status_register})")
            return False
    
    def detect_angle(self, detection_mode: int = 0) -> AngleDetectionResult:
        """執行角度檢測 - CASE模式版本
        
        這是主要的公開方法，供Flow流程調用
        與CCD3模組交握，獲取角度檢測結果
        參考DR版本的完整交握流程
        
        Args:
            detection_mode: 檢測模式 (0=CASE橢圓擬合, 1=DR最小外接矩形)
            
        Returns:
            AngleDetectionResult: 包含檢測結果的完整資訊
        """
        start_time = time.time()
        
        try:
            logger.info(f"=== 開始執行角度檢測 (CASE模式:{detection_mode}) ===")
            
            # 步驟1: 檢查連接狀態
            if not self.modbus_client or not self.modbus_client.connected:
                return AngleDetectionResult(
                    result=AngleOperationResult.CONNECTION_ERROR,
                    message="Modbus連接未建立，請先調用connect()"
                )
            
            # 步驟2: 檢查CCD3系統狀態 (801是否為9) - 參考DR版本
            logger.info("檢查CCD3系統準備狀態 (期望801=9)...")
            if not self.is_ccd3_ready():
                return AngleDetectionResult(
                    result=AngleOperationResult.NOT_READY,
                    message="CCD3角度檢測系統未準備就緒，請檢查系統狀態"
                )
            
            # 步驟3: 設置檢測模式
            logger.info(f"設置檢測模式: {detection_mode}")
            if not self._set_detection_mode(detection_mode):
                return AngleDetectionResult(
                    result=AngleOperationResult.FAILED,
                    message="設置檢測模式失敗"
                )
            
            # 步驟4: 向800寫入16 - 參考DR版本的直接發送方式
            logger.info("向800寫入指令16...")
            if not self._send_detection_command_direct():
                return AngleDetectionResult(
                    result=AngleOperationResult.FAILED,
                    message="發送角度檢測指令失敗"
                )
            
            # 步驟5: 檢查801是否變為8 - 參考DR版本
            logger.info("等待801狀態變為8...")
            completion_result = self._wait_for_status_8()
            if completion_result.result != AngleOperationResult.SUCCESS:
                return completion_result
            
            # 步驟6: 檢查840是否為1，如果是則提取結果 - 參考DR版本
            logger.info("檢查840結果標誌並讀取檢測結果...")
            result_data = self._read_detection_results()
            
            # 步驟7: 向800及840寫入0 - 參考DR版本
            logger.info("清除800控制指令和840結果標誌...")
            clear_result = self._clear_command_and_result_flags()
            if clear_result.result != AngleOperationResult.SUCCESS:
                return clear_result
            
            # 步驟8: 檢查801是否變回9 - 參考DR版本
            logger.info("確認801狀態回到9...")
            ready_result = self._wait_for_status_9()
            if ready_result.result != AngleOperationResult.SUCCESS:
                return ready_result
            
            # 處理檢測結果
            execution_time = time.time() - start_time
            
            if result_data and result_data.get('success', False):
                logger.info(f"CASE角度檢測成功完成，耗時: {execution_time:.2f}秒")
                logger.info(f"檢測中心: {result_data.get('center')}")
                logger.info(f"檢測角度: {result_data.get('angle'):.2f}度")
                logger.info(f"輪廓面積: {result_data.get('contour_area')}")
                
                return AngleDetectionResult(
                    result=AngleOperationResult.SUCCESS,
                    message="CASE角度檢測完成",
                    target_angle=result_data.get('angle'),
                    detected_center=result_data.get('center'),
                    contour_area=result_data.get('contour_area'),
                    execution_time=execution_time
                )
            else:
                return AngleDetectionResult(
                    result=AngleOperationResult.NO_VALID_CONTOUR,
                    message="CASE角度檢測失敗，無有效輪廓",
                    execution_time=execution_time
                )
            
        except Exception as e:
            execution_time = time.time() - start_time
            logger.error(f"CASE角度檢測過程發生異常: {e}")
            return AngleDetectionResult(
                result=AngleOperationResult.SYSTEM_ERROR,
                message="CASE角度檢測系統異常",
                execution_time=execution_time,
                error_details=str(e)
            )
    
    def reset_ccd3_errors(self) -> AngleOperationResult:
        """重置CCD3錯誤狀態
        
        Returns:
            AngleOperationResult: 重置結果
        """
        try:
            logger.info("執行CCD3錯誤重置...")
            
            # 發送重新初始化指令 (32)
            result = self.modbus_client.write_register(
                address=self.ccd3_base_address, value=32, slave=1
            )
            
            if result.isError():
                logger.error("CCD3錯誤重置指令發送失敗")
                return AngleOperationResult.FAILED
            
            # 等待指令處理
            time.sleep(1.0)
            
            # 清除指令
            self.modbus_client.write_register(
                address=self.ccd3_base_address, value=0, slave=1
            )
            
            logger.info("CCD3錯誤重置完成")
            return AngleOperationResult.SUCCESS
            
        except Exception as e:
            logger.error(f"CCD3錯誤重置異常: {e}")
            return AngleOperationResult.SYSTEM_ERROR
    
    def get_ccd3_status(self) -> Optional[Dict[str, Any]]:
        """獲取CCD3系統狀態資訊
        
        Returns:
            Dict: CCD3系統狀態字典，包含詳細狀態資訊
        """
        return self._read_ccd3_status()
    
    def get_last_detection_result(self) -> Optional[Dict[str, Any]]:
        """獲取最後一次角度檢測結果
        
        Returns:
            Dict: 檢測結果字典
        """
        return self._read_detection_results()
    
    def _set_detection_mode(self, mode: int) -> bool:
        """設置檢測模式 (私有方法)"""
        try:
            # 寫入檢測模式到寄存器810
            result = self.modbus_client.write_register(
                address=self.ccd3_base_address + 10, value=mode, slave=1
            )
            return not result.isError()
        except Exception as e:
            logger.error(f"設置檢測模式異常: {e}")
            return False
    
    def _send_detection_command_direct(self) -> bool:
        """直接發送角度檢測指令 (私有方法) - 參考DR版本"""
        try:
            # 直接發送拍照+角度檢測指令 (16) 到寄存器800
            result = self.modbus_client.write_register(
                address=self.ccd3_base_address, value=16, slave=1
            )
            
            if result.isError():
                logger.error("發送檢測指令失敗")
                return False
                
            logger.debug("檢測指令16已發送到800寄存器")
            return True
            
        except Exception as e:
            logger.error(f"發送角度檢測指令異常: {e}")
            return False
    
    def _wait_for_status_8(self) -> AngleDetectionResult:
        """等待801狀態變為8 (私有方法) - 參考DR版本"""
        start_time = time.time()
        
        logger.debug("監控801寄存器，等待狀態值變為8...")
        
        while time.time() - start_time < self.detection_timeout:
            try:
                status = self._read_ccd3_status()
                if not status:
                    time.sleep(self.status_check_interval)
                    continue
                
                status_register = status.get('status_register', 0)
                alarm = status.get('alarm', False)
                
                logger.debug(f"CCD3執行狀態監控: status_register={status_register}")
                
                # 檢查是否有錯誤
                if alarm:
                    return AngleDetectionResult(
                        result=AngleOperationResult.FAILED,
                        message="CCD3檢測過程發生錯誤，系統進入Alarm狀態"
                    )
                
                # 檢查是否達到狀態值8
                if status_register == 8:
                    logger.info("CCD3檢測完成，801狀態值=8")
                    return AngleDetectionResult(
                        result=AngleOperationResult.SUCCESS,
                        message="CCD3檢測執行完成"
                    )
                
                time.sleep(self.status_check_interval)
                
            except Exception as e:
                logger.error(f"CCD3狀態檢查異常: {e}")
                time.sleep(self.status_check_interval)
        
        logger.error(f"等待801狀態變為8超時 ({self.detection_timeout}秒)")
        return AngleDetectionResult(
            result=AngleOperationResult.TIMEOUT,
            message=f"等待CCD3檢測完成超時 ({self.detection_timeout}秒)"
        )
    
    def _clear_command_and_result_flags(self) -> AngleDetectionResult:
        """清除800控制指令和840結果標誌 (私有方法) - 參考DR版本"""
        try:
            # 清除控制指令 (寫入0到寄存器800)
            logger.debug("清除800控制指令...")
            result = self.modbus_client.write_register(
                address=self.ccd3_base_address, value=0, slave=1
            )
            if result.isError():
                return AngleDetectionResult(
                    result=AngleOperationResult.FAILED,
                    message="清除CCD3控制指令失敗"
                )
            
            # 清除檢測結果標誌 (寫入0到寄存器840)
            logger.debug("清除840檢測結果標誌...")
            result = self.modbus_client.write_register(
                address=self.ccd3_base_address + 40, value=0, slave=1
            )
            if result.isError():
                return AngleDetectionResult(
                    result=AngleOperationResult.FAILED,
                    message="清除CCD3檢測結果標誌失敗"
                )
            
            logger.debug("800和840寄存器已清零")
            return AngleDetectionResult(
                result=AngleOperationResult.SUCCESS,
                message="指令和結果標誌清除成功"
            )
            
        except Exception as e:
            logger.error(f"清除指令和結果標誌異常: {e}")
            return AngleDetectionResult(
                result=AngleOperationResult.SYSTEM_ERROR,
                message=f"清除指令和結果標誌異常: {e}"
            )
    
    def _wait_for_status_9(self) -> AngleDetectionResult:
        """等待801狀態變回9 (私有方法) - 參考DR版本"""
        start_time = time.time()
        
        logger.debug("監控801寄存器，等待狀態值變為9...")
        
        while time.time() - start_time < self.command_confirm_timeout:
            try:
                status = self._read_ccd3_status()
                if not status:
                    time.sleep(0.1)
                    continue
                
                status_register = status.get('status_register', 0)
                ready = status.get('ready', False)
                initialized = status.get('initialized', False)
                alarm = status.get('alarm', False)
                
                logger.debug(f"CCD3準備狀態監控: status_register={status_register}, Ready={ready}, Initialized={initialized}")
                
                if alarm:
                    return AngleDetectionResult(
                        result=AngleOperationResult.FAILED,
                        message="CCD3系統發生錯誤"
                    )
                
                # 檢查是否達到狀態值9 (Ready=1, Initialized=1)
                if status_register == 9 or (ready and initialized):
                    logger.info(f"CCD3系統回到Ready狀態，801狀態值={status_register}")
                    return AngleDetectionResult(
                        result=AngleOperationResult.SUCCESS,
                        message="CCD3系統Ready狀態確認"
                    )
                
                time.sleep(0.1)
                
            except Exception as e:
                logger.error(f"CCD3狀態檢查異常: {e}")
                time.sleep(0.1)
        
        logger.error(f"等待801狀態變為9超時 ({self.command_confirm_timeout}秒)")
        return AngleDetectionResult(
            result=AngleOperationResult.TIMEOUT,
            message="等待CCD3系統回到Ready狀態超時"
        )
    
    def _check_detection_result_flag(self) -> bool:
        """檢查840寄存器檢測成功標誌 (私有方法) - 參考DR版本新增"""
        try:
            # 讀取檢測結果標誌 (840)
            result = self.modbus_client.read_holding_registers(
                address=self.ccd3_base_address + 40, count=1, slave=1
            )
            
            if result.isError():
                logger.debug("讀取840寄存器失敗")
                return False
            
            success_flag = result.registers[0]
            logger.debug(f"840寄存器檢測成功標誌: {success_flag}")
            
            return success_flag == 1
            
        except Exception as e:
            logger.error(f"檢查840寄存器異常: {e}")
            return False
    
    def _read_ccd3_status(self) -> Optional[Dict[str, Any]]:
        """讀取CCD3系統狀態 (私有方法)"""
        try:
            # 讀取CCD3狀態寄存器 (801)
            result = self.modbus_client.read_holding_registers(
                address=self.ccd3_base_address + 1, count=1, slave=1
            )
            
            if result.isError():
                return None
            
            status_register = result.registers[0]
            
            return {
                'status_register': status_register,
                'ready': bool(status_register & (1 << 0)),
                'running': bool(status_register & (1 << 1)),
                'alarm': bool(status_register & (1 << 2)),
                'initialized': bool(status_register & (1 << 3))
            }
            
        except Exception as e:
            logger.error(f"讀取CCD3系統狀態異常: {e}")
            return None
    
    def _read_detection_results(self) -> Optional[Dict[str, Any]]:
        """讀取CCD3角度檢測結果 (私有方法) - 修正版：參考DR版本，讀取前檢查840標誌"""
        try:
            # 檢查840寄存器確認結果有效 - 參考DR版本新增
            logger.debug("檢查840寄存器結果標誌...")
            if not self._check_detection_result_flag():
                logger.warning("CASE角度檢測失敗: 840寄存器=0，無有效檢測結果")
                return {'success': False, 'error': '檢測結果標誌無效，840寄存器=0'}
            
            logger.debug("840寄存器=1，檢測結果有效，開始讀取...")
            
            # 讀取CCD3檢測結果寄存器 (840-849)
            result = self.modbus_client.read_holding_registers(
                address=self.ccd3_base_address + 40, count=10, slave=1
            )
            
            if result.isError():
                logger.error("讀取CCD3檢測結果寄存器失敗")
                return None
            
            registers = result.registers
            success = bool(registers[0])
            
            if not success:
                logger.warning("檢測結果顯示失敗 (840寄存器內容=0)")
                return {'success': False}
            
            # 解析檢測結果
            center_x = registers[1]
            center_y = registers[2]
            
            # 角度32位解析 (寄存器843-844)
            angle_high = registers[3]
            angle_low = registers[4]
            angle_int = (angle_high << 16) | angle_low
            if angle_int >= 2**31:
                angle_int -= 2**32
            angle = angle_int / 100.0
            
            # 其他檢測資訊
            contour_area = registers[9] if len(registers) > 9 else None
            
            logger.info(f"成功讀取CASE檢測結果: 中心({center_x}, {center_y}), 角度{angle:.2f}度, 面積{contour_area}")
            
            return {
                'success': True,
                'center': (center_x, center_y),
                'angle': angle,
                'major_axis': registers[5] if len(registers) > 5 else None,
                'minor_axis': registers[6] if len(registers) > 6 else None,
                'rect_width': registers[7] if len(registers) > 7 else None,
                'rect_height': registers[8] if len(registers) > 8 else None,
                'contour_area': contour_area
            }
            
        except Exception as e:
            logger.error(f"讀取CCD3檢測結果異常: {e}")
            return None

# 便利函數，供快速調用 - CASE模式預設
def detect_angle_with_ccd3(host: str = "127.0.0.1", port: int = 502, detection_mode: int = 0) -> AngleDetectionResult:
    """便利函數：一鍵執行CCD3角度檢測 - CASE模式版本
    
    自動處理連接/斷開，適合簡單的一次性調用
    
    Args:
        host: Modbus服務器IP
        port: Modbus服務器端口
        detection_mode: 檢測模式 (0=CASE, 1=DR)
        
    Returns:
        AngleDetectionResult: 檢測結果
    """
    angle_detector = AngleHighLevel(host, port)
    
    if not angle_detector.connect():
        return AngleDetectionResult(
            result=AngleOperationResult.CONNECTION_ERROR,
            message="無法連接到Modbus服務器"
        )
    
    try:
        result = angle_detector.detect_angle(detection_mode)
        return result
    finally:
        angle_detector.disconnect()