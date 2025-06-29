import threading
import time
import os
import sys
from typing import Dict, Any, Callable, Optional

# 嘗試導入專案中的所有GUIClass
try:
    # 添加GUITools路徑
    gui_tools_path = os.path.join(os.path.dirname(__file__), 'GUITools')
    if gui_tools_path not in sys.path:
        sys.path.append(gui_tools_path)
    
    # 導入所有GUIClass模組
    from CCD1GUIClass import CCD1GUIClass
    from CCD2GUIClass import CCD2GUIClass  
    from CCD3GUIClass import CCD3GUIClass
    from GripperGUIClass import GripperGUIClass
    from DobotM1ProGUIClass import DobotM1ProGUIClass
    
    GUICLASS_AVAILABLE = True
    print("✓ UX層成功載入所有GUIClass模組")
except ImportError as e:
    print(f"✗ UX層GUIClass模組載入失敗: {e}")
    GUICLASS_AVAILABLE = False

# 導入PyModbus
try:
    from pymodbus.client import ModbusTcpClient
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False
    print("✗ PyModbus未安裝，基礎模組功能受限")

class VPController:
    """VP震動盤控制器 - 業務邏輯層"""
    
    def __init__(self, host="127.0.0.1", port=502):
        self.host = host
        self.port = port
        self.client = None
        self.connected = False
        self.base_address = 300
        self.command_id = 1
        
        # VP動作映射
        self.action_map = {
            'stop': 0, 'up': 1, 'down': 2, 'left': 3, 'right': 4,
            'upleft': 5, 'downleft': 6, 'upright': 7, 'downright': 8,
            'horizontal': 9, 'vertical': 10, 'spread': 11
        }
        
        self.status_callbacks = []
        self.monitoring_thread = None
        self.running = False
    
    def add_status_callback(self, callback: Callable):
        """添加狀態變化回調"""
        self.status_callbacks.append(callback)
    
    def connect(self) -> bool:
        """連接Modbus TCP服務器"""
        if not MODBUS_AVAILABLE:
            return False
        try:
            self.client = ModbusTcpClient(host=self.host, port=self.port, timeout=1.0)
            self.connected = self.client.connect()
            if self.connected:
                self.start_monitoring()
            return self.connected
        except Exception as e:
            print(f"VP連接失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        self.running = False
        if self.client:
            self.client.close()
            self.connected = False
    
    def send_command(self, cmd_code, param1=0, param2=0, param3=0) -> bool:
        """發送指令到VP設備"""
        if not self.connected:
            return False
        try:
            self.client.write_register(address=self.base_address + 20, value=cmd_code, slave=1)
            self.client.write_register(address=self.base_address + 21, value=param1, slave=1)
            self.client.write_register(address=self.base_address + 22, value=param2, slave=1)
            self.client.write_register(address=self.base_address + 23, value=param3, slave=1)
            self.client.write_register(address=self.base_address + 24, value=self.command_id, slave=1)
            
            self.command_id += 1
            if self.command_id > 65535:
                self.command_id = 1
            return True
        except Exception as e:
            print(f"VP指令發送失敗: {e}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """獲取VP狀態"""
        if not self.connected:
            return {}
        try:
            result = self.client.read_holding_registers(address=self.base_address, count=15, slave=1)
            if hasattr(result, 'registers') and not result.isError():
                status_data = result.registers
                return {
                    'module_status': status_data[0],
                    'device_connection': status_data[1],
                    'device_status': status_data[2],
                    'error_code': status_data[3],
                    'command_status': status_data[8],
                    'brightness_status': status_data[10],
                    'backlight_status': status_data[11],
                    'vibration_status': status_data[12],
                    'frequency_status': status_data[13],
                }
        except Exception:
            pass
        return {}
    
    def start_monitoring(self):
        """啟動狀態監控"""
        self.running = True
        self.monitoring_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitoring_thread.start()
    
    def _monitor_loop(self):
        """狀態監控循環"""
        while self.running and self.connected:
            try:
                status = self.get_status()
                for callback in self.status_callbacks:
                    callback(status)
            except Exception:
                pass
            time.sleep(1)
    
    # VP專用控制方法
    def enable_backlight(self) -> bool:
        """開啟背光"""
        return self.send_command(1)
    
    def disable_backlight(self) -> bool:
        """關閉背光"""
        return self.send_command(2)
    
    def set_brightness(self, brightness: int) -> bool:
        """設定背光亮度 (0-255)"""
        return self.send_command(4, brightness)
    
    def execute_action(self, action: str, strength: int = 50, frequency: int = 128) -> bool:
        """執行震動動作"""
        action_code = self.action_map.get(action, 0)
        return self.send_command(5, action_code, strength, frequency)

class LightController:
    """LED照明控制器 - 業務邏輯層"""
    
    def __init__(self, host="127.0.0.1", port=502):
        self.host = host
        self.port = port
        self.client = None
        self.connected = False
        self.base_address = 600
        self.command_id = 1
        self.status_callbacks = []
        self.monitoring_thread = None
        self.running = False
    
    def add_status_callback(self, callback: Callable):
        """添加狀態變化回調"""
        self.status_callbacks.append(callback)
    
    def connect(self) -> bool:
        """連接Modbus TCP服務器"""
        if not MODBUS_AVAILABLE:
            return False
        try:
            self.client = ModbusTcpClient(host=self.host, port=self.port, timeout=1.0)
            self.connected = self.client.connect()
            if self.connected:
                self.start_monitoring()
            return self.connected
        except Exception as e:
            print(f"Light連接失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        self.running = False
        if self.client:
            self.client.close()
            self.connected = False
    
    def send_command(self, cmd_code, param1=0, param2=0) -> bool:
        """發送指令到LED設備"""
        if not self.connected:
            return False
        try:
            self.client.write_register(address=self.base_address + 20, value=cmd_code, slave=1)
            self.client.write_register(address=self.base_address + 21, value=param1, slave=1)
            self.client.write_register(address=self.base_address + 22, value=param2, slave=1)
            self.client.write_register(address=self.base_address + 23, value=self.command_id, slave=1)
            
            self.command_id += 1
            if self.command_id > 65535:
                self.command_id = 1
            return True
        except Exception as e:
            print(f"Light指令發送失敗: {e}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """獲取LED狀態"""
        if not self.connected:
            return {}
        try:
            result = self.client.read_holding_registers(address=self.base_address, count=16, slave=1)
            if hasattr(result, 'registers') and not result.isError():
                status_data = result.registers
                return {
                    'module_status': status_data[0],
                    'device_connection': status_data[1],
                    'active_channels': status_data[2],
                    'error_code': status_data[3],
                    'l1_state': status_data[4],
                    'l2_state': status_data[5],
                    'l3_state': status_data[6],
                    'l4_state': status_data[7],
                    'l1_brightness': status_data[8],
                    'l2_brightness': status_data[9],
                    'l3_brightness': status_data[10],
                    'l4_brightness': status_data[11],
                    'operation_count': status_data[12],
                    'error_count': status_data[13],
                }
        except Exception:
            pass
        return {}
    
    def start_monitoring(self):
        """啟動狀態監控"""
        self.running = True
        self.monitoring_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitoring_thread.start()
    
    def _monitor_loop(self):
        """狀態監控循環"""
        while self.running and self.connected:
            try:
                status = self.get_status()
                for callback in self.status_callbacks:
                    callback(status)
            except Exception:
                pass
            time.sleep(1)
    
    # LED專用控制方法
    def all_on(self) -> bool:
        """全部開啟"""
        return self.send_command(1)
    
    def all_off(self) -> bool:
        """全部關閉"""
        return self.send_command(2)
    
    def reset_device(self) -> bool:
        """重置設備"""
        return self.send_command(3)
    
    def set_channel_brightness(self, channel: int, brightness: int) -> bool:
        """設定通道亮度 (通道1-4, 亮度0-511)"""
        return self.send_command(4, channel, brightness)
    
    def channel_on(self, channel: int) -> bool:
        """開啟通道"""
        return self.send_command(5, channel)
    
    def channel_off(self, channel: int) -> bool:
        """關閉通道"""
        return self.send_command(6, channel)

class GUIClassController:
    """通用GUIClass控制器 - 業務邏輯層"""
    
    def __init__(self, module_name: str, gui_class_instance):
        self.module_name = module_name
        self.gui_instance = gui_class_instance
        self.status_callbacks = []
        
        # 如果GUIClass有狀態回調，註冊它
        if hasattr(self.gui_instance, 'add_status_callback'):
            self.gui_instance.add_status_callback(self._on_status_changed)
    
    def add_status_callback(self, callback: Callable):
        """添加狀態變化回調"""
        self.status_callbacks.append(callback)
    
    def _on_status_changed(self, status):
        """GUIClass狀態變化回調"""
        for callback in self.status_callbacks:
            callback(status)
    
    def connect(self) -> bool:
        """連接模組"""
        try:
            if hasattr(self.gui_instance, 'connect'):
                return self.gui_instance.connect()
            return True  # 假設已連接
        except Exception as e:
            print(f"{self.module_name}連接失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        try:
            if hasattr(self.gui_instance, 'disconnect'):
                self.gui_instance.disconnect()
        except Exception as e:
            print(f"{self.module_name}斷開失敗: {e}")
    
    def get_status(self):
        """獲取模組狀態"""
        try:
            if hasattr(self.gui_instance, 'get_status'):
                return self.gui_instance.get_status()
        except Exception as e:
            print(f"獲取{self.module_name}狀態失敗: {e}")
        return None
    
    def get_status_text(self) -> str:
        """獲取狀態文字描述"""
        try:
            status = self.get_status()
            if status:
                status_text = f"模組狀態更新: {time.strftime('%H:%M:%S')}\n"
                
                if hasattr(status, '__dict__'):
                    for key, value in status.__dict__.items():
                        status_text += f"{key}: {value}\n"
                elif isinstance(status, dict):
                    for key, value in status.items():
                        status_text += f"{key}: {value}\n"
                else:
                    status_text += f"狀態: {status}\n"
                
                return status_text
        except Exception as e:
            return f"狀態獲取失敗: {e}\n"
        
        return "無狀態資訊\n"
    
    # 通用控制方法
    def initialize_camera(self) -> bool:
        """初始化相機"""
        try:
            if hasattr(self.gui_instance, 'initialize_camera'):
                return self.gui_instance.initialize_camera()
        except Exception as e:
            print(f"{self.module_name}初始化相機失敗: {e}")
        return False
    
    def capture_image(self) -> bool:
        """拍照"""
        try:
            if hasattr(self.gui_instance, 'capture_image'):
                return self.gui_instance.capture_image()
        except Exception as e:
            print(f"{self.module_name}拍照失敗: {e}")
        return False
    
    def capture_and_detect(self) -> bool:
        """拍照+檢測"""
        try:
            if hasattr(self.gui_instance, 'capture_and_detect'):
                return self.gui_instance.capture_and_detect()
            elif hasattr(self.gui_instance, 'capture_and_classify'):
                return self.gui_instance.capture_and_classify()
        except Exception as e:
            print(f"{self.module_name}拍照+檢測失敗: {e}")
        return False
    
    def clear_command(self) -> bool:
        """清除指令"""
        try:
            if hasattr(self.gui_instance, 'clear_command'):
                return self.gui_instance.clear_command()
        except Exception as e:
            print(f"{self.module_name}清除指令失敗: {e}")
        return False
    
    def initialize_all_grippers(self):
        """初始化所有夾爪"""
        try:
            if hasattr(self.gui_instance, 'initialize_all_grippers'):
                return self.gui_instance.initialize_all_grippers()
        except Exception as e:
            print(f"{self.module_name}初始化夾爪失敗: {e}")
        return False
    
    def stop_all_grippers(self) -> bool:
        """停止所有夾爪"""
        try:
            if hasattr(self.gui_instance, 'stop_all_grippers'):
                return self.gui_instance.stop_all_grippers()
        except Exception as e:
            print(f"{self.module_name}停止夾爪失敗: {e}")
        return False
    
    def open_all_grippers(self) -> bool:
        """全部開啟夾爪"""
        try:
            if hasattr(self.gui_instance, 'open_all_grippers'):
                return self.gui_instance.open_all_grippers()
        except Exception as e:
            print(f"{self.module_name}開啟夾爪失敗: {e}")
        return False
    
    def close_all_grippers(self) -> bool:
        """全部關閉夾爪"""
        try:
            if hasattr(self.gui_instance, 'close_all_grippers'):
                return self.gui_instance.close_all_grippers()
        except Exception as e:
            print(f"{self.module_name}關閉夾爪失敗: {e}")
        return False
    
    def connect_robot(self) -> bool:
        """連接機械臂"""
        try:
            if hasattr(self.gui_instance, 'connect_robot'):
                return self.gui_instance.connect_robot()
        except Exception as e:
            print(f"{self.module_name}連接機械臂失敗: {e}")
        return False
    
    def enable_robot(self) -> bool:
        """使能機械臂"""
        try:
            if hasattr(self.gui_instance, 'enable_robot'):
                return self.gui_instance.enable_robot()
        except Exception as e:
            print(f"{self.module_name}使能機械臂失敗: {e}")
        return False
    
    def clear_error(self) -> bool:
        """清除錯誤"""
        try:
            if hasattr(self.gui_instance, 'clear_error'):
                return self.gui_instance.clear_error()
        except Exception as e:
            print(f"{self.module_name}清除錯誤失敗: {e}")
        return False
    
    def emergency_stop(self) -> bool:
        """緊急停止"""
        try:
            if hasattr(self.gui_instance, 'emergency_stop'):
                return self.gui_instance.emergency_stop()
        except Exception as e:
            print(f"{self.module_name}緊急停止失敗: {e}")
        return False

class UnifiedUXManager:
    """統一UX管理器 - 協調所有業務邏輯"""
    
    def __init__(self):
        # 初始化基礎控制器
        self.vp_controller = VPController()
        self.light_controller = LightController()
        
        # 初始化GUIClass控制器
        self.guiclass_controllers = {}
        self._init_guiclass_controllers()
        
        # UI回調集合
        self.ui_callbacks = {}
        
        # 狀態更新線程
        self.status_update_thread = None
        self.running = False
    
    def _init_guiclass_controllers(self):
        """初始化GUIClass控制器"""
        if GUICLASS_AVAILABLE:
            try:
                # 創建GUIClass實例
                guiclass_instances = {
                    'CCD1': CCD1GUIClass(),
                    'CCD2': CCD2GUIClass(),
                    'CCD3': CCD3GUIClass(),
                    'Gripper': GripperGUIClass(),
                    'DobotM1Pro': DobotM1ProGUIClass()
                }
                
                # 包裝為控制器
                for name, instance in guiclass_instances.items():
                    self.guiclass_controllers[name] = GUIClassController(name, instance)
                
                print(f"✓ UX層成功初始化 {len(self.guiclass_controllers)} 個GUIClass控制器")
            except Exception as e:
                print(f"✗ UX層GUIClass控制器初始化失敗: {e}")
                self.guiclass_controllers = {}
    
    def set_ui_callback(self, callback_name: str, callback: Callable):
        """設置UI回調"""
        self.ui_callbacks[callback_name] = callback
    
    def _call_ui_callback(self, callback_name: str, *args):
        """調用UI回調"""
        if callback_name in self.ui_callbacks:
            self.ui_callbacks[callback_name](*args)
    
    def get_connection_info(self) -> Dict[str, str]:
        """獲取連接資訊"""
        return {
            'basic_modules': ['VP', 'Light'],
            'advanced_modules': list(self.guiclass_controllers.keys()),
            'total_modules': 2 + len(self.guiclass_controllers)
        }
    
    def get_system_info(self) -> str:
        """獲取系統資訊"""
        info = f"""基礎模組寄存器:
VP: 300-349 (震動盤)
LED: 600-649 (照明)

進階模組:
CCD1: 200-299 (視覺檢測)
CCD2: 1100-1199 (圖像分類)
CCD3: 800-899 (角度檢測)
Gripper: 500-599 (夾爪控制)
Dobot: 1200-1299 (機械臂)

狀態: {'✓ 完整功能' if GUICLASS_AVAILABLE else '✗ 基礎功能'}
PyModbus: {'✓ 可用' if MODBUS_AVAILABLE else '✗ 不可用'}"""
        return info
    
    def toggle_connection(self, host: str, port: str) -> Dict[str, Any]:
        """切換連接狀態"""
        results = {'vp': False, 'light': False, 'advanced': []}
        
        try:
            port_int = int(port)
            
            # 連接基礎模組
            self.vp_controller.host = host
            self.vp_controller.port = port_int
            self.light_controller.host = host
            self.light_controller.port = port_int
            
            results['vp'] = self.vp_controller.connect()
            results['light'] = self.light_controller.connect()
            
            # 連接進階模組
            for name, controller in self.guiclass_controllers.items():
                success = controller.connect()
                results['advanced'].append({'name': name, 'connected': success})
                
                if success:
                    self._call_ui_callback('add_log', f"{name}模組連接成功")
                else:
                    self._call_ui_callback('add_log', f"{name}模組連接失敗")
            
            # 啟動狀態監控
            if not self.running:
                self.start_status_monitoring()
            
            total_connected = sum([results['vp'], results['light']]) + sum(1 for item in results['advanced'] if item['connected'])
            
            if total_connected > 0:
                self._call_ui_callback('add_log', f"成功連接 {total_connected} 個模組")
            else:
                self._call_ui_callback('add_log', "所有模組連接失敗")
                
        except Exception as e:
            self._call_ui_callback('add_log', f"連接過程異常: {e}")
        
        return results
    
    def start_status_monitoring(self):
        """啟動狀態監控"""
        self.running = True
        self.status_update_thread = threading.Thread(target=self._status_monitor_loop, daemon=True)
        self.status_update_thread.start()
        self._call_ui_callback('add_log', "狀態監控已啟動")
    
    def _status_monitor_loop(self):
        """狀態監控循環"""
        while self.running:
            try:
                # 基礎模組狀態通過各自的監控線程更新
                # 這裡處理進階模組狀態更新
                for name, controller in self.guiclass_controllers.items():
                    try:
                        # 進階模組的狀態更新由各自的GUIClass處理
                        pass
                    except Exception:
                        pass
                        
            except Exception as e:
                print(f"狀態監控異常: {e}")
                
            time.sleep(3)  # 每3秒檢查一次
    
    def stop_monitoring(self):
        """停止狀態監控"""
        self.running = False
    
    def disconnect_all(self):
        """斷開所有連接"""
        self.vp_controller.disconnect()
        self.light_controller.disconnect()
        
        for name, controller in self.guiclass_controllers.items():
            try:
                controller.disconnect()
            except Exception as e:
                print(f"斷開{name}連接失敗: {e}")
        
        self.stop_monitoring()
        self._call_ui_callback('add_log', "所有模組已斷開連接")
    
    # VP控制器事件處理
    def get_vp_callbacks(self) -> Dict[str, Callable]:
        """獲取VP控制器回調函數"""
        return {
            'backlight_on': self._vp_backlight_on,
            'backlight_off': self._vp_backlight_off,
            'set_brightness': self._vp_set_brightness,
            'execute_action': self._vp_execute_action
        }
    
    def _vp_backlight_on(self):
        """VP背光開啟"""
        if self.vp_controller.enable_backlight():
            self._call_ui_callback('add_log', "VP: 背光開啟成功")
        else:
            self._call_ui_callback('add_log', "VP: 背光開啟失敗")
    
    def _vp_backlight_off(self):
        """VP背光關閉"""
        if self.vp_controller.disable_backlight():
            self._call_ui_callback('add_log', "VP: 背光關閉成功")
        else:
            self._call_ui_callback('add_log', "VP: 背光關閉失敗")
    
    def _vp_set_brightness(self, brightness: int):
        """VP設定亮度"""
        if self.vp_controller.set_brightness(brightness):
            self._call_ui_callback('add_log', f"VP: 亮度設定為{brightness}")
        else:
            self._call_ui_callback('add_log', f"VP: 亮度設定失敗")
    
    def _vp_execute_action(self, action: str, vibration_params: Dict[str, int] = None):
        """VP執行動作"""
        if vibration_params is None:
            vibration_params = {'strength': 50, 'frequency': 128}
        
        strength = vibration_params.get('strength', 50)
        frequency = vibration_params.get('frequency', 128)
        
        if self.vp_controller.execute_action(action, strength, frequency):
            self._call_ui_callback('add_log', f"VP: 執行{action}動作 (強度{strength}, 頻率{frequency})")
        else:
            self._call_ui_callback('add_log', f"VP: 執行{action}動作失敗")
    
    # Light控制器事件處理
    def get_light_callbacks(self) -> Dict[str, Callable]:
        """獲取Light控制器回調函數"""
        return {
            'all_on': self._light_all_on,
            'all_off': self._light_all_off,
            'reset_device': self._light_reset_device,
            'channel_on': self._light_channel_on,
            'channel_off': self._light_channel_off,
            'set_channel_brightness': self._light_set_channel_brightness
        }
    
    def _light_all_on(self):
        """LED全部開啟"""
        if self.light_controller.all_on():
            self._call_ui_callback('add_log', "LED: 全部開啟成功")
        else:
            self._call_ui_callback('add_log', "LED: 全部開啟失敗")
    
    def _light_all_off(self):
        """LED全部關閉"""
        if self.light_controller.all_off():
            self._call_ui_callback('add_log', "LED: 全部關閉成功")
        else:
            self._call_ui_callback('add_log', "LED: 全部關閉失敗")
    
    def _light_reset_device(self):
        """LED重置設備"""
        if self.light_controller.reset_device():
            self._call_ui_callback('add_log', "LED: 設備重置成功")
        else:
            self._call_ui_callback('add_log', "LED: 設備重置失敗")
    
    def _light_channel_on(self, channel: int):
        """LED通道開啟"""
        if self.light_controller.channel_on(channel):
            self._call_ui_callback('add_log', f"LED: L{channel}開啟成功")
        else:
            self._call_ui_callback('add_log', f"LED: L{channel}開啟失敗")
    
    def _light_channel_off(self, channel: int):
        """LED通道關閉"""
        if self.light_controller.channel_off(channel):
            self._call_ui_callback('add_log', f"LED: L{channel}關閉成功")
        else:
            self._call_ui_callback('add_log', f"LED: L{channel}關閉失敗")
    
    def _light_set_channel_brightness(self, channel: int, brightness: int):
        """LED設定通道亮度"""
        if self.light_controller.set_channel_brightness(channel, brightness):
            self._call_ui_callback('add_log', f"LED: L{channel}亮度設定為{brightness}")
        else:
            self._call_ui_callback('add_log', f"LED: L{channel}亮度設定失敗")
    
    # GUIClass控制器事件處理
    def get_guiclass_callbacks(self, module_name: str) -> Dict[str, Callable]:
        """獲取GUIClass控制器回調函數"""
        if module_name not in self.guiclass_controllers:
            return {}
        
        controller = self.guiclass_controllers[module_name]
        
        return {
            'initialize_camera': lambda: self._guiclass_action(controller, 'initialize_camera', '初始化相機'),
            'capture_image': lambda: self._guiclass_action(controller, 'capture_image', '拍照'),
            'capture_and_detect': lambda: self._guiclass_action(controller, 'capture_and_detect', '拍照+檢測'),
            'clear_command': lambda: self._guiclass_action(controller, 'clear_command', '清除指令'),
            'initialize_all_grippers': lambda: self._guiclass_gripper_action(controller, 'initialize_all_grippers', '初始化夾爪'),
            'stop_all_grippers': lambda: self._guiclass_action(controller, 'stop_all_grippers', '停止所有夾爪'),
            'open_all_grippers': lambda: self._guiclass_action(controller, 'open_all_grippers', '全部開啟夾爪'),
            'close_all_grippers': lambda: self._guiclass_action(controller, 'close_all_grippers', '全部關閉夾爪'),
            'connect_robot': lambda: self._guiclass_action(controller, 'connect_robot', '連接機械臂'),
            'enable_robot': lambda: self._guiclass_action(controller, 'enable_robot', '使能機械臂'),
            'clear_error': lambda: self._guiclass_action(controller, 'clear_error', '清除錯誤'),
            'emergency_stop': lambda: self._guiclass_action(controller, 'emergency_stop', '緊急停止')
        }
    
    def _guiclass_action(self, controller: GUIClassController, action: str, action_name: str):
        """執行GUIClass動作"""
        try:
            method = getattr(controller, action)
            result = method()
            status = "成功" if result else "失敗"
            self._call_ui_callback('add_log', f"{controller.module_name}: {action_name}{status}")
            
            # 更新狀態顯示
            self._call_ui_callback('update_guiclass_status', controller.module_name, controller.get_status_text())
            
        except Exception as e:
            self._call_ui_callback('add_log', f"{controller.module_name}: {action_name}異常 - {e}")
    
    def _guiclass_gripper_action(self, controller: GUIClassController, action: str, action_name: str):
        """執行夾爪特殊動作"""
        try:
            method = getattr(controller, action)
            result = method()
            
            if isinstance(result, dict):
                success_count = sum(1 for v in result.values() if v)
                self._call_ui_callback('add_log', f"{controller.module_name}: {action_name} {success_count}/{len(result)} 成功")
            else:
                status = "成功" if result else "失敗"
                self._call_ui_callback('add_log', f"{controller.module_name}: {action_name}{status}")
            
            # 更新狀態顯示
            self._call_ui_callback('update_guiclass_status', controller.module_name, controller.get_status_text())
            
        except Exception as e:
            self._call_ui_callback('add_log', f"{controller.module_name}: {action_name}異常 - {e}")
    
    def get_module_list(self) -> Dict[str, list]:
        """獲取模組列表"""
        return {
            'basic': ['VP', 'Light'],
            'advanced': list(self.guiclass_controllers.keys())
        }
    
    def get_controller_status_text(self, module_name: str) -> str:
        """獲取控制器狀態文字"""
        if module_name in self.guiclass_controllers:
            return self.guiclass_controllers[module_name].get_status_text()
        return "模組不存在"