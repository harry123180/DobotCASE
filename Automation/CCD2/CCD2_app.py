# -*- coding: utf-8 -*-
"""
CCD2_app.py - CCD2圖像分類Web控制應用 (修正版)
作為純Modbus TCP Client連接主服務器，通過CCD2主模組控制圖像分類
修正版：增強指令發送、連接檢查和調試功能
"""

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import threading
import time
import json
import os
from datetime import datetime
from pymodbus.client import ModbusTcpClient
from typing import Dict, Any, Optional

class CCD2WebApp:
    """CCD2圖像分類Web控制應用 - 純Modbus TCP Client (修正版)"""
    
    def __init__(self):
        # 載入配置
        self.config = self.load_config()
        
        # Modbus TCP Client (連接主服務器)
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected_to_server = False
        
        # 狀態監控
        self.status_monitor_thread = None
        self.monitoring = False
        
        # 寄存器映射 (與CCD2主模組一致 - 基地址1000)
        self.base_address = self.config['modbus_mapping']['base_address']
        self.init_register_mapping()
        
        # 指令計數
        self.command_id_counter = 1
        
        # 初始化Flask應用
        self.init_flask_app()
        
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "module_id": "CCD2分類Web UI",
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "modbus_mapping": {
                "base_address": 1000
            },
            "web_server": {
                "host": "localhost",
                "port": 5055,
                "debug": False
            },
            "classification": {
                "condition_folder": "condition",
                "auto_scan": True
            }
        }
        
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(current_dir, "ccd2_app_config.json")
            
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            print(f"載入配置檔案失敗: {e}")
            
        return default_config
    
    def init_register_mapping(self):
        """初始化寄存器映射 (與CCD2主模組一致)"""
        base = self.base_address
        
        # 控制寄存器
        self.control_registers = {
            'command': base + 0,      # 1000: 控制指令
            'state': base + 1,        # 1001: 狀態寄存器
        }
        
        # 參數寄存器
        self.param_registers = {
            'gaussian_kernel': base + 10,    # 1010: 高斯模糊核大小
            'threshold_mode': base + 11,     # 1011: 閾值處理模式
            'manual_threshold': base + 12,   # 1012: 手動閾值
            'canny_low': base + 13,          # 1013: Canny低閾值
            'canny_high': base + 14,         # 1014: Canny高閾值
            'lbp_radius': base + 15,         # 1015: LBP半徑
            'lbp_points': base + 16,         # 1016: LBP點數
            'roi_mode': base + 17,           # 1017: ROI模式
        }
        
        # ROI寄存器
        self.roi_registers = {
            'roi_x': base + 20,      # 1020: ROI X座標
            'roi_y': base + 21,      # 1021: ROI Y座標
            'roi_width': base + 22,  # 1022: ROI寬度
            'roi_height': base + 23, # 1023: ROI高度
        }
        
        # 結果寄存器
        self.result_registers = {
            'success': base + 40,           # 1040: 分類成功標誌
            'category_id': base + 41,       # 1041: 分類類別ID
            'confidence_high': base + 42,   # 1042: 信心度高位
            'confidence_low': base + 43,    # 1043: 信心度低位
            'matched_conditions': base + 44, # 1044: 匹配條件數量
        }
        
        # 特徵寄存器
        self.feature_registers = {
            'mean_high': base + 50,     # 1050: 平均值高位
            'mean_low': base + 51,      # 1051: 平均值低位
            'std_high': base + 52,      # 1052: 標準差高位
            'std_low': base + 53,       # 1053: 標準差低位
            'skew_high': base + 54,     # 1054: 偏度高位
            'skew_low': base + 55,      # 1055: 偏度低位
            'kurt_high': base + 56,     # 1056: 峰度高位
            'kurt_low': base + 57,      # 1057: 峰度低位
        }
        
        # 統計寄存器
        self.stats_registers = {
            'last_capture_time': base + 80, # 1080: 最後拍照耗時
            'last_process_time': base + 81, # 1081: 最後處理耗時
            'last_total_time': base + 82,   # 1082: 最後總耗時
            'classification_count': base + 83, # 1083: 分類計數器
            'error_count': base + 84,       # 1084: 錯誤計數器
            'json_load_count': base + 85,   # 1085: JSON載入次數
            'version_major': base + 90,     # 1090: 軟體版本主號
            'version_minor': base + 91,     # 1091: 軟體版本次號
            'runtime_hours': base + 92,     # 1092: 運行時間小時
            'runtime_minutes': base + 93,   # 1093: 運行時間分鐘
        }
    
    def init_flask_app(self):
        """初始化Flask應用"""
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'ccd2_classification_secret_key'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # 設置路由
        self.setup_routes()
        self.setup_socketio_events()
    
    def setup_routes(self):
        """設置Flask路由"""
        
        @self.app.route('/')
        def index():
            return render_template('ccd2_classification.html')
        
        @self.app.route('/api/status')
        def get_status():
            """獲取系統狀態"""
            try:
                status = self.get_system_status()
                return jsonify({
                    'success': True,
                    'data': status
                })
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        @self.app.route('/api/connect', methods=['POST'])
        def connect_server():
            """連接Modbus服務器"""
            try:
                data = request.get_json()
                host = data.get('host', self.config['tcp_server']['host'])
                port = data.get('port', self.config['tcp_server']['port'])
                
                result = self.connect_modbus_server(host, port)
                
                if result['success']:
                    self.start_monitoring()
                
                return jsonify(result)
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        @self.app.route('/api/classification/execute', methods=['POST'])
        def execute_classification():
            """執行分類檢測"""
            try:
                result = self.send_classification_command()
                return jsonify(result)
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        @self.app.route('/api/parameters/set', methods=['POST'])
        def set_parameters():
            """設置處理參數"""
            try:
                data = request.get_json()
                result = self.set_processing_parameters(data)
                return jsonify(result)
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        @self.app.route('/api/roi/set', methods=['POST'])
        def set_roi():
            """設置ROI區域"""
            try:
                data = request.get_json()
                result = self.set_roi_parameters(data)
                return jsonify(result)
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        @self.app.route('/api/config/load', methods=['POST'])
        def load_config():
            """重新載入JSON配置"""
            try:
                result = self.reload_json_config()
                return jsonify(result)
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        @self.app.route('/api/config/scan')
        def scan_configs():
            """掃描condition資料夾中的配置檔案"""
            try:
                current_dir = os.path.dirname(os.path.abspath(__file__))
                condition_dir = os.path.join(current_dir, "condition")
                
                if not os.path.exists(condition_dir):
                    return jsonify({
                        'success': True,
                        'data': {
                            'files': [],
                            'message': 'condition資料夾不存在'
                        }
                    })
                
                json_files = []
                for file in os.listdir(condition_dir):
                    if file.lower().endswith('.json'):
                        file_path = os.path.join(condition_dir, file)
                        try:
                            # 驗證JSON格式並取得基本資訊
                            with open(file_path, 'r', encoding='utf-8') as f:
                                config = json.load(f)
                            
                            file_info = {
                                'filename': file,
                                'module_name': config.get('module_info', {}).get('module_name', 'Unknown'),
                                'version': config.get('module_info', {}).get('version', 'Unknown'),
                                'description': config.get('module_info', {}).get('description', ''),
                                'categories_count': len(config.get('categories', [])),
                                'created_date': config.get('module_info', {}).get('created_date', 'Unknown')
                            }
                            json_files.append(file_info)
                        except Exception as e:
                            # 如果JSON格式錯誤，仍然列出但標記為無效
                            json_files.append({
                                'filename': file,
                                'error': str(e),
                                'valid': False
                            })
                
                return jsonify({
                    'success': True,
                    'data': {
                        'files': json_files,
                        'total_count': len(json_files),
                        'valid_count': len([f for f in json_files if f.get('valid', True)])
                    }
                })
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        # === 新增調試路由 ===
        @self.app.route('/api/debug/registers')
        def debug_registers():
            """調試：檢查寄存器狀態"""
            try:
                result = self.debug_register_status()
                return jsonify(result)
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
        
        @self.app.route('/api/debug/connection')
        def debug_connection():
            """調試連接狀態"""
            try:
                status = {
                    'connected_to_server': self.connected_to_server,
                    'modbus_client_exists': self.modbus_client is not None,
                    'server_config': {
                        'host': self.config['tcp_server']['host'],
                        'port': self.config['tcp_server']['port'],
                        'unit_id': self.config['tcp_server']['unit_id'],
                        'timeout': self.config['tcp_server']['timeout']
                    },
                    'register_mapping': {
                        'command_register': self.control_registers['command'],
                        'state_register': self.control_registers['state'],
                        'base_address': self.base_address
                    }
                }
                
                if self.modbus_client and self.connected_to_server:
                    # 嘗試ping測試
                    try:
                        test_result = self.modbus_client.read_holding_registers(
                            address=self.control_registers['state'],
                            count=1,
                            slave=self.config['tcp_server']['unit_id']
                        )
                        status['ping_test'] = {
                            'success': not test_result.isError(),
                            'result': str(test_result) if test_result.isError() else test_result.registers[0]
                        }
                    except Exception as e:
                        status['ping_test'] = {
                            'success': False,
                            'error': str(e)
                        }
                
                return jsonify({
                    'success': True,
                    'debug_status': status
                })
                
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500

        @self.app.route('/api/test/write_command', methods=['POST'])
        def test_write_command():
            """測試寫入指令"""
            try:
                data = request.get_json()
                command_value = data.get('command', 16)
                
                if not self.connected_to_server:
                    return jsonify({'success': False, 'error': '未連接到服務器'})
                
                print(f"測試寫入指令: {command_value}")
                
                result = self.modbus_client.write_register(
                    address=self.control_registers['command'],
                    value=command_value,
                    slave=self.config['tcp_server']['unit_id']
                )
                
                success = not (hasattr(result, 'isError') and result.isError())
                
                return jsonify({
                    'success': success,
                    'message': f'測試寫入指令 {command_value}',
                    'result': str(result) if not success else 'OK',
                    'register_address': self.control_registers['command']
                })
                
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
    
    def setup_socketio_events(self):
        """設置SocketIO事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            print('Web客戶端已連接')
            emit('status_update', self.get_system_status())
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print('Web客戶端已斷開')
        
        @self.socketio.on('request_status')
        def handle_status_request():
            emit('status_update', self.get_system_status())
    
    def connect_modbus_server(self, host=None, port=None) -> Dict[str, Any]:
        """連接Modbus服務器 - 修正版"""
        try:
            if host is None:
                host = self.config['tcp_server']['host']
            if port is None:
                port = self.config['tcp_server']['port']
            
            print(f"正在連接Modbus服務器: {host}:{port}")
            
            if self.modbus_client:
                try:
                    self.modbus_client.close()
                    print("已關閉舊連接")
                except:
                    pass
            
            self.modbus_client = ModbusTcpClient(
                host=host,
                port=port,
                timeout=self.config['tcp_server']['timeout']
            )
            
            if self.modbus_client.connect():
                self.connected_to_server = True
                print(f"✅ Modbus服務器連接成功: {host}:{port}")
                
                # 連接後測試讀寫功能
                try:
                    # 測試讀取狀態寄存器
                    test_read = self.modbus_client.read_holding_registers(
                        address=self.control_registers['state'],
                        count=1,
                        slave=self.config['tcp_server']['unit_id']
                    )
                    
                    if test_read.isError():
                        print(f"⚠️ 連接成功但讀取測試失敗: {test_read}")
                    else:
                        state_value = test_read.registers[0]
                        print(f"✅ 讀取測試成功，狀態寄存器值: {state_value}")
                    
                    return {
                        'success': True,
                        'message': f'已連接到 {host}:{port}，讀取測試正常',
                        'test_read_success': not test_read.isError(),
                        'state_register_value': test_read.registers[0] if not test_read.isError() else None
                    }
                    
                except Exception as test_error:
                    print(f"⚠️ 連接測試異常: {test_error}")
                    return {
                        'success': True,
                        'message': f'已連接到 {host}:{port} (測試異常: {test_error})',
                        'test_read_success': False
                    }
            else:
                self.connected_to_server = False
                error_msg = f"無法連接到 {host}:{port}"
                print(f"❌ {error_msg}")
                return {
                    'success': False,
                    'error': error_msg
                }
        except Exception as e:
            self.connected_to_server = False
            error_msg = f"連接異常: {str(e)}"
            print(f"❌ {error_msg}")
            return {
                'success': False,
                'error': error_msg
            }
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        status = {
            'connected': self.connected_to_server,
            'server_info': {
                'host': self.config['tcp_server']['host'],
                'port': self.config['tcp_server']['port'],
                'base_address': self.base_address
            },
            'state': {},
            'parameters': {},
            'roi': {},
            'result': {},
            'features': {},
            'statistics': {},
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        
        if self.connected_to_server and self.modbus_client:
            try:
                # 讀取狀態信息
                status['state'] = self.read_state_registers()
                status['parameters'] = self.read_parameter_registers()
                status['roi'] = self.read_roi_registers()
                status['result'] = self.read_result_registers()
                status['features'] = self.read_feature_registers()
                status['statistics'] = self.read_statistics_registers()
            except Exception as e:
                print(f"讀取狀態失敗: {e}")
        
        return status
    
    def debug_register_status(self) -> Dict[str, Any]:
        """調試：檢查關鍵寄存器狀態"""
        if not self.connected_to_server or not self.modbus_client:
            return {'error': '未連接到服務器'}
        
        try:
            debug_info = {}
            
            # 讀取控制指令寄存器
            cmd_response = self.modbus_client.read_holding_registers(
                address=self.control_registers['command'],
                count=1,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if not cmd_response.isError():
                debug_info['control_command'] = {
                    'address': self.control_registers['command'],
                    'value': cmd_response.registers[0],
                    'status': '正常'
                }
            else:
                debug_info['control_command'] = {
                    'address': self.control_registers['command'],
                    'error': str(cmd_response),
                    'status': '讀取失敗'
                }
            
            # 讀取狀態寄存器
            state_response = self.modbus_client.read_holding_registers(
                address=self.control_registers['state'],
                count=1,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if not state_response.isError():
                state_value = state_response.registers[0]
                debug_info['state_register'] = {
                    'address': self.control_registers['state'],
                    'value': state_value,
                    'ready': bool(state_value & 0x01),
                    'running': bool(state_value & 0x02),
                    'alarm': bool(state_value & 0x04),
                    'initialized': bool(state_value & 0x08),
                    'status': '正常'
                }
            else:
                debug_info['state_register'] = {
                    'address': self.control_registers['state'],
                    'error': str(state_response),
                    'status': '讀取失敗'
                }
            
            return {
                'success': True,
                'debug_info': debug_info,
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
            }
            
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }
    
    def read_state_registers(self) -> Dict[str, Any]:
        """讀取狀態寄存器"""
        try:
            # 讀取狀態信息
            response = self.modbus_client.read_holding_registers(
                address=self.control_registers['state'],
                count=1,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if response.isError():
                return {}
            
            state_value = response.registers[0]
            
            return {
                'ready': bool(state_value & 0x01),
                'running': bool(state_value & 0x02),
                'alarm': bool(state_value & 0x04),
                'initialized': bool(state_value & 0x08),
                'config_loaded': bool(state_value & 0x10),
                'raw_value': state_value
            }
        except Exception as e:
            print(f"讀取狀態寄存器失敗: {e}")
            return {}
    
    def read_parameter_registers(self) -> Dict[str, Any]:
        """讀取處理參數寄存器"""
        try:
            # 讀取處理參數寄存器
            response = self.modbus_client.read_holding_registers(
                address=self.base_address + 10,
                count=8,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if response.isError():
                return {}
            
            registers = response.registers
            
            return {
                'gaussian_kernel': registers[0],
                'threshold_mode': 'OTSU自動' if registers[1] == 0 else '手動',
                'manual_threshold': registers[2],
                'canny_low': registers[3],
                'canny_high': registers[4],
                'lbp_radius': registers[5],
                'lbp_points': registers[6],
                'roi_mode': '啟用' if registers[7] == 1 else '關閉'
            }
        except Exception as e:
            print(f"讀取參數寄存器失敗: {e}")
            return {}
    
    def read_roi_registers(self) -> Dict[str, Any]:
        """讀取ROI寄存器"""
        try:
            # 讀取ROI寄存器
            response = self.modbus_client.read_holding_registers(
                address=self.base_address + 20,
                count=4,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if response.isError():
                return {}
            
            registers = response.registers
            
            return {
                'x': registers[0],
                'y': registers[1],
                'width': registers[2],
                'height': registers[3]
            }
        except Exception as e:
            print(f"讀取ROI寄存器失敗: {e}")
            return {}
    
    def read_result_registers(self) -> Dict[str, Any]:
        """讀取分類結果寄存器"""
        try:
            # 讀取分類結果寄存器
            response = self.modbus_client.read_holding_registers(
                address=self.base_address + 40,
                count=10,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if response.isError():
                return {}
            
            registers = response.registers
            
            # 重構32位信心度
            confidence_int = (registers[2] << 16) | registers[3]
            confidence = confidence_int / 100.0
            
            return {
                'success': bool(registers[0]),
                'category_id': registers[1],
                'confidence': confidence,
                'matched_conditions': registers[4],
                'status': '成功' if registers[0] else '失敗'
            }
        except Exception as e:
            print(f"讀取結果寄存器失敗: {e}")
            return {}
    
    def read_feature_registers(self) -> Dict[str, Any]:
        """讀取特徵數值寄存器"""
        try:
            # 讀取特徵數值寄存器
            response = self.modbus_client.read_holding_registers(
                address=self.base_address + 50,
                count=8,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if response.isError():
                return {}
            
            registers = response.registers
            
            # 重構32位特徵值
            mean = ((registers[0] << 16) | registers[1]) / 100.0
            std = ((registers[2] << 16) | registers[3]) / 100.0
            skew = ((registers[4] << 16) | registers[5]) / 1000.0
            kurt = ((registers[6] << 16) | registers[7]) / 1000.0
            
            return {
                'mean': round(mean, 2),
                'std': round(std, 2),
                'skewness': round(skew, 3),
                'kurtosis': round(kurt, 3)
            }
        except Exception as e:
            print(f"讀取特徵寄存器失敗: {e}")
            return {}
    
    def read_statistics_registers(self) -> Dict[str, Any]:
        """讀取統計資訊寄存器"""
        try:
            # 讀取統計資訊寄存器
            response = self.modbus_client.read_holding_registers(
                address=self.base_address + 80,
                count=14,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if response.isError():
                return {}
            
            registers = response.registers
            
            return {
                'last_capture_time': registers[0],
                'last_process_time': registers[1],
                'last_total_time': registers[2],
                'classification_count': registers[3],
                'error_count': registers[4],
                'json_load_count': registers[5],
                'version': f"{registers[10]}.{registers[11]}",
                'runtime_hours': registers[12],
                'runtime_minutes': registers[13]
            }
        except Exception as e:
            print(f"讀取統計寄存器失敗: {e}")
            return {}
    
    def send_classification_command(self) -> Dict[str, Any]:
        """發送分類檢測指令 - 修正版"""
        try:
            if not self.connected_to_server:
                return {'success': False, 'error': '未連接到服務器'}
            
            if not self.modbus_client:
                return {'success': False, 'error': 'Modbus客戶端未初始化'}
            
            print(f"正在發送分類檢測指令到地址 {self.control_registers['command']} (值=16)")
            
            # 發送拍照+分類檢測指令 (16) 
            result = self.modbus_client.write_register(
                address=self.control_registers['command'],
                value=16,
                slave=self.config['tcp_server']['unit_id']
            )
            
            # 檢查寫入結果
            if hasattr(result, 'isError') and result.isError():
                error_msg = f"寫入失敗: {result}"
                print(f"❌ {error_msg}")
                return {'success': False, 'error': error_msg}
            
            # 驗證寫入是否成功 - 讀回確認
            try:
                time.sleep(0.1)  # 短暫延遲
                read_result = self.modbus_client.read_holding_registers(
                    address=self.control_registers['command'],
                    count=1,
                    slave=self.config['tcp_server']['unit_id']
                )
                
                if not read_result.isError():
                    actual_value = read_result.registers[0]
                    print(f"✅ 指令寫入成功，讀回值: {actual_value}")
                    
                    if actual_value == 16:
                        return {
                            'success': True,
                            'message': '分類檢測指令已發送且確認寫入',
                            'command_value': 16,
                            'register_address': self.control_registers['command']
                        }
                    else:
                        return {
                            'success': True,
                            'message': f'指令已發送，當前值: {actual_value}',
                            'command_value': actual_value,
                            'register_address': self.control_registers['command']
                        }
                else:
                    print(f"⚠️ 無法讀回確認，但寫入命令已執行")
                    return {
                        'success': True,
                        'message': '分類檢測指令已發送 (無法確認)',
                        'command_value': 16,
                        'register_address': self.control_registers['command']
                    }
            except Exception as verify_error:
                print(f"⚠️ 驗證讀取失敗: {verify_error}")
                return {
                    'success': True,
                    'message': '分類檢測指令已發送 (驗證失敗)',
                    'command_value': 16,
                    'register_address': self.control_registers['command']
                }
                
        except Exception as e:
            error_msg = f"發送指令異常: {str(e)}"
            print(f"❌ {error_msg}")
            return {
                'success': False,
                'error': error_msg
            }
    
    def set_processing_parameters(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """設置處理參數"""
        try:
            if not self.connected_to_server:
                return {'success': False, 'error': '未連接到服務器'}
            
            # 準備寄存器數據
            registers = [
                params.get('gaussian_kernel', 5),
                1 if params.get('use_otsu', True) else 0,
                params.get('manual_threshold', 127),
                params.get('canny_low', 50),
                params.get('canny_high', 150),
                params.get('lbp_radius', 3),
                params.get('lbp_points', 24),
                1 if params.get('roi_enabled', False) else 0
            ]
            
            # 寫入參數寄存器
            self.modbus_client.write_registers(
                address=self.base_address + 10,
                values=registers,
                slave=self.config['tcp_server']['unit_id']
            )
            
            return {
                'success': True,
                'message': '處理參數已更新'
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }
    
    def set_roi_parameters(self, roi_params: Dict[str, Any]) -> Dict[str, Any]:
        """設置ROI參數"""
        try:
            if not self.connected_to_server:
                return {'success': False, 'error': '未連接到服務器'}
            
            # 準備ROI寄存器數據
            registers = [
                roi_params.get('x', 0),
                roi_params.get('y', 0),
                roi_params.get('width', 100),
                roi_params.get('height', 100)
            ]
            
            # 寫入ROI寄存器
            self.modbus_client.write_registers(
                address=self.base_address + 20,
                values=registers,
                slave=self.config['tcp_server']['unit_id']
            )
            
            return {
                'success': True,
                'message': 'ROI參數已更新'
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }
    
    def reload_json_config(self) -> Dict[str, Any]:
        """重新載入JSON配置 - 修正版"""
        try:
            if not self.connected_to_server:
                return {'success': False, 'error': '未連接到服務器'}
            
            print("正在發送重新載入JSON配置指令 (64)")
            
            # 發送重新載入JSON配置指令 (64)
            result = self.modbus_client.write_register(
                address=self.control_registers['command'],
                value=64,
                slave=self.config['tcp_server']['unit_id']
            )
            
            # 檢查寫入結果
            if hasattr(result, 'isError') and result.isError():
                return {
                    'success': False,
                    'error': f'寫入失敗: {result}'
                }
            
            print("✅ JSON配置重新載入指令已發送")
            return {
                'success': True,
                'message': 'JSON配置重新載入指令已發送',
                'command_value': 64,
                'register_address': self.control_registers['command']
            }
            
        except Exception as e:
            error_msg = f"重新載入配置失敗: {str(e)}"
            print(f"❌ {error_msg}")
            return {
                'success': False,
                'error': error_msg
            }
    
    def read_classification_config(self) -> Dict[str, Any]:
        """讀取分類配置檔案 - 從condition資料夾讀取"""
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            condition_dir = os.path.join(current_dir, "condition")
            
            if not os.path.exists(condition_dir):
                return {'error': 'condition資料夾不存在'}
            
            # 搜尋JSON檔案
            json_files = [f for f in os.listdir(condition_dir) if f.lower().endswith('.json')]
            
            if not json_files:
                return {'error': 'condition資料夾中沒有JSON配置檔案'}
            
            # 使用第一個JSON檔案
            json_files.sort()
            config_file = json_files[0]
            config_path = os.path.join(condition_dir, config_file)
            
            with open(config_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            # 添加檔案資訊
            config_data['_file_info'] = {
                'current_file': config_file,
                'available_files': json_files,
                'total_files': len(json_files)
            }
            
            return config_data
        except Exception as e:
            return {'error': str(e)}
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.status_monitor_thread = threading.Thread(target=self.monitor_status, daemon=True)
        self.status_monitor_thread.start()
        print("狀態監控已啟動")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        self.monitoring = False
        print("狀態監控已停止")
    
    def monitor_status(self):
        """狀態監控線程"""
        while self.monitoring:
            try:
                if self.connected_to_server:
                    status = self.get_system_status()
                    self.socketio.emit('status_update', status)
            except Exception as e:
                print(f"狀態監控異常: {e}")
            
            time.sleep(2)  # 2秒更新一次
    
    def create_templates_directory(self):
        """創建templates目錄和HTML檔案"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        templates_dir = os.path.join(current_dir, 'templates')
        
        if not os.path.exists(templates_dir):
            os.makedirs(templates_dir)
            print(f"已創建templates目錄: {templates_dir}")
    
    def run(self):
        """運行Web應用"""
        print("=" * 60)
        print("CCD2圖像分類Web控制應用 (純Modbus TCP Client - 修正版)")
        print("=" * 60)
        
        # 創建templates目錄
        self.create_templates_directory()
        
        # 自動連接主服務器
        print("嘗試自動連接主服務器...")
        result = self.connect_modbus_server()
        if result['success']:
            self.start_monitoring()
            print("✅ 自動連接成功，狀態監控已啟動")
        else:
            print(f"⚠️ 自動連接失敗: {result.get('error', '未知錯誤')}")
        
        web_config = self.config['web_server']
        print(f"Web服務器啟動 - http://{web_config['host']}:{web_config['port']}")
        print(f"主Modbus服務器: {self.config['tcp_server']['host']}:{self.config['tcp_server']['port']}")
        print(f"CCD2模組基地址: {self.base_address}")
        print("架構: CCD2_app → 主Modbus服務器 → CCD2主模組 → 相機(192.168.1.9)")
        print("功能列表:")
        print("  - CCD2主模組寄存器監控")
        print("  - 圖像分類檢測控制")
        print("  - 處理參數調整")
        print("  - ROI區域設定")
        print("  - JSON配置管理")
        print("  - 分類結果查看")
        print("  - 特徵數值監控")
        print("  - ⭐ 新增：調試功能 (寄存器檢查、連接測試)")
        print("")
        print("調試API:")
        print(f"  - 連接狀態: http://{web_config['host']}:{web_config['port']}/api/debug/connection")
        print(f"  - 寄存器狀態: http://{web_config['host']}:{web_config['port']}/api/debug/registers")
        print(f"  - 測試寫入: POST http://{web_config['host']}:{web_config['port']}/api/test/write_command")
        print("按 Ctrl+C 停止應用")
        
        try:
            self.socketio.run(
                self.app,
                host=web_config['host'],
                port=web_config['port'],
                debug=web_config['debug'],
                allow_unsafe_werkzeug=True
            )
        except Exception as e:
            print(f"Web服務器啟動失敗: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理資源"""
        print("正在清理資源...")
        self.stop_monitoring()
        if self.modbus_client:
            try:
                self.modbus_client.close()
                print("✅ 主Modbus連接已安全斷開")
            except:
                pass
        print("✅ 資源清理完成")


def main():
    """主函數"""
    app = CCD2WebApp()
    
    try:
        app.run()
    except KeyboardInterrupt:
        print("\n收到中斷信號，正在關閉...")
    except Exception as e:
        print(f"應用運行異常: {e}")
    finally:
        app.cleanup()
        print("應用已安全關閉")


if __name__ == '__main__':
    main()