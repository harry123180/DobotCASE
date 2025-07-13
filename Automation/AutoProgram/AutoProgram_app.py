#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_app.py - AutoProgram Web控制界面 (修正版)
提供AutoProgram協調控制、狀態監控、手動操作等功能
基於Flask + SocketIO架構
針對1300基地址、AutoFeeding協調控制
"""

import os
import time
import json
import threading
from typing import Dict, Any, Optional
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException, ConnectionException

# 創建Flask應用
app = Flask(__name__)
app.config['SECRET_KEY'] = 'autoprogram_v1.1'
socketio = SocketIO(app, cors_allowed_origins="*")

class AutoProgramWebController:
    """AutoProgram Web控制器 (修正版)"""
    
    def __init__(self, modbus_host="127.0.0.1", modbus_port=502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 狀態監控執行緒
        self.monitor_thread = None
        self.monitoring = False
        
        # 寄存器地址映射 (修正版)
        self.REGISTERS = {
            # AutoProgram狀態 (1300-1319)
            'SYSTEM_STATUS': 1300,           # 系統狀態
            'ROBOT_STATUS': 1301,            # 機械臂狀態
            'PREPARE_DONE': 1302,            # prepare_done狀態
            'FEEDING_READY': 1303,           # feeding_ready狀態
            'FLOW1_COMPLETE': 1304,          # Flow1完成狀態
            'FLOW5_COMPLETE': 1305,          # Flow5完成狀態
            'COORDINATION_CYCLE_COUNT': 1306, # 協調週期計數
            'FLOW1_TRIGGER_COUNT': 1307,     # Flow1觸發次數
            'FLOW5_COMPLETE_COUNT': 1308,    # Flow5完成次數
            'ERROR_CODE': 1309,              # 錯誤代碼
            
            # AutoProgram控制 (1320-1339)
            'SYSTEM_CONTROL': 1320,          # 系統控制
            'FLOW1_CONTROL': 1321,           # Flow1控制
            'ERROR_CLEAR': 1322,             # 錯誤清除
            'FORCE_RESET': 1323,             # 強制重置
            
            # 機械臂協調 (1340-1359)
            'FLOW1_EXECUTE_TRIGGER': 1340,   # Flow1執行觸發
            'TARGET_X_HIGH': 1341,           # 目標座標X高位
            'TARGET_X_LOW': 1342,            # 目標座標X低位
            'TARGET_Y_HIGH': 1343,           # 目標座標Y高位
            'TARGET_Y_LOW': 1344,            # 目標座標Y低位
            
            # AutoFeeding模組 (900-999)
            'AF_MODULE_STATUS': 900,         # AutoFeeding模組狀態
            'AF_CYCLE_COUNT': 901,           # 檢測週期計數
            'AF_CASE_F_FOUND_COUNT': 902,    # CASE_F找到次數
            'AF_FLOW4_TRIGGER_COUNT': 903,   # Flow4觸發次數
            'AF_VP_VIBRATION_COUNT': 904,    # VP震動次數
            'AF_FEEDING_COMPLETE': 940,      # 入料完成標誌
            'AF_RUN_CONTROL': 920,           # AutoFeeding運行控制
            'AF_PAUSE_CONTROL': 921,         # AutoFeeding暫停控制
            
            # Dobot M1Pro (1200-1299)
            'DOBOT_MOTION_STATUS': 1200,     # 運動狀態寄存器
            'DOBOT_CURRENT_FLOW': 1201,      # 當前運動Flow
            'DOBOT_MOTION_PROGRESS': 1202,   # 運動進度
            'DOBOT_FLOW1_COMPLETE': 1204,    # Flow1完成狀態
            'DOBOT_FLOW5_COMPLETE': 1206,    # Flow5完成狀態
            'DOBOT_FLOW1_CONTROL': 1240,     # Flow1控制
            'DOBOT_FLOW5_CONTROL': 1242,     # Flow5控制
            
            # CCD1檢測結果
            'CCD1_STATUS': 201,              # CCD1狀態
            'CASE_F_COUNT': 240,             # CASE_F數量
            'CASE_B_COUNT': 241,             # CASE_B數量
            'STACK_COUNT': 242,              # STACK數量
            'TOTAL_DETECTIONS': 243,         # 總檢測數量
            
            # VP狀態
            'VP_STATUS': 300,                # VP模組狀態
            'VP_DEVICE_CONNECTION': 301,     # VP設備連接
            
            # Flow4直振供應
            'FLOW4_CONTROL': 448,            # Flow4控制
        }
        
        print("AutoProgram Web控制器初始化完成 (修正版)")
    
    def connect_modbus(self) -> bool:
        """連接Modbus服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"✓ Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                # 啟動狀態監控
                self.start_monitoring()
            else:
                print(f"✗ Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
            
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect_modbus(self):
        """斷開Modbus連接"""
        self.stop_monitoring()
        
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            return None
            
        except Exception:
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            
            return not result.isError()
            
        except Exception:
            return False
    
    def read_32bit_coordinate(self, high_reg: str, low_reg: str) -> float:
        """讀取32位座標"""
        high_val = self.read_register(high_reg) or 0
        low_val = self.read_register(low_reg) or 0
        
        # 合併32位值
        combined = (high_val << 16) + low_val
        
        # 處理補碼(負數)
        if combined >= 2147483648:  # 2^31
            combined = combined - 4294967296  # 2^32
        
        # 轉換為毫米(除以100)
        return combined / 100.0
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        status = {
            # 連接狀態
            'connected': self.connected,
            'modbus_host': self.modbus_host,
            'modbus_port': self.modbus_port,
            
            # AutoProgram狀態
            'system_status': self.read_register('SYSTEM_STATUS') or 0,
            'robot_status': self.read_register('ROBOT_STATUS') or 0,
            'prepare_done': bool(self.read_register('PREPARE_DONE')),
            'feeding_ready': bool(self.read_register('FEEDING_READY')),
            'flow1_complete': bool(self.read_register('FLOW1_COMPLETE')),
            'flow5_complete': bool(self.read_register('FLOW5_COMPLETE')),
            'coordination_cycle_count': self.read_register('COORDINATION_CYCLE_COUNT') or 0,
            'flow1_trigger_count': self.read_register('FLOW1_TRIGGER_COUNT') or 0,
            'flow5_complete_count': self.read_register('FLOW5_COMPLETE_COUNT') or 0,
            'error_code': self.read_register('ERROR_CODE') or 0,
            
            # AutoFeeding狀態
            'af_module_status': self.read_register('AF_MODULE_STATUS') or 0,
            'af_cycle_count': self.read_register('AF_CYCLE_COUNT') or 0,
            'af_case_f_found_count': self.read_register('AF_CASE_F_FOUND_COUNT') or 0,
            'af_flow4_trigger_count': self.read_register('AF_FLOW4_TRIGGER_COUNT') or 0,
            'af_vp_vibration_count': self.read_register('AF_VP_VIBRATION_COUNT') or 0,
            'af_feeding_complete': bool(self.read_register('AF_FEEDING_COMPLETE')),
            
            # Dobot M1Pro狀態
            'dobot_motion_status': self.read_register('DOBOT_MOTION_STATUS') or 0,
            'dobot_current_flow': self.read_register('DOBOT_CURRENT_FLOW') or 0,
            'dobot_motion_progress': self.read_register('DOBOT_MOTION_PROGRESS') or 0,
            'dobot_flow1_complete': bool(self.read_register('DOBOT_FLOW1_COMPLETE')),
            'dobot_flow5_complete': bool(self.read_register('DOBOT_FLOW5_COMPLETE')),
            
            # CCD1檢測結果
            'ccd1_status': self.read_register('CCD1_STATUS') or 0,
            'case_f_count': self.read_register('CASE_F_COUNT') or 0,
            'case_b_count': self.read_register('CASE_B_COUNT') or 0,
            'stack_count': self.read_register('STACK_COUNT') or 0,
            'total_detections': self.read_register('TOTAL_DETECTIONS') or 0,
            
            # VP狀態
            'vp_status': self.read_register('VP_STATUS') or 0,
            'vp_device_connection': bool(self.read_register('VP_DEVICE_CONNECTION')),
            
            # Flow4狀態
            'flow4_control': self.read_register('FLOW4_CONTROL') or 0,
            
            # 目標座標
            'target_x': self.read_32bit_coordinate('TARGET_X_HIGH', 'TARGET_X_LOW'),
            'target_y': self.read_32bit_coordinate('TARGET_Y_HIGH', 'TARGET_Y_LOW'),
            
            # 時間戳
            'timestamp': time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        # 判斷系統運行狀態
        status['system_running'] = self._get_system_running_status(status)
        status['feeding_process_status'] = self._get_feeding_process_status(status)
        
        return status
    
    def _get_system_running_status(self, status: Dict) -> str:
        """判斷系統運行狀態"""
        system_status = status['system_status']
        
        if system_status == 0:
            return "系統停止"
        elif system_status == 1:
            return "運行中"
        elif system_status == 2:
            return "Flow1執行"
        elif system_status == 3:
            return "Flow5執行"
        elif system_status == 4:
            return "錯誤"
        else:
            return f"未知狀態({system_status})"
    
    def _get_feeding_process_status(self, status: Dict) -> str:
        """判斷入料流程狀態"""
        af_status = status['af_module_status']
        
        if af_status == 0:
            return "入料程序停止"
        elif af_status == 1:
            return "入料程序運行中"
        elif af_status == 2:
            return "入料程序暫停"
        elif af_status == 3:
            return "CCD1檢測中"
        elif af_status == 4:
            return "VP震動中"
        elif af_status == 5:
            return "入料程序錯誤"
        else:
            return f"未知狀態({af_status})"
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("✓ 狀態監控已啟動")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        if self.monitoring:
            self.monitoring = False
            if self.monitor_thread and self.monitor_thread.is_alive():
                self.monitor_thread.join(timeout=2.0)
            print("狀態監控已停止")
    
    def _monitor_loop(self):
        """狀態監控循環"""
        while self.monitoring and self.connected:
            try:
                # 獲取系統狀態
                status = self.get_system_status()
                
                # 通過SocketIO發送狀態更新
                socketio.emit('status_update', status)
                
                # 2秒間隔
                time.sleep(2.0)
                
            except Exception as e:
                print(f"狀態監控異常: {e}")
                time.sleep(5.0)

# 創建全局控制器實例
controller = AutoProgramWebController()

# ==================== Flask路由 ====================

@app.route('/')
def index():
    """主頁面"""
    return render_template('AutoProgram.html')

@app.route('/test')
def test():
    return "AutoProgram Web Server is running! (修正版)"

@app.route('/api/connect', methods=['POST'])
def connect_modbus():
    """連接Modbus服務器"""
    try:
        success = controller.connect_modbus()
        
        return jsonify({
            'success': success,
            'message': '連接成功' if success else '連接失敗',
            'status': controller.get_system_status() if success else None
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/disconnect', methods=['POST'])
def disconnect_modbus():
    """斷開Modbus連接"""
    try:
        controller.disconnect_modbus()
        
        return jsonify({
            'success': True,
            'message': '連接已斷開'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/status', methods=['GET'])
def get_status():
    """獲取系統狀態"""
    try:
        status = controller.get_system_status()
        return jsonify({'success': True, 'status': status})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/system', methods=['POST'])
def control_system():
    """控制AutoProgram系統"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'start' or 'stop'
        
        if action == 'start':
            success = controller.write_register('SYSTEM_CONTROL', 1)
            message = 'AutoProgram系統已啟動 (1320=1)' if success else 'AutoProgram系統啟動失敗'
        elif action == 'stop':
            success = controller.write_register('SYSTEM_CONTROL', 0)
            message = 'AutoProgram系統已停止 (1320=0)' if success else 'AutoProgram系統停止失敗'
        else:
            return jsonify({'success': False, 'message': '無效的操作'})
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/autofeeding', methods=['POST'])
def control_autofeeding():
    """控制AutoFeeding模組"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'start', 'stop', 'pause', 'resume'
        
        if action == 'start':
            success = controller.write_register('AF_RUN_CONTROL', 1)
            message = 'AutoFeeding已啟動 (920=1)' if success else 'AutoFeeding啟動失敗'
        elif action == 'stop':
            success = controller.write_register('AF_RUN_CONTROL', 0)
            message = 'AutoFeeding已停止 (920=0)' if success else 'AutoFeeding停止失敗'
        elif action == 'pause':
            success = controller.write_register('AF_PAUSE_CONTROL', 1)
            message = 'AutoFeeding已暫停 (921=1)' if success else 'AutoFeeding暫停失敗'
        elif action == 'resume':
            success = controller.write_register('AF_PAUSE_CONTROL', 0)
            message = 'AutoFeeding已恢復 (921=0)' if success else 'AutoFeeding恢復失敗'
        else:
            return jsonify({'success': False, 'message': '無效的操作'})
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/flow1', methods=['POST'])
def control_flow1():
    """控制Flow1"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'trigger', 'clear'
        
        if action == 'trigger':
            success = controller.write_register('FLOW1_CONTROL', 1)
            message = 'Flow1已觸發 (1321=1)' if success else 'Flow1觸發失敗'
        elif action == 'clear':
            success = controller.write_register('FLOW1_COMPLETE', 0)
            message = 'Flow1完成狀態已清除 (1304=0)' if success else 'Flow1完成狀態清除失敗'
        else:
            return jsonify({'success': False, 'message': '無效的操作'})
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})
# 在AutoProgram_app.py中添加這個路由

@app.route('/api/control/auto_handshake', methods=['POST'])
def auto_handshake():
    """自動交握 - Flow1完成後自動觸發Flow5"""
    try:
        logMessage = []
        
        # 1. 檢查Flow1完成狀態
        flow1_complete = controller.read_register('DOBOT_FLOW1_COMPLETE')
        logMessage.append(f"檢查Flow1完成狀態: {flow1_complete}")
        
        if not flow1_complete:
            return jsonify({
                'success': False,
                'message': 'Flow1尚未完成，無法執行自動交握'
            })
        
        logMessage.append("✓ Flow1已完成，開始自動交握流程")
        
        # 2. 清除Flow1完成狀態
        clear_success = controller.write_register('DOBOT_FLOW1_COMPLETE', 0)
        if clear_success:
            logMessage.append("✓ Flow1完成狀態已清除")
        else:
            logMessage.append("✗ Flow1完成狀態清除失敗")
            
        # 3. 觸發Flow5
        trigger_success = controller.write_register('DOBOT_FLOW5_CONTROL', 1)
        if trigger_success:
            logMessage.append("✓ Flow5已觸發")
        else:
            logMessage.append("✗ Flow5觸發失敗")
            
        # 4. 等待一小段時間後清除Flow5控制狀態
        import time
        time.sleep(0.1)
        controller.write_register('DOBOT_FLOW5_CONTROL', 0)
        logMessage.append("✓ Flow5控制狀態已清除")
        
        success = clear_success and trigger_success
        message = " | ".join(logMessage)
        
        return jsonify({
            'success': success,
            'message': f"自動交握{'成功' if success else '部分失敗'}: {message}"
        })
        
    except Exception as e:
        return jsonify({
            'success': False, 
            'message': f'自動交握執行失敗: {str(e)}'
        })
@app.route('/api/control/dobot_flow1', methods=['POST'])
def control_dobot_flow1():
    """直接控制Dobot Flow1"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'trigger', 'clear'
        
        if action == 'trigger':
            success = controller.write_register('DOBOT_FLOW1_CONTROL', 1)
            message = 'Dobot Flow1已觸發 (1240=1)' if success else 'Dobot Flow1觸發失敗'
        elif action == 'clear':
            success = controller.write_register('DOBOT_FLOW1_CONTROL', 0)
            message = 'Dobot Flow1控制已清除 (1240=0)' if success else 'Dobot Flow1控制清除失敗'
        else:
            return jsonify({'success': False, 'message': '無效的操作'})
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/dobot_flow5', methods=['POST'])
def control_dobot_flow5():
    """直接控制Dobot Flow5"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'trigger', 'clear'
        
        if action == 'trigger':
            success = controller.write_register('DOBOT_FLOW5_CONTROL', 1)
            message = 'Dobot Flow5已觸發 (1242=1)' if success else 'Dobot Flow5觸發失敗'
        elif action == 'clear':
            success = controller.write_register('DOBOT_FLOW5_CONTROL', 0)
            message = 'Dobot Flow5控制已清除 (1242=0)' if success else 'Dobot Flow5控制清除失敗'
        else:
            return jsonify({'success': False, 'message': '無效的操作'})
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/error_clear', methods=['POST'])
def error_clear():
    """清除錯誤"""
    try:
        success = controller.write_register('ERROR_CLEAR', 1)
        message = '錯誤已清除 (1322=1)' if success else '錯誤清除失敗'
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

# ==================== SocketIO事件處理 ====================

@socketio.on('connect')
def handle_connect():
    """客戶端連接"""
    print("客戶端已連接")
    emit('status_update', controller.get_system_status())

@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開連接"""
    print("客戶端已斷開連接")

@socketio.on('request_status')
def handle_request_status():
    """請求狀態更新"""
    emit('status_update', controller.get_system_status())

def main():
    """主函數"""
    print("=" * 60)
    print("AutoProgram Web控制界面啟動中... (修正版)")
    print("AutoProgram機械臂協調控制與監控")
    print("=" * 60)
    
    # 檢查模板文件
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    template_file = os.path.join(template_dir, 'AutoProgram.html')
    
    if not os.path.exists(template_dir):
        os.makedirs(template_dir)
        print(f"已創建模板目錄: {template_dir}")
    
    if not os.path.exists(template_file):
        print(f"⚠️ 警告: 模板文件不存在 - {template_file}")
        print("請確保AutoProgram.html文件在templates目錄中")
    
    try:
        print("🌐 Web服務器啟動中...")
        print("📱 訪問地址: http://localhost:5093")
        print("🎯 功能特性:")
        print("   • AutoProgram協調控制 (1300基地址)")
        print("   • AutoFeeding模組控制 (920/921)")
        print("   • Dobot M1Pro Flow控制 (1240/1242)")
        print("   • 三重交握狀態監控")
        print("   • 即時座標顯示")
        print("   • 協調週期統計")
        print("=" * 60)
        
        # 啟動Web服務器
        socketio.run(
            app,
            host='0.0.0.0',
            port=5093,
            debug=False
        )
        
    except KeyboardInterrupt:
        print("\n收到中斷信號，正在關閉...")
    except Exception as e:
        print(f"Web服務器錯誤: {e}")
    finally:
        # 清理資源
        controller.disconnect_modbus()
        print("Web服務器已關閉")

if __name__ == '__main__':
    main()