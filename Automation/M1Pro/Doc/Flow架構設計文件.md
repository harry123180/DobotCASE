# Flow架構設計文件

## Flow架構概述

Flow架構是機械臂工作流程的執行單元，每個Flow負責完成特定的作業任務。Flow與Dobot_main採用分工合作模式：Main負責指令排程和狀態管理，Flow負責具體的執行邏輯和步驟控制。

## Flow與Main分工

### Dobot_main職責
- **指令接收**: 監控Modbus寄存器，接收PLC指令
- **指令排程**: 將指令加入優先權佇列，分派給相應執行緒
- **狀態管理**: 維護系統狀態，更新狀態寄存器
- **資源協調**: 管理機械臂API的共用存取
- **錯誤處理**: 系統級錯誤處理和恢復機制
- **外部模組交握**: 與CCD、VP、夾爪等模組通訊

### Flow職責  
- **流程邏輯**: 定義具體的工作流程和步驟順序
- **步驟執行**: 執行每個步驟的具體操作
- **參數管理**: 管理流程相關的參數和配置
- **進度追蹤**: 追蹤執行進度和狀態變化
- **結果回報**: 回報執行結果和錯誤資訊
- **局部錯誤處理**: Flow級別的錯誤處理和重試

## 統一Flow介面設計

### 1. FlowExecutor基底類別

```python
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

class FlowStatus(Enum):
    IDLE = 0
    RUNNING = 1  
    COMPLETED = 2
    ERROR = 3
    PAUSED = 4

@dataclass
class FlowResult:
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 0
    flow_data: Dict[str, Any] = None

class FlowExecutor(ABC):
    """Flow執行器基底類別"""
    
    def __init__(self, flow_id: int, flow_name: str):
        self.flow_id = flow_id
        self.flow_name = flow_name
        self.status = FlowStatus.IDLE
        self.current_step = 0
        self.total_steps = 0
        self.start_time = 0.0
        self.last_error = ""
        
        # 共用資源 (由Main傳入)
        self.robot = None
        self.state_machine = None
        self.external_modules = {}
        
    def initialize(self, robot, state_machine, external_modules):
        """初始化Flow (由Main呼叫)"""
        self.robot = robot
        self.state_machine = state_machine  
        self.external_modules = external_modules
        
    @abstractmethod
    def execute(self) -> FlowResult:
        """執行Flow主邏輯"""
        pass
        
    @abstractmethod
    def pause(self) -> bool:
        """暫停Flow"""
        pass
        
    @abstractmethod  
    def resume(self) -> bool:
        """恢復Flow"""
        pass
        
    @abstractmethod
    def stop(self) -> bool:
        """停止Flow"""
        pass
        
    @abstractmethod
    def get_progress(self) -> int:
        """取得執行進度 (0-100)"""
        pass
        
    def get_status_info(self) -> Dict[str, Any]:
        """取得狀態資訊"""
        return {
            'flow_id': self.flow_id,
            'flow_name': self.flow_name,
            'status': self.status.value,
            'current_step': self.current_step,
            'total_steps': self.total_steps,
            'progress': self.get_progress(),
            'last_error': self.last_error
        }
```

### 2. 外部模組交握介面

```python
class ExternalModuleInterface:
    """外部模組交握介面"""
    
    def __init__(self, module_name: str, base_address: int, state_machine):
        self.module_name = module_name
        self.base_address = base_address
        self.state_machine = state_machine
        
    def send_command(self, command_code: int, params: Dict = None, timeout: float = 10.0) -> bool:
        """發送指令並等待完成"""
        try:
            # 1. 檢查Ready狀態
            if not self._check_ready():
                return False
                
            # 2. 設定參數 (如果有)
            if params:
                self._set_parameters(params)
                
            # 3. 發送指令
            control_offset = self._get_control_register_offset()
            self.state_machine.write_register(control_offset, command_code)
            
            # 4. 等待完成
            return self._wait_completion(timeout)
            
        except Exception as e:
            print(f"{self.module_name}指令執行失敗: {e}")
            return False
            
    def _check_ready(self) -> bool:
        """檢查模組Ready狀態"""
        status_offset = self._get_status_register_offset()
        status = self.state_machine.read_register(status_offset)
        return status and (status & 1)  # bit0=Ready
        
    def _wait_completion(self, timeout: float) -> bool:
        """等待指令完成"""
        import time
        start_time = time.time()
        status_offset = self._get_status_register_offset()
        
        while time.time() - start_time < timeout:
            status = self.state_machine.read_register(status_offset)
            if status and not (status & 2):  # bit1=Running=0
                return True
            time.sleep(0.1)
            
        return False  # 超時
        
    def _get_control_register_offset(self) -> int:
        """取得控制寄存器偏移 (各模組不同)"""
        module_offsets = {
            'CCD1': 0,    # 200-200=0
            'VP': 20,     # 320-300=20  
            'GRIPPER': 120, # 520-400=120
            'CCD3': 400   # 800-400=400
        }
        return module_offsets.get(self.module_name, 0)
        
    def _get_status_register_offset(self) -> int:
        """取得狀態寄存器偏移"""
        module_offsets = {
            'CCD1': 1,    # 201-200=1
            'VP': 0,      # 300-300=0
            'GRIPPER': 100, # 500-400=100  
            'CCD3': 401   # 801-400=401
        }
        return module_offsets.get(self.module_name, 1)
```

## 三種Flow實現範例

### 1. 運動Flow執行器 (MotionFlowExecutor)

```python
class MotionFlowExecutor(FlowExecutor):
    """運動控制Flow執行器"""
    
    def __init__(self, flow_id: int, flow_name: str):
        super().__init__(flow_id, flow_name)
        self.motion_steps = []
        self.speed_ratio = 100
        
    def add_motion_step(self, step_type: str, params: Dict):
        """新增運動步驟"""
        self.motion_steps.append({
            'type': step_type,
            'params': params,
            'completed': False
        })
        self.total_steps = len(self.motion_steps)
        
    def execute(self) -> FlowResult:
        """執行運動Flow"""
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        try:
            for i, step in enumerate(self.motion_steps):
                if self.status != FlowStatus.RUNNING:
                    break
                    
                self.current_step = i + 1
                
                if not self._execute_motion_step(step):
                    return FlowResult(
                        success=False,
                        error_message=f"步驟{self.current_step}執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step - 1,
                        total_steps=self.total_steps
                    )
                    
                step['completed'] = True
                
            self.status = FlowStatus.COMPLETED
            return FlowResult(
                success=True,
                execution_time=time.time() - self.start_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            self.last_error = str(e)
            return FlowResult(
                success=False,
                error_message=str(e),
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
            
    def _execute_motion_step(self, step: Dict) -> bool:
        """執行單一運動步驟"""
        step_type = step['type']
        params = step['params']
        
        if step_type == 'move_j':
            return self._move_j(params)
        elif step_type == 'move_l':
            return self._move_l(params)
        elif step_type == 'move_to_point':
            return self._move_to_point(params)
        elif step_type == 'set_speed':
            return self._set_speed(params)
        elif step_type == 'wait':
            return self._wait(params)
        else:
            print(f"未知運動步驟類型: {step_type}")
            return False
            
    def _move_j(self, params: Dict) -> bool:
        """關節運動"""
        try:
            x = params.get('x', 0)
            y = params.get('y', 0) 
            z = params.get('z', 0)
            r = params.get('r', 0)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.move_api.MovJ(x, y, z, r)
                print(f"MovJ({x}, {y}, {z}, {r}) -> {result}")
                return "0" in str(result)  # 成功返回包含0
            return False
        except Exception as e:
            print(f"MovJ執行失敗: {e}")
            return False
            
    def _move_l(self, params: Dict) -> bool:
        """直線運動"""
        try:
            x = params.get('x', 0)
            y = params.get('y', 0)
            z = params.get('z', 0) 
            r = params.get('r', 0)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.move_api.MovL(x, y, z, r)
                print(f"MovL({x}, {y}, {z}, {r}) -> {result}")
                return "0" in str(result)
            return False
        except Exception as e:
            print(f"MovL執行失敗: {e}")
            return False
            
    def _move_to_point(self, params: Dict) -> bool:
        """移動到預設點位"""
        try:
            point_name = params.get('point_name')
            move_type = params.get('move_type', 'J')  # J或L
            
            if self.robot and hasattr(self.robot, 'points_manager'):
                point = self.robot.points_manager.get_point(point_name)
                if point:
                    if move_type == 'J':
                        return self._move_j({'x': point.x, 'y': point.y, 'z': point.z, 'r': point.r})
                    else:
                        return self._move_l({'x': point.x, 'y': point.y, 'z': point.z, 'r': point.r})
            return False
        except Exception as e:
            print(f"移動到點位{point_name}失敗: {e}")
            return False
            
    def _set_speed(self, params: Dict) -> bool:
        """設定速度"""
        try:
            speed = params.get('speed', 50)
            if self.robot and self.robot.is_connected:
                result = self.robot.dashboard_api.SpeedFactor(speed)
                print(f"設定速度{speed}% -> {result}")
                return "0" in str(result)
            return False
        except Exception as e:
            print(f"設定速度失敗: {e}")
            return False
            
    def _wait(self, params: Dict) -> bool:
        """等待"""
        try:
            duration = params.get('duration', 1.0)
            time.sleep(duration)
            print(f"等待{duration}秒")
            return True
        except Exception as e:
            print(f"等待失敗: {e}")
            return False
            
    def pause(self) -> bool:
        """暫停Flow"""
        if self.status == FlowStatus.RUNNING:
            self.status = FlowStatus.PAUSED
            return True
        return False
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
```

### 2. DIO Flow執行器 (DIOFlowExecutor)

```python
class DIOFlowExecutor(FlowExecutor):
    """DIO控制Flow執行器"""
    
    def __init__(self, flow_id: int, flow_name: str):
        super().__init__(flow_id, flow_name)
        self.dio_sequence = []
        
    def add_dio_step(self, step_type: str, params: Dict):
        """新增DIO步驟"""
        self.dio_sequence.append({
            'type': step_type,
            'params': params,
            'completed': False
        })
        self.total_steps = len(self.dio_sequence)
        
    def execute(self) -> FlowResult:
        """執行DIO Flow"""
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        try:
            for i, step in enumerate(self.dio_sequence):
                if self.status != FlowStatus.RUNNING:
                    break
                    
                self.current_step = i + 1
                
                if not self._execute_dio_step(step):
                    return FlowResult(
                        success=False,
                        error_message=f"DIO步驟{self.current_step}執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step - 1,
                        total_steps=self.total_steps
                    )
                    
                step['completed'] = True
                
            self.status = FlowStatus.COMPLETED
            return FlowResult(
                success=True,
                execution_time=time.time() - self.start_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            self.last_error = str(e)
            return FlowResult(
                success=False,
                error_message=str(e),
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
            
    def _execute_dio_step(self, step: Dict) -> bool:
        """執行DIO步驟"""
        step_type = step['type']
        params = step['params']
        
        if step_type == 'set_do':
            return self._set_digital_output(params)
        elif step_type == 'check_di':
            return self._check_digital_input(params)
        elif step_type == 'pulse_do':
            return self._pulse_digital_output(params)
        elif step_type == 'wait_di':
            return self._wait_digital_input(params)
        elif step_type == 'sequence':
            return self._execute_dio_sequence(params)
        else:
            print(f"未知DIO步驟類型: {step_type}")
            return False
            
    def _set_digital_output(self, params: Dict) -> bool:
        """設定數位輸出"""
        try:
            pin = params.get('pin', 1)
            value = params.get('value', 0)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.dashboard_api.DO(pin, value)
                print(f"DO({pin}, {value}) -> {result}")
                return "0" in str(result)
            return False
        except Exception as e:
            print(f"設定DO失敗: {e}")
            return False
            
    def _check_digital_input(self, params: Dict) -> bool:
        """檢查數位輸入"""
        try:
            pin = params.get('pin', 1)
            expected_value = params.get('expected_value', 1)
            
            if self.robot and self.robot.is_connected:
                result = self.robot.dashboard_api.DI(pin)
                print(f"DI({pin}) -> {result}")
                
                # 解析DI回應
                if f",{expected_value}," in result:
                    return True
                    
            return False
        except Exception as e:
            print(f"檢查DI失敗: {e}")
            return False
            
    def _pulse_digital_output(self, params: Dict) -> bool:
        """脈衝輸出"""
        try:
            pin = params.get('pin', 1)
            pulse_width = params.get('pulse_width', 100)  # 毫秒
            
            # 設定為高
            if not self._set_digital_output({'pin': pin, 'value': 1}):
                return False
                
            # 等待
            time.sleep(pulse_width / 1000.0)
            
            # 設定為低  
            return self._set_digital_output({'pin': pin, 'value': 0})
            
        except Exception as e:
            print(f"脈衝輸出失敗: {e}")
            return False
            
    def _wait_digital_input(self, params: Dict) -> bool:
        """等待數位輸入"""
        try:
            pin = params.get('pin', 1)
            expected_value = params.get('expected_value', 1)
            timeout = params.get('timeout', 10.0)
            
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self._check_digital_input({'pin': pin, 'expected_value': expected_value}):
                    return True
                time.sleep(0.1)
                
            return False
        except Exception as e:
            print(f"等待DI失敗: {e}")
            return False
            
    def _execute_dio_sequence(self, params: Dict) -> bool:
        """執行DIO序列"""
        try:
            sequence = params.get('sequence', [])
            
            for seq_step in sequence:
                step_type = seq_step.get('type')
                step_params = seq_step.get('params', {})
                
                if not self._execute_dio_step({'type': step_type, 'params': step_params}):
                    return False
                    
                # 序列間延遲
                delay = seq_step.get('delay', 0)
                if delay > 0:
                    time.sleep(delay / 1000.0)
                    
            return True
        except Exception as e:
            print(f"執行DIO序列失敗: {e}")
            return False
            
    def pause(self) -> bool:
        if self.status == FlowStatus.RUNNING:
            self.status = FlowStatus.PAUSED
            return True
        return False
        
    def resume(self) -> bool:
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            return True
        return False
        
    def stop(self) -> bool:
        self.status = FlowStatus.ERROR
        return True
        
    def get_progress(self) -> int:
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
```

### 3. 外部模組Flow執行器 (ExternalModuleFlowExecutor)

```python
class ExternalModuleFlowExecutor(FlowExecutor):
    """外部模組交握Flow執行器"""
    
    def __init__(self, flow_id: int, flow_name: str):
        super().__init__(flow_id, flow_name)
        self.module_steps = []
        
    def add_module_step(self, module_name: str, operation: str, params: Dict = None):
        """新增模組操作步驟"""
        self.module_steps.append({
            'module': module_name,
            'operation': operation,
            'params': params or {},
            'completed': False
        })
        self.total_steps = len(self.module_steps)
        
    def execute(self) -> FlowResult:
        """執行外部模組Flow"""
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        try:
            for i, step in enumerate(self.module_steps):
                if self.status != FlowStatus.RUNNING:
                    break
                    
                self.current_step = i + 1
                
                if not self._execute_module_step(step):
                    return FlowResult(
                        success=False,
                        error_message=f"模組步驟{self.current_step}執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step - 1,
                        total_steps=self.total_steps
                    )
                    
                step['completed'] = True
                
            self.status = FlowStatus.COMPLETED
            return FlowResult(
                success=True,
                execution_time=time.time() - self.start_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            self.last_error = str(e)
            return FlowResult(
                success=False,
                error_message=str(e),
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
            
    def _execute_module_step(self, step: Dict) -> bool:
        """執行模組步驟"""
        module_name = step['module']
        operation = step['operation']
        params = step['params']
        
        # 取得模組介面
        if module_name not in self.external_modules:
            print(f"模組{module_name}未配置")
            return False
            
        module = self.external_modules[module_name]
        
        # 根據模組和操作執行相應邏輯
        if module_name == 'CCD1':
            return self._handle_ccd1_operation(module, operation, params)
        elif module_name == 'VP':
            return self._handle_vp_operation(module, operation, params)
        elif module_name == 'GRIPPER':
            return self._handle_gripper_operation(module, operation, params)
        elif module_name == 'CCD3':
            return self._handle_ccd3_operation(module, operation, params)
        else:
            print(f"未支援的模組: {module_name}")
            return False
            
    def _handle_ccd1_operation(self, module, operation: str, params: Dict) -> bool:
        """處理CCD1操作"""
        try:
            if operation == 'detect':
                # 拍照+檢測指令 (16)
                return module.send_command(16, params, timeout=10.0)
            elif operation == 'capture':
                # 拍照指令 (8)
                return module.send_command(8, params, timeout=5.0)
            elif operation == 'read_results':
                # 讀取檢測結果
                return self._read_ccd1_results(module, params)
            else:
                print(f"未知CCD1操作: {operation}")
                return False
        except Exception as e:
            print(f"CCD1操作失敗: {e}")
            return False
            
    def _handle_vp_operation(self, module, operation: str, params: Dict) -> bool:
        """處理VP操作"""
        try:
            if operation == 'vibrate':
                # 震動指令 (5)
                return module.send_command(5, params, timeout=5.0)
            elif operation == 'stop':
                # 停止指令 (6)
                return module.send_command(6, params, timeout=2.0)
            elif operation == 'light':
                # 背光控制指令 (4)
                return module.send_command(4, params, timeout=2.0)
            else:
                print(f"未知VP操作: {operation}")
                return False
        except Exception as e:
            print(f"VP操作失敗: {e}")
            return False
            
    def _handle_gripper_operation(self, module, operation: str, params: Dict) -> bool:
        """處理夾爪操作"""
        try:
            if operation == 'open':
                # 開啟指令 (7)
                return module.send_command(7, params, timeout=3.0)
            elif operation == 'close':
                # 關閉指令 (8)
                return module.send_command(8, params, timeout=3.0)
            elif operation == 'position':
                # 位置控制指令 (3)
                return module.send_command(3, params, timeout=5.0)
            elif operation == 'init':
                # 初始化指令 (1)
                return module.send_command(1, params, timeout=10.0)
            else:
                print(f"未知夾爪操作: {operation}")
                return False
        except Exception as e:
            print(f"夾爪操作失敗: {e}")
            return False
            
    def _handle_ccd3_operation(self, module, operation: str, params: Dict) -> bool:
        """處理CCD3操作"""
        try:
            if operation == 'angle_detect':
                # 角度檢測指令 (16)
                return module.send_command(16, params, timeout=15.0)
            elif operation == 'read_angle':
                # 讀取角度結果
                return self._read_ccd3_angle(module, params)
            else:
                print(f"未知CCD3操作: {operation}")
                return False
        except Exception as e:
            print(f"CCD3操作失敗: {e}")
            return False
            
    def _read_ccd1_results(self, module, params: Dict) -> bool:
        """讀取CCD1檢測結果"""
        try:
            # 讀取檢測數量 (240寄存器)
            count_offset = 40  # 240-200=40
            count = self.state_machine.read_register(count_offset)
            
            if count and count > 0:
                print(f"CCD1檢測到{count}個物體")
                
                # 讀取座標 (241-243為第一個物體)
                for i in range(min(count, 5)):
                    x_offset = 41 + i * 3  # 241, 244, 247...
                    y_offset = 42 + i * 3  # 242, 245, 248...
                    r_offset = 43 + i * 3  # 243, 246, 249...
                    
                    x = self.state_machine.read_register(x_offset)
                    y = self.state_machine.read_register(y_offset)
                    r = self.state_machine.read_register(r_offset)
                    
                    print(f"物體{i+1}: X={x}, Y={y}, R={r}")
                    
                return True
            else:
                print("CCD1未檢測到物體")
                return False
                
        except Exception as e:
            print(f"讀取CCD1結果失敗: {e}")
            return False
            
    def _read_ccd3_angle(self, module, params: Dict) -> bool:
        """讀取CCD3角度結果"""
        try:
            # 讀取成功標誌 (840寄存器)
            success_offset = 40  # 840-800=40
            success = self.state_machine.read_register(success_offset)
            
            if success:
                # 讀取角度 (843-844為32位角度)
                angle_h_offset = 43  # 843-800=43
                angle_l_offset = 44  # 844-800=44
                
                angle_h = self.state_machine.read_register(angle_h_offset) or 0
                angle_l = self.state_machine.read_register(angle_l_offset) or 0
                
                # 32位角度恢復
                angle_int = (angle_h << 16) | angle_l
                angle_degrees = angle_int / 100.0
                
                print(f"CCD3檢測角度: {angle_degrees}度")
                return True
            else:
                print("CCD3角度檢測失敗")
                return False
                
        except Exception as e:
            print(f"讀取CCD3角度失敗: {e}")
            return False
            
    def pause(self) -> bool:
        if self.status == FlowStatus.RUNNING:
            self.status = FlowStatus.PAUSED
            return True
        return False
        
    def resume(self) -> bool:
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            return True
        return False
        
    def stop(self) -> bool:
        self.status = FlowStatus.ERROR
        return True
        
    def get_progress(self) -> int:
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
```

## 具體Flow實現範例

### Flow1: VP視覺抓取流程

```python
class Flow1VisionPickExecutor(MotionFlowExecutor):
    """Flow1: VP視覺抓取流程"""
    
    def __init__(self):
        super().__init__(flow_id=1, flow_name="VP視覺抓取")
        self._build_flow_steps()
        
    def _build_flow_steps(self):
        """建構Flow1步驟"""
        # 1. 移動到待機位置
        self.add_motion_step('move_to_point', {'point_name': 'standby', 'move_type': 'J'})
        
        # 2. 移動到VP檢測位置
        self.add_motion_step('move_to_point', {'point_name': 'VP_TOPSIDE', 'move_type': 'J'})
        
        # 3. 觸發CCD1檢測
        self.add_external_step('CCD1', 'detect')
        
        # 4. 讀取檢測結果
        self.add_external_step('CCD1', 'read_results')
        
        # 5. 移動到第一個檢測點
        self.add_motion_step('move_l', {'x': 0, 'y': 0, 'z': 238.86, 'r': 0})  # CCD1檢測高度
        
        # 6. 開啟夾爪
        self.add_external_step('GRIPPER', 'open')
        
        # 7. 下降到抓取高度
        self.add_motion_step('move_l', {'x': 0, 'y': 0, 'z': 137.52, 'r': 0})  # 抓取高度
        
        # 8. 關閉夾爪
        self.add_external_step('GRIPPER', 'close')
        
        # 9. 上升到安全高度
        self.add_motion_step('move_l', {'x': 0, 'y': 0, 'z': 200, 'r': 0})
        
        # 10. 移動到旋轉工位
        self.add_motion_step('move_to_point', {'point_name': 'Rotate_V2', 'move_type': 'J'})
        self.add_motion_step('move_to_point', {'point_name': 'Rotate_top', 'move_type': 'L'})
        self.add_motion_step('move_to_point', {'point_name': 'Rotate_down', 'move_type': 'L'})
        
        # 11. 放下料件
        self.add_external_step('GRIPPER', 'open')
        
        # 12. 上升離開
        self.add_motion_step('move_to_point', {'point_name': 'Rotate_top', 'move_type': 'L'})
        
        # 13. 回到待機位置
        self.add_motion_step('move_to_point', {'point_name': 'standby', 'move_type': 'J'})
        
    def add_external_step(self, module: str, operation: str, params: Dict = None):
        """新增外部模組步驟到運動流程中"""
        self.add_motion_step('external_module', {
            'module': module,
            'operation': operation,
            'params': params or {}
        })
        
    def _execute_motion_step(self, step: Dict) -> bool:
        """擴展運動步驟執行 (增加外部模組支援)"""
        if step['type'] == 'external_module':
            return self._execute_external_module_step(step['params'])
        else:
            return super()._execute_motion_step(step)
            
    def _execute_external_module_step(self, params: Dict) -> bool:
        """執行外部模組步驟"""
        module_name = params['module']
        operation = params['operation']
        op_params = params['params']
        
        if module_name in self.external_modules:
            module = self.external_modules[module_name]
            
            if module_name == 'CCD1' and operation == 'detect':
                return module.send_command(16, op_params, timeout=10.0)
            elif module_name == 'CCD1' and operation == 'read_results':
                return self._read_ccd1_and_update_position(module)
            elif module_name == 'GRIPPER' and operation == 'open':
                return module.send_command(7, op_params, timeout=3.0)
            elif module_name == 'GRIPPER' and operation == 'close':
                return module.send_command(8, op_params, timeout=3.0)
                
        return False
        
    def _read_ccd1_and_update_position(self, module) -> bool:
        """讀取CCD1結果並更新抓取位置"""
        try:
            # 讀取檢測數量
            count = self.state_machine.read_register(40)  # 240-200=40
            
            if count and count > 0:
                # 讀取第一個物體座標
                x = self.state_machine.read_register(41)  # 241
                y = self.state_machine.read_register(42)  # 242
                
                # 更新後續步驟的座標
                self._update_pickup_coordinates(x, y)
                print(f"更新抓取座標: X={x}, Y={y}")
                return True
            else:
                print("CCD1未檢測到物體")
                return False
                
        except Exception as e:
            print(f"讀取CCD1結果失敗: {e}")
            return False
            
    def _update_pickup_coordinates(self, x: int, y: int):
        """更新抓取座標到後續步驟"""
        # 這裡需要根據實際的座標轉換邏輯
        # 將像素座標轉換為機械臂座標
        
        # 假設的轉換邏輯 (需要根據實際標定結果調整)
        robot_x = x * 0.1  # 假設縮放係數
        robot_y = y * 0.1
        
        # 找到移動步驟並更新座標
        for step in self.motion_steps:
            if step['type'] == 'move_l' and step['params'].get('z') == 238.86:
                step['params']['x'] = robot_x
                step['params']['y'] = robot_y
            elif step['type'] == 'move_l' and step['params'].get('z') == 137.52:
                step['params']['x'] = robot_x
                step['params']['y'] = robot_y
```

### Flow2: 出料流程

```python
class Flow2UnloadExecutor(MotionFlowExecutor):
    """Flow2: 出料流程"""
    
    def __init__(self):
        super().__init__(flow_id=2, flow_name="出料流程")
        self._build_flow_steps()
        
    def _build_flow_steps(self):
        """建構Flow2步驟"""
        # 1. 從待機位置開始
        self.add_motion_step('move_to_point', {'point_name': 'standby', 'move_type': 'J'})
        
        # 2. 移動到旋轉工位
        self.add_motion_step('move_to_point', {'point_name': 'Rotate_V2', 'move_type': 'J'})
        self.add_motion_step('move_to_point', {'point_name': 'Rotate_top', 'move_type': 'L'})
        self.add_motion_step('move_to_point', {'point_name': 'Rotate_down', 'move_type': 'L'})
        
        # 3. 撐開料件 (夾爪張開)
        self.add_external_step('GRIPPER', 'position', {'position': 370})
        
        # 4. 上升離開
        self.add_motion_step('move_to_point', {'point_name': 'Rotate_top', 'move_type': 'L'})
        
        # 5. 移動到組裝位置
        self.add_motion_step('move_to_point', {'point_name': 'back_stanby_from_asm', 'move_type': 'J'})
        self.add_motion_step('move_to_point', {'point_name': 'put_asm_Pre', 'move_type': 'L'})
        self.add_motion_step('move_to_point', {'point_name': 'put_asm_top', 'move_type': 'L'})
        self.add_motion_step('move_to_point', {'point_name': 'put_asm_down', 'move_type': 'L'})
        
        # 6. 關閉夾爪 (放下料件)
        self.add_external_step('GRIPPER', 'close')
        
        # 7. 返回路徑
        self.add_motion_step('move_to_point', {'point_name': 'put_asm_top', 'move_type': 'L'})
        self.add_motion_step('move_to_point', {'point_name': 'put_asm_Pre', 'move_type': 'L'})
        self.add_motion_step('move_to_point', {'point_name': 'back_stanby_from_asm', 'move_type': 'L'})
        self.add_motion_step('move_to_point', {'point_name': 'standby', 'move_type': 'J'})
```

### Flow3: 混合DIO+外部模組流程

```python
class Flow3MixedExecutor(ExternalModuleFlowExecutor):
    """Flow3: 混合DIO和外部模組操作流程"""
    
    def __init__(self):
        super().__init__(flow_id=3, flow_name="混合控制流程")
        self._build_flow_steps()
        
    def _build_flow_steps(self):
        """建構混合控制流程"""
        # 1. 設定輸出訊號開始
        self.add_dio_step('set_do', {'pin': 1, 'value': 1})
        
        # 2. 啟動VP震動
        self.add_module_step('VP', 'vibrate', {'action': 1, 'intensity': 50})
        
        # 3. 等待輸入訊號
        self.add_dio_step('wait_di', {'pin': 2, 'expected_value': 1, 'timeout': 10.0})
        
        # 4. 停止VP震動
        self.add_module_step('VP', 'stop')
        
        # 5. 觸發CCD1檢測
        self.add_module_step('CCD1', 'detect')
        
        # 6. 脈衝輸出訊號
        self.add_dio_step('pulse_do', {'pin': 3, 'pulse_width': 200})
        
        # 7. 讀取檢測結果
        self.add_module_step('CCD1', 'read_results')
        
        # 8. 序列DIO操作
        self.add_dio_step('sequence', {
            'sequence': [
                {'type': 'set_do', 'params': {'pin': 4, 'value': 1}, 'delay': 100},
                {'type': 'set_do', 'params': {'pin': 5, 'value': 1}, 'delay': 200},
                {'type': 'set_do', 'params': {'pin': 4, 'value': 0}, 'delay': 100},
                {'type': 'set_do', 'params': {'pin': 5, 'value': 0}, 'delay': 100}
            ]
        })
        
        # 9. 清除所有輸出
        self.add_dio_step('set_do', {'pin': 1, 'value': 0})
        
    def add_dio_step(self, step_type: str, params: Dict):
        """新增DIO步驟到混合流程"""
        self.add_module_step('DIO', step_type, params)
        
    def _execute_module_step(self, step: Dict) -> bool:
        """擴展模組步驟執行 (增加DIO支援)"""
        module_name = step['module']
        
        if module_name == 'DIO':
            return self._execute_dio_operation(step['operation'], step['params'])
        else:
            return super()._execute_module_step(step)
            
    def _execute_dio_operation(self, operation: str, params: Dict) -> bool:
        """執行DIO操作"""
        try:
            if operation == 'set_do':
                pin = params.get('pin', 1)
                value = params.get('value', 0)
                
                if self.robot and self.robot.is_connected:
                    result = self.robot.dashboard_api.DO(pin, value)
                    return "0" in str(result)
                    
            elif operation == 'wait_di':
                return self._wait_digital_input(params)
                
            elif operation == 'pulse_do':
                return self._pulse_digital_output(params)
                
            elif operation == 'sequence':
                return self._execute_dio_sequence(params)
                
            return False
        except Exception as e:
            print(f"DIO操作失敗: {e}")
            return False
    
    def _wait_digital_input(self, params: Dict) -> bool:
        """等待數位輸入"""
        try:
            pin = params.get('pin', 1)
            expected_value = params.get('expected_value', 1)
            timeout = params.get('timeout', 10.0)
            
            import time
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.robot and self.robot.is_connected:
                    result = self.robot.dashboard_api.DI(pin)
                    if f",{expected_value}," in result:
                        return True
                time.sleep(0.1)
                
            return False
        except Exception as e:
            print(f"等待DI失敗: {e}")
            return False
    
    def _pulse_digital_output(self, params: Dict) -> bool:
        """脈衝輸出"""
        try:
            pin = params.get('pin', 1)
            pulse_width = params.get('pulse_width', 100)
            
            # 設定為高
            if self.robot and self.robot.is_connected:
                self.robot.dashboard_api.DO(pin, 1)
                time.sleep(pulse_width / 1000.0)
                self.robot.dashboard_api.DO(pin, 0)
                return True
            return False
        except Exception as e:
            print(f"脈衝輸出失敗: {e}")
            return False
    
    def _execute_dio_sequence(self, params: Dict) -> bool:
        """執行DIO序列"""
        try:
            sequence = params.get('sequence', [])
            
            for seq_step in sequence:
                step_type = seq_step.get('type')
                step_params = seq_step.get('params', {})
                
                if not self._execute_dio_operation(step_type, step_params):
                    return False
                    
                delay = seq_step.get('delay', 0)
                if delay > 0:
                    time.sleep(delay / 1000.0)
                    
            return True
        except Exception as e:
            print(f"執行DIO序列失敗: {e}")
            return False
```

## Flow管理器

### FlowManager類別

```python
class FlowManager:
    """Flow管理器 - 負責Flow的建立、管理和執行"""
    
    def __init__(self):
        self.flows = {}
        self.robot = None
        self.state_machine = None
        self.external_modules = {}
        
    def initialize(self, robot, state_machine, external_modules):
        """初始化Flow管理器"""
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        
        # 註冊所有Flow
        self._register_flows()
        
    def _register_flows(self):
        """註冊所有可用的Flow"""
        # Flow1: VP視覺抓取
        flow1 = Flow1VisionPickExecutor()
        flow1.initialize(self.robot, self.state_machine, self.external_modules)
        self.flows[1] = flow1
        
        # Flow2: 出料流程
        flow2 = Flow2UnloadExecutor()
        flow2.initialize(self.robot, self.state_machine, self.external_modules)
        self.flows[2] = flow2
        
        # Flow3: 混合控制流程
        flow3 = Flow3MixedExecutor()
        flow3.initialize(self.robot, self.state_machine, self.external_modules)
        self.flows[3] = flow3
        
    def get_flow(self, flow_id: int) -> Optional[FlowExecutor]:
        """取得指定Flow"""
        return self.flows.get(flow_id)
        
    def execute_flow(self, flow_id: int) -> FlowResult:
        """執行指定Flow"""
        flow = self.get_flow(flow_id)
        if flow:
            return flow.execute()
        else:
            return FlowResult(
                success=False,
                error_message=f"Flow {flow_id} 不存在"
            )
            
    def get_all_flows_status(self) -> Dict[int, Dict]:
        """取得所有Flow狀態"""
        status = {}
        for flow_id, flow in self.flows.items():
            status[flow_id] = flow.get_status_info()
        return status
        
    def stop_all_flows(self):
        """停止所有Flow"""
        for flow in self.flows.values():
            flow.stop()
```

## Main與Flow整合機制

### 在MotionFlowThread中整合FlowManager

```python
class MotionFlowThread(threading.Thread):
    """運動控制執行緒 - 整合FlowManager"""
    
    def __init__(self, robot, command_queue, state_machine, external_modules):
        super().__init__(daemon=True, name="MotionFlow")
        self.robot = robot
        self.command_queue = command_queue
        self.state_machine = state_machine
        self.running = False
        
        # 建立Flow管理器
        self.flow_manager = FlowManager()
        self.flow_manager.initialize(robot, state_machine, external_modules)
        
        # 狀態追蹤
        self.current_flow = None
        self.status = "停止"
        self.operation_count = 0
        
    def _execute_motion_command(self, command: Command):
        """執行運動指令 - 支援Flow執行"""
        cmd_data = command.command_data
        cmd_type = cmd_data.get('type')
        
        try:
            if cmd_type.startswith('flow'):
                # 執行Flow
                flow_id = int(cmd_type.replace('flow', ''))
                self._execute_flow(flow_id)
            elif cmd_type == 'move_j':
                self._execute_move_j(cmd_data)
            elif cmd_type == 'move_l':
                self._execute_move_l(cmd_data)
            # ... 其他指令
            
        except Exception as e:
            self.last_error = f"執行運動指令失敗: {e}"
            print(self.last_error)
            
    def _execute_flow(self, flow_id: int):
        """執行指定Flow"""
        print(f"開始執行Flow{flow_id}...")
        
        # 更新狀態機
        if self.state_machine:
            self.state_machine.write_register(2, flow_id)  # 402: CURRENT_FLOW
            self.state_machine.write_register(0, 10)  # 400: STATUS_REGISTER (Running=1)
        
        # 執行Flow
        flow = self.flow_manager.get_flow(flow_id)
        if flow:
            self.current_flow = flow
            result = flow.execute()
            
            # 更新結果
            if result.success:
                print(f"Flow{flow_id}執行成功")
                self.operation_count += 1
            else:
                print(f"Flow{flow_id}執行失敗: {result.error_message}")
                
            # 清除狀態
            self.current_flow = None
            if self.state_machine:
                self.state_machine.write_register(2, 0)  # 清除CURRENT_FLOW
                self.state_machine.write_register(0, 9)  # STATUS_REGISTER (Ready=1)
        else:
            print(f"Flow{flow_id}不存在")
```

## 配置和參數管理

### Flow配置檔案結構

```json
{
    "flows": {
        "flow1": {
            "name": "VP視覺抓取",
            "enabled": true,
            "timeout": 60.0,
            "retry_count": 2,
            "parameters": {
                "ccd1_detection_height": 238.86,
                "pickup_height": 137.52,
                "safety_height": 200.0,
                "gripper_open_position": 370,
                "gripper_close_position": 0
            },
            "points": [
                "standby", "VP_TOPSIDE", "Rotate_V2", 
                "Rotate_top", "Rotate_down"
            ]
        },
        "flow2": {
            "name": "出料流程",
            "enabled": true,
            "timeout": 45.0,
            "retry_count": 1,
            "parameters": {
                "gripper_spread_position": 370,
                "safety_height": 200.0
            },
            "points": [
                "standby", "Rotate_V2", "Rotate_top", "Rotate_down",
                "back_stanby_from_asm", "put_asm_Pre", 
                "put_asm_top", "put_asm_down"
            ]
        },
        "flow3": {
            "name": "混合控制流程",
            "enabled": false,
            "timeout": 30.0,
            "retry_count": 1,
            "parameters": {
                "vp_vibration_intensity": 50,
                "di_timeout": 10.0,
                "pulse_width": 200
            }
        }
    },
    "external_modules": {
        "CCD1": {
            "base_address": 200,
            "control_offset": 0,
            "status_offset": 1,
            "timeout": 10.0
        },
        "VP": {
            "base_address": 300,
            "control_offset": 20,
            "status_offset": 0,
            "timeout": 5.0
        },
        "GRIPPER": {
            "base_address": 500,
            "control_offset": 120,
            "status_offset": 100,
            "timeout": 3.0
        },
        "CCD3": {
            "base_address": 800,
            "control_offset": 400,
            "status_offset": 401,
            "timeout": 15.0
        }
    }
}
```

## 錯誤處理和恢復

### Flow級錯誤處理

```python
class FlowExecutor(ABC):
    def execute_with_retry(self, max_retries: int = 2) -> FlowResult:
        """執行Flow with重試機制"""
        for attempt in range(max_retries + 1):
            try:
                result = self.execute()
                if result.success:
                    return result
                    
                print(f"Flow執行失敗 (嘗試 {attempt + 1}/{max_retries + 1}): {result.error_message}")
                
                if attempt < max_retries:
                    # 重試前的恢復動作
                    self._recovery_action()
                    time.sleep(1.0)
                    
            except Exception as e:
                print(f"Flow執行異常 (嘗試 {attempt + 1}/{max_retries + 1}): {e}")
                
        return FlowResult(
            success=False,
            error_message=f"Flow在{max_retries + 1}次嘗試後仍然失敗"
        )
        
    def _recovery_action(self):
        """恢復動作 - 子類別可覆寫"""
        # 基本恢復動作：重置狀態
        self.status = FlowStatus.IDLE
        self.current_step = 0
        
        # 停止機械臂運動
        if self.robot and self.robot.is_connected:
            try:
                self.robot.dashboard_api.pause()
                time.sleep(0.5)
                self.robot.dashboard_api.Continue()
            except:
                pass
```

## 監控和診斷

### Flow執行監控

```python
class FlowMonitor:
    """Flow執行監控器"""
    
    def __init__(self, flow_manager: FlowManager):
        self.flow_manager = flow_manager
        self.execution_history = []
        self.performance_metrics = {}
        
    def log_flow_execution(self, flow_id: int, result: FlowResult):
        """記錄Flow執行結果"""
        log_entry = {
            'timestamp': time.time(),
            'flow_id': flow_id,
            'success': result.success,
            'execution_time': result.execution_time,
            'steps_completed': result.steps_completed,
            'total_steps': result.total_steps,
            'error_message': result.error_message
        }
        
        self.execution_history.append(log_entry)
        self._update_performance_metrics(flow_id, result)
        
    def _update_performance_metrics(self, flow_id: int, result: FlowResult):
        """更新性能指標"""
        if flow_id not in self.performance_metrics:
            self.performance_metrics[flow_id] = {
                'total_executions': 0,
                'successful_executions': 0,
                'total_time': 0.0,
                'average_time': 0.0,
                'success_rate': 0.0
            }
            
        metrics = self.performance_metrics[flow_id]
        metrics['total_executions'] += 1
        
        if result.success:
            metrics['successful_executions'] += 1
            
        metrics['total_time'] += result.execution_time
        metrics['average_time'] = metrics['total_time'] / metrics['total_executions']
        metrics['success_rate'] = metrics['successful_executions'] / metrics['total_executions']
        
    def get_flow_statistics(self, flow_id: int) -> Dict:
        """取得Flow統計資訊"""
        return self.performance_metrics.get(flow_id, {})
        
    def get_recent_executions(self, count: int = 10) -> List[Dict]:
        """取得最近的執行記錄"""
        return self.execution_history[-count:]
```

## 使用範例

### 在Dobot_main中使用Flow

```python
# 在DobotConcurrentController類別中
def _handle_flow1_command(self):
    """處理Flow1指令"""
    command = Command(
        command_type=CommandType.MOTION,
        command_data={'type': 'flow1'},
        priority=CommandPriority.HIGH
    )
    
    if self.command_queue.put_command(command):
        print("Flow1指令已加入佇列")
        
        # 更新狀態寄存器
        if self.modbus_handler:
            self.modbus_handler.write_register(2, 1)  # CURRENT_FLOW = 1
            self.modbus_handler.write_register(0, 10) # STATUS_REGISTER (Running=1)
    else:
        print("佇列已滿，Flow1指令丟棄")

# 在MotionFlowThread中執行Flow
def _execute_flow(self, flow_id: int):
    flow = self.flow_manager.get_flow(flow_id)
    if flow:
        # 執行Flow (支援重試)
        result = flow.execute_with_retry(max_retries=2)
        
        # 記錄執行結果
        if hasattr(self, 'flow_monitor'):
            self.flow_monitor.log_flow_execution(flow_id, result)
            
        # 更新狀態寄存器
        if result.success:
            print(f"Flow{flow_id}執行成功")
            # 更新完成狀態 (如Flow1的420寄存器)
            if flow_id == 1:
                self.state_machine.write_register(20, 1)  # FLOW1_COMPLETE = 1
        else:
            print(f"Flow{flow_id}執行失敗: {result.error_message}")
            # 設定錯誤狀態
            self.state_machine.write_register(0, 12)  # STATUS_REGISTER (Alarm=1)
```

## 總結

### 架構優勢

1. **分工明確**: Main負責排程，Flow負責執行邏輯
2. **高度模組化**: 每個Flow獨立，便於維護和擴展
3. **統一介面**: 所有Flow遵循相同的介面規範
4. **靈活配置**: 支援參數化配置和動態調整
5. **完整監控**: 提供執行狀態和性能監控
6. **錯誤處理**: 多層級錯誤處理和恢復機制

### 擴展指南

1. **新增Flow**: 繼承對應的FlowExecutor基類別，實現execute()方法
2. **新增外部模組**: 在配置檔案中新增模組配置，更新ExternalModuleInterface
3. **新增指令類型**: 擴展各執行緒的指令處理邏輯
4. **客製化監控**: 擴展FlowMonitor類別，新增監控指標

此架構確保了系統的可擴展性和維護性，同時保持了高效的執行效能和可靠的錯誤處理能力。