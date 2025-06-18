# -*- coding: utf-8 -*-
"""
vibration_plate.py - [U+9707][U+52D5][U+76E4][U+63A7][U+5236][U+6A21][U+7D44]
[U+652F][U+63F4]ModbusTCP[U+901A][U+8A0A][U+5354][U+5B9A][U+FF0C][U+63D0][U+4F9B][U+5B8C][U+6574][U+7684][U+9707][U+52D5][U+76E4][U+63A7][U+5236][U+529F][U+80FD]
"""

from pymodbus.client import ModbusTcpClient
import threading
import time
import logging
from typing import Dict, Any, Optional, List

class VibrationPlate:
    """
    [U+9707][U+52D5][U+76E4][U+63A7][U+5236]API[U+985E][U+5225]
    [U+652F][U+63F4]ModbusTCP[U+901A][U+8A0A][U+5354][U+5B9A]
    """
    
    # [U+52D5][U+4F5C][U+7DE8][U+78BC][U+6620][U+5C04]
    ACTION_MAP = {
        'stop': 0,
        'up': 1,
        'down': 2,
        'left': 3,
        'right': 4,
        'upleft': 5,
        'downleft': 6,
        'upright': 7,
        'downright': 8,
        'horizontal': 9,
        'vertical': 10,
        'spread': 11
    }
    
    # [U+5BC4][U+5B58][U+5668][U+4F4D][U+5740][U+5B9A][U+7FA9]
    REGISTERS = {
        'single_action_trigger': 4,      # [U+55AE][U+4E00][U+52D5][U+4F5C][U+89F8][U+767C]
        'vibration_status': 6,           # [U+9707][U+52D5][U+76E4][U+72C0][U+614B]
        'backlight_test': 58,            # [U+80CC][U+5149][U+6E2C][U+8A66][U+958B][U+95DC]
        'backlight_brightness': 46,      # [U+80CC][U+5149][U+4EAE][U+5EA6]
        
        # [U+5F37][U+5EA6][U+5BC4][U+5B58][U+5668] (20-30)
        'strength': {
            'up': 20, 'down': 21, 'left': 22, 'right': 23,
            'upleft': 24, 'downleft': 25, 'upright': 26, 'downright': 27,
            'horizontal': 28, 'vertical': 29, 'spread': 30
        },
        
        # [U+983B][U+7387][U+5BC4][U+5B58][U+5668] (60-70)
        'frequency': {
            'up': 60, 'down': 61, 'left': 62, 'right': 63,
            'upleft': 64, 'downleft': 65, 'upright': 66, 'downright': 67,
            'horizontal': 68, 'vertical': 69, 'spread': 70
        }
    }

    def __init__(self, ip: str, port: int, slave_id: int, auto_connect: bool = True):
        """
        [U+521D][U+59CB][U+5316][U+9707][U+52D5][U+76E4][U+63A7][U+5236][U+5668]
        
        Args:
            ip: ModbusTCP[U+4F3A][U+670D][U+5668]IP[U+4F4D][U+5740]
            port: ModbusTCP[U+4F3A][U+670D][U+5668][U+57E0][U+53E3]
            slave_id: [U+5F9E][U+6A5F]ID
            auto_connect: [U+662F][U+5426][U+81EA][U+52D5][U+9023][U+7DDA]
        """
        self.ip = ip
        self.port = port
        self.slave_id = slave_id
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.lock = threading.Lock()
        
        # [U+72C0][U+614B][U+5FEB][U+53D6]
        self.current_action = 'stop'
        self.last_brightness = 128
        self.last_backlight_state = True
        self.action_parameters = {}
        
        # [U+521D][U+59CB][U+5316][U+52D5][U+4F5C][U+53C3][U+6578]
        self.init_action_parameters()
        
        # [U+8A2D][U+5B9A][U+65E5][U+8A8C]
        self.logger = logging.getLogger(f"VibrationPlate_{ip}")
        
        if auto_connect:
            self.connect()

    def init_action_parameters(self):
        """[U+521D][U+59CB][U+5316][U+52D5][U+4F5C][U+53C3][U+6578]"""
        actions = ['up', 'down', 'left', 'right', 'upleft', 'downleft',
                  'upright', 'downright', 'horizontal', 'vertical', 'spread']
        
        for action in actions:
            self.action_parameters[action] = {
                'strength': 100,
                'frequency': 100
            }

    def connect(self) -> bool:
        """[U+9023][U+7DDA][U+5230]ModbusTCP[U+4F3A][U+670D][U+5668]"""
        try:
            if self.client:
                self.client.close()
            
            self.client = ModbusTcpClient(host=self.ip, port=self.port)
            
            if self.client.connect():
                self.connected = True
                self.logger.info(f"[U+6210][U+529F][U+9023][U+7DDA][U+5230][U+9707][U+52D5][U+76E4] {self.ip}:{self.port} ([U+5F9E][U+6A5F]ID: {self.slave_id})")
                
                # [U+9023][U+7DDA][U+6210][U+529F][U+5F8C][U+8B80][U+53D6][U+7576][U+524D][U+72C0][U+614B]
                self.read_current_status()
                
                return True
            else:
                self.connected = False
                self.logger.error(f"[U+7121][U+6CD5][U+9023][U+7DDA][U+5230][U+9707][U+52D5][U+76E4] {self.ip}:{self.port}")
                return False
                
        except Exception as e:
            self.connected = False
            self.logger.error(f"[U+9023][U+7DDA][U+7570][U+5E38]: {e}")
            return False

    def disconnect(self):
        """[U+4E2D][U+65B7][U+9023][U+7DDA]"""
        try:
            if self.client:
                # [U+505C][U+6B62][U+6240][U+6709][U+52D5][U+4F5C]
                self.stop()
                self.client.close()
                self.connected = False
                self.logger.info("[U+9707][U+52D5][U+76E4][U+9023][U+7DDA][U+5DF2][U+4E2D][U+65B7]")
        except Exception as e:
            self.logger.error(f"[U+4E2D][U+65B7][U+9023][U+7DDA][U+6642][U+767C][U+751F][U+932F][U+8AA4]: {e}")

    def is_connected(self) -> bool:
        """[U+6AA2][U+67E5][U+9023][U+7DDA][U+72C0][U+614B]"""
        if not self.client:
            return False
        
        try:
            # [U+5617][U+8A66][U+8B80][U+53D6][U+4E00][U+500B][U+5BC4][U+5B58][U+5668][U+4F86][U+6AA2][U+67E5][U+9023][U+7DDA]
            result = self.client.read_holding_registers(
                address=self.REGISTERS['vibration_status'],
                count=1,
                slave=self.slave_id
            )
            
            is_ok = not result.isError()
            self.connected = is_ok
            return is_ok
            
        except:
            self.connected = False
            return False

    def write_register(self, address: int, value: int) -> bool:
        """
        [U+5BEB][U+5165][U+55AE][U+500B][U+5BC4][U+5B58][U+5668]
        
        Args:
            address: [U+5BC4][U+5B58][U+5668][U+4F4D][U+5740]
            value: [U+5BEB][U+5165][U+503C]
            
        Returns:
            bool: [U+64CD][U+4F5C][U+662F][U+5426][U+6210][U+529F]
        """
        if not self.is_connected():
            self.logger.warning("[U+8A2D][U+5099][U+672A][U+9023][U+7DDA][U+FF0C][U+5617][U+8A66][U+91CD][U+65B0][U+9023][U+7DDA]...")
            if not self.connect():
                return False

        try:
            with self.lock:
                response = self.client.write_register(
                    address=address, 
                    value=value, 
                    slave=self.slave_id
                )
                
                if response.isError():
                    self.logger.error(f"[U+5BEB][U+5165][U+5BC4][U+5B58][U+5668] {address} [U+5931][U+6557]: {response}")
                    return False
                else:
                    self.logger.debug(f"[U+5BC4][U+5B58][U+5668] {address} [U+8A2D][U+5B9A][U+70BA] {value}")
                    return True
                    
        except Exception as e:
            self.logger.error(f"[U+5BEB][U+5165][U+5BC4][U+5B58][U+5668][U+7570][U+5E38]: {e}")
            self.connected = False
            return False

    def read_register(self, address: int) -> Optional[int]:
        """
        [U+8B80][U+53D6][U+55AE][U+500B][U+5BC4][U+5B58][U+5668]
        
        Args:
            address: [U+5BC4][U+5B58][U+5668][U+4F4D][U+5740]
            
        Returns:
            int or None: [U+5BC4][U+5B58][U+5668][U+503C][U+FF0C][U+5931][U+6557][U+8FD4][U+56DE]None
        """
        if not self.is_connected():
            self.logger.warning("[U+8A2D][U+5099][U+672A][U+9023][U+7DDA][U+FF0C][U+5617][U+8A66][U+91CD][U+65B0][U+9023][U+7DDA]...")
            if not self.connect():
                return None

        try:
            with self.lock:
                response = self.client.read_holding_registers(
                    address=address, 
                    count=1, 
                    slave=self.slave_id
                )
                
                if response.isError():
                    self.logger.error(f"[U+8B80][U+53D6][U+5BC4][U+5B58][U+5668] {address} [U+5931][U+6557]: {response}")
                    return None
                else:
                    return response.registers[0]
                    
        except Exception as e:
            self.logger.error(f"[U+8B80][U+53D6][U+5BC4][U+5B58][U+5668][U+7570][U+5E38]: {e}")
            self.connected = False
            return None

    def read_multiple_registers(self, start_address: int, count: int) -> Optional[List[int]]:
        """
        [U+6279][U+91CF][U+8B80][U+53D6][U+5BC4][U+5B58][U+5668]
        
        Args:
            start_address: [U+8D77][U+59CB][U+4F4D][U+5740]
            count: [U+8B80][U+53D6][U+6578][U+91CF]
            
        Returns:
            list or None: [U+5BC4][U+5B58][U+5668][U+503C][U+5217][U+8868][U+FF0C][U+5931][U+6557][U+8FD4][U+56DE]None
        """
        if not self.is_connected():
            if not self.connect():
                return None

        try:
            with self.lock:
                response = self.client.read_holding_registers(
                    address=start_address, 
                    count=count, 
                    slave=self.slave_id
                )
                
                if response.isError():
                    self.logger.error(f"[U+6279][U+91CF][U+8B80][U+53D6][U+5BC4][U+5B58][U+5668][U+5931][U+6557]: {response}")
                    return None
                else:
                    return response.registers
                    
        except Exception as e:
            self.logger.error(f"[U+6279][U+91CF][U+8B80][U+53D6][U+5BC4][U+5B58][U+5668][U+7570][U+5E38]: {e}")
            self.connected = False
            return None

    def read_current_status(self):
        """[U+8B80][U+53D6][U+7576][U+524D][U+72C0][U+614B]"""
        try:
            # [U+8B80][U+53D6][U+80CC][U+5149][U+4EAE][U+5EA6]
            brightness = self.read_register(self.REGISTERS['backlight_brightness'])
            if brightness is not None:
                self.last_brightness = brightness
            
            # [U+8B80][U+53D6][U+9707][U+52D5][U+72C0][U+614B]
            vibration_status = self.read_register(self.REGISTERS['vibration_status'])
            if vibration_status is not None:
                self.current_action = 'stop' if vibration_status == 0 else 'running'
            
            # [U+8B80][U+53D6][U+52D5][U+4F5C][U+53C3][U+6578]
            self.read_action_parameters()
            
        except Exception as e:
            self.logger.error(f"[U+8B80][U+53D6][U+7576][U+524D][U+72C0][U+614B][U+5931][U+6557]: {e}")

    def read_action_parameters(self):
        """[U+8B80][U+53D6][U+6240][U+6709][U+52D5][U+4F5C][U+53C3][U+6578]"""
        try:
            # [U+8B80][U+53D6][U+5F37][U+5EA6][U+5BC4][U+5B58][U+5668]
            strength_registers = self.read_multiple_registers(20, 11)
            # [U+8B80][U+53D6][U+983B][U+7387][U+5BC4][U+5B58][U+5668]
            frequency_registers = self.read_multiple_registers(60, 11)
            
            if strength_registers and frequency_registers:
                actions = ['up', 'down', 'left', 'right', 'upleft', 'downleft',
                          'upright', 'downright', 'horizontal', 'vertical', 'spread']
                
                for i, action in enumerate(actions):
                    self.action_parameters[action] = {
                        'strength': strength_registers[i],
                        'frequency': frequency_registers[i]
                    }
                    
        except Exception as e:
            self.logger.error(f"[U+8B80][U+53D6][U+52D5][U+4F5C][U+53C3][U+6578][U+5931][U+6557]: {e}")

    def set_backlight(self, state: bool) -> bool:
        """
        [U+8A2D][U+5B9A][U+80CC][U+5149][U+958B][U+95DC]
        
        Args:
            state: True[U+958B][U+555F][U+FF0C]False[U+95DC][U+9589]
            
        Returns:
            bool: [U+64CD][U+4F5C][U+662F][U+5426][U+6210][U+529F]
        """
        success = self.write_register(self.REGISTERS['backlight_test'], int(bool(state)))
        if success:
            self.last_backlight_state = state
            self.logger.info(f"[U+80CC][U+5149]{'[U+958B][U+555F]' if state else '[U+95DC][U+9589]'}")
        return success

    def set_backlight_brightness(self, brightness: int) -> bool:
        """
        [U+8A2D][U+5B9A][U+80CC][U+5149][U+4EAE][U+5EA6]
        
        Args:
            brightness: [U+4EAE][U+5EA6][U+503C] (0-255)
            
        Returns:
            bool: [U+64CD][U+4F5C][U+662F][U+5426][U+6210][U+529F]
        """
        brightness = max(0, min(255, int(brightness)))
        success = self.write_register(self.REGISTERS['backlight_brightness'], brightness)
        if success:
            self.last_brightness = brightness
            self.logger.info(f"[U+80CC][U+5149][U+4EAE][U+5EA6][U+8A2D][U+5B9A][U+70BA]: {brightness}")
        return success

    def trigger_action(self, action: str) -> bool:
        """
        [U+89F8][U+767C][U+6307][U+5B9A][U+52D5][U+4F5C]
        
        Args:
            action: [U+52D5][U+4F5C][U+540D][U+7A31][U+6216][U+7DE8][U+78BC]
            
        Returns:
            bool: [U+64CD][U+4F5C][U+662F][U+5426][U+6210][U+529F]
        """
        if isinstance(action, str):
            if action not in self.ACTION_MAP:
                self.logger.error(f"[U+672A][U+77E5][U+52D5][U+4F5C]: {action}")
                return False
            action_code = self.ACTION_MAP[action]
        else:
            action_code = int(action)
            
        success = self.write_register(self.REGISTERS['single_action_trigger'], action_code)
        if success:
            self.current_action = action if isinstance(action, str) else str(action_code)
            self.logger.info(f"[U+89F8][U+767C][U+52D5][U+4F5C]: {action} ([U+4EE3][U+78BC]: {action_code})")
        return success

    def set_action_parameters(self, action: str, strength: int = None, frequency: int = None) -> bool:
        """
        [U+8A2D][U+5B9A][U+52D5][U+4F5C][U+53C3][U+6578]
        
        Args:
            action: [U+52D5][U+4F5C][U+540D][U+7A31]
            strength: [U+5F37][U+5EA6][U+503C] (0-255)
            frequency: [U+983B][U+7387][U+503C] (0-255)
            
        Returns:
            bool: [U+64CD][U+4F5C][U+662F][U+5426][U+6210][U+529F]
        """
        if action not in self.REGISTERS['strength']:
            self.logger.error(f"[U+672A][U+77E5][U+52D5][U+4F5C]: {action}")
            return False
            
        success = True
        
        # [U+66F4][U+65B0][U+672C][U+5730][U+5FEB][U+53D6]
        if action not in self.action_parameters:
            self.action_parameters[action] = {'strength': 100, 'frequency': 100}
        
        if strength is not None:
            strength = max(0, min(255, int(strength)))
            if self.write_register(self.REGISTERS['strength'][action], strength):
                self.action_parameters[action]['strength'] = strength
                self.logger.debug(f"[U+8A2D][U+5B9A] {action} [U+5F37][U+5EA6]: {strength}")
            else:
                success = False
            
        if frequency is not None:
            frequency = max(0, min(255, int(frequency)))
            if self.write_register(self.REGISTERS['frequency'][action], frequency):
                self.action_parameters[action]['frequency'] = frequency
                self.logger.debug(f"[U+8A2D][U+5B9A] {action} [U+983B][U+7387]: {frequency}")
            else:
                success = False
                
        return success

    def execute_action(self, action: str, strength: int = None, frequency: int = None, duration: float = None) -> bool:
        """
        [U+57F7][U+884C][U+52D5][U+4F5C][U+FF08][U+8A2D][U+5B9A][U+53C3][U+6578][U+4E26][U+89F8][U+767C][U+FF09]
        
        Args:
            action: [U+52D5][U+4F5C][U+540D][U+7A31]
            strength: [U+5F37][U+5EA6][U+503C] (0-255)
            frequency: [U+983B][U+7387][U+503C] (0-255)
            duration: [U+6301][U+7E8C][U+6642][U+9593]([U+79D2])[U+FF0C]None[U+8868][U+793A][U+4E0D][U+81EA][U+52D5][U+505C][U+6B62]
            
        Returns:
            bool: [U+64CD][U+4F5C][U+662F][U+5426][U+6210][U+529F]
        """
        # [U+8A2D][U+5B9A][U+53C3][U+6578]
        if strength is not None or frequency is not None:
            if not self.set_action_parameters(action, strength, frequency):
                self.logger.warning(f"[U+8A2D][U+5B9A] {action} [U+53C3][U+6578][U+5931][U+6557][U+FF0C][U+4F46][U+4ECD][U+5617][U+8A66][U+57F7][U+884C][U+52D5][U+4F5C]")
        
        # [U+89F8][U+767C][U+52D5][U+4F5C]
        if not self.trigger_action(action):
            return False
        
        # [U+5982][U+679C][U+6307][U+5B9A][U+4E86][U+6301][U+7E8C][U+6642][U+9593][U+FF0C][U+5247][U+5EF6][U+6642][U+5F8C][U+505C][U+6B62]
        if duration is not None and duration > 0:
            def stop_after_delay():
                time.sleep(duration)
                self.stop()
                
            threading.Thread(target=stop_after_delay, daemon=True).start()
            self.logger.info(f"[U+52D5][U+4F5C] {action} [U+5C07][U+5728] {duration} [U+79D2][U+5F8C][U+505C][U+6B62]")
            
        return True

    def stop(self) -> bool:
        """[U+505C][U+6B62][U+6240][U+6709][U+52D5][U+4F5C]"""
        success = self.trigger_action('stop')
        if success:
            self.current_action = 'stop'
            self.logger.info("[U+6240][U+6709][U+52D5][U+4F5C][U+5DF2][U+505C][U+6B62]")
        return success

    def get_status(self) -> Dict[str, Any]:
        """
        [U+53D6][U+5F97][U+9707][U+52D5][U+76E4][U+72C0][U+614B]
        
        Returns:
            dict: [U+72C0][U+614B][U+8CC7][U+8A0A]
        """
        status = {
            'connected': self.is_connected(),
            'vibration_active': False,
            'backlight_brightness': self.last_brightness,
            'backlight_on': self.last_backlight_state,
            'current_action': self.current_action,
            'action_parameters': self.action_parameters.copy(),
            'device_info': {
                'ip': self.ip,
                'port': self.port,
                'slave_id': self.slave_id
            }
        }
        
        if not self.is_connected():
            return status
            
        try:
            # [U+8B80][U+53D6][U+5373][U+6642][U+9707][U+52D5][U+72C0][U+614B]
            vibration_status = self.read_register(self.REGISTERS['vibration_status'])
            if vibration_status is not None:
                status['vibration_active'] = bool(vibration_status)
                
            # [U+8B80][U+53D6][U+5373][U+6642][U+80CC][U+5149][U+4EAE][U+5EA6]
            brightness = self.read_register(self.REGISTERS['backlight_brightness'])
            if brightness is not None:
                status['backlight_brightness'] = brightness
                self.last_brightness = brightness
                
            # [U+66F4][U+65B0][U+52D5][U+4F5C][U+53C3][U+6578]
            self.read_action_parameters()
            status['action_parameters'] = self.action_parameters.copy()
            
        except Exception as e:
            self.logger.error(f"[U+8B80][U+53D6][U+72C0][U+614B][U+5931][U+6557]: {e}")
                
        return status

    def get_action_list(self) -> List[str]:
        """[U+53D6][U+5F97][U+6240][U+6709][U+53EF][U+7528][U+52D5][U+4F5C][U+5217][U+8868]"""
        return list(self.ACTION_MAP.keys())

    def get_register_map(self) -> Dict[str, Any]:
        """[U+53D6][U+5F97][U+5BC4][U+5B58][U+5668][U+6620][U+5C04][U+8CC7][U+8A0A]"""
        return {
            'action_map': self.ACTION_MAP,
            'registers': self.REGISTERS,
            'device_info': {
                'ip': self.ip,
                'port': self.port,
                'slave_id': self.slave_id
            }
        }

    def test_connection(self) -> Dict[str, Any]:
        """[U+6E2C][U+8A66][U+9023][U+7DDA]"""
        test_result = {
            'success': False,
            'message': '',
            'details': {}
        }
        
        try:
            # [U+6E2C][U+8A66][U+57FA][U+672C][U+9023][U+7DDA]
            if not self.is_connected():
                if not self.connect():
                    test_result['message'] = f"[U+7121][U+6CD5][U+9023][U+7DDA][U+5230] {self.ip}:{self.port}"
                    return test_result
            
            # [U+6E2C][U+8A66][U+8B80][U+53D6][U+5BC4][U+5B58][U+5668]
            vibration_status = self.read_register(self.REGISTERS['vibration_status'])
            brightness = self.read_register(self.REGISTERS['backlight_brightness'])
            
            if vibration_status is not None and brightness is not None:
                test_result['success'] = True
                test_result['message'] = "[U+9023][U+7DDA][U+6E2C][U+8A66][U+6210][U+529F]"
                test_result['details'] = {
                    'vibration_status': vibration_status,
                    'backlight_brightness': brightness,
                    'connected': True
                }
            else:
                test_result['message'] = "[U+7121][U+6CD5][U+8B80][U+53D6][U+8A2D][U+5099][U+5BC4][U+5B58][U+5668]"
                
        except Exception as e:
            test_result['message'] = f"[U+9023][U+7DDA][U+6E2C][U+8A66][U+5931][U+6557]: {e}"
            
        return test_result

    def __enter__(self):
        """[U+4E0A][U+4E0B][U+6587][U+7BA1][U+7406][U+5668][U+5165][U+53E3]"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """[U+4E0A][U+4E0B][U+6587][U+7BA1][U+7406][U+5668][U+51FA][U+53E3]"""
        self.disconnect()

    def __del__(self):
        """[U+6790][U+69CB][U+51FD][U+6578]"""
        try:
            self.disconnect()
        except:
            pass