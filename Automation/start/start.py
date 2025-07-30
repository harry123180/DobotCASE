# -*- coding: utf-8 -*-
"""
CCD1記憶體監控與重啟評估工具
監控CCD1進程記憶體使用量，測量重啟時間性能
"""

import os
import sys
import subprocess
import time
import threading
import psutil
from pathlib import Path
import logging
from datetime import datetime
from logging.handlers import RotatingFileHandler

class CCD1MemoryMonitor:
    """CCD1記憶體監控器"""
    
    def __init__(self):
        # 設置logging
        self.logger = self.setup_logging()
        
        # conda環境名稱
        self.conda_env = "ROBOT"
        
        # CCD1腳本配置
        self.ccd1_config = {
            'path': r'C:\Users\user\Documents\GitHub\DobotCASE\Automation\CCD1\CCD1VisionCodeYOLO.py',
            'name': 'CCD1視覺系統',
            'port': 5051,
            'description': 'CCD1 YOLOv11視覺檢測系統'
        }
        
        # 監控狀態
        self.process_info = None
        self.monitoring = False
        self.memory_data = []
        self.restart_times = []
        
    def setup_logging(self):
        """設置logging配置"""
        # 日誌目錄：執行檔同層目錄下的logs資料夾
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        
        # 格式化器
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        # 文件處理器 (輪替日誌，保存一週)
        file_handler = RotatingFileHandler(
            os.path.join(log_dir, 'ccd1_monitor.log'),
            maxBytes=10*1024*1024,  # 10MB
            backupCount=7,          # 保留7個檔案
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        
        # 控制台處理器
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        
        # 配置logger
        logger = logging.getLogger('CCD1Monitor')
        logger.setLevel(logging.DEBUG)
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    def check_conda_environment(self):
        """檢查conda環境是否存在"""
        try:
            self.logger.info(f"檢查conda環境: {self.conda_env}")
            
            # 檢查conda是否可用
            result = subprocess.run(['conda', '--version'], capture_output=True, text=True)
            if result.returncode != 0:
                self.logger.error("conda未安裝或不在PATH中")
                return False
            
            self.logger.info(f"conda版本: {result.stdout.strip()}")
            
            # 檢查指定環境是否存在
            result = subprocess.run(['conda', 'env', 'list'], capture_output=True, text=True)
            if result.returncode != 0:
                self.logger.error("無法列出conda環境")
                return False
            
            env_lines = result.stdout.split('\n')
            env_exists = False
            
            for line in env_lines:
                if self.conda_env in line:
                    env_exists = True
                    self.logger.info(f"找到環境: {line.strip()}")
                    break
            
            if not env_exists:
                self.logger.error(f"conda環境 '{self.conda_env}' 不存在")
                return False
            
            self.logger.info("conda環境檢查通過")
            return True
            
        except FileNotFoundError:
            self.logger.error("找不到conda命令，請確認conda已安裝並加入PATH")
            return False
        except Exception as e:
            self.logger.error(f"檢查conda環境失敗: {e}", exc_info=True)
            return False
    
    def check_script_file(self):
        """檢查CCD1腳本檔案是否存在"""
        self.logger.info("檢查CCD1腳本檔案")
        
        script_path = Path(self.ccd1_config['path'])
        
        if script_path.exists():
            file_size = script_path.stat().st_size / 1024  # KB
            self.logger.info(f"CCD1腳本: {script_path} ({file_size:.1f} KB)")
            return True
        else:
            self.logger.error(f"CCD1腳本不存在: {script_path}")
            return False
    
    def start_ccd1(self):
        """啟動CCD1腳本"""
        script_path = self.ccd1_config['path']
        script_name = self.ccd1_config['name']
        
        self.logger.info(f"啟動 {script_name}")
        self.logger.info(f"腳本路徑: {script_path}")
        
        start_time = time.time()
        
        try:
            # 使用conda run啟動
            cmd = [
                'conda', 'run', '-n', self.conda_env,
                'python', script_path
            ]
            
            self.logger.info(f"執行命令: {' '.join(cmd[:4])} ... {Path(script_path).name}")
            
            # 設置工作目錄為腳本所在目錄
            working_dir = Path(script_path).parent
            
            process = subprocess.Popen(
                cmd,
                cwd=str(working_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            self.process_info = {
                'process': process,
                'start_time': datetime.now(),
                'startup_time': None,
                'psutil_process': None
            }
            
            # 等待進程啟動
            time.sleep(3)
            
            if process.poll() is None:
                # 獲取psutil進程對象用於記憶體監控
                try:
                    self.process_info['psutil_process'] = psutil.Process(process.pid)
                    startup_time = time.time() - start_time
                    self.process_info['startup_time'] = startup_time
                    
                    self.logger.info(f"{script_name} 啟動成功")
                    self.logger.info(f"PID: {process.pid}")
                    self.logger.info(f"啟動時間: {startup_time:.2f} 秒")
                    
                    return True
                except psutil.NoSuchProcess:
                    self.logger.error("無法獲取進程信息")
                    return False
            else:
                stdout, stderr = process.communicate()
                self.logger.error(f"{script_name} 啟動失敗")
                self.logger.error(f"返回碼: {process.returncode}")
                if stderr:
                    self.logger.error(f"錯誤: {stderr[:300]}")
                return False
                
        except Exception as e:
            self.logger.error(f"啟動 {script_name} 失敗: {e}", exc_info=True)
            return False
    
    def get_memory_info(self):
        """獲取記憶體使用資訊"""
        if not self.process_info or not self.process_info['psutil_process']:
            return None
        
        try:
            proc = self.process_info['psutil_process']
            
            # 檢查進程是否還存在
            if not proc.is_running():
                return None
            
            memory_info = proc.memory_info()
            memory_percent = proc.memory_percent()
            
            # 獲取子進程記憶體（如果有）
            children_memory = 0
            try:
                for child in proc.children(recursive=True):
                    if child.is_running():
                        children_memory += child.memory_info().rss
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
            
            return {
                'rss': memory_info.rss,  # 物理記憶體
                'vms': memory_info.vms,  # 虛擬記憶體
                'percent': memory_percent,  # 記憶體使用百分比
                'children_rss': children_memory,  # 子進程記憶體
                'total_rss': memory_info.rss + children_memory,  # 總記憶體
                'timestamp': datetime.now()
            }
            
        except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
            self.logger.warning(f"獲取記憶體資訊失敗: {e}")
            return None
        except Exception as e:
            self.logger.error(f"記憶體監控異常: {e}", exc_info=True)
            return None
    
    def format_memory_size(self, bytes_size):
        """格式化記憶體大小顯示"""
        for unit in ['B', 'KB', 'MB', 'GB']:
            if bytes_size < 1024.0:
                return f"{bytes_size:.2f} {unit}"
            bytes_size /= 1024.0
        return f"{bytes_size:.2f} TB"
    
    def start_memory_monitoring(self):
        """啟動記憶體監控執行緒"""
        if self.monitoring:
            self.logger.warning("記憶體監控已在運行中")
            return
        
        self.monitoring = True
        self.memory_data = []
        
        def monitor_loop():
            self.logger.info("記憶體監控執行緒已啟動")
            
            while self.monitoring:
                memory_info = self.get_memory_info()
                
                if memory_info:
                    self.memory_data.append(memory_info)
                    
                    # 每10次記錄輸出一次詳細資訊
                    if len(self.memory_data) % 10 == 0:
                        rss_mb = memory_info['rss'] / (1024 * 1024)
                        total_mb = memory_info['total_rss'] / (1024 * 1024)
                        
                        self.logger.debug(
                            f"記憶體使用 - 主進程: {rss_mb:.2f} MB, "
                            f"總計: {total_mb:.2f} MB, "
                            f"百分比: {memory_info['percent']:.2f}%"
                        )
                else:
                    self.logger.warning("無法獲取記憶體資訊，進程可能已終止")
                    break
                
                time.sleep(1)  # 每秒監控一次
            
            self.logger.info("記憶體監控執行緒已停止")
        
        monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        monitor_thread.start()
    
    def stop_memory_monitoring(self):
        """停止記憶體監控"""
        self.monitoring = False
    
    def kill_and_restart_ccd1(self):
        """殺掉CCD1進程並重啟，測量時間"""
        if not self.process_info:
            self.logger.error("沒有運行中的CCD1進程")
            return None
        
        self.logger.info("開始CCD1重啟測試")
        
        # 記錄當前記憶體狀態
        final_memory = self.get_memory_info()
        if final_memory:
            final_mb = final_memory['total_rss'] / (1024 * 1024)
            self.logger.info(f"重啟前記憶體使用: {final_mb:.2f} MB")
        
        # 停止記憶體監控
        self.stop_memory_monitoring()
        
        # 殺掉進程
        kill_start_time = time.time()
        
        try:
            process = self.process_info['process']
            psutil_proc = self.process_info['psutil_process']
            
            self.logger.info(f"終止進程 PID: {process.pid}")
            
            # 嘗試優雅終止
            process.terminate()
            
            # 等待進程終止
            try:
                process.wait(timeout=5)
                kill_time = time.time() - kill_start_time
                self.logger.info(f"進程已正常終止，耗時: {kill_time:.2f} 秒")
            except subprocess.TimeoutExpired:
                self.logger.warning("進程未響應，強制終止")
                process.kill()
                kill_time = time.time() - kill_start_time
                self.logger.info(f"進程已強制終止，耗時: {kill_time:.2f} 秒")
            
        except Exception as e:
            self.logger.error(f"終止進程失敗: {e}", exc_info=True)
            kill_time = time.time() - kill_start_time
        
        # 等待系統資源釋放
        time.sleep(2)
        
        # 重啟進程
        restart_start_time = time.time()
        self.process_info = None
        
        if self.start_ccd1():
            restart_time = time.time() - restart_start_time
            total_time = time.time() - kill_start_time
            
            self.logger.info(f"CCD1重啟成功")
            self.logger.info(f"終止時間: {kill_time:.2f} 秒")
            self.logger.info(f"重啟時間: {restart_time:.2f} 秒")
            self.logger.info(f"總計時間: {total_time:.2f} 秒")
            
            restart_data = {
                'timestamp': datetime.now(),
                'kill_time': kill_time,
                'restart_time': restart_time,
                'total_time': total_time,
                'final_memory_mb': final_mb if final_memory else 0
            }
            
            self.restart_times.append(restart_data)
            
            # 重新開始記憶體監控
            self.start_memory_monitoring()
            
            return restart_data
        else:
            self.logger.error("CCD1重啟失敗")
            return None
    
    def show_memory_statistics(self):
        """顯示記憶體使用統計"""
        if not self.memory_data:
            self.logger.info("沒有記憶體監控數據")
            return
        
        # 計算統計資訊
        rss_values = [data['rss'] for data in self.memory_data]
        total_values = [data['total_rss'] for data in self.memory_data]
        percent_values = [data['percent'] for data in self.memory_data]
        
        rss_avg = sum(rss_values) / len(rss_values) / (1024 * 1024)  # MB
        rss_max = max(rss_values) / (1024 * 1024)  # MB
        rss_min = min(rss_values) / (1024 * 1024)  # MB
        
        total_avg = sum(total_values) / len(total_values) / (1024 * 1024)  # MB
        total_max = max(total_values) / (1024 * 1024)  # MB
        total_min = min(total_values) / (1024 * 1024)  # MB
        
        percent_avg = sum(percent_values) / len(percent_values)
        percent_max = max(percent_values)
        
        print("\n" + "="*60)
        print("記憶體使用統計報告")
        print("="*60)
        print(f"監控時間: {len(self.memory_data)} 秒")
        print(f"採樣次數: {len(self.memory_data)} 次")
        print()
        print("主進程記憶體 (RSS):")
        print(f"  平均: {rss_avg:.2f} MB")
        print(f"  最大: {rss_max:.2f} MB")
        print(f"  最小: {rss_min:.2f} MB")
        print()
        print("總記憶體 (含子進程):")
        print(f"  平均: {total_avg:.2f} MB")
        print(f"  最大: {total_max:.2f} MB")
        print(f"  最小: {total_min:.2f} MB")
        print()
        print("記憶體使用百分比:")
        print(f"  平均: {percent_avg:.2f}%")
        print(f"  最大: {percent_max:.2f}%")
        print("="*60)
    
    def show_restart_statistics(self):
        """顯示重啟時間統計"""
        if not self.restart_times:
            self.logger.info("沒有重啟時間數據")
            return
        
        kill_times = [data['kill_time'] for data in self.restart_times]
        restart_times = [data['restart_time'] for data in self.restart_times]
        total_times = [data['total_time'] for data in self.restart_times]
        
        print("\n" + "="*60)
        print("重啟時間統計報告")
        print("="*60)
        print(f"重啟次數: {len(self.restart_times)}")
        print()
        print("終止時間:")
        print(f"  平均: {sum(kill_times)/len(kill_times):.2f} 秒")
        print(f"  最快: {min(kill_times):.2f} 秒")
        print(f"  最慢: {max(kill_times):.2f} 秒")
        print()
        print("重啟時間:")
        print(f"  平均: {sum(restart_times)/len(restart_times):.2f} 秒")
        print(f"  最快: {min(restart_times):.2f} 秒")
        print(f"  最慢: {max(restart_times):.2f} 秒")
        print()
        print("總計時間:")
        print(f"  平均: {sum(total_times)/len(total_times):.2f} 秒")
        print(f"  最快: {min(total_times):.2f} 秒")
        print(f"  最慢: {max(total_times):.2f} 秒")
        print("="*60)
        
        # 顯示每次重啟詳細記錄
        print("\n重啟記錄明細:")
        print("-"*60)
        for i, data in enumerate(self.restart_times, 1):
            print(f"第{i}次 [{data['timestamp'].strftime('%H:%M:%S')}] "
                  f"終止:{data['kill_time']:.2f}s 重啟:{data['restart_time']:.2f}s "
                  f"總計:{data['total_time']:.2f}s 記憶體:{data['final_memory_mb']:.1f}MB")
    
    def show_current_status(self):
        """顯示當前狀態"""
        print("\n" + "="*60)
        print("CCD1記憶體監控狀態")
        print("="*60)
        
        if self.process_info:
            process = self.process_info['process']
            
            if process.poll() is None:
                # 進程運行中
                uptime = datetime.now() - self.process_info['start_time']
                startup_time = self.process_info.get('startup_time', 'N/A')
                
                print(f"狀態: 運行中")
                print(f"PID: {process.pid}")
                print(f"啟動時間: {startup_time} 秒" if isinstance(startup_time, float) else f"啟動時間: {startup_time}")
                print(f"運行時間: {str(uptime).split('.')[0]}")
                print(f"監控狀態: {'啟用' if self.monitoring else '停用'}")
                print(f"記憶體數據點: {len(self.memory_data)}")
                
                # 顯示當前記憶體使用
                current_memory = self.get_memory_info()
                if current_memory:
                    rss_mb = current_memory['rss'] / (1024 * 1024)
                    total_mb = current_memory['total_rss'] / (1024 * 1024)
                    print(f"當前記憶體: {total_mb:.2f} MB (主進程: {rss_mb:.2f} MB)")
                    print(f"記憶體百分比: {current_memory['percent']:.2f}%")
            else:
                print(f"狀態: 已停止")
        else:
            print(f"狀態: 未啟動")
        
        print(f"重啟測試次數: {len(self.restart_times)}")
        print("="*60)
    
    def monitor_interactive(self):
        """互動式監控模式"""
        print("\n進入互動監控模式")
        print("可用命令: status, memory, restart, stats, stop, help, quit")
        
        try:
            while True:
                command = input("\n> ").strip().lower()
                
                if command == 'status':
                    self.show_current_status()
                elif command == 'memory':
                    self.show_memory_statistics()
                elif command == 'restart':
                    restart_data = self.kill_and_restart_ccd1()
                    if restart_data:
                        print(f"重啟完成 - 總時間: {restart_data['total_time']:.2f} 秒")
                elif command == 'stats':
                    self.show_restart_statistics()
                elif command == 'stop':
                    self.stop_ccd1()
                    break
                elif command == 'help':
                    self.show_help()
                elif command in ['quit', 'exit']:
                    print("正在關閉監控...")
                    self.stop_ccd1()
                    break
                else:
                    print("未知命令，輸入 'help' 查看可用命令")
                    
        except KeyboardInterrupt:
            print("\n收到中斷信號，正在關閉...")
            self.stop_ccd1()
    
    def stop_ccd1(self):
        """停止CCD1進程"""
        if not self.process_info:
            self.logger.info("沒有運行中的CCD1進程")
            return
        
        self.logger.info("停止CCD1進程")
        
        # 停止記憶體監控
        self.stop_memory_monitoring()
        
        try:
            process = self.process_info['process']
            
            if process.poll() is None:
                self.logger.info(f"終止進程 PID: {process.pid}")
                process.terminate()
                
                try:
                    process.wait(timeout=5)
                    self.logger.info("CCD1進程已正常停止")
                except subprocess.TimeoutExpired:
                    self.logger.warning("CCD1未響應，強制終止")
                    process.kill()
                    self.logger.info("CCD1進程已強制停止")
            else:
                self.logger.info("CCD1進程已經停止")
                
        except Exception as e:
            self.logger.error(f"停止CCD1進程失敗: {e}", exc_info=True)
        finally:
            self.process_info = None
    
    def show_help(self):
        """顯示幫助信息"""
        print("\n可用命令:")
        print("  status   - 顯示當前運行狀態")
        print("  memory   - 顯示記憶體使用統計")
        print("  restart  - 執行重啟測試")
        print("  stats    - 顯示重啟時間統計")
        print("  stop     - 停止CCD1並退出")
        print("  help     - 顯示此幫助")
        print("  quit     - 退出監控程序")


def main():
    """主函數"""
    print("="*60)
    print("CCD1記憶體監控與重啟評估工具")
    print("="*60)
    print("功能:")
    print("  • 監控CCD1進程記憶體使用量")
    print("  • 測量CCD1重啟時間性能")
    print("  • 提供詳細統計分析")
    print("="*60)
    
    monitor = CCD1MemoryMonitor()
    
    # 檢查conda環境
    if not monitor.check_conda_environment():
        monitor.logger.error("conda環境檢查失敗，無法繼續")
        input("按Enter退出...")
        return
    
    # 檢查腳本檔案
    if not monitor.check_script_file():
        monitor.logger.error("CCD1腳本檔案檢查失敗，無法繼續")
        input("按Enter退出...")
        return
    
    # 啟動CCD1
    if not monitor.start_ccd1():
        monitor.logger.error("CCD1啟動失敗")
        input("按Enter退出...")
        return
    
    # 開始記憶體監控
    monitor.start_memory_monitoring()
    
    # 進入互動模式
    monitor.monitor_interactive()
    
    print("\nCCD1監控工具已關閉")


if __name__ == "__main__":
    main()