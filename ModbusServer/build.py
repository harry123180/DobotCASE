#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[U+81EA][U+52D5][U+5316][U+5EFA][U+7F6E][U+8173][U+672C]
[U+7528][U+65BC][U+5C07] Modbus TCP Server [U+6253][U+5305][U+6210][U+53EF][U+57F7][U+884C][U+6A94][U+6848]
"""

import os
import sys
import shutil
import subprocess
import zipfile
from pathlib import Path

def check_requirements():
    """[U+6AA2][U+67E5][U+5EFA][U+7F6E][U+9700][U+6C42]"""
    print("[U+1F50D] [U+6AA2][U+67E5][U+5EFA][U+7F6E][U+74B0][U+5883]...")
    
    # [U+6AA2][U+67E5] Python [U+7248][U+672C]
    if sys.version_info < (3, 7):
        print("[FAIL] [U+9700][U+8981] Python 3.7 [U+6216][U+66F4][U+9AD8][U+7248][U+672C]")
        return False
    
    # [U+6AA2][U+67E5][U+5FC5][U+8981][U+6A94][U+6848]
    required_files = [
        'TCPServer.py',
        'templates/index.html',
        'static/style.css',
        'static/script.js'
    ]
    
    for file in required_files:
        if not os.path.exists(file):
            print(f"[FAIL] [U+7F3A][U+5C11][U+5FC5][U+8981][U+6A94][U+6848]: {file}")
            return False
    
    print("[OK] [U+5EFA][U+7F6E][U+74B0][U+5883][U+6AA2][U+67E5][U+901A][U+904E]")
    return True

def install_dependencies():
    """[U+5B89][U+88DD][U+5EFA][U+7F6E][U+4F9D][U+8CF4]"""
    print("[U+1F4E6] [U+5B89][U+88DD][U+5EFA][U+7F6E][U+4F9D][U+8CF4]...")
    
    dependencies = [
        'pyinstaller>=5.0',
        'pymodbus>=3.0.0',
        'flask>=2.0.0',
        'werkzeug>=2.0.0'
    ]
    
    for dep in dependencies:
        print(f"[U+5B89][U+88DD] {dep}...")
        result = subprocess.run([sys.executable, '-m', 'pip', 'install', dep], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print(f"[FAIL] [U+5B89][U+88DD] {dep} [U+5931][U+6557]")
            print(result.stderr)
            return False
    
    print("[OK] [U+4F9D][U+8CF4][U+5B89][U+88DD][U+5B8C][U+6210]")
    return True

def create_directories():
    """[U+5275][U+5EFA][U+5FC5][U+8981][U+7684][U+76EE][U+9304][U+7D50][U+69CB]"""
    print("[U+1F4C1] [U+5275][U+5EFA][U+76EE][U+9304][U+7D50][U+69CB]...")
    
    directories = [
        'templates',
        'static',
        'build',
        'dist'
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
    
    print("[OK] [U+76EE][U+9304][U+7D50][U+69CB][U+5275][U+5EFA][U+5B8C][U+6210]")

def create_html_template():
    """[U+5275][U+5EFA] HTML [U+6A21][U+677F][U+6A94][U+6848]"""
    print("[U+1F4C4] [U+5275][U+5EFA] HTML [U+6A21][U+677F]...")
    
    html_content = '''<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Modbus TCP Server [U+7BA1][U+7406][U+4ECB][U+9762]</title>
    <link rel="stylesheet" href="/static/style.css">
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>[U+1F527] Modbus TCP Server [U+7BA1][U+7406][U+4ECB][U+9762]</h1>
            <p>[U+4F3A][U+670D][U+5668][U+5730][U+5740]: 0.0.0.0:502 | Web[U+7BA1][U+7406]: 0.0.0.0:8000 | [U+6578][U+503C][U+7BC4][U+570D]: 0-65535 ([U+7121][U+7B26][U+865F])</p>
        </div>
        
        <!-- [U+4F3A][U+670D][U+5668][U+72C0][U+614B][U+5340][U+584A] -->
        <div class="section status">
            <h2>[U+1F4CA] [U+4F3A][U+670D][U+5668][U+72C0][U+614B]</h2>
            <div id="status-info">[U+8F09][U+5165][U+4E2D]...</div>
            <button onclick="refreshStatus()">[U+1F504] [U+5237][U+65B0][U+72C0][U+614B]</button>
            <button onclick="toggleAutoRefresh()" id="auto-refresh-btn">[U+1F504] [U+958B][U+555F][U+81EA][U+52D5][U+5237][U+65B0]</button>
        </div>
        
        <!-- [U+63A7][U+5236][U+9762][U+677F] -->
        <div class="section controls">
            <h2>[U+2699][U+FE0F] [U+63A7][U+5236][U+9762][U+677F]</h2>
            <div class="control-row">
                <label>SlaveID (1-247):</label>
                <input type="number" id="slave-id" min="1" max="247" value="1">
                <button onclick="updateSlaveId()">[U+66F4][U+65B0] SlaveID</button>
            </div>
            <div class="control-row">
                <label>[U+66AB][U+5B58][U+5668][U+5730][U+5740] (0-999):</label>
                <input type="number" id="reg-address" min="0" max="999" value="0">
                <label>[U+503C] (0 ~ 65535):</label>
                <input type="number" id="reg-value" min="0" max="65535" value="0">
                <button onclick="writeRegister()">[U+5BEB][U+5165][U+66AB][U+5B58][U+5668]</button>
                <button onclick="readRegister()">[U+8B80][U+53D6][U+66AB][U+5B58][U+5668]</button>
            </div>
            <div id="result-message"></div>
        </div>
        
        <!-- [U+66AB][U+5B58][U+5668][U+7BC4][U+570D][U+986F][U+793A][U+8A2D][U+5B9A] -->
        <div class="section display-settings">
            <h2>[U+1F5A5][U+FE0F] [U+986F][U+793A][U+8A2D][U+5B9A]</h2>
            <div class="control-row">
                <label>[U+8D77][U+59CB][U+5730][U+5740]:</label>
                <input type="number" id="display-start" min="0" max="999" value="0">
                <label>[U+986F][U+793A][U+6578][U+91CF]:</label>
                <input type="number" id="display-count" min="1" max="100" value="20">
                <label>[U+986F][U+793A][U+683C][U+5F0F]:</label>
                <select id="display-format">
                    <option value="decimal">[U+7121][U+7B26][U+865F][U+5341][U+9032][U+5236]</option>
                    <option value="hex">[U+5341][U+516D][U+9032][U+5236]</option>
                    <option value="binary">[U+4E8C][U+9032][U+5236]</option>
                    <option value="signed">[U+6709][U+7B26][U+865F][U+5341][U+9032][U+5236]</option>
                </select>
                <button onclick="updateDisplay()">[U+66F4][U+65B0][U+986F][U+793A]</button>
            </div>
        </div>
        
        <!-- [U+66AB][U+5B58][U+5668][U+986F][U+793A][U+5340][U+57DF] -->
        <div class="section registers">
            <h2>[U+1F4CB] [U+66AB][U+5B58][U+5668][U+72C0][U+614B]</h2>
            <div class="registers-header">
                <span>[U+5730][U+5740][U+7BC4][U+570D]: <span id="address-range">0-19</span></span>
                <span>[U+683C][U+5F0F]: <span id="current-format">[U+7121][U+7B26][U+865F][U+5341][U+9032][U+5236]</span></span>
            </div>
            <div id="registers-grid" class="register-grid">[U+8F09][U+5165][U+4E2D]...</div>
        </div>
        
        <!-- [U+5FEB][U+901F][U+64CD][U+4F5C][U+5340] -->
        <div class="section quick-actions">
            <h2>[U+26A1] [U+5FEB][U+901F][U+64CD][U+4F5C]</h2>
            <div class="control-row">
                <button onclick="clearAllRegisters()">[U+6E05][U+9664][U+6240][U+6709][U+66AB][U+5B58][U+5668]</button>
                <button onclick="setTestData()">[U+8A2D][U+5B9A][U+6E2C][U+8A66][U+6578][U+64DA]</button>
                <button onclick="exportRegisters()">[U+532F][U+51FA][U+66AB][U+5B58][U+5668][U+6578][U+64DA]</button>
                <input type="file" id="import-file" accept=".json" style="display:none" onchange="importRegisters()">
                <button onclick="document.getElementById('import-file').click()">[U+532F][U+5165][U+66AB][U+5B58][U+5668][U+6578][U+64DA]</button>
            </div>
        </div>
        
        <!-- [U+4F7F][U+7528][U+8AAA][U+660E] -->
        <div class="section">
            <h2>[U+1F4D6] [U+4F7F][U+7528][U+8AAA][U+660E]</h2>
            <ul>
                <li><strong>Modbus Poll [U+9023][U+63A5][U+8A2D][U+5B9A]:</strong> [U+9023][U+63A5][U+5230][U+76EE][U+6A19][U+96FB][U+8166]IP:502</li>
                <li><strong>Function Code:</strong> [U+4F7F][U+7528] 0x03 (Read Holding Registers) [U+8B80][U+53D6][U+66AB][U+5B58][U+5668]</li>
                <li><strong>SlaveID:</strong> [U+9810][U+8A2D][U+70BA] 1[U+FF0C][U+53EF][U+900F][U+904E][U+63A7][U+5236][U+9762][U+677F][U+4FEE][U+6539]</li>
                <li><strong>[U+66AB][U+5B58][U+5668][U+7BC4][U+570D]:</strong> 0-999 ([U+5171]1000[U+500B][U+66AB][U+5B58][U+5668])</li>
                <li><strong>[U+6578][U+64DA][U+985E][U+578B]:</strong> 16[U+4F4D][U+7121][U+7B26][U+865F][U+6574][U+6578] (0 [U+5230] 65535)</li>
                <li><strong>[U+6709][U+7B26][U+865F][U+986F][U+793A]:</strong> [U+9078][U+64C7][U+300C][U+6709][U+7B26][U+865F][U+5341][U+9032][U+5236][U+300D][U+683C][U+5F0F][U+6642][U+FF0C][U+5927][U+65BC]32767[U+7684][U+503C][U+6703][U+986F][U+793A][U+70BA][U+8CA0][U+6578]</li>
                <li><strong>[U+5373][U+6642][U+540C][U+6B65]:</strong> [U+652F][U+63F4][U+5916][U+90E8]Modbus[U+8A2D][U+5099][U+8B80][U+5BEB][U+6642][U+7684][U+5373][U+6642][U+66F4][U+65B0]</li>
                <li><strong>[U+8A3B][U+89E3][U+529F][U+80FD]:</strong> [U+9EDE][U+64CA][U+66AB][U+5B58][U+5668][U+683C][U+5B50][U+53EF][U+6DFB][U+52A0][U+7528][U+9014][U+8A3B][U+89E3]</li>
                <li><strong>[U+683C][U+5F0F][U+8F49][U+63DB]:</strong> [U+652F][U+63F4][U+5341][U+9032][U+5236][U+3001][U+5341][U+516D][U+9032][U+5236][U+3001][U+4E8C][U+9032][U+5236][U+548C][U+6709][U+7B26][U+865F][U+5341][U+9032][U+5236][U+986F][U+793A]</li>
            </ul>
        </div>
    </div>

    <script src="/static/script.js"></script>
</body>
</html>'''
    
    with open('templates/index.html', 'w', encoding='utf-8') as f:
        f.write(html_content)
    
    print("[OK] HTML [U+6A21][U+677F][U+5275][U+5EFA][U+5B8C][U+6210]")

def build_executable():
    """[U+4F7F][U+7528] PyInstaller [U+5EFA][U+7F6E][U+53EF][U+57F7][U+884C][U+6A94]"""
    print("[U+1F528] [U+958B][U+59CB][U+5EFA][U+7F6E][U+53EF][U+57F7][U+884C][U+6A94]...")
    
    # [U+6E05][U+7406][U+820A][U+7684][U+5EFA][U+7F6E][U+6A94][U+6848]
    if os.path.exists('build'):
        shutil.rmtree('build')
    if os.path.exists('dist'):
        shutil.rmtree('dist')
    
    # [U+57F7][U+884C] PyInstaller
    cmd = [
        sys.executable, '-m', 'PyInstaller',
        '--clean',
        '--noconfirm',
        'modbus_server.spec'
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        print("[FAIL] [U+5EFA][U+7F6E][U+5931][U+6557]")
        print(result.stderr)
        return False
    
    print("[OK] [U+53EF][U+57F7][U+884C][U+6A94][U+5EFA][U+7F6E][U+5B8C][U+6210]")
    return True

def create_installer_config():
    """[U+5275][U+5EFA][U+5B89][U+88DD][U+7A0B][U+5F0F][U+914D][U+7F6E][U+6A94]"""
    print("[U+1F4DD] [U+5275][U+5EFA][U+90E8][U+7F72][U+914D][U+7F6E]...")
    
    # [U+5275][U+5EFA] README
    readme_content = '''# Modbus TCP Server

## [U+7CFB][U+7D71][U+9700][U+6C42]
- Windows 10/11 (64-bit)
- [U+53EF][U+7528][U+8A18][U+61B6][U+9AD4]: [U+6700][U+5C11] 512MB
- [U+7DB2][U+8DEF][U+9023][U+63A5][U+57E0]: 502 (Modbus TCP), 8000 (Web[U+7BA1][U+7406][U+4ECB][U+9762])

## [U+5B89][U+88DD][U+6B65][U+9A5F]
1. [U+89E3][U+58D3][U+7E2E][U+6A94][U+6848][U+5230][U+76EE][U+6A19][U+76EE][U+9304]
2. [U+4EE5][U+7BA1][U+7406][U+54E1][U+6B0A][U+9650][U+57F7][U+884C] ModbusTCPServer.exe
3. [U+5728][U+700F][U+89BD][U+5668][U+4E2D][U+958B][U+555F] http://localhost:8000

## [U+9632][U+706B][U+7246][U+8A2D][U+5B9A]
[U+8ACB][U+78BA][U+4FDD] Windows [U+9632][U+706B][U+7246][U+5141][U+8A31][U+7A0B][U+5F0F][U+4F7F][U+7528][U+9023][U+63A5][U+57E0] 502 [U+548C] 8000

## [U+7DB2][U+8DEF][U+9023][U+63A5]
- Modbus TCP [U+9023][U+63A5][U+57E0]: 502
- Web [U+7BA1][U+7406][U+4ECB][U+9762]: 8000
- [U+652F][U+63F4][U+7684] Modbus [U+529F][U+80FD][U+78BC]: 0x03 (Read Holding Registers), 0x06 (Write Single Register)

## [U+6280][U+8853][U+652F][U+63F4]
[U+5982][U+6709][U+554F][U+984C][U+8ACB][U+6AA2][U+67E5] modbus_server.log [U+65E5][U+8A8C][U+6A94][U+6848]
'''
    
    with open('dist/ModbusTCPServer/README.txt', 'w', encoding='utf-8') as f:
        f.write(readme_content)
    
    # [U+5275][U+5EFA][U+555F][U+52D5][U+8173][U+672C]
    start_script = '''@echo off
title Modbus TCP Server
echo [U+6B63][U+5728][U+555F][U+52D5] Modbus TCP Server...
echo.
echo Modbus TCP [U+9023][U+63A5][U+57E0]: 502
echo Web [U+7BA1][U+7406][U+4ECB][U+9762]: http://localhost:8000
echo.
echo [U+8ACB][U+4FDD][U+6301][U+6B64][U+8996][U+7A97][U+958B][U+555F][U+FF0C][U+95DC][U+9589][U+8996][U+7A97][U+5C07][U+505C][U+6B62][U+4F3A][U+670D][U+5668]
echo [U+6309] Ctrl+C [U+53EF][U+5B89][U+5168][U+95DC][U+9589][U+4F3A][U+670D][U+5668]
echo.
ModbusTCPServer.exe
pause
'''
    
    with open('dist/ModbusTCPServer/start_server.bat', 'w', encoding='utf-8') as f:
        f.write(start_script)
    
    print("[OK] [U+90E8][U+7F72][U+914D][U+7F6E][U+5275][U+5EFA][U+5B8C][U+6210]")

def create_zip_package():
    """[U+5275][U+5EFA][U+90E8][U+7F72][U+58D3][U+7E2E][U+5305]"""
    print("[U+1F4E6] [U+5275][U+5EFA][U+90E8][U+7F72][U+58D3][U+7E2E][U+5305]...")
    
    zip_filename = 'ModbusTCPServer_Production.zip'
    
    with zipfile.ZipFile(zip_filename, 'w', zipfile.ZIP_DEFLATED) as zipf:
        # [U+904D][U+6B77] dist [U+76EE][U+9304][U+4E26][U+52A0][U+5165][U+6240][U+6709][U+6A94][U+6848]
        for root, dirs, files in os.walk('dist/ModbusTCPServer'):
            for file in files:
                file_path = os.path.join(root, file)
                arcname = os.path.relpath(file_path, 'dist')
                zipf.write(file_path, arcname)
    
    file_size = os.path.getsize(zip_filename) / (1024 * 1024)  # MB
    print(f"[OK] [U+90E8][U+7F72][U+5305][U+5275][U+5EFA][U+5B8C][U+6210]: {zip_filename} ({file_size:.1f} MB)")

def main():
    """[U+4E3B][U+5EFA][U+7F6E][U+6D41][U+7A0B]"""
    print("[U+1F680] Modbus TCP Server [U+5EFA][U+7F6E][U+5DE5][U+5177]")
    print("=" * 50)
    
    # [U+6AA2][U+67E5][U+74B0][U+5883]
    if not check_requirements():
        return False
    
    # [U+5B89][U+88DD][U+4F9D][U+8CF4]
    if not install_dependencies():
        return False
    
    # [U+5275][U+5EFA][U+76EE][U+9304]
    create_directories()
    
    # [U+5275][U+5EFA][U+6A94][U+6848] ([U+5982][U+679C][U+4E0D][U+5B58][U+5728])
    if not os.path.exists('templates/index.html'):
        create_html_template()
    
    # [U+5EFA][U+7F6E][U+53EF][U+57F7][U+884C][U+6A94]
    if not build_executable():
        return False
    
    # [U+5275][U+5EFA][U+90E8][U+7F72][U+914D][U+7F6E]
    create_installer_config()
    
    # [U+5275][U+5EFA][U+58D3][U+7E2E][U+5305]
    create_zip_package()
    
    print("\n[U+1F389] [U+5EFA][U+7F6E][U+5B8C][U+6210][U+FF01]")
    print(f"[U+1F4C1] [U+53EF][U+57F7][U+884C][U+6A94][U+4F4D][U+7F6E]: dist/ModbusTCPServer/")
    print(f"[U+1F4E6] [U+90E8][U+7F72][U+5305]: ModbusTCPServer_Production.zip")
    print(f"[U+1F310] [U+4F7F][U+7528][U+65B9][U+5F0F]: [U+89E3][U+58D3][U+7E2E][U+5F8C][U+57F7][U+884C] start_server.bat")
    
    return True

if __name__ == "__main__":
    success = main()
    if not success:
        print("\n[FAIL] [U+5EFA][U+7F6E][U+5931][U+6557][U+FF0C][U+8ACB][U+6AA2][U+67E5][U+932F][U+8AA4][U+8A0A][U+606F]")
        sys.exit(1)
    else:
        print("\n[OK] [U+5EFA][U+7F6E][U+6210][U+529F][U+FF01]")
        sys.exit(0)