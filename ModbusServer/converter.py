#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[U+5716][U+793A][U+8F49][U+63DB][U+5DE5][U+5177]
[U+5C07] PNG [U+5716][U+793A][U+8F49][U+63DB][U+70BA] ICO [U+683C][U+5F0F][U+FF0C][U+7528][U+65BC] PyInstaller [U+5EFA][U+7F6E]
"""

import os
import sys
from PIL import Image

def convert_png_to_ico(png_path, ico_path):
    """[U+5C07] PNG [U+5716][U+793A][U+8F49][U+63DB][U+70BA] ICO [U+683C][U+5F0F]"""
    try:
        print(f"[U+1F504] [U+8F49][U+63DB][U+5716][U+793A]: {png_path} -> {ico_path}")
        
        # [U+958B][U+555F] PNG [U+5716][U+7247]
        with Image.open(png_path) as img:
            # [U+78BA][U+4FDD][U+5716][U+7247][U+70BA] RGBA [U+6A21][U+5F0F]
            if img.mode != 'RGBA':
                img = img.convert('RGBA')
            
            # [U+5275][U+5EFA][U+591A][U+7A2E][U+5C3A][U+5BF8][U+7684][U+5716][U+793A] (Windows [U+6A19][U+6E96][U+5C3A][U+5BF8])
            sizes = [(16, 16), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)]
            
            # [U+8ABF][U+6574][U+5716][U+7247][U+5C3A][U+5BF8][U+4E26][U+4FDD][U+5B58][U+70BA] ICO
            resized_images = []
            for size in sizes:
                resized_img = img.resize(size, Image.Resampling.LANCZOS)
                resized_images.append(resized_img)
            
            # [U+4FDD][U+5B58][U+70BA] ICO [U+6A94][U+6848]
            resized_images[0].save(
                ico_path, 
                format='ICO', 
                sizes=sizes
            )
        
        print(f"[OK] [U+5716][U+793A][U+8F49][U+63DB][U+5B8C][U+6210]: {ico_path}")
        return True
        
    except ImportError:
        print("[FAIL] [U+7F3A][U+5C11] Pillow [U+5EAB][U+FF0C][U+6B63][U+5728][U+5B89][U+88DD]...")
        import subprocess
        result = subprocess.run([sys.executable, '-m', 'pip', 'install', 'Pillow'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("[OK] Pillow [U+5B89][U+88DD][U+5B8C][U+6210][U+FF0C][U+8ACB][U+91CD][U+65B0][U+904B][U+884C][U+6B64][U+8173][U+672C]")
        else:
            print("[FAIL] Pillow [U+5B89][U+88DD][U+5931][U+6557]")
        return False
        
    except Exception as e:
        print(f"[FAIL] [U+5716][U+793A][U+8F49][U+63DB][U+5931][U+6557]: {e}")
        return False

def create_alternative_spec():
    """[U+5275][U+5EFA][U+4F7F][U+7528] PNG [U+5716][U+793A][U+7684][U+66FF][U+4EE3] spec [U+6A94][U+6848]"""
    spec_content = '''# -*- mode: python ; coding: utf-8 -*-
# modbus_server_png.spec
# PyInstaller [U+898F][U+683C][U+6A94][U+6848][U+7528][U+65BC][U+6253][U+5305] Modbus TCP Server ([U+4F7F][U+7528] PNG [U+5716][U+793A])

block_cipher = None

# [U+6536][U+96C6][U+6240][U+6709][U+9700][U+8981][U+7684][U+6578][U+64DA][U+6A94][U+6848]
datas = [
    ('templates', 'templates'),
    ('static', 'static'),
    ('640.png', '.'),  # [U+5305][U+542B][U+5716][U+793A][U+6A94][U+6848]
]

# [U+96B1][U+85CF][U+5C0E][U+5165][U+7684][U+6A21][U+7D44]
hiddenimports = [
    'pymodbus',
    'pymodbus.server',
    'pymodbus.device',
    'pymodbus.datastore',
    'pymodbus.client',
    'flask',
    'werkzeug',
    'jinja2',
    'markupsafe',
    'click',
    'itsdangerous',
    'cryptography',
    'pyserial',
    'twisted',
    'pymodbus.framer.socket_framer',
    'pymodbus.framer.rtu_framer',
    'pymodbus.framer.ascii_framer',
    'pymodbus.transaction',
    'pymodbus.constants',
    'pymodbus.utilities',
    'logging.handlers',
]

a = Analysis(
    ['TCPServer.py'],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'matplotlib',
        'numpy',
        'pandas',
        'scipy',
        'tkinter',
        'PIL',
        'cv2',
        'torch',
        'tensorflow',
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='ModbusTCPServer',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    # [U+6CE8][U+610F]: PNG [U+5716][U+793A][U+9700][U+8981][U+5148][U+8F49][U+63DB][U+70BA] ICO [U+683C][U+5F0F]
    # icon='640.ico'  # [U+5982][U+679C][U+6709][U+8F49][U+63DB][U+7684] ICO [U+6A94][U+6848]
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='ModbusTCPServer'
)'''
    
    with open('modbus_server_png.spec', 'w', encoding='utf-8') as f:
        f.write(spec_content)
    
    print("[OK] [U+5DF2][U+5275][U+5EFA] PNG [U+5716][U+793A][U+5C08][U+7528][U+7684] spec [U+6A94][U+6848]: modbus_server_png.spec")

def main():
    """[U+4E3B][U+51FD][U+6578]"""
    print("[U+1F3A8] Modbus TCP Server [U+5716][U+793A][U+8F49][U+63DB][U+5DE5][U+5177]")
    print("=" * 50)
    
    png_file = '640.png'
    ico_file = '640.ico'
    
    # [U+6AA2][U+67E5] PNG [U+6A94][U+6848][U+662F][U+5426][U+5B58][U+5728]
    if not os.path.exists(png_file):
        print(f"[FAIL] [U+627E][U+4E0D][U+5230][U+5716][U+793A][U+6A94][U+6848]: {png_file}")
        return False
    
    print(f"[U+1F4C1] [U+627E][U+5230][U+5716][U+793A][U+6A94][U+6848]: {png_file}")
    
    # [U+5617][U+8A66][U+8F49][U+63DB][U+70BA] ICO [U+683C][U+5F0F]
    if convert_png_to_ico(png_file, ico_file):
        print(f"\n[OK] [U+8F49][U+63DB][U+6210][U+529F][U+FF01][U+73FE][U+5728][U+53EF][U+4EE5][U+5728] PyInstaller [U+4E2D][U+4F7F][U+7528] {ico_file}")
        print("[U+1F4A1] [U+5EFA][U+8B70][U+5728] modbus_server.spec [U+4E2D][U+5C07] icon [U+8A2D][U+5B9A][U+70BA] '640.ico'")
    else:
        print(f"\n[WARN][U+FE0F]  [U+7121][U+6CD5][U+8F49][U+63DB][U+70BA] ICO [U+683C][U+5F0F][U+FF0C][U+5275][U+5EFA][U+66FF][U+4EE3][U+65B9][U+6848]...")
        create_alternative_spec()
        print("[U+1F4A1] [U+8ACB][U+4F7F][U+7528] modbus_server_png.spec [U+9032][U+884C][U+5EFA][U+7F6E]")
    
    return True

if __name__ == "__main__":
    success = main()
    if not success:
        print("\n[FAIL] [U+5716][U+793A][U+8655][U+7406][U+5931][U+6557]")
        sys.exit(1)
    else:
        print("\n[OK] [U+5716][U+793A][U+8655][U+7406][U+5B8C][U+6210][U+FF01]")
        sys.exit(0)