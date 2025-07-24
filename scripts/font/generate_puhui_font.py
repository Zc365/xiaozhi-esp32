#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Puhui Font Generator for ESP32 LVGL Project

This script generates Chinese Puhui fonts for LVGL using lv_font_conv.
It creates font files with common Chinese characters and ASCII characters.

Requirements:
- Node.js v14+ installed
- lv_font_conv installed globally: npm i lv_font_conv -g
- Puhui TTF font file

Usage:
    python generate_puhui_font.py
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path

# Font configuration
FONT_CONFIGS = {
    'puhui_14_1': {
        'size': 14,
        'bpp': 1,
        'name': 'font_puhui_14_1',
        'output': 'font_puhui_14_1.c'
    },
    'puhui_16_1': {
        'size': 16,
        'bpp': 1,
        'name': 'font_puhui_16_1',
        'output': 'font_puhui_16_1.c'
    },
    'puhui_18_1': {
        'size': 18,
        'bpp': 1,
        'name': 'font_puhui_18_1',
        'output': 'font_puhui_18_1.c'
    },
    'puhui_20_1': {
        'size': 20,
        'bpp': 1,
        'name': 'font_puhui_20_1',
        'output': 'font_puhui_20_1.c'
    }
}

# Character ranges for Chinese fonts
CHARACTER_RANGES = [
    # Basic ASCII characters
    '0x20-0x7F',
    # Common Chinese characters (CJK Unified Ideographs)
    '0x4E00-0x9FFF',
    # Chinese punctuation
    '0x3000-0x303F',
    # CJK symbols and punctuation
    '0x3400-0x4DBF',
    # Additional common symbols
    '0xFF00-0xFFEF'
]

# Reduced character ranges for smaller fonts (commonly used characters)
COMMON_CHINESE_RANGES = [
    # Basic ASCII characters
    '0x20-0x7F',
    # Most common Chinese characters (first 3000 characters)
    '0x4E00-0x5FFF',
    # Chinese punctuation
    '0x3000-0x303F'
]

def check_dependencies():
    """Check if required dependencies are installed."""
    try:
        # Check Node.js
        result = subprocess.run(['node', '--version'], capture_output=True, text=True)
        if result.returncode != 0:
            print("Error: Node.js is not installed or not in PATH")
            return False
        print(f"Node.js version: {result.stdout.strip()}")
        
        # Check lv_font_conv
        result = subprocess.run(['lv_font_conv', '--help'], capture_output=True, text=True)
        if result.returncode != 0:
            print("Error: lv_font_conv is not installed")
            print("Install it with: npm i lv_font_conv -g")
            return False
        print("lv_font_conv is available")
        
        return True
    except FileNotFoundError:
        print("Error: Node.js or lv_font_conv not found")
        return False

def find_puhui_font():
    """Find Puhui TTF font file in common locations."""
    possible_paths = [
        # Current directory
        './PuHuiTi-Regular.ttf',
        './puhui.ttf',
        './fonts/PuHuiTi-Regular.ttf',
        './fonts/puhui.ttf',
        # Scripts directory
        '../fonts/PuHuiTi-Regular.ttf',
        '../fonts/puhui.ttf',
        # Common system font directories (Windows)
        'C:/Windows/Fonts/PuHuiTi-Regular.ttf',
        # User provided path
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            print(f"Found Puhui font: {path}")
            return path
    
    return None

def generate_font(font_path, config_name, config, output_dir, use_common_ranges=False):
    """Generate a single font file using lv_font_conv."""
    print(f"\nGenerating {config_name}...")
    
    # Choose character ranges based on font size
    ranges = COMMON_CHINESE_RANGES if use_common_ranges or config['size'] <= 16 else CHARACTER_RANGES
    
    # Build lv_font_conv command
    cmd = [
        'lv_font_conv',
        '--font', font_path,
        '--size', str(config['size']),
        '--bpp', str(config['bpp']),
        '--format', 'lvgl',
        '--output', os.path.join(output_dir, config['output'])
    ]
    
    # Add character ranges
    for range_str in ranges:
        cmd.extend(['-r', range_str])
    
    # Add compression (optional)
    if config['bpp'] > 1:
        cmd.append('--no-compress')
    
    print(f"Command: {' '.join(cmd)}")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        if result.returncode == 0:
            print(f"✓ Successfully generated {config['output']}")
            
            # Add font declaration to the generated file
            output_file = os.path.join(output_dir, config['output'])
            add_font_declaration(output_file, config['name'])
            
            return True
        else:
            print(f"✗ Error generating {config['output']}:")
            print(f"stdout: {result.stdout}")
            print(f"stderr: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print(f"✗ Timeout generating {config['output']}")
        return False
    except Exception as e:
        print(f"✗ Exception generating {config['output']}: {e}")
        return False

def add_font_declaration(file_path, font_name):
    """Add LV_FONT_DECLARE to the generated font file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Add declaration at the beginning
        declaration = f"#include \"lvgl.h\"\n\nLV_FONT_DECLARE({font_name});\n\n"
        
        # Find the font variable definition and replace it
        lines = content.split('\n')
        new_lines = []
        
        for line in lines:
            if line.startswith('const lv_font_t '):
                # Replace the font variable name
                new_line = line.replace('const lv_font_t ', f'const lv_font_t {font_name} = ')
                new_lines.append(new_line)
            else:
                new_lines.append(line)
        
        # Write back the modified content
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(new_lines))
        
        print(f"  Added font declaration for {font_name}")
        
    except Exception as e:
        print(f"  Warning: Could not add font declaration: {e}")

def create_font_header(output_dir):
    """Create a header file with all font declarations."""
    header_content = '''#ifndef FONT_PUHUI_H
#define FONT_PUHUI_H

#include "lvgl.h"

// Puhui font declarations
'''
    
    for config_name, config in FONT_CONFIGS.items():
        header_content += f'LV_FONT_DECLARE({config["name"]});\n'
    
    header_content += '''\n#endif // FONT_PUHUI_H\n'''
    
    header_file = os.path.join(output_dir, 'font_puhui.h')
    with open(header_file, 'w', encoding='utf-8') as f:
        f.write(header_content)
    
    print(f"\n✓ Created header file: {header_file}")

def main():
    parser = argparse.ArgumentParser(description='Generate Puhui fonts for LVGL')
    parser.add_argument('--font', help='Path to Puhui TTF font file')
    parser.add_argument('--output', default='./generated_fonts', help='Output directory')
    parser.add_argument('--common-only', action='store_true', help='Use only common Chinese characters')
    parser.add_argument('--config', choices=list(FONT_CONFIGS.keys()), help='Generate specific font config only')
    
    args = parser.parse_args()
    
    print("Puhui Font Generator for ESP32 LVGL")
    print("====================================")
    
    # Check dependencies
    if not check_dependencies():
        sys.exit(1)
    
    # Find font file
    font_path = args.font or find_puhui_font()
    if not font_path:
        print("\nError: Puhui font file not found!")
        print("Please provide the path to Puhui TTF font file using --font argument")
        print("Or place the font file in one of these locations:")
        print("  - ./PuHuiTi-Regular.ttf")
        print("  - ./fonts/PuHuiTi-Regular.ttf")
        sys.exit(1)
    
    # Create output directory
    output_dir = args.output
    os.makedirs(output_dir, exist_ok=True)
    print(f"\nOutput directory: {output_dir}")
    
    # Generate fonts
    success_count = 0
    total_count = 0
    
    configs_to_generate = {args.config: FONT_CONFIGS[args.config]} if args.config else FONT_CONFIGS
    
    for config_name, config in configs_to_generate.items():
        total_count += 1
        if generate_font(font_path, config_name, config, output_dir, args.common_only):
            success_count += 1
    
    # Create header file
    if success_count > 0:
        create_font_header(output_dir)
    
    # Summary
    print(f"\n{'='*50}")
    print(f"Font generation completed: {success_count}/{total_count} successful")
    
    if success_count > 0:
        print(f"\nGenerated files in {output_dir}:")
        for file in os.listdir(output_dir):
            if file.endswith(('.c', '.h')):
                print(f"  - {file}")
        
        print("\nTo use these fonts in your ESP32 project:")
        print("1. Copy the generated .c and .h files to your project")
        print("2. Add the .c files to your CMakeLists.txt")
        print("3. Include font_puhui.h in your code")
        print("4. Use the fonts like: lv_style_set_text_font(&style, &font_puhui_14_1);")
    
    if success_count < total_count:
        sys.exit(1)

if __name__ == '__main__':
    main()