#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Common Chinese Font Generator for ESP32 LVGL Project

This script generates Chinese fonts with only the most commonly used characters
to reduce font file size for embedded systems.

Requirements:
- Node.js v14+ installed
- lv_font_conv installed globally: npm i lv_font_conv -g
- Chinese TTF font file (Puhui, SimSun, etc.)

Usage:
    python generate_common_chinese_font.py --font path/to/font.ttf
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path

# Most common Chinese characters (top 1000)
COMMON_CHINESE_CHARS = (
    "的一是在不了有和人这中大为上个国我以要他时来用们生到作地于出就分对成会可主发年动同工也能下过子说产种面而方后多定行学法所民得经十三之进着等部度家电力里如水化高自二理起小物现实加量都两体制机当使点从业本去把性好应开它合还因由其些然前外天政四日那社义事平形相全表间样与关各重新线内数正心反你明看原又么利比或但质气第向道命此变条只没结解问意建月公无系军很情者最立代想已通并提直题党程展五果料象员革位入常文总次品式活设及管特件长求老头基资边流路级少图山统接知较将组见计别她手角期根论运农指几九区强放决西被干做必战先回则任取据处队南给色光门即保治北造百规热领七海口东导器压志世金增争济阶油思术极交受联什认六共权收证改清己美再采转更单风切打白教速花带安场身车例真务具万每目至达走积示议声报斗完类八离华名确才科张信马节话米整空元况今集温传土许步群广石记需段研界拉林律叫且究观越织装影算低持音众书布复容儿须际商非验连断深难近矿千周委素技备半办青省列习响约支般史感劳便团往酸历市克何除消构府称太准精值号率族维划选标写存候毛亲快效斯院查江型眼王按格养易置派层片始却专状育厂京识适属圆包火住调满县局照参红细引听该铁价严"
    "首底液官德调随病苦倒注意云造字施展台环境食住房严格执行计划经济体制改革开放政策法律法规制度建设发展社会主义市场经济科学技术教育文化卫生体育新闻出版广播电视电影艺术哲学社会科学自然科学工程技术农业林业水利交通运输邮电通信商业贸易金融保险房地产建筑业制造业采掘业电力煤气水生产供应业"
)

# Convert characters to Unicode ranges
def chars_to_unicode_list(chars):
    """Convert character string to list of Unicode code points."""
    unicode_list = []
    for char in chars:
        code_point = ord(char)
        unicode_list.append(f"0x{code_point:X}")
    return unicode_list

# Basic character ranges
BASIC_RANGES = [
    '0x20-0x7F',    # ASCII
    '0x3000-0x303F', # CJK punctuation
]

def check_lv_font_conv():
    """Check if lv_font_conv is available."""
    try:
        result = subprocess.run(['lv_font_conv', '--help'], capture_output=True, text=True)
        return result.returncode == 0
    except FileNotFoundError:
        return False

def generate_font_with_common_chars(font_path, size, bpp, output_name, output_dir):
    """Generate font with common Chinese characters."""
    print(f"Generating {output_name} (size: {size}, bpp: {bpp})...")
    
    # Get Unicode points for common characters
    unicode_chars = chars_to_unicode_list(COMMON_CHINESE_CHARS)
    
    # Build command
    cmd = [
        'lv_font_conv',
        '--font', font_path,
        '--size', str(size),
        '--bpp', str(bpp),
        '--format', 'lvgl',
        '--output', os.path.join(output_dir, f'{output_name}.c')
    ]
    
    # Add basic ranges
    for range_str in BASIC_RANGES:
        cmd.extend(['-r', range_str])
    
    # Add common Chinese characters in batches (lv_font_conv has command line length limits)
    batch_size = 100
    for i in range(0, len(unicode_chars), batch_size):
        batch = unicode_chars[i:i+batch_size]
        batch_str = ','.join(batch)
        cmd.extend(['-r', batch_str])
    
    print(f"Total characters: {len(COMMON_CHINESE_CHARS)} + ASCII + punctuation")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        if result.returncode == 0:
            print(f"✓ Successfully generated {output_name}.c")
            
            # Modify the generated file to use correct font name
            output_file = os.path.join(output_dir, f'{output_name}.c')
            fix_font_name(output_file, output_name)
            
            return True
        else:
            print(f"✗ Error: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("✗ Timeout during font generation")
        return False
    except Exception as e:
        print(f"✗ Exception: {e}")
        return False

def fix_font_name(file_path, font_name):
    """Fix the font variable name in the generated C file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Replace the default font variable name
        lines = content.split('\n')
        new_lines = []
        
        for line in lines:
            if 'const lv_font_t' in line and '=' in line:
                # Extract everything after 'const lv_font_t ' and before ' ='
                parts = line.split('const lv_font_t ')
                if len(parts) > 1:
                    after_parts = parts[1].split(' = ')
                    if len(after_parts) > 1:
                        new_line = f"const lv_font_t {font_name} = {after_parts[1]}"
                        new_lines.append(new_line)
                        continue
            new_lines.append(line)
        
        # Add header includes
        header = '#include "lvgl.h"\n\n'
        final_content = header + '\n'.join(new_lines)
        
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(final_content)
        
        print(f"  Fixed font name to {font_name}")
        
    except Exception as e:
        print(f"  Warning: Could not fix font name: {e}")

def create_header_file(font_names, output_dir):
    """Create header file with font declarations."""
    header_content = '''#ifndef COMMON_CHINESE_FONTS_H
#define COMMON_CHINESE_FONTS_H

#include "lvgl.h"

// Common Chinese font declarations
'''
    
    for font_name in font_names:
        header_content += f'LV_FONT_DECLARE({font_name});\n'
    
    header_content += '''\n#endif // COMMON_CHINESE_FONTS_H\n'''
    
    header_file = os.path.join(output_dir, 'common_chinese_fonts.h')
    with open(header_file, 'w', encoding='utf-8') as f:
        f.write(header_content)
    
    print(f"✓ Created header file: {header_file}")

def main():
    parser = argparse.ArgumentParser(description='Generate common Chinese fonts for LVGL')
    parser.add_argument('--font', required=True, help='Path to Chinese TTF font file')
    parser.add_argument('--output', default='./common_fonts', help='Output directory')
    parser.add_argument('--sizes', default='14,16,18,20', help='Font sizes (comma-separated)')
    parser.add_argument('--bpp', type=int, default=1, choices=[1,2,4,8], help='Bits per pixel')
    
    args = parser.parse_args()
    
    print("Common Chinese Font Generator for ESP32 LVGL")
    print("=============================================")
    print(f"Using {len(COMMON_CHINESE_CHARS)} most common Chinese characters")
    
    # Check dependencies
    if not check_lv_font_conv():
        print("Error: lv_font_conv not found")
        print("Install it with: npm i lv_font_conv -g")
        sys.exit(1)
    
    # Check font file
    if not os.path.exists(args.font):
        print(f"Error: Font file not found: {args.font}")
        sys.exit(1)
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    print(f"Output directory: {args.output}")
    
    # Parse sizes
    try:
        sizes = [int(s.strip()) for s in args.sizes.split(',')]
    except ValueError:
        print("Error: Invalid sizes format. Use comma-separated integers like '14,16,18'")
        sys.exit(1)
    
    # Generate fonts
    generated_fonts = []
    success_count = 0
    
    for size in sizes:
        font_name = f"font_chinese_common_{size}_{args.bpp}"
        if generate_font_with_common_chars(args.font, size, args.bpp, font_name, args.output):
            generated_fonts.append(font_name)
            success_count += 1
    
    # Create header file
    if generated_fonts:
        create_header_file(generated_fonts, args.output)
    
    # Summary
    print(f"\n{'='*50}")
    print(f"Generated {success_count}/{len(sizes)} fonts successfully")
    
    if generated_fonts:
        print(f"\nGenerated files:")
        for font in generated_fonts:
            print(f"  - {font}.c")
        print(f"  - common_chinese_fonts.h")
        
        print(f"\nFont file sizes (approximate):")
        for font in generated_fonts:
            file_path = os.path.join(args.output, f"{font}.c")
            if os.path.exists(file_path):
                size_kb = os.path.getsize(file_path) / 1024
                print(f"  - {font}.c: {size_kb:.1f} KB")
        
        print("\nUsage in your ESP32 project:")
        print("1. Copy generated .c and .h files to your project")
        print("2. Add .c files to CMakeLists.txt")
        print("3. Include common_chinese_fonts.h")
        print("4. Use fonts like: lv_style_set_text_font(&style, &font_chinese_common_16_1);")

if __name__ == '__main__':
    main()