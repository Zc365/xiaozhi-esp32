# 中文字体生成工具使用说明

本目录包含用于生成ESP32 LVGL项目中文字体的Python脚本。

## 文件说明

- `generate_puhui_font.py` - 完整的普惠字体生成器，支持全字符集
- `generate_common_chinese_font.py` - 常用中文字符字体生成器，文件更小
- `README_font_generation.md` - 本说明文档

## 环境要求

### 1. 安装 Node.js
下载并安装 Node.js v14 或更高版本：
- 官网：https://nodejs.org/
- 推荐安装 LTS 版本

### 2. 安装 lv_font_conv
```bash
# 全局安装 lv_font_conv
npm install -g lv_font_conv

# 验证安装
lv_font_conv --help
```

### 3. 准备字体文件
需要准备中文TTF字体文件，推荐：
- 阿里巴巴普惠体 (PuHuiTi-Regular.ttf)
- 思源黑体 (SourceHanSansSC-Regular.ttf)
- 微软雅黑 (msyh.ttf)

## 使用方法

### 方法一：生成完整普惠字体

```bash
# 基本用法（自动查找字体文件）
python generate_puhui_font.py

# 指定字体文件路径
python generate_puhui_font.py --font /path/to/PuHuiTi-Regular.ttf

# 指定输出目录
python generate_puhui_font.py --font /path/to/font.ttf --output ./my_fonts

# 只生成常用字符（减小文件大小）
python generate_puhui_font.py --font /path/to/font.ttf --common-only

# 只生成特定尺寸
python generate_puhui_font.py --font /path/to/font.ttf --config puhui_16_1
```

### 方法二：生成常用中文字体（推荐用于ESP32）

```bash
# 基本用法
python generate_common_chinese_font.py --font /path/to/font.ttf

# 指定字体尺寸
python generate_common_chinese_font.py --font /path/to/font.ttf --sizes "12,14,16,18"

# 指定输出目录和位深
python generate_common_chinese_font.py --font /path/to/font.ttf --output ./fonts --bpp 2
```

## 字符范围说明

### 完整字体包含：
- ASCII字符 (0x20-0x7F)
- 中文标点符号 (0x3000-0x303F)
- CJK统一汉字 (0x4E00-0x9FFF) - 约20,000字符
- CJK扩展A (0x3400-0x4DBF)
- 全角字符 (0xFF00-0xFFEF)

### 常用字体包含：
- ASCII字符 (0x20-0x7F)
- 中文标点符号 (0x3000-0x303F)
- 最常用的1000+个中文字符

## 生成的文件

### 完整字体生成器输出：
```
generated_fonts/
├── font_puhui_14_1.c
├── font_puhui_16_1.c
├── font_puhui_18_1.c
├── font_puhui_20_1.c
└── font_puhui.h
```

### 常用字体生成器输出：
```
common_fonts/
├── font_chinese_common_14_1.c
├── font_chinese_common_16_1.c
├── font_chinese_common_18_1.c
├── font_chinese_common_20_1.c
└── common_chinese_fonts.h
```

## 在ESP32项目中使用

### 1. 复制文件到项目
将生成的 `.c` 和 `.h` 文件复制到你的ESP32项目中。

### 2. 修改 CMakeLists.txt
在组件的 `CMakeLists.txt` 中添加字体文件：

```cmake
idf_component_register(
    SRCS 
        "main.c"
        "font_puhui_16_1.c"  # 添加字体文件
        "font_chinese_common_16_1.c"
        # ... 其他源文件
    INCLUDE_DIRS 
        "."
    REQUIRES 
        "lvgl"
        # ... 其他依赖
)
```

### 3. 在代码中使用

```c
#include "font_puhui.h"
// 或
#include "common_chinese_fonts.h"

// 设置字体
lv_style_t style;
lv_style_init(&style);
lv_style_set_text_font(&style, &font_puhui_16_1);
// 或
lv_style_set_text_font(&style, &font_chinese_common_16_1);

// 应用到标签
lv_obj_t *label = lv_label_create(parent);
lv_obj_add_style(label, &style, 0);
lv_label_set_text(label, "你好世界");
```

## 字体文件大小对比

| 字体类型 | 字符数量 | 14px文件大小 | 16px文件大小 | 18px文件大小 |
|---------|---------|-------------|-------------|-------------|
| 完整CJK | ~20,000 | ~800KB | ~1.2MB | ~1.6MB |
| 常用字符 | ~1,000 | ~40KB | ~60KB | ~80KB |

**建议：**
- 对于内存受限的ESP32项目，推荐使用常用字符字体
- 如果需要显示更多汉字，可以使用完整字体但选择较小的尺寸
- 1bpp字体比4bpp字体小4倍，但显示效果较差

## 故障排除

### 1. lv_font_conv 命令未找到
```bash
# 检查Node.js安装
node --version

# 重新安装lv_font_conv
npm uninstall -g lv_font_conv
npm install -g lv_font_conv
```

### 2. 字体文件未找到
确保字体文件路径正确，支持的格式：
- TTF (TrueType Font)
- OTF (OpenType Font)
- WOFF/WOFF2

### 3. 生成的字体文件过大
- 使用 `--common-only` 选项
- 减少字体尺寸
- 使用1bpp而不是4bpp
- 使用常用字符生成器

### 4. 编译错误
确保：
- 字体文件已添加到CMakeLists.txt
- 包含了正确的头文件
- LVGL配置正确

## Unicode字符范围参考

<mcreference link="https://en.wikipedia.org/wiki/CJK_Unified_Ideographs" index="1">1</mcreference> <mcreference link="https://symbl.cc/en/unicode/blocks/cjk-unified-ideographs/" index="3">3</mcreference>

- **CJK统一汉字**: 0x4E00-0x9FFF (20,992个字符)
- **CJK扩展A**: 0x3400-0x4DBF (6,592个字符)
- **中文标点**: 0x3000-0x303F (64个字符)
- **ASCII**: 0x20-0x7F (95个字符)

## 相关链接

<mcreference link="https://www.npmjs.com/package/lv_font_conv" index="1">1</mcreference> <mcreference link="https://github.com/lvgl/lv_font_conv" index="3">3</mcreference> <mcreference link="https://docs.lvgl.io/master/details/main-modules/font.html" index="2">2</mcreference>

- [lv_font_conv NPM包](https://www.npmjs.com/package/lv_font_conv)
- [lv_font_conv GitHub仓库](https://github.com/lvgl/lv_font_conv)
- [LVGL字体文档](https://docs.lvgl.io/master/details/main-modules/font.html)
- [LVGL在线字体转换器](https://lvgl.io/tools/fontconverter)