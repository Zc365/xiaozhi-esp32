@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

echo ========================================
echo 中文字体生成工具 - ESP32 LVGL项目
echo ========================================
echo.

:: 检查Python是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo 错误: Python未安装或未添加到PATH
    echo 请先安装Python 3.6+
    pause
    exit /b 1
)

:: 检查Node.js是否安装
node --version >nul 2>&1
if errorlevel 1 (
    echo 错误: Node.js未安装或未添加到PATH
    echo 请先安装Node.js v14+
    pause
    exit /b 1
)

:: 检查lv_font_conv是否安装
lv_font_conv --help >nul 2>&1
if errorlevel 1 (
    echo 警告: lv_font_conv未安装
    echo 正在安装lv_font_conv...
    npm install -g lv_font_conv
    if errorlevel 1 (
        echo 错误: lv_font_conv安装失败
        echo 请手动运行: npm install -g lv_font_conv
        pause
        exit /b 1
    )
    echo lv_font_conv安装成功
    echo.
)

:: 查找字体文件
set "FONT_FILE="
set "FONT_PATHS=PuHuiTi-Regular.ttf puhui.ttf fonts\PuHuiTi-Regular.ttf fonts\puhui.ttf C:\Windows\Fonts\msyh.ttf C:\Windows\Fonts\simsun.ttc"

for %%f in (%FONT_PATHS%) do (
    if exist "%%f" (
        set "FONT_FILE=%%f"
        goto :found_font
    )
)

:found_font
if "%FONT_FILE%"=="" (
    echo 未找到字体文件，请选择以下选项：
    echo 1. 手动指定字体文件路径
    echo 2. 退出
    set /p choice="请选择 (1-2): "
    
    if "!choice!"=="1" (
        set /p FONT_FILE="请输入字体文件完整路径: "
        if not exist "!FONT_FILE!" (
            echo 错误: 指定的字体文件不存在
            pause
            exit /b 1
        )
    ) else (
        exit /b 0
    )
) else (
    echo 找到字体文件: %FONT_FILE%
)

echo.
echo 请选择字体生成方式：
echo 1. 生成常用中文字体 (推荐，文件较小)
echo 2. 生成完整普惠字体 (文件较大)
echo 3. 生成完整普惠字体 (仅常用字符)
echo 4. 退出
echo.
set /p choice="请选择 (1-4): "

if "%choice%"=="1" goto :common_chinese
if "%choice%"=="2" goto :full_puhui
if "%choice%"=="3" goto :puhui_common
if "%choice%"=="4" goto :exit
goto :invalid_choice

:common_chinese
echo.
echo 正在生成常用中文字体...
echo 字体文件: %FONT_FILE%
echo 输出目录: .\common_fonts
echo.
python generate_common_chinese_font.py --font "%FONT_FILE%" --output .\common_fonts --sizes "14,16,18,20" --bpp 1
if errorlevel 1 (
    echo 字体生成失败
    pause
    exit /b 1
)
echo.
echo 常用中文字体生成完成！
echo 生成的文件位于: .\common_fonts\
goto :success

:full_puhui
echo.
echo 正在生成完整普惠字体...
echo 字体文件: %FONT_FILE%
echo 输出目录: .\generated_fonts
echo.
python generate_puhui_font.py --font "%FONT_FILE%" --output .\generated_fonts
if errorlevel 1 (
    echo 字体生成失败
    pause
    exit /b 1
)
echo.
echo 完整普惠字体生成完成！
echo 生成的文件位于: .\generated_fonts\
goto :success

:puhui_common
echo.
echo 正在生成普惠字体 (仅常用字符)...
echo 字体文件: %FONT_FILE%
echo 输出目录: .\generated_fonts
echo.
python generate_puhui_font.py --font "%FONT_FILE%" --output .\generated_fonts --common-only
if errorlevel 1 (
    echo 字体生成失败
    pause
    exit /b 1
)
echo.
echo 普惠字体 (常用字符) 生成完成！
echo 生成的文件位于: .\generated_fonts\
goto :success

:success
echo.
echo ========================================
echo 字体生成成功！
echo ========================================
echo.
echo 使用说明：
echo 1. 将生成的 .c 和 .h 文件复制到你的ESP32项目中
echo 2. 在 CMakeLists.txt 中添加 .c 文件
echo 3. 在代码中包含相应的 .h 文件
echo 4. 使用字体: lv_style_set_text_font(^&style, ^&font_name);
echo.
echo 详细说明请查看: README_font_generation.md
echo.
goto :end

:invalid_choice
echo 无效选择，请重新运行脚本
pause
exit /b 1

:exit
echo 退出字体生成工具
exit /b 0

:end
echo 按任意键退出...
pause >nul