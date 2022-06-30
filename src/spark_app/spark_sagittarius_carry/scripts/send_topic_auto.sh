#!/bin/bash

Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 流程如下："
echo -e "${Info} 1. 获取方块的 hsv"
echo -e "${Info}      1). 机械臂会运行到指定位置, 请把方块放置到机械臂夹爪正下方"
echo -e "${Info}      2). 调整摄像头位置, 让绿色框对准蓝色方块顶面"
echo -e "${Info}      3). 程序获取绿色框内 HSV 均值"
echo -e "${Info} 2. 机械臂开始进行标定, 过程自动"
echo -e "${Info} 3. 计算线性回归后的值, 并保存到文件"

echo -e ""
echo -e "${Info} 一切准备好？\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c, 完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'start'" -1 >/dev/null
rostopic pub /cali_cmd_topic std_msgs/String "data: 'start'" -1 >/dev/null

echo -e ""
echo -e "${Info} 请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c, 完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null
rostopic pub /cali_cmd_topic std_msgs/String "data: 'start2'" -1 >/dev/null

echo -e ""
echo -e "${Info} 请调整摄像头, 把绿色框对准了蓝色方块顶面位置\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c, 完成后按回车键继续..." 
rostopic pub /cali_cmd_topic std_msgs/String "data: 'start3'" -1 >/dev/null

sleep 5
echo -e ""
echo -e "${Info} 机械臂开始自动标定"
echo -e "${Info} 当机械臂停止运动, 而且原来窗口显示完成标定的提示"
echo -e "${Info} 说明标定完成, 可在原来的窗口按 Ctrl-C 退出程序"
echo && stty erase '^H' && read -p "当前窗口可退出, 按回车键退出当前窗口..." 
exit