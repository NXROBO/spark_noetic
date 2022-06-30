#!/bin/bash

Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 流程如下："
echo -e "${Info} 1. 获取方块的 hsv"
echo -e "${Info}      1). 机械臂会运行到指定位置，请把方块放置到机械臂夹爪正下方"
echo -e "${Info}      2). 调整摄像头位置，让绿色框对准蓝色方块顶面"
echo -e "${Info}      3). 获取绿色框内 HSV 均值"
echo -e "${Info} 2. 使用九点标定法标定机械臂"
echo -e "${Info}      1). 机械臂会运行到指定位置，请把方块放置到机械臂夹爪正下方"
echo -e "${Info}      2). 程序根据 HSV 获取并保存方块在摄像头上的位置"
echo -e "${Info}      3). 运行下一个标定点，直到九个标定点都完成"
echo -e "${Info} 3. 计算线性回归后的值，并保存到文件"

echo -e ""
echo -e "${Info} 一切准备好？\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'start'" -1 >/dev/null
rostopic pub /cali_cmd_topic std_msgs/String "data: 'start'" -1 >/dev/null

echo -e ""
echo -e "${Info} 请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null
rostopic pub /cali_cmd_topic std_msgs/String "data: 'start2'" -1 >/dev/null

echo -e ""
echo -e "${Info} 请调整摄像头，把绿色框对准了蓝色方块顶面位置\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_cmd_topic std_msgs/String "data: 'start3'" -1 >/dev/null

sleep 5
echo -e ""
echo -e "${Info} (1/9) No.01"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 3
echo -e ""
echo -e "${Info} (2/9) No.02"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 3
echo -e ""
echo -e "${Info} (3/9) No.03"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 3
echo -e ""
echo -e "${Info} (4/9) No.04"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 3
echo -e ""
echo -e "${Info} (5/9) No.05"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 3
echo -e ""
echo -e "${Info} (6/9) No.06"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 3
echo -e ""
echo -e "${Info} (7/9) No.07"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 3
echo -e ""
echo -e "${Info} (8/9) No.08"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 3
echo -e ""
echo -e "${Info} (9/9) No.09"
echo -e "${Info} 等待机械臂运动到新位置后，请把蓝色方块放到机械臂夹爪的正下方\c"
echo && stty erase '^H' && read -p "退出请输入 Ctrl + c，完成后按回车键继续..." 
rostopic pub /cali_arm_cmd_topic std_msgs/String "data: 'go'" -1 >/dev/null

sleep 5
echo && stty erase '^H' && read -p "标定结束，按任意键退出..." 
exit