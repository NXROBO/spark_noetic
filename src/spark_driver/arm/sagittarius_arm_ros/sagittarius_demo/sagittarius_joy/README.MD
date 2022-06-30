使用方式

连接 xbox 或 ps4, 新终端输入下面指令

```
roslaunch sagittarius_joy joy_demo.launch
```
`controller_type` 是手柄类型，可选为[xbox360, ps4]，默认使用 xbox 手柄

然后可以使用手柄控制机械臂

控制方式

左摇杆前/后: end-effect x 加/减
左摇杆左/右: 腰部旋转 左摆/右摆
左摇杆摁下: 无

右摇杆前/后: end-effect pitch 往下看/往上看
右摇杆左/右: end-effect y 加/减
右摇杆摁下: 特殊设置 摆正夹爪

LB(L1): 高度减少
RB(R1): 高度增加

BACK(SHARE): 逆时针翻滚
START(OPTION): 顺时针翻滚

LT(L2): 夹爪减少间距
RT(R2): 夹爪增加间距

方向键上/下: 速度增加/减少
方向键上/下: 无

B(cycle): 预设姿态 home
A(cross): 预设姿态 sleep

长按 power 1秒: 扭矩输出切换
