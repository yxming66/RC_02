autoCtrlAPI 开发计划表
阶段 0：先定边界与原则
0.1 明确 autoCtrlAPI 的职责
负责：
区块定义
区块转移查询
自动流程状态机
执行前 yaw 对正
跨越模板调度
过程状态/结果上报
不负责：
全场路径规划
最终行为决策
上位机定位算法
底层电机驱动细节
0.2 明确输入输出
输入：
当前区块
目标区块 / 下一跳区块
IMU yaw
SICK 距离
光电状态
杆状态 / 底盘状态
输出：
底盘速度指令
杆控制指令
当前 auto 状态
成功/失败/故障码
阶段 1：建立核心定义层（core）
目标：先把“自动系统说的话”统一下来，不写复杂逻辑。

1.1 建立基础头文件
建议先建：

User/module/autoCtrlAPI/core/auto_ctrl_def.h
User/module/autoCtrlAPI/core/auto_ctrl_math.h
1.2 定义核心枚举
先定义：

auto_zone_e
auto_template_e
auto_sensor_mode_e
auto_run_state_e
auto_result_e
auto_fault_e
1.3 定义区块属性结构体
字段建议：

区块编号
区块名称
高度 height_mm
是否平台区
1.4 定义转移表结构体
字段建议最少包含：

from_zone
to_zone
template_id
required_yaw_deg
yaw_tolerance_deg
sensor_mode
height_delta_mm
阶段 2：建立区块转移层（transition）
目标：先让系统知道“有哪些区块、哪些相邻、每条边怎么走”。

2.1 建立区块表
第一版只做最小集合：

R2_ENTRY
R2_EXIT
PLATFORM_1 ~ PLATFORM_12
2.2 建立平台高度表
按你当前定义录入：

2/4/10/12 -> 200
1/3/5/7/9/11 -> 400
6/8 -> 600
2.3 建立第一版转移表
先只录入明确要用的相邻转移，不要一次做满全图。

建议第一批先录：

平台内部相邻横向边
平台内部相邻纵向边
R2_ENTRY 到上排平台
下排平台到 R2_EXIT
2.4 给每条边填模板类型
先做这几类：

FLAT_MOVE
ASCEND_200_STD
DESCEND_200_STD
ASCEND_400_STD
DESCEND_400_STD
ASCEND_200_NO_FRONT_SICK
DESCEND_200_NO_FRONT_SICK
阶段 3：建立查询接口
目标：先能“查数据”，不急着动作。

3.1 实现区块查询函数
例如：

按 zone id 查区块信息
查区块高度
判断是否平台区
3.2 实现转移查询函数
例如：

查 from -> to 是否存在
返回对应 transition
返回所需模板和目标 yaw
3.3 实现辅助判断函数
例如：

高差是否合法
模板是否合法
当前目标是否相邻
阶段 4：建立调用接口层（api）
目标：先把 API 壳子搭起来，让上层能调用，但不要提前混入模板/基础动作实现。

4.1 定义控制上下文 auto_ctrl_t
建议包含：

当前运行状态
当前任务
当前区块
目标区块
当前转移指针
当前模板
子步骤索引
定时器/超时计数
yaw 零位偏置
当前结果/故障码
4.2 提供初始化接口
例如：

AutoCtrl_Init()
AutoCtrl_Reset()
4.3 提供零位设置接口
例如：

AutoCtrl_SetYawZeroOffset(raw_yaw)
AutoCtrl_CalibrateYawZero()
4.4 提供任务启动接口
例如：

AutoCtrl_StartTransition(from, to)
AutoCtrl_StartGoto(target_zone)（后面再扩）
4.5 提供周期运行接口
例如：

AutoCtrl_Update()
4.6 提供状态查询接口
例如：

AutoCtrl_IsBusy()
AutoCtrl_GetState()
AutoCtrl_GetResult()
AutoCtrl_GetFault()
阶段 5：建立基础动作层（primitive）与 yaw 数学模块（core）
目标：先把“执行前自动校正方向”单独做稳。

5.1 实现 yaw 归一化函数
必须有：

角度 wrap 到 [-180, 180]
计算目标角误差
5.2 实现 auto yaw 计算
基于：

原始 IMU yaw
yaw_zero_offset
得到：

yaw_auto_deg
5.3 实现对正判定逻辑
输入：

required_yaw_deg
yaw_tolerance_deg
输出：

是否已对正
5.4 实现对正控制输出
先做简单版本：

只输出底盘角速度命令
先不加复杂横向修正
5.5 预留 SICK 辅助修正接口
先把接口留好：

use_sick_assist
sick_alignment_valid
第一版先可为空实现。

此外基础动作层至少应独立文件承载：

- 杆控制原语
- 底盘速度控制原语
- 传感器读取原语/占位适配
- 超时与保护原语

阶段 6：建立 step 层
目标：先把模板依赖的单步动作抽象出来，不直接在模板层或 API 层里堆动作细节。

6.1 定义 step 上下文
至少包含：

step id
step enter time
step timeout
step result

6.2 定义 step 语义
每个 step 至少具备：

进入动作
持续执行
完成条件
失败条件
超时条件

6.3 先实现最小 step 执行器
例如：

STOP_CHASSIS
SET_POLE_TARGET
FLAT_MOVE_TIME
WAIT_TIMEOUT

阶段 7：建立模板执行框架（template）
目标：先有“模板调度器”，即使模板内部先写空壳也行。

7.1 定义模板执行状态
例如：

IDLE
PREALIGN
RUN_TEMPLATE
SUCCESS
FAIL
ABORT
7.2 先实现模板框架，不填满动作细节
先让模板能按序跑：

进入模板
进入 step0
step 完成后进入 step1
全部完成后返回 success
阶段 8：实现第一批模板
目标：先跑通最常用跨越，不要一口气写完全部。

建议实现顺序如下。

8.1 FLAT_MOVE
最简单，先打通调度链路。

8.2 ASCEND_200_STD
最核心优先。

8.3 DESCEND_200_STD
按你说的逆过程实现。

8.4 ASCEND_200_NO_FRONT_SICK
处理 6/8 那类特殊情况。

8.5 DESCEND_200_NO_FRONT_SICK
与上一条配对。

8.6 ASCEND_400_STD
在 200 稳定后再做。

8.7 DESCEND_400_STD
最后补。

阶段 9：把模板与转移表联通
目标：真正做到“查表 -> 对正 -> 跑模板”。

9.1 任务启动流程
当收到 from -> to：

查转移表
检查是否合法
读取 required_yaw_deg
进入 PREALIGN
9.2 对正完成后进入模板
对正成功 -> 进入模板
对正超时 -> fail
9.3 模板成功后更新状态
current_zone = to_zone
清除当前任务
上报 success
9.4 模板失败后上报
保留故障码
停车
收敛到安全状态
阶段 10：加入故障与恢复机制
目标：避免现场出事后完全失控。

10.1 定义故障码
建议至少有：

无效转移
yaw 对正超时
SICK 无效
光电超时
杆动作超时
模板执行超时
底盘异常
10.2 定义失败后的统一动作
例如：

停车
杆保持当前 or 收回安全位
退出 auto
通知上位机
10.3 定义中止接口
例如：

AutoCtrl_Abort()
阶段 11：日志与调试接口
目标：后续调试时能看懂它在干什么。

11.1 输出当前信息
建议可查询：

当前区块
目标区块
当前模板
当前 step
当前 yaw / 目标 yaw / 误差
当前故障码
11.2 保留事件记录
至少打印/缓存这些事件：

启动转移
对正成功/失败
step 切换
模板成功/失败
阶段 12：联调顺序
目标：别一开始就上全自动。

12.1 先单测数据层
区块表正确
转移表能查到
模板映射正确
12.2 单测 yaw 对正
零位偏置正确
角度误差归一化正确
对正控制不会反向
12.3 单测基础动作
杆动作到位
SICK 读数有效
光电触发正常
12.4 单测单个模板
顺序建议：

平移
上200
下200
特殊上200
上400
下400
12.5 最后再测“查表自动执行”
输入：

当前区块
目标区块
看是否能完整执行一次区块转移。

建议的实施顺序（最重要）
如果让别人按表写代码，我建议严格按这个顺序：

定义头文件和枚举
写区块表和转移表
写查询接口
写 auto_ctrl_t 和 API 壳子
写 yaw 零位与角度处理
写 PREALIGN 状态
写模板执行框架
先实现 FLAT_MOVE
实现 ASCEND_200_STD
实现 DESCEND_200_STD
再补特殊模板和 400 模板
最后加故障恢复和日志
给 codex 的执行原则
你后面可以直接让它按这几条写：

先搭框架，不要一上来填复杂动作细节
先数据驱动，不要写大量 if-else
先保证 from -> to 查表闭环跑通
先做一个模板通路成功，再扩展其余模板
所有模板都必须带超时和失败退出

补充执行约束：

- 不允许把模板分发函数放在 primitive 层
- 不允许把 step 执行逻辑放在 api 层
- 不允许把区块转移查询放在 template 层
- 可以在 `autoCtrlAPI` 内继续建立 `core/primitive/step/template/transition/api` 子目录
yaw 使用 raw_yaw - offset 后归一化到 [-180, 180]
下台阶模板按“保持朝向不变的逆过程”实现