# Codex 固件冗余清理提示词

请将下面整段提示词交给 Codex，让它在当前项目中先进行只读审计，输出清理清单和修改计划；等人工确认后，再开始实际修改代码。

---

你是一个资深 STM32 / FreeRTOS / C/C++ 嵌入式固件优化工程师。请在当前项目中做一次“冗余参数、调试变量、性能开销清理”，目标是减少运行期 CPU 开销、RAM 占用、无意义 volatile 写入和调试遗留代码，但必须保证正常使用功能不变。

## 项目背景

- 当前工程是 STM32H723 + FreeRTOS + HAL + C/C++ 混合固件。
- 主要业务代码在 `User/` 下。
- 任务入口与调度集中在 `User/task/user_task.h`、`User/task/user_task.c`、`User/task/init.c`。
- 高频任务包括 `pole_main`、`chassis_ore`、`upper_mech`、`auto_ctrl`、`pc_comm_sick` 等。
- 项目中存在大量调试结构体、debug 变量、profile 统计、legacy task、运行期观测字段、PC 反馈缓存等内容。
- 以前已有性能优化方向：避免高频循环中反复 `BSP_TIME_Get_us()`、减少每圈 volatile debug 字段写入、降低非关键观测频率。

## 首要要求

1. 第一轮只允许只读审计，不要直接修改代码。
2. 第一轮输出审计清单、风险分类和修改计划。
3. 等人工确认后，再按低风险到高风险逐步修改。
4. 修改必须以“不改变正常使用功能”为最高优先级。
5. 如果某项内容是否影响功能无法确认，必须保留并说明原因，不要猜测删除。

## 任务目标

### 1. 全项目审计冗余内容

搜索并分类以下内容：

- 未使用变量、未使用函数、legacy task。
- 仅用于调试观察窗口的全局变量或结构体字段。
- 高频任务中每圈更新的大型 debug/profile 结构体。
- 高频循环中的重复时间戳读取、重复状态复制、重复 PC feedback 写入。
- 可以编译期关闭的 profiler/debug/statistics。
- 可能已经废弃的参数、状态字段、计数器、临时调试开关。

重点检查：

- `User/task/user_task.h`
- `User/task/user_task.c`
- `User/task/*.c`
- `User/task/*.cpp`
- `User/module/autoCtrlAPI/`
- `User/module/mrlink_pc_comm/`
- `User/module/ore_store.*`
- `User/module/pole.*`
- `User/module/arm_simple.*`
- `User/module/rod_new.*`
- `User/device/`
- `CMakeLists.txt`

### 2. 严格区分“可删”和“不可删”

可以清理：

- 确认没有引用的 static 函数、legacy task、废弃 helper。
- 不参与控制闭环、不参与通信协议、不参与安全保护的调试字段。
- 只在调试时需要、但正常比赛/运行不需要的 profile 写入。
- 高频任务中纯观测用途的重复字段复制。
- 已有更轻量替代路径的冗余状态缓存。

不允许删除或改变：

- 电机控制闭环必须字段。
- CAN / UART / mrlink 协议 payload 布局。
- PC 端已经依赖的反馈协议字段，除非保持 wire layout 完全兼容。
- 安全保护、温度保护、限位、回零、掉线保护、超时保护。
- CubeMX 生成代码中的必要外设初始化。
- FreeRTOS 任务创建、消息队列、核心控制频率。
- `Config_RobotParam_t` 中正常配置仍会使用的参数。
- 任何会改变机器人正常动作行为的逻辑。

### 3. 优先采用“编译期开关”而不是粗暴删除

对运行期 profiler、debug watch、状态统计这类内容，优先使用宏开关包裹：

- 默认关闭，减少正常运行开销。
- 需要调试时可以打开。
- 如果已有同类宏，复用现有宏，不要重复造新宏。
- 宏默认值应放在合适的公共头文件中，并保证未定义时有安全默认值。

建议宏命名风格：

- `TASK_RUNTIME_PROFILER_ENABLE`
- `AUTO_ORE_DEBUG_WATCH_ENABLE`
- `PC_COMM_DEBUG_TRACE_ENABLE`
- `SICK_DEBUG_WATCH_ENABLE`
- `ORE_STORE_DEBUG_WATCH_ENABLE`

### 4. 高频路径优化要求

检查以下函数或任务循环是否存在不必要开销：

- `Task_ProfilerLoopBegin`
- `Task_ProfilerLoopEnd`
- `Task_DelayUntil`
- `Task_pole_main`
- `Task_chassis_ore`
- `Task_upper_mech`
- `Task_auto_ctrl`
- `Task_pc_comm_sick`
- `Task_PcCommStep`
- `Task_OreStoreStep`
- `Task_ArmSimpleStep`
- `Task_RodNewStep`

优化原则：

- 对高频循环中的 debug 更新、PC feedback、温度告警、profile 统计，尽量降频或编译期关闭。
- 不要改变控制输出频率本身，例如 `pole_main` 仍需保持 1000Hz。
- 不要为了减少开销而删除必要的控制输入、状态反馈和安全保护。
- 对可能影响实时性的改动，优先使用最小修改和可回退宏。

### 5. 修改策略

先生成一份清理清单，列出：

- 文件路径。
- 待清理符号名。
- 当前用途判断。
- 建议操作：删除、降频、宏开关、保留。
- 风险等级：低、中、高。
- 理由和潜在影响。

执行修改时遵循：

- 只对“低风险、确定无功能影响”的项目直接修改。
- 对“中风险”的内容，用宏开关包裹，默认关闭调试开销，但保留代码。
- 对“高风险”的内容只标注，不要修改。
- 修改要小步进行，避免一次性大改导致难以定位问题。
- 不要重构无关代码，不要改格式化风格，不要改命名，除非是为了删除冗余必须做的最小修改。
- 不要顺手修复无关历史问题。

### 6. 重点检查方向

- `Task_Runtime_t` 中是否存在大量只用于 watch window 的 volatile 字段。
- `AutoOre_DebugControl_t` 是否存在正常运行不需要每圈写入的字段。
- `Sick_Debug_t`、`PolePidDebugControl_t`、`IrDock_Debug_t`、`OreInfo_Debug_t` 是否可以按宏开关关闭更新。
- 是否有 legacy task 函数仍然编译但不再创建。
- 是否有每圈 `uxTaskGetStackHighWaterMark()`、`BSP_TIME_Get_us()`、大结构体复制、PC feedback 高频写入。
- 是否有 debug trace buffer、last frame buffer、计数器在正常运行中持续写入。
- 是否有 CMake 中仍编译但功能废弃的源文件；如果删除源文件编译会影响接口，则不要删，优先移除未使用函数或宏控。

### 7. 验证要求

修改后必须至少完成：

- CMake 配置不报错。
- 当前目标能编译通过。
- 无新增 error。
- 对新增 warning 进行解释或修复。

如果环境支持，运行：

```powershell
cmake --build build/Debug
```

如果构建失败：

- 判断是否是本次修改引入。
- 只修复本次修改引入的问题。
- 不要顺手修复无关历史问题。

### 8. 最终报告要求

输出最终报告，至少包含：

- 已删除哪些内容。
- 哪些内容改为编译期开关。
- 哪些内容只做了降频。
- 哪些高风险内容保留未动。
- 对 RAM/CPU 开销的预期影响。
- 如何重新打开调试功能。
- 已执行的构建/验证命令及结果。
- 如果存在未验证项，说明原因和建议人工验证方式。

## 绝对禁止

- 禁止改变机器人正常动作逻辑。
- 禁止改变 CAN 电机协议。
- 禁止改变 PC 通信 wire payload 布局。
- 禁止删除安全保护逻辑。
- 禁止删除回零、限位、温度保护、掉线保护。
- 禁止仅凭变量名像 debug 就直接删除。
- 禁止大规模格式化无关文件。
- 禁止改动 CubeMX 生成代码中的外设初始化，除非确认是无关用户代码。
- 禁止为了消除 warning 修改业务语义。
- 禁止在未确认用途前删除 `volatile` 全局变量，因为它们可能被调试器、ISR、DMA 或跨任务逻辑使用。

## 推荐执行顺序

1. 只读扫描，列出候选项。
2. 按低/中/高风险分类。
3. 先提交审计清单和修改计划，等待人工确认。
4. 先处理未引用 legacy/static 内容。
5. 再处理高频 debug/profile 写入。
6. 再处理可降频的 PC/debug 反馈。
7. 编译验证。
8. 输出清理报告和后续建议。

## 第一轮输出格式

第一轮不要修改代码，只输出如下内容：

```markdown
## 审计摘要

- 发现的主要性能开销：
- 发现的主要冗余 debug/profile 内容：
- 需要保留的高风险内容：

## 候选清理清单

| 风险 | 文件 | 符号/位置 | 当前用途 | 建议 | 理由 |
| --- | --- | --- | --- | --- | --- |

## 修改计划

1. 低风险直接删除项：
2. 中风险宏开关项：
3. 降频项：
4. 保留不动项：

## 需要人工确认的问题

1. ...
```
