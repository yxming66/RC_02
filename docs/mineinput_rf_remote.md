# MineInput RF 遥控接收说明

`User/device/rf_remote.c` 接收 MineInput 的新版 E34 半双工协议：

- KFS：`AA 55 02 MSG_ID SIDE C1..C12 CRC8`，固定 18 字节。
- R2 重试：`AA 55 03 MSG_ID MODE CRC8`，固定 6 字节。
- ACK：`AA 55 82 MSG_ID STATUS CRC8`，固定 6 字节。
- CRC8 初值 `0xFF`、反射多项式 `0x8C`，复用 `User/component/crc8.c`。

接收器没有连接 E34 AUX。UART9 在收到完整帧后暂停接收，等待 15 ms，
使用 `PD15/UART9_TX` 阻塞发送 6 字节 ACK，发送完成后恢复 DMA/IDLE 接收。
上述等待和解析均在 `rc_main` 任务中完成，中断回调只记录长度并通知任务。

默认每帧只回一次 ACK。现场干扰较强时，可在编译选项中把
`RF_REMOTE_ACK_REPEAT_COUNT` 设为 `2` 或 `3`；第二、第三次发送前分别等待
25 ms 和 40 ms，重复内容完全相同。

业务层可重写两个弱回调：

```c
void RF_Remote_OnKfsFrame(uint8_t side, const uint8_t cells[12],
                          uint8_t msg_id);
void RF_Remote_OnRetryFrame(uint8_t mode, uint8_t msg_id);
```

业务暂时无法接收新命令时调用 `RF_Remote_SetBusy(true)`，驱动会回复
`STATUS=0x01` 且不触发业务回调；恢复后调用 `RF_Remote_SetBusy(false)`。

合法性检查与发送端一致：阵营、12 个位置状态、R1/R2/Fake 数量、R1
允许位置、Fake 禁止顶排，以及 R2 重试模式 `1..12`。相同命令和
`MSG_ID` 在 1 秒内重复到达时只重发 ACK，不重复触发业务。
