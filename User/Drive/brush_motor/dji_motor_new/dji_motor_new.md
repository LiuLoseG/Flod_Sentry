由于电机反馈的数据的噪声和扰动很大，且先对电机缩放再滤波的效果，比先滤波再缩放的效果差很多，因此，此处采用两种数据来源，一种是电机输出轴的原始数据，直接参与下面的电机内部闭环控制程序中，另一种是电机转子(不算减速箱)的数据，便于后续的滤波等处理，获取更好的观测数据，进行闭环控制。

---

## Ozone / J-Link 观测 `rx_data` 出现 inf/NaN 的原因

### 根因：STM32H723 D-Cache 与调试探头的数据不一致

Cortex-M7 内核有 **D-Cache**（数据缓存，Write-Back 模式），这是 J-Link 读到脏数据的根本原因。

**数据流路径差异：**

```
CPU 写 rx_data (CAN 中断)              J-Link 读 rx_data (Ozone Timeline)
       │                                        │
       ▼                                        ▼
   D-Cache (L1, Write-Back)                AHB-AP 调试端口
       │                                        │
       │ (延迟写回, 非实时同步)                   │
       ▼                                        ▼
  物理 SRAM  ←──── 两条路径不同步 ────→   物理 SRAM
```

- **CPU** 写 `rx_data.now_omega` → 先写入 D-Cache line，不立即同步到物理 SRAM
- **J-Link** 通过 AHB-AP 读 → 直接读**物理 SRAM**，绕过 CPU 的 D-Cache
- 若 Cache line 尚未写回（Write-Back 延迟），J-Link 读到的是**旧值或上电随机值**

### 为什么表现为 inf

IEEE 754 浮点中 `inf` 的位模式为指数位全 1、尾数位全 0：

```
0x7F800000 = +∞
0xFF800000 = -∞
```

SRAM 上电残留的随机 32-bit 模式中，任何匹配 `0x7F8xxxxx` 的值都会被 Ozone 解释为 `inf`；匹配 `0x7FCxxxxx` 的则被解释为 `NaN`。这在统计学上每 ~512 个随机 32-bit 值就会出现一次。

### 触发场景

| 场景 | 机制 |
|---|---|
| **烧录后复位** | D-Cache 硬件清零，物理 SRAM 残留前次会话随机值，Ozone 采样 → inf/NaN |
| **运行时采样** | CPU 更新了 Cache 但未写回 SRAM，J-Link 读到旧的 Cache line 对应物理地址 → 脏数据 |
| **CAN 中断瞬间** | CPU 正在写 Cache line 的同时 J-Link 读同一物理地址 → 读到部分更新值 |

### 软件层面的防护措施

**已实施的防护（`data_process()` 中）：**

1. **`memset` 清零**（`init()` 末尾）：复位后立即将 `rx_data` 归零，消除 SRAM 残留随机值
2. **NaN/inf 运行时钳位**（`data_process()` 末尾）：每次 CAN 数据更新后检测并替换异常浮点值
   ```cpp
   if (rx_data.now_omega != rx_data.now_omega       // NaN 检测: NaN≠NaN 恒为真
       || rx_data.now_omega > 1e10f                  // +Inf 检测
       || rx_data.now_omega < -1e10f)                // -Inf 检测
       rx_data.now_omega = 0.0f;
   ```

这些措施确保 `rx_data` 本身不会存储 inf/NaN，但由于 Cache 延迟写回，**J-Link 在 Cache 写回前采样仍可能看到旧 SRAM 值**。

### 彻底解决方案

| 方案 | 效果 | 代价 |
|---|---|---|
| 关闭 D-Cache: `SCB_DisableDCache()` | 完全消除不一致 | 性能下降 20-50% |
| 将 `rx_data` 放入 DTCM | CPU 与调试器始终一致 | DTCM 仅 64KB，需合理分配 |
| MPU 配置为 Write-Through | CPU 每次写同步到 SRAM | 性能影响小，推荐 |