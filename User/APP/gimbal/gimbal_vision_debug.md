# 云台视觉联调说明文档

## 注意事项

- 注意云台的角速度和发送给视觉的加速度量（`gyrox_w`）。

## 云台 IMU 旋转矩阵说明

yaw 轴电机控制的旋转方向（绕世界 Z 轴），与 IMU 在 pitch ≠ 0 时的 Z 轴不再重合 → 需要旋转矩阵把 IMU 读数重新投影到世界 Z 轴。  
pitch 轴电机控制的旋转方向（绕 Y 轴），与 IMU 的 Y 轴始终重合 → 直接读取即可。

当 pitch ≠ 0° 时（比如云台仰起 60°），IMU 的 Z 轴也跟着倾斜了 60°，此时：

- `Get_Omega_Z()` 不再纯粹是 yaw 角速度 — 它混入了绕 IMU Z 轴转动的分量，但这条轴已经不垂直地面了。
- `Get_Omega_X()`（roll 方向）也会贡献 yaw 分量 — 当云台仰起后，绕 IMU X 轴的水平旋转实际上也包含了一部分绕世界 Z 轴的转动。

### 坐标变换公式

世界坐标系下的角速度与 IMU 体坐标系下的角速度关系为：

$$
\omega_{\text{world}} = R_y(-\text{pitch}) \cdot \omega_{\text{IMU}}
$$

展开后得到世界 Z 轴（真 yaw）角速度：

$$
\omega(\text{world\_Z}) = \cos(-\text{pitch}) \cdot \omega(\text{IMU\_Z}) \;+\; -\sin(-\text{pitch}) \cdot \omega(\text{IMU\_X})
$$

### 各项物理意义

| 项 | 物理意义 |
|:--|:---------|
| $\cos(-\text{pitch}) \cdot \omega_Z$ | 把 IMU 的 Z 轴读数投影到真 yaw 轴上。pitch 越大，投影系数 $\cos$ 越小 |
| $-\sin(-\text{pitch}) \cdot \omega_X$ | 把 IMU 的 X 轴（roll）读数中混入的 yaw 分量提取出来。pitch 越大，耦合越严重 |
