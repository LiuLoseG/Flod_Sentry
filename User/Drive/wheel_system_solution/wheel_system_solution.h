/**
 * @file wheel_system_solution.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-10-30
 * @version 1.44
 * @copyright HNAU-Robot (c) 2026
 * @note 正方向定义: 机器人坐标系 +X 向前, +Y 向左, +Z 向上; 角速度绕 Z 轴 CCW 为正
 * @note 1.44: 力控解算"命名与处理方式"对齐运动学逆解: ①轮端力方向先换成**电机域**目标角
 *            force_target = steer_encoder_direction·atan2f(force_y, force_x) + Enc_Zero_Angle,
 *            之后就近转位 / 单圈折算 / align_error / effective_error / 斜坡限幅与
 *            inverse_calculate_callback() 逐行同构, 删除几何域的 steer / target_angle /
 *            delta_angle / target_motor 四个局部量; ②局部量改名对齐逆解: force_magnitude→
 *            force_modulus (与 vel_modulus 同一命名口径), direction→optimize_direction,
 *            混控的 cos_effective→cos_angle_raw (+cos_angle); ③死区直接比力幅
 *            force_modulus <= force_deadband, 与下游 wheel_force_force_to_torque() 的
 *            force_modulus <= deadband 完全同形 (不再中转 force_modulus_sq / force_deadband_sq:
 *            4 次 VSQRT 在 1 kHz 下约 56 周期/拍可忽略, 且 arm_sqrt_f32 的判据是 in >= 0,
 *            NaN 输入得到 0 ⇒ 落进死区分支(锁舵向、不出力), 比"先比平方"的失效方向更安全);
 *            ④斜坡输出改用与逆解同为三元表达式的写法. 副作用: error_angle 在
 *            steer_encoder_direction = -1 时从几何角变为电机角 (恰好互为相反数, |error| 与
 *            cos³ 零差异), 与逆解同一个坐标域. 验证: 力矩/多圈舵向目标/余弦系数在
 *            force+mixed × dir±1 × 正方形/长方形 × 8 组开关组合下差 <= 2.2e-15;
 *            两项死区判据 4 万样本 × 4 档死区零分支差异.
 *      1.43: 删除 force_calculate_core() 内的局部量 plan_steer, 三处直接用
 *            (control_method != Wheel_Mixed_Control_Method) 判断: 它只读一个成员、无副作用,
 *            不值得为它多一个名字 (与"不抽中间变量"的既有风格一致).
 *      1.42: 余弦削减也内联进 force_calculate_core(): 删除独立的 force_cosine_optimize()
 *            (它只有 core 一处调用). effective_error 与 cosine 在循环内直接算并写入 status;
 *            死区条件由 |e| < db 取反为 |e| >= db, 以便 cosine 保持初值 1. 行为零差异.
 *      1.41: force_calculate_core() 改为**无参**函数, 与 forward_calculate_callback() 同形:
 *            读 target_accel / 写 status, 不再传递 (fx, fy, mz, plan_steer). 两条入口各自退化为
 *            一行转发, 加速度合法性校验与非法分支合并进 core (之前两条入口各写一份).
 *            plan_steer 改由 control_method 判定 ⇒ **新增契约**: init 首参须与实际调用的入口一致
 *            (Force↔force_/Mixed↔mixed_), 已在 control_method 与 core 的注释里注明.
 *      1.40: 舵轮力控合并为**单循环单函数** force_calculate_core(): 力分配 / 舵向规划 / 出力
 *            在同一循环内串行, 与运动学逆解"一行直写输出"同形. 删除 force_wheel_force_vector()
 *            与 force_project_torque(), 去掉 wheel_force_x[4]/y[4] 两个中间数组与两趟遍历;
 *            力分配公式与出力公式各只出现一次, 纯力控与混控通过 __plan_steer 区分.
 *            验证: 与三趟批处理版状态零差异、力矩差 2.2e-16 (浮点噪声).
 *      1.39: 舵轮 force_wheel_force_vector() 改为**循环 + 符号表现算**轮心坐标:
 *            展开写法等于把 Wheel_Sign_X/Y 表拄了第二份字面量, 改符号表时这里就静默失配
 *            (与此前硬编码 ±√2/2 是同一类错误). 循环版与展开版 float32 逐位零差异.
 *            至此逆解/正解/位移解算/力分配四处均只认 Lx/Ly/符号表这一份几何来源.
 *      1.38: 舵轮几何改为**半轴距/半轮距**形式, 支持正方形与长方形布局:
 *            新增 Wheel_Projection_X (Lx) / Wheel_Projection_Y (Ly) 与轮心符号表, 删除
 *            Wheel_Distance_To_Core[4] 与 Wheel_Unit_Cos/Sin[4]; 轮心坐标直接取 (±Lx, ±Ly),
 *            角速度相关公式只用 d² = Lx²+Ly² (= Wheel_D2), 全程不必开方.
 *            ⚠ **行为变化**: 原来 0.12 被当作径向距离 d 用 (等价于 Lx=Ly=0.0849), 现在按
 *            半轴距解释 (Lx=Ly=0.12), d 由 0.12 变为 0.16971 —— 纯偏航轮速大 41%、
 *            同样轮速下正解 vw 小 29%、偏航力减半. 这才是 1.37 里存疑的正确语义.
 *      1.37: 舵轮力分配切向改回查 Wheel_Unit_Cos/Sin 表 (1.34 硬编码 ±√2/2 是错的写法):
 *            表是几何的唯一真值源, 与逆解/正解/位移解算同源; 硬编码只在正方形布局下恰好一致,
 *            布局改矩形后会静默偏小 (0.15/0.10 −1.9%, 0.20/0.08 −8.1%) 且与逆解矛盾.
 *            另在常量注释里明确 d 是**径向距离**而非半轴距 (填错会让偏航轮速与里程计航向
 *            按互为倒数的比例同时失真, 正逆解自身仍自洽, 极难从数据上发现).
 *      1.36: 舵轮出力入口统一 (参考成熟实现的 Swerve_Force_Torque 结构):
 *            force_project_torque() 成为**纯力控与力速混控共用的唯一出力入口**,
 *            死区门限/模长/方向符号/力矩上限只在那里定义一份, 两条流水线不会各自漂移.
 *            __cosine 非空时直接采用 (纯力控已按角度算好并写进 status.cosine_optimize),
 *            为空时内部点积直达 (混控). 同时死区改平方比较, 死区轮免开方.
 *      1.35: 动力学 (力控) 解算重整: ①force_cosine_optimize() 内联死区判断, 调用方不再重写;
 *            ②force_project_torque() 改用点积直取 cos e (dir·F·û(θ)/|F|), 省掉 atan2f +
 *            arm_cos_f32 回算 + 两次角度归一化, 四轮合计约 800 周期/拍;
 *            ③死区判据换算到余弦域 cos e > cos(deadband), 循环外只算一次 cos(deadband),
 *            与纯力控的角度判据严格等价 (六档 deadband × 30 万样本验证偏差 ≤ 1e-14);
 *            ④力区判据统一为 <= 与 wheel_force_force_to_torque 扣除式同门限.
 *      1.34: 舵轮 force_wheel_force_vector() 改为与运动学逆解同形: 四轮逐行展开、带 φᵢ 轮位注释,
 *            切向直接写成 ±√2/2 (不再查 Wheel_Unit_Cos/Sin 表), Σdᵢ² 写在分母里不再写求和循环.
 *            与旧实现数值逐位一致 (0 误差).
 *      1.33: 舵轮 force_wheel_force_vector() 化简: 偏航力增益提到循环外, 只算一次除法
 *            (原先每轮一次除法); 删除 `if (Σdᵢ² > 1e-6)` 判据 —— 分母由常量表求和, 恒正,
 *            该判据恒真, 属假保护. 头文件常量区未新增任何内容.
 *      1.32: .cpp 目录结构改为"一个轮系一段": 力控实现 (speed_/force_/mixed_calculate_callback
 *            与力分配辅助函数) 从文件末尾的独立分区移入各自轮系段, 紧跟在该类的里程计/
 *            位姿接口之后; 删除三段 "xxx 力控" 分区标题, 文件只剩 4 个分区 (静态辅助 + 三系).
 *            纯排版调整, 代码_1 字节未改;
 *      1.31: 删除舵轮 init() 末尾重复清 status.wheel_torque 的循环: 舵轮的力矩在
 *            status 内, 上面那轮逐字段清零已覆盖; 全向轮/麦轮的 wheel_torque 是类成员
 *            (不在 status 内, 且它们用 status = {} 整体清), 才需要单独补一段, 不要在舵轮同抄;
 *      1.30: 底盘质量/转动惯量改为编译期常量 Chassis_Mass / Chassis_Inertia:
 *            从三系 init 形参中删除, 也不会再作为运行期成员. 这两项只由整车配重和
 *            几何决定, 改一次要重标定, 没必要留运行期入口; init 内只剩力矩上限与死区两项
 *            需校验 (wheel_force_config_valid 缩为两项).
 *      1.29: 舵轮逐轮力臂也改为编译期常量数组 Wheel_Distance_To_Core[4]:
 *            删除 distance_to_core[4] 成员与 set_distance_to_core(i, d), init 校验循环一并删除.
 *            至此三系几何量均只作编译期常量 (code 1.20 的"舵轮例外"作废);
 *      1.28: 加速度输入改为三轴 setter (set_robot_accel_x/y/w), 与 set_wheel_speed 同形:
 *            取代 update_robot_acceleration(struct) 与三个诊断用 getter. 单轴校验互不影响;
 *            target_accel 仍作内部存储, 零加速度 = 不出力;
 *      1.27: 删除三类的 force_enable: 它从未与 target_accel 分离写入, 而零加速度解出的
 *            力本就是零, 使能位不携带额外信息. 使能边界即"目标加速度是否为零"
 *            (speed_calculate_callback 入口清零), get_robot_accel_x/y/w 直接返回 target_accel;
 *      1.26: inverse_calculate_callback() 收进 protected (三系): 它只服务 speed_/mixed_
 *            的内部复用, 对外暴露会让人误以为可以绕过"关断力控输出"那一步. 对外仅保留
 *            speed_ / force_ / mixed_calculate_callback 三条流水线;
 *      1.25: 舵轮 speed_calculate_callback() 补上"关断力控输出" (此前只转发 inverse):
 *            从力控/混控切回速控后 force_enable 仍为 true, get_robot_accel_x/y/w() 会读回
 *            上一拍旧值. 至此三系 speed 语义一致 = 关断力控 + 逆解;
 *            inverse_calculate_callback() 定位为纯逆运动学 (不碰力控状态), 供混控复用;
 *      1.24: 删除 torque_feedforward_calculate_callback() 与 zero_force_calculate_callback():
 *            前馈段内联进 mixed_calculate_callback(), 关断力控输出内联进 speed_calculate_callback();
 *            对外只剩三条流水线;
 *      1.23: 三条流水线改名并取消内部分派:
 *            control_calculate_callback() -> speed_calculate_callback() (速控: 关断力控 + 逆解),
 *            mixed_control_calculate_callback() -> mixed_calculate_callback();
 *            不再由模块按 control_method 分派, 上层可直接调对应流水线;
 *            inverse_calculate_callback() 保留为纯逆运动学 (不改力矩);
 *      1.22: 删除各类的 get_control_method(): 控制方式由上层自己持有 (同一常量既传给
 *            init(), 也用于自己的分支判断), 模块不再回读;
 *      1.21: 删除各类的 set_force_config(): 力控四项只在 init() 里整份校验后写入,
 *            不再另开运行期改参旁路 (改参就重新 init, 避免出现"只改了力矩上限、
 *            质量仍是旧的"这类半新半旧的组合);
 *      1.20: 几何量 (轮径 / 轮心到底盘圆心距离 / 轮心 X,Y 投影) 一律只作编译期常量:
 *            删除 set_distance_to_core() / set_projection_x/y() 与对应的运行期成员,
 *            公式里直接用常量; 麦轮另加 Wheel_Sum_XY = a + b 常量替代旧的 sum_xy 缓存.
 *            (舵轮当时作为例外保留了逐轮力臂, 已于 1.29 统一改为常量数组);
 *      1.19: 全向轮公式去掉硬编码负号, 极性只由 wheel_encoder_direction 承担:
 *            逆解 w = σ·u/R (u = 轮心切向投影), 正解 ws = σ·轮速, 里程计 ds ×= σ, 力分配同翻,
 *            四处严格互逆. 全向轮 init 第二参默认值改为 -1 (本车电机正方向与切向相反),
 *            所以不传参仍与旧行为一致; 麦轮默认保持 +1;
 *      1.18: 全向轮/麦轮的 init 去掉轮距几何形参, 改为与舵轮对称的 __wheel_encoder_direction
 *            (四个轮向电机共用一个 ±1);
 *      1.17: 舵向死区 (速控与力控两条路径) 只取消出力削减 (k = 1), 不再把 optimize_angle
 *            改写成 now_angle. 改写等于永久放弃残余误差: 四轮各自锁在死区内距目标至多
 *            deadband 的任意位置, 摩擦/齿隙不同 -> 锁定位置不同 -> 静态指向不一致;
 *            前进时四轮本就在零位附近 (一致, 走直线), 横移时四轮要齐步转到 90° 却锁在不同位置
 *            -> 前后不对称 -> 持续净力矩 -> 横移走圆弧. 现在 error_angle 也保留真实残差不再置 0;
 *      1.16: 就近转位开关补齐到力控路径 (此前 nearest_optimize_enable 对纯力控静默失效);
 *      1.15: 三系 init 形参顺序统一为「控制参数 / 速控参数 / 力控限幅 / 优化开关 / 车体参数」;
 *      1.14: 力控配置并回 init(): 舵轮 init 接收全部十五项, 全向轮/麦轮 init 也加力控四项,
 *            三系同构; 删除 set_speed_config() (set_force_config 已于 1.21 删除);
 *      1.12: 删除 wheel_torque_t 及其 saturated[4]: 轮端力矩直接用各类内的
 *            float wheel_torque[4] 成员保存 (与 wheel_state_t 的 angle/speed 同范式),
 *            取消 get_wheel_torque_saturated(); 舵轮改为 status.wheel_torque[4];
 *      1.11: 力控配置不再用结构体: 四个字段 (chassis_mass / chassis_inertia / max_wheel_torque /
 *            force_deadband) 直接作为各轮系类的成员声明并带默认值,
 *            删除 wheel_force_config_t 与 get_force_config();
 *      1.10: 力控默认值下沉为字段默认初始化, 删除全局常量 Wheel_Force_Config_Default;
 *      1.9: 结构体重整: 导出类型按「基础量 → 配置 → 输入快照 → 输出」分区;
 *            力控输出由独立结构体改为各类内的 float wheel_torque[4] (不包结构体, 也不设饱和标志位:
 *            力矩是否被 max_wheel_torque 截断, 直接拿力矩与该上限比即可);
 *            三个类的成员统一按「配置 / 输入 / 内部状态 / 输出」四段排列;
 *      1.8: 删除加速度限幅 (max_linear_accel / max_angular_accel), 加速度由上层裁剪,
 *            本模块只保留输出端单轮力矩上限; 不限幅后"参与解算的加速度"恒等于下发值,
 *            所以力控输出结构不再存加速度副本,
 *            get_robot_accel_x/y/w() 改由 target_accel 派生 (未出力时为 0);
 *      1.7: 力控配置改为整份校验 (任一字段非法则整份拒绝), 新增加速度指令超时保护,
 *            力死区由硬截断改扣除式 (幅值连续);
 *      1.6: 力控输入为目标加速度 ax/ay/aw, 模块内按 F = m·a / M = J·α 换算 (质量/惯量为配置量);
 *      1.5: steer_wheel_status_t 合并力控输出 (速控与力控写同一组目标字段);
 *      1.4: 舵轮力控加入余弦修舵 (|cos e|^3), 与速控同口径;
 *      1.3: 三种轮系内建力控 (目标加速度 ax/ay/aw -> 轮端力矩 N·m, 舵轮另给舵向对齐目标)
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "user_lib.h"
#include "basic_math.h"
#include "arm_math.h"
#include <math.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 机器人速度 (机体系): 逆解输入 / 正解输出, vx/vy: m/s, vw: rad/s
 */
typedef struct
{
    float vx; // +X 向前, m/s
    float vy; // +Y 向左, m/s
    float vw; // 绕 +Z 逆时针 (CCW) 为正, rad/s
} wheel_robot_speed_t;

/**
 * @brief 机器人加速度 (机体系): 力控与力速混控的目标加速度, ax/ay: m/s^2, aw: rad/s^2
 * @note 由 set_robot_accel_x/y/w() 逐轴写入, 模块内按
 *        F = m·a、M = J·α 换算成等效力/力矩后再分配, 因此质量/惯量是各轮系类的
 *        编译期常量 (Chassis_Mass / Chassis_Inertia), 换配重改常量后重新编译即可.
 * @note  零加速度即不出力, 无单独使能位; 速控流水线入口会把三轴整体清零.
 * @note  **不限幅**: 加速度是上层规划量, 限到多大属整车能力, 由上层自行裁剪;
 *        本结构原样进入解算, 模块只负责把 NaN/Inf 逐轴拒掉 (被拒的那一轴保持原值).
 */
typedef struct
{
    float ax; // +X 向前, m/s^2
    float ay; // +Y 向左, m/s^2
    float aw; // 绕 +Z 逆时针 (CCW) 为正, rad/s^2
} wheel_robot_accel_t;

/**
 * @brief 机器人位姿 (世界系): x/y: m, yaw: rad
 * @note  yaw 折到 [-π, π); 描述的是**底盘中心**, 其它参考点需上层自行平移.
 */
typedef struct
{
    float x;   // 世界系 X 位置, m
    float y;   // 世界系 Y 位置, m
    float yaw; // 世界系航向角, rad (CCW 为正, [-π, π))
} wheel_robot_position_t;

/**
 * @brief 底盘控制方式 (init 首参选定, 决定上层每拍调哪条流水线)
 * @note  只有这三种, 不再让上层在每拍里自己挑 inverse_/force_/mixed_ 回调:
 *        控制方式决定"速度环是否参与", 属整机配置而非逐拍决策, 每拍切换会让舵向目标、
 *        余弦系数与斜坡基准在两套流水线之间来回踩, 状态不连续.
 * @note  与 chassis_mode_e(无力/跟随/小陀螺)不是一回事: 后者是行为模式, 前者是控制律结构.
 */
typedef enum
{
    Wheel_Speed_Control_Method = 0, // 纯速控: 逆解给轮速/舵向目标, 不做力矩前馈
    Wheel_Force_Control_Method,     // 纯力控: 加速度 -> 轮端力矩, 舵轮另给舵向对齐目标
    Wheel_Mixed_Control_Method,     // 力速混控: 逆解给轮速/舵向目标 + 加速度 -> 力矩前馈
} wheel_control_method_e;

/**
 * @brief 轮向电机测量状态: 多圈累计转角(编码器里程计输入) + 轮向角速度(正/逆解输入输出)
 * @note  angle 只喂编码器里程计, speed 只喂运动学; 轮子线速度 = speed × Wheel_Radius
 */
typedef struct
{
    float angle[4]; // 四个轮子多圈累计转角, rad ([0]=FL [1]=RL [2]=RR [3]=FR)
    float speed[4]; // 四个轮子轮向角速度, rad/s ([0]=FL [1]=RL [2]=RR [3]=FR)
} wheel_state_t;

/**
 * @brief 舵向电机测量状态: 多圈累计转角 + 舵向角速度, rad / rad/s (以 +X 为 0, CCW 为正)
 * @note  与 wheel_state_t 布局相同但语义不同, 刻意分型以防混用 (轮向角/舵向角);
 *        angle 同时喂正/逆解与编码器里程计, speed 供上层舵向速度环使用
 */
typedef struct
{
    float angle[4]; // 四个舵向累计转角, rad ([0]=FL [1]=RL [2]=RR [3]=FR)
    float speed[4]; // 四个舵向电机角速度, rad/s ([0]=FL [1]=RL [2]=RR [3]=FR)
} steer_state_t;

/**
 * @brief 舵轮解算状态
 */
typedef struct
{
    // 当前角度和角速度
    float now_angle[4];          // 舵向当前多圈角度反馈, rad
    float now_speed[4];          // 轮向当前角速度反馈, rad/s

    // 速控解算数据(不进行优化的控制参数)
    float solution_angle[4];     // 舵向几何解算角, rad (单圈几何角, 未加零位)
    float solution_speed[4];     // 轮向未优化目标速度, rad/s

    // 优化后的输出参数
    float optimize_angle[4];     // 舵向目标角, rad (多圈, 含零位; 同时是下一拍斜坡基准)
    float optimize_speed[4];     // 轮向目标速度, rad/s (已含余弦削减与方向符号; 纯力控为 0)

    float optimize_direction[4]; // 轮向方向, ±1 (就近转位的翻转状态, 兼作下一拍迟滞记忆)
    float cosine_optimize[4];    // 余弦削减系数, 无量纲 (1 = 已对齐不削减)
    float error_angle[4];        // 真实未对齐角, rad (相对所选方向分支, 0 表示已对齐)

    // 力控轮输出力矩
    float wheel_torque[4];       // 四轮轮端目标力矩, N·m ([0]=FL [1]=RL [2]=RR [3]=FR)
} steer_wheel_status_t;

/*-----------------------------------omni_wheel-----------------------------------*/

/**
 * @brief 全向轮运动学解算与编码器里程计
 * @note  status.speed 同时是正解输入与逆解输出, status.angle 为里程计用轮向多圈转角;
 *        单周期内应按 update_now_state() → forward_calculate_callback() → speed_calculate_callback() 顺序调用;
 *        若先解算后正解, 正解会把指令值当反馈, 实测速度恒等于目标速度.
 * @note  三种控制方式用法一致, 均为"初始化一次 → 每拍喂参数 → 调对应流水线":
 *          速控    : init(Wheel_Speed_Control_Method, ...) → update_robot_state(速度)  → speed_calculate_callback()
 *          力控    : init(Wheel_Force_Control_Method, ...) → set_robot_accel_x/y/w()   → force_calculate_callback()
 *          力速混控: init(Wheel_Mixed_Control_Method, ...) → 两者都喂               → mixed_calculate_callback()
 *        控制方式只在 init 里给一次, 之后不再区分; 零加速度时输出恒为 0, 不会残留上一拍力矩.
 */
class omni_wheel_class
{
public:

    void init(
              // 控制模式和控制参数
              const wheel_control_method_e &__control_method = Wheel_Speed_Control_Method,
              const float &__wheel_encoder_direction = -1.0f,
              const float &__control_period = 0.001f,
              // 力控死区和最大力矩限制
              const float &__max_wheel_torque = 4.9f,
              const float &__force_deadband = 0.1f);

    // 获取单轮角速度
    inline float get_wheel_speed(const uint8_t &__index) const;

    // 运动学正解(vx/vy/vw: m/s, rad/s)
    inline float get_robot_speed_vx() const;
    inline float get_robot_speed_vy() const;
    inline float get_robot_speed_vw() const;

    // 编码器里程计位姿(编码器累计) (x/y: m, yaw: rad)
    inline float get_position_x() const;
    inline float get_position_y() const;
    inline float get_position_yaw() const;

    // 速度里程计位姿(速度积分) (x/y: m, yaw: rad)
    inline float get_speed_position_x() const;
    inline float get_speed_position_y() const;
    inline float get_speed_position_yaw() const;

    // 轮端目标力矩, N·m
    inline float get_wheel_torque(const uint8_t &__index) const;

    // 轮向电机方向: +1 
    inline void set_wheel_encoder_direction(const float &__direction);
    // 目标加速度, m/s^2 / rad/s^2
    inline void set_robot_accel_x(const float &__accel_x);
    inline void set_robot_accel_y(const float &__accel_y);
    inline void set_robot_accel_w(const float &__accel_w);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);
    // 单轮角速度(正解输入)
    inline void set_wheel_speed(const uint8_t &__index, const float &__speed);

    // 更新底盘目标速度
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed);

    // 更新四轮当前测量状态
    inline void update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad = NAN);


    // 速控: 关断力控输出 + 逆解 -> 轮向目标角速度
    void speed_calculate_callback();
    // 力控: 目标加速度 -> 轮端力矩 (不使用速度目标)
    void force_calculate_callback();
    // 混控: 速度逆解 -> 轮向目标角速度 + 目标加速度 -> 轮端力矩前馈
    void mixed_calculate_callback();

    // 正运动学解算
    void forward_calculate_callback();

    // 编码器里程计(编码器航向参考)
    void encoder_odometry_callback();

    // 编码器里程计(IMU航向参考)
    void imu_odometry_callback();

    // 速度里程计
    void speed_odometry_callback();

    // 编码器里程计位姿清零
    void reset_position();
    // 编码器里程计位姿重定位
    void set_position(const float &__x, const float &__y, const float &__yaw);

    // 速度里程计位姿清零
    void reset_speed_position();
    // 速度里程计位姿重定位
    void set_speed_position(const float &__x, const float &__y, const float &__yaw);

protected:
    // 内部函数

    // 逆运动学解算 (纯几何解, 不碰力控状态): 只由 speed_/mixed_calculate_callback 内部调用
    void inverse_calculate_callback();
    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);
    // 力控: 等效力/力矩 -> 四轮期望轮端力, N; 满足 ΣF_i = F 且 Σ(r_i×F_i) = fw
    void force_wheel_force_vector(const float &__force_x, const float &__force_y, const float &__torque_z,
                                  float (&__wheel_force)[4]) const;

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0761f;
    // 轮心到底盘圆心的距离，m
    static constexpr float Wheel_Distance_To_Core = 0.25075f;
    // 底盘等效质量, kg (> 0); 换配重只改这一项
    static constexpr float Chassis_Mass = 10.0f;
    // 底盘绕 Z 轴等效转动惯量, kg·m^2 (> 0)
    static constexpr float Chassis_Inertia = 0.35f;

    // 编码器里程计位移累积死区, m: 单帧位移不足该值时只累积不积分, 累积到该值后一次吐出
    static constexpr float Odom_Linear_Deadband = 0.0001f;
    // 编码器里程计角度累积死区, rad: 同线位移, 抖动正负互抵不会触发
    static constexpr float Odom_Angle_Deadband = 0.001f;
    // 速度里程计线速度死区, m/s
    static constexpr float Speed_Odom_Linear_Deadband = 0.01f;
    // 速度里程计角速度死区, rad/s
    static constexpr float Speed_Odom_Angle_Deadband = 0.05f;

    // 轮向电机方向, ±1
    float wheel_encoder_direction = -1.0f;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;
    // 控制方式
    wheel_control_method_e control_method = Wheel_Speed_Control_Method;

    // 单轮轮端力矩上限, N·m (> 0); 
    float max_wheel_torque = 4.9f;
    // 单轮力死区(扣除式), N (0 = 关闭)
    float force_deadband = 0.1f;

    // 运动学: 底盘目标速度
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    // 力控: 目标加速度
    wheel_robot_accel_t target_accel = {0.0f, 0.0f, 0.0f};
    // 里程计: IMU 绝对航向角, rad
    float imu_yaw = 0.0f;
     // 内部变量

    // 上一帧累计轮转角, rad
    float last_angle_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 首帧标志, 上电首次调用建立里程计基准
    bool first_fb_flag = true;
    // 编码器里程计零帧抑制累积器: 未达死区的机体系位移暂存于此, 达阈值后一次积分
    float odom_residual_dx = 0.0f;
    float odom_residual_dy = 0.0f;
    float odom_residual_dyaw = 0.0f;
    // 读变量

    // angle 为里程计用轮向多圈转角, speed 为正/逆解共用
    wheel_state_t status = {};
    // 四轮轮端目标力矩
    float wheel_torque[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 正运动学结果 (机器人速度, vx/vy: m/s, vw: rad/s)
    wheel_robot_speed_t robot_speed = {0.0f, 0.0f, 0.0f};
    // 编码器里程计世界系位姿 (x,y: m; yaw: rad)
    wheel_robot_position_t encoder_pose = {0.0f, 0.0f, 0.0f};
    // 速度里程计世界系位姿 (x,y: m; yaw: rad), 与 encoder_pose 相互独立
    wheel_robot_position_t speed_pose = {0.0f, 0.0f, 0.0f};
};

/*-----------------------------------mecanum_wheel-----------------------------------*/

/**
 * @brief 麦轮运动学解算与编码器里程计
 * @note  外八辊子布局, 辊子方向 FL\ RL/ RR\ FR/ (小口→车内, 大口→车外)
 * @note  status.speed 同时是正解输入与逆解输出, status.angle 为里程计用轮向多圈转角;
 *        单周期内应按 update_now_state() → forward_calculate_callback() → speed_calculate_callback() 顺序调用;
 *        若先解算后正解, 正解会把指令值当反馈, 实测速度恒等于目标速度.
 * @note  三种控制方式用法一致, 均为"初始化一次 → 每拍喂参数 → 调对应流水线":
 *          速控    : init(Wheel_Speed_Control_Method, ...) → update_robot_state(速度)  → speed_calculate_callback()
 *          力控    : init(Wheel_Force_Control_Method, ...) → set_robot_accel_x/y/w() → force_calculate_callback()
 *          力速混控: init(Wheel_Mixed_Control_Method, ...) → 两者都喂               → mixed_calculate_callback()
 *        控制方式只在 init 里给一次, 之后不再区分; 零加速度时输出恒为 0, 不会残留上一拍力矩.
 */
class mecanum_wheel_class
{
public:

    void init(
              // 控制模式和控制参数
              const wheel_control_method_e &__control_method = Wheel_Speed_Control_Method,
              const float &__wheel_encoder_direction = 1.0f,
              const float &__control_period = 0.001f,
              // 力控死区和最大力矩限制
              const float &__max_wheel_torque = 4.9f,
              const float &__force_deadband = 0.1f);

    // 获取单轮角速度
    inline float get_wheel_speed(const uint8_t &__index) const;

    // 运动学正解(vx/vy/vw: m/s, rad/s)
    inline float get_robot_speed_vx() const;
    inline float get_robot_speed_vy() const;
    inline float get_robot_speed_vw() const;

    // 编码器里程计位姿(编码器累计) (x/y: m, yaw: rad)
    inline float get_position_x() const;
    inline float get_position_y() const;
    inline float get_position_yaw() const;

    // 速度里程计位姿(速度积分) (x/y: m, yaw: rad)
    inline float get_speed_position_x() const;
    inline float get_speed_position_y() const;
    inline float get_speed_position_yaw() const;

    // 轮端目标力矩, N·m
    inline float get_wheel_torque(const uint8_t &__index) const;

    // 轮向电机方向 +1 
    inline void set_wheel_encoder_direction(const float &__direction);
    // 目标加速度, m/s^2 / rad/s^2 (零 = 不出力; 速控流水线入口会整体清零)
    inline void set_robot_accel_x(const float &__accel_x);
    inline void set_robot_accel_y(const float &__accel_y);
    inline void set_robot_accel_w(const float &__accel_w);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);
    // 单轮角速度(正解输入)
    inline void set_wheel_speed(const uint8_t &__index, const float &__speed);

    // 更新底盘目标速度
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed);

    // 四轮当前测量状态
    inline void update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad = NAN);


    // 速控: 关断力控输出 + 逆解 -> 轮向目标角速度
    void speed_calculate_callback();
    // 力控: 目标加速度 -> 轮端力矩 (不使用速度目标)
    void force_calculate_callback();
    // 混控: 速度逆解 -> 轮向目标角速度 + 目标加速度 -> 轮端力矩前馈
    void mixed_calculate_callback();

    // 正运动学解算
    void forward_calculate_callback();

    // 编码器里程计(编码器航向参考)
    void encoder_odometry_callback();

    // 编码器里程计(IMU航向参考)
    void imu_odometry_callback();

    // 速度里程计
    void speed_odometry_callback();

    // 编码器里程计位姿清零
    void reset_position();
    // 编码器里程计位姿重定位
    void set_position(const float &__x, const float &__y, const float &__yaw);

    // 速度里程计位姿清零
    void reset_speed_position();
    // 速度里程计位姿重定位
    void set_speed_position(const float &__x, const float &__y, const float &__yaw);

protected:
    // 内部函数

    // 逆运动学解算 (纯几何解, 不碰力控状态)
    void inverse_calculate_callback();
    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);
    // 力控: 等效力/力矩 -> 四轮期望轮端力, N; 满足 ΣF_i = F 且 Σ(r_i×F_i) = fw
    void force_wheel_force_vector(const float &__force_x, const float &__force_y, const float &__torque_z, float (&__wheel_force)[4]) const;

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0485f;
    // 轮心在机体系 X/Y 方向的投影，m (半轴距 a / 半轮距 b)
    static constexpr float Wheel_Projection_X = 0.12f;
    static constexpr float Wheel_Projection_Y = 0.11f;
    // 投影距离和 (a+b), m
    static constexpr float Wheel_Sum_XY = Wheel_Projection_X + Wheel_Projection_Y;
    // 底盘等效质量, kg (> 0); 换配重只改这一项
    static constexpr float Chassis_Mass = 10.0f;
    // 底盘绕 Z 轴等效转动惯量, kg·m^2 (> 0)
    static constexpr float Chassis_Inertia = 0.35f;

    // 编码器里程计位移累积死区, m: 单帧位移不足该值时只累积不积分, 累积到该值后一次吐出
    static constexpr float Odom_Linear_Deadband = 0.0001f;
    // 编码器里程计角度累积死区, rad: 同线位移, 抖动正负互抵不会触发
    static constexpr float Odom_Angle_Deadband = 0.001f;
    // 速度里程计线速度死区, m/s
    static constexpr float Speed_Odom_Linear_Deadband = 0.01f;
    // 速度里程计角速度死区, rad/s
    static constexpr float Speed_Odom_Angle_Deadband = 0.05f;

    // 轮向电机方向 ±1
    float wheel_encoder_direction = 1.0f;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;
    // 控制方式
    wheel_control_method_e control_method = Wheel_Speed_Control_Method;

    // 单轮轮端力矩上限, N·m (> 0); 力控直接对电机出力, 输出端必须留住这一层防过流
    float max_wheel_torque = 4.9f;
    // 单轮力死区(扣除式), N (0 = 关闭)
    float force_deadband = 0.1f;

    // 运动学: 底盘目标速度
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    // 力控: 目标加速度
    wheel_robot_accel_t target_accel = {0.0f, 0.0f, 0.0f};
    // 里程计: IMU 绝对航向角, rad
    float imu_yaw = 0.0f;


    // 上一帧累计轮转角, rad
    float last_angle_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 首帧标志, 上电首次调用建立里程计基准
    bool first_fb_flag = true;
    // 编码器里程计零帧抑制累积器: 未达死区的机体系位移暂存于此, 达阈值后一次积分
    float odom_residual_dx = 0.0f;
    float odom_residual_dy = 0.0f;
    float odom_residual_dyaw = 0.0f;

    // 运动学: angle 为里程计用轮向多圈转角, speed 为正/逆解共用
    wheel_state_t status = {};
    // 力控: 四轮轮端目标力矩, N·m ([0]=FL [1]=RL [2]=RR [3]=FR)
    float wheel_torque[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 正运动学结果 (机器人速度, vx/vy: m/s, vw: rad/s)
    wheel_robot_speed_t robot_speed = {0.0f, 0.0f, 0.0f};
    // 编码器里程计世界系位姿 (x,y: m; yaw: rad)
    wheel_robot_position_t encoder_pose = {0.0f, 0.0f, 0.0f};
    // 速度里程计世界系位姿 (x,y: m; yaw: rad), 与 encoder_pose 相互独立
    wheel_robot_position_t speed_pose = {0.0f, 0.0f, 0.0f};
};

/*-----------------------------------steer_wheel-----------------------------------*/

/**
 * @brief 舵轮运动学解算、余弦优化与编码器里程计
 * @note  初始化只需调一次 init(): 十五个形参全部带默认值, 按
 *        控制模式和控制参数 → 速控参数 → 力控死区和最大力矩限制 → 优化开关 → 车体质量和转动惯量
 *        分组排列并加注释分隔.
 *        速控参数逐项校验(单项非法只跳过该项), 力控四项整份校验(任一项非法则四项全不改).
 * @note  **三个优化开关是三系共的一份舵向能力配置, 速控与力控同时受控**: 关闭就近转位后,
 *        速控只转解算角本体、力控只转力方向本体 (dir 恒 +1); 关闭余弦削减后两条路径都不再按
 *        cos³e 降低出力; 关闭斜坡限幅后两条路径的舵向目标都直接跟随. 不会出现"只关掉一边".
 * @note  三种控制方式用法一致, 均为"初始化一次 → 每拍喂参数 → 调对应流水线":
 *          速控    : init(Wheel_Speed_Control_Method, ...)  → update_robot_state(速度, 模式) → speed_calculate_callback()
 *          力控    : init(Wheel_Force_Control_Method, ...)  → set_robot_accel_x/y/w()       → force_calculate_callback()
 *          力速混控: init(Wheel_Mixed_Control_Method, ...)  → 两者都喂                    → mixed_calculate_callback()
 *        控制方式只在 init 里给一次; 两条流水线写同一组目标字段, 因此舵向斜坡基准天然连续.
 */
class steer_wheel_class
{
public:

    void init(
              // 控制模式和控制参数
              const wheel_control_method_e &__control_method = Wheel_Speed_Control_Method,
              const float &__steer_encoder_direction = 1.0f,
              const float &__control_period = 0.001f,
              // 速控参数
              const float &__target_max_step = 0.12f,
              const float &__max_wheel_speed = 25.0f,
              const float &__min_limit = 0.1f,
              const float &__steer_angle_deadband = 0.003f,
              const float &__flip_hysteresis = 0.1f,
              // 力控死区和最大力矩限制
              const float &__max_wheel_torque = 4.9f,
              const float &__force_deadband = 0.1f,
              // 优化开关
              const bool &__cosine_optimize_enable = true,
              const bool &__nearest_optimize_enable = true,
              const bool &__target_step_limit_enable = true);

    // 获取逆解输出量(运动学)
    inline float get_solution_angle(const uint8_t &__index) const;
    inline float get_solution_speed(const uint8_t &__index) const;

    inline float get_optimize_angle(const uint8_t &__index) const;
    inline float get_optimize_speed(const uint8_t &__index) const;

    // 轮端目标力矩(动力学), N·m
    inline float get_wheel_torque(const uint8_t &__index) const;

    inline float get_optimize_direction(const uint8_t &__index) const;
    inline float get_error_angle(const uint8_t &__index) const;
    inline float get_cosine_optimize(const uint8_t &__index) const;

    // 运动学正解(vx/vy/vw: m/s, rad/s)
    inline float get_robot_speed_vx() const;
    inline float get_robot_speed_vy() const;
    inline float get_robot_speed_vw() const;

    // 编码器里程计位姿(编码器累计) (x/y: m, yaw: rad)
    inline float get_position_x() const;
    inline float get_position_y() const;
    inline float get_position_yaw() const;

    // 速度里程计位姿(速度积分) (x/y: m, yaw: rad)
    inline float get_speed_position_x() const;
    inline float get_speed_position_y() const;
    inline float get_speed_position_yaw() const;

    // 舵向目标单周期最大变化量, rad
    inline void set_target_max_step(const float &__target_max_step);
    // 目标加速度, m/s^2 / rad/s^2 
    inline void set_robot_accel_x(const float &__accel_x);
    inline void set_robot_accel_y(const float &__accel_y);
    inline void set_robot_accel_w(const float &__accel_w);
    // 轮向电机最大角速度, rad/s
    inline void set_max_wheel_speed(const float &__max_wheel_speed);
    // 静止判定阈值, m/s 与 rad/s (三轴均低于此值视为零指令)
    inline void set_min_limit(const float &__min_limit);
    // 非小陀螺模式舵向误差死区, rad (小于该值不再修舵)
    inline void set_steer_angle_deadband(const float &__steer_angle_deadband);
    // 就近转位翻转迟滞, rad (须在 [0, π/2) 内, 0 表示无迟滞)
    inline void set_flip_hysteresis(const float &__flip_hysteresis);
    // 余弦优化开关 (速控与力控共用: 关闭时两条流水线同时不再按 cos³e 削减出力)
    inline void set_cosine_optimize_enable(const bool &__enable);
    // 就近转位优化开关 (速控与力控共用: 关闭时两条流水线都只转到解算/力方向本体, dir 恒 +1)
    inline void set_nearest_optimize_enable(const bool &__enable);
    // 舵向目标斜坡限幅开关 (速控与力控共用)
    inline void set_target_step_limit_enable(const bool &__enable);
    // 舵向编码器方向 +1 
    inline void set_steer_encoder_direction(const float &__direction);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);

    // 更新底盘目标速度和控制模式
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed, const uint8_t &__control_mode);
    // 各轮状态更新
    inline void update_now_state(const steer_state_t &__now_steer_state, const wheel_state_t &__now_wheel_state, const float &__imu_yaw_rad = NAN);


    // 速控: 关断力控输出 + 逆解 -> 舵向/轮向目标
    void speed_calculate_callback();
    // 力控: 目标加速度 -> 舵向对齐目标 + 轮端力矩 (不使用速度目标)
    void force_calculate_callback();
    // 混控: 速度逆解 -> 舵向/轮向目标, 再按当前实际轮向叠加力矩前馈
    void mixed_calculate_callback();

    // 正运动学解算
    void forward_calculate_callback();

    // 编码器里程计(编码器航向参考)
    void encoder_odometry_callback();

    // 编码器里程计(IMU航向参考)
    void imu_odometry_callback();

    // 速度里程计
    void speed_odometry_callback();

    // 编码器里程计位姿清零
    void reset_position();
    // 编码器里程计位姿重定位
    void set_position(const float &__x, const float &__y, const float &__yaw);

    // 速度里程计位姿清零
    void reset_speed_position();
    // 速度里程计位姿重定位
    void set_speed_position(const float &__x, const float &__y, const float &__yaw);

protected:
    // 内部函数

    // 逆运动学解算 (纯几何解, 不碰力控状态)
    void inverse_calculate_callback();
    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);
    // 由各轮线位移与舵向角解算机体系位移; 任一通道非法时输出清零并返回 false
    bool calc_body_displacement(const float *__ds, float &__dx, float &__dy, float &__dyaw) const;
    // 力控核心 (无参): 读 target_accel, 写 status.wheel_torque; 力分配 / 舵向规划 / 余弦削减 / 出力
    // 在同一循环内串行完成. 与 forward_calculate_callback() 同形, 纯力控与混控共用;
    // 由 control_method 决定是否由力方向规划舵向
    void force_calculate_core();

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0485f;

    // ---- 轮心几何: 正方形 / 长方形布局通用, 换构型只改下面两项 ----
    // 轮心在机体系 X/Y 方向的投影, m: 前后半轴距 Lx / 左右半轮距 Ly
    // 四轮对称布置: [0]=FL(+Lx,+Ly) [1]=RL(-Lx,+Ly) [2]=RR(-Lx,-Ly) [3]=FR(+Lx,-Ly)
    // 注意是**半**轴距 / 半轮距 (轮心到中心线的距离), 不是整轴距 / 整轮距
    static constexpr float Wheel_Projection_X = 0.12f;
    static constexpr float Wheel_Projection_Y = 0.12f;
    // 轮心坐标符号 (乘上 Lx / Ly 即得轮心 (x_i, y_i)), 供逆解/正解/位移解算的循环使用
    static constexpr float Wheel_Sign_X[4] = { 1.0f, -1.0f, -1.0f,  1.0f};
    static constexpr float Wheel_Sign_Y[4] = { 1.0f,  1.0f, -1.0f, -1.0f};
    // 轮心到底盘中心的径向距离平方 d² = Lx² + Ly², m²; 四轮相同 (对角轮心落在同一圆周上)
    // 角速度相关公式只用到 d², 因此全程不必开方: 逆解/力分配用轮心坐标 (x_i, y_i), 正解用 (x_i, y_i)/d²
    static constexpr float Wheel_D2 = Wheel_Projection_X * Wheel_Projection_X
                                    + Wheel_Projection_Y * Wheel_Projection_Y;

    // 编码器里程计位移累积死区, m: 单帧位移不足该值时只累积不积分, 累积到该值后一次吐出
    static constexpr float Odom_Linear_Deadband = 0.0001f;
    // 编码器里程计角度累积死区, rad: 同线位移, 抖动正负互抵不会触发
    static constexpr float Odom_Angle_Deadband = 0.001f;
    // 速度里程计线速度死区, m/s
    static constexpr float Speed_Odom_Linear_Deadband = 0.01f;
    // 速度里程计角速度死区, rad/s
    static constexpr float Speed_Odom_Angle_Deadband = 0.05f;

    // 编码器零位角, rad: 编码器读数为 0 时轮子实际指向相对 +X 的偏差
    static constexpr float Enc_Zero_Angle[4] = {-1.07455f, -0.02924f, 1.01923f, -3.11878f};
    // 底盘等效质量, kg (> 0);
    static constexpr float Chassis_Mass = 10.0f;
    // 底盘绕 Z 轴等效转动惯量, kg·m^2 (> 0)
    static constexpr float Chassis_Inertia = 0.35f;

    // 舵向目标单周期最大变化量(斜坡斜率), rad/拍; 仅在 target_step_limit_enable 为 true 时生效
    float target_max_step = 0.12f;
    // 轮向电机最大角速度, rad/s (超限时四轮等比例缩放)
    float max_wheel_speed = 25.0f;
    // 静止判定阈值, m/s 与 rad/s (三轴指令均不超过该值视为零指令)
    float min_limit = 0.1f;
    // 非小陀螺模式舵向误差死区, rad (小于该值不再修舵, 防抖)
    float steer_angle_deadband = 0.003f;
    // 就近转位翻转迟滞, rad ∈ [0, π/2): 已反转时需 |误差| 小于 90°-迟滞 才转回, 正向时需大于 90°+迟滞 才反转
    float flip_hysteresis = 0.1f;
    // 舵向电机方向(与解算正方向一致为正±1)
    float steer_encoder_direction = 1.0f;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;
    // 优化开关: 余弦削减 / 就近转位 / 舵向斜坡限幅
    bool cosine_optimize_enable = true;
    bool nearest_optimize_enable = true;
    bool target_step_limit_enable = true;
    // 控制方式
    wheel_control_method_e control_method = Wheel_Speed_Control_Method;

    // 单轮轮端力矩上限, N·m (> 0); 力控直接对电机出力, 输出端必须留住这一层防过流
    float max_wheel_torque = 4.9f;
    // 力矩合力死区
    float force_deadband = 0.1f;

    // 运动学: 底盘目标速度 + 底盘行为模式
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    uint8_t control_mode = 0;
    // 力控: 目标加速度
    wheel_robot_accel_t target_accel = {0.0f, 0.0f, 0.0f};
    // 里程计: 舵向累计转角 + 轮向累计转角 + IMU 航向
    float wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float steer_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float imu_yaw = 0.0f;

    // 上一帧累计轮转角, rad
    float last_wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 首帧标志, 上电首次调用建立里程计基准
    bool first_fb_flag = true;
    // 舵向斜坡基准有效性: init()/zero_force() 后为 false, 首拍直接从当前反馈接管, 避免从 0 跳变
    // 速控与力控共用: 两条路径写同一份 optimize_angle, 基准天然连续
    bool optimize_angle_valid[4] = {false, false, false, false};
    // 编码器里程计零帧抑制累积器: 未达死区的机体系位移暂存于此, 达阈值后一次积分
    float odom_residual_dx = 0.0f;
    float odom_residual_dy = 0.0f;
    float odom_residual_dyaw = 0.0f;


    steer_wheel_status_t status = {};

    // 正运动学 (机器人速度, vx/vy: m/s, vw: rad/s)
    wheel_robot_speed_t robot_speed = {0.0f, 0.0f, 0.0f};
    // 编码器里程计世界系位姿 (x,y: m; yaw: rad)
    wheel_robot_position_t encoder_pose = {0.0f, 0.0f, 0.0f};
    // 速度里程计世界系位姿 (x,y: m; yaw: rad), 与 encoder_pose 相互独立
    wheel_robot_position_t speed_pose = {0.0f, 0.0f, 0.0f};
};

/*-----------------------------------omni_wheel inline-----------------------------------*/

inline float omni_wheel_class::get_wheel_torque(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return (0.0f);
    }
    return (wheel_torque[__index]);
}

inline void omni_wheel_class::set_robot_accel_x(const float &__accel_x)
{
    // 单轴校验: 非法值跳过该轴, 不影响其他两轴
    if (!isfinite(__accel_x))
    {
        return;
    }

    target_accel.ax = __accel_x;
}

inline void omni_wheel_class::set_robot_accel_y(const float &__accel_y)
{
    if (!isfinite(__accel_y))
    {
        return;
    }

    target_accel.ay = __accel_y;
}

inline void omni_wheel_class::set_robot_accel_w(const float &__accel_w)
{
    if (!isfinite(__accel_w))
    {
        return;
    }

    target_accel.aw = __accel_w;
}

inline void omni_wheel_class::set_wheel_encoder_direction(const float &__direction)
{
    // 只接受 ±1: 其它值既不是"同向"也不是"反向", 直接忽略避免引入莫名的比例缩放
    if (!isfinite(__direction) || __direction == 0.0f)
    {
        return;
    }

    wheel_encoder_direction = (__direction > 0.0f) ? 1.0f : -1.0f;
}

inline void omni_wheel_class::set_control_period(const float &__control_period)
{
    // 速度里程计以该周期做积分, 必须是有限正值
    if (!isfinite(__control_period) || __control_period <= 0.0f)
    {
        return;
    }
    control_period = __control_period;
}

inline void omni_wheel_class::update_robot_state(const wheel_robot_speed_t &__target_speed)
{
    if (!isfinite(__target_speed.vx) || !isfinite(__target_speed.vy) || !isfinite(__target_speed.vw))
    {
        return;
    }
    target_speed = __target_speed;
}

inline void omni_wheel_class::update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad)
{
    // 轮速逐轮独立校验: 单个通道异常只跳过该通道
    for (uint8_t i = 0; i < 4; i++)
    {
        if (isfinite(__now_state.speed[i]))
        {
            status.speed[i] = __now_state.speed[i];
        }
    }

    // 四轮累计角必须同帧有效才提交: 里程计把四轮位移合成三轴, 只替换坏通道等于用旧值冒充"该轮没动",
    // 会算出虚假平移; 整帧丢弃且不推进基准, 坏通道恢复后位移一次补齐, 一条都不丢
    bool frame_valid = true;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(__now_state.angle[i]))
        {
            frame_valid = false;
        }
    }

    if (frame_valid)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            status.angle[i] = __now_state.angle[i];
        }
    }

    // IMU 航向与轮角相互独立: 任一通道异常不影响另一路
    if (isfinite(__imu_yaw_rad))
    {
        imu_yaw = __imu_yaw_rad;
    }
}

inline float omni_wheel_class::get_wheel_speed(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.speed[__index]);
}

inline void omni_wheel_class::set_wheel_speed(const uint8_t &__index, const float &__speed)
{
    if (__index >= 4 || !isfinite(__speed))
    {
        return;
    }
    status.speed[__index] = __speed;
}

inline float omni_wheel_class::get_robot_speed_vx() const
{
    return (robot_speed.vx);
}

inline float omni_wheel_class::get_robot_speed_vy() const
{
    return (robot_speed.vy);
}

inline float omni_wheel_class::get_robot_speed_vw() const
{
    return (robot_speed.vw);
}

inline float omni_wheel_class::get_position_x() const
{
    return (encoder_pose.x);
}

inline float omni_wheel_class::get_position_y() const
{
    return (encoder_pose.y);
}

inline float omni_wheel_class::get_position_yaw() const
{
    return (encoder_pose.yaw);
}

inline float omni_wheel_class::get_speed_position_x() const
{
    return (speed_pose.x);
}

inline float omni_wheel_class::get_speed_position_y() const
{
    return (speed_pose.y);
}

inline float omni_wheel_class::get_speed_position_yaw() const
{
    return (speed_pose.yaw);
}

/*-----------------------------------mecanum_wheel inline-----------------------------------*/

inline float mecanum_wheel_class::get_wheel_torque(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return (0.0f);
    }
    return (wheel_torque[__index]);
}

inline void mecanum_wheel_class::set_robot_accel_x(const float &__accel_x)
{
    // 单轴校验: 非法值跳过该轴, 不影响其他两轴
    if (!isfinite(__accel_x))
    {
        return;
    }

    target_accel.ax = __accel_x;
}

inline void mecanum_wheel_class::set_robot_accel_y(const float &__accel_y)
{
    if (!isfinite(__accel_y))
    {
        return;
    }

    target_accel.ay = __accel_y;
}

inline void mecanum_wheel_class::set_robot_accel_w(const float &__accel_w)
{
    if (!isfinite(__accel_w))
    {
        return;
    }

    target_accel.aw = __accel_w;
}

inline void mecanum_wheel_class::set_wheel_encoder_direction(const float &__direction)
{
    // 只接受 ±1: 其它值既不是"同向"也不是"反向", 直接忽略避免引入莫名的比例缩放
    if (!isfinite(__direction) || __direction == 0.0f)
    {
        return;
    }

    wheel_encoder_direction = (__direction > 0.0f) ? 1.0f : -1.0f;
}

inline void mecanum_wheel_class::set_control_period(const float &__control_period)
{
    // 速度里程计以该周期做积分, 必须是有限正值
    if (!isfinite(__control_period) || __control_period <= 0.0f)
    {
        return;
    }
    control_period = __control_period;
}

inline void mecanum_wheel_class::update_robot_state(const wheel_robot_speed_t &__target_speed)
{
    if (!isfinite(__target_speed.vx) || !isfinite(__target_speed.vy) || !isfinite(__target_speed.vw))
    {
        return;
    }
    target_speed = __target_speed;
}

inline void mecanum_wheel_class::update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad)
{
    // 轮速逐轮独立校验: 单个通道异常只跳过该通道
    for (uint8_t i = 0; i < 4; i++)
    {
        if (isfinite(__now_state.speed[i]))
        {
            status.speed[i] = __now_state.speed[i];
        }
    }

    // 四轮累计角必须同帧有效才提交: 里程计把四轮位移合成三轴, 只替换坏通道等于用旧值冒充"该轮没动",
    // 会算出虚假平移; 整帧丢弃且不推进基准, 坏通道恢复后位移一次补齐, 一条都不丢
    bool frame_valid = true;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(__now_state.angle[i]))
        {
            frame_valid = false;
        }
    }

    if (frame_valid)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            status.angle[i] = __now_state.angle[i];
        }
    }

    // IMU 航向与轮角相互独立: 任一通道异常不影响另一路
    if (isfinite(__imu_yaw_rad))
    {
        imu_yaw = __imu_yaw_rad;
    }
}

inline float mecanum_wheel_class::get_wheel_speed(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.speed[__index]);
}

inline void mecanum_wheel_class::set_wheel_speed(const uint8_t &__index, const float &__speed)
{
    if (__index >= 4 || !isfinite(__speed))
    {
        return;
    }
    status.speed[__index] = __speed;
}

inline float mecanum_wheel_class::get_robot_speed_vx() const
{
    return (robot_speed.vx);
}

inline float mecanum_wheel_class::get_robot_speed_vy() const
{
    return (robot_speed.vy);
}

inline float mecanum_wheel_class::get_robot_speed_vw() const
{
    return (robot_speed.vw);
}

inline float mecanum_wheel_class::get_position_x() const
{
    return (encoder_pose.x);
}

inline float mecanum_wheel_class::get_position_y() const
{
    return (encoder_pose.y);
}

inline float mecanum_wheel_class::get_position_yaw() const
{
    return (encoder_pose.yaw);
}

inline float mecanum_wheel_class::get_speed_position_x() const
{
    return (speed_pose.x);
}

inline float mecanum_wheel_class::get_speed_position_y() const
{
    return (speed_pose.y);
}

inline float mecanum_wheel_class::get_speed_position_yaw() const
{
    return (speed_pose.yaw);
}

/*-----------------------------------steer_wheel inline-----------------------------------*/

inline float steer_wheel_class::get_wheel_torque(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.wheel_torque[__index]);
}

inline void steer_wheel_class::set_robot_accel_x(const float &__accel_x)
{
    // 单轴校验: 非法值跳过该轴, 不影响其他两轴
    if (!isfinite(__accel_x))
    {
        return;
    }

    target_accel.ax = __accel_x;
}

inline void steer_wheel_class::set_robot_accel_y(const float &__accel_y)
{
    if (!isfinite(__accel_y))
    {
        return;
    }

    target_accel.ay = __accel_y;
}

inline void steer_wheel_class::set_robot_accel_w(const float &__accel_w)
{
    if (!isfinite(__accel_w))
    {
        return;
    }

    target_accel.aw = __accel_w;
}

inline float steer_wheel_class::get_solution_angle(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.solution_angle[__index]);
}

inline float steer_wheel_class::get_solution_speed(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.solution_speed[__index]);
}

inline float steer_wheel_class::get_optimize_angle(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.optimize_angle[__index]);
}

inline float steer_wheel_class::get_optimize_speed(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.optimize_speed[__index]);
}

inline float steer_wheel_class::get_optimize_direction(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.optimize_direction[__index]);
}

inline float steer_wheel_class::get_error_angle(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.error_angle[__index]);
}

inline float steer_wheel_class::get_cosine_optimize(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.cosine_optimize[__index]);
}

inline float steer_wheel_class::get_robot_speed_vx() const
{
    return (robot_speed.vx);
}

inline float steer_wheel_class::get_robot_speed_vy() const
{
    return (robot_speed.vy);
}

inline float steer_wheel_class::get_robot_speed_vw() const
{
    return (robot_speed.vw);
}

inline float steer_wheel_class::get_position_x() const
{
    return (encoder_pose.x);
}

inline float steer_wheel_class::get_position_y() const
{
    return (encoder_pose.y);
}

inline float steer_wheel_class::get_position_yaw() const
{
    return (encoder_pose.yaw);
}

inline float steer_wheel_class::get_speed_position_x() const
{
    return (speed_pose.x);
}

inline float steer_wheel_class::get_speed_position_y() const
{
    return (speed_pose.y);
}

inline float steer_wheel_class::get_speed_position_yaw() const
{
    return (speed_pose.yaw);
}

inline void steer_wheel_class::set_target_max_step(const float &__target_max_step)
{
    // 限幅斜率必须为正, 否则目标角无法推进
    if (!isfinite(__target_max_step) || __target_max_step <= 0.0f)
    {
        return;
    }
    target_max_step = __target_max_step;
}

inline void steer_wheel_class::set_max_wheel_speed(const float &__max_wheel_speed)
{
    if (!isfinite(__max_wheel_speed) || __max_wheel_speed <= 0.0f)
    {
        return;
    }
    max_wheel_speed = __max_wheel_speed;
}

inline void steer_wheel_class::set_min_limit(const float &__min_limit)
{
    if (!isfinite(__min_limit) || __min_limit < 0.0f)
    {
        return;
    }
    min_limit = __min_limit;
}

inline void steer_wheel_class::set_steer_angle_deadband(const float &__steer_angle_deadband)
{
    // 死区可为 0 (不防抖), 但不能为负
    if (!isfinite(__steer_angle_deadband) || __steer_angle_deadband < 0.0f)
    {
        return;
    }
    steer_angle_deadband = __steer_angle_deadband;
}

inline void steer_wheel_class::set_flip_hysteresis(const float &__flip_hysteresis)
{
    // 迟滞必须小于 90 度: 否则正向阈值 >= 180 度(永不反转)、反向阈值 <= 0 度(永不转回),
    // 方向会被永久锁死在初始分支, 就近转位退化为"只认一侧"
    if (!isfinite(__flip_hysteresis) || __flip_hysteresis < 0.0f || __flip_hysteresis >= PI * 0.5f)
    {
        return;
    }
    flip_hysteresis = __flip_hysteresis;
}

inline void steer_wheel_class::set_cosine_optimize_enable(const bool &__enable)
{
    cosine_optimize_enable = __enable;
}

inline void steer_wheel_class::set_nearest_optimize_enable(const bool &__enable)
{
    nearest_optimize_enable = __enable;
}

inline void steer_wheel_class::set_target_step_limit_enable(const bool &__enable)
{
    target_step_limit_enable = __enable;
}

inline void steer_wheel_class::set_steer_encoder_direction(const float &__direction)
{
    // 只接受 ±1: 其它值既不是"同向"也不是"反向", 直接忽略避免引入莫名的比例缩放
    if (!isfinite(__direction) || __direction == 0.0f)
    {
        return;
    }

    steer_encoder_direction = (__direction > 0.0f) ? 1.0f : -1.0f;
}

inline void steer_wheel_class::set_control_period(const float &__control_period)
{
    // 速度里程计以该周期做积分, 必须是有限正值
    if (!isfinite(__control_period) || __control_period <= 0.0f)
    {
        return;
    }
    control_period = __control_period;
}

inline void steer_wheel_class::update_robot_state(const wheel_robot_speed_t &__target_speed, const uint8_t &__control_mode)
{
    // 控制模式与速度相互独立: 速度非法时仍更新模式, 保持原 update_control_mode() 的语义
    control_mode = __control_mode;

    if (!isfinite(__target_speed.vx) || !isfinite(__target_speed.vy) || !isfinite(__target_speed.vw))
    {
        return;
    }
    target_speed = __target_speed;
}

inline void steer_wheel_class::update_now_state(const steer_state_t &__now_steer_state, const wheel_state_t &__now_wheel_state, const float &__imu_yaw_rad)
{
    // 逐轮独立校验: 单个通道异常只跳过该通道, 不丢弃另一组有效反馈
    for (uint8_t i = 0; i < 4; i++)
    {
        if (isfinite(__now_steer_state.angle[i]))
        {
            status.now_angle[i] = __now_steer_state.angle[i];
        }

        if (isfinite(__now_wheel_state.speed[i]))
        {
            status.now_speed[i] = __now_wheel_state.speed[i];
        }
    }

    // 轮向角与舵向角必须同帧有效才提交: 位移解算用"轮位移 × 舵向角", 混用新旧快照会算出虚假平移/航向;
    // 整帧丢弃且不推进基准, 恢复后位移一次补齐, 一条都不丢
    bool frame_valid = true;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(__now_wheel_state.angle[i]) || !isfinite(__now_steer_state.angle[i]))
        {
            frame_valid = false;
        }
    }

    if (frame_valid)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_angle[i] = __now_wheel_state.angle[i];
            steer_angle[i] = __now_steer_state.angle[i];
        }
    }

    // IMU 航向与轮/舵向角相互独立: 任一通道异常不影响另一路
    if (isfinite(__imu_yaw_rad))
    {
        imu_yaw = __imu_yaw_rad;
    }
}
