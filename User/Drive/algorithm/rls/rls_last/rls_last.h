#ifndef RLS_LAST_H
#define RLS_LAST_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RLS_LAST_MAX_PARAMETERS 5u
#define RLS_LAST_MAX_OBSERVATIONS 32u

/* One instance, one caller task; synchronize external measurements separately.
 * Pure C99 / float / static storage. Compile without -ffast-math.
 * A ready result is a candidate-data gate, NOT parameter identifiability,
 * a confidence interval, permission to change control gains, or motor enable.
 */
typedef struct {
    uint8_t parameter_count;       /* 1..5 */ // 模型参数个数
    uint8_t excitation_window;     /* parameter_count..32, fixed rolling window */  // 持续激励(PE)检查的固定滚动窗口行数
    uint32_t min_updates;          /* >=1 consecutive non-projected updates */      // 判定 ready 所需的最少"连续非投影更新"次数。防止刚更新一两次就宣称"可用"
    float initial[RLS_LAST_MAX_PARAMETERS]; /* Physical parameter units. */         // 物理量纲的初值， 惯量下界须为正、阻尼/摩擦下界非负、重力系数可正可负
    float minimum[RLS_LAST_MAX_PARAMETERS]; /* Inclusive physical bounds. */        // 物理量纲的下界
    float maximum[RLS_LAST_MAX_PARAMETERS]; /* Inclusive physical bounds. */        // 物理量纲的上界
    float feature_scale[RLS_LAST_MAX_PARAMETERS]; /* Finite, >0, fixed. */          // 每个特征列的归一化尺度， 不同物理量级（角度/角速度/重力/摩擦项）拉到同一数量级
    float output_scale;            /* Finite, >0, fixed. */                         // 观测量（力矩）的归一化尺度
    float covariance_initial;      /* P0 diagonal in normalized coordinates. */     // P0 的对角初值（归一化坐标）。越大表示"初始越不确定"、初值收敛越快但方差大。
    float covariance_limit;        /* Maximum allowed diagonal of P, >=P0. */       // P 对角线允许的最大值
    float forgetting_time_s;       /* 0: no forgetting; else rho=exp(-dt/tau). */   // 遗忘因子时间常数，0 表示不遗忘；否则 rho=exp(-dt/tau)
    float dt_min_s;                                                                 // 可接受步长下限
    float dt_max_s;                                                                 // 可接受步长上限       
    float min_feature_energy;      /* Mean squared normalized feature, >0. */       // 每列特征的最小均方能量
    float min_excitation_pivot;    /* (0,1]; correlation Cholesky pivot BEFORE sqrt. */ // 激励相关阵 Cholesky 的最小主元门限（开方前）。把 Gram 归一化成相关系数矩阵后做 Cholesky，主元低于此值说明特征近似共线/不可分辨 
    float innovation_rms_limit;    /* Physical output units; >0. */                 // 更新前残差 RMS 的上限。用于筛掉模型失配或数据异常的时段。
} rls_last_config_t;

typedef struct {
    float features[RLS_LAST_MAX_PARAMETERS]; /* Active columns only. */             // 物理量纲的回归特征
    float target;                /* Physical output units. */                       // 物理量纲的观测值
    float dt_s;                   /* Interval for this regression observation. */   // 本次回归观测的时间间隔
    bool enabled;   // 使能标志
    bool valid;     // 数据有效性声明
} rls_last_input_t;

enum {
    RLS_LAST_DISABLED         = 1u << 0,    // 禁用 / 未使能
    RLS_LAST_BAD_CONFIG       = 1u << 1,    // 配置非法 / 未初始化 / 状态体检发现配置问题
    RLS_LAST_BAD_INPUT        = 1u << 2,    // 输入空/特征或 target 非有限
    RLS_LAST_BAD_DT           = 1u << 3,    // 非有限或越界
    RLS_LAST_LOW_EXCITATION   = 1u << 4,    // 窗口未满 / 能量不足 / 相关阵近奇异
    RLS_LAST_PROJECTED        = 1u << 5,    // 参数被投影到物理边界（本拍不 ready）
    RLS_LAST_NUMERIC_FAULT    = 1u << 6,    // 数值溢出/非有限/环形游标越界
    RLS_LAST_COVARIANCE_FAULT = 1u << 7     // P 非对称/非正定/对角超限
};

typedef struct {
    float parameters[RLS_LAST_MAX_PARAMETERS]; /* Retained/updated physical values. */      // 更新后（或保留）的物理参数值
    float prediction;             /* Current observation, BEFORE parameter update. */       // 本次观测的预测值
    float residual;               /* target - prediction, physical output units. */         // 物理输出单位。
    float innovation_rms;          /* RMS of up to excitation_window pre-update             // 更新前残差的 RMS（含投影更新）。激励不足/故障时被清零。
                                   * residuals from successful updates (including
                                   * projection). Cleared on insufficient PE/fault. */
    float covariance_diag[RLS_LAST_MAX_PARAMETERS]; /* Normalized P, NOT CI. */             // 归一化的协方差矩阵对角线，非置信区间。用于判断参数收敛性。
    uint32_t flags;
    uint32_t accepted_updates;     /* Lifetime accepted updates, including projection. */   // 生命周期内累计被接受的更新数
    uint32_t consecutive_updates;  /* Consecutive accepted non-projected updates. */        // 连续的非投影更新数
    bool updated;   // 本次调用是否真正写入了参数
    bool ready;     // 连续非投影更新数达到 min_updates，且协方差矩阵对角线均小于 covariance_limit
    bool innovation_rms_valid;  // innovation_rms 是否有效。激励不足/故障时被清零。
} rls_last_output_t;

/* Storage is public only for static allocation. ALL fields are PRIVATE to this
 * module; never edit config/theta/covariance/rings after init. Use snapshot().
 * theta_i = physical_parameter_i * feature_scale_i / output_scale;
 * normalized_feature_i = physical_feature_i / feature_scale_i.
 */
typedef struct {
    rls_last_config_t config;
    float theta[RLS_LAST_MAX_PARAMETERS];   // 归一化参数（真正被更新的量）。
    float covariance[RLS_LAST_MAX_PARAMETERS][RLS_LAST_MAX_PARAMETERS]; // 归一化 P 矩阵，Joseph 形式更新。
    float observations[RLS_LAST_MAX_OBSERVATIONS][RLS_LAST_MAX_PARAMETERS]; // 特征滚动环（存归一化特征，供 PE 判定）。
    float innovations[RLS_LAST_MAX_OBSERVATIONS];   // 更新前残差滚动环（存归一化残差，供 RMS 判定）。
    uint32_t accepted_updates;  // 生命周期内累计被接受的更新数，包括投影更新
    uint32_t consecutive_updates;  // 连续的非投影更新数
    uint8_t observation_count;  // 特征环当前已填行数
    uint8_t observation_next;   // 特征环下一行写入索引
    uint8_t innovation_count;   // 更新前残差环当前已填行数
    uint8_t innovation_next;    // 更新前残差环下一行写入索引
    bool initialized;           // 是否已初始化
} rls_last_t;

/* Defaults demonstrate synthetic regression, not calibrated motor parameters. */
void rls_last_default_config(rls_last_config_t *config, uint8_t parameter_count);
/* A failed init clears the entire object. Configuration changes require init. */
bool rls_last_init(rls_last_t *estimator, const rls_last_config_t *config);
/* Restore configured initial parameters and P0; clear observations and counters. */
void rls_last_reset(rls_last_t *estimator);
/* Keep parameters/P and lifetime accepted_updates. Clear PE, innovations and
 * consecutive_updates. Call on discontinuities; update also calls this on
 * disable/invalid input/numerical faults. Low PE retains only the feature ring.
 */
void rls_last_clear_observations(rls_last_t *estimator);
/* Read retained diagnostics only; always updated=false and ready=false.
 * On failure output is cleared, apart from a diagnostic fault flag. */
bool rls_last_snapshot(const rls_last_t *estimator, rls_last_output_t *output);
/* Every call produces a new ready decision. PE requires a full feature window,
 * per-column energy and correlation pivots. Insufficient PE freezes BOTH theta
 * and P; time spent without PE is not later applied as accumulated forgetting.
 * Joseph update uses normalized measurement variance R=1, symmetrization and
 * Cholesky SPD checks. Numerical failures roll back theta/P for the whole call.
 * Physical bounds project theta itself; a projected update cannot be ready.
 * Failed/disabled calls return updated=false/ready=false; retained physical
 * parameters/P are returned only when that state passes snapshot checks.
 */
uint32_t rls_last_update(rls_last_t *estimator, const rls_last_input_t *input,
                         rls_last_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
