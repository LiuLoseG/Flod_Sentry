/**
 * @file power_ctrl.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-11-26
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "power_ctrl.h"
#include <math.h>
#include <stddef.h>

#define PQ_EPS          1e-6f       // 判"是否为零"、"区间是否相交"、"二次是否退化为线性"
#define PQ_SCAN_ITERS   16          // 迭代次数 三分搜索定位功率谷值
#define PQ_BOUND_ITERS  16          // 迭代次数 二分搜索可行区间边界
#define PQ_GOLDEN_ITERS 22          // 迭代次数 黄金分割搜索最优解
#define PQ_PRIORITY_GAIN 32.0f      // 优先增益 平移/旋转优先级倍数

// 二维求解器使用的功率二次函数系数、目标权重和约束
typedef struct {
    float A, B, C, D, E, F;
    float w_tt, w_tr, w_rr;
    float limit;
    const power_motion_node_t *nodes;
    uint8_t count;
} power_quadctx_t;

// 电机 M3508 的功率模型参数，单位 W、A、rad/s
const power_motor_model_t model_m3508 = 
{
    .k1 = 1.5756e-02f, .k2 = 1.94e-01f,
    .k3 = 1.9202e-05f, .k4 = 0.8f,
};

// 根据电机转速和 raw 电流指令预测单个电机的输入功率
static inline float predict_motor_power(const power_node_t *node, float current_cmd)
{
    return node->model->k1 * node->state->speed_rad_s * current_cmd +
           node->model->k2 * current_cmd * current_cmd +
           node->model->k3 * node->state->speed_rad_s * node->state->speed_rad_s +
           node->model->k4;
}

// 按平移/旋转比例计算含固定轮级修正的带符号预测功率
static float motion_power(const power_motion_node_t *nodes, uint8_t count, float translation_scale, float rotation_scale)
{
    float power = 0.0f;
    for (uint8_t i = 0; i < count; i++)
    {
        float cmd = translation_scale * nodes[i].translation_cmd +
                    rotation_scale * nodes[i].rotation_cmd +
                    nodes[i].compensation_cmd;

        float p = predict_motor_power(&nodes[i].motor, cmd);
        if (!isfinite(p)) return INFINITY; // 避免出现 NaN
        power += p;
    }
    return power;
}

// 功率分配失败、系统输出零电流时，零电流指令下的模型功率基线
static float motion_idle_power(const power_motion_node_t *nodes, uint8_t count)
{
    float power = 0.0f;
    for (uint8_t i = 0; i < count; i++)
    {
        const power_motor_model_t *model = nodes[i].motor.model;
        float speed_rad = nodes[i].motor.state->speed_rad_s;

        power += model->k3 * speed_rad * speed_rad + 
                 model->k4;
    }
    return power;
}

// 检查合成后的 raw 电流是否满足每个电机的电流上限
static bool motion_current_valid(const power_motion_node_t *nodes, uint8_t count, 
                                 float ts, float rs)
{
    for (uint8_t i = 0; i < count; i++)
    {
        float cmd = ts * nodes[i].translation_cmd + rs * nodes[i].rotation_cmd +
                    nodes[i].compensation_cmd;
        if (!isfinite(cmd) || fabsf(cmd) > nodes[i].max_cmd) return false;
    }
    return true;
}


// 校验节点、模型和数值输入，并确保每个节点使用独立状态对象
static bool validate_nodes(const power_motion_node_t *nodes, uint8_t count)
{
    if (nodes == NULL || count == 0) return false;
    for (uint8_t i = 0; i < count; i++) {
        const power_motion_node_t *n = &nodes[i];
        const power_motor_model_t *m = n->motor.model;

        if (n->motor.state == NULL || m == NULL ||
            !isfinite(n->motor.state->speed_rad_s) ||
            !isfinite(n->translation_cmd) || !isfinite(n->rotation_cmd) ||
            !isfinite(n->compensation_cmd) ||
            !isfinite(n->max_cmd) || n->max_cmd < 0.0f ||
            !isfinite(m->k1) || m->k1 < 0.0f ||
            !isfinite(m->k2) || m->k2 < 0.0f ||
            !isfinite(m->k3) || m->k3 < 0.0f ||
            !isfinite(m->k4) || m->k4 < 0.0f) return false;

        for (uint8_t j = 0; j < i; j++)
            if (nodes[j].motor.state == n->motor.state) return false;
    }
    return true;
}

// 计算比例点 (t, r) 对应的展开功率二次函数值
static float quad_p(const power_quadctx_t *q, float t, float r)
{
    return q->A * t * t + q->B * t * r + q->C * r * r +
           q->D * t + q->E * r + q->F;
}

// 固定 t 时，求所有电机给出的 r 可行区间交集
static bool current_r_interval(const power_quadctx_t *q, float t, float *lo, float *hi)
{
    float l = 0.0f, h = 1.0f;
    for (uint8_t i = 0; i < q->count; i++) {
        float base = t * q->nodes[i].translation_cmd + q->nodes[i].compensation_cmd;
        float slope = q->nodes[i].rotation_cmd;
        float m = q->nodes[i].max_cmd;

        if (fabsf(slope) <= PQ_EPS) {
            if (fabsf(base) > m + PQ_EPS) return false;
            continue;
        }

        float r1 = (-m - base) / slope;
        float r2 = ( m - base) / slope;
        l = fmaxf(l, fminf(r1, r2));
        h = fminf(h, fmaxf(r1, r2));
    }

    *lo = l; *hi = h;
    return l <= h + PQ_EPS;
}

// 在固定 t 的 r 可行区间内计算最小预测功率
static float min_power_over_r(const power_quadctx_t *q, float t)
{
    float lo, hi;

    if (!current_r_interval(q, t, &lo, &hi)) return INFINITY;

    if (q->C > PQ_EPS) {
        float rp = -(q->B * t + q->E) / (2.0f * q->C);
        return quad_p(q, t, fminf(hi, fmaxf(lo, rp)));
    }

    // 功率关于 r 为线性时，最小值位于区间端点
    return fminf(quad_p(q, t, lo), quad_p(q, t, hi));
}

// 判断给定 t 是否至少存在一个满足约束的 r
static bool t_feasible(const power_quadctx_t *q, float t)
{
    return min_power_over_r(q, t) <= q->limit + PQ_EPS;
}

// 求固定 t 下的最优可行 r，并返回对应的加权运动误差
static float evaluate_cost(const power_quadctx_t *q, float t, float *out_r)
{
    float lo, hi;

    if (!current_r_interval(q, t, &lo, &hi)) return INFINITY;

    float BtE = q->B * t + q->E;
    float rhs = q->A * t * t + q->D * t + q->F - q->limit;   /* r⁰ 系数 −W */

    // 根据二次方程的判别式，确定 r 的可行区间
    if (q->C <= PQ_EPS)  /* 退化：r 线性 */
    {                                   
        if (fabsf(BtE) <= PQ_EPS) 
        {
            if (rhs > 0.0f) return INFINITY;
        }

        else if (BtE > 0.0f)   hi = fminf(hi, -rhs / BtE);

        else                   lo = fmaxf(lo, -rhs / BtE);

    }
    else 
    {
        float disc = BtE * BtE - 4.0f * q->C * rhs;
        if (disc < 0.0f) return INFINITY;
        float s = sqrtf(disc);

        lo = fmaxf(lo, (-BtE - s) / (2.0f * q->C));

        hi = fminf(hi, (-BtE + s) / (2.0f * q->C));
    }

    if (lo > hi + PQ_EPS) return INFINITY;

    // 对 r 求二次目标函数极小值，并将结果限制在可行区间内
    float r;
    if (q->w_rr > PQ_EPS) 
    {
        float r_target = 1.0f + q->w_tr * (1.0f - t) / q->w_rr;
        r = fminf(hi, fmaxf(lo, r_target));
    } 
    else 
    {
        r = hi;
    }

    *out_r = r;
    float dt = 1.0f - t, dr = 1.0f - r;
    return q->w_tt * dt * dt + 2.0f * q->w_tr * dt * dr + q->w_rr * dr * dr;
}

// 搜索可行的 t 区间，并最小化加权运动误差
static bool solve_optimal(const power_quadctx_t *q, float t_lo, float t_hi,
                          float *out_t, float *out_r)
{
    /* 对 F(t) 进行有界三分搜索，定位功率谷值。 */
    float lo = t_lo, hi = t_hi;
    for (uint8_t it = 0; it < PQ_SCAN_ITERS; it++) {
        float a = lo + (hi - lo) / 3.0f;
        float b = hi - (hi - lo) / 3.0f;
        if (min_power_over_r(q, a) <= min_power_over_r(q, b)) hi = b;
        else lo = a;
    }
    float t_m = 0.5f * (lo + hi);
    if (min_power_over_r(q, t_m) > q->limit + PQ_EPS) return false;

    /* 以功率谷值为中心，使用二分搜索可行区间的左右边界。 */
    float fa = t_lo, fb = t_hi;
    if (!t_feasible(q, fa)) {
        float l = fa, h = t_m;
        for (uint8_t it = 0; it < PQ_BOUND_ITERS; it++) {
            float mid = 0.5f * (l + h);
            if (t_feasible(q, mid)) h = mid; else l = mid;
        }
        fa = h;
    }
    if (!t_feasible(q, fb)) {
        float l = t_m, h = fb;
        for (uint8_t it = 0; it < PQ_BOUND_ITERS; it++) {
            float mid = 0.5f * (l + h);
            if (t_feasible(q, mid)) l = mid; else h = mid;
        }
        fb = l;
    }

    /* 在功率可行带内最小化加权运动误差。 */
    const float G = 0.61803399f;
    float x1 = fb - G * (fb - fa), x2 = fa + G * (fb - fa);
    float r1, r2;
    float f1 = evaluate_cost(q, x1, &r1);
    float f2 = evaluate_cost(q, x2, &r2);
    float bt = 0.0f, br = 0.0f, bc = INFINITY;
    bool found = false;

    float er;
    float ec = evaluate_cost(q, fa, &er);              /* 左端点 */
    if (ec < bc) { bc = ec; bt = fa; br = er; found = true; }
    ec = evaluate_cost(q, fb, &er);                    /* 检查右端点 */
    if (ec < bc) { bc = ec; bt = fb; br = er; found = true; }

    for (uint8_t it = 0; it < PQ_GOLDEN_ITERS; it++) {
        if (f1 <= f2) {
            fb = x2; x2 = x1; f2 = f1; r2 = r1;
            x1 = fb - G * (fb - fa); f1 = evaluate_cost(q, x1, &r1);
        } else {
            fa = x1; x1 = x2; f1 = f2; r1 = r2;
            x2 = fa + G * (fb - fa); f2 = evaluate_cost(q, x2, &r2);
        }
        if (f1 < bc) { bc = f1; bt = x1; br = r1; found = true; }
        if (f2 < bc) { bc = f2; bt = x2; br = r2; found = true; }
    }

    /* 黄金分割采样全部无效时，使用低分辨率均匀扫描。 */
    if (!found) {
        for (uint8_t k = 0; k <= 8; k++) {
            float t = t_lo + (t_hi - t_lo) * (float)k / 8.0f;
            float rr, cc = evaluate_cost(q, t, &rr);
            if (cc < bc) { bc = cc; bt = t; br = rr; found = true; }
        }
    }
    if (!found) return false;
    *out_t = bt; *out_r = br;
    return true;
}

// 等权分配和模式加权分配共用的内部实现
static power_motion_status_e power_ctrl_allocate_motion_optimal(
        float allowed_limit, power_motion_node_t *nodes,
        uint8_t node_count, float translation_preference,
        float rotation_preference, power_motion_result_t *result)
{
    /* 先清零可访问的输出，避免输入错误时继续使用上一次结果。 */
    if (nodes != NULL) {
        for (uint8_t i = 0; i < node_count; i++)
            if (nodes[i].motor.state != NULL) {
                nodes[i].motor.state->limited_cmd  = 0.0f;
            }
    }
    if (result == NULL) return POWER_MOTION_INVALID_INPUT;
    *result = (power_motion_result_t){.status = POWER_MOTION_INVALID_INPUT};

    if (!validate_nodes(nodes, node_count) || !isfinite(translation_preference) || 
        translation_preference <= 0.0f || !isfinite(rotation_preference) || 
        rotation_preference <= 0.0f || !isfinite(allowed_limit) || allowed_limit < 0.0f)
        {
            return result->status;
        }

    // 根据节点数据构造功率二次函数和运动误差权重
    power_quadctx_t q = {.limit = allowed_limit, .nodes = nodes, .count = node_count};
    float command_scale = 0.0f;
    for (uint8_t i = 0; i < node_count; i++) 
    {
        const power_motion_node_t *n = &nodes[i];

        float w = n->motor.state->speed_rad_s;
        float a = n->motor.model->k2;
        float b = n->motor.model->k1 * w;
        float o = n->compensation_cmd;

        q.A += a * n->translation_cmd * n->translation_cmd;
        q.B += 2.0f * a * n->translation_cmd * n->rotation_cmd;
        q.C += a * n->rotation_cmd * n->rotation_cmd;
        q.D += (2.0f * a * o + b) * n->translation_cmd;
        q.E += (2.0f * a * o + b) * n->rotation_cmd;
        q.F += a * o * o + b * o + n->motor.model->k3 * w * w + n->motor.model->k4;

        command_scale = fmaxf(command_scale, fmaxf(fabsf(n->translation_cmd), fabsf(n->rotation_cmd)));
    }

    if (command_scale > PQ_EPS) 
    {
        float wtt = 0.0f, wtr = 0.0f, wrr = 0.0f;
        for (uint8_t i = 0; i < node_count; i++) 
        {
            float tn = nodes[i].translation_cmd / command_scale;
            float rn = nodes[i].rotation_cmd / command_scale;
            wtt += tn * tn;
            wtr += tn * rn;          /* 交叉项，可为负 */
            wrr += rn * rn;
        }

        float s = wtt + wrr;
        q.w_tt = fmaxf(wtt / s, PQ_EPS) * translation_preference;
        q.w_tr = (wtr / s) * sqrtf(translation_preference * rotation_preference);
        q.w_rr = fmaxf(wrr / s, PQ_EPS) * rotation_preference;
    } 

    else 
    {
        q.w_tt = 0.5f * translation_preference;
        q.w_tr = 0.0f;
        q.w_rr = 0.5f * rotation_preference;
    }

    /* 先计算完整请求功率。 */
    result->requested_power = motion_power(nodes, node_count, 1.0f, 1.0f);
    if (!isfinite(result->requested_power)) return result->status;

    /* 比例默认取零：求解失败时输出零电流，满足状态码的约定。 */
    float ts = 0.0f, rs = 0.0f;
    bool full_ok = result->requested_power <= allowed_limit &&
                   motion_current_valid(nodes, node_count, 1.0f, 1.0f);
    bool solved;
    if (full_ok) 
    {
        /* 完整请求同时满足功率和每轮电流限制。 */
        ts = 1.0f; rs = 1.0f;
        result->status = POWER_MOTION_OK;
        solved = true;
    } 
    else 
    {
        /* 根据各轮电流约束限制 t 的搜索区间。 */
        float t_lo = 0.0f, t_hi = 1.0f;
        for (uint8_t i = 0; i < node_count; i++) {
            float T = nodes[i].translation_cmd, m = nodes[i].max_cmd;
            if (fabsf(T) <= PQ_EPS) continue;
            float bound = (m + fabsf(nodes[i].rotation_cmd) +
                           fabsf(nodes[i].compensation_cmd)) / fabsf(T);
            t_hi = fminf(t_hi, bound);
        }
        t_hi = fmaxf(0.0f, t_hi);
        solved = solve_optimal(&q, t_lo, t_hi, &ts, &rs);
        result->status = solved ? POWER_MOTION_LIMITED : POWER_MOTION_NO_FEASIBLE_CANDIDATE;
    }

    /* 使用完整模型复核结果，必要时按 ULP 向零方向回退。 */
    for (uint8_t att = 0; solved && att < 8; att++) 
    {
        if (motion_current_valid(nodes, node_count, ts, rs) &&
            motion_power(nodes, node_count, ts, rs) <= allowed_limit) break;
        ts = nextafterf(ts, 0.0f);
        rs = nextafterf(rs, 0.0f);
    }
    if (solved && (!motion_current_valid(nodes, node_count, ts, rs) ||
                   motion_power(nodes, node_count, ts, rs) > allowed_limit)) {
        ts = 0.0f; rs = 0.0f; solved = false;
        result->status = POWER_MOTION_NO_FEASIBLE_CANDIDATE;
    }

    /* 保存比例、功率和每个节点的最终合成电流。 */
    result->translation_scale = ts;
    result->rotation_scale = rs;
    result->allocated_power = motion_power(nodes, node_count, ts, rs);
    for (uint8_t i = 0; i < node_count; i++) 
    {
        nodes[i].motor.state->limited_cmd = solved
            ? ts * nodes[i].translation_cmd + rs * nodes[i].rotation_cmd +
              nodes[i].compensation_cmd
            : 0.0f;
    }
    if (!solved) result->allocated_power = motion_idle_power(nodes, node_count);
    return result->status;
}

// 对平移和旋转使用等权目标进行二维功率分配
power_motion_status_e power_ctrl_allocate_motion_optimal(
    float allowed_limit, power_motion_node_t *nodes,
    uint8_t node_count, power_motion_result_t *result)
{
    return power_ctrl_allocate_motion_optimal(allowed_limit, nodes,
                                               node_count, 1.0f, 1.0f, result);
}

// 按指定运动分量优先级进行二维功率分配
power_motion_status_e power_ctrl_allocate_motion_with_priority(
    float allowed_limit, power_motion_node_t *nodes, uint8_t node_count, 
    power_motion_priority_e priority, power_motion_result_t *result)
{
    if (priority == POWER_PRIORITY_TRANSLATION) {
        return power_ctrl_allocate_motion_optimal(allowed_limit, nodes,
                                                   node_count, PQ_PRIORITY_GAIN,
                                                   1.0f, result);
    }
    if (priority == POWER_PRIORITY_ROTATION) {
        return power_ctrl_allocate_motion_optimal(allowed_limit, nodes,
                                                   node_count, 1.0f,
                                                   PQ_PRIORITY_GAIN, result);
    }
    return power_ctrl_allocate_motion_optimal(allowed_limit, nodes,
                                               node_count, 0.0f, 0.0f, result);
}

