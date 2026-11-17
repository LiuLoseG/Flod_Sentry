#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "pid.h"

namespace
{
int failures = 0;
int checks = 0;

void expect_near(float actual, float expected, const char *name)
{
    ++checks;
    if (!std::isfinite(actual) || std::fabs(actual - expected) > 1e-5f)
    {
        ++failures;
        std::printf("FAIL: %s (actual=%g, expected=%g)\n", name, actual, expected);
    }
}

void check_tracking(pid_anti_windup_e mode, float hard_limit, float tracking_limit,
                    float ki, float direction)
{
    pid_class pid;
    pid.init(2.0f, ki, 0.0f, 0.0f, 0.0f, hard_limit, 0.1f,
             0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE,
             mode, 1.0f, tracking_limit);
    pid.set_target(direction);
    pid.pid_calculate_callback();

    float limit = std::fabs(hard_limit);
    if (mode != PID_Anti_Windup_DISABLE && tracking_limit != 0.0f &&
        (limit == 0.0f || std::fabs(tracking_limit) < limit))
    {
        limit = std::fabs(tracking_limit);
    }

    const float proposed = 2.0f + 0.1f * ki;
    const bool tracking = mode != PID_Anti_Windup_DISABLE && limit != 0.0f && proposed > limit;
    float expected_integral = direction * 0.1f;
    float expected_raw = direction * proposed;
    if (tracking && mode == PID_Anti_Windup_CLAMP && ki > 0.0f)
    {
        expected_integral = 0.0f;
        expected_raw = direction * 2.0f;
    }
    else if (tracking && mode == PID_Anti_Windup_BACK_CALC)
    {
        expected_integral += direction * (limit - proposed) * 0.1f / ki;
    }

    const float expected_out = hard_limit == 0.0f ? expected_raw :
        basic_math_constrain(expected_raw, -std::fabs(hard_limit), std::fabs(hard_limit));
    expect_near(pid.get_integral_error(), expected_integral, "tracking integral");
    expect_near(pid.get_out_unsaturated(), expected_raw, "output decision timing");
    expect_near(pid.get_out(), expected_out, "hard output limit is independent");
}

void check_unwinding(float direction)
{
    pid_class pid;
    pid.init(2.0f, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.1f,
             0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE,
             PID_Anti_Windup_CLAMP, 1.0f, 1.0f);
    pid.set_integral_error(direction * 5.0f);
    pid.set_target(-direction);
    pid.pid_calculate_callback();
    expect_near(pid.get_integral_error(), direction * 4.9f, "CLAMP permits unwinding");
}

struct loop_result
{
    double iae = 0.0;
    double final_error = 0.0;
    double peak_integral = 0.0;
    bool valid = true;
};

loop_result simulate_loop(pid_anti_windup_e mode, float dt, float hard_limit,
                          float direction, bool variable_speed = false, bool loaded = false)
{
    pid_class pid;
    const float tracking_limit = hard_limit == 1.0f ? 0.0f : 1.0f;
    pid.init(2.0f, 4.0f, 0.05f, 0.0f, 0.0f, hard_limit, dt,
             0.0f, variable_speed ? 0.05f : 0.0f, 0.0f, 0.0f,
             PID_D_First_ENABLE, mode, 10.0f, tracking_limit);
    pid.set_now(0.0f);
    pid.set_target(0.0f);
    pid.pid_clear();

    loop_result result;
    double y = 0.0;
    const double decay = std::exp(-static_cast<double>(dt) / 0.2);
    const int charge_steps = static_cast<int>(std::lround(3.0 / dt));
    const int recovery_steps = static_cast<int>(std::lround(7.0 / dt));
    const int total_steps = charge_steps + recovery_steps;
    for (int step = 0; step < total_steps; ++step)
    {
        const float target = direction * (step < charge_steps ? 3.0f : 0.4f);
        pid.set_target(target);
        pid.set_now(static_cast<float>(y));
        pid.pid_calculate_callback();
        const float command = pid.get_out();
        const float integral = pid.get_integral_error();
        if (!std::isfinite(command) || !std::isfinite(integral) ||
            !std::isfinite(pid.get_out_unsaturated()) ||
            (hard_limit != 0.0f && std::fabs(command) > hard_limit + 1e-5f))
        {
            result.valid = false;
            break;
        }
        // The actuator really clips the command, including when the PID hard limit is wider.
        const double applied = basic_math_constrain(command, -1.0f, 1.0f);
        const double load = loaded && step >= charge_steps ? -direction * 0.2 : 0.0;
        y = decay * y + (1.0 - decay) * (applied + load);
        if (!std::isfinite(y))
        {
            result.valid = false;
            break;
        }
        result.peak_integral = std::fmax(result.peak_integral, std::fabs(integral));
        if (step >= charge_steps)
            result.iae += std::fabs(target - y) * dt;
    }
    result.final_error = std::fabs(direction * 0.4 - y);
    return result;
}

void check_closed_loops()
{
    for (float dt : {0.001f, 0.002f})
        for (float direction : {1.0f, -1.0f})
            for (bool loaded : {false, true})
            {
                const loop_result baseline = simulate_loop(PID_Anti_Windup_DISABLE, dt, 1.0f,
                                                           direction, false, loaded);
                ++checks;
                if (!baseline.valid || baseline.iae < 1.0)
                {
                    ++failures;
                    std::printf("FAIL: saturated baseline (dt=%g, direction=%g, load=%d)\n",
                                dt, direction, loaded);
                }
                for (int mode = PID_Anti_Windup_CLAMP; mode <= PID_Anti_Windup_BACK_CALC; ++mode)
                {
                    const auto aw = static_cast<pid_anti_windup_e>(mode);
                    const loop_result reference = simulate_loop(aw, dt, 1.0f, direction, false, loaded);
                    for (float hard_limit : {1.0f, 10.0f, 0.0f})
                    {
                        const loop_result result = simulate_loop(aw, dt, hard_limit, direction,
                                                                false, loaded);
                        ++checks;
                        if (!result.valid || result.iae >= baseline.iae * 0.5 ||
                            result.final_error >= 0.001 || result.peak_integral > 2.0)
                        {
                            ++failures;
                            std::printf("FAIL: recovery (mode=%d, dt=%g, dir=%g, hard=%g, load=%d, "
                                        "IAE=%g, final=%g, peakI=%g)\n", mode, dt, direction,
                                        hard_limit, loaded, result.iae, result.final_error,
                                        result.peak_integral);
                        }
                        expect_near(static_cast<float>(result.iae), static_cast<float>(reference.iae),
                                    "external clipping matches internal clipping recovery");
                        const loop_result opposite = simulate_loop(aw, dt, hard_limit, -direction,
                                                                  false, loaded);
                        expect_near(static_cast<float>(result.iae), static_cast<float>(opposite.iae),
                                    "closed-loop recovery is sign symmetric");
                        const loop_result b_zero = simulate_loop(aw, dt, hard_limit, direction,
                                                                true, loaded);
                        expect_near(static_cast<float>(result.iae), static_cast<float>(b_zero.iae),
                                    "B=0 preserves full-speed closed-loop recovery with A>0");
                        if (dt == 0.002f)
                        {
                            const loop_result faster = simulate_loop(aw, 0.001f, hard_limit,
                                                                     direction, false, loaded);
                            ++checks;
                            if (!faster.valid || std::fabs(result.iae - faster.iae) > faster.iae * 0.02)
                            {
                                ++failures;
                                std::printf("FAIL: recovery IAE changes more than 2%% with sample period\n");
                            }
                        }
                        std::printf("LOOP: mode=%d dt=%g dir=%g hard=%g load=%d IAE=%.6f final=%.8f\n",
                                    mode, dt, direction, hard_limit, loaded, result.iae,
                                    result.final_error);
                    }
                }
            }
}
}

int main()
{
    const float hard_limits[] = {10.0f, 0.0f, 1.0f, -10.0f};
    const float tracking_limits[] = {1.0f, -1.0f, 0.0f, 20.0f};
    const float gains[] = {1.0f, -1.0f};
    const float directions[] = {1.0f, -1.0f};
    for (float hard_limit : hard_limits)
        for (float tracking_limit : tracking_limits)
            for (float ki : gains)
                for (float direction : directions)
                    for (int mode = PID_Anti_Windup_DISABLE; mode <= PID_Anti_Windup_BACK_CALC; ++mode)
                        check_tracking(static_cast<pid_anti_windup_e>(mode), hard_limit,
                                       tracking_limit, ki, direction);

    for (float direction : directions)
    {
        check_unwinding(direction);
        for (float a : {0.0f, 0.05f})
        {
            pid_class pid;
            pid.init(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, a, 0.0f);
            pid.set_target(direction * 0.2f);
            for (int i = 0; i < 1000; ++i)
                pid.pid_calculate_callback();
            expect_near(pid.get_out(), direction * 0.2f, "B=0 disables variable-speed weighting");
        }
        for (float error : {0.03f, 0.075f, 0.2f})
        {
            pid_class pid;
            pid.init(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, 0.05f, 0.1f);
            pid.set_target(direction * error);
            pid.pid_calculate_callback();
            const float ratio = error <= 0.05f ? 1.0f :
                (error >= 0.1f ? 0.0f : (0.1f - error) / 0.05f);
            expect_near(pid.get_out(), direction * error * 0.001f * ratio,
                        "valid variable-speed weighting is unchanged");
        }
    }

    check_closed_loops();
    std::printf("checks: %d, failures: %d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
