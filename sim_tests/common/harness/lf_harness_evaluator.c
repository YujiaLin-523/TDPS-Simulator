#include "lf_harness_evaluator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lf_app.h"
#include "lf_config.h"
#include "lf_harness_core.h"
#include "lf_harness_scenarios.h"
#include "lf_platform.h"

static double lfh_clamp_d(double v, double lo, double hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static double lfh_score_scenario(const LFH_ScenarioResult *r)
{
    double score;

    if (r->has_runtime_error) {
        return 0.0;
    }

    score = 100.0;
    score -= (1.0 - r->line_detection_rate) * 60.0;
    score -= fmin(r->mean_abs_error_m / 0.08, 1.0) * 20.0;
    score -= fmin(r->longest_lost_sec / 1.2, 1.0) * 15.0;
    score -= fmin(r->motor_saturation_rate / 0.6, 1.0) * 5.0;
    return lfh_clamp_d(score, 0.0, 100.0);
}

LFH_ScenarioResult LFH_Evaluator_RunScenario(const LFH_Scenario *scenario, const LFH_TestConfig *cfg)
{
    const uint32_t target_steps = (uint32_t)fmax(1.0, llround(cfg->duration_sec / cfg->dt_sec));
    LFH_ScenarioResult result;
    double current_lost_sec = 0.0;
    double abs_error_accum = 0.0;
    double sq_error_accum = 0.0;
    uint32_t error_samples = 0U;
    uint32_t motor_saturation_steps = 0U;
    bool prev_valid = false;
    bool prev_line_detected = false;
    uint32_t step;

    memset(&result, 0, sizeof(result));
    result.id = scenario->id;
    result.name = scenario->name;
    result.preset = LFH_Scenarios_TrackName(scenario->track);
    result.duration_target_sec = cfg->duration_sec;

    LFH_Core_Reset(scenario, cfg);
    LF_Platform_BoardInit();
    LF_App_Init();

    for (step = 0U; step < target_steps; ++step) {
        double sensor_norm[LF_SENSOR_COUNT];
        uint16_t sensor_raw[LF_SENSOR_COUNT];
        LFH_LineState st;
        const LF_AppContext *ctx;

        LFH_Core_ReadSensorNormAndRaw(sensor_norm, sensor_raw);
        st = LFH_Core_EstimateLineState(sensor_norm, cfg->line_threshold);

        LFH_Core_AdvanceTime(cfg->dt_sec);
        LF_App_RunStep();
        result.distance_m += LFH_Core_UpdatePoseFromCommand(cfg->dt_sec);
        result.steps += 1U;

        if (st.line_detected) {
            result.line_detection_rate += 1.0;
            abs_error_accum += fabs(st.line_error_m);
            sq_error_accum += st.line_error_m * st.line_error_m;
            if (fabs(st.line_error_m) > result.max_abs_error_m) {
                result.max_abs_error_m = fabs(st.line_error_m);
            }
            error_samples += 1U;
            if (current_lost_sec > 0.0) {
                if (current_lost_sec > result.longest_lost_sec) {
                    result.longest_lost_sec = current_lost_sec;
                }
                current_lost_sec = 0.0;
            }
        } else {
            current_lost_sec += cfg->dt_sec;
            result.total_lost_sec += cfg->dt_sec;
            if (prev_valid && prev_line_detected) {
                result.line_lost_transitions += 1U;
            }
        }

        if (prev_valid && (!prev_line_detected) && st.line_detected) {
            result.line_recovered_transitions += 1U;
        }
        prev_line_detected = st.line_detected;
        prev_valid = true;

        if ((abs(LFH_Core_GetLeftCommand()) >= (g_lf_config.max_motor_cmd - 1)) ||
            (abs(LFH_Core_GetRightCommand()) >= (g_lf_config.max_motor_cmd - 1))) {
            motor_saturation_steps += 1U;
        }

        ctx = LF_App_GetContext();
        if (ctx->state == LF_APP_STATE_FAULT) {
            result.has_runtime_error = true;
            snprintf(result.runtime_error, sizeof(result.runtime_error), "State entered FAULT");
            break;
        }
    }

    if (current_lost_sec > result.longest_lost_sec) {
        result.longest_lost_sec = current_lost_sec;
    }

    result.duration_simulated_sec = result.steps * cfg->dt_sec;
    if (result.steps > 0U) {
        result.line_detection_rate /= (double)result.steps;
        result.motor_saturation_rate = (double)motor_saturation_steps / (double)result.steps;
    }
    if (error_samples > 0U) {
        result.mean_abs_error_m = abs_error_accum / (double)error_samples;
        result.rms_error_m = sqrt(sq_error_accum / (double)error_samples);
    }

    result.score = lfh_score_scenario(&result);
    return result;
}

void LFH_Evaluator_BuildSuiteSummary(const LFH_ScenarioResult *results,
                                     size_t result_count,
                                     LFH_SuiteSummary *summary,
                                     uint32_t *low_score_count,
                                     char runtime_ids[256],
                                     size_t *runtime_issue_count)
{
    size_t i;
    uint32_t completed = 0U;
    double score_sum = 0.0;
    double detect_sum = 0.0;
    double max_longest = 0.0;

    *low_score_count = 0U;
    *runtime_issue_count = 0U;
    runtime_ids[0] = '\0';

    for (i = 0U; i < result_count; ++i) {
        if (!results[i].has_runtime_error) {
            completed += 1U;
        }

        if (results[i].longest_lost_sec > max_longest) {
            max_longest = results[i].longest_lost_sec;
        }

        score_sum += results[i].score;
        detect_sum += results[i].line_detection_rate;

        if (results[i].has_runtime_error) {
            if (*runtime_issue_count > 0U) {
                strncat(runtime_ids, ", ", 255U - strlen(runtime_ids));
            }
            strncat(runtime_ids, results[i].id, 255U - strlen(runtime_ids));
            *runtime_issue_count += 1U;
        }

        if (results[i].score < LFH_GATE_MIN_SCENARIO_SCORE) {
            *low_score_count += 1U;
        }
    }

    summary->scenario_count = (uint32_t)result_count;
    summary->completed_count = completed;
    summary->aborted = false;
    summary->overall_score = score_sum / (double)result_count;
    summary->avg_line_detection_rate = detect_sum / (double)result_count;
    summary->max_longest_lost_sec = max_longest;
}

void LFH_Evaluator_CollectIssues(const LFH_SuiteSummary *summary,
                                 uint32_t low_score_count,
                                 const char runtime_ids[256],
                                 size_t runtime_issue_count,
                                 LFH_IssueList *issue_list)
{
    static char runtime_issue[320];

    issue_list->issue_count = 0U;

    if (runtime_issue_count > 0U) {
        snprintf(runtime_issue, sizeof(runtime_issue), "Runtime errors in: %s", runtime_ids);
        issue_list->issues[issue_list->issue_count++] = runtime_issue;
    }
    if (summary->avg_line_detection_rate < LFH_GATE_DETECTION_MIN) {
        issue_list->issues[issue_list->issue_count++] =
            "Average line detection is below 94%; tracking robustness is insufficient.";
    }
    if (summary->max_longest_lost_sec > LFH_GATE_MAX_LOST_SEC) {
        issue_list->issues[issue_list->issue_count++] =
            "Longest line loss exceeds 0.35s in at least one scenario.";
    }
    if (summary->overall_score < LFH_GATE_OVERALL_SCORE_MIN) {
        issue_list->issues[issue_list->issue_count++] =
            "Overall score is below 82; tune speed/PID/recovery for stable behavior.";
    }
    if (low_score_count > 0U) {
        issue_list->issues[issue_list->issue_count++] =
            "At least one scenario score is below 70; local weakness remains.";
    }
}
