/**
 * @file    heater.c
 * @brief   Heater PWM driver (TIM3_CH1, PC6) implementation.
 *
 * Implements Requirement 5 of the @c hardware-base-drivers spec:
 *   - PWM start-up and frequency self-correction to ~1 Hz.
 *   - Duty-cycle clamping to [0, 1] with comparator update.
 *   - Emergency-stop / fault-latch state machine.
 *   - Over-temperature trip threshold.
 *
 * The driver intentionally keeps zero dependencies on FreeRTOS — the timer
 * register accesses are atomic enough that no mutex is required, and
 * @c Heater_ApplyDuty (in @c common/hardware.c) is the entry point that
 * couples this module to the shared @c SensorData_t mutex.
 */
#include "hardware.h"
#include "tim.h"

/* ================================================================== */
/* Module-private state                                                */
/* ================================================================== */

/** True after @c Heater_Init has completed successfully. */
static bool  s_initialized   = false;

/** Latched-fault flag.  Set by @c Heater_EmergencyStop and
 *  @c Heater_OverheatCheck; cleared only by @c Heater_ClearFault. */
static bool  s_fault_latched = false;

/** Most recent duty value applied to TIM3_CH1's CCR register. */
static float s_current_duty  = 0.0f;

/* ================================================================== */
/* Constants for the 1 Hz frequency self-correction step (Req 5.2)     */
/* ================================================================== */

/** APB1 timer kernel clock (240 MHz on STM32H743 with default RCC). */
#define HEATER_TIM3_CLOCK_HZ    240000000.0f

/** Acceptable PWM frequency band [0.9, 1.1] Hz. */
#define HEATER_FREQ_LO_HZ       0.9f
#define HEATER_FREQ_HI_HZ       1.1f

/** Forced 1 Hz configuration: 24000 * 10000 = 2.4e8 ticks per period. */
#define HEATER_FORCE_PSC        (24000U - 1U)
#define HEATER_FORCE_ARR        (10000U - 1U)

/* ================================================================== */
/* Helpers                                                              */
/* ================================================================== */

/**
 * @brief Clamp @p duty into [0, 1].  NaN inputs are reported via @p out_nan.
 */
static float heater_clamp_duty(float duty, bool *out_nan)
{
    /* IEEE-754: NaN compares unequal to itself. */
    if (duty != duty) {
        if (out_nan) *out_nan = true;
        return 0.0f;
    }
    if (out_nan) *out_nan = false;

    if (duty < 0.0f) {
        return 0.0f;
    }
    if (duty > 1.0f) {
        return 1.0f;
    }
    return duty;
}

/**
 * @brief Internal version of EmergencyStop that skips the @c s_initialized
 *        guard.  Used from @c Heater_OverheatCheck after the public guard
 *        has already passed, so we don't return a misleading status.
 */
static void heater_emergency_stop_locked(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0u);
    s_current_duty  = 0.0f;
    s_fault_latched = true;
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

Driver_Status Heater_Init(void)
{
    /* --- 1. Frequency self-correction (Requirement 5.2) ----------------- */
    const uint32_t psc_plus_1 = htim3.Init.Prescaler + 1U;
    const uint32_t arr_plus_1 = htim3.Init.Period    + 1U;

    /* Defensive: a degenerate (PSC, ARR) of (0, 0) would mean +1=1 each, so
     * f = 240 MHz, way out of band — handled by the if-check below.  We
     * still guard against literal zero-overflow products just in case. */
    bool needs_force = false;
    if (psc_plus_1 == 0U || arr_plus_1 == 0U) {
        needs_force = true;
    } else {
        const float ticks_per_period = (float)psc_plus_1 * (float)arr_plus_1;
        const float f_hz = HEATER_TIM3_CLOCK_HZ / ticks_per_period;
        if (f_hz < HEATER_FREQ_LO_HZ || f_hz > HEATER_FREQ_HI_HZ) {
            needs_force = true;
        }
    }

    if (needs_force) {
        __HAL_TIM_SET_PRESCALER (&htim3, HEATER_FORCE_PSC);
        __HAL_TIM_SET_AUTORELOAD(&htim3, HEATER_FORCE_ARR);
        /* The HAL macros write the registers; on host the shim mirrors them
         * into @c htim3.Init.{Prescaler,Period}.  Be defensive and keep the
         * cached @c Init fields in sync for the on-target case as well — the
         * later @c Heater_SetDuty math reads them directly. */
        htim3.Init.Prescaler = HEATER_FORCE_PSC;
        htim3.Init.Period    = HEATER_FORCE_ARR;
    }

    /* --- 2. Reset duty to 0 before enabling the output ------------------ */
    s_current_duty = 0.0f;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0u);

    /* --- 3. Start PWM --------------------------------------------------- */
    HAL_StatusTypeDef hs = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    if (hs != HAL_OK) {
        return Driver_MapHalStatus(hs);
    }

    /* --- 4. Mark ready -------------------------------------------------- */
    s_fault_latched = false;
    s_initialized   = true;
    return DRV_OK;
}

Driver_Status Heater_SetDuty(float duty)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    /* Fault-latched path: never touch the comparator (Requirement 5.7). */
    if (s_fault_latched) {
        return DRV_ERR_NOT_INIT;
    }

    bool is_nan = false;
    const float clamped = heater_clamp_duty(duty, &is_nan);
    if (is_nan) {
        return DRV_ERR_PARAM;
    }

    /* CCR = duty * (ARR + 1).  ARR + 1 = number of timer ticks per period;
     * a clamped duty of 1 maps to (ARR+1), giving a full-on output. */
    const uint32_t ccr = (uint32_t)(clamped * (float)(htim3.Init.Period + 1U));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
    s_current_duty = clamped;
    return DRV_OK;
}

Driver_Status Heater_GetDuty(float *duty)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (duty == NULL) {
        return DRV_ERR_PARAM;
    }
    *duty = s_current_duty;
    return DRV_OK;
}

Driver_Status Heater_EmergencyStop(void)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    heater_emergency_stop_locked();
    return DRV_OK;
}

Driver_Status Heater_ClearFault(void)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    s_fault_latched = false;
    return DRV_OK;
}

Driver_Status Heater_OverheatCheck(float temperature_c)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (temperature_c >= HEATER_OVERHEAT_THRESHOLD_C) {
        heater_emergency_stop_locked();
        return DRV_ERR_PARAM;
    }
    return DRV_OK;
}

bool Heater_IsFaultLatched(void)
{
    /* Read-only; safe to query before init (returns false because the static
     * s_fault_latched is zero-initialised). */
    if (!s_initialized) {
        return false;
    }
    return s_fault_latched;
}
