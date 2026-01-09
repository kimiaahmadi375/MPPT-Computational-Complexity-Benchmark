# ============================================================
# Proposed Adaptive Gradient-Descent MPPT with Initialization
# (Flowchart-faithful pseudocode, architecture-aware)
# ============================================================

# -------------------------
# I/O (digital interface)
# -------------------------
# read Vpv, Ipv from ADCs (10-bit in your system)
# apply_duty(D) writes duty to PWM

function measure_Vpv() -> Vpv
function measure_Ipv() -> Ipv
procedure apply_duty(D)

# -------------------------
# Tunable parameters
# -------------------------
const a        # steady-state lock threshold gain (alpha in your text)
const beta     # gradient-to-step scaling (β)
const step_size    # sweep increment for initialization (e.g., 0.01 or design-chosen)
const D_MIN = 0.0
const D_MAX = 0.99  # as in your flowchart

# -------------------------
# State registers (keep across iterations)
# -------------------------
reg start                # 0: still initializing, 1: normal tracking
reg Dprev, Dnew
reg duty_cycle_sweep
reg Dprev_init

reg Vprev, Pprev
reg delta_P_prev

# -------------------------
# Initialization of state
# -------------------------
procedure MPPT_Init():
    start           = 0
    duty_cycle_sweep = D_MIN
    Dprev_init      = duty_cycle_sweep
    Dprev           = Dprev_init

    # Prime history with one measurement
    Vprev = measure_Vpv()
    Iprev = measure_Ipv()
    Pprev = Vprev * Iprev
    delta_P_prev = 0

    apply_duty(Dprev)

# ============================================================
# One MPPT update (call every Ts_mppt)
# ============================================================
procedure MPPT_Iter():

    # ---- 1) Read PV point ----
    Vpv = measure_Vpv()
    Ipv = measure_Ipv()
    Ppv = Vpv * Ipv

    # =========================================================
    # Phase A: Initialization (duty sweep)  [start == 0]
    # =========================================================
    if start == 0:

        # (A1) Optional “best so far” capture as shown
        # NOTE: your flowchart compares Ppv > Pprev (Pprev is effectively Pprev in init)
        if Ppv > Pprev:
            Dprev_init = duty_cycle_sweep
            Pprev      = Ppv
            Vprev      = Vpv
        end if

        # (A2) Sweep duty
        duty_cycle_sweep = duty_cycle_sweep + step_size

        # (A3) End of sweep condition
        if duty_cycle_sweep >= D_MAX:
            Dprev = Dprev_init     # lock to best duty found
            start = 1              # exit init
        else
            Dprev = duty_cycle_sweep
        end if

        apply_duty(Dprev)
        return

    end if

    # =========================================================
    # Phase B: Normal adaptive gradient-descent P&O
    # =========================================================

    # ---- 2) Compute deltas ----
    delta_P = Ppv - Pprev
    delta_V = Vpv - Vprev

    # ---- 3) Compute dpdv = delta_P / delta_V ----
    # Architecture note: protect divide-by-zero
    if delta_V != 0:
        dpdv = delta_P / delta_V
    else
        dpdv = 0
    end if

    # ---- 4) Steady-state lock test ----
    # abs(delta_P_prev - delta_P) < a * Ppv  => ΔD = 0
    # else ΔD = abs(dpdv) * beta
    if abs(delta_P_prev - delta_P) < (a * Ppv):
        delta_D = 0
    else
        delta_D = abs(dpdv) * beta
    end if

    # ---- 5) Duty update rule (sign of dpdv) ----
    # If dpdv > 0  => left of MPP => decrease duty (per your flowchart)
    if dpdv > 0:
        Dnew = Dprev - delta_D
    else
        Dnew = Dprev + delta_D
    end if

    # ---- 6) Clamp duty ----
    if Dnew < D_MIN: Dnew = D_MIN
    if Dnew > D_MAX: Dnew = D_MAX

    # ---- 7) Commit outputs ----
    apply_duty(Dnew)

    # ---- 8) Update state for next iteration ----
    Dprev         = Dnew
    Vprev         = Vpv
    Pprev         = Ppv
    delta_P_prev  = delta_P

end procedure
