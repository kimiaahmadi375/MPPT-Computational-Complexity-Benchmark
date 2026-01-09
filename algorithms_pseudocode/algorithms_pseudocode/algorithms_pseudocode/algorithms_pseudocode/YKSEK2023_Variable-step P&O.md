# -------------------------------------------------------------
# Yüksek2023 – Proposed Variable Step-Size P&O MPPT Algorithm
# -------------------------------------------------------------
# Hardware interface
#   measure_voltage()          → PV voltage V(k) = Vpv(k)
#   measure_inductor_current() → Inductor current Iind(k)
#   apply_duty(D)              → Set converter duty cycle
#   wait(Ts_mppt)              → MPPT sampling period

# Parameters
N        # scaling factor for control variable CV (≈ 0.001)
Kp       # proportional gain for duty update (≈ 0.01)
Dmin     # minimum duty
Dmax     # maximum duty

# -------------------------------------------------------------
# Initialization  (Initialize P(k−1), Iref, D)
# -------------------------------------------------------------
P_prev = 0                         # initial power P(k−1) assumed 0
Iref   = measure_inductor_current()
D      = D_init                    # e.g. 0.5
apply_duty(D)

# -------------------------------------------------------------
# Main MPPT loop
# -------------------------------------------------------------
loop forever:

    # 1) Measure present voltage and inductor current
    V    = measure_voltage()       # V(k)
    Iind = measure_inductor_current()   # Iind(k)

    # 2) Compute present power using reference current
    #    P(k) = V(k) * Iref
    P = V * Iref

    # 3) Control variable from scaled power difference
    #    CV = P(k) − P(k−1)
    CV = P - P_prev

    # 4) Update reference current depending on sign of CV
    if CV > 0:
        Iref = Iref + N * CV
    else:
        Iref = Iref - N * CV
    end if

    # 5) Current tracking error
    #    Error = Iref − Iind
    Error = Iref - Iind

    # 6) Update stored previous values
    P_prev = P
    V_prev = V

    # 7) Duty-cycle update
    #    D = D + Error * Kp
    D = D + Error * Kp

    # 8) Limit duty to allowable range
    D = saturate(D, Dmin, Dmax)

    # 9) Apply new duty cycle and wait for next MPPT step
    apply_duty(D)
    wait(Ts_mppt)

end loop
