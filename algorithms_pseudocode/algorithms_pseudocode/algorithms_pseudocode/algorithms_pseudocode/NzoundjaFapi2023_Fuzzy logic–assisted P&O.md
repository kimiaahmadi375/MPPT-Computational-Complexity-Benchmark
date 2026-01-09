measure_voltage() → Vpv(k)
measure_current() → Ipv(k)
apply_duty(D)
Ts_mppt  → MPPT sampling period

# --- Initialization ---
D_prev = D_initial          # duty cycle (start near 0.5 or design value)
apply_duty(D_prev)
wait Ts_mppt

V_prev = measure_voltage()
I_prev = measure_current()
P_prev = V_prev * I_prev

loop forever:

    # 1 — Measure PV quantities
    V = measure_voltage()
    I = measure_current()
    P = V * I

    # 2 — Compute slopes
    ΔV = V - V_prev
    ΔP = P - P_prev

    # 3 — Fuzzy logic: variable step from ΔP, ΔV
    ΔD_fuzzy = FLC(ΔP, ΔV)

    # 4 — Improved P&O decision
    if ΔP > 0:                       # Power increasing
        if ΔV > 0:
            D_new = D_prev + ΔD_fuzzy   # move right on P–V curve
        else:
            D_new = D_prev - ΔD_fuzzy   # move left
    else:                            # Power decreasing
        if ΔV > 0:
            D_new = D_prev - ΔD_fuzzy
        else:
            D_new = D_prev + ΔD_fuzzy
    end if

    # 5 — Saturate duty cycle
    D_new = saturate(D_new, Dmin, Dmax)

    # 6 — Apply updated duty cycle
    apply_duty(D_new)

    # 7 — Update buffers for next iteration
    D_prev = D_new
    V_prev = V
    I_prev = I
    P_prev = P

    wait(Ts_mppt)

end loop
