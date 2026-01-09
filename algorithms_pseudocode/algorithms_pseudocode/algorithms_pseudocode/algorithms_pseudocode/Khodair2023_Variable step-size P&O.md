α = α_initial
Offset0 = predefined_constant
V_old = measure_voltage()
I_old = measure_current()
P_old = V_old * I_old

loop forever:

    V = measure_voltage()
    I = measure_current()
    P = V * I

    ΔP = P - P_old
    ΔV = V - V_old

    Offset = Offset0 * abs(ΔP)          // as in Fig. 3

    if ΔP == 0:
        α = α
    else if ΔP > 0:
        if ΔV > 0:
            α = α + Offset
        else:
            α = α - Offset
    else:   // ΔP < 0
        if ΔV > 0:
            α = α - Offset
        else:
            α = α + Offset
    end if

    α = clamp(α, 0, 1)                 // optional implementation constraint
    apply_duty_cycle(α)

    V_old = V
    P_old = P
end loop
