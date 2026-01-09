# -------------------------------------------------------------
# Jabbar2023 – Modified P&O MPPT with:
#   • Fuzzy-logic variable perturbation step M(ΔP, |ΔV|)
#   • Drift-avoidance using ΔI
#   • 4-sample steady-state oscillation detector
#   • Irradiance-change detector + reinitialization around new MPP
# -------------------------------------------------------------
# Interfaces
#   Vpv = measure_voltage()
#   Ipv = measure_current()
#   apply_duty(D)
#   wait(Ts_mppt)                     # MPPT sampling period

# -------------------------------------------------------------
# Helper blocks
# -------------------------------------------------------------

function M = FLC_step(ΔP, ΔV_abs):
    # Mamdani FLC with inputs:
    #   x1 = ΔP, x2 = |ΔV|
    # Output:
    #   M   = adaptive duty perturbation step
    # Linguistic sets & rules as in Table I and Figs. 3–5
    M = fuzzy_inference(ΔP, ΔV_abs)
    return M
end function


function [Vmpp_hat, Impp_hat] = estimate_MPP_from_Isc(Isc, T):
    # Uses PV model Eqs. (3)–(4) to estimate irradiance S and Vmpp
    # S   = f(Isc, T)                     (Eq. 3)
    # Vmpp_hat = f(S, T, module params)  (Eq. 4)
    # Impp_hat ≈ current at MPP for (S, T)
    # Here we treat the detailed single-diode maths as a black box.
    S = compute_irradiance_from_Isc(Isc, T)
    Vmpp_hat, Impp_hat = compute_MPP_from_S_T(S, T)
    return [Vmpp_hat, Impp_hat]
end function


procedure reinitialize_for_new_irradiance():

    # --- 1) Force operating point near short-circuit to sense new irradiance
    D = 0.8
    apply_duty(D)
    wait(Ts_mppt)

    Isc_new = measure_current()
    T_new   = measure_temperature()         # or fixed 25°C in simulations

    # --- 2) Estimate new MPP (Vmpp, Impp) from PV model (Eqs. 3–4)
    [Vmpp_hat, Impp_hat] = estimate_MPP_from_Isc(Isc_new, T_new)

    # --- 3) Compute current at Vmpp_hat (optional – paper sets Ipv = Impp, Vpv = Vmpp)
    #       and store corresponding duty as D_at_Vmpp
    D_at_Vmpp = duty_from_desired_voltage(Vmpp_hat)   # via converter model

    # --- 4) Compare starting powers at D = 0.5 and at D_at_Vmpp
    # Power at D = 0.5
    D_test1 = 0.5
    apply_duty(D_test1)
    wait(Ts_mppt)
    P1 = measure_voltage() * measure_current()

    # Power at D_at_Vmpp
    D_test2 = D_at_Vmpp
    apply_duty(D_test2)
    wait(Ts_mppt)
    P2 = measure_voltage() * measure_current()

    if P2 > P1:
        D_init = D_test2
    else:
        D_init = D_test1
    end if

    return D_init
end procedure


# -------------------------------------------------------------
# Main online MPPT algorithm
# -------------------------------------------------------------

procedure Jabbar2023_Fuzzy_PO_MPPT():

    # ---- Global constants / thresholds (from paper) ----
    F      = 0.02          # constant used in irradiance estimation
    Pch    = 0.0           # helper for irradiance-change block
    Dmin   = 0.0           # hardware duty limits
    Dmax_hw = 1.0

    # Steady-state / irradiance-change thresholds
    eps_irr = 0.05         # |Pmax − Ppv| / Pmax ≤ 0.05 ⇒ no irradiance change

    # ---- Initial conditions ----
    Vold = 0.0
    Pold = 0.0
    Iold = 0.0

    Dold = 0.5            # starting duty cycle for main P&O loop
    Dmax = 0.0            # duty corresponding to Pmax in 4-sample window

    k    = 0              # index for 4-sample window (1..4)
    Sum  = 0              # Σ sign(ΔP/ΔV) over last 4 samples
    DS[1..4] = {0,0,0,0}  # stored duties for steady-state test
    PS[1..4] = {0,0,0,0}  # corresponding powers

    apply_duty(Dold)
    wait(Ts_mppt)

    # Measure initial point
    Vold = measure_voltage()
    Iold = measure_current()
    Pold = Vold * Iold

    # --------------- Main MPPT loop ---------------
    loop forever:

        # ---- 1) Acquire present PV point ----
        Vpv = measure_voltage()
        Ipv = measure_current()
        P   = Vpv * Ipv

        ΔV = Vpv - Vold
        ΔP = P   - Pold
        ΔI = Ipv - Iold

        # ---- 2) Fuzzy logic: adaptive perturbation step M ----
        M = FLC_step(ΔP, abs(ΔV))

        # ---- 3) Modified P&O with drift-avoidance using ΔI ----
        if ΔP > 0 then
            if ΔV > 0 then
                # Region where classical P&O suffers from drift
                if ΔI > 0 then
                    # Power increased due mainly to irradiance rise → avoid drift
                    D = Dold - M
                else
                    # Power increase due to perturbation → keep normal direction
                    D = Dold + M
                end if
            else    # ΔV < 0
                D = Dold - M
            end if
        else        # ΔP ≤ 0
            if ΔV > 0 then
                D = Dold - M
            else    # ΔV < 0
                D = Dold + M
            end if
        end if

        # Saturate duty to converter limits
        D = saturate(D, Dmin, Dmax_hw)

        # ---- 4) Update 4-sample steady-state detector ----
        k = k + 1
        if (0 < k) and (k ≤ 4) then
            DS[k] = D
            PS[k] = P

            if ΔV ≠ 0 then
                Sigma = sign(ΔP / ΔV)   # +1, 0, or −1
            else
                Sigma = 0               # avoid division by zero
            end if
            Sum = Sum + Sigma
        end if

        # ---- 5) When 4 samples collected, test for steady-state ----
        if k == 4 then

            if Sum == 0 then
                # Steady-state oscillation pattern detected (Eq. 5)

                # Find duty that gave highest power among the 4 samples
                Pmax = PS[1]
                idx  = 1
                for j = 2 .. 4:
                    if PS[j] > Pmax:
                        Pmax = PS[j]
                        idx  = j
                    end if
                end for
                Dmax = DS[idx]

                # ---- 5a) Irradiance-change test (Eq. 6) ----
                if abs(Pmax - P) / Pmax ≤ eps_irr then
                    # True steady-state around MPP → stop fuzzy perturbations
                    D = Dmax
                    M = 0.0          # no further perturbation
                else
                    # Irradiance has changed → reinitialize around new MPP
                    Pch = Ipv * Ipv  # helper for irradiance estimation
                    D_init = reinitialize_for_new_irradiance()

                    # Reset history and restart P&O from new initial duty
                    Dold = D_init
                    Vold = Vpv
                    Iold = Ipv
                    Pold = P

                    k   = 0
                    Sum = 0

                    apply_duty(Dold)
                    wait(Ts_mppt)
                    continue   # start next MPPT iteration with new operating point
                end if
            else
                # Not in steady oscillation yet → reset window and continue
                k   = 0
                Sum = 0
            end if
        end if

        # ---- 6) Commit duty & prepare for next iteration ----
        apply_duty(D)

        Vold = Vpv
        Iold = Ipv
        Pold = P
        Dold = D

        wait(Ts_mppt)

    end loop

end procedure
