Vpv(k), Ipv(k)          # Measured PV voltage & current at step k
Ts_mppt                 # MPPT control period

err_Tol                 # Steady-state voltage tolerance (≈ 1% Voc)
errV_Th                 # Irradiance-change voltage threshold (≈ 4% Voc)
errI_Th                 # Irradiance-change current threshold (≈ 1.5% Isc)

D_init = 0.5            # Initial duty cycle
Dmin, Dmax              # Duty-cycle limits

# FLC parameters: membership functions & rule base
FLC_params


steady   = 0            # 0 = not in steady state, 1 = steady state at MPP
ident    = 0            # steady-state identification stage {0,1,2}
count    = 0            # perturbation counter

V_k      = 0            # V(k)
V_k_1    = 0            # V(k-1)
V_k_2    = 0            # V(k-2)

P_k      = 0            # P(k)
P_k_1    = 0            # P(k-1)

Vmpp     = 0            # Stored MPP voltage when steady=1
Impp     = 0            # Stored MPP current when steady=1

D        = D_init       # Present duty cycle


function ΔD = FLC_step(V_k, V_k_1, P_k, P_k_1):

    ΔV = V_k - V_k_1
    ΔP = P_k - P_k_1

    if ΔV ≠ 0:
        slope_abs = |ΔP / ΔV|
    else:
        slope_abs = very_large_value    # treat as far from MPP

    # FLC with inputs: slope_abs and ΔP, output: ΔD
    ΔD = fuzzy_inference(FLC_params, slope_abs, ΔP)

    return ΔD
end function


function D_new = PAndO_update(D_old, ΔD, V_k, V_k_1, P_k, P_k_1):

    ΔV = V_k - V_k_1
    ΔP = P_k - P_k_1

    if ΔP > 0:
        if ΔV > 0:
            D_new = D_old + ΔD
        else:
            D_new = D_old - ΔD
        end if
    else:   # ΔP < 0
        if ΔV > 0:
            D_new = D_old - ΔD
        else:
            D_new = D_old + ΔD
        end if
    end if

    D_new = saturate(D_new, Dmin, Dmax)
    return D_new
end function


loop forever:

    # --- 1) Measure present PV values ---
    V_k = measure_voltage()
    I_k = measure_current()
    P_k = V_k * I_k


    # =======================================================
    # 2) If already in steady state → only check irradiance change
    # =======================================================
    if steady == 1 then

        # Region (4): sensorless irradiance-change detection
        Err_V = |Vmpp - V_k|
        Err_I = |Impp - I_k|

        if (Err_V > errV_Th) AND (Err_I > errI_Th) then
            # Irradiance change detected → restart tracking
            steady = 0
            ident  = 0

            ΔD = FLC_step(V_k, V_k_1, P_k, P_k_1)
            D  = PAndO_update(D, ΔD, V_k, V_k_1, P_k, P_k_1)
            apply_duty_cycle(D)
        else
            # Stay at stored MPP, no perturbation
            apply_duty_cycle(D)     # D unchanged
        end if


    # =======================================================
    # 3) Not in steady state → Adaptive P&O + SS detection
    # =======================================================
    else

        # Region (1/2/3): three-step steady-state identification
        Err = |V_k - V_k_2|

        if (Err < err_Tol) AND (V_k > V_k_1) then

            if ident == 0 then
                # Step 1 passed
                ident = 1
                count = 2

            elseif ident == 1 then
                # Step 2 passed
                ident = 2
                count = 4

            elseif ident == 2 then
                # Step 3 passed → Steady state confirmed
                steady = 1
                Vmpp   = V_k
                Impp   = I_k
                # Store present MPP power if needed:
                # Pmpp = Vmpp * Impp
            end if

        else
            # Steady-state pattern not satisfied → normal tracking
            ident = 0          # reset identification sequence

            # FLC computes variable step size ΔD
            ΔD = FLC_step(V_k, V_k_1, P_k, P_k_1)

            # P&O updates duty cycle direction using ΔD
            D  = PAndO_update(D, ΔD, V_k, V_k_1, P_k, P_k_1)
            apply_duty_cycle(D)

            count = count + 1
        end if

    end if   # steady check


    # --- 4) Prepare for next iteration ---
    V_k_2 = V_k_1
    V_k_1 = V_k

    P_k_1 = P_k

    wait(Ts_mppt)
end loop
