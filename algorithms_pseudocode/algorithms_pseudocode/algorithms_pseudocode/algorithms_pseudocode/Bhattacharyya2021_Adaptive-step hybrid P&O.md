# -------------------------------------------------------------
# SOFT-MPPT (Bhattacharyya 2021)
# Works with either P&O or InC tracking core (Direct Duty-Ratio).
# -------------------------------------------------------------
# Low-level interfaces
#    V = measure_voltage()
#    I = measure_current()
#    apply_duty(D)
#    wait(Ts_mppt)

# ---- Tunable parameters (taken from paper) ----
del_D_min   = 0.02          # minimum perturbation step (ΔDmin)
del_D_max   = 0.07          # maximum perturbation step (ΔDmax)
err_Tol     = 0.01*Voc      # steady-state voltage tolerance (~1% Voc)
eV_Th       = 0.04*Voc      # voltage threshold for irradiance change (~4% Voc)
eI_Th       = 0.015*Isc     # current threshold for irradiance change (~1.5% Isc)
Ts_mppt     = 1 / 25        # perturbation frequency 25 Hz in paper

# ---- State variables ----
steady  = 0                 # 0 = not in steady state, 1 = steady at MPP
ident   = 0                 # identification stage {0,1,2}
count   = 0                 # perturbation counter after ident ≠ 0

Vmpp    = 0                 # stored steady-state MPP voltage
Impp    = 0                 # stored steady-state MPP current

# history of PV voltage / power
V_k     = 0
V_k_1   = 0                 # V(k-1)
V_k_2   = 0                 # V(k-2)
P_k     = 0
P_k_1   = 0

# duty-cycle state
D       = 0.5               # initial duty ratio

# -------------------------------------------------------------
# Helper: compute adaptive step size ΔD(k)
# -------------------------------------------------------------
function compute_step_size(V_k, V_k_1, P_k, P_k_1, I_k):

    dV = V_k - V_k_1
    dP = P_k - P_k_1

    if dV != 0:
        slope = dP / dV
    else:
        slope = 0      # treat as 0; ΔD will be ≈ del_D_min

    # Modulating factor M(k) ≈ 1–2.5% of 1/I(k)
    M_k = k_select_between(0.01, 0.025) * (1 / I_k)

    # Adaptive step (Eq. (1))
    del_D = del_D_min + M_k * slope

    # Bound step size
    if del_D > del_D_max: del_D = del_D_max
    if del_D < -del_D_max: del_D = -del_D_max

    return del_D
end function

# -------------------------------------------------------------
# Helper: one DDR-P&O (or InC) update using adaptive ΔD
# -------------------------------------------------------------
function D_new = Adaptive_MPPT_step(V_k, V_k_1, I_k, I_k_1, P_k, P_k_1, D_old, mode):

    del_D = compute_step_size(V_k, V_k_1, P_k, P_k_1, I_k)

    if mode == "P&O":

        dV = V_k - V_k_1
        dP = P_k - P_k_1

        if dP > 0:
            if dV > 0:
                D_new = D_old + del_D
            else:
                D_new = D_old - del_D
            end if
        else:            # dP < 0
            if dV > 0:
                D_new = D_old - del_D
            else:
                D_new = D_old + del_D
            end if
        end if

    elseif mode == "InC":

        dV = V_k - V_k_1
        dI = I_k - I_k_1

        # Incremental-conductance decision
        if dV == 0:
            D_new = D_old       # avoid divide-by-zero
        else
            inc_cond = dI / dV
            inst_cond = - I_k / V_k

            if inc_cond > inst_cond:      # left of MPP → increase Vpv → reduce D (boost)
                D_new = D_old - del_D
            elseif inc_cond < inst_cond:  # right of MPP → decrease Vpv → increase D
                D_new = D_old + del_D
            else                          # at MPP
                D_new = D_old
            end if
        end if

    end if

    # saturate duty
    if D_new > 1:   D_new = 1
    if D_new < 0:   D_new = 0

    return D_new
end function

# -------------------------------------------------------------
# MAIN: SOFT-MPPT supervisor around adaptive P&O/InC core
# -------------------------------------------------------------
procedure SOFT_MPPT(mode):        # mode ∈ {"P&O", "InC"}

    # initial sensing
    V_k   = measure_voltage()
    I_k   = measure_current()
    P_k   = V_k * I_k

    V_k_1 = V_k
    V_k_2 = V_k
    P_k_1 = P_k

    apply_duty(D)
    wait(Ts_mppt)

    loop forever:

        # ---- 1) Sense present PV point ----
        V_k = measure_voltage()
        I_k = measure_current()
        P_k = V_k * I_k

        # =====================================================
        # 2) If steady-state already reached → only monitor for
        #    irradiance / operating-condition change
        # =====================================================
        if steady == 1 then

            e_V = Vmpp - V_k
            e_I = Impp - I_k

            if (abs(e_V) > eV_Th) and (abs(e_I) > eI_Th) then
                # operating conditions changed → restart tracking
                steady = 0
                ident  = 0
                count  = 0
                # fall through to tracking section below
            else
                # hold at stored MPP; no artificial perturbation
                apply_duty(D)
                # refresh histories for monitoring only
                V_k_2 = V_k_1
                V_k_1 = V_k
                P_k_1 = P_k
                wait(Ts_mppt)
                continue    # next iteration
            end if
        end if

        # =====================================================
        # 3) Not in steady state → check 3-level pattern using
        #    V(k), V(k-1), V(k-2) (Fig. 3)
        # =====================================================
        err = abs(V_k - V_k_2)

        if ident == 0 then
            # First detection window (A-B-C)
            if (err < err_Tol) and (V_k_1 > V_k) then
                ident = 1
                count = 0
            end if

        elseif ident == 1 then
            # Second window (C-D-E) after two more perturbations
            if count == 2 then
                if (err < err_Tol) and (V_k_1 < V_k) then
                    ident = 2
                    count = 0
                end if
            end if

        elseif ident == 2 then
            # Third window (E-F-G) after two more perturbations
            if count == 4 then
                if (err < err_Tol) and (V_k_1 > V_k) then
                    # Steady state confirmed
                    steady = 1
                    Vmpp   = V_k
                    Impp   = I_k
                    # lock at this MPP; no adaptive call this cycle
                    D = D        # unchanged
                    apply_duty(D)

                    # update histories and go to next iteration
                    V_k_2 = V_k_1
                    V_k_1 = V_k
                    P_k_1 = P_k
                    wait(Ts_mppt)
                    continue
                end if
            end if
        end if

        # =====================================================
        # 4) Still tracking → call adaptive P&O/InC step
        # =====================================================
        D = Adaptive_MPPT_step(V_k, V_k_1, I_k, I_k_1, P_k, P_k_1, D, mode)

        apply_duty(D)

        # update counters for steady-state identification
        if ident > 0:
            count = count + 1
        end if

        # shift histories
        V_k_2 = V_k_1
        V_k_1 = V_k

        I_k_1 = I_k
        P_k_1 = P_k

        wait(Ts_mppt)

    end loop

end procedure
