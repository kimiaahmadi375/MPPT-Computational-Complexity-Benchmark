# -------------------------------------------------------------
# Ahmed2023 – Adaptive P&O With Enhanced Skipping Feature
# -------------------------------------------------------------
# Interfaces
#   measure_voltage()  → Vpv(k)
#   measure_current()  → Ipv(k)
#   apply_duty(D)      → converter duty input (buck–boost)
#   Vref_to_duty(Vref) → inner PI loop converts Vref to duty
#   wait(Ts)           → MPPT sampling period (0.005 s used)
#
# Symbols (paper notation)
#   Voc_mod    : module open-circuit voltage
#   Voc_array  : array open-circuit voltage = M * Voc_mod
#   M          : number of series modules in the string
# -------------------------------------------------------------

# -------------------- Global parameters ----------------------

ΔV_scan      = 3.0      # Vref scanning step used in examples
ΔV_PO        = 0.3      # small step for conventional P&O subroute
Vstart_frac  = 0.45     # initial Vref = 0.45*Voc_mod
Vmax_frac    = 0.9      # Vmax = 0.9*Voc_array  (Eq. 9)
reinit_thr   = 0.05     # 5% power deviation for re-initialization (Eq. 11)

# For SSO reduction in P&O (Section II-C)
ODP_window   = 5        # evaluate oscillation detection parameter every 5 iters
ODP_thr      = 3        # |ODP| ≤ 3 ⇒ assume GMPP reached

# -------------------------------------------------------------
# Helper: read PV power
# -------------------------------------------------------------
function read_PV():
    V = measure_voltage()
    I = measure_current()
    P = V * I
    return (V, I, P)
end function


# -------------------------------------------------------------
# Conventional P&O subroute with SSO-reduction (Fig. 2 + Sec. II-C)
# Starts from current Vref and returns tracked GMPP (Vgmp, Pgmp)
# -------------------------------------------------------------
function [Vgmp, Pgmp] = PO_subroute_with_SSO(Vref_start):

    Vref = Vref_start
    ODP  = 0           # oscillation detection parameter
    iter = 0

    # Initial sample at Vref
    set_reference_voltage(Vref)
    wait(Ts)
    (V_prev, I_prev, P_prev) = read_PV()

    Pgmp = P_prev
    Vgmp = V_prev

    loop:

        # 1) Perturb reference voltage
        Vref = Vref + ΔV_PO   # sign will be flipped by hill-climbing rule

        set_reference_voltage(Vref)
        wait(Ts)
        (V, I, P) = read_PV()

        dP = P - P_prev
        dV = V - V_prev

        # 2) Standard hill-climbing decision
        if dP > 0:
            if dV > 0:
                Vref = Vref + ΔV_PO
                ODP  = ODP + 1
            else:
                Vref = Vref - ΔV_PO
                ODP  = ODP - 1
            end if
        else:
            if dV > 0:
                Vref = Vref - ΔV_PO
                ODP  = ODP - 1
            else:
                Vref = Vref + ΔV_PO
                ODP  = ODP + 1
            end if
        end if

        # 3) Apply updated Vref
        set_reference_voltage(Vref)

        # 4) Update best power
        if P > Pgmp:
            Pgmp = P
            Vgmp = V
        end if

        # 5) Update previous sample
        P_prev = P
        V_prev = V

        iter = iter + 1

        # 6) Every ODP_window iterations, check for oscillation only
        if (iter mod ODP_window) == 0:

            if |ODP| <= ODP_thr:
                # GMPP assumed reached → stop perturbation, hold Vref
                set_reference_voltage(Vgmp)
                return [Vgmp, Pgmp]
            else:
                # Still away from GMPP → continue P&O, reset ODP
                ODP = 0
            end if
        end if

        wait(Ts)

    end loop
end function


# -------------------------------------------------------------
# Main proposed algorithm (Fig. 4 – main subroute)
# -------------------------------------------------------------
procedure Adaptive_PO_Skipping_MPPT(M, Voc_mod):

    Voc_array = M * Voc_mod
    Vmax      = Vmax_frac * Voc_array     # Eq. (9)

    # ----------------- 1. Initialization (block 1) -------------
    Vref = Vstart_frac * Voc_mod          # 0.45*Voc_mod
    set_reference_voltage(Vref)

    # Track the first (leftmost) peak with conventional P&O
    [Vmpp1, Pbest] = PO_subroute_with_SSO(Vref)   # block 2–3
    Vbest  = Vmpp1

    # N is the module index used in Eqs. (4) and (8) (block 5)
    N = M

    # ----------------- 2. Skipping & scanning loop --------------
    while TRUE:

        # ---- Compute lower & upper scanning ranges for next peak ----
        # Using generalized relations (Eqs. 4 & 8)
        ΔV_lower = 0.83 * Voc_mod + (M - N) * 1.005 * Voc_mod
        ΔV_upper = 1.18 * Voc_mod + (M - N) * 1.005 * Voc_mod

        # positions on P–V curve, measured from first peak
        Vscan_L = Vmpp1 + ΔV_lower
        Vscan_U = Vmpp1 + ΔV_upper

        # Clamp to Vmax
        if Vscan_U > Vmax:
            Vscan_U = Vmax
        end if

        # If lower bound already beyond Vmax → scanning finished
        if Vscan_L >= Vmax:
            break
        end if

        # Start scanning in this window: set initial Vref (block 6, 7)
        Vref = Vscan_L
        set_reference_voltage(Vref)
        wait(Ts)
        (V_prev, I_prev, P_prev) = read_PV()

        # Update minimum current bound as function of current best power (Eq. 10)
        Imin = Pbest / Vmax

        # ------------- Scan from Vscan_L → Vscan_U (blocks 7–13) ------------
        while Vref <= Vscan_U:

            # increase reference voltage by ΔV_scan (block 8)
            Vref = Vref + ΔV_scan
            set_reference_voltage(Vref)
            wait(Ts)

            (V, I, P) = read_PV()

            # (block 10–11) update global best if needed
            if P > Pbest:
                Pbest = P
                Vbest = V
            end if

            # (block 12) if power decreased vs previous sample, peak in this
            # zone is already passed → jump to next zone (skip block 13)
            if P < P_prev:
                break   # exit this scanning window, go to next N
            end if

            # (block 14–15) scanning termination on I < Imin or V > Vmax
            if (I < Imin) or (V > Vmax):
                # No chance of further GMPP on right side
                Vscan_finished = TRUE
                exit both loops   # will jump to final P&O
            end if

            P_prev = P
        end while

        # ------------- Prepare for next window -----------------
        N = N - 1            # move to next possible peak (block 16–17)

        if (N < 2):
            # All candidate windows examined
            break
        end if

        # loop back to compute next (ΔV_lower, ΔV_upper) and repeat skipping
    end while

    # ----------------- 3. Final GMPP tracking ------------------
    # After scanning, move Vref close to voltage with best power (block 18)
    set_reference_voltage(Vbest)

    # Track GMPP using P&O with SSO reduction (block 19)
    [Vgmp, Pgmp] = PO_subroute_with_SSO(Vbest)

    # ----------------- 4. Re-initialization check --------------
    loop forever:

        wait(Ts)
        (V_now, I_now, P_now) = read_PV()

        # Eq. (11): if |Ppv(k) − Pbest| / Pbest < 0.05 ⇒ still optimal
        if |P_now - Pgmp| / Pgmp < reinit_thr:
            # keep reference voltage at Vgmp (no change)
            set_reference_voltage(Vgmp)
        else:
            # irradiance pattern changed ⇒ re-run whole process
            Adaptive_PO_Skipping_MPPT(M, Voc_mod)
            return
        end if

    end loop

end procedure
