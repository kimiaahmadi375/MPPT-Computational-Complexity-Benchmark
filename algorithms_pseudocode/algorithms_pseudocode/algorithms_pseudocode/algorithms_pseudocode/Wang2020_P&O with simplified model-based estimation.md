# =============================================================
# Wang2020 – Two-stage MPPT:
#   Stage-1: SMSE (simplified model-based state estimation) + OVC
#   Stage-2: α-P&O around OVC with adaptive voltage step shrinking
# =============================================================

# Interfaces / signals
measure_voltage()   -> Vpv
measure_current()   -> Ipv
set_voltage_cmd(Vcmd)   # move operating point to a commanded PV voltage (via inner loop / PID)
Ts_mppt                  # MPPT update interval

# Parameters (from paper)
Sg, Tg                   # guessed irradiance & temperature used by SMSE (constants)
alpha                    # α in (9), chosen as 0.5 in paper
ΔVmax                    # maximum allowed perturbation voltage step
ΔVmin                    # minimum allowed perturbation voltage step
ΔP_threshold             # irradiance-change detection threshold in α-P&O stage

# Internal state for α-P&O
Vcmd          # operating voltage command (OVC / Vref)
ΔVcmd         # present perturbation step size (voltage step)
V_hist[ ]     # store V(n), V(n-1), ...
I_hist[ ]     # store I(n), I(n-1), ...
P_hist[ ]     # store P(n), P(n-1), ...

# -------------------------------------------------------------
# Helper: Compute OVC (operating voltage command) from irradiance
# Using fitted polynomial (11): Vmpp0 = f(S)  (paper’s curve-fit)
# -------------------------------------------------------------
function Vovc = OVC_from_irradiance(S_est):
    # 3rd-order polynomial from Eq. (11)
    Vovc = 16.86*S_est^3 - 37.97*S_est^2 + 32.76*S_est + 55.76
    return Vovc
end function

# -------------------------------------------------------------
# Helper: SMSE subroutine (single-run SE using 2 (V,I) samples)
# (Paper simplifies WLS-SE into one estimation run, Eqs. (6)-(8))
# -------------------------------------------------------------
function S_est = SMSE_estimate_irradiance(V1,I1,V2,I2):
    # Build Jacobian Hf and mismatch ΔIf at guessed (Sg, Tg)
    #   Hf = [[∂I(V1,Sg,Tg)/∂S, ∂I(V1,Sg,Tg)/∂T],
    #         [∂I(V2,Sg,Tg)/∂S, ∂I(V2,Sg,Tg)/∂T]]
    #   ΔIf = [I_model(V1,Sg,Tg)-I1,  I_model(V2,Sg,Tg)-I2]^T
    #
    # Then one-shot update (6):
    #   Sfe = Sg + (1/D)*H22*ΔI1 - (1/D)*H12*ΔI2
    #   Tfe = Tg - (1/D)*H21*ΔI1 + (1/D)*H11*ΔI2
    #   D = H11*H22 - H12*H21
    #
    # For MPPT control, only Sfe is needed for OVC.
    compute H11,H12,H21,H22 at (Sg,Tg,V1,V2)
    compute ΔI1 = I_model(V1,Sg,Tg) - I1
    compute ΔI2 = I_model(V2,Sg,Tg) - I2
    D = H11*H22 - H12*H21
    S_est = Sg + (H22*ΔI1 - H12*ΔI2)/D
    return S_est
end function

# -------------------------------------------------------------
# MAIN MPPT PROCEDURE (matches Fig. 6)
# -------------------------------------------------------------
procedure Wang2020_SMSE_alphaPO_MPPT():

    # =========================
    # Stage-1: SMSE + OVC
    # =========================
    # (1) Measure two operating points (V1,I1) and (V2,I2)
    V1 = measure_voltage()
    I1 = measure_current()
    wait Ts_mppt
    V2 = measure_voltage()
    I2 = measure_current()

    # (2) Run SMSE (single-run estimation) to estimate irradiance S
    S_est = SMSE_estimate_irradiance(V1,I1,V2,I2)

    # (3) Compute OVC from fitted curve, and move OP to OVC
    Vcmd = OVC_from_irradiance(S_est)
    set_voltage_cmd(Vcmd)
    wait Ts_mppt

    # Initialize α-P&O memory
    V_hist[n-1] = measure_voltage()
    I_hist[n-1] = measure_current()
    P_hist[n-1] = V_hist[n-1]*I_hist[n-1]

    V_hist[n-2] = V_hist[n-1]
    I_hist[n-2] = I_hist[n-1]
    P_hist[n-2] = P_hist[n-1]

    ΔVcmd = ΔVmax

    # =========================
    # Stage-2: α-P&O loop
    # =========================
    loop forever:

        # (4) Measure current operating point and compute power
        V_hist[n] = measure_voltage()
        I_hist[n] = measure_current()
        P_hist[n] = V_hist[n]*I_hist[n]

        ΔP = P_hist[n] - P_hist[n-1]
        ΔV = V_hist[n] - V_hist[n-1]
        dI = I_hist[n] - I_hist[n-1]     # used in flowchart for step computation / detection

        # (5) Irradiance-change detection (paper: ΔP compared with threshold)
        if abs(ΔP) > ΔP_threshold then
            # Perform two additional perturbation steps (paper note) then re-run SMSE+OVC
            # (Implementation detail: do two normal α-P&O updates quickly)
            repeat 2 times:
                # use same decision logic below to update Vcmd
                # (kept minimal here)
                Vcmd = Vcmd + sign(ΔP)*ΔVcmd
                set_voltage_cmd(Vcmd)
                wait Ts_mppt
            end repeat

            # Re-run Stage-1 (SMSE + OVC) under new condition
            V1 = measure_voltage(); I1 = measure_current()
            wait Ts_mppt
            V2 = measure_voltage(); I2 = measure_current()
            S_est = SMSE_estimate_irradiance(V1,I1,V2,I2)
            Vcmd  = OVC_from_irradiance(S_est)
            set_voltage_cmd(Vcmd)
            wait Ts_mppt

            # Reset step size after relocation near new MPP
            ΔVcmd = ΔVmax

            # Reset history buffers
            V_hist[n-1] = measure_voltage()
            I_hist[n-1] = measure_current()
            P_hist[n-1] = V_hist[n-1]*I_hist[n-1]
            V_hist[n-2] = V_hist[n-1]
            I_hist[n-2] = I_hist[n-1]
            P_hist[n-2] = P_hist[n-1]
            continue
        end if

        # (6) Adaptive step shrinking rule of α-P&O:
        # apply ΔV(n) = α·ΔV(n−1) only when (10) is satisfied:
        #   P(n-2) < P(n-1) and P(n-1) > P(n)   (i.e., OP passed MPP)
        if (P_hist[n-2] < P_hist[n-1]) AND (P_hist[n-1] > P_hist[n]) then
            ΔVcmd = alpha * ΔVcmd
            if ΔVcmd < ΔVmin: ΔVcmd = ΔVmin
        else
            # otherwise keep previous ΔVcmd
            ΔVcmd = ΔVcmd
        end if

        # (7) Conventional P&O direction decision using ΔP and ΔV (as in Fig. 6)
        if ΔP > 0 then
            if ΔV > 0 then
                Vcmd = Vcmd + ΔVcmd
            else
                Vcmd = Vcmd - ΔVcmd
            end if
        else   # ΔP < 0
            if ΔV > 0 then
                Vcmd = Vcmd - ΔVcmd
            else
                Vcmd = Vcmd + ΔVcmd
            end if
        end if

        # (8) Apply the updated command and shift histories
        set_voltage_cmd(Vcmd)

        P_hist[n-2] = P_hist[n-1]
        V_hist[n-2] = V_hist[n-1]
        I_hist[n-2] = I_hist[n-1]

        P_hist[n-1] = P_hist[n]
        V_hist[n-1] = V_hist[n]
        I_hist[n-1] = I_hist[n]

        wait Ts_mppt

    end loop

end procedure
