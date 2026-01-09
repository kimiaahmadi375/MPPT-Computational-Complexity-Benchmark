# =========================================================
# Swaminathan 2022 – Fixed Zone P&O (FZPO) MPPT Algorithm
# Control parameter CP can be duty ratio, phase shift, etc.
# =========================================================

# Hardware I/O
#   measure_voltage()  -> Vpv  (PV array voltage)
#   measure_current()  -> Ipv  (PV array current)
#   apply_CP(CP)       -> write control parameter to converter
#   wait(Ts_mppt)      -> MPPT sampling period

# -------------------------------------------------------------------
# A) OFFLINE DESIGN STAGE  (runs once, not part of real-time load)
# -------------------------------------------------------------------
# Using PV model + panel datasheet, compute:
#   - Zone boundary equations B12, B23, B34, B45   (eqs. (6)–(9))
#   - Step-size linear laws for Zones 1,2,4,5      (eq. (10))
#   - Fixed small step for Zone 3 (near MPP)
# :contentReference[oaicite:1]{index=1}

procedure FZPO_Offline_Design():

    # 1) Define two “extreme” operating conditions
    #    (max irradiance/min temp, min irradiance/max temp)
    cond1 = {G = G_max, T = T_min}
    cond2 = {G = G_min, T = T_max}

    # 2) For each condition, generate P–V curve from PV model
    PV_curve1 = simulate_PV_curve(cond1)
    PV_curve2 = simulate_PV_curve(cond2)

    # 3) From PV model, compute slope dP/dV vs V for both conditions
    slope1 = compute_dP_dV_vs_V(PV_curve1)
    slope2 = compute_dP_dV_vs_V(PV_curve2)

    # 4) Using linear fitting on slope–V curves, find 4 boundary points
    #    (voltages where conductance changes), giving:
    #    VB12(max/min), VB23(max/min), VB34(max/min), VB45(max/min)
    [VB12_max, VB23_max, VB34_max, VB45_max] = find_boundary_voltages(slope1)
    [VB12_min, VB23_min, VB34_min, VB45_min] = find_boundary_voltages(slope2)

    # 5) Fit straight lines for boundaries (eqs. (6)–(9)):
    #    VB23(g) = m23 * Ipv + c23
    #    VB34(g) = m34 * Ipv + c34
    #    VB12(g) = VB23(g) + A1
    #    VB45(g) = VB34(g) + A2
    (m23, c23, A1) = fit_boundary_pair(VB23_max, VB23_min,
                                       VB12_max, VB12_min)
    (m34, c34, A2) = fit_boundary_pair(VB34_max, VB34_min,
                                       VB45_max, VB45_min)

    store_constants(m23, c23, m34, c34, A1, A2)

    # 6) Choose desired min/max step sizes per zone (e.g. :contentReference[oaicite:2]{index=2})
    #    Zones 1 & 5: step_min = 6%, step_max = 8%
    #    Zones 2 & 4: step_min = 2%, step_max = 6%
    #    Zone 3: fixed small step, e.g. step_Z3 = 2%

    # 7) For each zone z ∈ {1,2,4,5}, use eq. (10):
    #       step_z = m_step_z * (Vn - VB_target_z) + c_step_z
    #    Solve for m_step_z, c_step_z from chosen step_min/step_max
    (m_step_1, c_step_1) = design_step_law_for_zone1()
    (m_step_2, c_step_2) = design_step_law_for_zone2()
    (m_step_4, c_step_4) = design_step_law_for_zone4()
    (m_step_5, c_step_5) = design_step_law_for_zone5()

    step_Z3 = choose_fixed_small_step()

    store_step_laws(m_step_1, c_step_1,
                    m_step_2, c_step_2,
                    m_step_4, c_step_4,
                    m_step_5, c_step_5,
                    step_Z3)

end procedure


# -------------------------------------------------------------------
# B) ONLINE MPPT STAGE – FZPO DIRECT-DUTY (or phase-shift) CONTROL
# -------------------------------------------------------------------

# Design-time constants (from Offline_Design)
m23, c23, m34, c34, A1, A2
m_step_1, c_step_1, m_step_2, c_step_2
m_step_4, c_step_4, m_step_5, c_step_5
step_Z3                        # fixed small step in Zone 3

CP_min, CP_max                 # hardware limits for control parameter
CP                             # actual control parameter (duty/phase shift)

# For Zone-3 conventional P&O behaviour
V_prev = measure_voltage()
I_prev = measure_current()
P_prev = V_prev * I_prev

# Initial control parameter (e.g. from nominal Vmpp/Vout)
CP = CP_init
apply_CP(CP)
wait(Ts_mppt)


procedure FZPO_MPPT_Loop():

    loop forever:

        # ---- Step 1: measure present operating point ----
        Vn = measure_voltage()
        In = measure_current()
        Pn = Vn * In

        # ---- Step 2: compute zone-boundary voltages for this current In ----
        VB23 = m23 * In + c23            # boundary B23 (Zone2/3)   (eq. 7)
        VB34 = m34 * In + c34            # boundary B34 (Zone3/4)   (eq. 8)
        VB12 = VB23 + A1                 # boundary B12 (Zone1/2)   (eq. 6)
        VB45 = VB34 + A2                 # boundary B45 (Zone4/5)   (eq. 9)

        # ---- Step 3: identify present operating zone (Fig. 3) ----
        if      Vn < VB12:   zone = 1
        elseif  Vn < VB23:   zone = 2
        elseif  Vn < VB34:   zone = 3
        elseif  Vn < VB45:   zone = 4
        else                 zone = 5
        end if

        # ---- Step 4: compute perturbation step "step" ----
        if zone == 3:
            # Near MPP: small fixed step, P&O-like behaviour
            ΔP = Pn - P_prev
            ΔV = Vn - V_prev

            if ΔP > 0:
                if ΔV > 0:
                    step = +step_Z3
                else
                    step = -step_Z3
                end if
            else:
                if ΔV > 0:
                    step = -step_Z3
                else
                    step = +step_Z3
                end if
            end if

        else:
            # Zones 1,2,4,5 – step from linear law (eq. 10)
            if zone == 1:
                VB_target = VB12
                step = m_step_1 * (Vn - VB_target) + c_step_1
            elseif zone == 2:
                VB_target = VB23
                step = m_step_2 * (Vn - VB_target) + c_step_2
            elseif zone == 4:
                VB_target = VB34
                step = m_step_4 * (Vn - VB_target) + c_step_4
            else  # zone == 5
                VB_target = VB45
                step = m_step_5 * (Vn - VB_target) + c_step_5
            end if

            # NOTE: sign of (Vn − VB_target) makes perturbation
            # move toward Zone 3 → natural drift-free behaviour.
        end if

        # ---- Step 5: update control parameter CP ----
        CP = CP + step

        # Enforce converter limits
        if CP < CP_min:  CP = CP_min
        if CP > CP_max:  CP = CP_max

        apply_CP(CP)

        # ---- Step 6: store previous sample and wait next period ----
        V_prev = Vn
        I_prev = In
        P_prev = Pn

        wait(Ts_mppt)

    end loop

end procedure
