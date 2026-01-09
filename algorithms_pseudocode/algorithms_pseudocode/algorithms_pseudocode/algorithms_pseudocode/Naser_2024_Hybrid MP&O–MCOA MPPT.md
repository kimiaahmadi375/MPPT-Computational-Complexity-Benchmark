(measure_voltage(), measure_current()     # PV measurements
apply_duty(D)                            # DC–DC converter interface
Ts_mppt                                  # MPPT sampling period (0.05 s)

# General converter limits
Dmin, Dmax                               # Duty-cycle bounds

# Detection & operating mode
D_init_detect    = 0.85                  # Duty used for detection & Isc measurement
USC_PSC_I_thresh = 0.05                  # |IMPP - IPV| / IMPP threshold (Eqs. 10–11)

# MP&O (USC mode)
ΔDmin                 # minimum duty perturbation step
m_trap                # trapezoidal-scale factor (1–2%) (Eqs. 15–16)
P_tol_small  = 0.1    # |ΔP| < 0.1 W steady-state band (Eq. 17)
irr_USC_thresh = 0.05 # |Pmpp - Ppv|/Pmpp > 0.05 ⇒ irradiance change (Eq. 19)

# MCOA (PSC mode)
PopSize      = 4
D_coot_init[1..4] = [0.2, 0.4, 0.6, 0.8] # initial coot positions
Iter_max_MCOA      # maximum iterations
z_prob             # probability threshold used in COA branching
W, L, TF, gBest    # MCOA internal parameters per Eqs. (23)–(28)
ϵ_PGbest           # convergence tolerance on PGbest (Eq. 21)

# Post-tracking irradiance monitoring (both modes)
irr_global_thresh = 0.05                 # |PMPP - PPV(t)|/PMPP > 0.05 (Eq. 29)


function mode = Detect_Condition_USC_or_PSC():

    # 1) Set D = 0.85 and measure array short-circuit behaviour
    apply_duty(D_init_detect)
    wait Ts_mppt

    Isc_array = measure_current()
    Vpv       = measure_voltage()

    # 2) Compute predicted MPP current & duty from PV model
    IMPP = compute_IMPP_from_model(Isc_array)             # Eq. (5) / model
    Zout = compute_load_impedance()                       # from circuit
    DMPP = compute_DMPP_from_impedance(VMPP, IMPP, Zout)  # Eq. (8)–(9)

    # 3) Apply DMPP and measure actual current
    apply_duty(DMPP)
    wait Ts_mppt
    Ipv = measure_current()

    mismatch = |IMPP - Ipv| / IMPP

    if mismatch < USC_PSC_I_thresh:
        mode = "USC"
    else:
        mode = "PSC"
    end if

    return mode
end function


function [D_mpp, P_mpp] = MPO_USC():

    # Initialization for P&O loop
    D      = D_init_detect
    apply_duty(D)
    wait Ts_mppt

    V_prev = measure_voltage()
    I_prev = measure_current()
    P_prev = V_prev * I_prev

    Pmpp   = P_prev
    Dmpp   = D

    loop:

        # Measure present point
        V = measure_voltage()
        I = measure_current()
        P = V * I

        ΔP = P - P_prev

        # --- Steady-state stopping condition (Eq. 17) ---
        if (ΔP < 0) OR (|ΔP| < P_tol_small):
            D = Dmpp                      # hold duty at best MPP duty
            apply_duty(D)

            # Irradiance-change check during USC (Eq. 19)
            if |Pmpp - P| / Pmpp > irr_USC_thresh:
                # irradiance changed → reinitialize MP&O from D = 0.85
                D_re1 = 0.85
                apply_duty(D_re1)
                wait Ts_mppt
                P_re1 = measure_voltage() * measure_current()

                D_re2 = 0.5
                apply_duty(D_re2)
                wait Ts_mppt
                P_re2 = measure_voltage() * measure_current()

                if P_re2 > P_re1:
                    D = D_re2
                else:
                    D = D_re1
                end if

                Dmpp = D
                Pmpp = max(P_re1, P_re2)
                P_prev = Pmpp
                continue   # restart tracking around new point
            else
                # stay at local MPP
                D_mpp = Dmpp
                P_mpp = Pmpp
                return [D_mpp, P_mpp]
            end if
        end if

        # --- Adaptive ΔD using trapezoidal rule (Eqs. 15–16, 18) ---
        # Area A ≈ (P + Pmpp) / (2 * m_trap)
        A  = (P + Pmpp) / (2 * m_trap)
        if ΔP > 0:
            ΔD = ΔDmin + A
        else:
            ΔD = ΔDmin - A
        end if

        # Conventional P&O decision on duty direction
        ΔV = V - V_prev
        if ΔP > 0:
            if ΔV > 0:
                D = D + ΔD
            else:
                D = D - ΔD
            end if
        else:
            if ΔV > 0:
                D = D - ΔD
            else:
                D = D + ΔD
            end if
        end if

        D = saturate(D, Dmin, Dmax)
        apply_duty(D)

        # Update best MPP estimate
        if P > Pmpp:
            Pmpp = P
            Dmpp = D
        end if

        # Prepare next loop
        P_prev = P
        V_prev = V

        wait Ts_mppt
    end loop
end function


function [D_gmpp, P_gmpp] = MCOA_PSC():

    # --- Initialization of coots (duty positions) ---
    for i = 1..PopSize:
        D[i] = D_coot_init[i]
        apply_duty(D[i])
        wait Ts_mppt
        P[i] = measure_voltage() * measure_current()
        Pbest[i] = P[i]
        Dbest[i] = D[i]
    end for

    PGbest_prev = -∞
    iter        = 0

    # ------------- Main MCOA loop -------------
    while iter < Iter_max_MCOA:

        # 1) Update personal bests (Eq. 20)
        for i = 1..PopSize:
            if P[i] > Pbest[i]:
                Pbest[i] = P[i]
                Dbest[i] = D[i]
            end if
        end for

        # 2) Determine global best power & duty
        PGbest = max(Pbest[i])
        idxG   = argmax_i(Pbest[i])
        DGbest = Dbest[idxG]

        # 3) Termination condition for global MPP (Eq. 21)
        if (iter > 0) AND (|PGbest - PGbest_prev|/PGbest_prev < ϵ_PGbest) AND (ΔD_last < 0.05):
            D_gmpp  = DGbest
            P_gmpp  = PGbest
            return [D_gmpp, P_gmpp]
        end if

        # 4) Position update for each coot (Eqs. 22–28)

        # 4a) Random wandering / group following (Eq. 22, 23)
        for i = 1..PopSize:

            R1 = rand(0,1)
            R2 = rand(0,1)

            # Random search enhancement (Eq. 23)
            D[i] = D[i] + W * abs(DGbest - D[i])

            # Weight update (Eq. 24)
            W = 2 - L * (1/Iter_max_MCOA)

            # Chain motion (Eq. 25)
            if i > 1:
                D[i] = 0.5 * (D[i-1] + D[i])
            end if
        end for

        # 4b) Leader selection & update (Eqs. 26–28)
        # number of leaders NL, index M from Eq. 26
        for i = 1..PopSize:
            R = rand(0,1)
            B = 2 - L * (1/Iter_max_MCOA)      # Eq. 28

            if R < 0.5:
                DGbest = B * cos(2*R*π) * TF * abs(gBest - DGbest) + gBest
            else:
                DGbest = B * cos(2*R*π) * TF * abs(gBest - DGbest) - gBest
            end if
        end for

        # 5) Apply new positions & evaluate power
        for i = 1..PopSize:
            D[i] = saturate(D[i], Dmin, Dmax)
            apply_duty(D[i])
            wait Ts_mppt
            P[i] = measure_voltage() * measure_current()
        end for

        PGbest_prev = PGbest
        ΔD_last     = max_i |D[i] - DGbest|

        iter = iter + 1
    end while

    # If max iterations reached, return best found
    D_gmpp = DGbest
    P_gmpp = PGbest
    return [D_gmpp, P_gmpp]
end function


procedure Hybrid_MPO_MCOA_MPPT():

    loop forever:

        # 1) Detect operating condition (USC vs PSC)
        mode = Detect_Condition_USC_or_PSC()

        if mode == "USC":

            # --- Use MP&O with trapezoidal rule ---
            [D_star, P_star] = MPO_USC()

        else:

            # --- Use MCOA to find GMPP ---
            [D_star, P_star] = MCOA_PSC()

        end if

        # 2) Hold at best duty and monitor irradiance (Eq. 29)
        while TRUE:

            apply_duty(D_star)
            wait Ts_mppt

            V_now = measure_voltage()
            I_now = measure_current()
            P_now = V_now * I_now

            if |P_star - P_now| / P_star > irr_global_thresh:
                # irradiance changed → restart whole hybrid procedure
                break
            end if
        end while

    end loop
end procedure
)
