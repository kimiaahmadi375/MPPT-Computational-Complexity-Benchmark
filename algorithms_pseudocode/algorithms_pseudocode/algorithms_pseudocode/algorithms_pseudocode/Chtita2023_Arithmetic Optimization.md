# -------------------------------------------------------------
# AO-based MPPT (Chtita 2023) – flowchart-equivalent pseudocode
# -------------------------------------------------------------

# 1) Set control parameters
Np   ← number of search operators (duty cycles)
MCN  ← maximum cycle number
Ts   ← sampling period ( > converter settling time)
α, μ ← AO tuning parameters
LB, UB ← min/max duty bounds
ΔPpv_thresh ← climatic-change threshold from Eq. (11)

# Initialize search operators (duty cycles)
for i = 1..Np:
    dc[i] ← random_uniform(LB, UB)

C        ← 0          # current AO cycle
Ppv_last ← 0          # best PV power from previous cycle (for Eq. 11)

# -------------------------------------------------------------
# Main loop – stop when SOC reaches 100%
# -------------------------------------------------------------
while SOC < 100%:

    sense_SOC()
    if SOC ≥ 100%:
        break

    # ===== 1) Evaluate all search operators (duty cycles) =====
    for i = 1..Np:

        # Output duty cycle to converter and measure PV power
        apply_duty(dc[i])
        wait(Ts)
        Vpv_i ← measure_Vpv()
        Ipv_i ← measure_Ipv()
        Ppv_i ← Vpv_i * Ipv_i

        # Store last evaluated power (needed for Ppv(i−1))
        if i == 1:
            Ppv_prev ← Ppv_i          # simple choice for first operator
        else:
            # Compare with previous sample as in flowchart
            if Ppv_i > Ppv_prev:
                Pbest[i] ← Ppv_prev
            else:
                Pbest[i] ← Ppv_i
            end if
            Ppv_prev ← Ppv_i
        end if

    end for

    # Best power in current cycle (for stopping & Eq. 11)
    Ppv_best_cycle ← max_i(Pbest[i])

    # ===== 2) Update MOA & MOP (Eqs. 5 and 7) =====
    MOA ← compute_MOA(C, MCN)        # Eq. (5)
    MOP ← compute_MOP(C, MCN, α)     # Eq. (7)

    # ===== 3) Update all duty cycles using AO rules =====
    for i = 1..Np:

        r1, r2, r3 ← rand(0,1), rand(0,1), rand(0,1)

        if r1 > MOA:
            # --- Exploration phase: D or M (Eq. 9) ---
            if r2 > 0.5:
                dc[i] ← update_D_operator(dc, i, MOP, μ, LB, UB)  # Division
            else:
                dc[i] ← update_M_operator(dc, i, MOP, μ, LB, UB)  # Multiplication
            end if
        else:
            # --- Exploitation phase: S or A (Eq. 10) ---
            if r3 > 0.5:
                dc[i] ← update_S_operator(dc, i, MOP, μ, LB, UB)  # Subtraction
            else:
                dc[i] ← update_A_operator(dc, i, MOP, μ, LB, UB)  # Addition
            end if
        end if

        # Keep duty within bounds
        dc[i] ← clamp(dc[i], LB, UB)

    end for

    # ===== 4) Cycle counter and stopping conditions =====
    C ← C + 1

    # (a) Stop if maximum cycle reached OR power unchanged
    if (C = MCN) or power_unchanged_for_several_cycles(Ppv_best_cycle):
        D_best ← duty_with_max_Pbest(Pbest[1..Np])
        apply_duty(D_best)
        return D_best
    end if

    # (b) Climatic change detection (Eq. 11)
    if abs(Ppv_best_cycle - Ppv_last) / max(Ppv_last, small_value) ≥ ΔPpv_thresh:
        # Reinitialize duty cycles and restart AO search
        for i = 1..Np:
            dc[i] ← random_uniform(LB, UB)
        C        ← 0
        Ppv_last ← Ppv_best_cycle
        continue   # back to while SOC<100
    end if

    # Prepare for next cycle
    Ppv_last ← Ppv_best_cycle

end while

# SOC reached 100% → stop MPPT
stop_algorithm()
