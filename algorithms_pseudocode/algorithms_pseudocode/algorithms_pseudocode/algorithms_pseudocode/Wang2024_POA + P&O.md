Vpv(k), Ipv(k)          # Measured PV voltage and current at step k
Ts_mppt                 # MPPT control period

PopSize                 # Number of pelicans (candidate duty cycles)
MaxIter                 # Max POA iterations per global search
D_min, D_max            # Duty cycle bounds

θ        = 0.85         # Convergence coefficient (POA exploration, Eq. 11)
α, β     = α_step, 1.5  # Levy flight parameters (POA exploitation, Eq. 12)

switch_tol       = 0.05 # |Dt - D*| ≤ 0.05 → switch from POA to PO (local)
restart_threshold = 0.05# Relative power change > 5% → restart POA
ΔD_PO             = ΔD0  # Fixed perturbation step for PO phase
εP_small          = 1e-6 # Small value to avoid division by zero

function evaluate_power(D):
    apply_duty_cycle(D)
    wait_until_converter_settles()
    V = measure_voltage()
    I = measure_current()
    return V * I, V, I
end function

function exploration_update(D_t, D_best):
    # Implements Eq. (11) with coefficient θ
    # Returns new candidate duty cycle D_new
    ...
end function

function exploitation_update(D_t, D_best):
    # Implements Eq. (12) with Levy flight
    # Returns new candidate duty cycle D_new
    ...
end function


GMPP_POA_Stage:

# --- Initialization of pelican population ---
for t = 1 to PopSize:
    D_t[t] = random_uniform(D_min, D_max)     # Eq. (7)
    P_t[t], V_t[t], I_t[t] = evaluate_power(D_t[t])
end for

# Global best
D_best, P_best = argmax_over_t(P_t[t])
iter = 0

# --- Main POA loop: exploration + exploitation ---
while iter < MaxIter:

    # ----- Exploration phase (Eq. 11) -----
    for t = 1 to PopSize:
        D_candidate = exploration_update(D_t[t], D_best)
        D_candidate = saturate(D_candidate, D_min, D_max)

        P_candidate, _, _ = evaluate_power(D_candidate)

        # Greedy selection (keep better position)
        if P_candidate > P_t[t]:
            D_t[t] = D_candidate
            P_t[t] = P_candidate
        end if
    end for

    # ----- Exploitation phase (Eq. 12) -----
    for t = 1 to PopSize:
        D_candidate = exploitation_update(D_t[t], D_best)
        D_candidate = saturate(D_candidate, D_min, D_max)

        P_candidate, _, _ = evaluate_power(D_candidate)

        if P_candidate > P_t[t]:
            D_t[t] = D_candidate
            P_t[t] = P_candidate
        end if
    end for

    # Update global best pelican
    D_best, P_best = argmax_over_t(P_t[t])

    # Check convergence to best position (|Dt − D*| ≤ 0.05)
    # Here Dt_current is duty cycle of the currently selected best pelican
    Dt_current = D_best
    if abs(Dt_current - D_best) <= switch_tol:
        # Store references to start PO phase
        D_ref = D_best
        P_ref = P_best
        V_ref, I_ref = voltage_current_at(D_ref)   # measured once
        goto LMPP_PO_Stage
    end if

    iter = iter + 1
end while

# If MaxIter reached without convergence, still move to PO phase
D_ref = D_best
P_ref = P_best
V_ref, I_ref = voltage_current_at(D_ref)
goto LMPP_PO_Stage

LMPP_PO_Stage:

# Initialize local references
D = D_ref
P_prev = P_ref
V_prev = V_ref

while TRUE:

    # 1) Measurement
    V = measure_voltage()
    I = measure_current()

    # 2) Power and increments
    P = V * I
    ΔP = P - P_prev
    ΔV = V - V_prev          # computed but not used in decision here

    # 3) Check for irradiance change → restart POA if ΔP_rel > 5 %
    ΔP_rel = abs(ΔP) / max(P_prev, εP_small)

    if ΔP_rel > restart_threshold:
        # Large power change → irradiance change → global re-search
        goto GMPP_POA_Stage
    end if

    # 4) Basic PO decision: sign of ΔP chooses perturbation direction
    if ΔP > 0:
        # Positive perturbation
        D = D + ΔD_PO
    else:
        # Negative perturbation
        D = D - ΔD_PO
    end if

    # 5) Duty cycle saturation and application
    D = saturate(D, D_min, D_max)
    apply_duty_cycle(D)

    # 6) Update references for next iteration
    P_prev = P
    V_prev = V

    wait(Ts_mppt)
end while

