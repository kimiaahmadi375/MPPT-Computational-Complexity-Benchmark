# -----------------------------------------
# EVO–MPPT Algorithm (Based on Fig. 6–7 and Eqs. 16–22)
# -----------------------------------------

Inputs:
    measure_voltage(), measure_current()
    apply_duty_cycle(D)
    Ts_mppt                      # MPPT sampling period
Parameters:
    PopSize = 4                  # Number of duty–cycle particles
    N_iter_max = 50              # Maximum iterations
    eps_env = 0.02               # ΔP ≥ 2% Pmax ⇒ reinitialization
    Dmin = 0.1
    Dmax = 0.9

# -----------------------------------------
# Initialization
# -----------------------------------------
Initialize particle positions Xi (i = 1..4) randomly in [Dmin, Dmax]
t = 0
Evaluate fitness Fi for each particle using Eq. (20)
Compute NEL_i for each particle
Compute Enrichment Bound EB = (sum(NEL_i))/PopSize

# Main loop
while t < N_iter_max:

    # Compute Stability Levels (SL_i) using Eq. (15)
    Determine BS = best fitness (minimum objective)
    Determine WS = worst fitness (maximum objective)
    for each particle i:
        SL_i = (NEL_i - BS) / (WS - BS)

    # -----------------------------------------
    # Update particles according to flowchart
    # -----------------------------------------
    for each particle i:

        if NEL_i > EB:

            # Determine Stability Bound SB for particle i
            SB = stability_bound(i)   # Eq. (19)

            if SL_i > SB:
                # High stability → use α and γ decay (Eq. 16, 18)
                Xi = alpha_decay_update(Xi, X_best)      # Eq. (16)
                Xi = gamma_decay_update(Xi, neighbors)   # Eq. (18)

            else:
                # Medium stability → use β update (Eq. 20, 21)
                Xi = beta_decay_update(Xi, X_best, X_center)    # Eq. (20)
                Xi = neighbor_update(Xi, X_best, X_neighbor)    # Eq. (21)

        else:
            # Low enrichment → electron capture / random exploration (Eq. 22)
            Xi = electron_capture_update(Xi, X_best, ... )           # Eq. (22)

        # Bound the duty cycle
        Xi = saturate(Xi, Dmin, Dmax)

    end for

    # -----------------------------------------
    # Recompute fitness after updates
    # -----------------------------------------
    for i = 1..4:
        Fi = compute_fitness(Xi)       # Evaluate PV power

    # Track best particle (global optimum)
    X_best_prev = X_best
    F_best_prev = F_best

    X_best = particle_with_max_power()
    F_best = max(Fi)

    # -----------------------------------------
    # Stopping condition 1: iteration limit
    # -----------------------------------------
    t = t + 1
    if t >= N_iter_max:
        break

    # -----------------------------------------
    # Stopping condition 2: environmental change
    # ΔP ≥ 2% Pmax triggers reinitialization
    # -----------------------------------------
    if |F_best - F_best_prev| / F_best ≥ eps_env:
        Reinitialize population Xi
        continue

end while

# -----------------------------------------
# Return most stable particle as MPPT duty cycle
# -----------------------------------------
D_MPP = X_best
apply_duty_cycle(D_MPP)
return D_MPP
