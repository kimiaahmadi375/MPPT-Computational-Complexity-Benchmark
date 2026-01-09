# =============================================================
# MCOA Stage (Modified Coot Optimization Algorithm) for GMPP
# Invoked only when PSC is detected by the hybrid supervisor
# Search variable: duty cycle D (or Vref if the paper uses voltage control)
# =============================================================

Inputs:
    measure_voltage(), measure_current()
    apply_duty_cycle(D)
Parameters:
    N          # number of coots (population size)
    Imax       # maximum MCOA iterations
    Dmin, Dmax # duty bounds (e.g., 0.1 to 0.9)
    a          # single tuning parameter (paper indicates one main parameter)
    eps_conv   # convergence threshold (optional)

State (outputs):
    D_best, P_best

# ---------------------------
# 1) Initialization (PSC mode)
# ---------------------------
for i = 1..N:
    D[i] = Dmin + (i-1)*(Dmax-Dmin)/(N-1)      # uniform (or random) init
    apply_duty_cycle(D[i])
    wait settle
    P[i] = measure_voltage() * measure_current()

(best_idx) = argmax(P[1..N])
D_best = D[best_idx]
P_best = P[best_idx]

# ---------------------------
# 2) Main MCOA iteration loop
# ---------------------------
iter = 1
while iter ≤ Imax:

    # 2.1 Identify leader / best coot
    leader = argmax(P[1..N])
    D_lead = D[leader]
    P_lead = P[leader]

    # 2.2 Update positions (Modified COA rules)
    # NOTE: exact equations depend on the paper; below is a standard template:
    for i = 1..N:

        if i == leader:
            continue  # keep leader (or apply slight exploration per paper)

        r = rand()              # one (or few) random numbers, per "low RNG" claim
        s = rand()

        # --- Rule A: follow leader (exploitation) ---
        # pull coot toward leader with strength controlled by 'a'
        D_new = D[i] + a*r*(D_lead - D[i])

        # --- Rule B: social following / random peer (exploration) ---
        # (optional) choose another coot j != i
        j = random_integer(1..N, j != i)
        D_new = D_new + (1-a)*s*(D[j] - D[i])

        # --- Rule C: (optional) search-space skipping ---
        # if paper uses skipping: skip updates in regions deemed "non-promising"
        # e.g., based on previous peak locations or voltage windows
        # if skip_condition(i): continue

        # clamp bounds
        if D_new > Dmax: D_new = Dmax
        if D_new < Dmin: D_new = Dmin

        # evaluate updated candidate
        apply_duty_cycle(D_new)
        wait settle
        P_new = measure_voltage() * measure_current()

        # greedy selection
        if P_new > P[i]:
            D[i] = D_new
            P[i] = P_new

    # 2.3 Update global best
    best_idx = argmax(P[1..N])
    if P[best_idx] > P_best:
        P_best = P[best_idx]
        D_best = D[best_idx]

    # 2.4 Convergence test (optional)
    # e.g., if abs(P_best - P_lead)/max(P_best, tiny) < eps_conv:
    #     break

    iter = iter + 1

# ---------------------------
# 3) Output of MCOA stage
# ---------------------------
apply_duty_cycle(D_best)
return D_best, P_best
