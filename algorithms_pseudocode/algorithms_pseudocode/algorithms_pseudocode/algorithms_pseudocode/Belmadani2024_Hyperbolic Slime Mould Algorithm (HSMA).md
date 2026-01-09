measure_voltage(), measure_current()   # PV sensing
apply_duty_cycle(D)                    # PWM / converter interface
Ts_mppt                                # MPPT sampling period (0.05 s in paper)

PopSize   = 4           # Number of candidate duty cycles
MaxIter   = 10          # Maximum HSMA iterations (T)
Dmin      = 0.1         # Duty-cycle lower bound
Dmax      = 0.9         # Duty-cycle upper bound

z         = 0.03        # Random-exploration probability in original SMA (deleted in HSMA)
S[1..4]   = [ +1, -1, +1, -1 ]   # Direction flags

# Convergence thresholds (Eq. (17))
eps_D     = 0.01        # Relative duty-cycle change
eps_P     = 0.01        # Relative power change

# Environment-change threshold (Eq. (18))
eps_env   = 0.02        # 2% drop in best power ⇒ re-run HSMA

function [P, V, I] = evaluate_power(D):
    D = saturate(D, Dmin, Dmax)
    apply_duty_cycle(D)
    wait_until_converter_settles()
    V = measure_voltage()
    I = measure_current()
    P = V * I
    return [P, V, I]
end function

function w_i = compute_weight(i, fitness_sorted, bestFitness, worstFitness):
    # fitness_sorted = S(i) sequence sorted from best to worst
    r  = rand(0,1)
    bF = bestFitness
    wF = worstFitness

    if i < PopSize/2:
        w_i = 1 + r * log( (bF - fitness_sorted[i]) / (bF - wF) + 1 )
    else:
        w_i = 1 - r * log( (bF - fitness_sorted[i]) / (bF - wF) + 1 )
    end if
    return w_i
end function

function [Dbest1, Pbest1, Dbest2, Pbest2] = update_best_two(D[1..4], P[1..4]):

    # 1) Find global best
    idx1   = argmax_i P[i]
    Dbest1 = D[idx1]
    Pbest1 = P[idx1]

    # 2) Find second best OUTSIDE ±0.08 around Dbest1 (region clustering)
    Pbest2 = -∞
    Dbest2 = Dbest1          # placeholder

    for i = 1..PopSize:
        if i == idx1:
            continue
        end if
        if |D[i] - Dbest1| < 0.08:
            continue    # same region as Dbest1 → skip
        end if
        if P[i] > Pbest2:
            Pbest2 = P[i]
            Dbest2 = D[i]
        end if
    end for

    # If no valid second region found, keep any second-best (rare corner case)
    if Pbest2 == -∞:
        idx2 = argmax_i, i≠idx1 P[i]
        Dbest2 = D[idx2]
        Pbest2 = P[idx2]
    end if

    return [Dbest1, Pbest1, Dbest2, Pbest2]
end function

function Db = select_Db(ite, i, Dbest1, Dbest2):
    # ite = iteration index (1..MaxIter), i = candidate index (1..4)

    if ite == 1 OR ite == 2:
        if i == 1 OR i == 2:
            Db = Dbest1
        else                # i == 3 or 4
            Db = Dbest2
        end if
    else
        # From 3rd iteration onward → only global best region
        Db = Dbest1
    end if
    return Db
end function

function a = compute_a(ite, MaxIter):
    # Convergence factor (Eq. 15)
    a = exp( -4 * (ite / MaxIter)^2 )
    return a
end function

function G = compute_G(ite, Db, D_i):
    if |Db - D_i| < 0.01 AND ite == 2:
        G = 0.8
    elseif |Db - D_i| < 0.01 AND ite == 3:
        G = 0.95
    else
        G = 1.0
    end if
    return G
end function

function d = compute_d(ite):
    if ite == 2:
        d = 2
    else
        d = 1
    end if
    return d
end function

function D_new = HSMA_update_iter1(i, Db, D[1..4], a, S):

    # Choose two successive duty ratios Dr1, Dr2 at random from D
    # e.g., (0.2,0.4), (0.4,0.6), (0.6,0.8) etc.
    idx  = random_integer(1, PopSize-1)
    Dr1  = D[idx]
    Dr2  = D[idx+1]

    r    = rand(0,1)

    D_new = Db + S[i] * a * r * ( tanh(Dr1) - tanh(Dr2) )

    return D_new
end function

function D_new = HSMA_update_general(i, Db, D_i, a, w_i, G, d, ite, S):

    # Exponential damping term
    E = exp( -(Db - D_i) * ite )

    # Inner hyperbolic term
    inner = (G * D_i) / (d * E)

    D_new = Db + S[i] * a * tanh( w_i * Db - tanh(inner) )

    return D_new
end function

procedure HSMA_MPPT():

    # --- Initialization of duty-cycle population (iteration 0) ---
    PopSize = 4
    MaxIter = 10
    D[1] = 0.2
    D[2] = 0.4
    D[3] = 0.6
    D[4] = 0.8

    for i = 1..PopSize:
        [P[i], V_i, I_i] = evaluate_power(D[i])
    end for

    [Dbest1, Pbest1, Dbest2, Pbest2] = update_best_two(D, P)
    Pt_best_prev = Pbest1
    D_best_prev  = Dbest1

    ite = 0
    converged = FALSE

    # ===========================
    # Main HSMA optimization loop
    # ===========================
    while (ite < MaxIter) AND (converged == FALSE):

        ite = ite + 1

        # Compute convergence factor 'a' for this iteration
        a = compute_a(ite, MaxIter)

        # Rank fitness and compute weights w_i (needed for general HSMA)
        fitness_sorted = sort_descending(P[1..PopSize])
        bestFitness    = fitness_sorted[1]
        worstFitness   = fitness_sorted[PopSize]

        # ---------- Update each duty-cycle candidate ----------
        for i = 1..PopSize:

            Db = select_Db(ite, i, Dbest1, Dbest2)

            if ite == 1:
                # First iteration uses Eq. (14)
                D_new = HSMA_update_iter1(i, Db, D, a, S)

            else:
                # General case uses Eq. (11)
                w_i = compute_weight(i, fitness_sorted, bestFitness, worstFitness)
                G   = compute_G(ite, Db, D[i])
                d   = compute_d(ite)

                D_new = HSMA_update_general(i, Db, D[i], a, w_i, G, d, ite, S)
            end if

            # Saturate and assign
            D[i] = saturate(D_new, Dmin, Dmax)

        end for   # i = 1..PopSize

        # ---------- Evaluate new population ----------
        for i = 1..PopSize:
            [P[i], V_i, I_i] = evaluate_power(D[i])
        end for

        # ---------- Update best two regions ----------
        [Dbest1, Pbest1, Dbest2, Pbest2] = update_best_two(D, P)

        # ---------- Convergence check (Eq. 17) ----------
        rel_D = |Dbest1 - D_best_prev| / max(|Dbest1|, small_value)
        rel_P = |Pbest1 - Pt_best_prev| / max(Pbest1, small_value)

        if (rel_D <= eps_D) AND (rel_P <= eps_P):
            converged = TRUE
        end if

        # Store for next iteration
        D_best_prev  = Dbest1
        Pt_best_prev = Pbest1

    end while

    # After convergence or reaching MaxIter, use Dbest1 as MPPT duty cycle
    D_mpp = Dbest1
    apply_duty_cycle(D_mpp)

    # Store final best power for environment-change monitoring
    P_mpp_best = Pt_best_prev

    return [D_mpp, P_mpp_best]
end procedure
