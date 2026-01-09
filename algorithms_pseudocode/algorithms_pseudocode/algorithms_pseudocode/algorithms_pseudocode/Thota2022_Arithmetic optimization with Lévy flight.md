# -------------------------------------------------------------
# Thota2022 – AOA-LF GMPPT Algorithm (Arithmetic Optim. + Lévy)
# -------------------------------------------------------------
# Interfaces to hardware
#   measure_voltage()  → VPV(k)
#   measure_current()  → IPV(k)
#   apply_duty(D)      → send duty D to DC–DC converter
#   wait(Ts_mppt)      → sampling / settling time

# Main AOA–LF parameters (from paper)
N_pop        = N_duty           # number of candidate duty cycles
MaxIter      = T_max            # maximum AOA iterations
dMin, dMax                      # allowed duty range (10–90% etc.)
Min_MOA, Max_MOA                # MOA limits
alpha        = 5                # sensitivity parameter (for MOP)
mu           = 0.5              # AOA control parameter (used in original AOA)
epsilon      = 2.22e-16         # small constant in AOA equations

# Lévy-flight parameters
beta         = 1.5              # Lévy index
lambda_LF    = random in [-1,1] # scale factor (implicit in step-size formula)

# -------------------------------------------------------------
# Helper: Lévy flight step around current position d(t)
# (Mantegna’s algorithm – simplified pseudocode)
# -------------------------------------------------------------
function step = levy_step():
    # draw m, n from normal distributions with std σ_m, σ_n
    m = randn(0, σ_m)
    n = randn(0, σ_n)

    step = m / ( |n|^(1/beta) )
    return step
end function

function d_new = LevyWalk(d_current):
    step = levy_step()
    step_size = 0.01 * step * d_current      # Eq. (10)–(12) compressed
    d_new = d_current + step_size
    return d_new
end function

# -------------------------------------------------------------
# Fitness evaluation at a given duty cycle
# -------------------------------------------------------------
function P = evaluate_fitness(D):
    D = saturate(D, dMin, dMax)
    apply_duty(D)
    wait(Ts_mppt)
    V = measure_voltage()
    I = measure_current()
    P = V * I                 # Eq. (3)
    return P
end function


# -------------------------------------------------------------
# Main AOA-LF GMPPT search for current irradiance pattern
# -------------------------------------------------------------
procedure AOA_LF_GMPPT():

    # ---- Step 1: initialize random duty population (Eq. 13) ----
    for i = 1 .. N_pop:
        d[i] = dMin + rand(0,1) * (dMax - dMin)
        P[i] = evaluate_fitness(d[i])
    end for

    # select initial global best (Eq. 4)
    idxBest = argmax_i P[i]
    dBest   = d[idxBest]
    Pmax    = P[idxBest]

    iter = 0

    # ------------- AOA–LF main optimization loop -------------
    while iter < MaxIter:

        iter = iter + 1

        # ---- Update AOA control coefficients (Eqs. 5 & 7) ----
        MOA = Min_MOA + (Max_MOA - Min_MOA) * (iter / MaxIter)
        MOP = 1 - (iter)^(1/alpha) / (MaxIter)^(1/alpha)

        # ---- Update each duty candidate using AOA-LF rules ----
        for i = 1 .. N_pop:

            r1 = rand(0,1)
            r2 = rand(0,1)
            r3 = rand(0,1)

            # Current position used for Lévy walk
            d_i   = d[i]
            d_LF  = LevyWalk(d_i)      # LevyWalk(d(t)) in Eqs. (14),(15)

            if r1 > MOA then
                # ---------------- Exploration phase ----------------
                if r2 > 0.5 then
                    # Division operator “D”  (Eq. 14, r2 > 0.5)
                    d_new = dBest - (MOP + epsilon) * d_LF
                else
                    # Multiplication operator “M”  (Eq. 14, otherwise)
                    d_new = dBest -  MOP          * d_LF
                end if

            else
                # ---------------- Exploitation phase ----------------
                if r3 > 0.5 then
                    # Subtraction operator “S”  (Eq. 15, r3 > 0.5)
                    d_new = dBest - MOP * d_LF
                else
                    # Addition operator “A”  (Eq. 15, otherwise)
                    d_new = dBest + MOP * d_LF
                end if
            end if

            # Bound duty cycle to [dMin, dMax]
            d_new = saturate(d_new, dMin, dMax)

            # Evaluate new fitness
            P_new = evaluate_fitness(d_new)

            # Replace candidate if improved
            if P_new > P[i]:
                d[i] = d_new
                P[i] = P_new
            end if

            # Update global best if needed
            if P_new > Pmax:
                Pmax  = P_new
                dBest = d_new
            end if

        end for   # end for each candidate

        # (Stopping test is just “iter ≥ MaxIter” in this paper;
        #  any additional convergence tests can be added here.)

    end while

    # After finishing iterations → apply best duty as GMPP duty
    D_gmpp = dBest
    apply_duty(D_gmpp)

    return D_gmpp, Pmax
end procedure
