# -------------------------------------------------------------
# Rayleigh-Distribution + TKEO Guided PSO MPPT (RD-PSO)
# Based on Fig. 10 and Eqs. (9)–(17)
# -------------------------------------------------------------

Inputs:
    Vpv(), Ipv()                      # PV measurements
    apply_duty(D)                    # PWM/converter interface
Parameters:
    PopSize = 4                      # Four particles (uniform spacing)
    Tmax    = 50                     # Maximum PSO iterations
    Dmin = 0.1, Dmax = 0.9
    γ = 4                             # RD scaling (Eq. 13)
    P_th                               # TKEO threshold (Eq. 16)

# -------------------------------------------------------------
# 1. Initialization (Section II-D-1)
# -------------------------------------------------------------
Initialize particles D[i] uniformly in [Dmin, Dmax]
Initialize velocity V[i] = 0

For each particle i:
    apply_duty(D[i])
    wait converter settle
    P[i] = Vpv() * Ipv()
    Pbest[i] = P[i]
    Dbest[i] = D[i]

Find global best:
    Gbest = argmax(P[i])

t = 1
W = 1.0                   # initial inertia coefficient

# =============================================================
# 2. Main RD-PSO Loop
# =============================================================
while t ≤ Tmax:

    # ---------------------------------------------------------
    # 2.1 Velocity update (Eq. 9)
    # ---------------------------------------------------------
    for each particle i:
        r1 = rand(), r2 = rand()

        V[i] = W * V[i]
              + C1*r1 * (Dbest[i] - D[i])
              + C2*r2 * (Gbest    - D[i])

    # ---------------------------------------------------------
    # 2.2 Position update (Eq. 11)
    # ---------------------------------------------------------
    for each particle i:
        D[i] = D[i] + V[i]
        D[i] = saturate(D[i], Dmin, Dmax)

    # ---------------------------------------------------------
    # 2.3 Apply Rayleigh-Distribution dispersion
    #     to 50% worst particles (Eqs. 12–14)
    # ---------------------------------------------------------
    sort particles by P (ascending → worst first)
    for worst half:
        X = 1 + γ * rand()          # Eq. 13
        R = (X^t / t^2) * exp( -X^2 / (2*t^2) )   # Eq. 12
        D[i] = D[i] + R * rand()
        D[i] = saturate(D[i], Dmin, Dmax)

    # ---------------------------------------------------------
    # 2.4 Evaluate fitness after updates
    # ---------------------------------------------------------
    for each particle i:
        apply_duty(D[i])
        wait converter settle
        P[i] = Vpv() * Ipv()

        if P[i] > Pbest[i]:
            Pbest[i] = P[i]
            Dbest[i] = D[i]

    Gbest_prev = Gbest
    Pbest_prev = max(Pbest)

    Gbest = particle_with_max(P)

    # ---------------------------------------------------------
    # 2.5 TKEO-based redundancy removal (Eq. 15)
    # ---------------------------------------------------------
    for each adjacent particle pair (i, i+1):
        E = sqrt( D[i]^2 - D[i]*D[i+1] )
        if E < small_threshold:
            # Replace both with their average
            Davg = (D[i] + D[i+1]) / 2
            D[i] = Davg
            D[i+1] = Davg

    # ---------------------------------------------------------
    # 2.6 TKEO termination test (Eq. 16)
    # ---------------------------------------------------------
    pick three random particles j, k, m
    P_tkeo = sqrt( P[m]^2 - P[j]*P[k] )   # Eq. 16

    if P_tkeo < P_th:
        break        # Converged at GMPP

    # ---------------------------------------------------------
    # 2.7 Dynamic detection (Eqs. 17–19)
    # ---------------------------------------------------------
    ΔP = (Pbest - Pbest_prev) / Pbest

    if ΔP > threshold:              # dynamics detected
        dV = Vpv(t) - Vpv(t-1)
        dI = Ipv(t) - Ipv(t-1)

        if dV*dI < 0:
            # Load variation → Step duty, NO reinitialization
            D = D + small_step
            apply_duty(D)
            continue

        if dV*dI > 0:
            # Now must differentiate irradiance vs shading
            measure Isc_lowV by forcing max duty
            if Isc_lowV changed significantly:
                # Irradiance change → step duty update
                D = D + small_step
                apply_duty(D)
                continue
            else:
                # Shading pattern change → reinitialize population
                reinitialize all D[i]
                continue

    # ---------------------------------------------------------
    # 2.8 Update inertia weight (Eq. 10)
    # ---------------------------------------------------------
    W = W * (1 - t/Tmax)

    t = t + 1

# =============================================================
# 3. Output
# =============================================================
apply_duty(Gbest)
return Gbest
