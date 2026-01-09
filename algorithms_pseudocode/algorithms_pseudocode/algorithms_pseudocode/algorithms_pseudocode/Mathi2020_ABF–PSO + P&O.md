# ------------------------------------------------------------
# Parameters / constants (per paper)
# ------------------------------------------------------------
N = 4                      # population count (4 particles) :contentReference[oaicite:1]{index=1}
lb = 0.10                  # lower bound of duty search space :contentReference[oaicite:2]{index=2}
ub = 0.85                  # upper bound of duty search space :contentReference[oaicite:3]{index=3}
s_thresh = 0.70            # GP region detection threshold :contentReference[oaicite:4]{index=4}

# BF/ABF-PSO coefficients (paper keeps c1,c2 like PSO; pk, s, w, alpha updated)
c1, c2 = constants
epsilon = small_value

# Variables
dc[1..N]      # particle positions = duty cycles
vel[1..N]     # particle velocities
p[1..N]       # fitness values (PV power)
Pbest[1..N]   # best fitness of each particle
Xlb[1..N]     # best position of each particle (local best position)
gbest         # best position among all particles
Pgbest        # best fitness among all particles

# ------------------------------------------------------------
# STAGE A: ABF-PSO (identify global region)
# ------------------------------------------------------------
procedure ABF_PSO_IDENTIFY_GLOBAL_REGION():

    # (Stage 1) initialize particles and counters :contentReference[oaicite:5]{index=5}
    for i = 1..N:
        dc[i] = uniform(lb, ub)     # paper says uniformly distributed
        vel[i] = 0
        p[i] = 0
        Pbest[i] = 0
        Xlb[i] = dc[i]
    end for
    u = 1

    loop:  # repeats until s > 0.7, then hand over to VS-P&O

        # (Stages 2–6) evaluate particles sequentially :contentReference[oaicite:6]{index=6}
        while u <= N:

            apply_duty(dc[u])
            wait(Ts)
            V = measure_voltage()
            I = measure_current()
            Ppv = V * I
            p[u] = Ppv

            # (Stage 3) compare with previous best of same particle
            if p[u] > Pbest[u]:
                # (Stage 4) update best fitness and best position
                Pbest[u] = p[u]
                Xlb[u] = dc[u]
            end if

            # (Stage 5) increment particle index
            u = u + 1

        end while

        # (Stage 7) compute gbest, Xmax, Xmin, and ABF parameters 
        idx = argmax_i(Pbest[i])
        gbest  = Xlb[idx]
        Pgbest = Pbest[idx]

        Xmax = max_i(dc[i])
        Xmin = min_i(dc[i])

        # Adaptive sensitivity (Eq. 9):  s = exp( - (Xmax - Xmin) / ub ) :contentReference[oaicite:8]{index=8}
        s = exp( - (Xmax - Xmin) / ub )

        # Adaptive weight factor (Eq. 10): w = (Xmax - Xmin) / ub :contentReference[oaicite:9]{index=9}
        w = (Xmax - Xmin) / ub

        # Probability pk (Eq. 4), alpha (Eq. 8) used in BF-PSO velocity update :contentReference[oaicite:10]{index=10}
        # (Implementation note: Eq.4 uses fitness(gbest)/sum fitness(lbest)k; here lbest are particles.)
        sumFit = sum_i(Pbest[i]) + epsilon
        pk = Pgbest / sumFit
        alpha = rand(0,1) * pk

        # (Stage 8) convergence / GP-region check: is s > 0.7? :contentReference[oaicite:11]{index=11}
        if s > s_thresh:
            # Global region found -> return best position for P&O start
            return gbest
        end if

        # (Stages 9–10) update velocity and particle positions using BF-PSO Eqs. (6) & (7) :contentReference[oaicite:12]{index=12}
        for i = 1..N:
            r1 = rand(0,1)
            r2 = rand(0,1)

            # Eq. (6): vel_{k+1} = w*vel_k + s*(1-pk)*c1*r1*(Xlb_i - X_i) + c2*r2*(gbest - X_i)
            vel[i] = w*vel[i] + s*(1 - pk)*c1*r1*(Xlb[i] - dc[i]) + c2*r2*(gbest - dc[i])

            # Eq. (7): X_{k+1} = X_k + alpha * vel_{k+1}
            dc[i] = dc[i] + alpha * vel[i]

            # enforce bounds
            dc[i] = clamp(dc[i], lb, ub)
        end for

        # restart evaluation index
        u = 1

    end loop
end procedure


# ------------------------------------------------------------
# STAGE B: Variable-Step P&O (track GMPP inside global region)
# ------------------------------------------------------------
procedure VS_PO_TRACK_FROM(gbest_init):

    # (Stage 11) initialize VS-P&O variables 
    dold = gbest_init
    apply_duty(dold)
    wait(Ts)

    Vold = measure_voltage()
    Iold = measure_current()
    Pold = Vold * Iold

    loop:

        # (Stage 12) compute dP, dV, dI and compute Δd using Eq. (11) :contentReference[oaicite:14]{index=14}
        Vpv = measure_voltage()
        Ipv = measure_current()
        Ppv = Vpv * Ipv

        dP = Ppv - Pold
        dV = Vpv - Vold
        dI = Ipv - Iold

        # Eq. (11): Δd = M * |dP/dV|
        if dV != 0:
            del_d = M * abs(dP / dV)
        else:
            del_d = 0
        end if

        # (Stages 13–14) apply perturbation, decide direction based on sign of dP (Eq. 12 logic) 
        if dP > 0:
            dnew = dold + del_d
        else:
            dnew = dold - del_d
        end if
        dnew = clamp(dnew, lb, ub)
        apply_duty(dnew)

        # (Stage 15) irradiance-change test: (|dV| > Vth) AND (|dI| > Ith) ? 
        if (abs(dV) > V_threshold) AND (abs(dI) > I_threshold):

            # (Stage 17) sort particles dc[1..4] ascending (from last ABF-PSO iteration) :contentReference[oaicite:17]{index=17}
            sort dc ascending

            # (Stage 18) decide increase/decrease by sign of dP (flowchart shows “is dP < 0?”) 
            if dP < 0:
                # (Stage 19a) irradiance decrease => set last particle to ub
                dc[4] = ub
            else:
                # (Stage 19b) irradiance increase => set first particle to lb
                dc[1] = lb
            end if

            # (Stage 20) set s = 0 and call ABF-PSO again 
            # (Practically: restart ABF-PSO global-region identification using these dc[] as initial population.)
            return RESTART_ABF_PSO_WITH_INITIAL_POP(dc)

        else:
            # (Stage 16) no irradiance change => update stored values and continue :contentReference[oaicite:20]{index=20}
            dold = dnew
            Vold = Vpv
            Iold = Ipv
            Pold = Ppv
        end if

        wait(Ts)

    end loop
end procedure


# ------------------------------------------------------------
# TOP-LEVEL HYBRID CONTROLLER
# ------------------------------------------------------------
procedure HYBRID_ABF_PSO_VSPO_GMPPT():

    loop forever:

        gbest0 = ABF_PSO_IDENTIFY_GLOBAL_REGION()
        VS_PO_TRACK_FROM(gbest0)

        # (If VS_PO_TRACK_FROM triggers irradiance change, it returns
        #  and the loop repeats, restarting ABF-PSO as in the flowchart.)

    end loop
end procedure
