Vpv(k), Ipv(k)          # PV voltage and current at step k
Vpv(k-1), Ipv(k-1)      # Previous-step voltage and current
Vout(k)                 # Converter output voltage
Ts_MPC                  # MPC sampling time
ΔIref_step              # Fixed IC step size for Iref
PSO_params              # {N_particles, w, c1, c2, max_iter, ...}


Isc_STC                 # PV short-circuit current at STC
P_change_th = 0.05      # 5% power-change threshold (PSC / large step)
Mode ∈ {PSO_MODE, IC_MODE}
search_space = [0, Isc_STC]      # Current range for PSO particles


function compute_PV_quantities(Vpv, Ipv, Vpv_prev, Ipv_prev):
    Ppv      = Vpv * Ipv
    Ppv_prev = Vpv_prev * Ipv_prev
    ΔPpv     = Ppv - Ppv_prev
    ΔIpv     = Ipv - Ipv_prev
    ΔVpv     = Vpv - Vpv_prev
    Gpv      = Ipv / Vpv                      # instantaneous conductance
    if ΔVpv ≠ 0:
        Ginc = ΔIpv / ΔVpv                    # incremental conductance
    else:
        Ginc = 0
    return Ppv, Ppv_prev, ΔPpv, ΔIpv, ΔVpv, Gpv, Ginc
end function


function update_search_space_using_MPC():
    # Apply Isc_STC as temporary reference current to MPC
    Iref_tmp = Isc_STC

    # Use MPC to drive real Ipv to the short-circuit current of the current PSC
    repeat
        Ipv_now, Vpv_now, Vout_now = measure_PV_signals()
        S_opt = MPC_switch_decision(Ipv_now, Vpv_now, Vout_now, Iref_tmp)
        apply_switch_S(S_opt)
    until PV_current_is_steady()

    I_sc_PSC = measured_PV_current()   # steady-state short-circuit current
    search_space = [0, I_sc_PSC]
end function


function initialize_particles_modified(search_space, N_particles):
    # Implements the mixed equal + random distribution (grid partition + randomness)
    # Positions Xi(0) are in current range search_space = [0, Imax]
    # Velocities Vi(0) typically set to zero or small random values
    ...
end function


function PSO_iteration(particles, search_space):
    # Standard PSO position/velocity update using power-based fitness
    # For each particle i:
    #   - position Xi = candidate current Iref_candidate
    #   - evaluate fitness as expected PV power at that current
    #   - update Pbest_i, Gbest, and velocity/position
    ...
    return updated_particles, Gbest_current
end function


function MPC_switch_decision(Ipv_k, Vpv_k, Vout_k, Iref_k):
    # Discrete model (S = 0 or 1)
    # Predict Ipv(k+1) for both S = 0 and S = 1
    for S in {0, 1}:
        Ipv_pred[S] = Ipv_k + (D * Ts_MPC / L1) * (Vpv_k * S - Vout_k)
        g[S] = abs(Ipv_pred[S] - Iref_k)
    end for
    if g[1] < g[0]:
        return 1
    else:
        return 0
end function

# Initial measurements
Vpv_prev, Ipv_prev, Vout_prev = initial_measurements()

# Initial search space and particle population
search_space = [0, Isc_STC]
particles    = initialize_particles_modified(search_space, N_particles)

Mode           = PSO_MODE
PSO_converged  = false
Gbest_current  = some_initial_Iref_from_particles()
Iref           = Gbest_current


while TRUE:

    # 1) Measure present PV signals
    Vpv, Ipv, Vout = measure_PV_signals()

    # 2) Compute PV power and conductance-related quantities
    Ppv, Ppv_prev, ΔPpv, ΔIpv, ΔVpv, Gpv, Ginc =
        compute_PV_quantities(Vpv, Ipv, Vpv_prev, Ipv_prev)

    # -----------------------------------------------
    # 3) Detect large power change (PSC / irradiance step)
    # -----------------------------------------------
    if abs(ΔPpv) > P_change_th * max(Ppv_prev, small_number):

        # -> Rebuild search space and reinitialize PSO
        update_search_space_using_MPC()
        particles   = initialize_particles_modified(search_space, N_particles)
        Mode        = PSO_MODE
        PSO_converged = false
    end if

    # -----------------------------------------------
    # 4) Global search using modified PSO
    # -----------------------------------------------
    if Mode == PSO_MODE:

        # One iteration of modified PSO over the current search space
        particles, Gbest_current = PSO_iteration(particles, search_space)

        # Use the current global best particle as reference current
        Iref = Gbest_current

        # Check PSO convergence (max_iter, small position spread, etc.)
        if PSO_convergence_condition(particles):
            PSO_converged = true
            Mode = IC_MODE          # switch to IC refinement
        end if

    # -----------------------------------------------
    # 5) Local refinement using Incremental Conductance
    # -----------------------------------------------
    else if Mode == IC_MODE:

        # Classic IC logic on current reference current Iref
        # Compare incremental and instantaneous conductance
        if ΔVpv == 0 and ΔIpv == 0:
            # At or very near MPP → keep Iref
            Iref = Iref

        else:
            # slope comparison: Ginc ? -Gpv
            if Ginc > -Gpv:
                # Left of MPP → decrease current reference
                Iref = Iref - ΔIref_step
            elseif Ginc < -Gpv:
                # Right of MPP → increase current reference
                Iref = Iref + ΔIref_step
            else:
                # Ginc ≈ -Gpv → at MPP
                Iref = Iref
            end if
        end if
    end if  # Mode selection

    # -----------------------------------------------
    # 6) MPC: choose optimal switch state to track Iref
    # -----------------------------------------------
    S_opt = MPC_switch_decision(Ipv, Vpv, Vout, Iref)
    apply_switch_S(S_opt)

    # -----------------------------------------------
    # 7) Update stored values for next iteration
    # -----------------------------------------------
    Vpv_prev  = Vpv
    Ipv_prev  = Ipv
    Ppv_prev  = Ppv

    wait(Ts_MPC)     # advance to next sampling instant

end while

