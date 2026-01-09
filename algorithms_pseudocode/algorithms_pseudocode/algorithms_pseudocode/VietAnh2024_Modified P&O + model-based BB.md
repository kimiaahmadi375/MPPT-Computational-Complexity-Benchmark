Vpv(k), Ipv(k)              # Measured PV voltage & current
SamplingPeriod               # MPPT update interval


N                           # Window length for preprocessing (MP window)
AD = 0.015                  # Initial perturbation magnitude
D0 = 0.1                    # Initial duty cycle
Dref_init = 0.8             # Initial reference duty
εP                          # Power threshold
εB                          # Loss threshold


i = 1
Voc_avg = measure_avg_open_circuit_voltage()
Isc[1] = measure_short_circuit_current()
Ve = Voc_avg / N

# Compute initial MP values
Vmp[1], Imp[1], Pmp[1], Dmp[1] = compute_initial_MP_values()

i = 2
Rpv = compute_Rpv()


while TRUE:

    # Compute duty cycle for this step
    D[i] = compute_D(i)

    # Measure PV values
    Ipv = measure_current()
    Vpv = measure_voltage()

    # Check primary Voc-based condition
    cond1 = (Voc_avg - Ve*(2 - i) < Vpv) AND (Vpv < Voc_avg - Ve)

    if cond1 == TRUE:

        # Update Rpv
        Rpv = update_Rpv()

        # Compute MP-related variables
        B = compute_BB()
        Vmp[i], Imp[i], Pmp[i], Dmp[i] = compute_MP_arrays()

        i = i + 1

        if i > (N + 1):
            goto Select_MPP
        else:
            continue      # Continue identification loop
    end if


    # Condition failed → evaluate secondary constraints
    cond2 = (Voc_avg - Ve*(2 - i) > Vpv) AND
            (Isc[i] < Isc[i - 1]) AND
            (abs(Isc[i] - Ipv) <= threshold)

    if cond2 == TRUE:
        goto Select_MPP
    else:
        # Update Isc and MP arrays
        Isc[i] = Ipv
        Vmp[i], Imp[i], Pmp[i], Dmp[i] = compute_MP_arrays()
        i = i + 1

        if i > (N + 1):
            goto Select_MPP
        else:
            continue
    end if
end while

Select_MPP:

Pref = max(Pmp[1 ... i-1])
idx  = index_of(Pref)

Dref = Dmp[idx]
Vref = Vmp[idx]

# Apply final duty cycle from identification
D = Dref

# Final measurement before tracking
I0 = measure_current()
V0 = measure_voltage()

goto Tracking_Loop


Tracking_Loop:

while TRUE:

    # Apply perturbation
    Dn = Dref + AD

    # Compute instantaneous power
    P = Vpv * Ipv
    ΔP = P - Pref
    ΔV = Vpv - Vref


    if ΔP > εP:
        if ΔV > 0:
            Dn = decrease_Dn(Dn)
        else:
            Dn = increase_Dn(Dn)

    elseif ΔP < -εP:
        if ΔV > 0:
            Dn = increase_Dn(Dn)
        else:
            Dn = decrease_Dn(Dn)

    else:
        # ΔP small → evaluate power loss
        Ploss = compute_Ploss(P, Pref)

        if Ploss < εB:
            continue      # stay in tracking loop
        else:
            break         # exit MPPT
    end if


    Pref = P
    Vref = Vpv
    Iref = Ipv

    # Saturate duty cycle
    Dn = saturate(Dn, Dmin, Dmax)

    D = Dn

end while

