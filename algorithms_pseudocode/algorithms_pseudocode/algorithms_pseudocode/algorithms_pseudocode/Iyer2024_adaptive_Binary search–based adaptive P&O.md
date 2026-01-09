Vpv(n), Ipv(n)           # Measured PV voltage and current
Ts_mppt                 # MPPT update period


Pth                      # Large positive dP threshold
Pth1                     # Small positive/negative dP threshold (near MPP)
δV                       # Small adaptive voltage step (near MPP)
Vref_init                # Initial reference voltage


Ppv_prev   = 0
Vpv_prev   = 0
Vref_prev  = 0
Vref_prev2 = 0           # V*(n-2)
first_run  = TRUE


while TRUE:

    # 1) Measure present voltage & current
    Vpv = measure_voltage()
    Ipv = measure_current()

    # 2) Compute power and increments (Eq. 1)
    Ppv = Vpv * Ipv
    dP  = Ppv - Ppv_prev
    dV  = Vpv - Vpv_prev


    if first_run == TRUE:
        Vref = Vref_init
        first_run = FALSE
        goto Update_Registers
    end if

    if Vref_prev == 0:
        Vref = Vref_init
        goto Update_Registers
    end if


    if Vref_prev2 == 0:
        Vref = Vref_init / 2
        goto Update_Registers
    end if


    if dP > Pth:

        if dV > 0:
            # Increase aggressively (Eq. 3)
            Vref = Vref_prev * 1.05
        else:
            Vref = Vref_prev * 0.95
        end if

        goto Update_Registers
    end if

    if dP > Pth1:

        if dV > 0:
            Vref = Vref_prev + δV
        else:
            Vref = Vref_prev - δV
        end if

        goto Update_Registers
    end if

    if dP < -Pth:
        Vref = (Vref_prev + Vref_prev2) / 2
        goto Update_Registers
    end if


    if dP < -Pth1:

        if dV > 0:
            Vref = Vref_prev - δV
        else:
            Vref = Vref_prev + δV
        end if

        goto Update_Registers
    end if
    # dP within deadband: stop perturbation
    Vref = Vref_prev


Update_Registers:

    Ppv_prev   = Ppv
    Vpv_prev   = Vpv
    Vref_prev2 = Vref_prev
    Vref_prev  = Vref

    apply_reference_voltage(Vref)

    wait(Ts_mppt)
end while


