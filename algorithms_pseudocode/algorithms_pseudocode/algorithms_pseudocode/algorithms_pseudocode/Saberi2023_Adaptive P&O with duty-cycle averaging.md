# -------------------------------------------------------------
# Saberi2023 – Improved P&O MPPT with:
#   • Irradiance change identifier
#   • Steady-state determiner
#   • Duty-cycle selector (3-sample averaging)
#   • Step-size alternator
#   • Boundary-condition imposer
# -------------------------------------------------------------
# Interfaces
#   measure_voltage()  → Vpv(k)
#   measure_current()  → Ipv(k)
#   apply_duty(D)      → write duty cycle to converter
#   wait(Ts)           → MPPT sampling period

# Fixed parameters (from the paper)
ΔD_init   = 0.01      # initial perturbation step
ΔD_mid    = 0.015     # reduced step near MPP
ΔD_large  = 0.03      # larger step when far from MPP
ΔD_reirr  = 0.02      # step after irradiance change
ΔD_min    = 0.001     # smallest step at steady state
Osc_max   = 2         # number of swings to confirm MPP

# State / helper variables
Flag  = 1             # 1: still searching, 0: MPP located (steady state)
cntr  = 0             # counts β samples (0..3)
Osc   = 0             # counts oscillations around MPP
ΔD    = ΔD_init       # current step size

φ      = D_init       # present duty cycle command
φ_old  = φ            # duty used at previous step
φ_mpp  = φ            # stored best duty cycle at MPP

β      = 0            # sign(+1/−1) from ΔI
β_sum  = 0            # sum of 3 consecutive β
δ[1..3] = {φ, φ, φ}   # last 3 duty cycles for averaging

BH = +∞               # upper boundary around MPP
BL = −∞               # lower boundary around MPP
ΔPth = 0              # power threshold for irradiance change

# Previous sample values
Vold = measure_voltage()
Iold = measure_current()
Pold = Vold * Iold

apply_duty(φ)
wait(Ts)

# -------------------------------------------------------------
# Main MPPT loop
# -------------------------------------------------------------
loop forever:

    # 1) Measure present PV point
    Vpv = measure_voltage()
    Ipv = measure_current()
    Ppv = Vpv * Ipv

    # 2) Incremental changes
    ΔP = Ppv - Pold
    ΔV = Vpv - Vold
    ΔI = Ipv - Iold

    # ---------------------------------------------------------
    # 3) Irradiance change identifier (block 1 in the paper)
    #    Active only AFTER MPP is found (Flag = 0)
    # ---------------------------------------------------------
    if Flag == 0 then
        if abs(ΔP) > ΔPth then
            # Irradiance has changed → leave steady state
            Flag  = 1
            ΔD    = ΔD_reirr
            cntr  = 0
            Osc   = 0
            β_sum = 0

            # Choose tracking direction only from ΔI sign
            if ΔI > 0:
                α = +1          # irradiance increased → PP moved right
            else:
                α = -1          # irradiance decreased → PP moved left
            end if
        else
            # Still in steady state region; boundaries will hold φ
            # Direction α will be computed below but may be ignored
            pass
        end if
    end if

    # ---------------------------------------------------------
    # 4) Conventional P&O core (defines α from ΔP, ΔV)
    # ---------------------------------------------------------
    if ΔP > 0 then
        if ΔV > 0:
            α = -1
        else:
            α = +1
        end if
    else:   # ΔP < 0
        if ΔV > 0:
            α = +1
        else:
            α = -1
        end if
    end if

    # β is based only on ΔI sign (for steady-state detection)
    if ΔI > 0:
        β = +1
    else:
        β = -1
    end if

    # ---------------------------------------------------------
    # 5) Steady-state determiner & duty-cycle selector
    #    (blocks 2 & 3 – active while searching, Flag = 1)
    # ---------------------------------------------------------
    if Flag == 1 then

        cntr       = cntr + 1
        δ[cntr]    = φ_old          # store last duty
        β_sum      = β_sum + β

        if cntr == 3 then
            # After 3 samples → build candidate MPP duty
            φ_mpp = (δ[1] + δ[2] + δ[3]) / 3.0

            # -------------------------------------------------
            # 6) Step-size alternator (block 4)
            # -------------------------------------------------
            if abs(β_sum) == 3 then
                # All β signs identical → still moving toward MPP
                # → use larger step to speed convergence
                ΔD  = ΔD_large
                Osc = 0
            else
                # β signs not identical → around MPP
                # → reduce step size and count oscillations
                ΔD  = ΔD_mid
                Osc = Osc + 1

                if Osc >= Osc_max then
                    # -------------------------------------------------
                    # 7) Boundary condition imposer – MPP located
                    # -------------------------------------------------
                    Flag = 0
                    Osc  = 0
                    ΔD   = ΔD_min

                    φ_old = φ_mpp
                    φ     = φ_mpp

                    BH = φ_mpp + ΔD          # upper boundary
                    BL = φ_mpp - ΔD          # lower boundary

                    ΔPth = 0.05 * Ppv        # 5% of PV power at MPP
                end if
            end if

            # Reset 3-sample accumulators
            cntr   = 0
            β_sum  = 0
        end if
    end if

    # ---------------------------------------------------------
    # 8) Duty-cycle update + boundary condition
    # ---------------------------------------------------------
    φ_candidate = φ_old + α * ΔD

    if Flag == 0 then
        # When MPP is located, boundaries freeze duty cycle
        if (φ_candidate > BH) or (φ_candidate < BL) then
            φ_candidate = φ_old       # keep duty unchanged
        end if
    end if

    φ = φ_candidate

    # ---------------------------------------------------------
    # 9) Apply new duty and prepare for next iteration
    # ---------------------------------------------------------
    apply_duty(φ)

    Vold = Vpv
    Iold = Ipv
    Pold = Ppv
    φ_old = φ

    wait(Ts)

end loop
