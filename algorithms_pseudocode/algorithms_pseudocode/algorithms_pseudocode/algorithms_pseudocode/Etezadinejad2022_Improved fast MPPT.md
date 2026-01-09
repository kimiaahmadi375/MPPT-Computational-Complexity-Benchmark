# -------------------------------------------------------------
# Etezadinejad2022 – Improved Analytical GMPPT Algorithm
# -------------------------------------------------------------
# Interfaces (array-level measurements)
#   measure_voltage()  → V[k]
#   measure_current()  → I[k]
#   set_operating_voltage(Vref)  → adjusts duty so array voltage ≈ Vref
#   wait(Ts)           → MPPT sampling period

# Model / array parameters (from PV model & datasheet)
Ns, Np                 # series / parallel modules in array
Ni_full                # series cells per array (full string, last peak)
PV_params              # {Rs, Rp, a, Vt, …}
Voc_STC                # module open-circuit voltage at STC
epsilon_I = 0.05       # PSC current-change threshold (Eq. 10)
deltaV_OCV = 0.015     # 1.5% of array OCV used as perturbation step

# State variables
PSCFlag      = 0       # 0 = normal (UIC mode), 1 = PSC/GMPPT mode
k             = 0
I_prev        = 0
P_prev        = 0
V_op          = 0      # present operating voltage of array

# -------------------------------------------------------------
# Helper functions (analytical equations from the paper)
# -------------------------------------------------------------

function Vm = compute_Vm_UIC(PV_params, Voc_array, Ni, Np):
    # Implements Eq. (9): analytic Vm for given Ni under UIC
    # (Uses Iph, Is, Rs, Rp, a, Vt, etc.; omitted algebra here.)
    Vm = analytic_MPP_voltage(PV_params, Voc_array, Ni, Np)
    return Vm
end function

function Voc_new = update_Voc_for_zone(i, PV_params, Iph_zone[1..i]):
    # Implements Eqs. (13)–(16):
    #   – ΔVOC_i from irradiance change
    #   – surplus current effect ΔVSur_i
    #   – VOC_new_i = VOC_STC + ΔVOC_i + ΔVSur_i
    Voc_new = analytic_updated_OCV(i, PV_params, Iph_zone, Voc_STC)
    return Voc_new
end function

# -------------------------------------------------------------
# Subroutine: scan all local peaks and find GMPP (PSC mode)
# -------------------------------------------------------------

function [V_gmpp, P_gmpp] = scan_GMPPT_zones():

    # Assume there are Z zones / local peaks (Z = number of BDs)
    Z = number_of_possible_zones()

    for i = 1 .. Z:

        # 1) Choose Ni value for this zone (multiples of series modules)
        Ni = Ni_value_for_zone(i)

        # 2) Compute ideal MPP voltage for this zone (Eq. 9)
        Vmi = compute_Vm_UIC(PV_params, Voc_STC, Ni, Np)

        # 3) Apply Vmi to the array and measure operating point
        set_operating_voltage(Vmi)
        wait(Ts)

        V[i]   = measure_voltage()
        I[i]   = measure_current()
        P[i]   = V[i] * I[i]

        # 4) Compute Iph for this zone (from PV model, Eq. (1))
        Iph_zone[i] = estimate_Iph_from(V[i], I[i], PV_params)

    end for

    # 5) Determine GMPP zone: index of maximum power
    idx_gmpp = argmax_i P[i]
    P_gmpp   = P[idx_gmpp]

    # 6) Update OCV of the GMPP zone using Eqs. (13)–(16)
    Voc_gmpp = update_Voc_for_zone(idx_gmpp, PV_params, Iph_zone)

    # 7) Recompute analytical GMPP voltage with updated OCV (Eq. 9)
    Ni_gmpp  = Ni_value_for_zone(idx_gmpp)
    V_gmpp   = compute_Vm_UIC(PV_params, Voc_gmpp, Ni_gmpp, Np)

    return [V_gmpp, P_gmpp]
end function

# -------------------------------------------------------------
# Main MPPT procedure
# -------------------------------------------------------------

procedure Improved_GMPPT():

    # -------- Initialization --------
    k = 0
    PSCFlag = 0

    # Take initial measurement
    V_prev = measure_voltage()
    I_prev = measure_current()
    P_prev = V_prev * I_prev
    V_op   = V_prev

    loop forever:

        k = k + 1

        # --- 1. Measure present array point ---
        V_now = measure_voltage()
        I_now = measure_current()
        P_now = V_now * I_now

        # --- 2. First PSC / irradiance-change test (Eq. 10) ---
        ratio_I = abs(I_prev - I_now) / max(I_now, tiny_value)

        if ratio_I >= epsilon_I then
            # Significant change in current → either PSC or fast irradiance change

            if PSCFlag == 1 then
                # PSCFlag already raised: PSC likely removed → return to UIC mode
                PSCFlag = 0
                # Use base UIC analytic MPPT with full Ni
                Voc_array = Voc_STC      # reset to UIC OCV
                Vm_ref    = compute_Vm_UIC(PV_params, Voc_array, Ni_full, Np)
                set_operating_voltage(Vm_ref)

            else
                # PSC not yet flagged → perform second PSC confirmation test
                #  (Add ΔV = 1.5% OCV to OPV and see if power increases.)
                V_test = V_op + deltaV_OCV * Voc_STC
                set_operating_voltage(V_test)
                wait(Ts)

                V_t = measure_voltage()
                I_t = measure_current()
                P_t = V_t * I_t

                if P_t > P_now then
                    # Output power increased ⇒ PSC confirmed
                    PSCFlag = 1

                    # Enter GMPPT mode: scan all local peaks
                    [V_gmpp, P_gmpp] = scan_GMPPT_zones()

                    # Apply final GMPP voltage and update reference
                    set_operating_voltage(V_gmpp)
                    V_op  = V_gmpp
                    P_now = P_gmpp

                else
                    # No PSC: treat as UIC with irradiance change
                    PSCFlag   = 0
                    Voc_array = Voc_STC    # (or updated via temperature model)
                    Vm_ref    = compute_Vm_UIC(PV_params, Voc_array, Ni_full, Np)
                    set_operating_voltage(Vm_ref)
                    V_op = Vm_ref
                end if
            end if

        else
            # --- 3. No large current change ---
            if PSCFlag == 0 then
                # Normal UIC operation: use base analytical MPPT (Eq. 9)
                Voc_array = Voc_STC      # possibly adjusted with Eq. (8)
                Vm_ref    = compute_Vm_UIC(PV_params, Voc_array, Ni_full, Np)
                set_operating_voltage(Vm_ref)
                V_op = Vm_ref

            else
                # PSCFlag = 1 and no new big current jump:
                # stay at the previously found GMPP voltage V_op.
                # (If PSC structure changes again, ratio_I test will trigger
                #  another scan_GMPPT_zones() call.)
                set_operating_voltage(V_op)
            end if
        end if

        # --- 4. Save previous sample for next loop ---
        V_prev = V_now
        I_prev = I_now
        P_prev = P_now

        # --- 5. Wait for next sampling instant ---
        wait(Ts)

    end loop

end procedure
