using PowerSystems
using PowerSimulationsDynamics

const PSY = PowerSystems
const PSID = PowerSimulationsDynamics

function create_ZIPE_load(sys, load_params)

    for l in collect(get_components(PSY.StandardLoad, sys))

        p_tot, q_tot = _compute_total_load_parameters(l)

        l.impedance_active_power = p_tot * load_params.z_percent
        l.impedance_reactive_power = q_tot * load_params.z_percent

        l.current_active_power = p_tot * load_params.i_percent
        l.current_reactive_power = q_tot * load_params.i_percent

        l.constant_active_power = p_tot * load_params.p_percent
        l.constant_reactive_power = q_tot * load_params.p_percent

        # Define a GFL inverter and set load setpoint to p_tot/q_tot * load_params.e_percent
        
        if load_params.e_percent > eps()
            static_inverter = create_static_inverter(l.bus, p_tot, q_tot, load_params)
            add_component!(sys, static_inverter)

            GFL_inverter = create_GFL_inverter(static_inverter)
            add_component!(sys, GFL_inverter, static_inverter)
        end
    end
end

function create_static_inverter(bus, p_tot, q_tot, load_params)
    static_inverter = RenewableDispatch(
        name = "load_GFL_inverter"*string(bus.number),
        available = true,
        bus = bus,
        active_power = - p_tot * load_params.e_percent,
        reactive_power = - q_tot * load_params.e_percent,
        rating = 99.00409082457148,
        prime_mover_type = PrimeMovers.ES,
        power_factor = p_tot/sqrt(p_tot^2 + q_tot^2),
        operation_cost = TwoPartCost(
            variable = VariableCost(
                (0.0,
                0.0)
            ),
            fixed = 0.0,
        ),
        reactive_power_limits = (min=-100.0, max=100.0),
        base_power = 100.00
        )
    return static_inverter
end

function create_GFL_inverter(stat_inv)
    case_inv = DynamicInverter(
            stat_inv.name, 
            1.0, # ω_ref,
            converter_high_power(), #converter
            GFL_outer_control(), #outer control
            GFL_inner_control(), #inner control voltage source
            dc_source_lv(), #dc source
            pll(), #pll
            filt(), #filter
        )
    return case_inv
end

#Define converter as an AverageConverter
converter_high_power() = AverageConverter(
    rated_voltage = 138.0, 
    rated_current = 100.0
)

GFL_outer_control() = OuterControl(
    ActivePowerPI(Kp_p = 0.0059 , Ki_p = 7.36, ωz = 1000.0, P_ref = 0.5),
    ReactivePowerPI(Kp_q = 0.0059, Ki_q = 7.36, ωf = 1000.0, V_ref = 1.0, Q_ref = 0.1)
)

GFL_inner_control() = CurrentModeControl(
    kpc = 1.27,     #Current controller proportional gain
    kic = 14.3,     #Current controller integral gain
    kffv = 0.0,     #Binary variable enabling the current feed-forward in output of current controllers
)

#Define DC Source as a FixedSource:
dc_source_lv() = FixedDCSource(
    voltage = 600.0
    )

#Define a Frequency Estimator as a PLL based on Vikram Kaura and Vladimir Blaskoc 1997 paper:
pll() = KauraPLL(
    ω_lp = 500.0, #Cut-off frequency for LowPass filter of PLL filter.
    kp_pll = 0.084,  #PLL proportional gain
    ki_pll = 4.69,   #PLL integral gain
)

#Define an LCL filter:
filt() = LCLFilter(
    lf = 0.08, 
    rf = 0.003, 
    cf = 0.074, 
    lg = 0.2, 
    rg = 0.01
)

function _compute_total_load_parameters(load::PSY.StandardLoad)
    # Constant Power Data
    constant_active_power = PSY.get_constant_active_power(load)
    constant_reactive_power = PSY.get_constant_reactive_power(load)
    max_constant_active_power = PSY.get_max_constant_active_power(load)
    max_constant_reactive_power = PSY.get_max_constant_reactive_power(load)
    
    # Constant Current Data
    current_active_power = PSY.get_current_active_power(load)
    current_reactive_power = PSY.get_current_reactive_power(load)
    max_current_active_power = PSY.get_max_current_active_power(load)
    max_current_reactive_power = PSY.get_max_current_reactive_power(load)
    
    # Constant Admittance Data
    impedance_active_power = PSY.get_impedance_active_power(load)
    impedance_reactive_power = PSY.get_impedance_reactive_power(load)
    max_impedance_active_power = PSY.get_max_impedance_active_power(load)
    max_impedance_reactive_power = PSY.get_max_impedance_reactive_power(load)
   
    # Total Load Calculations
    active_power = constant_active_power + current_active_power + impedance_active_power
    reactive_power = constant_reactive_power + current_reactive_power + impedance_reactive_power
    max_active_power = max_constant_active_power + max_current_active_power + max_impedance_active_power
    max_reactive_power = max_constant_reactive_power + max_current_reactive_power + max_impedance_reactive_power
    
    return active_power, reactive_power, max_active_power, max_reactive_power
end