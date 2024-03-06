using PowerSimulationsDynamics
using PowerSystems
using Parameters

@with_kw mutable struct LoadParams
    z_percent::Float64
    i_percent::Float64  
    p_percent::Float64
    e_percent::Float64
end