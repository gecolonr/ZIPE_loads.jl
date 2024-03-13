cd(@__DIR__)

using ZIPE_loads
using PowerSystems
using PowerSimulationsDynamics
using Revise

const PSY = PowerSystems
const PSID = PowerSimulationsDynamics

file_name = "../data/9bus_allgens.json"
sys = System(joinpath(pwd(), file_name))

load_params = LoadParams(
    z_percent = 0.8,
    i_percent = 0.05,
    p_percent = 0.05,
    e_percent = 0.1
)

create_ZIPE_load(sys, load_params)