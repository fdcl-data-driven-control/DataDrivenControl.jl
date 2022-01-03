module DataDrivenControl

using LinearAlgebra
using UnPack
using Transducers

## IRL
# Linear IRL
export LinearIRL, value_iteration!, policy_iteration!
export optimal_input

## Stop conditions
export DistanceStopCondition

## Costs
export QuadraticInInputCost, QuadraticCost


include("utils/utils.jl")
include("irl/irl.jl")


end
