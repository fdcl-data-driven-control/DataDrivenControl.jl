module DataDrivenControl

using LinearAlgebra
using UnPack
using Transducers

## Costs
export QuadraticInInputCost, QuadraticCost
## IRL
# Linear IRL
export LinearIRL, value_iteration!, policy_iteration!
export optimal_input


include("utils/utils.jl")
include("irl/irl.jl")


end
