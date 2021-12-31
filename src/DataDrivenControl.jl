module DataDrivenControl

using LinearAlgebra
using UnPack
using Transducers

## Costs
export QuadraticInInputCost, QuadraticCost
export LinearIRL
export optimal_input


include("utils/utils.jl")
include("irl/irl.jl")


end
