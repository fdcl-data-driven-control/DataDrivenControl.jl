module DataDrivenControl

using LinearAlgebra
using UnPack
using Transducers

## IRL
# Linear IRL
export LinearIRL, evaluate_policy!
export optimal_input


include("utils/utils.jl")
include("irl/irl.jl")


end
