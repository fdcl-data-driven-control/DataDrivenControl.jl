module DataDrivenControl

using LinearAlgebra
using UnPack
using Transducers

export LinearIRL
export optimal_input


include("utils/utils.jl")
include("irl/irl.jl")


end
