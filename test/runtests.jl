using DataDrivenControl
using Test

@testset "DataDrivenControl.jl" begin
    include("quadratic_cost.jl")
    include("utils/utils.jl")
end
