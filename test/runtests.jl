using DataDrivenControl
using Test

@testset "DataDrivenControl.jl" begin
    include("quadratic_cost.jl")
    include("irl.jl")
end
