using DataDrivenControl
using LinearAlgebra

@testset "quadratic_cost" begin
    x = ones(3)
    u = ones(3)
    Q = Matrix(I, 3, 3)
    R = Matrix(I, 3, 3)

    cost = DataDrivenControl.QuadraticCost(Q, R)
    r = cost(x, u)
    @test r == 6  # analytically obtained
end
