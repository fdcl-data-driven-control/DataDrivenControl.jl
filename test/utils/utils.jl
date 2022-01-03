using DataDrivenControl
using LinearAlgebra


@testset "utils" begin
    @testset "convert" begin
        # convert_to_matrix
        n = 2
        w = rand(Int(n*(n+1)/2))
        P = DataDrivenControl.convert_to_matrix(w)
        @test P == [w[1] 0.5*w[2]; 0.5*w[2] w[3]]
        # convert_quadratic_to_linear_basis
        x = rand(n)
        w' * DataDrivenControl.convert_quadratic_to_linear_basis(x) == x' * P * x
    end
    @testset "quadratic_cost" begin
        x = ones(3)
        u = ones(3)
        Q = Matrix(I, 3, 3)
        R = Matrix(I, 3, 3)

        cost = DataDrivenControl.QuadraticCost(Q, R)
        r = cost(x, u)
        @test r ≈ norm(x)^2 + norm(u)^2  # analytically obtained
    end
end
