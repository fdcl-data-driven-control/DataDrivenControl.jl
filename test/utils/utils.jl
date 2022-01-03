using Test
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
    @testset "cost" begin
        x = ones(3)
        u = ones(3)
        Q = Matrix(I, 3, 3)
        R = 2*Matrix(I, 3, 3)

        quadratic_cost = DataDrivenControl.QuadraticCost(Q, R)
        r = quadratic_cost(x, u)
        @test r == x'*Q*x + u'*R*u  # analytically obtained
        Q_func(x) = x'*Q*x
        R_func(x) = R
        quadratic_in_input_cost = DataDrivenControl.QuadraticInInputCost(Q_func, R_func)
        @test r == quadratic_in_input_cost(x, u)
    end
end
