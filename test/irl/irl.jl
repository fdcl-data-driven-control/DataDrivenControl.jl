using DataDrivenControl
using LinearAlgebra


@testset "irl" begin
    @testset "linear_irl" begin
        # See [1, Example 2. Continuous-Time Optimal Adaptive Control Using IRL]
        # Refs
        # [1] “Reinforcement Learning and Feedback Control: Using Natural Decision Methods to Design Optimal Adaptive Controllers,” IEEE Control Syst., vol. 32, no. 6, pp. 76–105, Dec. 2012, doi: 10.1109/MCS.2012.2214134.
        n, m = 2, 1
        Q = Matrix(I, n, n)
        R = Matrix(I, m, m)
        B = [0 2]'
        irl = DataDrivenControl.LinearIRL(Q, R, B)
        x = rand(2)
        P = [0.0500 0.0039;
             0.0039 0.2085]  # true solution
        w = [P[1, 1], P[2, 1]+P[1, 2], P[2, 2]]
        @test DataDrivenControl.optimal_input(irl, x, w) ≈ - inv(R) * B' * P * x
        @test DataDrivenControl.value(irl, w, x) ≈ x' * P * x
        # data buffer
        ŵ = rand(Int(n*(n+1)/2))
        û = DataDrivenControl.optimal_input(irl, x, ŵ)
        buffer = DataDrivenControl.DataBuffer()
        cost = DataDrivenControl.QuadraticCost(Q, R)
        ts = 0:irl.T:1
        for t in ts
            push!(buffer, irl, cost; t=t, x=x, u=û, w=ŵ)
        end
        @test length(buffer.data_array) == length(ts)
        ŵ_new = deepcopy(ŵ)
        i = deepcopy(irl.i)
        DataDrivenControl.evaluate_policy!(irl, buffer, ŵ_new)
        @test ŵ_new != ŵ  # updated?
        @test i + 1 == irl.i
        DataDrivenControl.reset!(irl)
        @test irl.i == irl.i_init
    end
end
