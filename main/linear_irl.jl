using DataDrivenControl
using FlightSims
using UnPack
using Plots
using LinearAlgebra
using Transducers
using LaTeXStrings


struct LinearSystem_ZOH_Gain
    linear::LinearSystem
    controller::LinearIRL
end

function FlightSims.State(env::LinearSystem_ZOH_Gain)
    @unpack linear = env
    State(linear)
end

function FlightSims.Dynamics!(env::LinearSystem_ZOH_Gain)
    @unpack linear, controller = env
    @Loggable function dynamics!(dx, x, w, t)
        u = optimal_input(controller, x, w)
        @onlylog param = w
        @nested_log Dynamics!(linear)(dx, x, w, t; u=u)
    end
end

# See [1, Example 2. Continuous-Time Optimal Adaptive Control Using IRL]
# Refs
# [1] “Reinforcement Learning and Feedback Control: Using Natural Decision Methods to Design Optimal Adaptive Controllers,” IEEE Control Syst., vol. 32, no. 6, pp. 76–105, Dec. 2012, doi: 10.1109/MCS.2012.2214134.
function main()
        n, m = 2, 1
        A = [   -10  1;
             -0.002 -2]
        B = [0 2]'
        Q = Matrix(I, n, n)
        R = Matrix(I, m, m)
        cost = DataDrivenControl.QuadraticCost(Q, R)
        linear = LinearSystem(A, B)
        controller = DataDrivenControl.LinearIRL(Q, R, B)
        env = LinearSystem_ZOH_Gain(linear, controller)
        x0 = State(env)(rand(2))
        # TODO: add callback to update param `w`
        # simulation
        P = [0.0500 0.0039;
             0.0039 0.2085]  # true solution
        w = [P[1, 1], P[2, 1]+P[1, 2], P[2, 2]]  # true solution
        tf = 10.0
        simulator = Simulator(x0, Dynamics!(env), w;
                              tf=tf,
                             )
        Δt = controller.T
        df = solve(simulator; savestep=Δt)
        # plot
        ts = df.time
        xs = df.sol |> Map(datum -> datum.state) |> collect
        us = df.sol |> Map(datum -> datum.input) |> collect
        ws = df.sol |> Map(datum -> datum.param) |> collect
        fig_x = plot(ts, hcat(xs...)'; label=[L"x_{1}" L"x_{2}"])
        fig_u = plot(ts, hcat(us...)'; label=L"u")
        fig_w = plot(ts, hcat(ws...)'; label=[L"w_{1}" L"w_{2}" L"w_{3}"])
        fig = plot(fig_x, fig_u, fig_w)


        # # data buffer
        # buffer = DataDrivenControl.DataBuffer()
        # # TODO
        # ts = 0:irl.T:1
        # for t in ts
        #     push!(buffer, irl, cost; t=t, x=x, u=û, w=ŵ)
        # end
        # @test length(buffer.data_array) == length(ts)
        # ŵ_new = deepcopy(ŵ)
        # i = deepcopy(irl.i)
        # DataDrivenControl.evaluate_policy!(irl, buffer, ŵ_new)
        # @test ŵ_new != ŵ  # updated?
        # @test i + 1 == irl.i
        # DataDrivenControl.reset!(irl)
        # @test irl.i == irl.i_init
end
