using DataDrivenControl
using FlightSims
using UnPack
using Plots
using LinearAlgebra
using Transducers
using LaTeXStrings
using DiffEqCallbacks


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
        @onlylog i = controller.i
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
        controller = DataDrivenControl.LinearIRL(Q, R, B; T=0.04)
        env = LinearSystem_ZOH_Gain(linear, controller)
        x0 = State(env)(rand(2))
        # TODO: add callback to update param `w`
        # simulation
        P_true = [0.0500 0.0039;
                  0.0039 0.2085]  # true solution
        w_true = [P_true[1, 1], P_true[2, 1]+P_true[1, 2], P_true[2, 2]]
        scale = 0.1
        w0 = w_true + scale*randn(3)  # perturbed initial guess
        tf = 5.0
        simulator = Simulator(x0, Dynamics!(env), w0;
                              tf=tf,
                             )
        Δt = 0.01
        # TODO: data buffer is a duplicate of saving callback (see `df`)
        # data buffer
        buffer = DataDrivenControl.DataBuffer()
        function update!(integrator)
            t = integrator.t
            x = integrator.u  # convention of DifferentialEquations.jl
            w = integrator.p  # convention of DifferentialEquations.jl
            u = optimal_input(controller, x, w)  # TODO: not to be duplicate of control input in dynamics for stable coding
            push!(buffer, controller, cost;
                  t=t, x=copy(x), u=copy(u), w=copy(w))
            eps = 1e-1
            sc = DistanceStopCondition(eps)
            # value_iteration!(controller, buffer, w; sc=sc)
            policy_iteration!(controller, buffer, w; sc=sc)
        end
        cb_irl = PeriodicCallback(update!, controller.T; initial_affect=true)  # stack initial data
        df = solve(simulator;
                   callback=cb_irl,
                   savestep=Δt,
                  )
        # plot
        ts = df.time
        xs = df.sol |> Map(datum -> datum.state) |> collect
        us = df.sol |> Map(datum -> datum.input) |> collect
        ws = df.sol |> Map(datum -> datum.param) |> collect
        is = df.sol |> Map(datum -> datum.i) |> collect
        fig_x = plot(ts, hcat(xs...)'; label=[L"x_{1}" L"x_{2}"], legend=:outerbottomright)
        fig_u = plot(ts, hcat(us...)'; label=L"u", legend=:outerbottomright)
        fig_w = plot(ts, hcat(ws...)'; label=[L"w_{1}" L"w_{2}" L"w_{3}"], legend=:outerbottomright)
        fig_iter = plot(ts, hcat(is...)'; label="iter")
        ws_true = ts |> Map(t -> w_true) |> collect
        plot!(fig_w, ts, hcat(ws_true...)'; color=:black, label=nothing)
        fig = plot(fig_x, fig_u, fig_w, fig_iter)
end
