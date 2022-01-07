"""
[1, Section IV.B]
# Refs
[1] T. Bian and Z.-P. Jiang, “Reinforcement Learning and Adaptive Optimal Control for Continuous-Time Nonlinear Systems: A Value Iteration Approach,” IEEE Trans. Neural Netw. Learning Syst., pp. 1–10, 2021, doi: 10.1109/TNNLS.2020.3045087.
[2] T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.
"""
struct CTVIADP <: AbstractEnv  # from FlightSims
    ϕs
    ψs
    K_ϕ
    K_ψ
    buffer::DataBuffer
end

function State(env::CTVIADP)
    @unpack ϕs = env
    N_ϕ = length(ϕs)
    return function (w=zeros(N_ϕ))
        w
    end
end

function Dynamics!(env::CTVIADP)
    error("Complete this")
    @unpack ϕs, K_ϕ, K_ψ, buffer = env
    @unpack data_array = buffer
    ts = data_array |> Map(datum -> datum.t) |> collect
    xs = data_array |> Map(datum -> datum.x) |> collect
    Φs = xs |> Map(x ->
                   (ϕs |> Map(ϕ -> ϕ(x)) |> collect)  # Φ
                  ) |> collect
    K_ϕ_inv = inv(K_ϕ)
    K_ψ_inv = inv(K_ψ)
    function dynamics!(dw, w, p, t)
        error("Complete this")
        # ĉ = K_ψ_inv * integrate(ts, blahblah)  # Eq. (14)
        # dw .= K_ϕ_inv *   # Eq. (13)
    end
end

function value_iteration!(env::CTVIADP, w)
    error("Complete this")
    error("Add a new stop cond")
end

function CTVIADP_Simulator(env::CTVIADP)
    error("Complete this")
    simulator = Simulator()
end
