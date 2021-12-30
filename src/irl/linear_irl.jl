"""
# Refs
[1] “Reinforcement Learning and Feedback Control: Using Natural Decision Methods to Design Optimal Adaptive Controllers,” IEEE Control Syst., vol. 32, no. 6, pp. 76–105, Dec. 2012, doi: 10.1109/MCS.2012.2214134.
# Notes
- T: Data stack period
- N: The maximum length of stacked data
"""
struct LinearIRL <: AbstractIRL
    Q::Matrix
    R_inv::Matrix
    B::Matrix
    T::Real
    N::Int
    function LinearIRL(Q, R, B; T=0.04, N=3)
        @assert T > 0 && N > 0
        R_inv = inv(R)
        new(Q, R_inv, B, T, N)
    end
end

"""
Value iteration [1, Eq. 99]; updated in least-square sense
# Notes
w: critic parameter (vectorised)
ϕs: the vector of bases (evaluated)
V̂_nexts: the vector of approximate values (evaluated)
"""
function update!(irl::LinearIRL, w, ϕs::AbstractArray, V̂_nexts::AbstractArray,
    )
    error("TODO")
    @unpack N = irl
    if length(V̂_nexts) >= N
        w .= pinv(hcat(ϕs[end-irl.N:end-1]...)') * hcat(V̂_nexts...)'  # least square sense
    end
end

"""
Policy evaluation [1, Eq. 96].
"""
function _optimal_input(R_inv, B, P, x)
    -0.5 * R_inv * B' * (2 * P * x)
end

function optimal_input(irl::LinearIRL, x, w::Vector)
    @unpack R_inv, B = irl
    P = convert_to_matrix(w)
    _optimal_input(R_inv, B, P, x)
end

function optimal_input(irl::LinearIRL, x, P::Matrix)
    @unpack R_inv, B = irl
    _optimal_input(R_inv, B, P, x)
end

function value(irl::LinearIRL, P::Matrix, x)
    x' * P * x
end

function value(irl::LinearIRL, w::Vector, x)
    P = convert_to_matrix(w)
    value(irl, P, x)
end

function push!(buffer::DataBuffer, irl::LinearIRL, cost::AbstractCost, P::Matrix;
        t, x, u,
    )
    @unpack data_array = buffer
    data_sorted = sort(data_array, by = x -> x.t)  # sorted by t
    V̂_with_prev_P = value(irl, P, x)
    # prev data
    t_prev = data_sorted[end].t
    x_prev = data_sorted[end].x
    u_prev = data_sorted[end].u
    # numerical integration
    Δt = t - t_prev
    r = cost(x, u)
    r_prev = cost(x_prev, u_prev)
    ∫r = 0.5 * (r + r_prev) * Δt # trapezoidal
    V̂ = ∫r + V̂_with_prev_P
    datum = (;
             t=t,
             x=x,
             u=u,
             V̂=V̂,
            )
    push!(buffer, datum)
end
