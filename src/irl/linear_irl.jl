"""
See [1, "IRL Optimal Adaptive Control Using Value Iteration"].

# Refs
[1] “Reinforcement Learning and Feedback Control: Using Natural Decision Methods to Design Optimal Adaptive Controllers,” IEEE Control Syst., vol. 32, no. 6, pp. 76–105, Dec. 2012, doi: 10.1109/MCS.2012.2214134.

# Notes
- T: Data stack period
- N: The maximum length of stacked data
- ϕs_prev: the vector of bases (evaluated)
- V̂: the vector of approximate values (evaluated)
"""
struct LinearIRL <: AbstractIRL
    Q::Matrix
    R_inv::Matrix
    B::Matrix
    T::Real
    N::Int
    i::Int
    function LinearIRL(Q::Matrix, R::Matrix, B::Matrix;
            T=0.04,
            N=nothing,
        )
        @assert T > 0 && N > 0
        n1, n2 = size(Q)
        @assert n1 == n2
        N_min = n1*(n1 + 1)/2
        if N == nothing
            N = N_min
        else
            @assert N >= N_min
        end
        R_inv = inv(R)
        i = 0  # iteration number
        new(Q, R_inv, B, T, N, i)
    end
end

"""
Value iteration [1, Eq. 99]; updated in least-square sense
# Notes
w: critic parameter (vectorised)
"""
function evaluate_policy!(irl::LinearIRL, buffer::DataBuffer, w,
    )
    @unpack i, N = irl
    @unpack data_array = buffer
    data_filtered = filter(x -> x.i == i, data_array)  # data from the current policy
    if length(data_filtered) >= N
        data_sorted = sort(data_filtered, by=x -> x.t)  # sort by time index
        ϕs_prev = data_sorted[end-N, end-1] |> Map(datum -> datum.ϕ) |> collect
        V̂s = data_sorted[end-(N-1), end] |> Map(datum -> datum.V̂) |> collect
        irl.i += 1  # update iteration number
        # update the critic parameter
        w .= ( hcat(V̂...) * pinv(hcat(ϕs_prev...)) )'  # to reduce the computation time
        # w .= pinv(hcat(ϕs_prev...)') * hcat(V̂...)'  # least square sense
    end
end

"""
Policy improvement [1, Eq. 96].
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

"""
w: critic parameter (vectorised)
"""
function push!(buffer::DataBuffer, irl::LinearIRL, cost::AbstractCost;
        t, x, u, w,
    )
    P = convert_to_matrix(w)
    @unpack data_array = buffer
    @unpack i = irl
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
    ϕ = convert_quadratic_to_linear_basis(x)  # x'Px = w'ϕ(x)
    datum = (;
             t=t,
             x=x,
             u=u,
             w=w,  # logging
             ϕ=ϕ,
             V̂=V̂,
             i=i,  # iteration number
            )
    push!(buffer, datum)
end
