"""
See [1, "Online Implementation of IRL: A Hybrid Optimal Adaptive Controller"].

# Refs
[1] “Reinforcement Learning and Feedback Control: Using Natural Decision Methods to Design Optimal Adaptive Controllers,” IEEE Control Syst., vol. 32, no. 6, pp. 76–105, Dec. 2012, doi: 10.1109/MCS.2012.2214134.

# Notes
- T: Data stack period
- N: The maximum length of stacked data
"""
mutable struct LinearIRL <: AbstractIRL
    Q::AbstractMatrix
    R_inv::AbstractMatrix
    B::AbstractMatrix
    T::Real
    N::Int
    i::Int
    i_init::Int
    function LinearIRL(Q::AbstractMatrix, R::AbstractMatrix, B::AbstractMatrix;
            T=0.04,
            N=nothing,
            i_init=0,  # iteration number
        )
        n1, n2 = size(Q)
        @assert n1 == n2
        N_min = Int(n1*(n1 + 1)/2)
        if N == nothing
            N = N_min
        else
            @assert N >= N_min
        end
        @assert T > 0 && N > 0
        R_inv = inv(R)
        i = i_init
        new(Q, R_inv, B, T, N, i, i_init)
    end
end

function reset!(irl::LinearIRL)
    irl.i = irl.i_init
end

"""
Value iteration [1, Eq. 99]; updated in least-square sense
# Notes
w: critic parameter (vectorised)
- ϕs_prev: the vector of bases (evaluated)
- V̂: the vector of approximate values (evaluated)
"""
function value_iteration!(irl::LinearIRL, buffer::DataBuffer, w)
    @unpack i, N = irl
    @unpack data_array = buffer
    data_filtered = filter(x -> x.i == i, data_array)  # data from the current policy
    if length(data_filtered) >= N + 1
        data_sorted = sort(data_filtered, by=x -> x.t)  # sort by time index
        ϕs_prev = data_sorted[end-N:end-1] |> Map(datum -> datum.ϕ) |> collect
        # V̂s = data_sorted[end-(N-1):end] |> Map(datum -> datum.V̂) |> collect
        xs = data_sorted[end-(N-1):end] |> Map(datum -> datum.x) |> collect
        ∫rs = data_sorted[end-(N-1):end] |> Map(datum -> datum.∫r) |> collect
        P = convert_to_matrix(w)
        V̂s_with_prev_P = xs |> Map(x -> value(irl, P, x)) |> collect
        V̂s = ∫rs .+ V̂s_with_prev_P
        # update the critic parameter
        w .= ( hcat(V̂s...) * pinv(hcat(ϕs_prev...)) )'[:]  # to reduce the computation time; [:] for vectorisation
        # w .= pinv(hcat(ϕs_prev...)') * hcat(V̂s...)'  # least square sense
        update_index!(irl)
    end
end

"""
Policy iteration [1, Eq. 98]; updated in least-square sense
# Notes
w: critic parameter (vectorised)
- Δϕs: the vector of (ϕ - ϕ_prev) (evaluated)
- ∫rs: the vector of integral running cost by numerical integration (evaluated)
"""
function policy_iteration!(irl::LinearIRL, buffer::DataBuffer, w)
    @unpack i, N = irl
    @unpack data_array = buffer
    data_filtered = filter(x -> x.i == i, data_array)  # data from the current policy
    if length(data_filtered) >= N + 1
        data_sorted = sort(data_filtered, by=x -> x.t)  # sort by time index
        ϕs_prev_and_present = data_sorted[end-N:end] |> Map(datum -> datum.ϕ) |> collect
        Δϕs = diff(ϕs_prev_and_present)
        ∫rs = data_sorted[end-(N-1):end] |> Map(datum -> datum.∫r) |> collect
        # update the critic parameter
        w .= ( hcat(∫rs...) * pinv(hcat(-Δϕs...)) )'[:]  # to reduce the computation time; [:] for vectorisation
        # w .= pinv(hcat(-Δϕs...)') * hcat(∫rs...)'  # least square sense
        update_index!(irl)
    end
end

function update_index!(irl::LinearIRL)
    irl.i += 1
end

"""
Policy improvement [1, Eq. 96].
"""
function _optimal_input(R_inv, B, P, x)
    -0.5 * R_inv * B' * (2 * P * x)
end

function optimal_input(irl::LinearIRL, x, w::AbstractVector)
    @unpack R_inv, B = irl
    P = convert_to_matrix(w)
    _optimal_input(R_inv, B, P, x)
end

function optimal_input(irl::LinearIRL, x, P::AbstractMatrix)
    @unpack R_inv, B = irl
    _optimal_input(R_inv, B, P, x)
end

function value(irl::LinearIRL, P::AbstractMatrix, x)
    x' * P * x
end

function value(irl::LinearIRL, w::AbstractVector, x)
    P = convert_to_matrix(w)
    value(irl, P, x)
end

"""
w: critic parameter (vectorised)
"""
function Base.push!(buffer::DataBuffer, irl::LinearIRL, cost::AbstractCost;
        t, x, u, w,
    )
    # P = convert_to_matrix(w)
    @unpack data_array = buffer
    @unpack i = irl
    data_sorted = sort(data_array, by = x -> x.t)  # sorted by t
    # V̂_with_prev_P = value(irl, P, x)
    # prev data
    if length(data_sorted) != 0
        t_prev = data_sorted[end].t
        x_prev = data_sorted[end].x
        u_prev = data_sorted[end].u
        # numerical integration
        Δt = t - t_prev
        r = cost(x, u)
        r_prev = cost(x_prev, u_prev)
        ∫r = 0.5 * (r + r_prev) * Δt # trapezoidal
        # V̂ = ∫r + V̂_with_prev_P
    else
        # V̂ = missing
        ∫r = missing
    end
    ϕ = convert_quadratic_to_linear_basis(x)  # x'Px = w'ϕ(x)
    datum = (;
             t=t,
             x=x,
             u=u,
             w=w,  # logging
             ϕ=ϕ,
             ∫r=∫r,
             # V̂=V̂,
             i=i,  # iteration number
            )
    push!(buffer.data_array, datum)
end
