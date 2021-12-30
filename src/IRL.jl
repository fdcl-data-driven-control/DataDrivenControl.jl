abstract type AbstractIRL end

"""
# Refs
[1] “Reinforcement Learning and Feedback Control: Using Natural Decision Methods to Design Optimal Adaptive Controllers,” IEEE Control Syst., vol. 32, no. 6, pp. 76–105, Dec. 2012, doi: 10.1109/MCS.2012.2214134.
# Notes
- T: Data stack period
- N: The maximum length of stacked data
"""
struct LinearIRL <: AbstractIRL where T <: Number
    Q::Matrix{T}
    R_inv::Matrix{T}
    T::Real
    N::Int
    function LinearIRL(Q, R; T=0.04, N=3)
        @assert T > 0 && N > 0
        R_inv = inv(R)
        new(Q, R_inv)
    end
end

"""
Value iteration [1, Eq. 99]; updated in least-square sense
# Notes
w: previous parameter
"""
function update!(irl::LinearIRL, w)
    error("TODO")
end

"""
Policy evaluation [1, Eq. 96]
"""
function OptimalInput(irl::LinearIRL, x, w)
    @unpack R_inv = irl
    P = convert_to_matrix(irl, w)
    -0.5 * R_inv * B' * P * x
end

function convert_to_matrix(irl::LinearIRL, w)
    error("TODO")
    # P = 
end
