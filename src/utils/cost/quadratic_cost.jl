abstract type AbstractQuadraticCost <: AbstractCost end

function (cost::AbstractQuadraticCost)(x::AbstractVector, u::AbstractVector)
    @unpack Q_func, R_func = cost
    Q_func(x) + u'*R_func(x)*u
end


"""
[1, Eq. (16)]
# Refs
[1] T. Bian and Z.-P. Jiang, “Reinforcement Learning and Adaptive Optimal Control for Continuous-Time Nonlinear Systems: A Value Iteration Approach,” IEEE Trans. Neural Netw. Learning Syst., pp. 1–10, 2021, doi: 10.1109/TNNLS.2020.3045087.
"""
struct QuadraticInInputCost <: AbstractQuadraticCost
    Q_func
    R_func
end

struct QuadraticCost <: AbstractQuadraticCost
    Q::AbstractMatrix
    R::AbstractMatrix
    Q_func
    R_func
    function QuadraticCost(Q, R)
        # assert ispossemidef(Q)... something like it?
        @assert isposdef(R)
        Q_func(x) = x'*Q*x
        R_func(x) = R
        new(Q, R, Q_func, R_func)
    end
end
