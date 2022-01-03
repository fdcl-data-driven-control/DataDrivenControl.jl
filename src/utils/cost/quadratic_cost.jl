struct QuadraticCost <: AbstractCost
    Q::AbstractMatrix
    R::AbstractMatrix
end

function (cost::QuadraticCost)(x, u)
    @unpack Q, R = cost
    x'*Q*x + u'*R*u
end
