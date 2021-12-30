struct QuadraticCost <: AbstractCost
    Q::Matrix
    R::Matrix
end

function (cost::QuadraticCost)(x, u)
    @unpack Q, R = cost
    x'*Q*x + u'*R*u
end
