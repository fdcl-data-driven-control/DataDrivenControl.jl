struct QuadraticCost <: AbstractCost
    Q::Matrix
    R::Matrix
end

function (cost::QuadraticCost)(x, u)
    x.'*Q*x + u.'*R*u
end
