struct QuadraticCost <: AbstractCost
    Q::Matrix
    R::Matrix
end

function QuadraticCost(x, u)
    x.'*Q*x + u.'*R*u
end
