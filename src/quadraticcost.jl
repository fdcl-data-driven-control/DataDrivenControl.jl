struct QuadraticCost <: AbstractCost
    Q
    R
end

function QuadraticCost(x, u)
    x.'*Q*x + u.'*R*u
end
