"""
Convert a vector `w` to a symmetric matrix `P` as
P = [
    w[1] ⋅ ⋅ ⋅ ⋅ ...;
    w[2] w[n+1]  ...;
    w[3] w[n+2]  ...;
    ...  ...        ;
    w[n] ...        ;
]
# Notes
It should be compatible with `convert_quadratic_to_linear_basis`.
"""
function convert_to_matrix(w::AbstractVector)::AbstractMatrix
    # n(n+1)/2 x 1 -> n x n
    _l = length(w)
    l = Int((-1 + sqrt(1+4*2*_l)) / 2)  # P is l x l lower triangular matrix with elements of w
    P = zeros(l, l)
    idx = 1
    for i in 1:l
        for j in i:l
            if i == j
                P[j, i] = w[idx]
            else
                P[j, i] = 0.5 * w[idx]
                P[i, j] = 0.5 * w[idx]
            end
            idx += 1
        end
    end
    P
    # # n^2 x 1 -> n x n for convenience...
    # l = Int(sqrt(length(w)))
    # P = zeros(l, l)
    # idx = 1
    # for i in 1:l
    #     for j in 1:l
    #         P[j, i] = w[idx]
    #         idx += 1
    #     end
    # end
    # P
end

"""
Basis transformation for linear ADP and IRL;
xᵀ * P * x = wᵀ * ϕ(x)
where ϕ(x) = [x1^2, x2*x1, x3*x1, ..., xn*x1, x2^2, x3*x2, x4*x2, ..., xn*x2, x3^2, x4*x3, ...]
# Notes
It should be compatible with `convert_to_matrix`.
"""
function convert_quadratic_to_linear_basis(x::AbstractVector)
    ## Notes
    ## ϕ(x) = (I_m ⊗ x) x;
    ## Px = (I_m ⊗ xᵀ) w
    ## where P ∈ ℝ^{m×n}, w ∈ ℝ^{nm×1}, I_m: m×m identity matrix, ⊗: Kronecker product
    # n = length(x)
    # kron(Matrix(I, n, n), x) * x
    ## for dimension reduction
    n = length(x)
    ϕ = zeros(Int(n*(n+1)/2))
    idx = 1
    for i in 1:n
        for j in i:n
            ϕ[idx] = x[j] * x[i]
            idx += 1
        end
    end
    ϕ
end
