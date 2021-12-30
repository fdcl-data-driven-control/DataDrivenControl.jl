"""
Convet a vector to lower triangular matrix
"""
function convert_to_matrix(w::Vector)::Matrix
    _l = length(w)
    l = Int((-1 + sqrt(1+4*2*_l)) / 2)  # P is l x l lower triangular matrix with elements of w
    P = zeros(l, l)
    idx = 1
    for i in 1:l
        for j in 1:l
            if i >= j
                P[i, j] = w[idx]
                idx += 1
            end
        end
    end
    P
end


