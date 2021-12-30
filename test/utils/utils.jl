@testset "utils" begin
    @testset "convert" begin
        # convert_to_matrix
        w = rand(3)
        P = DataDrivenControl.convert_to_matrix(w)
        @test P == [w[1] 0; w[2] w[3]]
        # convert_quadratic_to_linear_basis
        x = rand(2)
        P_vec = vec(P)
        P_vec' * DataDrivenControl.convert_quadratic_to_linear_basis(x) == x' * P * x
    end
end
