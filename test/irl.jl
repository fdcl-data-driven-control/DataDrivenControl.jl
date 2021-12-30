@testset "irl" begin
    w = [1, 2, 3]
    P = DataDrivenControl.convert_to_matrix(w)
    @test P == [1 0; 2 3]
end
