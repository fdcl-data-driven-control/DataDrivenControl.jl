abstract type AbstractStopCondition end

function is_terminated(sc::AbstractStopCondition, args...; kwrags...)
    error("Defined this method for type: $(typeof(sc))")
end


struct DistanceStopCondition <: AbstractStopCondition
    eps::Real
    p::Real
    function DistanceStopCondition(eps=1e-1, p=2)
        @assert p > 1
        new(p, eps)
    end
end

function is_terminated(sc::DistanceStopCondition, w, w_new)
    @unpack eps, p = sc
    norm(w_new - w, p) < eps
end
