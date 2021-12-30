abstract type AbstractBuffer end

struct DataBuffer <: AbstractBuffer
    data_array
end

function push!(buffer::DataBuffer, args...; kwargs...)
    error("Define specific method `push!`")
end
