abstract type AbstractBuffer end

struct DataBuffer <: AbstractBuffer
    data_array
    function DataBuffer()
        data_array = Any[]
        new(data_array)
    end
end

function Base.push!(buffer::DataBuffer, args...; kwargs...)
    error("Define specific method `push!`")
end
