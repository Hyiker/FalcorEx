add_falcor_executable(Benchmark)

target_sources(Benchmark PRIVATE
    Benchmark.cpp
    Benchmark.h
)

target_copy_shaders(Benchmark Samples/Benchmark)

target_source_group(Benchmark "Samples")
