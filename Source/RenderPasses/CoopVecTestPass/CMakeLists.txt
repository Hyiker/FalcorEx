add_plugin(CoopVecTestPass)

target_sources(CoopVecTestPass PRIVATE
    CoopVecTestPass.cs.slang
    CoopVecTestPass.cpp
    CoopVecTestPass.h
)

target_copy_shaders(CoopVecTestPass RenderPasses/CoopVecTestPass)

target_source_group(CoopVecTestPass "RenderPasses")
