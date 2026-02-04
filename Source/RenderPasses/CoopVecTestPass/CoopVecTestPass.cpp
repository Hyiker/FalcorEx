/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#include "CoopVecTestPass.h"
#include "RenderGraph/RenderPassHelpers.h"

namespace
{
const std::string kShaderFile = "RenderPasses/CoopVecTestPass/CoopVecTestPass.cs.slang";
const std::string kOutput = "output";
} // namespace

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, CoopVecTestPass>();
}

CoopVecTestPass::CoopVecTestPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    mpComputePass = ComputePass::create(mpDevice, kShaderFile, "main");

    if (!mpDevice->isShaderModelSupported(ShaderModel::SM6_9))
        FALCOR_THROW("BSDFIntegrator requires Shader Model 6.9 support");
}

Properties CoopVecTestPass::getProperties() const
{
    return {};
}

RenderPassReflection CoopVecTestPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addOutput(kOutput, "Output").bindFlags(ResourceBindFlags::UnorderedAccess).format(mOutputFormat);
    return reflector;
}

void CoopVecTestPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    auto pOutput = renderData.getTexture(kOutput);
    if (!pOutput)
    {
        logWarning("CoopVecTestPass::execute() - Missing output.");
        return;
    }

    mFrameDim = uint2(pOutput->getWidth(), pOutput->getHeight());
    mOutputFormat = pOutput->getFormat();

    auto var = mpComputePass->getRootVar();
    var["CB"]["frameDim"] = mFrameDim;
    var["output"] = pOutput;

    mpComputePass->execute(pRenderContext, mFrameDim.x, mFrameDim.y);
}

void CoopVecTestPass::renderUI(Gui::Widgets& widget) {}
