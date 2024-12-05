/******************************************************************************
This file is part of the Newcastle Vulkan Tutorial Series

Author:Rich Davison
Contact:richgdavison@gmail.com
License: MIT (see LICENSE file at the top of the source tree)
*//////////////////////////////////////////////////////////////////////////////
#include "VulkanRayTracingPipelineBuilder.h"

#include "VulkanMesh.h"
#include "VulkanUtils.h"

using namespace NCL;
using namespace Rendering;
using namespace Vulkan;

VulkanRayTracingPipelineBuilder::VulkanRayTracingPipelineBuilder(vk::Device device) : PipelineBuilderBase(device){
}

VulkanRayTracingPipelineBuilder::~VulkanRayTracingPipelineBuilder() {
}

VulkanRayTracingPipelineBuilder& VulkanRayTracingPipelineBuilder::WithRecursionDepth(uint32_t count) {
	pipelineCreate.maxPipelineRayRecursionDepth = count;
	return *this;
}

VulkanRayTracingPipelineBuilder& VulkanRayTracingPipelineBuilder::WithRayGenGroup(uint32_t shaderIndex) {
	vk::RayTracingShaderGroupCreateInfoKHR groupCreateInfo;
	groupCreateInfo.type = vk::RayTracingShaderGroupTypeKHR::eGeneral;
	groupCreateInfo.generalShader = shaderIndex;
	genGroups.push_back(groupCreateInfo);
	return *this;
}

VulkanRayTracingPipelineBuilder& VulkanRayTracingPipelineBuilder::WithMissGroup(uint32_t shaderIndex) {
	vk::RayTracingShaderGroupCreateInfoKHR groupCreateInfo;
	groupCreateInfo.type = vk::RayTracingShaderGroupTypeKHR::eGeneral;
	groupCreateInfo.generalShader = shaderIndex;
	missGroups.push_back(groupCreateInfo);
	return *this;
}


//VulkanRayTracingPipelineBuilder& VulkanRayTracingPipelineBuilder::WithShaderGroup(const vk::RayTracingShaderGroupCreateInfoKHR& groupCreateInfo) {
//	shaderGroups.push_back(groupCreateInfo);
//	return *this;
//}
//
//VulkanRayTracingPipelineBuilder& VulkanRayTracingPipelineBuilder::WithGeneralGroup(uint32_t index) {
//	vk::RayTracingShaderGroupCreateInfoKHR groupCreateInfo;
//
//	groupCreateInfo.type = vk::RayTracingShaderGroupTypeKHR::eGeneral;
//	groupCreateInfo.intersectionShader	= VK_SHADER_UNUSED_KHR;
//	groupCreateInfo.generalShader		= index;
//	groupCreateInfo.closestHitShader	= VK_SHADER_UNUSED_KHR;
//	groupCreateInfo.anyHitShader		= VK_SHADER_UNUSED_KHR;
//
//	shaderGroups.push_back(groupCreateInfo);
//
//	return *this;
//}

VulkanRayTracingPipelineBuilder& VulkanRayTracingPipelineBuilder::WithTriangleHitGroup(uint32_t closestHit, uint32_t anyHit) {
	vk::RayTracingShaderGroupCreateInfoKHR groupCreateInfo;

	groupCreateInfo.type				= vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup;
	groupCreateInfo.generalShader		= VK_SHADER_UNUSED_KHR;
	groupCreateInfo.intersectionShader	= VK_SHADER_UNUSED_KHR;
	groupCreateInfo.closestHitShader	= closestHit;
	groupCreateInfo.anyHitShader		= anyHit;

	hitGroups.push_back(groupCreateInfo);

	return *this;
}

VulkanRayTracingPipelineBuilder& VulkanRayTracingPipelineBuilder::WithProceduralHitGroup(uint32_t intersection, uint32_t closestHit, uint32_t anyHit){
	vk::RayTracingShaderGroupCreateInfoKHR groupCreateInfo;

	groupCreateInfo.type				= vk::RayTracingShaderGroupTypeKHR::eProceduralHitGroup;
	groupCreateInfo.generalShader		= VK_SHADER_UNUSED_KHR;
	groupCreateInfo.intersectionShader	= intersection;
	groupCreateInfo.closestHitShader	= closestHit;
	groupCreateInfo.anyHitShader		= anyHit;

	hitGroups.push_back(groupCreateInfo);

	return *this;
}

VulkanRayTracingPipelineBuilder& VulkanRayTracingPipelineBuilder::WithShader(VulkanRTShader& shader, vk::ShaderStageFlagBits stage, const string& entry) {
	ShaderEntry entryInfo;
	entryInfo.entryPoint = entry;
	entryInfo.shader = &shader;
	entryInfo.stage = stage;

	entries.push_back(entryInfo);

	shader.FillDescriptorSetLayouts(reflectionLayouts);
	shader.FillPushConstants(allPushConstants);
	
	return *this;
}

VulkanPipeline VulkanRayTracingPipelineBuilder::Build(const std::string& debugName, vk::PipelineCache cache) {
	for (const auto& i : entries) {
		vk::PipelineShaderStageCreateInfo stageInfo;

		stageInfo.pName = i.entryPoint.c_str();
		stageInfo.stage = i.stage;
		stageInfo.module = *i.shader->GetModule();
		shaderStages.push_back(stageInfo);
	}

	FinaliseDescriptorLayouts();

	allGroups.clear();
	allGroups.insert(allGroups.end(), genGroups.begin() , genGroups.end());
	allGroups.insert(allGroups.end(), missGroups.begin(), missGroups.end());
	allGroups.insert(allGroups.end(), hitGroups.begin() , hitGroups.end());

	pipelineCreate.groupCount	= allGroups.size();
	pipelineCreate.pGroups		= allGroups.data();

	pipelineCreate.stageCount	= shaderStages.size();
	pipelineCreate.pStages		= shaderStages.data();

	vk::PipelineLayoutCreateInfo pipeLayoutCreate = vk::PipelineLayoutCreateInfo()
		.setSetLayouts(allLayouts)
		.setPushConstantRanges(allPushConstants);

	VulkanPipeline output;
	output.layout = sourceDevice.createPipelineLayoutUnique(pipeLayoutCreate);

	pipelineCreate.layout = *output.layout;
	pipelineCreate.setPDynamicState(&dynamicCreate);

	output.pipeline = sourceDevice.createRayTracingPipelineKHRUnique({}, cache, pipelineCreate).value;

	if (!debugName.empty()) {
		SetDebugName(sourceDevice, vk::ObjectType::ePipeline, GetVulkanHandle(*output.pipeline), debugName);
	}

	return output;
}