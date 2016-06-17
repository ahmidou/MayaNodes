#include "pushDeformer.h"
#include <maya/MFnPlugin.h>


MStatus initializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin fnplugin(obj, "David Sparrow", "1.0", "any");

	//register the deformerNode
	status = fnplugin.registerNode("pushDeformer", 
		PushDeformer::id,
		PushDeformer::creator,
		PushDeformer::initialize,
		MPxNode::kDeformerNode);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj);

	//deregister the deformerNode
	status = plugin.deregisterNode(PushDeformer::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;


}