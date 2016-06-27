#ifndef PUSHDEFORMER_H
#define PUSHDEFORMER_H

#include <maya/MPxDeformerNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MItGeometry.h>
#include <maya/MFloatArray.h>
#include <maya/MMatrix.h>
#include <maya/MDataBlock.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnDoubleArrayData.h>
#include <vector>
#include <maya/MThreadPool.h>
#include <tbb/parallel_for.h>
#include <omp.h>

struct TaskData 
{
    MPointArray points;
    //MDataBlock dataBlock;
    MFloatVectorArray normals;
    MDoubleArray stressV;
    int geomIndex;
    double bulgeAmount;
    float envelope;
    bool useStressV;
};


struct ThreadData
{
    TaskData* pData;
    unsigned int start;
    unsigned int end;
    unsigned int numTasks;
};

class PushDeformer : public MPxDeformerNode
{
public:	
  PushDeformer();
  virtual ~PushDeformer();
	static void* creator();

  ThreadData*         createThreadData( int numTasks, TaskData* pTaskData );
  static void         createTasks( void* data, MThreadRootTask *pRoot );
  static MThreadRetVal threadEvaluate( void* pParam );

	virtual	MStatus deform(MDataBlock& dataBlock,
							MItGeometry& itGeo,
							const MMatrix& localToWorldMatrix,
							unsigned int geoIndex);

	static MStatus initialize();

	static MTypeId id;
  static MObject aDriverGeo;
	static MObject aAmount;
	static MObject aStressMap;
	static MObject aUseStress;
  static MObject aMultiThreadingType;
  

private:
      TaskData m_taskData;
      ThreadData* threadData;
      double m_MThreadPoolTime;
};

#endif // !BULGEDEFORMER_H