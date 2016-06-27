#include "pushDeformer.h"
#include <maya/MFloatPointArray.h>
MTypeId	PushDeformer::id(0x00124502);
MObject PushDeformer::aDriverGeo;
MObject PushDeformer::aAmount;
MObject PushDeformer::aStressMap;
MObject PushDeformer::aUseStress;
MObject PushDeformer::aMultiThreadingType;

#define NUM_TASKS	16

PushDeformer::PushDeformer()
{
  MThreadPool::init();
}

PushDeformer::~PushDeformer()
{
  MThreadPool::release();
}

void* PushDeformer::creator()
{
	return new PushDeformer();
}

ThreadData* PushDeformer::createThreadData( int numTasks, TaskData* pTaskData )
{
    ThreadData* pThreadData = new ThreadData[numTasks];
    unsigned int numPoints = pTaskData->points.length();
    unsigned int taskLength = (numPoints + numTasks - 1) / numTasks;
    unsigned int start = 0;
    unsigned int end = taskLength;

    int lastTask = numTasks - 1;
	for( int i = 0; i < numTasks; ++i )
	{
        if ( i == lastTask )
        {
            end = numPoints;
        }
        pThreadData[i].start = start;
        pThreadData[i].end = end;
        pThreadData[i].numTasks = numTasks;
        pThreadData[i].pData = pTaskData;

        start += taskLength;
        end += taskLength;
	}
    return pThreadData;
}


void PushDeformer::createTasks( void* pData, MThreadRootTask* pRoot )
{
    ThreadData* pThreadData = (ThreadData*)pData;

	if ( pThreadData )
	{
        int numTasks = pThreadData->numTasks;
		for( int i = 0; i < numTasks; ++i )
		{
            MThreadPool::createTask( threadEvaluate, (void*)&pThreadData[i], pRoot );
		}
		MThreadPool::executeAndJoin( pRoot );
	}
}

MThreadRetVal PushDeformer::threadEvaluate( void *pParam )
{
    MStatus status;
    ThreadData* pThreadData = (ThreadData*)(pParam);
    TaskData* pData = pThreadData->pData;
    
    unsigned int start = pThreadData->start;
    unsigned int end = pThreadData->end;

    MPointArray& points = pData->points;
    MFloatVectorArray& normals = pData->normals;
    //MDataBlock& dataBlock  = pData->dataBlock;
    MDoubleArray& stressV = pData->stressV;
    double bulgeAmount = pData->bulgeAmount;
    bool useStressV = pData->useStressV;
    float envelope = pData->envelope;
    int geomIndex = pData->geomIndex;
	float w = 1.0;
    for ( unsigned int i = start; i < end; ++i )
    {
		  //float w = weightValue(dataBlock, geomIndex, i);
	  double stressFactor = (useStressV == 1 ? stressV[i] : 1.0);
	  double factor = bulgeAmount * envelope * w * stressFactor;
	  points[i].x += normals[i].x * factor;
	  points[i].y += normals[i].y * factor;
	  points[i].z += normals[i].z * factor;
    }
    return 0;
}


MStatus PushDeformer::deform(MDataBlock& dataBlock,
								MItGeometry& itGeo,
								const MMatrix& localToWorldMatrix,
								unsigned int geomIndex)
{
	MStatus status;
	//get attribute handles
	double bulgeAmount = dataBlock.inputValue(aAmount, &status).asDouble();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	m_taskData.envelope = dataBlock.inputValue(envelope, &status).asFloat();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	bool useStressV = dataBlock.inputValue(aUseStress, &status).asBool();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	int multiThreadingType = dataBlock.inputValue(aMultiThreadingType, &status).asBool();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	if (m_taskData.envelope <= 0.001)
	{
		return MS::kSuccess;
	}
	// if the use stress plug is turned on pull 
	MDoubleArray stressV;
	if (useStressV == true)
	{
		//pull out the raw data as an Mobject
		MObject stressMap = dataBlock.inputValue(aStressMap, &status).data();
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MFnDoubleArrayData stressDataFn(stressMap);
        m_taskData.stressV = stressDataFn.array();
	}

	//retrieve the handle to the output array attribute
	MArrayDataHandle hInput = dataBlock.outputArrayValue(input, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	//get the input array index handle
	status = hInput.jumpToElement(geomIndex);
	//get the handle of geomIndex attribute
	MDataHandle hInputElement = hInput.outputValue(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	//Get the MObject of the input geometry of geomindex
	MObject oInputGeom = hInputElement.child(inputGeom).asMesh();
	MFnMesh fnMesh(oInputGeom, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

  

	
  //MGlobal::displayInfo( "test" );
  /*for (int i = 0; i < itGeo.count();  i++)
	{
    MGlobal::displayInfo( MFnAttribute(weightList).isArray );
  }*/
  m_taskData.bulgeAmount = bulgeAmount;

  if(multiThreadingType == 1)
  {
    itGeo.allPositions(m_taskData.points, MSpace::kWorld);
    fnMesh.getVertexNormals(false, m_taskData.normals, MSpace::kWorld);
    ThreadData* pThreadData = createThreadData( NUM_TASKS, &m_taskData );
    MThreadPool::newParallelRegion( createTasks, (void*)pThreadData );
    itGeo.setAllPositions(m_taskData.points);
    delete [] pThreadData;
    return MS::kSuccess;
  }


  else if(multiThreadingType == 2)
  {
    itGeo.allPositions(m_taskData.points, MSpace::kWorld);
    fnMesh.getVertexNormals(false, m_taskData.normals, MSpace::kWorld);
    bool useStressV = m_taskData.useStressV == true && (m_taskData.stressV.length() > 0);
	const float w = 1.0;
    tbb::parallel_for(size_t(0), size_t(itGeo.count()), [this, useStressV, w](size_t i)
    {
		//const float w = weightValue(dataBlock, geomIndex, i);
		
		double stressFactor = (useStressV == 1 ? m_taskData.stressV[i] : 1.0);
		double factor = m_taskData.bulgeAmount * m_taskData.envelope * w * stressFactor;
		//deform
		m_taskData.points[i].x += m_taskData.normals[i].x * factor;
		m_taskData.points[i].y += m_taskData.normals[i].y * factor;
		m_taskData.points[i].z += m_taskData.normals[i].z * factor;
    });
  }
  else if (multiThreadingType == 3)
  {
    const float w = 1.0;
    itGeo.allPositions(m_taskData.points, MSpace::kWorld);
    fnMesh.getVertexNormals(false, m_taskData.normals, MSpace::kWorld);
    bool useStressV = m_taskData.useStressV == true && (m_taskData.stressV.length() > 0);
    #pragma omp parallel for 
    for (int i = 0; i < itGeo.count(); i++)
    {
      //float w = weightValue(dataBlock, geomIndex, itGeo.index());
      double stressFactor = (useStressV == 1 ? m_taskData.stressV[i] : 1.0);
      double factor = bulgeAmount * m_taskData.envelope * w * stressFactor;
      m_taskData.points[i].x += m_taskData.normals[i].x * factor;
      m_taskData.points[i].y += m_taskData.normals[i].y * factor;
      m_taskData.points[i].z += m_taskData.normals[i].z * factor;
    }
  }
  else if (multiThreadingType == 4)
  {
    unsigned int start = 0;
    unsigned int end = itGeo.count();
    float w = 1.0;
    float ba = float(bulgeAmount);
    float factor = float(bulgeAmount) * m_taskData.envelope * w;
    MFloatVectorArray norm;
    //fnMesh.getNormals(norm);
    fnMesh.getVertexNormals(false, norm, MSpace::kWorld);
    MFloatPointArray pts;
    fnMesh.getPoints(pts);

    float * a = &pts[0].x;
    float * b = &norm[0].x;
    int  vEnd = end;
    int x = 0;
    //#pragma omp parallel for
    for (int i = 0; i < end; i++)
    {
      int k = i * 4;
      int l = k + 4;
      #pragma simd
      for (int j = k; j < l; j++)
      {
        if(k-i != 4)
          a[j] += b[j-x] * factor;
      }
      x++;
      m_taskData.points[i] = MFloatPoint(a[k], a[k+1], a[k+2], 1.0);
    }
  }
  else if (multiThreadingType == 5)
  {
      unsigned int start = 0;
      unsigned int end = itGeo.count();
      float w = 1.0;
      float ba = float(bulgeAmount);
      float factor = float(bulgeAmount) * m_taskData.envelope * w;
      MFloatVectorArray norm;
      //fnMesh.getNormals(norm);
      fnMesh.getVertexNormals(false, norm, MSpace::kWorld);
      const float * pts = fnMesh.getRawPoints(0);
      std::vector<float> a(end*3);
      float * b = &norm[0].x;
      int  vEnd = end;
      #pragma omp parallel for
      for (int i = 0; i < end; i++)
      {
          int k = i * 3;
          int l = k + 3;
          #pragma simd
          for (int j = k; j < l; j++)
          {
                  a[j] = pts[j] + (b[j] * factor);
          }
          m_taskData.points[i] = MFloatPoint(a[k], a[k + 1], a[k + 2], 1.0);
      }
  }
  else if (multiThreadingType == 6)
  {
      unsigned int start = 0;
      unsigned int end = itGeo.count();
      float w = 1.0;
      float ba = float(bulgeAmount);
      float factor = float(bulgeAmount) * m_taskData.envelope * w;
      MFloatVectorArray norm;
      //fnMesh.getNormals(norm);
      fnMesh.getVertexNormals(false, norm, MSpace::kWorld);
      const float * pts = fnMesh.getRawPoints(0);
      std::vector<float> a(end * 3);
      float * b = &norm[0].x;
      int  vEnd = end*3;
      #pragma omp parallel for
      //#pragma simd
      for (int i = 0; i < vEnd; i++)
      {
         a[i] = pts[i] + (b[i] * factor);
      }
      #pragma omp parallel for

      for (int k = 0; k < end; k++)
      {
          m_taskData.points[k] = MFloatVector(a[k * 3], a[k * 3 + 1], a[k * 3 + 2]);
      }

  }
  else
  {
    const float w = 1.0;
    itGeo.allPositions(m_taskData.points, MSpace::kWorld);
    fnMesh.getVertexNormals(false, m_taskData.normals, MSpace::kWorld);
    for (; !itGeo.isDone(); itGeo.next())
	  {
		  //float w = weightValue(dataBlock, geomIndex, itGeo.index());
          double stressFactor = (useStressV == 1 ? m_taskData.stressV[itGeo.index()] : 1.0);
          double factor = m_taskData.bulgeAmount * m_taskData.envelope * w * stressFactor;
          //deform
          m_taskData.points[itGeo.index()].x += m_taskData.normals[itGeo.index()].x * factor;
          m_taskData.points[itGeo.index()].y += m_taskData.normals[itGeo.index()].y * factor;
          m_taskData.points[itGeo.index()].z += m_taskData.normals[itGeo.index()].z * factor;
	  }
  }
	itGeo.setAllPositions(m_taskData.points);

	return MS::kSuccess;

}


MStatus PushDeformer::initialize()
{
	MStatus status;

	MFnNumericAttribute nAttr;
	MFnTypedAttribute tAttr;
	//add bulge attribute which affects the output geom
	aAmount = nAttr.create("amount", "amt", MFnNumericData::kDouble, 0.0);
	nAttr.setKeyable(true);
	addAttribute(aAmount);

	aStressMap = tAttr.create("aStressMap", "stMap", MFnData::kDoubleArray);
	tAttr.setKeyable(true);
	tAttr.setWritable(true);
	tAttr.setStorable(true);
	addAttribute(aStressMap);

	aUseStress = nAttr.create("useStress", "useStress", MFnNumericData::kBoolean, 0);
	nAttr.setKeyable(true);
	nAttr.setStorable(true);
	addAttribute(aUseStress);

	aMultiThreadingType = nAttr.create("multiThreadingType", "multiThreadingType", MFnNumericData::kInt, 0);
	nAttr.setKeyable(true);
	addAttribute(aMultiThreadingType);

	attributeAffects(aAmount, outputGeom);
	attributeAffects(aStressMap, outputGeom);
	attributeAffects(aUseStress, outputGeom);
  attributeAffects(aMultiThreadingType, outputGeom);
	// make paintable deformer
	MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer pushDeformer weights");

	return MS::kSuccess;
}