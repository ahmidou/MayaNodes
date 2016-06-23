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

    for ( unsigned int i = start; i < end; ++i )
    {
		  //float w = weightValue(dataBlock, geomIndex, i);
      float w = 1.0;
		  if (useStressV == true && (stressV.length() > 0))
		  {
			  //deform
			  points[i] += (MVector(normals[i]) * bulgeAmount * envelope * w * stressV[i]);
			
		  }
		  else
		  {
			  //deform
			  points[i] += normals[i] * bulgeAmount * envelope * w;

		  }
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

  

	itGeo.allPositions(m_taskData.points, MSpace::kWorld);
  //MGlobal::displayInfo( "test" );
  /*for (int i = 0; i < itGeo.count();  i++)
	{
    MGlobal::displayInfo( MFnAttribute(weightList).isArray );
  }*/
  m_taskData.bulgeAmount = bulgeAmount;

  if(multiThreadingType == 1)
  {
    fnMesh.getVertexNormals(false, m_taskData.normals, MSpace::kWorld);
    ThreadData* pThreadData = createThreadData( NUM_TASKS, &m_taskData );
    MThreadPool::newParallelRegion( createTasks, (void*)pThreadData );
    itGeo.setAllPositions(m_taskData.points);
    delete [] pThreadData;
    return MS::kSuccess;
  }


  else if(multiThreadingType == 2)
  {
    fnMesh.getVertexNormals(false, m_taskData.normals, MSpace::kWorld);
    bool useStressV = m_taskData.useStressV == true && (m_taskData.stressV.length() > 0);
    tbb::parallel_for(size_t(0), size_t(itGeo.count()), [this, useStressV](size_t i)
    {
		  //const float w = weightValue(dataBlock, geomIndex, i);
      const float w = 1.0;
      float factor = m_taskData.bulgeAmount * m_taskData.envelope * w * (useStressV ? m_taskData.stressV[i] : 1.0);
			//deform
			m_taskData.points[i] += m_taskData.normals[i] * factor;
    });
  }
  else if (multiThreadingType == 3)
  {
    const float w = 1.0;
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
  // Vectorized
  {
    unsigned int start = 0;
    unsigned int end = itGeo.count();
    float w = 1.0;
    //#pragma omp parallel for 
    float ba = float(bulgeAmount);
    float factor = float(bulgeAmount) * m_taskData.envelope * w;
    MFloatPointArray pts;
    fnMesh.getPoints(pts);
    MFloatVectorArray norm;
    fnMesh.getNormals(norm);

    for (unsigned int i = 0; i < end; i++)
    {
      //can't use pointers because:info C5002: loop not vectorized due to reason '1303'
      //https://msdn.microsoft.com/en-us/library/jj658585.aspx
      float ptCpy[4] = { pts[i].x,  pts[i].y,  pts[i].z, 0.0 };
      float nCpy[4] = { norm[i].x, norm[i].y, norm[i].z, 0.0 };
      for (int j = 0; j < 4; j++) {
        ptCpy[j] += factor * nCpy[j];
      }
      //pts[i] = MFloatPoint(ptCpy);
      m_taskData.points[i] = MFloatPoint(ptCpy);
    }
  }
  else if (multiThreadingType == 5)
  // Vectorized 2
  {
    unsigned int start = 0;
    unsigned int end = itGeo.count();
    float w = 1.0;
    //#pragma omp parallel for 
    float ba = float(bulgeAmount);
    float factor = float(bulgeAmount) * m_taskData.envelope * w;
    MFloatVectorArray norm;
    fnMesh.getNormals(norm);
    const float * pts = fnMesh.getRawPoints(0);
    std::vector<float> vPts(end * 4);
    std::vector<float> vNorms(end * 4);

    for (unsigned int i = 0; i < end; i++)
    {
      vPts[i * 3] = pts[i * 3];
      vPts[i * 3 + 1] = pts[i * 3 + 1];
      vPts[i * 3 + 2] = pts[i * 3 + 2];

      vNorms[i * 3] = norm[i].x;
      vNorms[i * 3 + 1] = norm[i].y;
      vNorms[i * 3 + 2] = norm[i].z;
    }
    unsigned int  vEnd = end * 3;
    for (unsigned int i = 0; i < vEnd; i++)
    {
      vPts[i] += vNorms[i] * factor;
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
    fnMesh.getNormals(norm);
    const float * pts = fnMesh.getRawPoints(0);
    float * vPts;
    float * vNorms;
    memcpy(vPts, pts, sizeof(float) * end * 3);
    memcpy(vNorms, &norm[0], sizeof(float) * end * 3);

    unsigned int  vEnd = end * 3;
    for (unsigned int i = 0; i < vEnd; i++)
    {
      vPts[i] += vNorms[i] * factor;
    }
    //memcpy(vNorms, &vPts[0], sizeof(float) * end * 3);
  }
  else if (multiThreadingType == 7)
  {
    unsigned int start = 0;
    unsigned int end = itGeo.count();
    float w = 1.0;
    float ba = float(bulgeAmount);
    float factor = float(bulgeAmount) * m_taskData.envelope * w;
    MFloatVectorArray norm;
    fnMesh.getNormals(norm);
    MFloatPointArray pts;
    fnMesh.getPoints(pts);

    float * a = &pts[0].x;
    float * b = &norm[0].x;
    int  vEnd = end;
    int x = 0;
    for (int i = 0; i < end; i++)
    {
      int k = i * 4;
      int l = k + 4;
      for (int j = k; j < l; j++)
      {
        if(k-i != 4)
          a[j] += b[j-x] * factor;
      }
      x++;
      m_taskData.points[i] = MFloatPoint(a[k], a[k+1], a[k+2], a[k+3]);
    }
  }
  else
  {
    const float w = 1.0;
    fnMesh.getVertexNormals(false, m_taskData.normals, MSpace::kWorld);
    for (; !itGeo.isDone(); itGeo.next())
	  {
		  //float w = weightValue(dataBlock, geomIndex, itGeo.index());
      
      float factor = m_taskData.bulgeAmount * m_taskData.envelope * w * (useStressV ? m_taskData.stressV[itGeo.index()] : 1.0);
      //deform
      m_taskData.points[itGeo.index()] += m_taskData.normals[itGeo.index()] * factor;
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