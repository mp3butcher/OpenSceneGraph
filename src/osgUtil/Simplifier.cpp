/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#include <osgUtil/Simplifier>

#include <osgUtil/SmoothingVisitor>
#include <osgUtil/TriStripVisitor>

#include <set>
#include <list>
#include <iterator>

using namespace osgUtil;

EdgeCollapse::Point* Simplifier::computeInterpolatedPoint(EdgeCollapse::Edge* edge,float r) {

    EdgeCollapse::Point* p1 = edge->_p1.get();
    EdgeCollapse::Point* p2 = edge->_p2.get();

    if (p1==0 || p2==0)
    {
        OSG_NOTICE<<"Error computeInterpolatedPoint("<<edge<<",r) p1 and/or p2==0"<<std::endl;
        return 0;
    }

    EdgeCollapse::Point* point = new EdgeCollapse::Point;
    float r1 = 1.0f-r;
    float r2 = r;

    point->_vertex = p1->_vertex * r1 + p2->_vertex * r2;
    unsigned int s = osg::minimum(p1->_attributes.size(),p2->_attributes.size());
    for(unsigned int i=0;i<s;++i)
    {
        point->_attributes.push_back(p1->_attributes[i]*r1 + p2->_attributes[i]*r2);
    }
    return point;
}

Simplifier::Simplifier(double sampleRatio, double maximumError, double maximumLength):
            osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
            _sampleRatio(sampleRatio),
            _maximumError(maximumError),
            _maximumLength(maximumLength),
            _triStrip(false),
            _smoothing(false)

{
}

void Simplifier::simplify(osg::Geometry& geometry)
{
    // pass an empty list of indices to simply(Geometry,IndexList)
    // so that this one method handle both cases of non protected indices
    // and specified indices.
   IndexList emptyList;
   return simplify(geometry,emptyList);
}

void Simplifier::simplify(osg::Geometry& geometry, const IndexList& protectedPoints)
{
    OSG_INFO<<"++++++++++++++simplifier************"<<std::endl;

    bool downSample = requiresDownSampling();

    EdgeCollapse ec(this);
    ec.setComputeErrorMetricUsingLength(!downSample);
    ec.setGeometry(&geometry, protectedPoints);
    ec.updateErrorMetricForAllEdges();

    unsigned int numOriginalPrimitives = ec._triangleSet.size();


    if (downSample)
    {
        while (!ec._edgeSet.empty() &&
               continueSimplification((*ec._edgeSet.begin())->getErrorMetric() , numOriginalPrimitives, ec._triangleSet.size()) &&
               ec.collapseMinimumErrorEdge())
        {
           //OSG_INFO<<"   Collapsed edge ec._triangleSet.size()="<<ec._triangleSet.size()<<" error="<<(*ec._edgeSet.begin())->getErrorMetric()<<" vs "<<getMaximumError()<<std::endl;
        }

        OSG_INFO<<"******* AFTER EDGE COLLAPSE *********"<<ec._triangleSet.size()<<std::endl;
    }
    else
    {

        // up sampling...
        while (!ec._edgeSet.empty() &&
               continueSimplification((*ec._edgeSet.rbegin())->getErrorMetric() , numOriginalPrimitives, ec._triangleSet.size()) &&
//               ec._triangleSet.size() < targetNumTriangles  &&
               ec.divideLongestEdge())
        {
           //OSG_INFO<<"   Edge divided ec._triangleSet.size()="<<ec._triangleSet.size()<<" error="<<(*ec._edgeSet.rbegin())->getErrorMetric()<<" vs "<<getMaximumError()<<std::endl;
        }
        OSG_INFO<<"******* AFTER EDGE DIVIDE *********"<<ec._triangleSet.size()<<std::endl;
    }

    OSG_INFO<<"Number of triangle errors after edge collapse= "<<ec.testAllTriangles()<<std::endl;
    OSG_INFO<<"Number of edge errors before edge collapse= "<<ec.testAllEdges()<<std::endl;
    OSG_INFO<<"Number of point errors after edge collapse= "<<ec.testAllPoints()<<std::endl;
    OSG_INFO<<"Number of triangles= "<<ec._triangleSet.size()<<std::endl;
    OSG_INFO<<"Number of points= "<<ec._pointSet.size()<<std::endl;
    OSG_INFO<<"Number of edges= "<<ec._edgeSet.size()<<std::endl;
    OSG_INFO<<"Number of boundary edges= "<<ec.computeNumBoundaryEdges()<<std::endl;

    if (!ec._edgeSet.empty())
    {
        OSG_INFO<<std::endl<<"Simplifier, in = "<<numOriginalPrimitives<<"\tout = "<<ec._triangleSet.size()<<"\terror="<<(*ec._edgeSet.begin())->getErrorMetric()<<"\tvs "<<getMaximumError()<<std::endl<<std::endl;
        OSG_INFO<<           "        !ec._edgeSet.empty()  = "<<!ec._edgeSet.empty()<<std::endl;
        OSG_INFO<<           "        continueSimplification(,,)  = "<<continueSimplification((*ec._edgeSet.begin())->getErrorMetric() , numOriginalPrimitives, ec._triangleSet.size())<<std::endl;
    }

    ec.copyBackToGeometry();

    if (_smoothing)
    {
        osgUtil::SmoothingVisitor::smooth(geometry);
    }

    if (_triStrip)
    {
        osgUtil::TriStripVisitor stripper;
        stripper.stripify(geometry);
    }
    osg::ref_ptr<osg::UIntArray> newmapping=new osg::UIntArray;
    for(EdgeCollapse::PointList::iterator itp=ec._originalPointList.begin(); itp != ec._originalPointList.end(); ++itp)
        newmapping->push_back((*itp)->_index);
   // return newmapping.release();

}
