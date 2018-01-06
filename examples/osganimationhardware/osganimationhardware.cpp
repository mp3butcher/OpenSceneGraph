/*  -*-c++-*-
 *  Copyright (C) 2009 Cedric Pinson <cedric.pinson@plopbyte.net>
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

#include <iostream>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/Simplifier>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osg/Drawable>
#include <osg/MatrixTransform>

#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/RigGeometry>
#include <osgAnimation/RigTransformHardware>
#include <osgAnimation/MorphGeometry>
#include <osgAnimation/MorphTransformHardware>
#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BoneMapVisitor>
#include <osg/Material>
#include <osg/BufferIndexBinding>
#include <osg/PrimitiveSetIndirect>
#include <osg/BufferTemplate>
#include <sstream>


#include <assert.h>

static unsigned int getRandomValueinRange(unsigned int v)
{
    return static_cast<unsigned int>((rand() * 1.0 * v)/(RAND_MAX-1));
}


osg::ref_ptr<osg::Program> program;
osg::ref_ptr<osg::Shader> skinningshader;
// show how to override the default RigTransformHardware for customized usage
struct MyRigTransformHardware : public osgAnimation::RigTransformHardware
{

    /*virtual void operator()(osgAnimation::RigGeometry& geom)
    {
        if (_needInit)
            if (!init(geom))
                return;
        computeMatrixPaletteUniform(geom.getMatrixFromSkeletonToGeometry(), geom.getInvMatrixFromSkeletonToGeometry());
    }*/

    bool init(osgAnimation::RigGeometry& geom)
    {
        osg::Vec3Array* pos = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
        if (!pos) {
            osg::notify(osg::WARN) << "RigTransformHardware no vertex array in the geometry " << geom.getName() << std::endl;
            return false;
        }

        if (!geom.getSkeleton()) {
            osg::notify(osg::WARN) << "RigTransformHardware no skeleting set in geometry " << geom.getName() << std::endl;
            return false;
        }

        osgAnimation::BoneMapVisitor mapVisitor;
        geom.getSkeleton()->accept(mapVisitor);
        osgAnimation::BoneMap bm = mapVisitor.getBoneMap();
        //if (!createPalette(pos->size(),bm, geom.getVertexInfluenceSet().getVertexToBoneWeightList()))
        // return false;

        int attribIndex = 11;
        int nbAttribs = getNumVertexAttrib();

        // use a global program for all avatar
        if (!program.valid()) {
            program = new osg::Program;
            program->setName("HardwareSkinning");
            if (!skinningshader.valid())
                _shader = osg::Shader::readShaderFile(osg::Shader::VERTEX,"shaders/skinning.vert");

            if (!_shader.valid()) {
                osg::notify(osg::WARN) << "RigTransformHardware can't load VertexShader" << std::endl;
                return false;
            }

            // replace max matrix by the value from uniform
            {
                std::string str = _shader->getShaderSource();
                std::string toreplace = std::string("MAX_MATRIX");
                std::size_t start = str.find(toreplace);
                std::stringstream ss;
                ss << getMatrixPaletteUniform()->getNumElements();
                str.replace(start, toreplace.size(), ss.str());
                _shader->setShaderSource(str);
                // osg::notify(osg::INFO) << "Shader " << str << std::endl;
            }

            program->addShader(_shader.get());

            for (int i = 0; i < nbAttribs; i++)
            {
                std::stringstream ss;
                ss << "boneWeight" << i;
                program->addBindAttribLocation(ss.str(), attribIndex + i);

                osg::notify(osg::INFO) << "set vertex attrib " << ss.str() << std::endl;
            }
        }
        for (int i = 0; i < nbAttribs; i++)
        {
            std::stringstream ss;
            ss << "boneWeight" << i;
            geom.setVertexAttribArray(attribIndex + i, getVertexAttrib(i));
        }

        osg::ref_ptr<osg::StateSet> ss = new osg::StateSet;
        ss->addUniform(getMatrixPaletteUniform());
        ss->addUniform(new osg::Uniform("nbBonesPerVertex", getNumBonesPerVertex()));
        ss->setAttributeAndModes(program.get());
        geom.setStateSet(ss.get());
        _needInit = false;
        return true;
    }

};


struct AnimationManagerFinder : public osg::NodeVisitor
{

    osg::ref_ptr<osgAnimation::BasicAnimationManager> _am;
    AnimationManagerFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Node& node) {
        if (_am.valid()) {
            osgAnimation::Animation * anim=_am->getRegisteredAnimation(0);
            OSG_WARN<<"aim0"<<anim->getName()<<std::endl;
            return;
        }
        if (node.getUpdateCallback()) {
            osgAnimation::BasicAnimationManager* b = dynamic_cast<osgAnimation::BasicAnimationManager*>(node.getUpdateCallback());
            if (b) {

                _am = b;//new osgAnimation::BasicAnimationManager(*b);
                return;
            }
        }
        traverse(node);
    }
};

class RigSimplifier: public osgUtil::Simplifier {

public:
    RigSimplifier(double sampleRatio=1.0,float weighttreshold=0.05f, double maximumError=FLT_MAX, double maximumLength=0.0):osgUtil::Simplifier(sampleRatio, maximumError, maximumLength),_weighttreshold(weighttreshold) {}
    osgAnimation::RigGeometry* rig;
    unsigned int oldverticesize;
    unsigned int newindex;
    float _weighttreshold;
    //typedef std::pair<osgAnimation::VertexInfluence*,osgAnimation::VertexInfluence::iterator > VecandInfIt;
    //typedef std::pair<osgAnimation::VertexInfluence*,float > VecandInfIt;
    typedef std::pair<std::string,float > BoneVecandInf;
    std::vector< std::vector<  BoneVecandInf> > index2influences;

    typedef std::map<std::string,float> Bone2Weight;
    typedef std::map<osg::ref_ptr<osgUtil::EdgeCollapse::Point> ,Bone2Weight > Point2BoneWeight;
    Point2BoneWeight tempPoints;

    virtual void simplify(osg::Geometry& geometry) {
        osgUtil::Simplifier::simplify(geometry);
    }
    virtual void simplify(osg::Geometry& geom, const IndexList& protectedPoints)
    {
        rig=dynamic_cast<osgAnimation::RigGeometry*>(&geom);

        ///construct index2influence
        oldverticesize = newindex = rig->getSourceGeometry()->getVertexArray()->getNumElements();
        index2influences.resize(oldverticesize);
        osgAnimation::VertexInfluenceMap & imap=*rig->getInfluenceMap();
        for(osgAnimation::VertexInfluenceMap::iterator mapit=imap.begin(); mapit!=imap.end(); ++mapit) {
            osgAnimation::IndexWeightList &curvecinf=mapit->second;
            for(osgAnimation::IndexWeightList::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end(); ++curinf) {
                osgAnimation:: VertexIndexWeight& inf=*curinf;
                index2influences[inf.first].push_back(BoneVecandInf(mapit->first, inf.second) );

            }
        }

        ///copy/paste from original Simplifier
        bool downSample = requiresDownSampling();

        osgUtil::EdgeCollapse ec(this);
        ec.setComputeErrorMetricUsingLength(!downSample);
        ec.setGeometry(rig->getSourceGeometry(), protectedPoints);
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


        std::vector<uint> old2new;
        for(unsigned int i=0; i<newindex; ++i)old2new.push_back(newindex);
        unsigned int cpt=0;
        for(osgUtil::EdgeCollapse::PointSet::iterator itp=ec._pointSet.begin(); itp != ec._pointSet.end(); ++itp)
            old2new[(*itp)->_index]=cpt++;
        ///NB: copyback modify _index for reindexation so pick the oldone before
        ///
        ec.copyBackToGeometry();

        /*  if (_smoothing)
          {
              osgUtil::SmoothingVisitor::smooth(geometry);
          }

          if (_triStrip)
          {
              osgUtil::TriStripVisitor stripper;
              stripper.stripify(geometry);
          }*/

        ///post simplifier : change Influences according ec points old and new indices
        /*  std::vector<uint> old2new;
        for(unsigned int i=0; i<newindex; ++i)old2new.push_back(newindex);
        unsigned int cpt=0;
        for(osgUtil::EdgeCollapse::PointList::iterator itp=ec._originalPointList.begin(); itp != ec._originalPointList.end(); ++itp)
           old2new[(*itp)->_index]=(*itp)->_newindex;*/
        // osgAnimation::VertexInfluenceMap & imap=*rig->getInfluenceMap();
        for(osgAnimation::VertexInfluenceMap::iterator mapit=imap.begin(); mapit!=imap.end(); ++mapit) {
            osgAnimation::IndexWeightList &curvecinf=mapit->second;
            for(osgAnimation::IndexWeightList::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end();) {
                osgAnimation:: VertexIndexWeight& inf=*curinf;
                if(old2new[inf.first]!=newindex) {
                    inf.first=old2new[inf.first];
                    ++curinf;
                } else {
                    curinf=curvecinf.erase(curinf);
                }
            }
        }
        rig->copyFrom(*rig->getSourceGeometry());
    }

    virtual void OnCollapseEdge(osgUtil::EdgeCollapse::Edge* , osgUtil::EdgeCollapse::Point* pNew) {
        //onCollapseEdge
        Bone2Weight & pnewinfs=tempPoints[pNew];
        pNew->_index = newindex++;
        if(index2influences.size()<newindex)index2influences.resize(newindex);
        for(Bone2Weight::iterator infit=pnewinfs.begin(); infit!=pnewinfs.end(); ++infit) {
            if(  infit->second >_weighttreshold) {
                osgAnimation::IndexWeightList &bonevec= (*rig->getInfluenceMap())[infit->first];
                bonevec.push_back( osgAnimation::VertexIndexWeight(pNew->_index , infit->second ) );
                index2influences[pNew->_index].push_back(BoneVecandInf(infit->first, infit->second  ));
            }
        }
    }
    virtual bool divideEdge(osgUtil::EdgeCollapse::Edge* edge, osgUtil::EdgeCollapse::Point* pNew) {
        OSG_WARN<<"EdgeDivide"<<std::endl;
    }
    virtual osgUtil::EdgeCollapse::Point* computeInterpolatedPoint(osgUtil::EdgeCollapse::Edge* edge,float r) {

        osgUtil::EdgeCollapse::Point* p1 = edge->_p1.get();
        osgUtil::EdgeCollapse::Point* p2 = edge->_p2.get();

        if (p1==0 || p2==0)
        {
            OSG_NOTICE<<"Error computeInterpolatedPoint("<<edge<<",r) p1 and/or p2==0"<<std::endl;
            return 0;
        }

        osgUtil::EdgeCollapse::Point* point = new osgUtil::EdgeCollapse::Point;
        //point->_index = newindex++;
        assert(r>=0&&r<=1);
        float r1 = 1.0f-r;
        float r2 = r;

        point->_vertex = p1->_vertex * r1 + p2->_vertex * r2;
        unsigned int s = osg::minimum(p1->_attributes.size(),p2->_attributes.size());
        for(unsigned int i=0; i<s; ++i)
        {
            point->_attributes.push_back(p1->_attributes[i]*r1 + p2->_attributes[i]*r2);
        }

        //create new influences for new Point combining influences of p1 and p2

        // index2influences.push_back( std::vector<  BoneVecandInfIt> ());
        std::vector<  BoneVecandInf> & p1i=index2influences[p1->_index];
        std::vector<  BoneVecandInf> & p2i=index2influences[p2->_index];
        bool found=false;
        for( std::vector<  BoneVecandInf>::iterator iit=p1i.begin(); iit!=p1i.end(); ++iit) {
            found=false;
            for( std::vector<  BoneVecandInf>::iterator iit2=p2i.begin(); iit2!=p2i.end(); ++iit2) {
                if( iit->first==iit2->first) {
                    ///same bone so interpolate
                    //if( iit->second *r1  +iit2->second*r2>_weighttreshold)
                    tempPoints[point][iit->first]=  iit->second *r1      +iit2->second*r2;
                    found=true;
                    break;
                }
            }

            if(!found) {
                ///not found  in both so simply copy p1inf
                //if( iit->second *r1>_weighttreshold)
                tempPoints[point][iit->first]=  iit->second*r1 ;
            }

        }
        for( std::vector<  BoneVecandInf>::iterator iit2=p2i.begin(); iit2!=p2i.end(); ++iit2) {
            found=false;
            for( std::vector<  BoneVecandInf>::iterator iit=p1i.begin(); iit!=p1i.end(); ++iit) {
                if( iit->first==iit2->first) {
                    ///already done earlier
                    found=true;
                    break;
                }
            }

            if(!found) {
                ///not found  in both so simply copy p2inf
                // if( iit2->second*r2>_weighttreshold)
                tempPoints[point][iit2->first]=  iit2->second*r2;
            }

        }
        return point;
    }
};

struct SetupRigGeometry : public osg::NodeVisitor
{
    bool _hardware;
    float _simplifierRatio;
    float _simplifierWeightTreshold;
    SetupRigGeometry(float simplifierRatio=0.1,float simplifierWeightTreshold=0.05, bool hardware = true) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), _hardware(hardware)
        ,_simplifierRatio(simplifierRatio),_simplifierWeightTreshold(simplifierWeightTreshold) {}

    void apply(osg::Geode& geode)
    {
        for (unsigned int i = 0; i < geode.getNumDrawables(); i++)
            apply(*geode.getDrawable(i));
    }
    void apply(osg::Drawable& geom)
    {
        if (_hardware) {
            osgAnimation::RigGeometry* rig = dynamic_cast<osgAnimation::RigGeometry*>(&geom);
            if (rig) {
                rig->getInfluenceMap()->normalize(rig->getSourceGeometry()->getVertexArray()->getNumElements());
                rig->getInfluenceMap()->cullInfluenceCountPerVertex(8,_simplifierWeightTreshold);

                rig->setRigTransformImplementation(new osgAnimation::RigTransformHardware);
                rig->getRigTransformImplementation()->prepareData(*rig);
                //rig->getInfluenceMap()->cullBoneCountPerMesh(2);
                //rig->buildVertexInfluenceSet();
#if 1
                //simplify
                osg::ref_ptr<RigSimplifier> simp=new RigSimplifier(_simplifierRatio,_simplifierWeightTreshold);

                const osg::BoundingBox& bb=rig->getBoundingBox();
                osg::Vec3 vb=bb._max-bb._min;
                simp->setMaximumError( //0.006*osg::maximum(vb[2],osg::maximum(vb[0],vb[1])));
                    0.006*(vb).length());
                osgAnimation::MorphGeometry *morph;
                // osg::ref_ptr<osg::UIntArray> res=
                if(!(morph=dynamic_cast<osgAnimation::MorphGeometry*>(rig->getSourceGeometry()))) {
                    if(_simplifierRatio<1.0f)
                        simp->simplify (*((osg::Geometry*)rig)) ;
                } else {
#if 1
                    ///convert morph to geometry

                    osg::ref_ptr<osg::Geometry> ge=new osg::Geometry();//*morph,osg::CopyOp::DEEP_COPY_ALL);
                    ge->setUseVertexArrayObject(true);
                    ge->setUseDisplayList(false);
                    osg::Geometry& target=*ge.get();
                    osgAnimation::MorphGeometry &from=*morph;



                    target.setStateSet(from.getStateSet());

                    // copy over primitive sets.
                    target.getPrimitiveSetList() = from.getPrimitiveSetList();

                    if (from.getVertexSource())
                    {
                        target.setVertexArray(from.getVertexSource());
                    }

                    if (from.getNormalSource())
                    {
                        target.setNormalArray(from.getNormalSource(), osg::Array::BIND_PER_VERTEX);
                    }

                    if (from.getColorArray())
                    {
                        target.setColorArray(from.getColorArray(), osg::Array::BIND_PER_VERTEX);
                    }

                    if (from.getSecondaryColorArray())
                    {
                        target.setSecondaryColorArray(from.getSecondaryColorArray(), osg::Array::BIND_PER_VERTEX);
                    }

                    if (from.getFogCoordArray())
                    {
                        target.setFogCoordArray(from.getFogCoordArray(), osg::Array::BIND_PER_VERTEX);
                    }

                    for(unsigned int ti=0; ti<from.getNumTexCoordArrays(); ++ti)
                    {
                        if (from.getTexCoordArray(ti))
                        {
                            target.setTexCoordArray(ti,from.getTexCoordArray(ti), osg::Array::BIND_PER_VERTEX);
                        }
                    }

                    osg::Geometry::ArrayList& arrayList = from.getVertexAttribArrayList();
                    for(unsigned int vi=0; vi< arrayList.size(); ++vi)
                    {
                        osg::Array* array = arrayList[vi].get();
                        if (array)
                        {
                            target.setVertexAttribArray(vi,array, osg::Array::BIND_PER_VERTEX);
                        }
                    }


                    if(_simplifierRatio<1.0f) rig->setSourceGeometry(ge);
                    from.setMorphTransformImplementation(new osgAnimation::MorphTransformHardware);

                    rig->setRigTransformImplementation(new osgAnimation::RigTransformHardware);

                    if(_simplifierRatio<1.0f)      simp->simplify (*((osg::Geometry*)rig)) ;


#else
                    ///try to simplify morphgeometry but fails // to do (if it is usefull:/) create a simplifier for morph
                    for(int i=0; i<morph->getMorphTargetList().size(); i++)
                    {
                        OSG_WARN<<"morphsimplify"<<i<<std::endl;
                        osg::ref_ptr<osgUtil::Simplifier> simp2=new osgUtil::Simplifier(_simplifierRatio,simp->getMaximumError());
                        if(  morph->getMethod()!=osgAnimation::MorphGeometry::RELATIVE)
                            if(_simplifierRatio<1.0f)   simp2->simplify(*  morph->getMorphTarget(i).getGeometry());
                            else {
                                osg::ref_ptr<osg::Geometry> ge=morph->getMorphTarget(i).getGeometry();//new osg::Geometry;
                                osg::Vec3Array* v=new osg::Vec3Array( osg::Array::BIND_PER_VERTEX);
                                osg::Vec3Array* n=new osg::Vec3Array( osg::Array::BIND_PER_VERTEX);
                                osg::Vec3Array* mtv=(osg::Vec3Array*)morph->getMorphTarget(i).getGeometry()->getVertexArray();
                                osg::Vec3Array* mtn=(osg::Vec3Array*)morph->getMorphTarget(i).getGeometry()->getNormalArray();
                                /*  osg::Vec3Array* ovs=(osg::Vec3Array*)morph->getVertexArray();
                                   osg::Vec3Array* ons=(osg::Vec3Array*)morph->getNormalArray();*/
                                osg::Vec3Array* ovs=(osg::Vec3Array*)morph->getVertexSource();
                                osg::Vec3Array* ons=(osg::Vec3Array*)morph->getNormalSource();

                                for(unsigned int j=0; j< ovs->getNumElements(); ++j) {
                                    v->push_back( (*ovs)[j]+(*mtv)[j]);
                                    n->push_back((*ons)[j]+(*mtn)[j]);
                                }

                                ge->setVertexArray(v);
                                ge->setNormalArray(n, osg::Array::BIND_PER_VERTEX);
                                if(_simplifierRatio<1.0f)   simp2->simplify(*ge.get());
                                // morph->getMorphTarget(i).setGeometry(ge);

                            }

                    }
                    if(  morph->getMethod()!=osgAnimation::MorphGeometry::RELATIVE)
                        morph->setMethod(osgAnimation::MorphGeometry::NORMALIZED);
                    //osg::ref_ptr<osgUtil::Simplifier> simp=new osgUtil::Simplifier(_simplifierRatio);


                    morph->setVertexArray(morph->getVertexSource());
                    morph->setNormalArray(morph->getNormalSource());
                    if(_simplifierRatio<1.0f)  simp->simplify(*rig);
                    morph->setVertexSource((osg::Vec3Array*)morph->getVertexArray());
                    morph->setNormalSource((osg::Vec3Array*)morph->getNormalArray());
                    //  morph->setVertexArray(0);
                    //   morph->setNormalArray(0);
#endif

                }

                rig->getInfluenceMap()->normalize(rig->getSourceGeometry()->getVertexArray()->getNumElements());
                osgAnimation:: RigGeometry::FindNearestParentSkeleton finder;
                if(rig->getParents().size() > 1)
                    osg::notify(osg::WARN) << "A RigGeometry should not have multi parent ( " << rig->getName() << " )" << std::endl;
                rig->getParents()[0]->accept(finder);

                if(finder._roots.empty() || !finder._roots[0].valid())
                {
                    osg::notify(osg::WARN) << "A RigGeometry did not find a parent skeleton for RigGeometry ( " << rig->getName() << " )" << std::endl;
                    return;
                }

                for(int i=0;i<finder._roots.size();++i)
                rig->addSkeleton(finder._roots[i].get());

                rig->getInfluenceMap()->cullInfluenceCountPerVertex(8,_simplifierWeightTreshold);
                //rig->getInfluenceMap()->        accumulateDuplicates();
                  rig->getInfluenceMap()->removeUnexpressedBones(rig->getSkeleton());

                rig->getRigTransformImplementation()->prepareData(*rig);


#endif
                morph = dynamic_cast<osgAnimation::MorphGeometry*>(rig->getSourceGeometry());
                if(morph) {
                    morph->setMorphTransformImplementation(new osgAnimation::MorphTransformHardware);

                }
                rig->setRigTransformImplementation(new osgAnimation::RigTransformHardware());

            }
        }

#if 0
        if (geom.getName() != std::string("BoundingBox")) // we disable compute of bounding box for all geometry except our bounding box
            geom.setComputeBoundingBoxCallback(new osg::Drawable::ComputeBoundingBoxCallback);
//            geom.setInitialBound(new osg::Drawable::ComputeBoundingBoxCallback);
#endif
    }
};

osg::Group* createCharacterInstance(osg::Group* character, bool hardware,float simplifierRatio=0.01,float simplifierWeightTreshold=0.05)
{
    osg::ref_ptr<osg::Group> c ;
    if (hardware)
        c = osg::clone(character, osg::CopyOp::DEEP_COPY_ALL & ~osg::CopyOp::DEEP_COPY_PRIMITIVES & ~osg::CopyOp::DEEP_COPY_ARRAYS);
    else
        c = osg::clone(character, osg::CopyOp::DEEP_COPY_ALL);

    AnimationManagerFinder animfinder;
    c->accept(animfinder);
    if(animfinder._am.valid()) {
        osgAnimation::AnimationManagerBase* animationManager = dynamic_cast<osgAnimation::AnimationManagerBase*>(animfinder._am.get());

        osgAnimation::BasicAnimationManager* anim = dynamic_cast<osgAnimation::BasicAnimationManager*>(animationManager);
        const osgAnimation::AnimationList& list = animationManager->getAnimationList();
        int v = getRandomValueinRange(list.size());
        //if (list[v]->getName() == std::string("MatIpo_ipo"))
        {
            anim->playAnimation(list[v].get());
            v = (v + 1)%list.size();
        }

        anim->playAnimation(list[v].get());
    } else
    {
        osg::notify(osg::FATAL) << "no AnimationManagerBase found, updateCallback need to animate elements" << std::endl;
        return c.release();//exit(-1);
    }
    SetupRigGeometry switcher(simplifierRatio, simplifierWeightTreshold,hardware);
    c->accept(switcher);

    return c.release();
}
typedef std::vector<const osg::Material*> MatList;
typedef std::map<osg::StateSet*, int/*matid*/>  StateSetSet;

struct PhongMaterial {
    osg::Vec4 _ambientfront;
    osg::Vec4 _diffusefront;
    osg::Vec4 _specularfront;
    osg::Vec4 _specExponentAndPadding;
};



///parse statesets and put material in an array
class MaterialArrayVisitor : public osg::NodeVisitor
{
public:
    META_NodeVisitor(osgAnimation, MaterialArrayVisitor)
    MaterialArrayVisitor(unsigned int ubbindex=7): osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
        _matArray= new osg::BufferTemplate<std::vector<PhongMaterial> > ;
        _matsUBO= new osg::UniformBufferBinding(ubbindex);
    }
    osg::ref_ptr<osg::UniformBufferBinding >_matsUBO;
    osg::ref_ptr<osg::BufferTemplate<std::vector<PhongMaterial> > >_matArray;

    //void apply(osg::Node&);
    void apply(osg::Geometry& node) {

        osg::StateSet * ss;

        if(ss=node.getStateSet())
        {
            /*const osg::StateSet::TextureAttributeList &texatts=ss->getTextureAttributeList();
            int curtu=0;
            for(osg::StateSet::TextureAttributeList::const_iterator texatt=texatts.begin();texatt!=texatts.end();++texatt,++curtu )
            {
                 const osg::StateSet::AttributeList  &texattlist=*texatt;

                 for(osg::StateSet::AttributeList::const_iterator att=texattlist.begin();att!=texattlist.end();++att ){

                     const osg::StateAttribute::TypeMemberPair &typemember=(*att).first;
                     const osg::StateSet::RefAttributePair &attr=(*att).second;
            if(attr.first->asTexture()){
            std::cout<<typemember.second<<"TexAtt foudn"<<attr.first->getName()<<std::endl;

            }
                 }
            }
            _list.push_back(&node);*/

            if(_list.find(ss)==_list.end()) {
                //const osg::StateSet::TextureAttribute* mat=ss->getAttribute(osg::StateAttribute::MATERIAL);
                const osg::Material* mat=dynamic_cast<osg::Material*>(ss->getAttribute(osg::StateAttribute::MATERIAL));
                if(mat)
                {
                    unsigned int cpt=0;
                    MatList::iterator matit;
                    for(matit=_mats.begin(); matit!=_mats.end(); ++matit,++cpt) {
                        if( *matit==mat) {
                            _list[ss]=cpt;
                            break;
                        }
                    }
                    if(matit==_mats.end()) {
                        _list[ss]=cpt;
                        _mats.push_back(mat);

                        PhongMaterial pmat;
                        pmat._ambientfront=mat->getAmbient(osg::Material::FRONT);
                        pmat._diffusefront=mat->getDiffuse(osg::Material::FRONT);
                        pmat._specularfront=mat->getSpecular(osg::Material::FRONT);
                        pmat._specExponentAndPadding[0]=mat->getShininess(osg::Material::FRONT);

                        _matArray->getData().push_back(pmat);

                    }

                    ///add ubo and uniform to the ss
                    ss->setAttribute(_matsUBO);
                    //DEBUGrm    ss->addUniform(new osg::Uniform(osg::Uniform::UNSIGNED_INT,"materialIndex",cpt));
                    (ss)->removeAttribute(const_cast<osg::Material*>(mat));
                }
            }
        }
    }
///finalizing stuff
    void generateMaterialArrayBufferBinding() {
        _matsUBO->setBufferData(_matArray);
        for(StateSetSet::iterator ssit=_list.begin(); ssit!= _list.end(); ++ssit) {

            //if(ssit->second)
        }
    }

    const StateSetSet& getMaterialStateSet() const
    {
        return _list;
    }

protected:
    StateSetSet _list;
    MatList _mats;
};

/*
class VirtualGeometryVisitor : public osg::NodeVisitor
{
public:
    META_NodeVisitor(osgAnimation, VirtualGeometryVisitor)

    VirtualGeometryVisitor(): osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
    }
    typedef std::vector<VirtualGeometry*> VGS;
    void apply(osg::Geometry& node) {
        VGS::iterator vgit;
        for( vgit=vgs.begin(); vgit!=vgs.end() && !(*vgit)->addChild(node) ; ++vgit);
        if(vgit==vgs.end()) {
            VirtualGeometry *vg=new VirtualGeometry();
            vgs.push_back(vg);
            if(!vg->addChild( node))
                OSG_WARN<<" WTF addchild failed"<<std::endl;

        }
    }


    VGS vgs;
};*/

typedef std::vector<osgAnimation::RigGeometry*> RigList;
class CollectRigVisitor : public osg::NodeVisitor
{
public:
    META_NodeVisitor(osgAnimation, CollectRigVisitor)
    CollectRigVisitor();

    //void apply(osg::Node&);
    void apply(osg::Geometry& node);
    const RigList& getRigList() const
    {
        return _list;
    }

protected:
    RigList _list;
};
CollectRigVisitor::CollectRigVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

//void CollectRigVisitor::apply(osg::Node&) { return; }
void CollectRigVisitor::apply(osg::Geometry& node)
{
    osgAnimation::RigGeometry* bone = dynamic_cast<osgAnimation::RigGeometry*>(&node);
    if (bone)
    {
        _list.push_back( bone);
        traverse(node);
    }

}



int main (int argc, char* argv[])
{
    std::cerr << "This example works better with nathan.osg" << std::endl;

    osg::ArgumentParser psr(&argc, argv);

    osgViewer::Viewer viewer(psr);

    bool hardware = true;
    int maxChar = 1;
    float ratio=0.1;
    float threshold=0.05;
    while (psr.read("--software")) {
        hardware = false;
    }
    while (psr.read("--number", maxChar)) {}
    while (psr.read("--ratio", ratio)) {}
    while (psr.read("--threshold", threshold)) {}


    osg::ref_ptr<osg::Group> root = dynamic_cast<osg::Group*>(osgDB::readNodeFiles(psr));
    if (!root)
    {
        std::cout << psr.getApplicationName() <<": No data loaded" << std::endl;
        return 1;
    }

    /*{
        osgAnimation::AnimationManagerBase* animationManager = dynamic_cast<osgAnimation::AnimationManagerBase*>(root->getUpdateCallback());
        if(!animationManager)
        {
            osg::notify(osg::FATAL) << "no AnimationManagerBase found, updateCallback need to animate elements" << std::endl;
            return 1;
        }
    }*/


    osg::ref_ptr<osg::Group> scene = new osg::Group;

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the thread model handler
    viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the help handler
    viewer.addEventHandler(new osgViewer::HelpHandler(psr.getApplicationUsage()));

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    // add the screen capture handler
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);


            osg::ref_ptr<osg::Group> c = createCharacterInstance(root.get(), hardware,ratio,threshold);
            osg::MatrixTransform* tr = new osg::MatrixTransform;
            tr->setMatrix(osg::Matrix::translate( 2.0 * (1),
                                                  0.0,
                                                  2.0 * (1)));
            tr->addChild(c.get());
            scene->addChild(tr);


    osgDB::writeNodeFile(*scene.get(),"testHW.osgb");

    viewer.setSceneData(scene.get());

    viewer.realize();
    return viewer.run();
}


