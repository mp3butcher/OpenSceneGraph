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


#include <osgAnimation/VertexInfluence>
#include <osgAnimation/RigTransformSoftware>
#include <osgAnimation/BoneMapVisitor>
#include <osgAnimation/RigGeometry>

#include <algorithm>
using namespace osgAnimation;

RigTransformSoftware::RigTransformSoftware()
{
    _needInit = true;
}

RigTransformSoftware::RigTransformSoftware(const RigTransformSoftware& rts,const osg::CopyOp& copyop):
    RigTransform(rts, copyop),
    _needInit(rts._needInit),
    _invalidInfluence(rts._invalidInfluence)
{

}

// sort by name and weight
struct SortByNameAndWeight : public std::less<VertexInfluenceSet::BoneWeight>
{
    bool operator()(const VertexInfluenceSet::BoneWeight& b0,
                    const VertexInfluenceSet::BoneWeight& b1) const
    {
        if (b0.getBoneName() < b1.getBoneName())
            return true;
        else if (b0.getBoneName() > b1.getBoneName())
            return false;
        if (b0.getWeight() < b1.getWeight())
            return true;
        return false;
    }
};

struct SortByBoneWeightList : public std::less<VertexInfluenceSet::BoneWeightList>
{
    bool operator()(const VertexInfluenceSet::BoneWeightList& b0,
                    const VertexInfluenceSet::BoneWeightList& b1) const
    {
        if (b0.size() < b1.size())
            return true;
        else if (b0.size() > b1.size())
            return false;

        int size = b0.size();
        for (int i = 0; i < size; i++)
        {
            bool result = SortByNameAndWeight()(b0[i], b1[i]);
            if (result)
                return true;
            else if (SortByNameAndWeight()(b1[i], b0[i]))
                return false;
        }
        return false;
    }
};
/*
void RigTransformSoftware::prepareData(RigGeometry&rig){
    typedef std::vector<VertexInfluenceSet::BoneWeight> BoneWeightList;
    typedef std::map<unsigned int,BoneWeightList> VertIDToBoneWeightList;
    VertIDToBoneWeightList _vertex2Bones;
    _vertex2Bones.clear();

    VertexInfluenceSet::BoneToVertexList _bone2Vertexes;
  //  typedef std::set<BoneWeight ,invweight_ordered2>  BoneWeightOrdered;
    //  std::map<int,BoneWeightOrdered >tempVec2Bones;
    VertexInfluenceMap *_vertexInfluenceMap=rig.getInfluenceMap();
    for (osgAnimation::VertexInfluenceMap::iterator it = _vertexInfluenceMap->begin();
         it != _vertexInfluenceMap->end();
         ++it){

       //for(VertexInfluence::iterator vit=it->second.begin();vit!=it->second.end();++vit)
 if(it->first!=it->second.getName()){
        OSG_WARN << "buildVertexInfluenceSet can't be called without VertexInfluence already set to the RigGeometry ( " << getName() << " ) " << std::endl;
 }
        _bone2Vertexes.push_back(it->second);
          const VertexInfluence& vi = it->second;
          int size = vi.size();
          for (int i = 0; i < size; i++)
          {
              VertexIndexWeight viw = vi[i];
              int index = viw.first;
              float weight = viw.second;
              if (vi.getName().empty()){
                  OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning vertex " << index << " is not assigned to a bone" << std::endl;
              }
              int size=_vertex2Bones[index].size();
              //tempVec2Bones[index]
              _vertex2Bones[index].push_back(VertexInfluenceSet::BoneWeight(vi.getName(), weight));;
              if(++size!=_vertex2Bones[index].size())
                  OSG_WARN<<"WTF"<<std::endl;
          }
      }


    // normalize weight per vertex
    for (VertIDToBoneWeightList::iterator it = _vertex2Bones.begin(); it != _vertex2Bones.end(); ++it)
    {
        BoneWeightList& bones = it->second;
        int size = bones.size();
        float sum = 0;
        for (int i = 0; i < size; i++)
            sum += bones[i].getWeight();
        if (sum < 1e-4)
        {
            OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning the vertex " << it->first << " seems to have 0 weight, skip normalize for this vertex" << std::endl;
        }
        else
        {
            float mult = 1.0/sum;
            for (int i = 0; i < size; i++)
                bones[i].setWeight(bones[i].getWeight() * mult);
        }
    }
/*     VertexInfluenceSet::UniqVertexGroupList
     _uniqVertexSetToBoneSet;
    _uniqVertexSetToBoneSet.clear();
//typedef std::vector<UniqVertexGroup>
    typedef std::map<VertexInfluenceSet::BoneWeightList,VertexInfluenceSet::UniqVertexGroupList, SortByBoneWeightList> UnifyBoneGroup;
    UnifyBoneGroup unifyBuffer;

    for (VertIDToBoneWeightList::iterator it = _vertex2Bones.begin(); it != _vertex2Bones.end(); ++it)
    {
        BoneWeightList bones = it->second;
        int vertexIndex = it->first;

        // sort the vector to have a consistent key
        std::sort(bones.begin(), bones.end(), SortByNameAndWeight());

        // we use the vector<BoneWeight> as key to differentiate group
        UnifyBoneGroup::iterator result = unifyBuffer.find(bones);
        if (result == unifyBuffer.end())
            unifyBuffer[bones].setBones(bones);
        unifyBuffer[bones].getVertexes().push_back(vertexIndex);
    }

    _uniqVertexSetToBoneSet.reserve(unifyBuffer.size());


    for (UnifyBoneGroup::iterator it = unifyBuffer.begin(); it != unifyBuffer.end(); ++it)
    {
        _uniqVertexSetToBoneSet.push_back(it->second);
    }* /

}*/
bool RigTransformSoftware::init(RigGeometry& geom)
{
    if (!geom.getSkeleton())
        return false;

    BoneMapVisitor mapVisitor;
    geom.getSkeleton()->accept(mapVisitor);
    BoneMap bm = mapVisitor.getBoneMap();
    initVertexSetFromBones(bm, geom.getVertexInfluenceSet().getUniqVertexGroupList());

    if (geom.getSourceGeometry())
        geom.copyFrom(*geom.getSourceGeometry());
    geom.setVertexArray(0);
    geom.setNormalArray(0);

    _needInit = false;
    return true;
}

void RigTransformSoftware::operator()(RigGeometry& geom)
{
    if (_needInit)
        if (!init(geom))
            return;

    if (!geom.getSourceGeometry()) {
        OSG_WARN << this << " RigTransformSoftware no source geometry found on RigGeometry" << std::endl;
        return;
    }
    osg::Geometry& source = *geom.getSourceGeometry();
    osg::Geometry& destination = geom;

    osg::Vec3Array* positionSrc = dynamic_cast<osg::Vec3Array*>(source.getVertexArray());
    osg::Vec3Array* positionDst = dynamic_cast<osg::Vec3Array*>(destination.getVertexArray());
    if (positionSrc )
    {
        if (!positionDst || (positionDst->size() != positionSrc->size()) )
        {
            if (!positionDst)
            {
                positionDst = new osg::Vec3Array;
                positionDst->setDataVariance(osg::Object::DYNAMIC);
                destination.setVertexArray(positionDst);
            }
            *positionDst = *positionSrc;
        }
        if (!positionDst->empty())
        {
            compute<osg::Vec3>(geom.getMatrixFromSkeletonToGeometry(),
                               geom.getInvMatrixFromSkeletonToGeometry(),
                               &positionSrc->front(),
                               &positionDst->front());
            positionDst->dirty();
        }

    }

    osg::Vec3Array* normalSrc = dynamic_cast<osg::Vec3Array*>(source.getNormalArray());
    osg::Vec3Array* normalDst = dynamic_cast<osg::Vec3Array*>(destination.getNormalArray());
    if (normalSrc )
    {
        if (!normalDst || (normalDst->size() != normalSrc->size()) )
        {
            if (!normalDst)
            {
                normalDst = new osg::Vec3Array;
                normalDst->setDataVariance(osg::Object::DYNAMIC);
                destination.setNormalArray(normalDst, osg::Array::BIND_PER_VERTEX);
            }
            *normalDst = *normalSrc;
        }
        if (!normalDst->empty())
        {
            computeNormal<osg::Vec3>(geom.getMatrixFromSkeletonToGeometry(),
                               geom.getInvMatrixFromSkeletonToGeometry(),
                               &normalSrc->front(),
                               &normalDst->front());
            normalDst->dirty();
        }
    }

}

void RigTransformSoftware::initVertexSetFromBones(const BoneMap& map, const VertexInfluenceSet::UniqVertexGroupList& influence)
{
    _boneSetVertexSet.clear();

    int size = influence.size();
    _boneSetVertexSet.resize(size);
    for (int i = 0; i < size; i++)
    {
        const VertexInfluenceSet::VertexGroup& inf = influence[i];
        int nbBones = inf.getBones().size();
        BoneWeightList& boneList = _boneSetVertexSet[i].getBones();

        double sumOfWeight = 0;
        for (int b = 0; b < nbBones; b++)
        {
            const std::string& bname = inf.getBones()[b].getBoneName();
            float weight = inf.getBones()[b].getWeight();
            BoneMap::const_iterator it = map.find(bname);
            if (it == map.end() )
            {
                if (_invalidInfluence.find(bname) != _invalidInfluence.end()) {
                    _invalidInfluence[bname] = true;
                    OSG_WARN << "RigTransformSoftware Bone " << bname << " not found, skip the influence group " <<bname  << std::endl;
                }
                continue;
            }
            Bone* bone = it->second.get();
            boneList.push_back(BoneWeight(bone, weight));
            sumOfWeight += weight;
        }
        // if a bone referenced by a vertexinfluence is missed it can make the sum less than 1.0
        // so we check it and renormalize the all weight bone
        /*const double threshold = 1e-4;
        if (!_boneSetVertexSet[i].getBones().empty() &&
            (sumOfWeight < 1.0 - threshold ||  sumOfWeight > 1.0 + threshold))
        {
            for (int b = 0; b < (int)boneList.size(); b++)
                boneList[b].setWeight(boneList[b].getWeight() / sumOfWeight);
        }*/
        _boneSetVertexSet[i].getVertexes() = inf.getVertexes();
    }
}
