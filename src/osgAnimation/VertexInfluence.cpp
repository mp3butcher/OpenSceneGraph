/*  -*-c++-*-
 *  Copyright (C) 2008 Cedric Pinson <cedric.pinson@plopbyte.net>
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
#include <osg/Notify>
#include <iostream>
#include <algorithm>
#include <set>
#include <osgAnimation/BoneMapVisitor>
#include <osgAnimation/RigGeometry>
#include <osgDB/OutputStream>
#include <osgDB/ClassInterface>
#include <osgDB/Options>

using namespace osgAnimation;
#if 0
void VertexInfluenceSet::addBoneInfluenceList(const BoneInfluenceList& v) {
    _bone2Vertexes.push_back(v);
}
// this class manage VertexInfluence database by mesh
// reference bones per vertex ...
struct invweight_ordered2
{
    inline bool operator() (const VertexInfluenceSet::BoneWeight& bw1, const VertexInfluenceSet::BoneWeight& bw2)
    {
        /*  if (bw1.getWeight() > bw2.getWeight())return true;
          if (bw1.getWeight() < bw2.getWeight())return false;
          return(bw1.getBoneName()<bw2.getBoneName());*/
        if (bw1.getWeight() < bw2.getWeight())
            return true;
        else if (bw1.getWeight() > bw2.getWeight())
            return false;
        if (bw1.getBoneName() < bw2.getBoneName())
            return true;
        return false;
    }
};
void VertexInfluenceSet::buildVertexToBoneWeightList(unsigned int numvertices)
{
    _vertex2Bones.clear();
    _vertex2Bones.reserve(numvertices);
    _vertex2Bones.resize(numvertices);
    typedef std::set<BoneWeight ,invweight_ordered2>  BoneWeightOrdered;
    std::map<int,BoneWeightOrdered,std::less<int> >tempVec2Bones;
    //unsigned int maxindex=0;
    for (BoneToVertexList::const_iterator it = _bone2Vertexes.begin(); it != _bone2Vertexes.end(); ++it)
    {
        const BoneInfluenceList& vi = (*it);
        int size = vi.size();
        for (int i = 0; i < size; i++)
        {
            IndexWeight viw = vi[i];
            int index = viw.first;
            float weight = viw.second;
            if (vi.getBoneName().empty()) {
                OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning vertex " << index << " is not assigned to a bone" << std::endl;
            }
            // if(maxindex<index)maxindex=index;
            _vertex2Bones[index].push_back(BoneWeight(vi.getBoneName(), weight));
            //tempVec2Bones[index].insert(BoneWeight(vi.getName(), weight));
            ;
        }
    }

    /*  if(tempVec2Bones.size()<numvertices ){
           OSG_WARN << "VertexInfluenceSet::some vertices may not have defined influences " << std::endl;

       }
       //copy tempVec2Bones to _vertex2Bones

       for(std::map<int,BoneWeightOrdered>::iterator iti = tempVec2Bones.begin(); iti != tempVec2Bones.end(); ++iti){
           BoneWeightList& destbonelist=_vertex2Bones[iti->first];

           for(BoneWeightOrdered::iterator itbw = iti->second.begin();itbw != iti->second.end(); ++itbw)
               destbonelist.push_back(*itbw);
             if( destbonelist.size()!=iti->second.size())
                 OSG_WARN<<"WTF"<<std::endl;
    }*/
    // normalize weight per vertex
    unsigned int vertid=0;
    for (VertIDToBoneWeightList::iterator it = _vertex2Bones.begin(); it != _vertex2Bones.end(); ++it,++vertid)
    {
        BoneWeightList& bones =*it;//->second;
        int size = bones.size();
        float sum = 0;
        for (int i = 0; i < size; i++)
            sum += bones[i].getWeight();
        if (sum < 1e-4)
        {
            OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning the vertex " <<vertid << " seems to have 0 weight, skip normalize for this vertex" << std::endl;
        }
        else
        {
            float mult = 1.0/sum;
            for (int i = 0; i < size; i++)
                bones[i].setWeight(bones[i].getWeight() * mult);
        }
    }
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

void VertexInfluenceSet::clear()
{
    _bone2Vertexes.clear();
    _uniqVertexSetToBoneSet.clear();
}

void VertexInfluenceSet::buildUniqVertexGroupList()
{
    _uniqVertexSetToBoneSet.clear();

    typedef std::map<BoneWeightList,VertexGroup, SortByBoneWeightList> UnifyBoneGroup;
    UnifyBoneGroup unifyBuffer;

    unsigned int vertexID=0;
    for (VertIDToBoneWeightList::iterator it = _vertex2Bones.begin(); it != _vertex2Bones.end(); ++it,++vertexID)
    {
        BoneWeightList boneweightlist = *it;//->second;
        //int vertexIndex = it->first;

        // sort the vector to have a consistent key
        std::sort(boneweightlist.begin(), boneweightlist.end(), SortByNameAndWeight());

        // we use the vector<BoneWeight> as key to differentiate group
        UnifyBoneGroup::iterator result = unifyBuffer.find(boneweightlist);
        if (result == unifyBuffer.end())
            unifyBuffer[boneweightlist].setBoneWeightList(boneweightlist);
        unifyBuffer[boneweightlist].getVertIDs().push_back(vertexID);
    }
    if(_vertex2Bones.size()==unifyBuffer.size()) {
        OSG_WARN << "VertexInfluenceSet::buildmap is useless no duplicate VertexGroup" << std::endl;

    }
    _uniqVertexSetToBoneSet.reserve(unifyBuffer.size());


    for (UnifyBoneGroup::iterator it = unifyBuffer.begin(); it != unifyBuffer.end(); ++it)
    {
        _uniqVertexSetToBoneSet.push_back(it->second);
    }
    ///DEBUG
    /// rebuild _vertex2bone
    /* _vertex2Bones.clear();
    for (UnifyBoneGroup::iterator it = unifyBuffer.begin(); it != unifyBuffer.end(); ++it)
    {
        const BoneWeightList& bwl=it->first;
        VertexGroup &vsbs=it->second;
        for(std::vector<unsigned int>::iterator vertit=vsbs.getVertexes().begin();vertit!=vsbs.getVertexes().end();++vertit){
            _vertex2Bones[*vertit]=vsbs.getBones();
        }


    }*/
}
#endif

struct invweight_ordered
{
    inline bool operator() (const BoneWeight& bw1, const BoneWeight& bw2)
    {
        if (bw1.getWeight() > bw2.getWeight())return true;
        if (bw1.getWeight() < bw2.getWeight())return false;
        return(bw1.getBoneName()<bw2.getBoneName());
    }
};
///cull weakest influences in order to fit targetted numbonepervertex
void VertexInfluenceMap::cullBoneInfluenceCountPerVertex(unsigned int numbonepervertex,float minweight,bool makeOddWeightPerVertex, bool renormalize) {

    typedef std::set<BoneWeight,invweight_ordered >  BoneWeightOrdered;
    std::map<int,BoneWeightOrdered > tempVec2Bones;
    for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end(); ++mapit) {
        IndexWeightList &curvecinf=mapit->second;
        const std::string bonename=mapit->first;
        for(IndexWeightList::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end(); ++curinf) {
            IndexWeight& inf=*curinf;
            if( bonename.empty()) {
                OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning vertex " << inf.first << " is not assigned to a bone" << std::endl;
            }
            else if(inf.second>minweight)tempVec2Bones[inf.first].insert(BoneWeight(bonename, inf.second));
        }
    }
    this->clear();

    for( std::map<int,BoneWeightOrdered >::iterator mapit=tempVec2Bones.begin(); mapit!=tempVec2Bones.end(); ++mapit) {
        BoneWeightOrdered& bwset=mapit->second;
        unsigned int newsize=numbonepervertex<bwset.size()?numbonepervertex:bwset.size();
        float sum=0;

        if(makeOddWeightPerVertex &&bwset.size()<=newsize)if(bwset.size()%2==1&&bwset.size()>2)bwset.erase(*bwset.rbegin());
        while(bwset.size()>newsize)bwset.erase(*bwset.rbegin());

        if(makeOddWeightPerVertex)if(bwset.size()%2==1&&bwset.size()>2)bwset.erase(*bwset.rbegin());
        if(renormalize) {
            for(BoneWeightOrdered::iterator bwit=bwset.begin(); bwit!=bwset.end(); ++bwit)
                sum+=bwit->getWeight();
            if(sum>1e-4) {
                sum=1.0f/sum;
                for(BoneWeightOrdered::iterator bwit=bwset.begin(); bwit!=bwset.end(); ++bwit) {
                    IndexWeightList & inf= (*this)[bwit->getBoneName()];
                    inf.push_back(IndexWeight(mapit->first, bwit->getWeight()*sum));
                }
            }
        } else {
            for(BoneWeightOrdered::iterator bwit=bwset.begin(); bwit!=bwset.end(); ++bwit) {
                IndexWeightList & inf= (*this)[bwit->getBoneName()];
                inf.push_back(IndexWeight(mapit->first,bwit->getWeight()));
            }

        }
    }
}
void VertexInfluenceMap::normalize(unsigned int numvert) {

    typedef std::pair<float, std::vector<float*> > PerVertWeights;
    std::vector<PerVertWeights > localstore;
    localstore.resize(numvert);
    for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end(); ++mapit) {
        IndexWeightList &curvecinf=mapit->second;
        for(IndexWeightList::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end(); ++curinf) {
            IndexWeight& inf=*curinf;
            localstore[inf.first].first+=inf.second;
            localstore[inf.first].second.push_back(&inf.second);

        }
    }
    unsigned int vertid=0;
    for(std::vector<PerVertWeights >::iterator itvert=localstore.begin(); itvert!=localstore.end(); ++itvert, ++vertid) {
        PerVertWeights & weights=*itvert;
        if(weights.first< 1e-4)
        {
            OSG_WARN << "VertexInfluenceMap::normalize warning the vertex " <<vertid << " seems to have 0 weight, skip normalize for this vertex" << std::endl;
        }
        else
        {
            float mult = 1.0/weights.first;
            for (std::vector<float*>::iterator itf =weights.second.begin(); itf!=weights.second.end(); ++itf)
                **itf*=mult;
        }
    }

}

void VertexInfluenceMap::cullBoneCountPerMesh(unsigned int numbonepermesh) {
    if (this->size()<=numbonepermesh)
        return;

    typedef std::set<BoneWeight, invweight_ordered>  BoneWeightOrdered;
    typedef std::map<std::string,std::pair<float,unsigned int> >  BoneName2TotalInf;
    BoneName2TotalInf bone2total;
    for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end(); ++mapit) {
        IndexWeightList &curvecinf=mapit->second;
        const std::string& bonename= mapit->first;
        std::pair<float,unsigned int> & bonetotal=bone2total[bonename];
        for(IndexWeightList::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end(); ++curinf) {
            IndexWeight& inf=*curinf;
            if( bonename.empty()) {
                OSG_WARN << "VertexInfluenceMap::cullBoneCountPerMesh warning vertex " << inf.first << " is not assigned to a bone" << std::endl;
            }
            else {
                bonetotal.first+= inf.second;
                bonetotal.second++;
            }

        }
    }
    ///sorting using a set
    BoneWeightOrdered totalinfset;
    for(BoneName2TotalInf::const_iterator mapit=bone2total.begin(); mapit!=bone2total.end(); ++mapit) {
        totalinfset.insert(BoneWeight(mapit->first,mapit->second.first/(float)mapit->second.second));
    }
    std::vector<BoneWeight> totalinfsorteddesc;
    for(BoneWeightOrdered::reverse_iterator it=totalinfset.rbegin(); it!=totalinfset.rend(); it++)
        totalinfsorteddesc.push_back(BoneWeight(it->getBoneName(),it->getWeight()));

    std::vector<BoneWeight>::iterator lastbone=totalinfsorteddesc.begin();
    while(totalinfsorteddesc.size()>numbonepermesh&&lastbone!=totalinfsorteddesc.end()) {

        //test bone removal==good if forall vert its not the unique influence
        bool goodforremoval=true;
        IndexWeightList &curvecinf=(*this)[lastbone->getBoneName()];
        for(IndexWeightList::iterator infit=curvecinf.begin(); infit!=curvecinf.end()&&goodforremoval; infit++) {

            uint index=infit->first;
            //check if index have other inf
            bool indok=false;
            for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end()&&!indok; ++mapit) {
                if(mapit->first!=lastbone->getBoneName()) {
                    for(VertexInfluence::iterator infit2=mapit->second.begin(); infit2!=mapit->second.end(); infit2++) {
                        if(infit2->first==index) {
                            //other inf found
                            indok=true;
                            break;
                        }
                    }
                }
            }
            if(!indok)goodforremoval=false;

        }
        if(goodforremoval) {
            OSG_WARN<<"droping bone"<<lastbone->getBoneName()<<" with average influence of "<<lastbone->getWeight()<<std::endl;
            this->erase( this->find(lastbone->getBoneName()));
            lastbone=totalinfsorteddesc.erase( lastbone );


        } else lastbone++;

    }

    if( this->size()!=numbonepermesh) {
        OSG_WARN<<"targetbonepermesh not reached "<<numbonepermesh<<"!="<<this->size()<<std::endl;

    }

}

//Expermental
typedef std::vector<RigGeometry*> RigList;
class CollectRigVisitor : public osg::NodeVisitor
{
public:
    META_NodeVisitor(osgAnimation, CollectRigVisitor)
    CollectRigVisitor();

    //void apply(osg::Node&);
    void apply(osg::Geometry& node);
    const RigList& getRigList() const;

protected:
    RigList _map;
};
CollectRigVisitor::CollectRigVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

//void CollectRigVisitor::apply(osg::Node&) { return; }
void CollectRigVisitor::apply(osg::Geometry& node)
{
    RigGeometry* bone = dynamic_cast<RigGeometry*>(&node);
    if (bone)
    {
        _map.push_back( bone);
        traverse(node);
    }
    Skeleton* skeleton = dynamic_cast<Skeleton*>(&node);
    if (skeleton)
        traverse(node);
}

const RigList& CollectRigVisitor::getRigList() const
{
    return _map;
}

void VertexInfluenceMap::destroyUnusedBones(Skeleton &skel) {
    BoneMapVisitor mapVisitor;
    skel.accept(mapVisitor);
    CollectRigVisitor rigvis;
    skel.accept(rigvis);
    RigList  rigs=rigvis.getRigList();
    BoneMap boneMap = mapVisitor.getBoneMap();
    Bone* child,*par;

    for(BoneMap::iterator bmit=boneMap.begin(); bmit!=boneMap.end();) {
        if( this->find(bmit->first) ==this->end())
        {
            bool isusless=true;
            for(RigList::iterator rigit=rigs.begin();
                    rigit!=rigs.end(); ++rigit) {
                if( ((*rigit)->getInfluenceMap()->find(bmit->first) !=(*rigit)->getInfluenceMap()->end())) {
                    isusless=false;
                    break;
                }
            }

            if(!isusless||!(par=bmit->second->getBoneParent())) {
                ++bmit;
                continue;
            }

            ///Bone can be removed
            Bone * bone2rm=bmit->second;
            /*   InvBindMatrixInSkeletonSpace,_boneInSkeletonSpace*/
            typedef std::vector<Bone*> BoneVec;
            BoneVec children;


//            osgDB::BaseSerializer* bs = bone2rm->getInvBindMatrixInSkeletonSpace().getClassInterface().getSerializer(object, containerPropertyName, type);

osgDB::OutputStream o(new osgDB::Options);
class OUT:public osgDB::OutputStream{
public:
   osgDB::OutputIterator *getOutputIterator(){return _out;}
};
    class CI:public osgDB::ClassInterface{

public:
   OUT& getOutputStream(){return (OUT&)_outputStream;}
};
CI ci;
ci.getOutputStream().getOutputIterator()->setStream(&std::cerr);

//o.get
            OSG_WARN<<"removing "<<bmit->first<<std::endl;
            ci.getOutputStream()<<bone2rm->getInvBindMatrixInSkeletonSpace()<<std::endl;
           // OSG_WARN<<"removing "<<bmit->first<<std::endl<<              ci.getOutputStream().getOutputIterator()->_s;

            for(unsigned int numchild=0; numchild<bone2rm->getNumChildren(); numchild++) {
                child=dynamic_cast<Bone*>(bone2rm->getChild(numchild));
                if(child) {
                    par->addChild(child);
                    bone2rm->removeChild(child);

                }
            }
            par->removeChild(bone2rm);

            skel.accept(mapVisitor);
            boneMap = mapVisitor.getBoneMap();
            bmit=boneMap.begin();

        } else ++bmit;
    }

}
