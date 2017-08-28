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
void VertexInfluenceMap::cullBoneInfluenceCountPerVertex(unsigned int numbonepervertex,float minweight, bool renormalize) {

    typedef std::set<BoneWeight,invweight_ordered >  BoneWeightOrdered;
    std::map<int,BoneWeightOrdered > tempVec2Bones;
    for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end(); ++mapit) {
        BoneInfluenceList &curvecinf=mapit->second;
        for(BoneInfluenceList::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end(); ++curinf) {
            IndexWeight& inf=*curinf;
            if( curvecinf.getBoneName().empty()) {
                OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning vertex " << inf.first << " is not assigned to a bone" << std::endl;
            }
            else if(inf.second>minweight)tempVec2Bones[inf.first].insert(BoneWeight(curvecinf.getBoneName(), inf.second));
        }
    }
    this->clear();
    for( std::map<int,BoneWeightOrdered >::iterator mapit=tempVec2Bones.begin(); mapit!=tempVec2Bones.end(); ++mapit) {
        BoneWeightOrdered& bwset=mapit->second;
        unsigned int newsize=numbonepervertex<bwset.size()?numbonepervertex:bwset.size();
        float sum=0;
        while(bwset.size()>newsize)bwset.erase(*bwset.rbegin());
        // if(renormalize)
        for(BoneWeightOrdered::iterator bwit=bwset.begin(); bwit!=bwset.end(); ++bwit)
            sum+=bwit->getWeight();
        sum=1.0f/sum;
        // if(!renormalize||sum>1e-4)
        for(BoneWeightOrdered::iterator bwit=bwset.begin(); bwit!=bwset.end(); ++bwit) {
            BoneInfluenceList & inf= (*this)[bwit->getBoneName()];
            inf.setBoneName(bwit->getBoneName());
            inf.push_back(IndexWeight(mapit->first,renormalize? bwit->getWeight()*sum: bwit->getWeight()));
        }
    }
}
void normalize(VertexInfluenceMap*map) {

    typedef std::pair<float, std::vector<float*> > PerVertWeights;
    std::map<unsigned int,PerVertWeights > localstore;
    for(VertexInfluenceMap::iterator mapit=map->begin(); mapit!=map->end(); ++mapit) {
        BoneInfluenceList &curvecinf=mapit->second;
        for(BoneInfluenceList::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end(); ++curinf) {
            IndexWeight& inf=*curinf;
            localstore[inf.first].first+=inf.second;
            localstore[inf.first].second.push_back(&inf.second);

        }
    }
    for(std::map<unsigned int,PerVertWeights >::iterator itvert=localstore.begin(); itvert!=localstore.end(); ++itvert) {
        PerVertWeights & weights=itvert->second;
        if(weights.first< 1e-4)
        {
            OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning the vertex " <<itvert->first << " seems to have 0 weight, skip normalize for this vertex" << std::endl;
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
        BoneInfluenceList &curvecinf=mapit->second;
        std::pair<float,unsigned int> & bonetotal=bone2total[curvecinf.getBoneName()];
        for(BoneInfluenceList::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end(); ++curinf) {
            IndexWeight& inf=*curinf;
            if( curvecinf.getBoneName().empty()) {
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
    //std::reverse_copy (totalinfset.begin(),totalinfset.end(),totalinfsorteddesc.begin());

    std::vector<BoneWeight>::iterator lastbone=totalinfsorteddesc.begin();
    while(totalinfsorteddesc.size()>numbonepermesh&&lastbone!=totalinfsorteddesc.end()) {

        //test bone removal==good if forall vert its not the unique influence
        bool goodforremoval=true;
        BoneInfluenceList &curvecinf=(*this)[lastbone->getBoneName()];
        for(BoneInfluenceList::iterator infit=curvecinf.begin(); infit!=curvecinf.end()&&goodforremoval; infit++) {

            uint index=infit->first;
            //check if index have other inf
            bool indok=false;
            for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end()&&!indok; ++mapit) {
                if(mapit->first!=lastbone->getBoneName()) {
                    for(BoneInfluenceList::iterator infit2=mapit->second.begin(); infit2!=mapit->second.end(); infit2++) {
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
            //lastbone=
            // totalinfsorteddesc.erase(lastbone);
            //std::advance(lastbone, 1);
            lastbone=totalinfsorteddesc.erase( lastbone );

            //todo: use a vector instead of a set not to loose iterator on erase
            //lastbone=totalinfsorteddesc.rbegin();
        } else lastbone++;

    }




    if( this->size()!=numbonepermesh) {
        OSG_WARN<<"targetbonepermesh not reached "<<numbonepermesh<<"!="<<this->size()<<std::endl;

    }




}
