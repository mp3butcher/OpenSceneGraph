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

void VertexInfluenceSet::addVertexInfluence(const VertexInfluence& v) { _bone2Vertexes.push_back(v); }
const VertexInfluenceSet::VertexIndexToBoneWeightMap& VertexInfluenceSet::getVertexToBoneList() const { return _vertex2Bones;}
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
void VertexInfluenceSet::buildVertex2BoneList()
{
    _vertex2Bones.clear();
    typedef std::set<BoneWeight ,invweight_ordered2>  BoneWeightOrdered;
      std::map<int,BoneWeightOrdered >tempVec2Bones;
      for (BoneToVertexList::const_iterator it = _bone2Vertexes.begin(); it != _bone2Vertexes.end(); ++it)
      {
          const VertexInfluence& vi = (*it);
          int size = vi.size();
          for (int i = 0; i < size; i++)
          {
              VertexIndexWeight viw = vi[i];
              int index = viw.first;
              float weight = viw.second;
              if (vi.getName().empty()){
                  OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning vertex " << index << " is not assigned to a bone" << std::endl;
              }
              int size=tempVec2Bones[index].size();
              tempVec2Bones[index].insert(BoneWeight(vi.getName(), weight));;
              if(++size!=tempVec2Bones[index].size())
                  OSG_WARN<<"WTF"<<std::endl;
          }
      }

      //copy tempVec2Bones to _vertex2Bones
      for(std::map<int,BoneWeightOrdered>::iterator iti = tempVec2Bones.begin(); iti != tempVec2Bones.end(); ++iti){
          BoneWeightList& destbonelist=_vertex2Bones[iti->first];
          for(BoneWeightOrdered::iterator itbw = iti->second.begin();itbw != iti->second.end(); ++itbw)
              destbonelist.push_back(*itbw);
            if( destbonelist.size()!=iti->second.size())
                OSG_WARN<<"WTF"<<std::endl;
  }
    // normalize weight per vertex
    for (VertexIndexToBoneWeightMap::iterator it = _vertex2Bones.begin(); it != _vertex2Bones.end(); ++it)
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

void VertexInfluenceSet::buildUniqVertexSetToBoneSetList()
{
    _uniqVertexSetToBoneSet.clear();

    typedef std::map<BoneWeightList,UniqVertexSetToBoneSet, SortByBoneWeightList> UnifyBoneGroup;
    UnifyBoneGroup unifyBuffer;

    for (VertexIndexToBoneWeightMap::iterator it = _vertex2Bones.begin(); it != _vertex2Bones.end(); ++it)
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
    }
        ///DEBUG
        /// rebuild _vertex2bone
 /*    _vertex2Bones.clear();
    for (UnifyBoneGroup::iterator it = unifyBuffer.begin(); it != unifyBuffer.end(); ++it)
    {
        const BoneWeightList& bwl=it->first;
        UniqVertexSetToBoneSet &vsbs=it->second;
        for(std::vector<unsigned int>::iterator vertit=vsbs.getVertexes().begin();vertit!=vsbs.getVertexes().end();++vertit){
            _vertex2Bones[*vertit]=vsbs.getBones();
        }


    }*/
}

struct invweight_ordered
{
    inline bool operator() (const VertexInfluenceSet::BoneWeight& bw1, const VertexInfluenceSet::BoneWeight& bw2)
    {
        if (bw1.getWeight() > bw2.getWeight())return true;
        if (bw1.getWeight() < bw2.getWeight())return false;
        return(bw1.getBoneName()<bw2.getBoneName());
    }
};
///cull weakest influences in order to fit targetted numbonepervertex
void VertexInfluenceMap::cullBoneInfluenceCountPerVertex(unsigned int numbonepervertex,float minweight, bool renormalize){

    typedef std::set<VertexInfluenceSet::BoneWeight,invweight_ordered >  BoneWeightOrdered;
    std::map<int,BoneWeightOrdered > tempVec2Bones;
    for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end(); ++mapit) {
        VertexInfluence &curvecinf=mapit->second;
        for(VertexInfluence::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end();++curinf) {
             VertexIndexWeight& inf=*curinf;
             if( curvecinf.getName().empty()){
                 OSG_WARN << "VertexInfluenceSet::buildVertex2BoneList warning vertex " << inf.first << " is not assigned to a bone" << std::endl;
             }
             else if(inf.second>minweight)tempVec2Bones[inf.first].insert(VertexInfluenceSet::BoneWeight(curvecinf.getName(), inf.second));
        }
    }
    this->clear();
    for( std::map<int,BoneWeightOrdered >::iterator mapit=tempVec2Bones.begin();mapit!=tempVec2Bones.end();++mapit){
        BoneWeightOrdered& bwset=mapit->second;
        unsigned int newsize=numbonepervertex<bwset.size()?numbonepervertex:bwset.size();
        float sum=0;
        while(bwset.size()>newsize)bwset.erase(*bwset.rbegin());
       // if(renormalize)
            for(BoneWeightOrdered::iterator bwit=bwset.begin();bwit!=bwset.end();++bwit)
                sum+=bwit->getWeight();
            sum=1.0f/sum;
       // if(!renormalize||sum>1e-4)
            for(BoneWeightOrdered::iterator bwit=bwset.begin();bwit!=bwset.end();++bwit){
                VertexInfluence & inf= (*this)[bwit->getBoneName()];
                inf.setName(bwit->getBoneName());
                inf.push_back(VertexIndexWeight(mapit->first,renormalize? bwit->getWeight()*sum: bwit->getWeight()));
            }
    }
}

void VertexInfluenceMap::cullBoneCountPerMesh(unsigned int numbonepermesh){
    if (this->size()<=numbonepermesh)
        return;

    typedef std::set<VertexInfluenceSet::BoneWeight, invweight_ordered>  BoneWeightOrdered;
    typedef std::map<std::string,std::pair<float,unsigned int> >  BoneName2TotalInf;
    BoneName2TotalInf bone2total;
    for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end(); ++mapit) {
        VertexInfluence &curvecinf=mapit->second;
        std::pair<float,unsigned int> & bonetotal=bone2total[curvecinf.getName()];
        for(VertexInfluence::iterator curinf=curvecinf.begin(); curinf!=curvecinf.end();++curinf) {
             VertexIndexWeight& inf=*curinf;
             if( curvecinf.getName().empty()){
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
    for(BoneName2TotalInf::const_iterator mapit=bone2total.begin();mapit!=bone2total.end();++mapit){
        totalinfset.insert(VertexInfluenceSet::BoneWeight(mapit->first,mapit->second.first/(float)mapit->second.second));
    }
    std::vector<VertexInfluenceSet::BoneWeight> totalinfsorteddesc;
    for(BoneWeightOrdered::reverse_iterator it=totalinfset.rbegin();it!=totalinfset.rend();it++)
    totalinfsorteddesc.push_back(VertexInfluenceSet::BoneWeight(it->getBoneName(),it->getWeight()));
    //std::reverse_copy (totalinfset.begin(),totalinfset.end(),totalinfsorteddesc.begin());

    std::vector<VertexInfluenceSet::BoneWeight>::iterator lastbone=totalinfsorteddesc.begin();
    while(totalinfsorteddesc.size()>numbonepermesh&&lastbone!=totalinfsorteddesc.end()){

        //test bone removal==good if forall vert its not the unique influence
        bool goodforremoval=true;
        VertexInfluence &curvecinf=(*this)[lastbone->getBoneName()];
        for(VertexInfluence::iterator infit=curvecinf.begin();infit!=curvecinf.end()&&goodforremoval;infit++){

            uint index=infit->first;
            //check if index have other inf
            bool indok=false;
            for(VertexInfluenceMap::iterator mapit=this->begin(); mapit!=this->end()&&!indok; ++mapit) {
                if(mapit->first!=lastbone->getBoneName()){
                    for(VertexInfluence::iterator infit2=mapit->second.begin();infit2!=mapit->second.end();infit2++){
                        if(infit2->first==index){
                            //other inf found
                            indok=true;break;
                        }
                    }
                }
            }
            if(!indok)goodforremoval=false;

         }
        if(goodforremoval){
            OSG_WARN<<"droping bone"<<lastbone->getBoneName()<<" with average influence of "<<lastbone->getWeight()<<std::endl;
            this->erase( this->find(lastbone->getBoneName()));
        //lastbone=
               // totalinfsorteddesc.erase(lastbone);
                //std::advance(lastbone, 1);
                lastbone=totalinfsorteddesc.erase( lastbone );

                //todo: use a vector instead of a set not to loose iterator on erase
                //lastbone=totalinfsorteddesc.rbegin();
        }else lastbone++;

   }




if( this->size()!=numbonepermesh){
    OSG_WARN<<"targetbonepermesh not reached "<<numbonepermesh<<"!="<<this->size()<<std::endl;

}




}
