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

#include <osgAnimation/RigTransformHardware>
#include <osgAnimation/RigGeometry>
#include <osgAnimation/BoneMapVisitor>
#include <sstream>

using namespace osgAnimation;


RigTransformHardware::RigTransformHardware()
{
    _needInit = true;
    _bonesPerVertex = 0;
    _nbVertexes = 0;
}

RigTransformHardware::RigTransformHardware(const RigTransformHardware& rth, const osg::CopyOp& copyop):
    RigTransform(rth, copyop),
    _bonesPerVertex(rth._bonesPerVertex),
    _nbVertexes(rth._nbVertexes),
    _bonePalette(rth._bonePalette),
    _boneNameToPalette(rth._boneNameToPalette),
    _boneWeightAttribArrays(rth._boneWeightAttribArrays),
    _uniformMatrixPalette(rth._uniformMatrixPalette),
    _shader(rth._shader),
    _needInit(rth._needInit)
{
}

osg::Vec4Array* RigTransformHardware::getVertexAttrib(unsigned int index)
{
    if (index >=  _boneWeightAttribArrays.size())
        return 0;
    return _boneWeightAttribArrays[index].get();
}

unsigned int RigTransformHardware::getNumVertexAttrib()
{
    return _boneWeightAttribArrays.size();
}

osg::Uniform* RigTransformHardware::getMatrixPaletteUniform()
{
    return _uniformMatrixPalette.get();
}


void RigTransformHardware::computeMatrixPaletteUniform(const osg::Matrix& transformFromSkeletonToGeometry, const osg::Matrix& invTransformFromSkeletonToGeometry)
{
    if(_uniformMatrixPalette->getNumElements()!=_bonePalette.size()){
        OSG_WARN << "WTFelements" << std::endl;

    }
    for (unsigned int i = 0; i <  _bonePalette.size(); i++)
    {
        osg::ref_ptr<Bone> bone = _bonePalette[i].get();
        const osg::Matrixf& invBindMatrix = bone->getInvBindMatrixInSkeletonSpace();
        const osg::Matrixf& boneMatrix = bone->getMatrixInSkeletonSpace();
        osg::Matrixf resultBoneMatrix = invBindMatrix * boneMatrix;
        osg::Matrixf result =  transformFromSkeletonToGeometry * resultBoneMatrix * invTransformFromSkeletonToGeometry;
        if (!_uniformMatrixPalette->setElement(i, result))
            OSG_WARN << "RigTransformHardware::computeUniformMatrixPalette can't set uniform at " << i << " elements" << std::endl;
    }_uniformMatrixPalette->dirty();_uniformMatrixPalette->getFloatArray()->dirty();
}


unsigned int RigTransformHardware::getNumBonesPerVertex() const { return _bonesPerVertex;}
unsigned int RigTransformHardware::getNumVertexes() const { return _nbVertexes;}

bool RigTransformHardware::createPalette(unsigned int nbVertexes, const BoneMap &boneMap, const VertexInfluenceSet::VertIDToBoneWeightList& vertexIndexToBoneWeightMap)
{
    _nbVertexes = nbVertexes;
    typedef std::map<std::string, int> BoneNameCountMap;
    _bonePalette.clear();
    _boneNameToPalette.clear();
    BoneNameCountMap boneNameCountMap;
    // init vertex attribute data
    VertexIndexWeightList vertexIndexWeight;
    vertexIndexWeight.resize(nbVertexes);

    unsigned int maxBonePerVertex = 0;
    if(vertexIndexToBoneWeightMap.size()!=nbVertexes) {
        OSG_WARN << "RigTransformHardware::some vertex has no transform  " <<vertexIndexToBoneWeightMap.size()<<"!="<< nbVertexes  << std::endl;
        return false;
    }
    unsigned int vertexID=0;
    for (VertexInfluenceSet::VertIDToBoneWeightList::const_iterator vit = vertexIndexToBoneWeightMap.begin(); vit != vertexIndexToBoneWeightMap.end(); ++vit,++vertexID)
    {
        //unsigned int vertexIndex = vit->first;
        const VertexInfluenceSet::BoneWeightList& boneWeightList = *vit;
        unsigned int bonesForThisVertex = 0;
        for (VertexInfluenceSet::BoneWeightList::const_iterator it = boneWeightList.begin(); it != boneWeightList.end(); ++it)
        {
            const VertexInfluenceSet::BoneWeight& bw = *it;
            if(fabs(bw.getWeight()) > 1e-4) // don't use bone with weight too small
            {
                if (( _boneNameToPalette.find(bw.getBoneName())) != _boneNameToPalette.end())
                {
                    boneNameCountMap[bw.getBoneName()]++;
                    bonesForThisVertex++; // count max number of bones per vertexes
                    vertexIndexWeight[vertexID].push_back(IndexWeightEntry(_boneNameToPalette[bw.getBoneName()],bw.getWeight()));
                }
                else
                {
                    BoneMap::const_iterator bonebyname;
                    if ((bonebyname=boneMap.find(bw.getBoneName())) == boneMap.end())
                    {
                        OSG_WARN << "RigTransformHardware::createPalette can't find bone " << bw.getBoneName() << "in skeleton bonemap:  skip this influence" << std::endl;
                        continue;
                    }
                    boneNameCountMap[bw.getBoneName()] = 1; // for stats
                    bonesForThisVertex++;
                    if( _boneNameToPalette.find(bw.getBoneName())!=_boneNameToPalette.end()){
                        OSG_WARN<<"WTFfdks"<<std::endl;
                    }

                    _boneNameToPalette[bw.getBoneName()] = _bonePalette.size() ;
                     vertexIndexWeight[vertexID].push_back(IndexWeightEntry(_bonePalette.size(),bw.getWeight()));

                    _bonePalette.push_back(bonebyname->second);
                  }
            }
            else
            {
                 OSG_WARN << "RigTransformHardware::createPalette Bone " << bw.getBoneName() << " has a weight " << bw.getWeight() << " for vertex " << vertexID << " this bone will not be in the palette" << std::endl;
            }
        }
        if(bonesForThisVertex==0) {
            OSG_WARN << "RigTransformHardware::no transform for vertex  " << vertexID << " this will induce a bug in vertex shader" << std::endl;
        }
        maxBonePerVertex = osg::maximum(maxBonePerVertex, bonesForThisVertex);
    }
    OSG_INFO << "RigTransformHardware::createPalette maximum number of bone per vertex is " << maxBonePerVertex << std::endl;
    OSG_INFO << "RigTransformHardware::createPalette matrix palette has " << boneNameCountMap.size() << " entries" << std::endl;

    for (BoneNameCountMap::iterator it = boneNameCountMap.begin(); it != boneNameCountMap.end(); ++it)
    {
        OSG_INFO << "RigTransformHardware::createPalette Bone " << it->first << " is used " << it->second << " times" << std::endl;
    }

    OSG_INFO << "RigTransformHardware::createPalette will use " << boneNameCountMap.size() * 4 << " uniforms" << std::endl;


   /* for (unsigned int i = 0 ; i < vertexIndexWeight.size(); i++)
        vertexIndexWeight[i].resize(maxBonePerVertex);
*/
    _bonesPerVertex = maxBonePerVertex;
    _uniformMatrixPalette = createVertexUniform();
     createVertexAttribList(vertexIndexWeight);
    return true;
}


//
// create vertex attribute by 2 bones
// vec4(boneIndex0, weight0, boneIndex1, weight1)
// if more bones are needed then other attributes are created
// vec4(boneIndex2, weight2, boneIndex3, weight3)
// the idea is to use this format to have a granularity smaller
// than the 4 bones using two vertex attributes
//
void RigTransformHardware::createVertexAttribList(VertexIndexWeightList&_vertexIndexMatrixWeightList)
{
    //BoneWeightAttribList arrayList;
    unsigned int nbArray = static_cast<unsigned int>(ceilf( ((float)getNumBonesPerVertex()) * 0.5f));
    if (!nbArray)
        return ;

    _boneWeightAttribArrays.resize(nbArray);

#if 0
    for (unsigned int i = 0; i < nbArray; i++)
    {
        osg::ref_ptr<osg::Vec4Array> array = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
        _boneWeightAttribArrays[i] = array;
        for (unsigned int j = 0; j < nbVertexes; j++)
        array->push_back(osg::Vec4(10000,0,100000,0));
        array->dirty();
    }

    //VertexIndexWeightList

    for (unsigned int j = 0; j < nbVertexes; j++){
          unsigned int iwid=0;
    for(   std::vector<IndexWeightEntry>::iterator iweit =_vertexIndexMatrixWeightList[j].begin();
           iweit!=_vertexIndexMatrixWeightList[j].end(); iweit++,iwid++){

        int arrayid=iwid/2;
        int idinarray=2*(iwid&1);
        if(iweit->getIndex()>=_bonePalette.size()){

            OSG_WARN<<"index "<<iweit->getIndex()<<" >= bonepalletsize "<< _bonePalette.size()<<std::endl;

        }
        (*_boneWeightAttribArrays[arrayid])[j][idinarray]=(iweit->getIndex());
        (*_boneWeightAttribArrays[arrayid])[j][idinarray+1]=iweit->getWeight();

    }
}
 #else
    for (unsigned int i = 0; i < nbArray; i++)
    {
        osg::ref_ptr<osg::Vec4Array> array = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
        _boneWeightAttribArrays[i] = array;
        array->resize( _nbVertexes);
        for (unsigned int j = 0; j < _nbVertexes; j++)
        {

            for (unsigned int b = 0; b < 2; b++)
            {
                // the granularity is 2 so if we have only one bone
                // it's convenient to init the second with a weight 0
                unsigned int boneIndexInList = i*2 + b;
                unsigned int boneIndexInVec4 = b*2;
                (*array)[j][0 + boneIndexInVec4] = 0;
                (*array)[j][1 + boneIndexInVec4] = 0;
               // if (boneIndexInList < getNumBonesPerVertex())
                {
                    float boneIndex = static_cast<float>(_vertexIndexMatrixWeightList[j][boneIndexInList].getIndex());
                    float boneWeight = _vertexIndexMatrixWeightList[j][boneIndexInList].getWeight();
                    // fill the vec4
                    (*array)[j][0 + boneIndexInVec4] = boneIndex;
                    (*array)[j][1 + boneIndexInVec4] = boneWeight;
                }
            }
        }


    }
  #endif
      ///check arrays*/

/*
    for (unsigned int j = 0; j < getNumVertexes(); j++){
       float sum=0;
       if((*_boneWeightAttribArrays[0].get())[j][1]==0){

           OSG_WARN<<"index "<<j<<" have weight ==0"<<std::endl;
       }
       for (unsigned int i = 0; i < nbArray; i++){
       sum+=(*_boneWeightAttribArrays[i].get())[j][1]+(*_boneWeightAttribArrays[i].get())[j][3];
       //chack index
       if((*_boneWeightAttribArrays[i].get())[j][0]>=_bonePalette.size() && (*_boneWeightAttribArrays[i].get())[j][1]!=0)
           OSG_WARN<<"index "<<(*_boneWeightAttribArrays[i].get())[j][0]<<" invamlid"<<std::endl;
       if((*_boneWeightAttribArrays[i].get())[j][2]>=_bonePalette.size()&& (*_boneWeightAttribArrays[i].get())[j][3]!=0)
           OSG_WARN<<"index "<<(*_boneWeightAttribArrays[i].get())[j][2]<<" invamlid"<<std::endl;

        }
    if(sum<0.9999){OSG_WARN<<"index "<<j<<" have sumweight =="<<sum<<std::endl;
        for (unsigned int i = 0; i < nbArray; i++){
        OSG_WARN<<(*_boneWeightAttribArrays[i].get())[j][0]<<":"<<(*_boneWeightAttribArrays[i].get())[j][1]<<","<<(*_boneWeightAttribArrays[i].get())[j][2]<<":"<<(*_boneWeightAttribArrays[i].get())[j][3]<<" ";
         }  OSG_WARN<<std::endl;
    }
    }*/
    return ;
}


osg::Uniform* RigTransformHardware::createVertexUniform()
{
    osg::Uniform* uniform = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "matrixPalette", _bonePalette.size());
    //debug ensure identity
    osg::Matrix m;m=m.identity();
    for(unsigned int i=0;i<_bonePalette.size();++i)
        uniform->setElement(i,m);
    return uniform;
}


void RigTransformHardware::setShader(osg::Shader* shader)
{
    _shader = shader;
}

bool RigTransformHardware::init(RigGeometry& geom)
{
    if (!geom.getSkeleton())
    {
        OSG_WARN << "RigTransformHardware no skeleton set in geometry " << geom.getName() << std::endl;
        return false;
    }
    BoneMapVisitor mapVisitor;
    geom.getSkeleton()->accept(mapVisitor);
    BoneMap bm = mapVisitor.getBoneMap();

    osg::Geometry& source = *geom.getSourceGeometry();
    osg::Vec3Array* positionSrc = dynamic_cast<osg::Vec3Array*>(source.getVertexArray());
    if (!positionSrc)
    {
        OSG_WARN << "RigTransformHardware no vertex array in the geometry " << geom.getName() << std::endl;
        return false;
    }

    // copy shallow from source geometry to rig
    geom.copyFrom(source);
//  geom.setStateSet((osg::StateSet *) osg::CopyOp()(source.getOrCreateStateSet()));
geom.buildVertexInfluenceSet();
    if (!createPalette(positionSrc->size(),bm,geom.getVertexInfluenceSet().getVertexToBoneList()))
        return false;

    osg::ref_ptr<osg::Program> program ;
    osg::ref_ptr<osg::Shader> vertexshader;
    osg::ref_ptr<osg::StateSet> stateset = geom.getOrCreateStateSet();

    //grab geom source program and vertex shader if _shader is not setted
    if(!_shader.valid() && (program = (osg::Program*)stateset->getAttribute(osg::StateAttribute::PROGRAM)))
    {
        ///    ensure stateset is not shared
        if(stateset->getParents().size()>2){
            //geom.setStateSet(          );
        }

        for(unsigned int i=0;i<program->getNumShaders();++i)
            if(program->getShader(i)->getType()==osg::Shader::VERTEX){
                vertexshader=program->getShader(i);
                program->removeShader(vertexshader);

            }
       // program = new osg::Program;
    }else {
        program = new osg::Program;
        program->setName("HardwareSkinning");
    }
    //set default source if _shader is not user setted
    if (!vertexshader.valid()){
        if (!_shader.valid())
            vertexshader = osg::Shader::readShaderFile(osg::Shader::VERTEX,"skinning.vert");
        else vertexshader=_shader;
    }


    if (!vertexshader.valid()) {
        OSG_WARN << "RigTransformHardware can't load VertexShader" << std::endl;
        return false;
    }

    // replace max matrix by the value from uniform
    {
    std::string str = vertexshader->getShaderSource();
    std::string toreplace = std::string("MAX_MATRIX");
    std::size_t start = str.find(toreplace);
    if (std::string::npos == start){
        ///perhaps remanance from previous init (if saved after init) so reload shader
       /* OSG_WARN << str << std::endl;
        vertexshader = osg::Shader::readShaderFile(osg::Shader::VERTEX,"skinning.vert");
        if (!vertexshader.valid()) {
            OSG_WARN << "RigTransformHardware can't load VertexShader" << std::endl;
            return false;
        }
        str = vertexshader->getShaderSource();
        start = str.find(toreplace);
       // _uniformMatrixPalette=stateset->getUniform("matrixPalette");
        unsigned int attribIndex = 11;
        unsigned int nbAttribs = getNumVertexAttrib();
        if(nbAttribs==0)
            OSG_WARN << "nbAttribs== " << nbAttribs << std::endl;
        for (unsigned int i = 0; i < nbAttribs; i++)
        {
            std::stringstream ss;
            ss << "boneWeight" << i;
            program->addBindAttribLocation(ss.str(), attribIndex + i);

            if(getVertexAttrib(i)->getNumElements()!=_nbVertexes)
                OSG_WARN << "getVertexAttrib== " << getVertexAttrib(i)->getNumElements() << std::endl;
            geom.setVertexAttribArray(attribIndex + i, getVertexAttrib(i));
            OSG_INFO << "set vertex attrib " << ss.str() << std::endl;
        }
            _needInit = false;
        return true;*/
    }
    if (std::string::npos != start) {
        std::stringstream ss;
        ss << getMatrixPaletteUniform()->getNumElements();
        str.replace(start, toreplace.size(), ss.str());
        vertexshader->setShaderSource(str);
    }
    else
    {
        OSG_INFO<< "MAX_MATRIX not found in Shader! " << str << std::endl;
    }
    OSG_INFO << "Shader " << str << std::endl;
    }

    unsigned int attribIndex = 11;
    unsigned int nbAttribs = getNumVertexAttrib();
    if(nbAttribs==0)
        OSG_WARN << "nbAttribs== " << nbAttribs << std::endl;
    for (unsigned int i = 0; i < nbAttribs; i++)
    {
        std::stringstream ss;
        ss << "boneWeight" << i;
        program->addBindAttribLocation(ss.str(), attribIndex + i);

        if(getVertexAttrib(i)->getNumElements()!=_nbVertexes)
            OSG_WARN << "getVertexAttrib== " << getVertexAttrib(i)->getNumElements() << std::endl;
        geom.setVertexAttribArray(attribIndex + i, getVertexAttrib(i));
        OSG_INFO << "set vertex attrib " << ss.str() << std::endl;
    }


    program->addShader(vertexshader.get());
    stateset->removeUniform("nbBonesPerVertex");
    stateset->removeUniform("matrixPalette");
    stateset->addUniform(getMatrixPaletteUniform());

osg::Uniform * bonepervert=new osg::Uniform(osg::Uniform::UNSIGNED_INT,"nbBonesPerVertex");
bonepervert->set(_bonesPerVertex);
    stateset->addUniform(bonepervert);
    stateset->removeAttribute(osg::StateAttribute::PROGRAM);
     if(!stateset->getAttribute(osg::StateAttribute::PROGRAM))
        stateset->setAttributeAndModes(program.get());

    _needInit = false;
    return true;
}
/*void RigTransformHardware::prepareData(RigGeometry&rig){


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
}*/

void RigTransformHardware::operator()(RigGeometry& geom)
{
    if (_needInit)
        if (!init(geom))
            return;
    computeMatrixPaletteUniform(geom.getMatrixFromSkeletonToGeometry(), geom.getInvMatrixFromSkeletonToGeometry());
}
