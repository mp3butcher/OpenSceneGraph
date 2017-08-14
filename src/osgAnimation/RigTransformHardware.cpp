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

osg::Vec4Array* RigTransformHardware::getVertexAttrib(int index)
{
    if (index >= (int)_boneWeightAttribArrays.size())
        return 0;
    return _boneWeightAttribArrays[index].get();
}

int RigTransformHardware::getNumVertexAttrib()
{
    return _boneWeightAttribArrays.size();
}

osg::Uniform* RigTransformHardware::getMatrixPaletteUniform()
{
    return _uniformMatrixPalette.get();
}


void RigTransformHardware::computeMatrixPaletteUniform(const osg::Matrix& transformFromSkeletonToGeometry, const osg::Matrix& invTransformFromSkeletonToGeometry)
{
    for (int i = 0; i < (int)_bonePalette.size(); i++)
    {
        osg::ref_ptr<Bone> bone = _bonePalette[i].get();
        const osg::Matrixf& invBindMatrix = bone->getInvBindMatrixInSkeletonSpace();
        const osg::Matrixf& boneMatrix = bone->getMatrixInSkeletonSpace();
        osg::Matrixf resultBoneMatrix = invBindMatrix * boneMatrix;
        osg::Matrixf result =  transformFromSkeletonToGeometry * resultBoneMatrix * invTransformFromSkeletonToGeometry;
         // result=result.identity();//DEBUG
        if (!_uniformMatrixPalette->setElement(i, result))
            OSG_WARN << "RigTransformHardware::computeUniformMatrixPalette can't set uniform at " << i << " elements" << std::endl;
    }_uniformMatrixPalette->dirty();
}


int RigTransformHardware::getNumBonesPerVertex() const { return _bonesPerVertex;}
int RigTransformHardware::getNumVertexes() const { return _nbVertexes;}

bool RigTransformHardware::createPalette(int nbVertexes, const BoneMap &boneMap, const VertexInfluenceSet::VertexIndexToBoneWeightMap& vertexIndexToBoneWeightMap)
{
    typedef std::map<std::string, int> BoneNameCountMap;
    _bonePalette.clear();
    BoneNameCountMap boneNameCountMap;
    // init vertex attribute data
    VertexIndexWeightList vertexIndexWeight;
    vertexIndexWeight.resize(nbVertexes);

    int maxBonePerVertex = 0;
    for (VertexInfluenceSet::VertexIndexToBoneWeightMap::const_iterator vit = vertexIndexToBoneWeightMap.begin(); vit != vertexIndexToBoneWeightMap.end(); ++vit)
    {
        int vertexIndex = vit->first;
        const VertexInfluenceSet::BoneWeightList& boneWeightList = vit->second;
        int bonesForThisVertex = 0;
        for (VertexInfluenceSet::BoneWeightList::const_iterator it = boneWeightList.begin(); it != boneWeightList.end(); ++it)
        {
            const VertexInfluenceSet::BoneWeight& bw = *it;
            if(fabs(bw.getWeight()) > 1e-4) // don't use bone with weight too small
            {
                if (( boneNameCountMap.find(bw.getBoneName())) != boneNameCountMap.end())
                {
                    boneNameCountMap[bw.getBoneName()]++;
                    bonesForThisVertex++; // count max number of bones per vertexes
                    vertexIndexWeight[vertexIndex].push_back(IndexWeightEntry(_boneNameToPalette[bw.getBoneName()],bw.getWeight()));
                }
                else
                {
                    if (boneMap.find(bw.getBoneName()) == boneMap.end())
                    {
                        OSG_INFO << "RigTransformHardware::createPalette can't find bone " << bw.getBoneName() << " skip this influence" << std::endl;
                        continue;
                    }
                    boneNameCountMap[bw.getBoneName()] = 1; // for stats
                    bonesForThisVertex++;
                    _bonePalette.push_back((boneMap.find(bw.getBoneName())->second));
                    _boneNameToPalette[bw.getBoneName()] = _bonePalette.size()-1;
                    vertexIndexWeight[vertexIndex].push_back(IndexWeightEntry(_boneNameToPalette[bw.getBoneName()],bw.getWeight()));
                }
            }
            else
            {
                 OSG_INFO << "RigTransformHardware::createPalette Bone " << bw.getBoneName() << " has a weight " << bw.getWeight() << " for vertex " << vertexIndex << " this bone will not be in the palette" << std::endl;
            }
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


    for (int i = 0 ; i < (int)vertexIndexWeight.size(); i++)
        vertexIndexWeight[i].resize(maxBonePerVertex);

    _nbVertexes = nbVertexes;
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
    int nbArray = static_cast<int>(ceilf(getNumBonesPerVertex() * 0.5));
    if (!nbArray)
        return ;

    _boneWeightAttribArrays.resize(nbArray);
    for (int i = 0; i < nbArray; i++)
    {
        osg::ref_ptr<osg::Vec4Array> array = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
        _boneWeightAttribArrays[i] = array;
        int nbVertexes = getNumVertexes();
        array->resize(nbVertexes);
        for (int j = 0; j < nbVertexes; j++)
        {
            for (int b = 0; b < 2; b++)
            {
                // the granularity is 2 so if we have only one bone
                // it's convenient to init the second with a weight 0
                int boneIndexInList = i*2 + b;
                int boneIndexInVec4 = b*2;
                (*array)[j][0 + boneIndexInVec4] = 0;
                (*array)[j][1 + boneIndexInVec4] = 0;
                if (boneIndexInList < getNumBonesPerVertex())
                {
                    float boneIndex = static_cast<float>(_vertexIndexMatrixWeightList[j][boneIndexInList].getIndex());
                    float boneWeight = _vertexIndexMatrixWeightList[j][boneIndexInList].getWeight();
                    // fill the vec4
                    (*array)[j][0 + boneIndexInVec4] = boneIndex;
                    (*array)[j][1 + boneIndexInVec4] = boneWeight;
                }

            }
        }


    } ///check arrays


    for (int j = 0; j < getNumVertexes(); j++){
       float sum=0;
       for (int i = 0; i < nbArray; i++){
       sum+=(*_boneWeightAttribArrays[i].get())[j][1]+(*_boneWeightAttribArrays[i].get())[j][3];
        }
    if(sum<0.9){OSG_WARN<<"index "<<j<<" have sumweight =="<<sum<<std::endl;
        for (int i = 0; i < nbArray; i++){
        OSG_WARN<<(*_boneWeightAttribArrays[i].get())[j][0]<<":"<<(*_boneWeightAttribArrays[i].get())[j][1]<<","<<(*_boneWeightAttribArrays[i].get())[j][2]<<":"<<(*_boneWeightAttribArrays[i].get())[j][3]<<" ";
         }  OSG_WARN<<std::endl;
    }
    }
    return ;
}


osg::Uniform* RigTransformHardware::createVertexUniform()
{
    osg::Uniform* uniform = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "matrixPalette", _bonePalette.size());
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

    if (!createPalette(positionSrc->size(),bm, geom.getVertexInfluenceSet().getVertexToBoneList()))
        return false;

    osg::ref_ptr<osg::Program> program ;
    osg::ref_ptr<osg::Shader> vertexshader;
    osg::ref_ptr<osg::StateSet> ss = geom.getOrCreateStateSet();
    //grab geom source program and vertex shader if _shader is not setted
    if(!_shader.valid() && (program = (osg::Program*)ss->getAttribute(osg::StateAttribute::PROGRAM)))
    {
        for(int i=0;i<program->getNumShaders();++i)
            if(program->getShader(i)->getType()==osg::Shader::VERTEX)
                vertexshader=program->getShader(i);
    }else {
        program = new osg::Program;
        program->setName("HardwareSkinning");
    }
    //set default source if _shader is not user setted
    if (!vertexshader.valid())
        if (!_shader.valid())
            vertexshader = osg::Shader::readShaderFile(osg::Shader::VERTEX,"skinning.vert");
        else vertexshader=_shader;

    if (!vertexshader.valid()) {
        OSG_WARN << "RigTransformHardware can't load VertexShader" << std::endl;
        return false;
    }

    // replace max matrix by the value from uniform
    {
    std::string str = vertexshader->getShaderSource();
    std::string toreplace = std::string("MAX_MATRIX");
    std::size_t start = str.find(toreplace);
    if (std::string::npos != start) {
        std::stringstream ss;
        ss << getMatrixPaletteUniform()->getNumElements();
        str.replace(start, toreplace.size(), ss.str());
        vertexshader->setShaderSource(str);
    }
    else
    {
        OSG_WARN << "MAX_MATRIX not found in Shader! " << str << std::endl;
    }
    OSG_INFO << "Shader " << str << std::endl;
    }

    int attribIndex = 11;
    int nbAttribs = getNumVertexAttrib();
    for (int i = 0; i < nbAttribs; i++)
    {
        std::stringstream ss;
        ss << "boneWeight" << i;
        program->addBindAttribLocation(ss.str(), attribIndex + i);
        geom.setVertexAttribArray(attribIndex + i, getVertexAttrib(i));
        OSG_INFO << "set vertex attrib " << ss.str() << std::endl;
    }
    program->addShader(vertexshader.get());

    ss->addUniform(getMatrixPaletteUniform());
    ss->addUniform(new osg::Uniform("nbBonesPerVertex", getNumBonesPerVertex()));
    if(!ss->getAttribute(osg::StateAttribute::PROGRAM))ss->setAttributeAndModes(program.get());

    _needInit = false;
    return true;
}
void RigTransformHardware::operator()(RigGeometry& geom)
{
    if (_needInit)
        if (!init(geom))
            return;
    computeMatrixPaletteUniform(geom.getMatrixFromSkeletonToGeometry(), geom.getInvMatrixFromSkeletonToGeometry());
}
