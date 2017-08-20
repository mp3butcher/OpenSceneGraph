/*  -*-c++-*-
 *  Copyleft 2016 Valentin Julien
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

#include <osgAnimation/MorphTransformHardware>
#include <osgAnimation/MorphGeometry>
#include <osgAnimation/BoneMapVisitor>
#include <osg/TextureBuffer>
#include <sstream>

///texture unit reserved for morphtarget TBO
#define MORPHTEXTUREUNIT 7

using namespace osgAnimation;

MorphTransformHardware::MorphTransformHardware()
{
    _needInit = true;

}

MorphTransformHardware::MorphTransformHardware(const MorphTransformHardware& rth, const osg::CopyOp& copyop):
    MorphTransform(rth, copyop),
    _uniformTargetsWeight(rth._uniformTargetsWeight),
    _shader(rth._shader),
    _needInit(rth._needInit)
{
}

void MorphTransformHardware::setShader(osg::Shader* shader)
{
    _shader = shader;
}

bool MorphTransformHardware::init(MorphGeometry& geom)
{
    osg::Vec3Array* pos = dynamic_cast<osg::Vec3Array*>(geom.getVertexArray());
    osg::Vec3Array & vertexSource = *(geom.getVertexSource());
    osg::Vec3Array& normalSource = *(geom.getNormalSource());

    geom.setDataVariance(osg::Object::STATIC);
    ///check for correct morph configuration (blender exporter doesn't set sources so assume morphgeom arrays are sources:/)
    if(pos)
    {
        ///check if source is setted correctly
        if ( vertexSource.size() != pos->size())
        {
            vertexSource =*(static_cast<osg::Vec3Array*>( pos->clone(osg::CopyOp::DEEP_COPY_ARRAYS)));//osg::Vec3Array(pos->begin(),pos->end());
            pos->setDataVariance(osg::Object::STATIC);
        }

        osg::Vec3Array* normal = dynamic_cast<osg::Vec3Array*>(geom.getNormalArray());
        bool normalmorphable = geom.getMorphNormals() && normal;
        if(!normalmorphable) {
            OSG_WARN << "MorphTransformHardware::morph geometry "<<geom.getName()<<" without normal morphing not supported! "  << std::endl;
            return false;
        }
        if (normal && normalSource.size() != normal->size())
        {
            normalSource =*(static_cast<osg::Vec3Array*>( normal->clone(osg::CopyOp::DEEP_COPY_ARRAYS)));//osg::Vec3Array(normal->begin(),normal->end());
            normal->setDataVariance(osg::Object::STATIC);
        }
    }
    ///end check
    geom.setVertexArray(geom.getVertexSource());
    geom.setNormalArray(geom.getNormalSource(),osg::Array::BIND_PER_VERTEX);
    geom.setDataVariance(osg::Object::STATIC);

    //create one TBO for all morphtargets (pack vertex/normal)
    osg::Vec3Array *  morphTargets=new osg::Vec3Array ;
    MorphGeometry::MorphTargetList & morphlist=geom.getMorphTargetList();
    for(MorphGeometry::MorphTargetList::const_iterator curmorph=morphlist.begin(); curmorph!=morphlist.end(); ++curmorph) {
        const osg::Geometry * morphgeom=                curmorph->getGeometry() ;
        const osg::Vec3Array *varray=(osg::Vec3Array*)morphgeom->getVertexArray();
        const osg::Vec3Array *narray=(osg::Vec3Array*)morphgeom->getNormalArray();
        if(geom.getMethod()==MorphGeometry::RELATIVE){
            for(unsigned int i=0; i<geom.getVertexArray()->getNumElements(); ++i) {
                morphTargets->push_back( (*varray)[i]);
                morphTargets->push_back( (*narray)[i]);
            }
        }else{
            //convert to RELATIVE as it involve less math in the VS than NORMALIZED
            const osg::Vec3Array *ovarray=(osg::Vec3Array*)geom.getVertexArray();
            const osg::Vec3Array *onarray=(osg::Vec3Array*)geom.getNormalArray();
            for(unsigned int i=0; i<geom.getVertexArray()->getNumElements(); ++i) {
                morphTargets->push_back( (*varray)[i]- (*ovarray)[i] );
                morphTargets->push_back( (*narray)[i]- (*onarray)[i] );
            }
        }
    }
    osg::TextureBuffer * morphTargetsTBO=new osg::TextureBuffer();
    morphTargetsTBO->setBufferData(morphTargets);
    morphTargetsTBO->setInternalFormat( GL_RGB32F_ARB );

    //create TBO Texture handle
    osg::Uniform * morphTBOHandle=new osg::Uniform(osg::Uniform::SAMPLER_BUFFER,"morphTargets");
    morphTBOHandle->set(MORPHTEXTUREUNIT);

    //create dynamic uniform for morphtargets animation weights
    _uniformTargetsWeight=new osg::Uniform(osg::Uniform::FLOAT,"morphWeights",morphlist.size());


    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->setName("HardwareMorphing");
    if (!_shader.valid())
        _shader = osg::Shader::readShaderFile(osg::Shader::VERTEX,"morphing.vert");

    if (!_shader.valid()) {
        OSG_WARN << "MorphTransformHardware can't load VertexShader" << std::endl;
        return false;
    }

    // replace max morph weight by the value from uniform
    {
        std::string str = _shader->getShaderSource();
        std::string toreplace = std::string("MAX_MORPHWEIGHT");
        std::size_t start = str.find(toreplace);
        if (std::string::npos != start) {
            std::stringstream ss;
            ss << _uniformTargetsWeight->getNumElements();
            str.replace(start, toreplace.size(), ss.str());
            _shader->setShaderSource(str);
        }
        else
        {
            OSG_WARN << "MAX_MORPHWEIGHT not found in Shader! " << str << std::endl;
        }
        OSG_INFO << "Shader " << str << std::endl;
    }

    program->addShader(_shader.get());

    osg::ref_ptr<osg::StateSet> ss = geom.getOrCreateStateSet();
    ss->addUniform(_uniformTargetsWeight);
    ss->setTextureAttribute(MORPHTEXTUREUNIT,morphTargetsTBO);
    ss->addUniform( morphTBOHandle);
    ss->addUniform(new osg::Uniform("nbMorphVertex", geom.getVertexArray()->getNumElements()));
    if (_shader.valid())  if(!ss->getAttribute(osg::StateAttribute::PROGRAM))        ss->setAttributeAndModes(program.get());
    _needInit = false;
    return true;
}
void MorphTransformHardware::operator()(MorphGeometry& geom)
{
    if (_needInit)
        if (!init(geom))
            return;

    ///upload new morph weights each update via uniform
    int curimorph=0;
    MorphGeometry::MorphTargetList & morphlist=geom.getMorphTargetList();
    for(MorphGeometry::MorphTargetList::const_iterator curmorph=morphlist.begin(); curmorph!=morphlist.end(); ++curmorph)
        _uniformTargetsWeight->setElement(curimorph++, curmorph->getWeight());
    _uniformTargetsWeight->dirty();
}
