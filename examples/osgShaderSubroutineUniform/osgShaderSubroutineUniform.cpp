/* OpenSceneGraph example, osgtransformfeedback
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

/* file:        examples/osgtransformfeedback/osgtransformfeedback.cpp
* author:      Julien Valentin 2016-06-18
* copyright:   (C) 2016
* license:     OpenSceneGraph Public License (OSGPL)
*
* A demo of OpenGL 4 Shader Subroutine Uniform
*
*/

#include <osg/GLExtensions>
#include <osg/Notify>
#include <osg/ref_ptr>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Program>
#include <osg/Shader>
#include <osg/BlendFunc>

#include <osg/Uniform>
#include <osgViewer/Viewer>
#include <osg/SubroutineUniform>

#include <osgDB/WriteFile>


///////////////////////////////////////////////////////////////////////////

class ShaderSubroutineSwaper: public osg::StateSet::Callback
{
public:
    ShaderSubroutineSwaper(osg::SubroutineUniform * su):
        _subu(su), _round(0)
    {}

    virtual void operator() (osg::StateSet*, osg::NodeVisitor*)
    {
        _subu->removeSubroutineName(_subu->getSubroutineName(0));

        switch(_round++)
        {
        case 0:
            _subu->addSubroutineName("ColorRed");break;
        case 1:
            _subu->addSubroutineName("ColorGreen");break;
        case 2:
            _subu->addSubroutineName("ColorBlue");break;
        }
        if(_round>2)_round=0;
    }

private:
    osg::SubroutineUniform *_subu;
    unsigned _round;
};

///////////////////////////////////////////////////////////////////////////

static const char* fragSource =
{
    "#version 400\n"
    "subroutine vec4 color_t();\n"
    "subroutine uniform color_t Color;\n"

    "subroutine(color_t)\n"
    "vec4 ColorRed()\n"
    "{\n"
    "  return vec4(1, 0, 0, 1);\n"
    "}\n"
    "subroutine(color_t)\n"
    "vec4 ColorBlue()\n"
    "{\n"
    "  return vec4(0, 0, 1, 1);\n"
    "}\n"
    "subroutine(color_t)\n"
    "vec4 ColorGreen()\n"
    "{\n"
    "  return vec4(0, 1, 0, 1);\n"
    "}\n"

    "void main(void)\n"
    "{\n"
    "    gl_FragColor =  Color();\n"
    "}\n"
};

///////////////////////////////////////////////////////////////////////////

int main( int , char** )
{
    osg::Geode* root( new osg::Geode );
    osg::ref_ptr<osg::Geometry >  geom= new osg::Geometry();// osg::createTexturedQuadGeometry(osg::Vec3f(-0.5,-0.5,-0.5),osg::Vec3f(1,0,0),osg::Vec3f(0,0,1));

    osg::Vec3Array * vary=new osg::Vec3Array();
    vary->push_back(osg::Vec3(0,0,0));
    vary->push_back(osg::Vec3(1,0,0));
    vary->push_back(osg::Vec3(1,1,0));
    vary->push_back(osg::Vec3(0,1,0));

    vary->push_back(osg::Vec3(0,0,1));
    vary->push_back(osg::Vec3(1,0,1));
    vary->push_back(osg::Vec3(1,1,1));
    vary->push_back(osg::Vec3(0,1,1));

    geom->setVertexArray(vary);

    osg::UShortArray * index1=new osg::UShortArray();
    index1->push_back(0);
    index1->push_back(1);
    index1->push_back(2);
    index1->push_back(3);

    osg::UShortArray * index2=new osg::UShortArray();
    index2->push_back(4);
    index2->push_back(5);
    index2->push_back(6);
    index2->push_back(7);
#if 1
    osg::MultiDrawElementsUShort * pr=new osg::MultiDrawElementsUShort();
    pr->setMode(GL_LINES);
    pr->addIndicesArray(index1);
    pr->addIndicesArray(index2);
   // pr->setBufferObject(new osg::ElementBufferObject);
#else
    osg::DrawElementsUShort * pr=new osg::DrawElementsUShort();
    pr->setMode(GL_LINES);
      pr->push_back(0);
    pr->push_back(1);
    pr->push_back(2);
    pr->push_back(3);
  //  pr->setBufferObject(new osg::ElementBufferObject);


#endif
    geom->addPrimitiveSet(pr);




    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);
   // geom->setUseVertexArrayObject(true);

    osg::StateSet* sset = geom->getOrCreateStateSet();

    osg::SubroutineUniform * subroutineu=new osg::SubroutineUniform(osg::Shader::FRAGMENT);
    subroutineu->addSubroutineName("ColorRed");
    sset->setAttribute(subroutineu);
    osg::Program* pgm = new osg::Program;
    pgm->setName( "osg shader subroutine uniform demo" );
    pgm->addShader( new osg::Shader( osg::Shader::FRAGMENT, fragSource ) );
    sset->setAttribute(pgm);
    sset->setUpdateCallback( new ShaderSubroutineSwaper(subroutineu));


    root->addChild(geom);
    osgDB::writeNodeFile(*root,"testsubroutine.osgt");
    osgViewer::Viewer viewer;
    viewer.setSceneData( root );
    return viewer.run();
}
