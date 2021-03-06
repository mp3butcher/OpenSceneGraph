/* -*-c++-*- OpenSceneGraph example, osgcomputeshaders.
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

// Written by Wang Rui
// This example can work only if GL version is 4.3 or greater

#include <osg/Texture2D>
#include <osg/BindImageTexture>
#include <osg/DispatchCompute>
#include <osg/Geode>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/BufferIndexBinding>
#include <osg/DispatchCompute>

static const char* computeSrc = {
    "#version 430 compatibility\n"
    "uniform float osg_FrameTime;\n"
    "layout (r32f, binding =0) uniform image2D targetTex;\n"
    "layout (local_size_x = 16, local_size_y = 16) in;\n"
    "layout(binding = 1) uniform atomic_uint acRed;\n"
    "void main() {\n"
    "  uint fok=atomicCounterIncrement(acRed); ivec2 storePos = ivec2(gl_GlobalInvocationID.xy);\n"
    "   float coeffcient = 0.5*sin(float(fok*gl_WorkGroupID.x + gl_WorkGroupID.y)*0.1 + osg_FrameTime);\n"
    "   coeffcient *= length(vec2(ivec2(gl_LocalInvocationID.xy) - ivec2(8)) / vec2(8.0));\n"
    "   imageStore(targetTex, storePos, vec4(1.0-coeffcient, 0.0, 0.0, 0.0));\n"
    "}\n"
};
class ReadBackAndResetCallback : public osg::SyncBufferDataUpdateCallback {
public:
    ReadBackAndResetCallback(osg::BufferData*bd):osg::SyncBufferDataUpdateCallback(bd) {
        this->setRamAccess(osg::SyncBufferDataUpdateCallback::READ_WRITE);
        setVRamAccess(osg::SyncBufferDataUpdateCallback::READ_WRITE);
    }
    virtual bool synctraversal(osg::Node *,osg::NodeVisitor*) {
             osg::UIntArray * array=dynamic_cast<  osg::UIntArray*>(_bd.get());
       //      if(!array->isDirty()) {

            unsigned int numPixel = osg::maximum(0u, array->front());

            // if ((renderInfo.getView()->getFrameStamp()->getFrameNumber() % 10) == 0)
            if(array->front()!=0)   {
                OSG_WARN << "osgatomiccounter : draw " << array->front() << " pixels." << std::endl;
                (*array)[0]=0;
                _bd->dirty();
            }
      //  }

    }

};
int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    // Create the texture as both the output of compute shader and the input of a normal quad
    osg::ref_ptr<osg::Texture2D> tex2D = new osg::Texture2D;
    tex2D->setTextureSize( 512, 512 );
    tex2D->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
    tex2D->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
    tex2D->setInternalFormat( GL_R32F );
    tex2D->setSourceFormat( GL_RED );
    tex2D->setSourceType( GL_FLOAT );
    // So we can use 'image2D' in the compute shader
    osg::ref_ptr<osg::BindImageTexture> imagbinding = new osg::BindImageTexture(0, tex2D, osg::BindImageTexture::WRITE_ONLY, GL_R32F);


    // The compute shader can't work with other kinds of shaders
    // It also requires the work group numbers. Setting them to 0 will disable the compute shader
    osg::ref_ptr<osg::Program> computeProg = new osg::Program;
    computeProg->addShader( new osg::Shader(osg::Shader::COMPUTE, computeSrc) );

    // Create a node for outputting to the texture.
    // It is OK to have just an empty node here, but seems inbuilt uniforms like osg_FrameTime won't work then.
    // TODO: maybe we can have a custom drawable which also will implement glMemoryBarrier?
    osg::ref_ptr<osg::Node> sourceNode = new osg::DispatchCompute(512/16, 512/16, 1 );
    sourceNode->setDataVariance( osg::Object::DYNAMIC );
    osg::ref_ptr<osg::UIntArray> atomicCounterArrayBlue = new osg::UIntArray;
    atomicCounterArrayBlue->push_back(0);
    atomicCounterArrayBlue->push_back(0);
    atomicCounterArrayBlue->push_back(0);
    atomicCounterArrayBlue->push_back(0);

    osg::ref_ptr<osg::AtomicCounterBufferObject> acboBlueBO = new osg::AtomicCounterBufferObject;
    acboBlueBO->setUsage(GL_STREAM_COPY);
    atomicCounterArrayBlue->setBufferObject(acboBlueBO);
    osg::ref_ptr<osg::AtomicCounterBufferBinding> acbbBlue = new osg::AtomicCounterBufferBinding(1,atomicCounterArrayBlue.get(), 0, sizeof(GLuint));

    sourceNode->getOrCreateStateSet()->setAttributeAndModes(acbbBlue);
    sourceNode->getOrCreateStateSet()->setAttributeAndModes( computeProg.get() );
    sourceNode->getOrCreateStateSet()->addUniform( new osg::Uniform("targetTex", (int)0) );
    sourceNode->getOrCreateStateSet()->setTextureAttributeAndModes( 0, tex2D.get() );
    ReadBackAndResetCallback *readbackandreset=new ReadBackAndResetCallback(acbbBlue->getBufferData());
    osg::ref_ptr<osg::Geode> quad = new osg::Geode;
    quad->addUpdateCallback( readbackandreset);
    // Display the texture on a quad. We will also be able to operate on the data if reading back to CPU side
    osg::Geometry* geom = osg::createTexturedQuadGeometry(
                              osg::Vec3(), osg::Vec3(1.0f,0.0f,0.0f), osg::Vec3(0.0f,0.0f,1.0f) );
    quad->addDrawable( geom );
    quad->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    quad->getOrCreateStateSet()->setTextureAttributeAndModes( 0, tex2D.get() );
    quad->getOrCreateStateSet()->setAttributeAndModes(imagbinding.get());
    // Create the scene graph and start the viewer
    osg::ref_ptr<osg::Group> scene = new osg::Group;
    scene->addChild( sourceNode );
    scene->addChild( quad.get() );

    osgViewer::Viewer viewer;
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    viewer.addEventHandler( new osgViewer::StatsHandler );
    viewer.addEventHandler( new osgViewer::WindowSizeHandler );
    viewer.setSceneData( scene.get() );
    return viewer.run();
}
