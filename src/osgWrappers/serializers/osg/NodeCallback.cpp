#undef OBJECT_CAST
#define OBJECT_CAST dynamic_cast
#include <osg/Node>
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

REGISTER_OBJECT_WRAPPER( NodeCallback,
                         new osg::NodeCallback,
                         osg::NodeCallback,
                         "osg::NodeCallback" )
{
}

#undef OBJECT_CAST
#define OBJECT_CAST static_cast
