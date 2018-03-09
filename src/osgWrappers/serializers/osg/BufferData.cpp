#include <osg/BufferObject>
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>
#include <assert.h>
static bool checkBufferObject( const osg::BufferData& node )
{
    return true;
}

///don't add BufferData to BufferObject (let BufferObject Serializer::readBufferData do it)
static bool readBufferObject( osgDB::InputStream& is, osg::BufferData& bd )
{
   //BufferData_serializer_BufferData &localbd  =static_cast<BufferData_serializer_BufferData&>(bd);
    osg::ref_ptr<osg::Object> obj = is.readObject();
    osg::BufferObject* bo = dynamic_cast<osg::BufferObject*>( obj.get() );

    if ( bo ) bd.setBufferObject(bo);

    //if ( bo ) localbd.setBufferObjectWithoutAddingBD2BO(bo);
    return true;
}

static bool writeBufferObject( osgDB::OutputStream& os, const osg::BufferData& bd )
{
   // if (os.getWriteBufferObjectConfiguration())
        os << bd.getBufferObject();
   // else os << (osg::BufferObject*)NULL;
    return true;
}
REGISTER_OBJECT_WRAPPER( BufferData,
                         0,
                         osg::BufferData,
                         "osg::Object osg::BufferData" )
{
    {
        UPDATE_TO_VERSION_SCOPED( 147 )
        //ADD_OBJECT_SERIALIZER(BufferObject, osg::BufferObject, NULL);
       ADD_USER_SERIALIZER(BufferObject);
    }
}
