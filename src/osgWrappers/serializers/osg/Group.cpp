#include <osg/Group>
#include <osg/ValueObject>
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

static bool checkChildren( const osg::Group& node )
{
    return node.getNumChildren()>0;
}

static bool readChildren( osgDB::InputStream& is, osg::Group& node )
{
    unsigned int size = 0; is >> size >> is.BEGIN_BRACKET;
    for ( unsigned int i=0; i<size; ++i )
    {
        osg::ref_ptr<osg::Object> obj = is.readObject();
        osg::Node* child = dynamic_cast<osg::Node*>( obj.get() );
        if ( child ) node.addChild( child );
    }
    is >> is.END_BRACKET;
    return true;
}

static bool writeChildren( osgDB::OutputStream& os, const osg::Group& node )
{
    unsigned int size = node.getNumChildren();
    os << size << os.BEGIN_BRACKET << std::endl;
    for ( unsigned int i=0; i<size; ++i )
    {
        os << node.getChild(i);
    }
    os << os.END_BRACKET << std::endl;
    return true;
}

struct GroupGetNumChildren : public osgDB::MethodObject
{
    virtual bool run(void* objectPtr, osg::Parameters& inputParameters, osg::Parameters& outputParameters) const
    {
        osg::Group* group = reinterpret_cast<osg::Group*>(objectPtr);
        outputParameters.push_back(new osg::UIntValueObject("return", group->getNumChildren()));
        return true;
    }
};

struct GroupGetChild : public osgDB::MethodObject
{
    virtual bool run(void* objectPtr, osg::Parameters& inputParameters, osg::Parameters& outputParameters) const
    {
        if (inputParameters.empty()) return false;

        unsigned int index = 0;
        osg::ValueObject* indexObject = inputParameters[0]->asValueObject();
        if (indexObject) indexObject->getScalarValue(index);

        osg::Group* group = reinterpret_cast<osg::Group*>(objectPtr);
        outputParameters.push_back(group->getChild(index));

        return true;
    }
};

struct GroupSetChild : public osgDB::MethodObject
{
    virtual bool run(void* objectPtr, osg::Parameters& inputParameters, osg::Parameters& outputParameters) const
    {
        if (inputParameters.size()<2) return false;

        unsigned int index = 0;
        osg::ValueObject* indexObject = inputParameters[0]->asValueObject();
        if (indexObject) indexObject->getScalarValue(index);

        osg::Node* child = dynamic_cast<osg::Node*>(inputParameters[1].get());
        if (!child) return false;

        osg::Group* group = reinterpret_cast<osg::Group*>(objectPtr);
        group->setChild(index, child);

        return true;
    }
};

struct GroupAddChild : public osgDB::MethodObject
{
    virtual bool run(void* objectPtr, osg::Parameters& inputParameters, osg::Parameters& outputParameters) const
    {
        if (inputParameters.empty()) return false;

        osg::Node* child = dynamic_cast<osg::Node*>(inputParameters[0].get());
        if (!child) return false;

        osg::Group* group = reinterpret_cast<osg::Group*>(objectPtr);
        group->addChild(child);

        return true;
    }
};


struct GroupRemoveChild : public osgDB::MethodObject
{
    virtual bool run(void* objectPtr, osg::Parameters& inputParameters, osg::Parameters& outputParameters) const
    {
        if (inputParameters.empty()) return false;

        osg::Node* child = dynamic_cast<osg::Node*>(inputParameters[0].get());
        if (!child) return false;

        osg::Group* group = reinterpret_cast<osg::Group*>(objectPtr);
        group->removeChild(child);

        return true;
    }
};

#define CHECKPARAMS(INPARAMRETRIV) \
{static char in[] = #INPARAMRETRIV;\
static short l = strlen(##INPARAMRETRIV);\
char  *inptr = &in[0];\
int i = 0; int j = 0; \
while (inptr < &in[l - 1])if (**inptr++ == ',') j++;\
}

#define CHECKPARAMS2(XXX) \
{const char* in = #XXX;\
}

#define  WRAP_VALUEOBJECT_METHOD2(OSGVALUEOBJECTTYPE,CLASS,GETTERMETHODNAME,INPARAMSCALL,INPARAMRETRIVE) \
struct wrap_##GETTERMETHODNAME : public osgDB::MethodObject{\
virtual bool run(void* objectPtr, osg::Parameters& inputParameters, osg::Parameters& outputParameters) const\
{\
CLASS* group = reinterpret_cast<CLASS*>(objectPtr); \
	CHECKPARAMS2(##INPARAMRETRIVE);\
	##INPARAMRETRIVE;\
outputParameters.push_back(new OSGVALUEOBJECTTYPE("return", group->GETTERMETHODNAME(INPARAMSCALL))); \
return true; \
}\
};



#define  WRAP_VALUEOBJECT_METHOD(OSGVALUEOBJECTTYPE,CLASS,GETTERMETHODNAME) \
struct wrap_##GETTERMETHODNAME : public osgDB::MethodObject{\
virtual bool run(void* objectPtr, osg::Parameters& inputParameters, osg::Parameters& outputParameters) const\
{\
CLASS* group = reinterpret_cast<CLASS*>(objectPtr); \
	\
outputParameters.push_back(new OSGVALUEOBJECTTYPE("return", group->GETTERMETHODNAME())); \
return true; \
}\
};

#define STR(XXX) (#XXX)

#define XSTR(XXX) STR(#XXX)


#define DUMMYSTUFF
WRAP_VALUEOBJECT_METHOD(osg::UIntValueObject, osg::Group, getNumChildren)
//WRAP_VALUEOBJECT_METHOD2(osg::UIntValueObject, osg::Group, insertChild, STR(p1, p2), STR(p1, p2));
//ADD_METHOD_OBJECT(##GETTERMETHODNAME, wrap_##GETTERMETHODNAME);

template <typename T,typename N>
struct policy : public T{
    using T::getString;
N n;
const char * getString(){return n.getString()+getString();};

};

struct sampleT{
    std::string str;
    sampleT(char *s){str=std::string(s);}
    const char* getString(){return str.c_str();}
};
struct endT{
    char* getString(){return 0;}
};
//policy<sampleT("fok"), policy<sampleT("fok"),endT> a;

REGISTER_OBJECT_WRAPPER( Group,
                         new osg::Group,
                         osg::Group,
                         "osg::Object osg::Node osg::Group" )
{
    ADD_USER_SERIALIZER( Children );  // _children

	ADD_METHOD_OBJECT("getNumChildren", wrap_getNumChildren);
    ADD_METHOD_OBJECT( "getChild", GroupGetChild );
    ADD_METHOD_OBJECT( "setChild", GroupSetChild );
    ADD_METHOD_OBJECT( "addChild", GroupAddChild );
    ADD_METHOD_OBJECT( "removeChild", GroupRemoveChild );
}
