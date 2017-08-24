#include <osgUtil/DrawElementTransform>
#include <osg/TriangleIndexFunctor>
#include <osgUtil/MeshOptimizers>
#include <osg/Geode>

template <typename InType, typename OutType>
OutType * copy(InType& array)
{
    unsigned int size = array.size();
    OutType * newArray = new OutType(array.getMode(), size);
    OutType & na = *newArray;

    for (unsigned int i = 0; i < size; ++i) na[i] = array[i];

    return newArray;
}

template <typename InType>
unsigned int getMax(InType& array)
{
    unsigned int max = 0;
    unsigned int size = array.size();

    for (unsigned int i = 0; i < size; ++i)
    {
        if (array[i] > max) max = array[i];
    }
    return (max);
}

using namespace osg;
namespace osgUtil
{


struct FindPointSharing
{

    FindPointSharing() {
        _result[0]=0xFFFFFFFF;
        _result[1]=0xFFFFFFFF;
        _result[2]=0xFFFFFFFF;
    }

    void setTriangle(unsigned int p1, unsigned int p2, unsigned int p3) {
        _p[0]=p1;
        _p[1]=p2;
        _p[2]=p3;
        _result[0]=p1;
        _result[1]=p2;
        _result[2]=p3;
    }


    // for use  in the triangle functor.
    inline void operator()(unsigned int p1, unsigned int p2, unsigned int p3)
    {
        if( p1!=_p[0]&&_p[1]!=p2&&_p[2]!=p3) {
            ///if not the same triangle
            /// test all combination
            unsigned int p[]= {p1,p2,p3};

            //rotate _p
            for(int i=0; i<3; i++) {

                //rotate p
                for(int j=0; j<3; j++) {
                    //candidate : p[(j+2)%3]

                    if( _p[(i+2)%3]=p[(j+2)%3] &&  _p[(i+1)%3] == p[(j+1)%3])
                        _result[i]=p[j];

                }
            }

        }
    }
    unsigned int _p[3],_result[3];

};
typedef osg::TriangleIndexFunctor<FindPointSharing> FindIndexSharing2Point;
class GenerateAdjacency : public osg::PrimitiveIndexFunctor//, public T
{
public:

    GenerateAdjacency(DrawElements&de):dre(de) {}
    virtual void setVertexArray(unsigned int,const Vec2*)
    {
    }

    virtual void setVertexArray(unsigned int ,const Vec3* )
    {
    }

    virtual void setVertexArray(unsigned int,const Vec4* )
    {
    }

    virtual void setVertexArray(unsigned int,const Vec2d*)
    {
    }

    virtual void setVertexArray(unsigned int ,const Vec3d* )
    {
    }

    virtual void setVertexArray(unsigned int,const Vec4d* )
    {
    }

    virtual void begin(GLenum mode)
    {
        //_modeCache = mode;
        // _indexCache.clear();
    }

    virtual void vertex(unsigned int vert)
    {
        //_indexCache.push_back(vert);
    }

    virtual void end()
    {
//        if (!_indexCache.empty())
        {
            //     drawElements(_modeCache,_indexCache.size(),&_indexCache.front());
        }
    }

    virtual void drawArrays(GLenum mode,GLint first,GLsizei count)
    {
        /*    switch(mode)
            {
                case(GL_TRIANGLES):
                {
                    unsigned int pos=first;
                    for(GLsizei i=2;i<count;i+=3,pos+=3)
                    {
                        this->operator()(pos,pos+1,pos+2);
                    }
                    break;
                }
                case(GL_TRIANGLE_STRIP):
                 {
                    unsigned int pos=first;
                    for(GLsizei i=2;i<count;++i,++pos)
                    {
                        if ((i%2)) this->operator()(pos,pos+2,pos+1);
                        else       this->operator()(pos,pos+1,pos+2);
                    }
                    break;
                }
                case(GL_QUADS):
                {
                    unsigned int pos=first;
                    for(GLsizei i=3;i<count;i+=4,pos+=4)
                    {
                        this->operator()(pos,pos+1,pos+2);
                        this->operator()(pos,pos+2,pos+3);
                    }
                    break;
                }
                case(GL_QUAD_STRIP):
                {
                    unsigned int pos=first;
                    for(GLsizei i=3;i<count;i+=2,pos+=2)
                    {
                        this->operator()(pos,pos+1,pos+2);
                        this->operator()(pos+1,pos+3,pos+2);
                    }
                    break;
                }
                case(GL_POLYGON): // treat polygons as GL_TRIANGLE_FAN
                case(GL_TRIANGLE_FAN):
                {
                    unsigned int pos=first+1;
                    for(GLsizei i=2;i<count;++i,++pos)
                    {
                        this->operator()(first,pos,pos+1);
                    }
                    break;
                }
                case(GL_POINTS):
                case(GL_LINES):
                case(GL_LINE_STRIP):
                case(GL_LINE_LOOP):
                default:
                    // can't be converted into to triangles.
                    break;
            }*/
    }
    std::vector<uint> newindex;
    osg::PrimitiveSet::Mode newmode;
    template<typename T> inline void drawElementsImplementation(GLenum mode, GLsizei count, const T* indices, const GLuint basevertex = 0)
    {
        if (indices==0 || count==0) return;

        typedef T Index;
        typedef const Index* IndexPointer;

        switch(mode)
        {
        case(GL_TRIANGLES):
        {
            newmode=osg::PrimitiveSet::TRIANGLES_ADJACENCY;
            IndexPointer ilast = &indices[count];
            for(IndexPointer  iptr=indices; iptr<ilast; iptr+=3) {

                FindIndexSharing2Point finder;
                finder.setTriangle
                ((*iptr)+ basevertex,*(iptr+1)+ basevertex,*(iptr+2)+ basevertex);
                dre.accept(finder);
                newindex.push_back((*iptr)+ basevertex);
                newindex.push_back(finder._result[2]);
                newindex.push_back(*(iptr+1)+ basevertex);
                newindex.push_back(finder._result[0]);
                newindex.push_back(*(iptr+2)+ basevertex);
                newindex.push_back(finder._result[1]);
            }
            break;
        }
        case(GL_TRIANGLE_STRIP):
        {
            //TODO
            newmode=osg::PrimitiveSet::TRIANGLES_ADJACENCY;
            IndexPointer iptr = indices;
            for(GLsizei i=2; i<count; ++i,++iptr)
            {
                if ((i%2)) {
                    FindIndexSharing2Point finder;
                    finder.setTriangle(*(iptr)+ basevertex,*(iptr+2)+ basevertex,*(iptr+1)+ basevertex);
                }
                else       {
                    FindIndexSharing2Point finder;
                    finder.setTriangle(*(iptr)+ basevertex,*(iptr+1)+ basevertex,*(iptr+2)+ basevertex);
                }
            }
            break;
        }
        case(GL_QUADS):
        {
            newmode=osg::PrimitiveSet::TRIANGLES_ADJACENCY;
            IndexPointer iptr = indices;
            for(GLsizei i=3; i<count; i+=4,iptr+=4)
            {
                FindIndexSharing2Point finder;
                finder.setTriangle(*(iptr)+ basevertex,*(iptr+1)+ basevertex,*(iptr+2)+ basevertex);
                FindIndexSharing2Point finder2;
                finder2.setTriangle(*(iptr)+ basevertex,*(iptr+2)+ basevertex,*(iptr+3)+ basevertex);
            }
            break;
        }
        case(GL_QUAD_STRIP):
        {
            newmode=osg::PrimitiveSet::TRIANGLES_ADJACENCY;
            IndexPointer iptr = indices;
            for(GLsizei i=3; i<count; i+=2,iptr+=2)
            {
                FindIndexSharing2Point finder;
                finder.setTriangle(*(iptr)+ basevertex,*(iptr+1)+ basevertex,*(iptr+2)+ basevertex);
                FindIndexSharing2Point finder2;
                finder2.setTriangle(*(iptr+1)+ basevertex,*(iptr+3)+ basevertex,*(iptr+2)+ basevertex);
            }
            break;
        }
        case(GL_POLYGON): // treat polygons as GL_TRIANGLE_FAN
        case(GL_TRIANGLE_FAN):
        {
            IndexPointer iptr = indices;
            Index first = *iptr;
            ++iptr;
            for(GLsizei i=2; i<count; ++i,++iptr)
            {
                FindIndexSharing2Point finder;
                finder.setTriangle(first+ basevertex,*(iptr)+ basevertex,*(iptr+1)+ basevertex);
            }
            break;
        }
        case(GL_POINTS):
        case(GL_LINES):
        case(GL_LINE_STRIP):
        case(GL_LINE_LOOP):
        default:
            // can't be converted into to triangles.
            break;
        }
    }

    virtual void drawElements(GLenum mode,GLsizei count,const GLubyte* indices, const GLuint basevertex = 0)
    {
        drawElementsImplementation(mode, count, indices, basevertex);
    }

    virtual void drawElements(GLenum mode,GLsizei count,const GLushort* indices, const GLuint basevertex = 0)
    {
        drawElementsImplementation(mode, count, indices, basevertex);
    }

    virtual void drawElements(GLenum mode,GLsizei count,const GLuint* indices, const GLuint basevertex = 0)
    {
        drawElementsImplementation(mode, count, indices, basevertex);
    }
    osg::DrawElements &dre;
};



void MakeDrawElementAdjacency::generate_adjacency(osg::Geometry & geometry,unsigned int notfoundindex) const
{
    osg::Geometry::PrimitiveSetList & psl = geometry.getPrimitiveSetList();


    osg::Geometry::DrawElementsList drelist;
    geometry.getDrawElementsList(drelist);
    osg::Geometry::DrawElementsList::iterator it, end = drelist.end();

    if(drelist.size()!=psl.size()) {
        OSG_WARN<<"MakeDrawElementAdjacency: Geometry traverse contains DrawArrays that aren't supported:"<<std::endl;
    }

    unsigned int max = 0;

    for (it = drelist.begin(); it!=end; ++it)
    {
        GenerateAdjacency  genadj(*(*it));
        (*it)->accept(genadj);
        switch ((*it)->getType())
        {
        case osg::PrimitiveSet::DrawElementsUIntPrimitiveType: {
            osg::DrawElementsUInt & de = *static_cast<osg::DrawElementsUInt*>(*it);
            //copy back
            de.setMode(genadj.newmode);
            de.clear();
            for(int i=0; i<genadj.newindex.size(); ++i)
                de.push_back(genadj.newindex[i]);
            de.dirty();
            break;
        }
        case osg::PrimitiveSet::DrawElementsUBytePrimitiveType: {
            osg::DrawElementsUByte & de = *static_cast<osg::DrawElementsUByte*>(*it);
            //copy back
            de.setMode(genadj.newmode);
            de.clear();
            for(int i=0; i<genadj.newindex.size(); ++i)
                de.push_back(genadj.newindex[i]);
            de.dirty();
            break;
        }
        case osg::PrimitiveSet::DrawElementsUShortPrimitiveType:
        {
            osg::DrawElementsUShort & de = *static_cast<osg::DrawElementsUShort*>(*it);
            //copy back
            de.setMode(genadj.newmode);
            de.clear();
            for(int i=0; i<genadj.newindex.size(); ++i)
                de.push_back(genadj.newindex[i]);
            de.dirty();
            break;
        }

        default:
            break;
        }
    }
}

void MakeDrawElementAdjacencyVisitor::apply(osg::Geometry& geom)
{
    IndexMeshVisitor im;
    im.setIsForcedReindexationEnabled(true);
    geom.accept(im);
    im.makeMesh(geom);
    //geom.getBoundingBox();
    // geom.setComputeBoundingBoxCallback(new osg::Drawable::ComputeBoundingBoxCallback);
    MakeDrawElementAdjacency dets;
    dets.generate_adjacency(geom);
}

}
