

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Vec3>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/PolygonStipple>
#include <osg/TriangleFunctor>
#include <osg/io_utils>
#include <assert.h>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgGA/TrackballManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osg/Math>

#include <iostream>

#include "SharedVAOGeometry"
#define NEWVAS 1

///1)transform Geometries into SharedVAOGeometries
///2)transform primset to their basevertex equivalents
///3)maximize BOset sharing among SharedVAOGeometries :1VAS/BOset (responsible of its create is so called master geometry )
///if attribute are per vertex nothing can go wrong, all arrays will share basevertex in their bo
///4)compute primsets basevertices according bo structure
class  MakeSharedBufferObjectsVisitor : public osg::NodeVisitor
{
public:

    MakeSharedBufferObjectsVisitor():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
        _hardMaxbuffsize=1000000000;///min of all glbuffermaxsize
        _softMaxbuffsize=900000000;///ofline: keep all boset during traversal
        _numVAOsInUsed=0;
        _hack=true;
    }
    virtual void apply(osg::Geode& transform);
    // virtual void apply(osg::Geometry& transform);
    void setHardBufferSize(unsigned int i)    {
        _hardMaxbuffsize=i;
    }
    unsigned int getHardBufferSize()const    {
        return _hardMaxbuffsize;
    }
    void setSoftBufferSize(unsigned int i)    {
        _softMaxbuffsize=i;
    }
    unsigned int getSoftBufferSize()const    {
        return _softMaxbuffsize;
    }
    unsigned int getNumBufferSetGenerated()const    {
        return _numVAOsInUsed;
    }

    bool isHackActivated()const    {
        return _hack;
    }
    void setIsHackActivated(bool i)    {
        _hack=i;
    }

protected:

    typedef std::vector< osg::BufferObject* > BuffSet;
#ifndef NEWVAS
    typedef  std::pair<BuffSet,SharedVAOGeometry *>  BuffSetAndMaster;
    typedef  std::vector< BuffSetAndMaster > VecBuffSetAndMaster;
    SharedVAOGeometry*  treatBufferObjects(SharedVAOGeometry* g);
#else
    typedef  std::pair<BuffSet,osg::Geometry *>  BuffSetAndMaster;
    typedef  std::vector< BuffSetAndMaster > VecBuffSetAndMaster;
    osg::Geometry*  treatBufferObjects(osg::Geometry* g);

#endif
    std::map<unsigned int,VecBuffSetAndMaster > _store;

    unsigned int _hardMaxbuffsize;///prohibit bufferdata concatenation in bufferobject
    unsigned int _softMaxbuffsize;///hint a bufferobject is full (and increment lastempty)
    unsigned int _numVAOsInUsed;///for stats
    bool _hack;
};


void setPrimitivesBaseVertex(osg::Geometry *g){

    GLint basevertex=0; uint elmtsize=static_cast<osg::Array*>(g->getVertexArray()->getBufferObject()->getBufferData(0))->getElementSize();
    if(g->getVertexArray()) {
        for(int i=0; i<g->getVertexArray()->getBufferIndex(); i++) {
assert(static_cast<osg::Array*>(g->getVertexArray()->getBufferObject()->getBufferData(i))->getElementSize()==elmtsize);
            basevertex+=g->getVertexArray()->getBufferObject()->getBufferData(i)->getTotalDataSize();
        }

        assert(g->getVertexArray()->getElementSize()==elmtsize);
        basevertex/=g->getVertexArray()->getElementSize();
///  setbasevertex without the pointer hack in drawElementsbasevertex

        osg::Geometry::PrimitiveSetList& drawelmts=g->getPrimitiveSetList();

        for(osg::Geometry::PrimitiveSetList::iterator prit=drawelmts.begin(); prit!=drawelmts.end(); prit++)
        {
            if(dynamic_cast<osg::DrawElementsUByte*>(prit->get()))
                dynamic_cast<DrawElementsBaseVertexUBYTE*>(prit->get())->_basevertex=basevertex;
            else if(dynamic_cast<osg::DrawElementsUInt*>(prit->get()))
                dynamic_cast<DrawElementsBaseVertexUINT*>(prit->get())->_basevertex=basevertex;
            else if(dynamic_cast<osg::DrawElementsUShort*>(prit->get()))
                dynamic_cast<DrawElementsBaseVertexUSHORT*>(prit->get())->_basevertex=basevertex;
            else    if(dynamic_cast<osg::DrawArrays*>(prit->get())){
             //computebound fail with classic DrawArray   dynamic_cast<DrawArraysBaseVertex*>(prit->get())->setFirst( basevertex);//basevertex;
                //don't understand why drawablecomputebound is based on primitivefunctor and not simply on the parsing the whole vertexarray ...kinda overkill
            dynamic_cast<DrawArraysBaseVertex*>(prit->get())->_basevertex = basevertex;//basevertex;
            }
        }
    }
}

void SharedVAOGeometry::setMaster(SharedVAOGeometry*m)
{
    //if(m!=this)
    master=m;

    if(!m)return;
    setPrimitivesBaseVertex(this);

}
/*
void MakeSharedBufferObjectsVisitor::apply(osg::Geometry&g)
{
    treatBufferObjects(&g);
}*/
void MakeSharedBufferObjectsVisitor::apply(osg::Geode&g)
{
    osg::ref_ptr<osg::Geode> gr=new osg::Geode;
    osg::Geometry * geom;
    for(uint i=0; i<g.getNumDrawables(); i++)
    {
        geom=g.getChild(i)->asGeometry();
        if(geom &&geom->getVertexArray()) {
///DEBUG remove stateset
            geom->setStateSet(0);
            if(_hack)
#ifndef NEWVAS
                gr->addDrawable(new SharedVAOGeometry(*(osg::Geometry*)g.getDrawable(i),osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL)));
#else
                gr->addDrawable(g.getDrawable(i));

#endif
            else gr->addDrawable(geom);
        }
    }
    g.removeDrawables(0,g.getNumDrawables());

    for(uint i=0; i<gr->getNumDrawables(); i++)
    {
        geom=gr->getChild(i)->asGeometry();
        ///force data variance to static
        geom->setDataVariance(osg::Object::STATIC);
        geom->setUseVertexBufferObjects(true);
        geom->setUseDisplayList(false);
        geom->setUseVertexArrayObject(true);


        if(_hack) geom=treatBufferObjects( (SharedVAOGeometry*) geom);
        if(geom)g.addDrawable(geom);
        else std::cerr<<"trowed"<<std::endl;
    }
}
///return Hash without lastbit (index array bit)
#define MAX_TEX_COORD 8
#define MAX_VERTEX_ATTRIB 16
///in case of array binding != PER_VERTEX drop the array
#define HACKARRAY(GEOM,ARRAYPROP) if (GEOM->get##ARRAYPROP() && GEOM->get##ARRAYPROP()->getBinding()!=osg::Array::BIND_PER_VERTEX) {OSG_DEBUG<<#ARRAYPROP<<"removed"<<std::endl; GEOM->set##ARRAYPROP(0);}

#define HACKARRAYS(GEOM,ARRAYPROP,I) if (GEOM->get##ARRAYPROP(I) && GEOM->get##ARRAYPROP(I)->getBinding()!=osg::Array::BIND_PER_VERTEX){OSG_DEBUG<<#ARRAYPROP<<"removed"<<std::endl;GEOM->set##ARRAYPROP(I,0);}

unsigned int getArrayList(osg::Geometry*g,osg::Geometry::ArrayList &arrayList)
{
    unsigned int hash=0;
    HACKARRAY(g,VertexArray)
            HACKARRAY(g,VertexArray)
            HACKARRAY(g,NormalArray)
            HACKARRAY(g,ColorArray)
            HACKARRAY(g,SecondaryColorArray)
            HACKARRAY(g,FogCoordArray)
    if (g->getVertexArray())
    {
        hash++;
        arrayList.push_back(g->getVertexArray());
    }
    hash<<=1;
    if (g->getNormalArray())
    {
        hash++;
        arrayList.push_back(g->getNormalArray());
    }
    hash<<=1;
    if (g->getColorArray())
    {
        hash++;
        arrayList.push_back(g->getColorArray());
    }
    hash<<=1;
    if (g->getSecondaryColorArray())
    {
        hash++;
        arrayList.push_back(g->getSecondaryColorArray());
    }
    hash<<=1;
    if (g->getFogCoordArray())
    {
        hash++;
        arrayList.push_back(g->getFogCoordArray());
    }
    hash<<=1;
    for(unsigned int unit=0; unit<g->getNumTexCoordArrays(); ++unit)
    {
        HACKARRAYS(g,TexCoordArray,unit)
        osg::Array* array = g->getTexCoordArray(unit);
        if (array)
        {
            hash++;
            arrayList.push_back(array);
        }
        hash<<=1;
    }
    hash<<=MAX_TEX_COORD-g->getNumTexCoordArrays();
    for(unsigned int  index = 0; index <g->getNumVertexAttribArrays(); ++index )
    {
        HACKARRAYS(g,VertexAttribArray,index)
        osg::Array* array =g->getVertexAttribArray(index);
        if (array)
        {
            hash++;
            arrayList.push_back(array);
        }
        hash<<=1;
    }
    hash<<=MAX_VERTEX_ATTRIB-g->getNumVertexAttribArrays();
    return hash;
}


///check bufferobject already guesting the bd
bool isGuesting(const osg::BufferObject&bo,const osg::BufferData*bd)
{
    for(unsigned int i=0; i<bo.getNumBufferData(); i++)
        if(bo.getBufferData(i)==bd)return true;
    return false;
}

#ifndef NEWVAS

    SharedVAOGeometry * MakeSharedBufferObjectsVisitor::treatBufferObjects(SharedVAOGeometry* g)
#else
/** Shared VAS createVASCallback **/
struct MasterGeomVertexArrayStateCallback : public virtual osg::Drawable::CreateVertexArrayStateCallback
{
    osg::ref_ptr<osg::Drawable> _master;
    MasterGeomVertexArrayStateCallback(osg::Drawable *dr=0):_master(dr) {}

    void setMasterGeometry(osg::Drawable*dr){_master=dr;}
    osg::Drawable * getMasterGeometry()const{return _master;}
    MasterGeomVertexArrayStateCallback(const MasterGeomVertexArrayStateCallback& rhs,const osg::CopyOp& copyop):
        osg::Drawable::CreateVertexArrayStateCallback(rhs, copyop) {_master=rhs._master;}

    META_Object(osg, MasterGeomVertexArrayStateCallback);

    /** do customized createVertexArrayState .*/
    virtual osg::VertexArrayState* createVertexArrayStateImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* /*drawable*/) const
    {
        return _master->createVertexArrayStateImplementation(renderInfo);
    }
};
    osg::Geometry*  MakeSharedBufferObjectsVisitor::treatBufferObjects(osg::Geometry* g)
#endif

{
    osg::Geometry::ArrayList  bdlist;
    unsigned int hash= getArrayList(g,bdlist),hasht;

    for(osg::Geometry::ArrayList::iterator prit=bdlist.begin(); prit!=bdlist.end(); prit++){

        if((*prit)->getBinding()!=    osg::Array::BIND_PER_VERTEX){
            ///cant do anything
            std::cerr<<"throwing geom"<<std::endl;
            g=0;return 0;
        }
    }
    osg::Geometry::PrimitiveSetList newdrawelmts;

    ///convert primset to BaseVertex
#if 1
    osg::Geometry::PrimitiveSetList& prlist=g->getPrimitiveSetList();;
    for(osg::Geometry::PrimitiveSetList::iterator prit=prlist.begin(); prit!=prlist.end(); prit++)
    {
        if(dynamic_cast<osg::DrawElementsUByte*>(prit->get()))
            newdrawelmts.push_back( new DrawElementsBaseVertexUBYTE(*dynamic_cast<osg::DrawElementsUByte*>(prit->get()),osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL)));
        else if(dynamic_cast<osg::DrawElementsUInt*>(prit->get()))
            newdrawelmts.push_back( new DrawElementsBaseVertexUINT(*dynamic_cast<osg::DrawElementsUInt*>(prit->get()),osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL)));
        else if(dynamic_cast<osg::DrawElementsUShort*>(prit->get()))
            newdrawelmts.push_back( new DrawElementsBaseVertexUSHORT(*dynamic_cast<osg::DrawElementsUShort*>(prit->get()),osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL)));
        else if(dynamic_cast<osg::DrawArrays*>(prit->get()))            newdrawelmts.push_back( new DrawArraysBaseVertex(*dynamic_cast<osg::DrawArrays*>(prit->get()),g,osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL)));
        else
        {
            std::cout<<"Not taken into account"<<std::endl; return 0;
        }
        // g->removePrimitiveSet(g->getPrimitiveSetIndex(*prit));

    }
    ///basevertices will be setted later
    g->removePrimitiveSet(0,g->getNumPrimitiveSets());
    for(osg::Geometry::PrimitiveSetList::iterator prit=newdrawelmts.begin(); prit!=newdrawelmts.end(); prit++)
        g->addPrimitiveSet(*prit);
#endif

    osg::Geometry::DrawElementsList drawelmts;
    //std::cout<<bdlist.size()<<" "<<hash<<std::endl;
    drawelmts.clear();
    g->getDrawElementsList(drawelmts);
    hasht=hash;
    unsigned int nbborequired=bdlist.size(),nbdrawelmt=0;


    for(unsigned index=0; index< drawelmts.size(); index++)
    {
        if(nbborequired==bdlist.size())
        {
            nbborequired=bdlist.size()+1;
            hash=hasht+1;
        }
        nbdrawelmt++;
    }

    VecBuffSetAndMaster & vecBuffSetANDmaster=_store[hash];


    VecBuffSetAndMaster::iterator itbuffset=  vecBuffSetANDmaster.begin();//+vecBuffSet.lastempty;

    unsigned int * buffSize=new unsigned int [nbborequired+nbdrawelmt],*ptrbuffsize=buffSize;

    //while !itbuffset.canGuest(bdlist)
    bool canGuest=false;
    bool alreadyGuesting=true;
    osg::Geometry::ArrayList::iterator arit;
    while(!canGuest && itbuffset!=vecBuffSetANDmaster.end())
    {
        canGuest=true;
        alreadyGuesting=true;

        BuffSet::iterator itbo=(*itbuffset).first.begin();
        for(arit=bdlist.begin(); arit!=bdlist.end(); itbo++,arit++,ptrbuffsize++)
        {
            *ptrbuffsize=(*itbo)->computeRequiredBufferSize()+(*arit)->getTotalDataSize();
            if(*ptrbuffsize>_hardMaxbuffsize)canGuest=false;
            ///check bufferobject already guesting the bd
            if(!isGuesting(**itbo,(*arit)))alreadyGuesting=false;

        }

        if(!drawelmts.empty())
            *ptrbuffsize=(*itbo)->computeRequiredBufferSize();
        for(unsigned index=0; index< drawelmts.size(); index++,ptrbuffsize++)
        {
            *ptrbuffsize+=drawelmts[index]->getTotalDataSize();
            if(*ptrbuffsize>_hardMaxbuffsize)                    canGuest=false;
            ///check bufferobject already guesting the bd
            if(!isGuesting(**itbo,drawelmts[index]))alreadyGuesting=false;
        }

        ptrbuffsize=buffSize; //reset ptr to bufferSizes table
        //DEBUG alreadyGuesting=false;
        if(alreadyGuesting)
        {
            delete [] buffSize;

            if(!(*itbuffset).second)
                std::cout<<"l258 reusing vao of "<<(*itbuffset).second<<std::endl;
#ifndef NEWVAS
            g->setMaster((*itbuffset).second);
#else

            setPrimitivesBaseVertex(g);

            //g->setVertexArrayState((*itbuffset).second->getVertexArrayState());
            g->setCreateVertexArrayStateCallback((*itbuffset).second->getCreateVertexArrayStateCallback());
#endif
            OSG_WARN<<"already guested by the buffer set"<<std::endl;
            return g;
        }
        if(!canGuest)itbuffset++;

    }

    //if(itbuffset!=vecBuffSetANDmaster.first.end())//==
        if(canGuest)
    {
        unsigned buffSetMaxSize=0;
        BuffSet::iterator itbo= itbuffset->first.begin();
        ptrbuffsize=buffSize;
        for( arit=bdlist.begin(); arit!=bdlist.end(); itbo++,arit++,ptrbuffsize++)
        {
            (*arit)->setBufferObject(*itbo);
            buffSetMaxSize=buffSetMaxSize<*ptrbuffsize?*ptrbuffsize:buffSetMaxSize;
        }

        for(unsigned index=0; index< drawelmts.size(); index++)
        {//assert(*itbo == itbuffset->back());
            drawelmts[index]->setBufferObject(*itbo);
            buffSetMaxSize=buffSetMaxSize<*ptrbuffsize?*ptrbuffsize:buffSetMaxSize;
            ptrbuffsize++;
        }
        ///check if bufferobject is full against soft limit
        //if( buffSetMaxSize>_softMaxbuffsize)            vecBuffSetANDmaster.first.erase(itbuffset);

        delete [] buffSize;
        if(!(*itbuffset).second)
            std::cout<<"l288 reusing vao of "<<(*itbuffset).second<<std::endl;
#ifndef NEWVAS
        g->setMaster((*itbuffset).second);
#else

        setPrimitivesBaseVertex(g);
       // g->setVertexArrayState((*itbuffset).second->getVertexArrayState());
        g->setCreateVertexArrayStateCallback((*itbuffset).second->getCreateVertexArrayStateCallback());
#endif

        return g;
    }



    ///new BuffSetis required
    BuffSetAndMaster     newBuffSet;
    newBuffSet.second=g;
    /// Check if buffer object set offsets configuration is compatible with base vertex draw
    int commonoffset=0;
    bool canbereused=true;
    bool ready2go=
        #if 1
            false;
#else
            true;
    arit=bdlist.begin();
    if(!bdlist.empty())
    {
        for(uint i=0; i<(*arit)->getBufferIndex(); i++)commonoffset+=(*arit)->getBufferObject()->getBufferData(i)->getTotalDataSize();
        newBuffSet.push_back((*arit)->getBufferObject());
        for( arit++; arit!=bdlist.end()&&ready2go; arit++)
        {
            int offset=0;
            for(uint i=0; i<(*arit)->getBufferIndex(); i++)offset+=(*arit)->getBufferObject()->getBufferData(i)->getTotalDataSize();
            if(offset!=commonoffset)ready2go=false;
            if((*arit)->getBufferObject()->computeRequiredBufferSize()>_softMaxbuffsize)canbereused=false;
            newBuffSet.push_back((*arit)->getBufferObject());
        }
    }
    else
        for(unsigned index=0; index< drawelmts.size()&&ready2go; index++)
        {
            if(drawelmts[0]->getBufferObject()!=drawelmts[index]->getBufferObject())ready2go=false;
            if(drawelmts[index]->getBufferObject()->computeRequiredBufferSize()>_softMaxbuffsize)canbereused=false;
            newBuffSet.push_back(drawelmts[index]->getBufferObject());
            break;
        }
#endif
    ///if configuration is good assume bo set is ready to be used as it is
    if(ready2go)
    {
        if(canbereused) vecBuffSetANDmaster.push_back(newBuffSet);
        delete [] buffSize;
        if(!newBuffSet.second)
            std::cout<<"l327 reusing vao of "<<newBuffSet.second<<std::endl;

#ifndef NEWVAS
        g->setMaster((SharedVAOGeometry*)newBuffSet.second);
#else

        setPrimitivesBaseVertex(g);
       // g->setVertexArrayState(newBuffSet.second->getVertexArrayState());
newBuffSet.second->setCreateVertexArrayStateCallback(new MasterGeomVertexArrayStateCallback(newBuffSet.second));
        g->setCreateVertexArrayStateCallback(newBuffSet.second->getCreateVertexArrayStateCallback());
#endif
        return g;
    }
    _numVAOsInUsed++;
   //if(vecBuffSetANDmaster.first.empty())
 //   vecBuffSetANDmaster.second=g;

    OSG_WARN<<"create new vao buffset, num vao currently in used:"<<_numVAOsInUsed<<" geom:"<<g<<std::endl;
    ///current configuration doesn't fit so create a new buffset
    newBuffSet.first.clear();
    std::cout<<newBuffSet.first.size()<<" "<<hash<<std::endl;

    osg::ElementBufferObject *ebo=0;
    for( arit=bdlist.begin(); arit!=bdlist.end(); arit++)
    {
        (*arit)->setBufferObject( new osg::VertexBufferObject());
        newBuffSet.first.push_back( (*arit)->getBufferObject());

    }
    if(!drawelmts.empty()) ebo=new osg::ElementBufferObject();
    for(unsigned index=0; index<drawelmts.size(); index++)
    {
        newBuffSet.first.push_back(ebo);
        drawelmts[index]->setBufferObject(ebo);
    }

    vecBuffSetANDmaster.push_back(newBuffSet);
    delete [] buffSize;
#ifndef NEWVAS
    g->setMaster(newBuffSet.second);
#else
    setPrimitivesBaseVertex(g);
   // g->setVertexArrayState(newBuffSet.second->getVertexArrayState());

    newBuffSet.second->setCreateVertexArrayStateCallback(new MasterGeomVertexArrayStateCallback(newBuffSet.second));
    g->setCreateVertexArrayStateCallback(newBuffSet.second->getCreateVertexArrayStateCallback());
#endif
    return g;
}

/// This demo illustrates how VAO Sharing and basevertex draw reduce CPU draw overhead
/// when mesh number get bigger
///(for massive mesh number Indirect draw is better)
int main(int argc, char **argv)
{

    osg::ArgumentParser args(&argc,argv);


    args.getApplicationUsage()->setApplicationName(args.getApplicationName());
    args.getApplicationUsage()->setDescription(args.getApplicationName()+" is an example on how to use  bufferobject factorization+basevertex drawing in order to minimize state changes.");
    args.getApplicationUsage()->setCommandLineUsage(args.getApplicationName()+" [options] filename ...");
    args.getApplicationUsage()->addCommandLineOption("--Hmaxsize <factor>","max bufferobject size allowed (hard limit)");
    args.getApplicationUsage()->addCommandLineOption("--Smaxsize <factor>","max bufferobject size allowed (soft limit)");
    args.getApplicationUsage()->addCommandLineOption("--classic","don't use basevertex");
    MakeSharedBufferObjectsVisitor vaovis;
    unsigned int  maxsize;

    while(args.read("--Hmaxsize",maxsize) ) {
        vaovis.setHardBufferSize(maxsize);
    }
    while(args.read("--Smaxsize",maxsize) ) {
        vaovis.setSoftBufferSize(maxsize);
    }
    if(args.read("--classic") ) {
        vaovis.setIsHackActivated(false);
    } else  vaovis.setIsHackActivated(true);
    osg::Node * loaded=osgDB::readNodeFiles(args);
    // create the model
    if(loaded)
    {

        loaded->accept(vaovis);

        osgViewer::Viewer viewer;

        viewer.addEventHandler(new osgViewer::StatsHandler);
        viewer.addEventHandler(new osgViewer::WindowSizeHandler);
        // add model to viewer.
        viewer.setSceneData( loaded );

        return viewer.run();
    }
    args.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);

}
