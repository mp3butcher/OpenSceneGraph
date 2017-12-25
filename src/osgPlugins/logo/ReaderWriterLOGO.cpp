#include <stdio.h>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/BlendFunc>
#include <osg/StateSet>
#include <osg/Notify>
#include <osg/Texture2D>
#include <osg/Viewport>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>

#include <osgUtil/CullVisitor>

using namespace osg;
using namespace osgDB;


class Logos: public osg::Geometry
{
    public:
        enum RelativePosition{
            Center,
            UpperLeft,
            UpperRight,
            LowerLeft,
            LowerRight,
            UpperCenter,
            LowerCenter,
            last_position
        };

        struct logosCullCallback : public osg::DrawableCullCallback
        {
            virtual bool cull(osg::NodeVisitor *visitor, osg::Drawable* drawable, osg::State*) const
            {
                Logos *logos = dynamic_cast<Logos *>(drawable);
                if (!logos) return true;

                osgUtil::CullVisitor *cv = visitor->asCullVisitor();
                if (!cv) return true;

                osg::State* state = cv->getState();

                unsigned int contextID = state!=0 ? state->getContextID() : 0;
                if(contextID != logos->getContextID())
                {
                    // logo not appropriate for window assigned to the cull visitor so cull it.
                    return true;
                }

                osg::Viewport *vp = cv->getViewport();
                if( vp != NULL )
                {
                    if(logos->getViewport()->width()!=vp->width() ||logos->getViewport()->height()!=vp->height()){
                        logos->getViewport()->width()=vp->width() ;
                        logos->getViewport()->height()=vp->height();

                        osg::Vec2f rviewport(1.0f/(float)vp->width(),1.0f/(float)vp->height());
                        osg::Vec2 r(logos->_logos[logos->_messy_pos][0].get()->s()*rviewport.x(),
                                logos->_logos[logos->_messy_pos][0].get()->t()*rviewport.y() );

                        osg::Vec3 corner(-r.x(),-r.y(),0),
                                widthVec(2*r.x(),0,0),
                                heightVec(0,2*r.y(),0);

                        osg::Vec3 center(0,0,0);
                        if(logos->_messy_pos==LowerLeft ||logos->_messy_pos==LowerRight||logos->_messy_pos==LowerCenter)center.y()-=1;
                        if( logos->_messy_pos==UpperLeft ||logos->_messy_pos==UpperRight||logos->_messy_pos==UpperCenter)center.y()+=1;
                        if( logos->_messy_pos==LowerLeft ||logos->_messy_pos==UpperLeft)center.x()-=1;
                        if( logos->_messy_pos==LowerRight ||logos->_messy_pos==UpperRight)center.x()+=1;

                        center.x()*=1.0f-r.x();
                        center.y()*=1.0f-r.y();
                        corner+=center;
                        osg::Vec3Array *coords=(osg::Vec3Array *)logos->getVertexArray();
                        (*coords)[0] = corner+heightVec;
                        (*coords)[1] = corner;
                        (*coords)[2] = corner+widthVec;
                        (*coords)[3] = corner+widthVec+heightVec;
                    }
                }
                return false;
            }
        };

        static osg::ref_ptr<osg::Program> _logoprog;
        Logos()
        {
            setUseDisplayList(false);
            setUseVertexBufferObjects(true);
            Vec3Array* coords = new Vec3Array(4);
            setVertexArray(coords);

            Vec2Array* tcoords = new Vec2Array(4);
            (*tcoords)[0].set(0,1);
            (*tcoords)[1].set(0,0);
            (*tcoords)[2].set(1,0);
            (*tcoords)[3].set(1,1);
            setTexCoordArray(0,tcoords);

            DrawElementsUByte* elems = new DrawElementsUByte(PrimitiveSet::TRIANGLES);
            elems->push_back(0);
            elems->push_back(1);
            elems->push_back(2);

            elems->push_back(2);
            elems->push_back(3);
            elems->push_back(0);
            addPrimitiveSet(elems);

            osg::StateSet *sset = new osg::StateSet;
            osg::BlendFunc *transp = new osg::BlendFunc;
            transp->setFunction(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
            sset->setAttribute( transp );
            sset->setMode( GL_BLEND, osg::StateAttribute::ON );
            sset->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );
            if(!_logoprog.valid()){
                _logoprog=new osg::Program();
                _logoprog->addShader(new osg::Shader(osg::Shader::VERTEX,
                                                     "uniform vec2 rviewport;\n"\
                                                     "uniform vec4 logosize;\n"\
                                                     "void main(){\n"\
                                                     "gl_Position=gl_Vertex;//vec4(gl_Vertex.xy*rviewport-logosize.zw*(vec2(1)-logosize.xy*rviewport),0,1);\n"\
                                                     "gl_TexCoord[0]=gl_MultiTexCoord0;}"));
                _logoprog->addShader(new osg::Shader(osg::Shader::FRAGMENT,"uniform sampler2D tex;\n"\
                                                     "void main(){\n"\
                                                     "gl_FragColor=texture(tex,gl_TexCoord[0].st);\n}"));
            }
            sset->setAttributeAndModes(_logoprog);
            sset->setTextureMode( 0, GL_TEXTURE_2D, osg::StateAttribute::ON );
#if 1
            // for now we'll crudely set the bin number to 100 to force it to draw later and ontop of the scene
            sset->setRenderBinDetails( 100 , "RenderBin" );
#else
            sset->setRenderBinDetails( StateSet::TRANSPARENT_BIN + 1 , "RenderBin" );
#endif
            setStateSet( sset );
            _viewport = new osg::Viewport();//osg::Uniform::FLOAT_VEC2,"rviewport");
            //  sset->addUniform(_viewport);
            setCullCallback( new logosCullCallback );
            _contextID = 0;
        }

        Logos(const Logos& logo, const CopyOp& copyop=CopyOp::SHALLOW_COPY) :
            Geometry( logo, copyop ),
            _contextID(0) {}

        virtual Object* cloneType() const { return new Logos(); }
        virtual Object* clone( const CopyOp& copyop) const { return new Logos(*this, copyop ); }
        virtual bool isSameKindAs(const Object* obj) const { return dynamic_cast<const Logos*>(obj)!=NULL; }
        virtual const char* className() const { return "Logos"; }
#if 0
        //oldstuff
        virtual void drawImplementation(osg::RenderInfo& renderInfo) const
        {
        #if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE) && !defined(OSG_GLES3_AVAILABLE) && !defined(OSG_GL3_AVAILABLE)

            if( renderInfo.getContextID() != _contextID )
                return;


            float vx = 0.0f;
            float vy = 0.0f;
            float vw = 1.0f;
            float vh = 1.0f;
            if (_viewport.valid())
            {
                vx = _viewport->x();
                vy = _viewport->y();
                vw = _viewport->width();
                vh = _viewport->height();
            }

            glMatrixMode( GL_PROJECTION );
            glPushMatrix();
            glLoadIdentity();
            glOrtho( 0.0, vw, 0.0, vh, -1.0, 1.0 );

            glMatrixMode( GL_MODELVIEW );
            glPushMatrix();
            glLoadIdentity();

            glColor4f( 1.0f, 1.0f, 1.0f, 1.0f );

            Images::const_iterator p;
            float th = 0.0;
            for( p = _logos[Center].begin(); p != _logos[Center].end(); p++ )
            {
                th += (*p)->t();
            }

            float place[][4] = {
                { vw*0.5f, ((vh*0.5f) + th*0.5f), -0.5f, -1.0f },
                { vx, vh, 0.0f, -1.0f },
                { vw, vh, -1.0f, -1.0f },
                { vx, vy, 0.0f, 1.0f },
                { vw, vy, -1.0f, 1.0f },
                { vw*0.5f, vh , -0.5f, -1.0f },
                { vw*0.5f, 0.0f , -0.5f, 1.0f },
            };

            for( int i = Center; i < last_position; i++ )
            {
                if( _logos[i].size() != 0 )
                {
                    float x = place[i][0];
                    float y = place[i][1];
                    float xi = place[i][2];
                    float yi = place[i][3];
                    for( p = _logos[i].begin(); p != _logos[i].end(); p++ )
                    {
                        osg::Image *img = (*p).get();
                        glPixelStorei(GL_UNPACK_ALIGNMENT, img->getPacking());
                        glPixelStorei(GL_UNPACK_ROW_LENGTH, img->getRowLength());
                        x = place[i][0] + xi * img->s();
                        if( i == Center || i == UpperLeft || i == UpperRight || i == UpperCenter)
                            y += yi * img->t();
                        glRasterPos2f( x, y );
                        glDrawPixels( img->s(), img->t(), img->getPixelFormat(), img->getDataType(), img->data() );
                        if( i == LowerLeft || i == LowerRight || i == LowerCenter)
                            y += yi * img->t();
                    }
                }
            }

            glPopMatrix();
            glMatrixMode( GL_PROJECTION );
            glPopMatrix();
            glMatrixMode( GL_MODELVIEW );
        #else
            OSG_NOTICE<<"Warning: Logos::drawImplementation(..) not supported."<<std::endl;
        #endif
        }
#endif

      RelativePosition  _messy_pos;

        void addLogo( RelativePosition pos, std::string name )
        {_messy_pos=pos;
            osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile( name.c_str() );
            if( image.valid())
            {
                _logos[pos].push_back( image );

         /*       osg::Vec4f center(image->s(),image->t(),0,0);//default is upperleft
                if( pos==LowerLeft ||pos==LowerRight||pos==LowerCenter)center.w()+=1;
                if( pos==UpperLeft ||pos==UpperRight||pos==UpperCenter)center.w()-=1;
                if( pos==LowerLeft ||pos==UpperLeft)center.z()+=1;
                if( pos==LowerRight ||pos==UpperRight)center.z()-=1;

                createTexturedQuadGeometry(this,osg::Vec3(-image->s(),-image->t(),0),osg::Vec3(2*image->s(),0,0),osg::Vec3(0,2*image->t(),0),0,0,1,1);
            */    getOrCreateStateSet()->setTextureAttribute(0,new osg::Texture2D(image));
            /*     osg::Uniform * logosize=new osg::Uniform("logosize",center);
                 getOrCreateStateSet()->addUniform(logosize);*/
            }
            else
            {
                OSG_WARN<< "Logos::addLogo image file not found : " << name << ".\n";
            }
        }

        osg::Viewport *getViewport() { return _viewport.get(); }

        void setContextID( unsigned int id ) { _contextID = id; }
        unsigned int getContextID() { return _contextID; }

        bool hasLogos()
        {
            int n = 0;
            for( int i = Center; i < last_position; i++ )
                n += _logos[i].size();
            return (n != 0);
        }

        virtual osg::BoundingBox computeBoundingBox() const
        {
            return osg::BoundingBox( -1, -1, -1, 1, 1, 1);
        }

        typedef std::vector < osg::ref_ptr<osg::Image> >  Images;
        Images _logos[last_position];
    protected:
        Logos& operator = (const Logos&) { return *this;}

        virtual ~Logos() {}
    private :

        osg::ref_ptr<osg::Viewport> _viewport;
        unsigned int _contextID;
};

 osg::ref_ptr<osg::Program> Logos::_logoprog=0;

class LOGOReaderWriter : public osgDB::ReaderWriter
{
    public:
        LOGOReaderWriter()
        {
            supportsExtension("logo","Ascii logo placement format");
        }

        virtual const char* className() const { return "Logo Database Reader/Writer"; }

        virtual ReadResult readNode(const std::string& file, const osgDB::ReaderWriter::Options* options) const
        {
            std::string ext = osgDB::getLowerCaseFileExtension(file);
            if (!acceptsExtension(ext)) return ReadResult::FILE_NOT_HANDLED;

            std::string fileName = osgDB::findDataFile( file, options );
            if (fileName.empty())
                return ReadResult::FILE_NOT_FOUND;

            OSG_INFO<< "ReaderWriterLOGO::readNode( "<<fileName.c_str()<<" )\n";

            std::string filePath = osgDB::getFilePath(fileName);
            if (!filePath.empty()) {
                OSG_DEBUG<< "Adding : " << filePath << " to the file data path\n";
                osgDB::getDataFilePathList().push_back(filePath);
            }


            osg::ref_ptr<osg::Geode> geode = new osg::Geode;

            unsigned int screen = 0;

            Logos* ld = new Logos;
            ld->setContextID( screen );

            Logos::RelativePosition pos = Logos::LowerRight;


            std::ifstream fin(fileName.c_str());
            if (!fin) return NULL;

            while(fin)
            {
                std::string str;
                fin >> str;

                if( str == "Center" )
                    pos = Logos::Center;
                else if( str == "UpperLeft" )
                    pos = Logos::UpperLeft;
                else if( str == "UpperRight" )
                    pos = Logos::UpperRight;
                else if( str == "LowerLeft" )
                    pos = Logos::LowerLeft;
                else if( str == "LowerRight" )
                    pos = Logos::LowerRight;
                else if( str == "UpperCenter" )
                    pos = Logos::UpperCenter;
                else if( str == "LowerCenter" )
                    pos = Logos::LowerCenter;
                else if( str == "Camera" )
                {
                    int tn;
                    fin >> tn;
                    if (fin.fail())
                    {
                        OSG_WARN << "Error... Camera requires an integer argument\n";
                        break;
                    }

                    if (tn < 0)
                    {
                        OSG_WARN << "Error... Camera requires an positive or null value argument\n";
                        break;
                    }

                    unsigned int n = static_cast<unsigned int>(tn);
                    if( screen != n )
                    {
                        screen = n;
                        if( ld->hasLogos() )
                        {
                        geode->addDrawable( ld );
                        ld = new Logos;
                        ld->setContextID( screen );
                    }
                    else
                        ld->setContextID( screen );
                    }
                }
                else
                {
                    if( str.length() )
                    ld->addLogo( pos, str );
                }
            }

            if( ld->hasLogos() )
                geode->addDrawable( ld );

            ld->setCullingActive(false);
            return geode;
        }
};


// now register with Registry to instantiate the above
// reader/writer.
REGISTER_OSGPLUGIN(logo, LOGOReaderWriter)
