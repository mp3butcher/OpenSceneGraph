/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
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
#include <osg/PrimitiveSet>
#include <osg/BufferObject>
#include <osg/State>
#include <osg/Notify>

using namespace osg;

#define VOA_NOTICE OSG_INFO
//#define VOA_NOTICE OSG_NOTICE

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// PrimitiveSet
//
unsigned int PrimitiveSet::getNumPrimitives() const
{
    switch(_mode)
    {
        case(POINTS): return getNumIndices();
        case(LINES): return getNumIndices()/2;
        case(TRIANGLES): return getNumIndices()/3;
        case(QUADS): return getNumIndices()/4;
        case(LINE_STRIP):
        case(LINE_LOOP):
        case(TRIANGLE_STRIP):
        case(TRIANGLE_FAN):
        case(QUAD_STRIP):
        case(PATCHES):
        case(POLYGON): return (getNumIndices()>0) ? 1 : 0;
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DrawArray
//
void DrawArrays::draw(State& state, bool, GLint vertexbase, bool useVAO) const
{
#if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE)
    GLenum mode = _mode;
    if (_mode==GL_QUADS)
    {
        state.drawQuads(_first, _count, _numInstances);
        return;
    }
    else if (mode==GL_POLYGON)
    {
        mode = GL_TRIANGLE_FAN;
    }
    else if (mode==GL_QUAD_STRIP)
    {
        mode = GL_TRIANGLE_STRIP;
    }

    if (_numInstances>=1) state.glDrawArraysInstanced(mode,_first,_count, _numInstances);
    else glDrawArrays(mode,_first,_count);
#else
    if (_numInstances>=1) state.glDrawArraysInstanced(_mode,_first+vertexbase,_count, _numInstances);
    else glDrawArrays(_mode,_first+vertexbase,_count);

     /*if(useVAO){
                if (_numInstances>=1)    state.get<GLExtensions>()->glDrawArraysInstancedBaseVertex(mode, size(), TYPE, (const GLvoid *)(ebo->getOffset(getBufferIndex())) , _numInstances ,vertexbase );\
                else
                state.get<GLExtensions>()->glDrawArraysBaseVertex(mode, size(), TYPE, (const GLvoid *)(ebo->getOffset(getBufferIndex())) , vertexbase );
            }else {
                if (_numInstances>=1) state.glDrawArraysInstanced(mode, _first,TYPE, (const GLvoid *)(ebo->getOffset(getBufferIndex())), _numInstances);
                else
                glDrawArrays(mode, _first, TYPE, (const GLvoid *)(ebo->getOffset(getBufferIndex())));
            }
*/

#endif
}

void DrawArrays::accept(PrimitiveFunctor& functor) const
{
    functor.drawArrays(_mode,_first,_count);
}

void DrawArrays::accept(PrimitiveIndexFunctor& functor) const
{
    functor.drawArrays(_mode,_first,_count);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DrawArrayLengths
//
unsigned int DrawArrayLengths::getNumPrimitives() const
{
    switch(_mode)
    {
        case(POINTS): return getNumIndices();
        case(LINES): return getNumIndices()/2;
        case(TRIANGLES): return getNumIndices()/3;
        case(QUADS): return getNumIndices()/4;
        case(LINE_STRIP):
        case(LINE_LOOP):
        case(TRIANGLE_STRIP):
        case(TRIANGLE_FAN):
        case(QUAD_STRIP):
        case(PATCHES):
        case(POLYGON): return size();
    }
    return 0;
}

void DrawArrayLengths::draw(State& state, bool, GLint vertexbase, bool useVAO) const
{
    GLenum mode = _mode;
    #if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE)
        if (_mode==GL_QUADS)
        {
            GLint first = _first;
            for(vector_type::const_iterator itr=begin();
                itr!=end();
                ++itr)
            {
                state.drawQuads(first, *itr, _numInstances);
                first += *itr;
            }

            return;
        }
        if (mode==GL_POLYGON) mode = GL_TRIANGLE_FAN;
        if (mode==GL_QUAD_STRIP) mode = GL_TRIANGLE_STRIP;
    #endif

    GLint first = _first;
    for(vector_type::const_iterator itr=begin();
        itr!=end();
        ++itr)
    {
        if (_numInstances>=1) state.glDrawArraysInstanced(mode,first,*itr,_numInstances);
        else glDrawArrays(mode,first,*itr);
        first += *itr;
    }

}

void DrawArrayLengths::accept(PrimitiveFunctor& functor) const
{
    GLint first = _first;
    for(vector_type::const_iterator itr=begin();
        itr!=end();
        ++itr)
    {
        functor.drawArrays(_mode,first,*itr);
        first += *itr;
    }
}

void DrawArrayLengths::accept(PrimitiveIndexFunctor& functor) const
{
    GLint first = _first;
    for(vector_type::const_iterator itr=begin();
        itr!=end();
        ++itr)
    {
        functor.drawArrays(_mode,first,*itr);
        first += *itr;
    }
}

unsigned int DrawArrayLengths::getNumIndices() const
{
    unsigned int count = 0;
    for(vector_type::const_iterator itr=begin();
        itr!=end();
        ++itr)
    {
        count += *itr;
    }
    return count;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DrawElementsUByte
//
DrawElementsUByte::~DrawElementsUByte()
{
    releaseGLObjects();
}
#define DRAWELEMENTS(TYPE) \
 if(useVAO){\
                if (_numInstances>=1)    state.get<GLExtensions>()->glDrawElementsInstancedBaseVertex(mode, size(), TYPE, (const GLvoid *)(ebo->getOffset(getBufferIndex())) , _numInstances ,vertexbase );\
                else \
                state.get<GLExtensions>()->glDrawElementsBaseVertex(mode, size(), TYPE, (const GLvoid *)(ebo->getOffset(getBufferIndex())) , vertexbase );\
            }else {\
                if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(),TYPE, (const GLvoid *)(ebo->getOffset(getBufferIndex())), _numInstances);\
                else\
                glDrawElements(mode, size(), TYPE, (const GLvoid *)(ebo->getOffset(getBufferIndex())));\
            }

void DrawElementsUByte::draw(State& state, bool useVertexBufferObjects, GLint vertexbase, bool useVAO) const
{
    GLenum mode = _mode;
    #if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE)
        if (mode==GL_POLYGON) mode = GL_TRIANGLE_FAN;
        if (mode==GL_QUAD_STRIP) mode = GL_TRIANGLE_STRIP;
    #endif

    if (useVertexBufferObjects)
    {
        GLBufferObject* ebo = getOrCreateGLBufferObject(state.getContextID());
<<<<<<< HEAD
        if(!useVAO)state.bindElementBufferObject(ebo);
        if (ebo)
        {
            DRAWELEMENTS(GL_UNSIGNED_BYTE)
=======

        if (ebo)
        {
            state.getCurrentVertexArrayState()->bindElementBufferObject(ebo);
            if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_BYTE, (const GLvoid *)(ebo->getOffset(getBufferIndex())), _numInstances);
            else glDrawElements(mode, size(), GL_UNSIGNED_BYTE, (const GLvoid *)(ebo->getOffset(getBufferIndex())));
>>>>>>> upstream/master
        }
        else
        {
            state.getCurrentVertexArrayState()->unbindElementBufferObject();
            if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_BYTE, &front(), _numInstances);
            else glDrawElements(mode, size(), GL_UNSIGNED_BYTE, &front());
        }
    }
    else
    {
        if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_BYTE, &front(), _numInstances);
        else glDrawElements(mode, size(), GL_UNSIGNED_BYTE, &front());
    }
}

void DrawElementsUByte::accept(PrimitiveFunctor& functor) const
{
    if (!empty()) functor.drawElements(_mode,size(),&front());
}

void DrawElementsUByte::accept(PrimitiveIndexFunctor& functor) const
{
    if (!empty()) functor.drawElements(_mode,size(),&front());
}

void DrawElementsUByte::offsetIndices(int offset)
{
    for(iterator itr=begin();
        itr!=end();
        ++itr)
    {
        *itr += offset;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DrawElementsUShort
//
DrawElementsUShort::~DrawElementsUShort()
{
    releaseGLObjects();
}

void DrawElementsUShort::draw(State& state, bool useVertexBufferObjects, GLint vertexbase, bool useVAO) const
{
    GLenum mode = _mode;
    #if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE)
        if (mode==GL_POLYGON) mode = GL_TRIANGLE_FAN;
        if (mode==GL_QUAD_STRIP) mode = GL_TRIANGLE_STRIP;
    #endif

    if (useVertexBufferObjects)
    {
        GLBufferObject* ebo = getOrCreateGLBufferObject(state.getContextID());
<<<<<<< HEAD
        if(!useVAO)state.bindElementBufferObject(ebo);
        if (ebo)
        {
           DRAWELEMENTS(GL_UNSIGNED_SHORT)
=======

        if (ebo)
        {
            state.getCurrentVertexArrayState()->bindElementBufferObject(ebo);
            if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_SHORT, (const GLvoid *)(ebo->getOffset(getBufferIndex())), _numInstances);
            else glDrawElements(mode, size(), GL_UNSIGNED_SHORT, (const GLvoid *)(ebo->getOffset(getBufferIndex())));
>>>>>>> upstream/master
        }
        else
        {
            state.getCurrentVertexArrayState()->unbindElementBufferObject();
            if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_SHORT, &front(), _numInstances);
            else glDrawElements(mode, size(), GL_UNSIGNED_SHORT, &front());
        }
    }
    else
    {
        if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_SHORT, &front(), _numInstances);
        else glDrawElements(mode, size(), GL_UNSIGNED_SHORT, &front());
    }
}

void DrawElementsUShort::accept(PrimitiveFunctor& functor) const
{
    if (!empty()) functor.drawElements(_mode,size(),&front());
}

void DrawElementsUShort::accept(PrimitiveIndexFunctor& functor) const
{
    if (!empty()) functor.drawElements(_mode,size(),&front());
}

void DrawElementsUShort::offsetIndices(int offset)
{
    for(iterator itr=begin();
        itr!=end();
        ++itr)
    {
        *itr += offset;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DrawElementsUInt
//
DrawElementsUInt::~DrawElementsUInt()
{
    releaseGLObjects();
}

void DrawElementsUInt::draw(State& state, bool useVertexBufferObjects, GLint vertexbase, bool useVAO) const
{
    GLenum mode = _mode;
    #if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE)
        if (mode==GL_POLYGON) mode = GL_TRIANGLE_FAN;
        if (mode==GL_QUAD_STRIP) mode = GL_TRIANGLE_STRIP;
    #endif

    if (useVertexBufferObjects)
    {
        GLBufferObject* ebo = getOrCreateGLBufferObject(state.getContextID());
<<<<<<< HEAD
        if(!useVAO)state.bindElementBufferObject(ebo);
        if (ebo)
        {

            DRAWELEMENTS(GL_UNSIGNED_INT)
=======

        if (ebo)
        {
            state.getCurrentVertexArrayState()->bindElementBufferObject(ebo);
            if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_INT, (const GLvoid *)(ebo->getOffset(getBufferIndex())), _numInstances);
            else glDrawElements(mode, size(), GL_UNSIGNED_INT, (const GLvoid *)(ebo->getOffset(getBufferIndex())));
>>>>>>> upstream/master
        }
        else
        {
            state.getCurrentVertexArrayState()->unbindElementBufferObject();
            if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_INT, &front(), _numInstances);
            else glDrawElements(mode, size(), GL_UNSIGNED_INT, &front());
        }
    }
    else
    {
        if (_numInstances>=1) state.glDrawElementsInstanced(mode, size(), GL_UNSIGNED_INT, &front(), _numInstances);
        else glDrawElements(mode, size(), GL_UNSIGNED_INT, &front());
    }
}

void DrawElementsUInt::accept(PrimitiveFunctor& functor) const
{
    if (!empty()) functor.drawElements(_mode,size(),&front());
}

void DrawElementsUInt::accept(PrimitiveIndexFunctor& functor) const
{
    if (!empty()) functor.drawElements(_mode,size(),&front());
}

void DrawElementsUInt::offsetIndices(int offset)
{
    for(iterator itr=begin();
        itr!=end();
        ++itr)
    {
        *itr += offset;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MultiDrawArrays
//
#ifdef OSG_HAS_MULTIDRAWARRAYS
void MultiDrawArrays::draw(osg::State& state, bool, GLint vertexbase, bool useVAO) const
{
    // VOA_NOTICE<<"osg::MultiDrawArrays::draw"<<std::endl;

    GLExtensions* ext = state.get<GLExtensions>();
    if (ext->glMultiDrawArrays)
    {
        GLsizei primcount = osg::minimum(_firsts.size(), _counts.size());

        ext->glMultiDrawArrays(_mode, &_firsts.front(), &_counts.front(), primcount);
    }
}

void MultiDrawArrays::accept(PrimitiveFunctor& functor) const
{
    unsigned int primcount = osg::minimum(_firsts.size(), _counts.size());
    for(unsigned int i=0; i<primcount; ++i)
    {
        functor.drawArrays(_mode, _firsts[i], _counts[i]);
    }
}

void MultiDrawArrays::accept(PrimitiveIndexFunctor& functor) const
{
    unsigned int primcount = osg::minimum(_firsts.size(), _counts.size());
    for(unsigned int i=0; i<primcount; ++i)
    {
        functor.drawArrays(_mode, _firsts[i], _counts[i]);
    }
}

unsigned int MultiDrawArrays::getNumIndices() const
{
    unsigned int total=0;
    for(Counts::const_iterator itr = _counts.begin(); itr!=_counts.end(); ++itr)
    {
        total += *itr;
    }
    return total;
}

unsigned int MultiDrawArrays::index(unsigned int pos) const
{
    unsigned int i;
    for(i=0; i<_counts.size(); ++i)
    {
        unsigned int count = _counts[i];
        if (pos<count) break;
        pos -= count;
    }
    if (i>=_firsts.size()) return 0;

    return _firsts[i] + pos;
}

void MultiDrawArrays::offsetIndices(int offset)
{
    for(Firsts::iterator itr = _firsts.begin(); itr!=_firsts.end(); ++itr)
    {
        *itr += offset;
    }
}

unsigned int MultiDrawArrays::getNumPrimitives() const
{
    switch(_mode)
    {
        case(POINTS): return getNumIndices();
        case(LINES): return getNumIndices()/2;
        case(TRIANGLES): return getNumIndices()/3;
        case(QUADS): return getNumIndices()/4;
        case(LINE_STRIP):
        case(LINE_LOOP):
        case(TRIANGLE_STRIP):
        case(TRIANGLE_FAN):
        case(QUAD_STRIP):
        case(PATCHES):
        case(POLYGON):
        {
            unsigned int primcount = osg::minimum(_firsts.size(), _counts.size());
            return primcount;
        }
    }
    return 0;
}

void MultiDrawArrays::add(GLint first, GLsizei count)
{
    _firsts.push_back(first);
    _counts.push_back(count);
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MultiDrawElements
//

#ifdef OSG_HAS_MULTIDRAWARRAYS

void MultiDrawElementsUShort::draw(osg::State& state, bool, GLint vertexbase, bool useVAO) const
{
    // OSG_NOTICE<<"osg::MultiDrawArrays::draw"<<std::endl;

    GLExtensions* ext = state.get<GLExtensions>();
    if (ext->glMultiDrawElements&&!_indicesholders.empty())
    {

        GLBufferObject* ebo = getOrCreateGLBufferObject(state.getContextID());

        if(!useVAO)
            state.bindElementBufferObject(ebo);

        if(_indices == NULL && ebo){ ///_indices invalidated
            _indices=new GLushort*[_indicesholders.size()];
            for (unsigned i =0; i <  _indicesholders.size(); i++)
                _indices[i]=(GLushort*)(ebo->getOffset(_indicesholders[i]->getBufferIndex()));
        }
         if(useVAO){/**TODO*/}//  ext->glMultiDrawElementsBaseVertex(_mode, &_counts.front(), GL_UNSIGNED_SHORT, (const GLvoid *const*)_indices,_indicesholders.size(),vertexbase);
       else ext->glMultiDrawElements(_mode, &_counts.front(), GL_UNSIGNED_SHORT, (const GLvoid *const*)_indices,_indicesholders.size());
    }
}

void MultiDrawElementsUShort::invalidateInnerIndices(){
        if(_indices != NULL){
            delete [] _indices;
            _indices=NULL;
        }
        _counts.clear();
        for(std::vector<ref_ptr<UShortArray> >::iterator it=_indicesholders.begin();it!=_indicesholders.end();it++)
            _counts.push_back((*it)->size());
}

unsigned int MultiDrawElementsUShort::index(unsigned int index) const
{
    unsigned int globalindex=0, current=0;
    while(current<getNumIndicesArrays() && globalindex+getIndicesArray(current)->getNumElements()<index  )
        globalindex+=getIndicesArray(current++)->getNumElements();
    return (*_indicesholders[current].get())[index-globalindex];
}


void MultiDrawElementsUShort::offsetIndices(int offset)
{
    for(std::vector<ref_ptr<UShortArray> >::iterator it=_indicesholders.begin();it!=_indicesholders.end();it++)
    {
        for(UShortArray::iterator itindex=(*(*it).get()).begin();itindex!=(*(*it).get()).end();itindex++ )
            *itindex+=offset;
    }

}

unsigned int MultiDrawElementsUShort::getNumPrimitives() const
{
    switch(_mode)
    {
        case(POINTS): return getNumIndices();
        case(LINES): return getNumIndices()/2;
        case(TRIANGLES): return getNumIndices()/3;
        case(QUADS): return getNumIndices()/4;
        case(LINE_STRIP):
        case(LINE_LOOP):
        case(TRIANGLE_STRIP):
        case(TRIANGLE_FAN):
        case(QUAD_STRIP):
        case(PATCHES):
        case(POLYGON):
        {
            unsigned int primcount = 0;
            for(Counts::const_iterator itr = _counts.begin(); itr!=_counts.end(); ++itr)
                primcount+=*itr;
            return primcount;
        }
    }
    return 0;
}


void MultiDrawElementsUShort::accept(PrimitiveFunctor& functor) const
{
    for(unsigned int i=0; i< _counts.size(); ++i)
    {
        functor.drawElements(_mode,_counts[i],&_indicesholders[i]->front());
    }
}

void MultiDrawElementsUShort::accept(PrimitiveIndexFunctor& functor) const
{
    for(unsigned int i=0; i< _counts.size(); ++i)
    {
        functor.drawElements(_mode,_counts[i],&_indicesholders[i]->front());
    }
}


#endif
