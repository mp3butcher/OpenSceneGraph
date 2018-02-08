/*  -*-c++-*-
 *  Copyright (C) 2017 Julien Valentin <mp3butcher@hotmail.com>
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

#include <osg/TextureView>

using namespace osg;
#define TEXTURE_VIEW_MIN_LEVEL                          0x82DB
#define TEXTURE_VIEW_NUM_LEVELS                         0x82DC
#define TEXTURE_VIEW_MIN_LAYER                          0x82DD
#define TEXTURE_VIEW_NUM_LAYERS                         0x82DE
#define TEXTURE_IMMUTABLE_LEVELS                        0x82DF

TextureView::TextureView(): Texture(),
    _target(GL_NONE),
    _minlayer(0),
    _numlayers(0),
    _parentTexture(0)

{
    _PCdirtyflags.setAllElementsTo(true);
    _PCTextureView.setAllElementsTo(0);
}
TextureView::TextureView(const TextureView& text, const CopyOp &copyop ) : Texture(text,copyop),
    _target(text._target),
    _parentTexture(text._parentTexture)
{
    _PCdirtyflags.setAllElementsTo(true);
    _PCTextureView.setAllElementsTo(0);
}
int TextureView::compare(const StateAttribute&sa) const
{
    COMPARE_StateAttribute_Types(TextureView,sa)
    COMPARE_StateAttribute_Parameter(_target)
    COMPARE_StateAttribute_Parameter(_parentTexture)
    COMPARE_StateAttribute_Parameter(_minlayer)
    COMPARE_StateAttribute_Parameter(_numlayers)
    return 0;
}

void TextureView::compileGLObjects(State& state) const
{

    apply(state);
}

void TextureView::applyTextureViewParameters(GLenum target, State& state) const
{
    // get the contextID (user defined ID of 0 upwards) for the
    // current OpenGL context.
    const unsigned int contextID = state.getContextID();
    const GLExtensions* extensions = state.get<GLExtensions>();

    TextureObject* to = getTextureObject(contextID);

    glTexParameterf(target, TEXTURE_VIEW_MIN_LEVEL, _minlod);
    glTexParameterf(target, TEXTURE_VIEW_NUM_LEVELS, (_maxlod-_minlod)>0?_maxlod-_minlod+1:1);
    glTexParameterf(target, TEXTURE_VIEW_MIN_LAYER, _minlayer);
    glTexParameterf(target, TEXTURE_VIEW_NUM_LAYERS, _numlayers);


}
/** bind TextureViewObject **/
void TextureView::apply(State&state) const
{

    /// TODO same test for all textures type
    //state.setReportGLErrors(true);

    // get the contextID (user defined ID of 0 upwards) for the
    // current OpenGL context.
    const unsigned int contextID = state.getContextID();

    // check parent texture dirtiness
     Texture::TextureObject * parenttextureObject = _parentTexture->getTextureObject(contextID);
    if(!parenttextureObject && true/*todo check texture data dirtiness*/){
        //_parentTexture->apply(state);
        state.applyTextureAttribute(state.getActiveTextureUnit(),_parentTexture);
    }
parenttextureObject->setAllocated(true);
    Texture::TextureObject * textureObject = _textureObjectBuffer[contextID];

  /*  if (textureObject)
    {
        bool textureObjectInvalidated = false;
        if (_image.valid() && getModifiedCount(contextID) != _image->getModifiedCount())
        {
            textureObjectInvalidated = !textureObjectValid(state);
        }

        if (textureObjectInvalidated)
        {
            // OSG_NOTICE<<"Discarding TextureObject"<<std::endl;
            textureObject->release();
            to->setTextureObject(contextID,0);
          //  _textureObjectBuffer[contextID]->release();
           // _textureObjectBuffer[contextID] = 0;
            textureObject = 0;
        }
    }
*/
    if (textureObject)
    {
        textureObject->bind();

        if (getTextureParameterDirty(state.getContextID()))
            applyTexParameters(_target,state);
}else if ((_internalFormat!=GL_NONE) )
    {
        GLuint err;

        textureObject = generateAndAssignTextureObject(contextID,_target);//,0,_internalFormat,_parentTexture->getTextureWidth(),_parentTexture->getTextureHeight(),1,_borderWidth);//,_numMipmapLevels,_internalFormat,_textureWidth,_textureHeight,1,_borderWidth);
       // textureObject->setAllocated(0,_internalFormat,_parentTexture->getTextureWidth(),_parentTexture->getTextureHeight(),1,_borderWidth);
        err=glGetError();
        if(err!=GL_NO_ERROR){
            OSG_WARN<<err<<std::endl;
        }


        if(state.get<GLExtensions>()->glTextureView)
            state.get<GLExtensions>()->glTextureView(textureObject->id(), _target,  parenttextureObject->id(), _internalFormat,
                          _minlod,(_maxlod-_minlod)>0?_maxlod-_minlod+1:1,_minlayer,_numlayers);
        err=glGetError();
        if(err!=GL_NO_ERROR){
               OSG_WARN<<err<<std::endl;
            }
        textureObject->bind();
        err=glGetError();
        if(err!=GL_NO_ERROR){
           OSG_WARN<<err<<std::endl;
        }
        applyTextureViewParameters(_target,state);
        _textureObjectBuffer[contextID]=textureObject;

       /* if (_readPBuffer.valid())
        {
            _readPBuffer->bindPBufferToTexture(GL_FRONT);
        }*/

    }
    else
    {
    //    glBindTexture( _target, 0 );
    }


}
/*
void TextureView::releaseGLObjects(State* state) const
{
    if(state)
    {
        if (state->get<GLExtensions>()->glDeleteTextureViews==0) return;

        unsigned int contextID=state->getContextID();
        state->get<GLExtensions>()->glDeleteTextureObs(1,&_PCTextureView[contextID]);
    }
}
*/
