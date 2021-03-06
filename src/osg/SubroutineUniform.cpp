/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 * Copyright (C) 2016 Julien Valentin
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

#include <osg/SubroutineUniform>
#include <osg/State>

using namespace osg;

void SubroutineUniform::apply(State& state) const
{
    GLExtensions* ext=state.get<GLExtensions>();
    GLuint contextID=state.getContextID();

     if(!_subroutineNames.empty())
    {
        if(_indicesSetted[contextID]==0)
        {
            ///update uniform index (assume program is the LastAppliedProgramObject)
            _indices[contextID].resize(_subroutineNames.size());

            if( !state.getLastAppliedProgramObject()){
                OSG_WARN<<"SubRoutineUniform : lastAppliedProgram is NULL: perhaps you haven't bind a Program to the StateSet"<<std::endl;
            }
            else{
                GLuint GLpo = state.getLastAppliedProgramObject()->getHandle();
                std::vector<GLuint>::iterator percontextroutineindexit=_indices[contextID].begin();
                for(std::vector<std::string>::const_iterator it=_subroutineNames.begin(); it!=_subroutineNames.end(); it++,percontextroutineindexit++)
                {
                    *percontextroutineindexit=
                        ext->glGetSubroutineIndex(GLpo,_shadertype,it->c_str());
                }
                _indicesSetted[contextID]=1;
            }
        }

        ext->glUniformSubroutinesuiv(_shadertype,_subroutineNames.size(),&_indices[contextID].front());
    }
}
bool SubroutineUniform::setShaderType( Shader::Type shadertype){
    if(shadertype == _shadertype) return true;
    ReassignToParents needToReassingToParentsWhenMemberValueChanges(this);
    _shadertype=shadertype;
    return true;
}

SubroutineUniform::~SubroutineUniform() {}
