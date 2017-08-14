/*  -*-c++-*-
 *  Copyright (C) 2009 Cedric Pinson <cedric.pinson@plopbyte.net>
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


#include <osgAnimation/VertexInfluence>
#include <osgAnimation/MorphTransformSoftware>
#include <osgAnimation/BoneMapVisitor>
#include <osgAnimation/MorphGeometry>

using namespace osgAnimation;


void MorphTransformSoftware::operator()(MorphGeometry& morphGeometry)
{

    if (morphGeometry.isDirty())
    {
        // See if we have an internal optimized geometry

        osg::Vec3Array* pos = dynamic_cast<osg::Vec3Array*>(morphGeometry.getVertexArray());
        osg::Vec3Array & vertexSource = *(morphGeometry.getVertexSource());
        osg::Vec3Array& normalSource = *(morphGeometry.getNormalSource());

        if(pos)
        {
            if ( vertexSource.size() != pos->size())
            {
                vertexSource =*(static_cast<osg::Vec3Array*>( pos->clone(osg::CopyOp::DEEP_COPY_ARRAYS)));//osg::Vec3Array(pos->begin(),pos->end());
                pos->setDataVariance(osg::Object::DYNAMIC);
            }

            osg::Vec3Array* normal = dynamic_cast<osg::Vec3Array*>(morphGeometry.getNormalArray());
            bool normalmorphable = morphGeometry.getMorphNormals() && normal;
            if (normal && normalSource.size() != normal->size())
            {
                normalSource =*(static_cast<osg::Vec3Array*>( normal->clone(osg::CopyOp::DEEP_COPY_ARRAYS)));//osg::Vec3Array(normal->begin(),normal->end());
                normal->setDataVariance(osg::Object::DYNAMIC);
            }


            if (!vertexSource.empty())
            {
                bool initialized = false;
                if (morphGeometry.getMethod() == MorphGeometry::NORMALIZED)
                {
                    // base * 1 - (sum of weights) + sum of (weight * target)
                    float baseWeight = 0;
                    for (unsigned int i=0; i < morphGeometry.getMorphTargetList().size(); i++)
                    {
                        baseWeight +=  morphGeometry.getMorphTarget(i).getWeight();
                    }
                    baseWeight = 1 - baseWeight;

                    if (baseWeight != 0)
                    {
                        initialized = true;
                        for (unsigned int i=0; i < pos->size(); i++)
                        {
                            (*pos)[i] = vertexSource[i] * baseWeight;
                        }
                        if (normalmorphable)
                        {
                            for (unsigned int i=0; i < normal->size(); i++)
                            {
                                (*normal)[i] = normalSource[i] * baseWeight;
                            }
                        }
                    }
                }
                else //if (_method == RELATIVE)
                {
                    // base + sum of (weight * target)
                    initialized = true;
                    for (unsigned int i=0; i < pos->size(); i++)
                    {
                        (*pos)[i] = vertexSource[i];
                    }
                    if (normalmorphable)
                    {
                        for (unsigned int i=0; i < normal->size(); i++)
                        {
                            (*normal)[i] = normalSource[i];
                        }
                    }
                }

                for (unsigned int i=0; i <  morphGeometry.getMorphTargetList().size(); i++)
                {
                    if (morphGeometry.getMorphTarget(i).getWeight() > 0)
                    {
                        // See if any the targets use the internal optimized geometry
                        osg::Geometry* targetGeometry =  morphGeometry.getMorphTarget(i).getGeometry();

                        osg::Vec3Array* targetPos = dynamic_cast<osg::Vec3Array*>(targetGeometry->getVertexArray());
                        osg::Vec3Array* targetNormals = dynamic_cast<osg::Vec3Array*>(targetGeometry->getNormalArray());
                        normalmorphable = normalmorphable && targetNormals;
                        if(targetPos)
                        {
                            if (initialized)
                            {
                                // If vertices are initialized, add the morphtargets
                                for (unsigned int j=0; j < pos->size(); j++)
                                {
                                    (*pos)[j] += (*targetPos)[j] *  morphGeometry.getMorphTarget(i).getWeight();
                                }

                                if (normalmorphable)
                                {
                                    for (unsigned int j=0; j < normal->size(); j++)
                                    {
                                        (*normal)[j] += (*targetNormals)[j] * morphGeometry.getMorphTarget(i).getWeight();
                                    }
                                }
                            }
                            else
                            {
                                // If not initialized, initialize with this morph target
                                initialized = true;
                                for (unsigned int j=0; j < pos->size(); j++)
                                {
                                    (*pos)[j] = (*targetPos)[j] * morphGeometry.getMorphTarget(i).getWeight();
                                }

                                if (normalmorphable)
                                {
                                    for (unsigned int j=0; j < normal->size(); j++)
                                    {
                                        (*normal)[j] = (*targetNormals)[j] * morphGeometry.getMorphTarget(i).getWeight();
                                    }
                                }
                            }
                        }
                    }
                }

                pos->dirty();
                if (normalmorphable)
                {
                    for (unsigned int j=0; j < normal->size(); j++)
                    {
                        (*normal)[j].normalize();
                    }
                    normal->dirty();
                }
            }

            morphGeometry.dirtyBound();

        }
        morphGeometry.dirty(false);
    }

}
