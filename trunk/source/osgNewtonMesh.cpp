/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/


#include "osgNewtonStdAfx.h"
#include "osgNewtonMesh.h"
#include "osgNewtonWorld.h"

namespace osg
{

class newtonMesh::MeshPrimitiveIndex: public PrimitiveIndexFunctor
{
	public:
	MeshPrimitiveIndex()
		:PrimitiveIndexFunctor()
		,m_indexList (new UIntArray)
	{
	}
	virtual ~MeshPrimitiveIndex() 
	{
	}

	virtual void setVertexArray(unsigned int count,const Vec2* vertices)
	{
	}
	virtual void setVertexArray(unsigned int count,const Vec3* vertices)
	{
	}
	virtual void setVertexArray(unsigned int count,const Vec4* vertices)
	{
	}

	virtual void setVertexArray(unsigned int count,const Vec2d* vertices)
	{
	}
	virtual void setVertexArray(unsigned int count,const Vec3d* vertices)
	{
	}
	virtual void setVertexArray(unsigned int count,const Vec4d* vertices)
	{
	}

	virtual void drawArrays(GLenum mode, GLint first, GLsizei count)
	{
		switch (mode)
		{
			case PrimitiveSet::TRIANGLES:
			{
				for (int i = 0; i < count; i ++) {
					m_indexList->push_back(first + i);
				}
				break;
			}

			case PrimitiveSet::TRIANGLE_STRIP:
			{
				int r0 = 0;
				int r1 = 1;
				for (int r2 = 2; r2 < count; r2 ++) {
					if (r2 & 1) {
						m_indexList->push_back(first + r0);
						m_indexList->push_back(first + r2);
						m_indexList->push_back(first + r1);
					} else {
						m_indexList->push_back(first + r0);
						m_indexList->push_back(first + r1);
						m_indexList->push_back(first + r2);
					}
					r0 = r1;
					r1 = r2;
				}
				break;
			}

			default:
				dAssert (0);
		}
		
	}
	virtual void drawElements(GLenum mode,GLsizei count,const GLubyte* indices)
	{
		dAssert(0);
	}
	virtual void drawElements(GLenum mode,GLsizei count,const GLushort* indices)
	{
		dAssert(0);
	}
	virtual void drawElements(GLenum mode,GLsizei count,const GLuint* indices)
	{
		dAssert(0);
	}

	virtual void begin(GLenum mode)
	{
		dAssert(0);
	}

	virtual void vertex(unsigned int pos)
	{
		dAssert(0);
	}

	virtual void end()
	{
		dAssert(0);
	}

	ref_ptr<UIntArray> m_indexList;
	
};

class newtonMesh::MeshVertexArrayData: public Drawable::ConstAttributeFunctor 
{
	public:
	MeshVertexArrayData()
		:ConstAttributeFunctor ()
		,m_vertices (new Vec3Array)
		,m_normals (new Vec3Array)
		,m_uv (new Vec2Array)
	{
	}

	void apply(Drawable::AttributeType, unsigned int,const GLbyte*) 
	{
		dAssert(0);
	}

	void apply(Drawable::AttributeType, unsigned int,const GLshort*) 
	{
		dAssert(0);
	}

	void apply(Drawable::AttributeType, unsigned int,const GLint*)
	{
		dAssert(0);
	}

	void apply(Drawable::AttributeType,unsigned int,const GLubyte*)
	{
		dAssert(0);
	}

	void apply(Drawable::AttributeType,unsigned int,const GLushort*)
	{
		dAssert(0);
	}

	void apply(Drawable::AttributeType,unsigned int,const GLuint*)
	{
		dAssert(0);
	}


	void apply(Drawable::AttributeType,unsigned int,const float*)
	{
		dAssert(0);
	}

	void apply (Drawable::AttributeType type, unsigned int count ,const Vec2* vertices)
	{
		switch(type)
		{	
			case Drawable::TEXTURE_COORDS_0:
			{	
				for (unsigned i = 0; i < count; i ++) {
					m_uv->push_back(vertices[i]);
				}
				break;
			}

			default:
				dAssert(0);
		}
	}

	void apply (Drawable::AttributeType type, unsigned int count ,const Vec3* vertices)
	{
		switch(type)
		{	
			case Drawable::VERTICES:
			{	
				for (unsigned i = 0; i < count; i ++) {
					m_vertices->push_back(vertices[i]);
				}
				break;
			}

			case Drawable::NORMALS:
			{	
				for (unsigned i = 0; i < count; i ++) {
					m_normals->push_back(vertices[i]);
				}
				break;
			}

			default:
				dAssert(0);
		}
	}

	void apply(Drawable::AttributeType type, unsigned int,const Vec4*)
	{
		dAssert (type == Drawable::COLORS);
	}

	void apply(Drawable::AttributeType,unsigned int,const Vec4ub*)
	{
		dAssert(0);
	}


	void apply(Drawable::AttributeType,unsigned int,const double*)
	{
		dAssert(0);
	}

	void apply(Drawable::AttributeType,unsigned int,const Vec2d*)
	{
		dAssert(0);
	}

	void apply(Drawable::AttributeType,unsigned int,const Vec3d*)
	{
		dAssert(0);
	}

	void apply(Drawable::AttributeType,unsigned int,const Vec4d*)
	{
		dAssert(0);
	}

	ref_ptr<Vec3Array> m_vertices;
	ref_ptr<Vec3Array> m_normals;
	ref_ptr<Vec2Array> m_uv;
};

class newtonMesh::TraverseNode: public NodeVisitor 
{
	public:
	TraverseNode(newtonMesh* const me, Node* const rootNode)
		:NodeVisitor (NodeVisitor::TRAVERSE_ALL_CHILDREN)
		,m_id(0)
		,m_rootNode(rootNode)
		,m_mesh(me)
	{
	}

	virtual void apply(Geode& node)
	{
		const NodePath& path = getNodePath();

//		Matrix matrix;
//		for (int i = path.size() - 1; (i >= 0) && (path[i] != m_rootNode); i --) {
//			if (path[i]->asTransform()) {
//				MatrixTransform* const transformNode = path[i]->asTransform()->asMatrixTransform();
//				matrix = matrix * transformNode->getMatrix();
//			}
//		}
		Matrix matrix (computeLocalToWorld(getNodePath()));

		int count = node.getNumDrawables();
		for (int i = 0; i < count; i ++) {
			Drawable* const drawable = node.getDrawable(i);
			SubmitFaces (matrix, drawable, m_id + i);
		}
		m_id += count;

		traverse (node);
	}

	void SubmitFaces(const Matrix& matrix, const Drawable* const drawable, int faceId)
	{
		MeshPrimitiveIndex indexPrimitive;
		dAssert (drawable->supports(indexPrimitive));
		drawable->accept(indexPrimitive);

		MeshVertexArrayData data;
		drawable->accept(data);

		Matrix normalMatrix(matrix);
		normalMatrix.setTrans(Vec3(0.0f, 0.0f, 0.0f));

		dFloat triangle[3][12];
		memset (triangle, 0, sizeof (triangle));
		int triangleCount = indexPrimitive.m_indexList->size() / 3; 
		for (int j = 0; j < triangleCount; j ++) {
			for (int i = 0; i < 3; i ++) {
				int index = indexPrimitive.m_indexList->at(j * 3 + i);

				Vec3 p (data.m_vertices->at(index) * matrix);
				triangle[i][0] = p.x();
				triangle[i][1] = p.y();
				triangle[i][2] = p.z();
				
				if (data.m_normals->size() > 0) {
					Vec3 n (data.m_normals->at(index) * normalMatrix);
					triangle[i][4] = n.x();
					triangle[i][5] = n.y();
					triangle[i][6] = n.z();
				}

				if (data.m_uv->size() > 0)  {
					triangle[i][7] = data.m_uv->at(index).x();
					triangle[i][8] = data.m_uv->at(index).y();
				}
			}
			m_mesh->AddFace(3, &triangle[0][0], 12 * sizeof (dFloat), faceId);
		}
	}

	int m_id;
	Node* m_rootNode;
	newtonMesh* m_mesh;
	
};

newtonMesh::newtonMesh (dNewton* const world)
	:dNewtonMesh (world)
	,m_materialMap()
{
}

newtonMesh::newtonMesh (dNewton* const world, Node* const node)
	:dNewtonMesh (world)
	,m_materialMap()
{
	BeginPolygon();
	TraverseNode traversal(this, node);
	node->accept(traversal);
	EndPolygon();
}



newtonMesh::newtonMesh (const dNewtonCollision* const collision)
	:dNewtonMesh (*collision)
{
}

newtonMesh::~newtonMesh()
{
}


int newtonMesh::AddMaterial (ref_ptr<Texture2D> material)
{
	m_materialMap.insert(std::make_pair (m_materialMap.m_enumerator, material));
	m_materialMap.m_enumerator ++;
	return m_materialMap.m_enumerator - 1;
}


Geode* newtonMesh::CreateGeodeNode () const
{
	Geode* const geometryNode = new Geode;	

	int pointCount = GetPointCount();
	int indexCount = GetTotalIndexCount();

	dNewtonScopeBuffer<int> indexList (indexCount);
	dNewtonScopeBuffer<int> remapIndex (indexCount);
	dNewtonScopeBuffer<dNewtonMesh::dPoint> posits (pointCount);
	dNewtonScopeBuffer<dNewtonMesh::dPoint> normals (pointCount);
	dNewtonScopeBuffer<dNewtonMesh::dUV> uv0 (pointCount);
	dNewtonScopeBuffer<dNewtonMesh::dUV> uv1 (pointCount);
	GetVertexStreams(&posits[0], &normals[0], &uv0[0], &uv1[0]);

	void* const materialsHandle = BeginMaterialHandle (); 
	for (int handle = GetMaterialIndex (materialsHandle); handle != -1; handle = GetNextMaterialIndex (materialsHandle, handle)) {

		// get the mesh vertices and triangles data
		int indexCount = MaterialGetIndexCount (materialsHandle, handle); 
		MaterialGetIndexStream (materialsHandle, handle, &indexList[0]); 

		ref_ptr<Vec3Array> geomPositions = new Vec3Array;
		ref_ptr<Vec3Array> geomNormals = new Vec3Array;
		ref_ptr<Vec2Array> geomUV = new Vec2Array;

		int vertexCount = 0;
		memset (&remapIndex[0], 0xff, indexCount * sizeof (remapIndex[0]));
		for( int i = 0; i < indexCount; i ++) {
			int index = indexList[i];
			if (remapIndex[index] == -1) {
				remapIndex[index] = vertexCount;
				geomPositions->push_back(Vec3 (posits[index].m_x, posits[index].m_y, posits[index].m_z));
				geomNormals->push_back(Vec3 (normals[index].m_x, normals[index].m_y, normals[index].m_z));
				geomUV->push_back(Vec2 (uv0[index].m_u, uv0[index].m_v));
				vertexCount ++;
			}
			indexList[i] = remapIndex[index];
		}

		ref_ptr<DrawElementsUInt> geomIndexList = new DrawElementsUInt(GL_TRIANGLES, indexCount);
		for (int i = 0; i < indexCount; i ++) {
			(*geomIndexList)[i] = indexList[i];
		}

		// create a geometry with this vertex data
		ref_ptr<Geometry> geometry = new Geometry;
		geometry->setVertexArray(geomPositions.get());
		geometry->setNormalArray(geomNormals.get());
		geometry->setTexCoordArray(0, geomUV.get());
		geometry->setNormalBinding(Geometry::BIND_PER_VERTEX);
		geometry->addPrimitiveSet(geomIndexList.get());

		// get the material for this drawable
		int materialIndex = MaterialGetMaterial (materialsHandle, handle); 
		MaterialMap::const_iterator materialItr = m_materialMap.find(materialIndex);
		ref_ptr<Texture2D> material = materialItr->second;

		// add this mesh segment to the geode node
		geometryNode->addDrawable(geometry.get());

		// add this material render state
		geometryNode->getOrCreateStateSet()->setTextureAttributeAndModes(0, material.get());
	}
	EndMaterialHandle (materialsHandle); 

	return geometryNode;
}

};


