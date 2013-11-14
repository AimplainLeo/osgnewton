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


#ifndef _OSG_NEWTON_BODY_H_
#define _OSG_NEWTON_BODY_H_

#include "osgNewtonStdAfx.h"

namespace osg
{
	class newtonWorld;

	class OSG_NEWTON_API newtonDynamicBody: public dNewtonDynamicBody
	{
		public:
		newtonDynamicBody (newtonWorld* const world, dFloat mass, const dNewtonCollision* const collision, MatrixTransform* const node, const Matrix& location);
		~newtonDynamicBody();
		virtual void OnForceAndTorque (dFloat timestep, int threadIndex);
		
		Matrix GetMatrix() const;
		void SetMatrix (const Matrix& matrix);

		dFloat GetMass() const; 
		Vec4 GetInertia() const ; 

		Vec4 GetCOG () const;
		void SetCOG (const Vec4& com);

		void SetVeloc (const Vec4& veloc);
		Vec4 GetVeloc () const;

		void SetOmega (const Vec4& omega);
		Vec4 GetOmega () const;

		void SetForce (const Vec4& force);
		void SetTorque (const Vec4& torque);

		void AddForce (const Vec4& force);
		void AddTorque (const Vec4& torque);

		Vec4 GetPointVeloc (const Vec4& point) const;
		void ApplyImpulseToDesiredPointVeloc (const Vec4& point, const Vec4& desiredveloc);

		void SetLinearDrag (const dFloat drag);
		void SetAngularDrag (const Vec4& drag);
		
		protected:
		newtonDynamicBody();
	};


	inline dFloat newtonDynamicBody::GetMass() const
	{
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;	

		GetMassAndInertia (mass, Ixx, Iyy, Izz);
		return mass;
	}

	inline Vec4 newtonDynamicBody::GetInertia() const
	{
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;	

		GetMassAndInertia (mass, Ixx, Iyy, Izz);
		return Vec4 (Ixx, Iyy, Izz, mass);
	}

	inline Vec4 newtonDynamicBody::GetCOG () const
	{
		Vec4 cog(0.0f, 0.0f, 0.0f, 0.0f);
		GetCenterOfMass (cog.ptr());
		return cog;
	}

	inline void newtonDynamicBody::SetCOG (const Vec4& cog)
	{
		SetCenterOfMass (cog.ptr());
	}


	inline Matrix newtonDynamicBody::GetMatrix() const
	{
		dMatrix tmp;
		dNewtonDynamicBody::GetMatrix(&tmp[0][0]);
		return Matrix(&tmp[0][0]);
	}

	inline void newtonDynamicBody::SetMatrix (const Matrix& matrix)
	{
		dMatrix tmp (matrix.ptr());
		dNewtonDynamicBody::SetMatrix(&tmp[0][0]);
	}

	inline void newtonDynamicBody::SetVeloc (const Vec4& veloc)
	{
		dVector tmp (veloc.ptr());
		dNewtonDynamicBody::SetVeloc(&tmp[0]);
	}

	inline Vec4 newtonDynamicBody::GetVeloc () const
	{
		dVector tmp;
		dNewtonDynamicBody::GetVeloc(&tmp[0]);
		return Vec4(tmp.m_x, tmp.m_y, tmp.m_z, 0.0f );
	}

	inline void newtonDynamicBody::SetOmega (const Vec4& omega)
	{
		dVector tmp (omega.ptr());
		dNewtonDynamicBody::SetOmega(&tmp[0]);
	}

	inline Vec4 newtonDynamicBody::GetOmega () const
	{
		dVector tmp;
		dNewtonDynamicBody::GetOmega(&tmp[0]);
		return Vec4(tmp.m_x, tmp.m_y, tmp.m_z, 0.0f );
	}

	inline void newtonDynamicBody::SetForce (const Vec4& force)
	{
		dVector tmp (force.ptr());
		dNewtonDynamicBody::SetForce (&tmp[0]);
	}

	inline void newtonDynamicBody::SetTorque (const Vec4& torque)
	{
		dVector tmp (torque.ptr());
		dNewtonDynamicBody::SetTorque (&tmp[0]);
	}

	inline void newtonDynamicBody::AddForce (const Vec4& force)
	{
		dVector tmp (force.ptr());
		dNewtonDynamicBody::AddForce (&tmp[0]);
	}

	inline void newtonDynamicBody::AddTorque (const Vec4& torque)
	{
		dVector tmp (torque.ptr());
		dNewtonDynamicBody::AddTorque (&tmp[0]);
	}


	inline Vec4 newtonDynamicBody::GetPointVeloc (const Vec4& point) const
	{
		dVector veloc;
		dVector tmp(point.ptr());
		dNewtonDynamicBody::GetPointVeloc (&tmp[0], &veloc[0]);
		return Vec4(veloc.m_x, veloc.m_y, veloc.m_z, 0.0f );
	}

	inline void newtonDynamicBody::ApplyImpulseToDesiredPointVeloc (const Vec4& point, const Vec4& desiredveloc)
	{
		dVector tmpPoint(point.ptr());
		dVector tmpVeloc(desiredveloc.ptr());
		dNewtonDynamicBody::ApplyImpulseToDesiredPointVeloc (&tmpPoint[0], &tmpVeloc[0]);
	}

	inline void newtonDynamicBody::SetLinearDrag (const dFloat drag)
	{
		dNewtonDynamicBody::SetLinearDrag (drag);
	}

	inline void newtonDynamicBody::SetAngularDrag (const Vec4& drag)
	{
		dVector tmp (drag.ptr());
		dNewtonDynamicBody::SetAngularDrag (&tmp[0]);
	}

};

#endif
