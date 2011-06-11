/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/


package TestBed{
	
	
	
	import Box2D.Collision.*;
	import Box2D.Collision.Shapes.*;
	import Box2D.Common.*;
	import Box2D.Common.Math.*;
	import Box2D.Dynamics.*;
	import Box2D.Dynamics.Contacts.*;
	import Box2D.Dynamics.Controllers.*;
	import Box2D.Dynamics.Joints.*;
	
	import Box2DParticles.b2IntParticleSystem;
	import Box2DParticles.b2ParticleDescriptor;
	import Box2DParticles.b2ParticleSystem;
	
	
	
	public class TestParticles extends Test
	{
		public function TestParticles()
		{
			var ps_bound:b2AABB = new b2AABB();
			ps_bound.lowerBound.x = 0;
			ps_bound.lowerBound.y = 0;
			ps_bound.upperBound.x = 10;
			ps_bound.upperBound.y = 10;
				
			var particleSystem:b2IntParticleSystem = new b2IntParticleSystem();
			/*const desc:b2ParticleDescriptor = new b2ParticleDescriptor(ps_bound.GetCenter(), 0.1, new b2Vec2(0.0, 0.0));
			for(var y:int = 0; y < 50; y++)
			{
				for(var x:int = -10; x < 10; x++)
				{
					desc.position = ps_bound.GetCenter();
					desc.position.x += x * desc.radius * 2.0;
					desc.position.y += y * desc.radius * 2.0;
					particleSystem.addParticle(desc);
				}
			}*/
			
			m_controller = particleSystem;
			m_world.AddController(m_controller);
			
			// Set Text field
			TestbedMain.m_aboutText.text = "Buoyancy";
		}
		
		public override function Update():void
		{
			super.Update();
		}
		
		private var m_bodies:Array = new Array();
		private var m_controller:b2Controller;
	}
	
}