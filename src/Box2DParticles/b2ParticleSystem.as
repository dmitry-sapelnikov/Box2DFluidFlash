package Box2DParticles
{
	import Box2D.Collision.b2AABB;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2Color;
	import Box2D.Dynamics.Controllers.b2Controller;
	import Box2D.Dynamics.b2DebugDraw;
	import Box2D.Dynamics.b2TimeStep;
	
	import Box2DParticles.b2Particle;
	import Box2DParticles.b2ParticleDescriptor;
	
	import mx.charts.chartClasses.BoundedValue;
	
	public class b2ParticleSystem extends b2Controller
	{
		public function b2ParticleSystem(bounds:b2AABB)
		{
			m_bounds.lowerBound.SetV(bounds.lowerBound);
			m_bounds.upperBound.SetV(bounds.upperBound);
		}
		
		public override function Draw(debugDraw:b2DebugDraw):void
		{
			const color:b2Color = new b2Color(0.5, 0.5, 0.3);
			for each (var particle:b2Particle in m_particles)
			{
				debugDraw.DrawCircle(particle.position, particle.radius, color);
			}
			var aabbVert:Array = [
				new b2Vec2(m_bounds.lowerBound.x, m_bounds.lowerBound.y),
				new b2Vec2(m_bounds.lowerBound.x, m_bounds.upperBound.y),
				new b2Vec2(m_bounds.upperBound.x, m_bounds.upperBound.y),
				new b2Vec2(m_bounds.upperBound.x, m_bounds.lowerBound.y)];
			debugDraw.DrawPolygon(aabbVert, 4, color);
		}
		
		public override function Step(step:b2TimeStep):void
		{
			for each(var particle:b2Particle in m_particles)
			{
				particle.applyAcceleration(GetWorld().GetGravity());
				particle.move(step.dt, m_prevTimeStep);
			}
			
			m_sapQueue.sort(particleSortFunc);
			
			//for(var iter:uint = 0; iter < 1; ++iter)
			//{
				var i:uint = 0;
				var j:uint = 0;
				for(i = 0; i < m_particleCount - 1; ++i)
				{
					var pI:b2Particle = m_sapQueue[i];
					for (j = i + 1; j < m_particleCount; ++j)
					{
						var pJ:b2Particle = m_sapQueue[j];
						if (pJ.position.x - pJ.radius > pI.position.x + pI.radius)
							break;
						Collide(pI, pJ);
					}
				}
				/*for(i = 0; i < m_particleCount - 1; ++i)
				{
					for(j = i + 1; j < m_particleCount; ++j)
					{
						Collide(m_particles[i], m_particles[j]);
					}
				}*/
				
				for each (var part_to_collide:b2Particle in m_particles)
				{
					CollideWithBorders(part_to_collide);
				}
			//}
			m_prevTimeStep = step.dt;
		}
		
		public function addParticle(desc:b2ParticleDescriptor):void
		{
			var particle:b2Particle = new b2Particle(desc);
			m_particles.push(particle);
			m_sapQueue.push(particle);
			m_particleCount++;
		}
		
		public function getParticle(index:uint):b2Particle
		{
			return m_particles[index];
		}
		
		public function get particlesCount():uint
		{
			return m_particleCount;
		}
		
		public function particleSortFunc(p1:b2Particle, p2:b2Particle):int
		{
			if (p1.position.x - p1.radius > p2.position.x - p2.radius) 
				return 1;
			else
				return -1;
		}
		
		private function CollideWithBorders(p:b2Particle):void
		{
			if (p.position.x < m_bounds.lowerBound.x + p.radius)
			{
				p.position.x = m_bounds.lowerBound.x + p.radius;
				if (p.deltaPos.x < 0.0) 
					p.deltaPos.x = Math.abs(p.deltaPos.x) * 0.1;
				
				//p.Push(Vector2(bounds.boxPoint1.x + p.radius, p.pos.y) - p.pos);
			}
			
			if (p.position.x > m_bounds.upperBound.x - p.radius)
			{
				p.position.x = m_bounds.upperBound.x - p.radius;
				if (p.deltaPos.x > 0.0) 
					p.deltaPos.x = -Math.abs(p.deltaPos.x) * 0.1;
				
				//p.Push(Vector2(bounds.boxPoint2.x - p.radius, p.pos.y) - p.pos);
			}
			
			if(p.position.y < m_bounds.lowerBound.y + p.radius)
			{
				p.position.y = m_bounds.lowerBound.y + p.radius;
				if (p.deltaPos.y < 0.0)
					p.deltaPos.y = Math.abs(p.deltaPos.y) * 0.1;
				
				//p.Push(Vector2(p.pos.x, bounds.boxPoint1.y + p.radius) - p.pos);
			}
			if (p.position.y > m_bounds.upperBound.y - p.radius)
			{
				p.position.y = m_bounds.upperBound.y - p.radius;
				if(p.deltaPos.y > 0.0) 
					p.deltaPos.y = -Math.abs(p.deltaPos.y) * 0.1;
				
				//p.Push(Vector2(p.pos.x, bounds.boxPoint2.y - p.radius) - p.pos);
			}
		}
		
		private function Collide(p1:b2Particle, p2:b2Particle):void
		{
			var delta:b2Vec2 = new b2Vec2();
			delta.SetV(p1.position);
			delta.Subtract(p2.position);
			
			
			const quadlen:Number = delta.LengthSquared();
			if (quadlen < (p1.radius + p2.radius) * (p1.radius + p2.radius))
			{
				/*if(quadlen < 1e-6f)
				{
				float ang = random(-pi, pi);
				Vector2 displacement = Vector2(cosf(ang), sinf(ang));
				Vector2 massCenterVelocity = (p1.velocity + p2.velocity) * 0.5f;
				p1.Push(p1.pos + displacement * ( p1.radius));
				p2.Push(p2.pos + displacement * (-p2.radius));
				p1.pos += displacement * ( p1.radius);
				p2.pos -= displacement * ( p2.radius);
				p1.velocity = massCenterVelocity;
				p2.velocity = massCenterVelocity;
				return;
				}*/
				const len:Number = Math.sqrt(quadlen);
				var dir:b2Vec2 = new b2Vec2(delta.x, delta.y);
				dir.Multiply(1.0 / len);
				
				const maxDist:Number = p1.radius + p2.radius;
				var dist:Number;//((p1.radius + p2.radius - len) * 0.5f) * 0.5f;
				var mult:Number = (maxDist - len) * 0.9;
				
				/*if(mult >  maxDist * 0.2f) mult =  maxDist * 0.2f;
				if(mult < -maxDist * 0.2f) mult = -maxDist * 0.2f;*/
				
				//mult = -1;
				
				//const float deltaDepth = 0.5f;
				var ratio:Number = p1.radius / (p1.radius + p2.radius);
				dist = mult/* * 0.9f*/;
				
				/*p1.pos += dir * ( dist);
				p2.pos -= dir * ( dist);*/
				
				var p1_delta:b2Vec2 = new b2Vec2(dir.x, dir.y);
				p1_delta.Multiply(dist * (1.0 - ratio));
				p1.push(p1_delta);
				
				var p2_delta:b2Vec2 = new b2Vec2(dir.x, dir.y);
				p2_delta.Multiply(-dist * ratio);
				p2.push(p2_delta);
				
				/*var bounce:Number = 0.0;
				var relVel:b2Vec2 = new b2Vec2();
				relVel.SetV(p2.deltaPos);
				relVel.Subtract(p1.deltaPos);
				var c:Number = (1.0 + bounce) * (relVel.x * dir.x + relVel.y * dir.y);
				if (c > 0.0)
				{
					p1_delta.SetV(dir);
					p1_delta.Multiply(c * (1.0 - ratio));
					p1.deltaPos.Add(p1_delta);
					
					p2_delta.SetV(dir);
					p2_delta.Multiply(-c * ratio);
					p2.deltaPos.Add(p2_delta);
				}*/
				
				
				/*Vector2 massCenterVelocity = (p1.velocity * displacement + p2.velocity * displacement) * 0.5f;
				p1.velocity = p1.velocity - (p1.velocity * displacement) * displacement + massCenterVelocity;
				p2.velocity = p2.velocity - (p2.velocity * displacement) * displacement + massCenterVelocity;*/
				
				//p1.Push(delta * (-mult) * 0.45f);
				//p2.Push(delta * (+mult) * 0.45f);
			}
		}
		
		private var m_particles:Vector.<b2Particle> = new Vector.<b2Particle>();
		private var m_sapQueue:Vector.<b2Particle> = new Vector.<b2Particle>();
		private var m_particleCount:uint = 0;
		private var m_prevTimeStep:Number = 1e+07;
		private var m_bounds:b2AABB = new b2AABB();
	}
}