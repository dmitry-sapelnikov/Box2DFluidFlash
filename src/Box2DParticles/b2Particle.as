package Box2DParticles
{
	import Box2D.Common.Math.b2Vec2;
	
	public class b2Particle
	{
		public function b2Particle(desc:b2ParticleDescriptor)
		{
			position.SetV(desc.position);
			deltaPos.SetV(desc.velocity);
			radius = desc.radius;
		}
		
		public function push(delta:b2Vec2):void
		{
			deltaPos.Add(delta);
			position.Add(delta);
		}

		public function move(timeStep:Number, prevTimeStep:Number):void
		{
			var delta:b2Vec2 = new b2Vec2();
			delta.SetV(m_acceleration);
			delta.Multiply(timeStep * timeStep / 2.0);
			deltaPos.Add(delta);
			deltaPos.Multiply(timeStep / prevTimeStep);
			
			if (deltaPos.LengthSquared() > (MAX_VELOCITY * timeStep) * (MAX_VELOCITY * timeStep))
			{
				deltaPos.Normalize();
				deltaPos.Multiply(MAX_VELOCITY * timeStep);
			}
			deltaPos.Multiply(DELTA_DAMPING);
			
			position.Add(deltaPos);
			m_acceleration.SetZero();
		}
		
		public function applyAcceleration(acc:b2Vec2):void
		{
			m_acceleration.Add(acc);
		}

		private static const MAX_VELOCITY:Number = 10.0;
		private static const DELTA_DAMPING:Number = 0.999;
		
		private var m_acceleration:b2Vec2 = new b2Vec2(0.0, 0.0);
		
		public var deltaPos:b2Vec2 = new b2Vec2(0.0, 0.0);
		public var position:b2Vec2 = new b2Vec2(0.0, 0.0);
		public var radius:Number = 0.0; 
	}
}