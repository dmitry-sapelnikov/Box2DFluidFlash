package Box2DParticles
{
	import Box2D.Common.Math.b2Vec2;

	public class b2ParticleDescriptor
	{
		public function b2ParticleDescriptor(_position:b2Vec2, _radius:Number, _velocity:b2Vec2)
		{
			position.SetV(_position);
			radius = _radius;
			velocity.SetV(_velocity);
		}
	
		public var position:b2Vec2 = new b2Vec2();
		public var radius:Number = 0.0;
		public var velocity:b2Vec2 = new b2Vec2();
	}
}