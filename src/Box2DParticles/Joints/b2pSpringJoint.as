package Box2DParticles.Joints
{
	import Box2D.Common.Math.b2Vec2;

	public class b2pSpringJoint extends b2pJoint
	{
		public function b2pSpringJoint(particles:Vector.<Number>, _p1Index:uint, _p2Index:uint, _length:Number, _stiffness:Number)
		{
			super(particles);
			p1Index = _p1Index;
			p2Index = _p2Index;
			length = _length;
			stiffness = _stiffness;
		}
		
		public override function Solve():void
		{
			var dir:b2Vec2 = new b2Vec2(
				m_particles[p1Index * 2] - m_particles[p2Index * 2], 
				m_particles[p1Index * 2 + 1] - m_particles[p2Index * 2 + 1]);
			const dist:Number = dir.Length();
			if (dist > Number.MIN_VALUE)
				dir.Multiply(1.0 / dist);
			
			dir.Multiply((dist - length) * stiffness * 0.5);
			
			m_particles[p1Index * 2] -= dir.x;
			m_particles[p1Index * 2 + 1] -= dir.y;
			
			m_particles[p2Index * 2] += dir.x;
			m_particles[p2Index * 2 + 1] += dir.y;
		}
		
		public var p1Index:uint;
		public var p2Index:uint;
		public var length:Number;
		public var stiffness:Number;
	}
}