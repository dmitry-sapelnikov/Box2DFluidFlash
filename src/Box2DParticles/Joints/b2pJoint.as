package Box2DParticles.Joints
{
	public class b2pJoint
	{
		public function b2pJoint(particles:Vector.<Number>):void
		{
			m_particles = particles;
		}
		
		public function Solve():void
		{
		}
		
		protected var m_particles:Vector.<Number> = null;
	}
}