package Box2DParticles.Joints
{
	import Box2D.Common.Math.b2Vec2;
	
	public class b2pPressureJoint extends b2pJoint
	{
		public function b2pPressureJoint(particles:Vector.<Number>, _pointIndices:Vector.<uint>, _pressure:Number)
		{
			super(particles);
			m_pointIndices = _pointIndices.slice();
			m_pressure = _pressure;
			m_original_volume = GetVolume();
		}
		
		public override function Solve():void
		{
			const indCount:uint = m_pointIndices.length;
			var offset:Number = 0.001 * (m_pressure * m_original_volume - GetVolume()) / 3.0;
			
			var offsetDirections:Vector.<b2Vec2> = new Vector.<b2Vec2>();
			for (var offsetInd:uint = 0; offsetInd < indCount; ++offsetInd)
			{
				var offsetDir:b2Vec2 = new b2Vec2();
				const i1:uint = m_pointIndices[offsetInd] * 2;
				const i2:uint = m_pointIndices[(offsetInd + 1) % indCount] * 2;
				const i3:uint = m_pointIndices[(offsetInd + 2) % indCount] * 2;
				
				//	i1-i2 points normal
				offsetDir.x = m_particles[i2 + 1] - m_particles[i1 + 1];
				offsetDir.y = m_particles[i1] - m_particles[i2];
			
				//	i2-i3 points normal
				const dx2:Number = m_particles[i3 + 1] - m_particles[i2 + 1];
				const dy2:Number = m_particles[i2] - m_particles[i3];
				
				offsetDir.x += dx2;
				offsetDir.y += dy2;
				
				const offsetLenSq:Number = offsetDir.LengthSquared();
				if (offsetLenSq > Number.MIN_VALUE)
					offsetDir.Multiply(1.0 / offsetLenSq);
				offsetDirections.push(offsetDir);
			}
			
			for (var particleInd:uint = 0; particleInd < indCount; ++particleInd)
			{
				const pi:uint = m_pointIndices[(particleInd + 1) % indCount] * 2;
				
				m_particles[pi] += offset * offsetDirections[particleInd].x;
				m_particles[pi + 1] += offset * offsetDirections[particleInd].y;
			}
		}
		
		private function GetVolume():Number
		{
			var volume:Number = 0.0;
			const indCount:uint = m_pointIndices.length;
			for (var ind:uint = 0; ind < indCount; ++ind)
			{
				const i1:uint = m_pointIndices[ind] * 2;
				const i2:uint = m_pointIndices[(ind + 1) % indCount] * 2;
				
				const dx:Number = m_particles[i1] - m_particles[i2];
				const dy:Number = m_particles[i1 + 1] - m_particles[i2 + 1];
				volume += (-m_particles[i1] * dy + m_particles[i1 + 1] * dx) * 0.5;  
			}
			return volume;
		}
		
		private var m_pointIndices:Vector.<uint> = null;
		private var m_pressure:Number;
		private var m_original_volume:Number;
	}
}
