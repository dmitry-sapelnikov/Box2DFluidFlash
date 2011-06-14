package Box2DParticles
{
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.b2AABB;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2Transform;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.Math.b2Vec3;
	import Box2D.Common.b2Color;
	import Box2D.Dynamics.Controllers.b2Controller;
	import Box2D.Dynamics.Controllers.b2ControllerEdge;
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2DebugDraw;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.Dynamics.b2TimeStep;
	
	import flash.display.Bitmap;
	import flash.display.BitmapData;
	import flash.geom.Rectangle;

	public class b2IntParticleSystem extends b2Controller
	{
		public function b2IntParticleSystem()
		{
			for (var p_ind:uint = 0; p_ind < m_particleCount; ++p_ind)
			{
				p.push(0, 0);
				op.push(0, 0);
				
				p_f.push(0.0, 0.0);
				op_f.push(0.0, 0.0);
			}
			
			for (var cell_ind:uint = 0; cell_ind < DEPTH * DEPTH; ++cell_ind)
			{
				c.push(0);
				g.push(0, 0);
			}
		}
		
		public function AddParticle(position:b2Vec2, speed:b2Vec2):void
		{
			p.push(FloatToIntCoord(position.x), FloatToIntCoord(position.y));
			op.push(FloatToIntCoord(position.x - speed.x), FloatToIntCoord(position.y - speed.y));
			
			p_f.push(0.0, 0.0);
			op_f.push(0.0, 0.0);
			
			++m_particleCount;
		}
		
		public function InitOrdered(particleCount:uint):void
		{
			m_particleCount = 0;
			p = new Vector.<int>();
			op = new Vector.<int>();
			
			p_f = new Vector.<Number>();
			op_f = new Vector.<Number>();
			
			var pos:b2Vec2 = new b2Vec2();
			var vel:b2Vec2 = new b2Vec2(0.0, 0.0);
			
			for (var i:uint = 0; i < particleCount; i++)
			{
				pos.x = IntToFloatCoord(ONE+MINIMAL  +  (i % (FIELDSIZE-5)) * ONE + i / 10);
				pos.y = IntToFloatCoord(ONE+MINIMAL + (i / (FIELDSIZE-5)) * ONE);
				AddParticle(pos, vel);
			}
			updateFloatPositions();
		}
		
		public function SetGravity(value:b2Vec2):void
		{
			m_gravity.x = value.x;
			m_gravity.y = value.y;
		}
		
		private function AddParticleToCell(i:uint, cell:uint):void
		{
			if (c[cell] < 2)
			{
				g[(cell << 1) + c[cell]] = i;
				++c[cell];
			}
		}
		
		private function CollideInCell(i:uint, cell:uint):void
		{               
			if (c[cell] == 1)
			{
				Collide(i, g[(cell << 1)]);
			}
			else
				if (c[cell] == 2)
				{
					Collide(i, g[(cell << 1)]);
					Collide(i, g[(cell << 1) + 1]);
				}         
		}
		
		private function Collide(i:uint, i2:uint):void
		{
			//if (i == i2) return;
			
			var dx:int = p[i] - p[i2];              
			var dy:int = p[i + 1] - p[i2 + 1];
			
			var rad:int = dx * dx - ONEQUAD + dy * dy;
			
			if (rad >= 0) 
				return;
			
			rad = rad / RESPONSE;
			dx = (dx * rad) >> ONEBITS;
			dy = (dy * rad) >> ONEBITS;
			
			p[i] -= dx; p[i + 1] -= dy;
			p[i2] += dx; p[i2 + 1] += dy;
		}
		
		public override function Draw(debugDraw:b2DebugDraw):void
		{
			const color:b2Color = new b2Color(0.0, 0.0, 0.0);
			/*if (m_bitmap_data == null)
			{
				m_bitmap_data = new BitmapData(debugDraw.GetSprite().width, debugDraw.GetSprite().height, true, 0x00ffffff);
				var bitmap:Bitmap = new Bitmap(m_bitmap_data); 
				debugDraw.GetSprite().addChild(bitmap);
			}
			else
			{
				m_bitmap_data.fillRect(new Rectangle(0, 0, m_bitmap_data.width, m_bitmap_data.height), 0x00ffffff);
			}*/
			
			const drawScale:Number = debugDraw.GetDrawScale();
			for (var i:uint = 0; i < m_particleCount; i++)
			{
				var position:b2Vec2 = new b2Vec2(p_f[i * 2], p_f[i * 2 + 1]);
				//m_bitmap_data.setPixel32(Math.round(position.x), Math.round(position.y), 0xff000000); 
				debugDraw.DrawCircle(position, 0.5 * m_scale, color);
			}
			
			var aabbVert:Array = [
				new b2Vec2(MINIMAL * TOFLOAT * m_scale, MINIMAL * TOFLOAT * m_scale),
				new b2Vec2(MAXIMAL * TOFLOAT * m_scale, MINIMAL * TOFLOAT * m_scale),
				new b2Vec2(MAXIMAL * TOFLOAT * m_scale, MAXIMAL * TOFLOAT * m_scale),
				new b2Vec2(MINIMAL * TOFLOAT * m_scale, MAXIMAL * TOFLOAT * m_scale)];
			debugDraw.DrawPolygon(aabbVert, 4, color);
		}
		
		private function CollideCircle(body:b2Body, pos:b2Vec2, r:Number, xf:b2Transform):void
		{		
			var tMat:b2Mat22 = xf.R;
			var cX:Number = xf.position.x + (tMat.col1.x * pos.x + tMat.col2.x * pos.y);
			var cY:Number = xf.position.y + (tMat.col1.y * pos.x + tMat.col2.y * pos.y);
			
			const pR:Number = 0.5 * m_scale;
			const border:Number = 0.0;
			const maxDist:Number = r + border; 
			//const maxDistTol:Number = maxDist - DIST_TOL;
			
			var to_p:b2Vec2 = new b2Vec2();
			for (var i:uint = 0; i < (m_particleCount << 1); i += 2)
			{
				to_p.x = p_f[i] - cX;              
				to_p.y = p_f[i + 1] - cY;
				var distSq:Number = to_p.LengthSquared();
				if (distSq < maxDist * maxDist)
				{
					const dist:Number = Math.sqrt(distSq);
					if (dist > Number.MIN_VALUE)
						to_p.Multiply(1.0 / dist);
					const delta:Number = Math.min(pR, (maxDist - dist) * 0.9);
					body.ApplyForce( new b2Vec2(-to_p.x * delta * 3000, -to_p.y * delta * 3000), new b2Vec2(p_f[i], p_f[i+1]) );
					p_f[i] += to_p.x * delta;
					p_f[i + 1] += to_p.y * delta;
				}
			}
		}
		
		private function CollidePolygon(body:b2Body, poly:b2PolygonShape, xf:b2Transform):void
		{
			const pR:Number = 0.5 * m_scale;
			const border:Number = 0.0;
			
			var normals:Vector.<b2Vec2> = poly.GetNormals();
			var verts:Vector.<b2Vec2> = poly.GetVertices();
			var rotatedNormals:Vector.<b2Vec2> = new Vector.<b2Vec2>();
			var edgeDist:Vector.<Number> = new Vector.<Number>();
			
			const normalCount:uint = normals.length;
			for (var ni:uint = 0; ni < normalCount; ++ni)
			{
				var normal:b2Vec2 = normals[ni];
				var vert:b2Vec2 = verts[ni];
				
				var rotatedNormal:b2Vec2 = new b2Vec2(normal.x, normal.y);
				rotatedNormal.MulM(xf.R);
				rotatedNormals.push(rotatedNormal);
				edgeDist.push(b2Math.Dot(vert, normal) + pR);
			}
			
			var center:b2Vec2 = xf.position;
			for (var i:uint = 0; i < (m_particleCount << 1); i += 2)
			{
				var pointLocal:b2Vec2 = new b2Vec2(p_f[i] - center.x, p_f[i + 1] - center.y);
				
				var diffMin:Number = 100000000;
				var pushDir:b2Vec2 = null;
				for (var nri:uint = 0; nri < normalCount; ++nri)
				{
					const diff:Number = edgeDist[nri] - b2Math.Dot(rotatedNormals[nri], pointLocal);
					if (diff <= 0.0)
					{
						pushDir = null;
						break;
					}
					
					if (diff < diffMin)
					{
						diffMin = diff;
						pushDir = rotatedNormals[nri];
					}
				}
				
				if (pushDir != null)
				{
					const len:Number = Math.min(pR * 0.9, diffMin * 0.9);
					body.ApplyForce( new b2Vec2(-pushDir.x * len * 2000, -pushDir.y * len * 2000), new b2Vec2(p_f[i], p_f[i+1]) );
					p_f[i] += pushDir.x * len;
					p_f[i + 1] += pushDir.y * len;
				}
			}
		}
		
		public override function Step(step:b2TimeStep):void
		{
			for (var cell_ind:uint = 0; cell_ind < DEPTH * DEPTH; ++cell_ind)
			{
				c[cell_ind] = 0;
			}
			
			const gravityX:int = Math.round((m_gravity.x * step.dt * step.dt * 0.5) * ONE); 
			const gravityY:int = Math.round((m_gravity.y * step.dt * step.dt * 0.5) * ONE);
			
			// Verlet integration
			var i:uint = 0;
			for (i = 0; i < (m_particleCount << 1); i += 2)
			{
				/*#ifdef SPD_CONST
				if (int(p[i] - op[i]) > +MAX_SPD) op[i] = p[i] - MAX_SPD; 
				if (int(p[i] - op[i]) < -MAX_SPD) op[i] = p[i] + MAX_SPD; 
				if (int(p[i + 1] - op[i + 1]) > +MAX_SPD) op[i + 1] = p[i + 1] - MAX_SPD; 
				if (int(p[i + 1] - op[i + 1]) < -MAX_SPD) op[i + 1] = p[i + 1] + MAX_SPD;   
				#endif*/
				
				var tmp:int = p[i];
				p[i] += p[i] - op[i] + gravityX;
				op[i] = tmp;
				
				tmp = p[i + 1];
				p[i + 1] += p[i + 1] - op[i + 1] + gravityY;
				op[i + 1] = tmp;
				
				//#ifdef SPD_DAMP
				//op[i] += sign_i(p[i] - op[i]) * DAMP_SPD;
				//op[i + 1] += sign_i_spec(p[i + 1] - op[i + 1]) * DAMP_SPD;
				//op[i] = p[i] - int(Number(p[i] - op[i]) * DAMP_SPD_F);
				//op[i + 1] = p[i + 1] - int(Number(p[i + 1] - op[i + 1]) * DAMP_SPD_F);
				//#endif
			}
			
			//	World collision
			updateFloatPositions();
			CollideWithWorld();
			FloatToIntPositions();
			
			//	Clipping and broad phase
			for (i = 0; i < (m_particleCount << 1); i += 2)
			{
				if (p[i] < MINIMAL) p[i] = MINIMAL; else if (p[i] > MAXIMAL) p[i] = MAXIMAL;
				if (p[i + 1] < MINIMAL) p[i + 1] = MINIMAL; else if (p[i + 1] > MAXIMAL) p[i + 1] = MAXIMAL; 
				
				var cell:uint = (p[i] >> ONEBITS) + (p[i + 1] >> ONEBITS) * DEPTH;
				AddParticleToCell(i, cell);
			}
			
			// Narrow phase
			for (i = 0; i < (m_particleCount << 1); i += 2)
			{
				var glob:uint = (p[i] >> ONEBITS) + (p[i + 1] >> ONEBITS)* DEPTH;
				
				if (c[glob] == 2)// current cell
				{
					//GLOB_IDX_P idx = &g[glob << 1];
					if (g[glob << 1] != i)
						Collide(i, g[glob << 1]); 
					else 
						Collide(i, g[(glob << 1) + 1]);
				}                            
				
				// Four cells around
				CollideInCell(i, glob + 1);
				glob += DEPTH; 
				CollideInCell(i, glob);
				CollideInCell(i, glob - 1);
				CollideInCell(i, glob + 1);
			}
		}
		
		private function updateFloatPositions():void
		{
			for (var i:uint = 0; i < m_particleCount * 2; ++i)
			{
				p_f[i] = p[i] * TOFLOAT * m_scale;
				op_f[i] = op[i] * TOFLOAT * m_scale;
			}
		}
		
		private function IntToFloatCoord(x:int):Number
		{
			return x * TOFLOAT * m_scale;
		}
		
		private function FloatToIntCoord(x:Number):int
		{
			return Math.round((x / m_scale) * ONE);
		}
		
		private function FloatToIntPositions():void
		{
			for (var i:uint = 0; i < m_particleCount * 2; ++i)
			{
				p[i] = Math.round((p_f[i] / m_scale) * ONE);
				op[i] = Math.round((op_f[i] / m_scale) * ONE);
			}
		}
		
		private function CollideWithWorld():void
		{
			var bodyEdge:b2ControllerEdge = GetBodyList();
			while (bodyEdge != null)
			{
				var body:b2Body = bodyEdge.body;
				var fixture:b2Fixture = body.GetFixtureList();
				while (fixture != null)
				{
					if (fixture.GetShape() is b2PolygonShape)
					{
						var polyShape:b2PolygonShape = fixture.GetShape() as b2PolygonShape;
						CollidePolygon(body, polyShape, body.GetTransform());
					}
					else
					{
						if (fixture.GetShape() is b2CircleShape)
						{
							var circleShape:b2CircleShape = fixture.GetShape() as b2CircleShape;
							CollideCircle(body, circleShape.GetLocalPosition(), circleShape.GetRadius(), body.GetTransform());
						}
					}
					fixture = fixture.GetNext();
				}
				bodyEdge = bodyEdge.nextBody;
			}
		}
		
	//	Private constants
		private static const ONE:int = 16384;   // 2^14
		private static const ONEBITS:int = 14;
		private static const FIELDSIZE:int = 75;
		private static const FIELD:int = ONE * FIELDSIZE;
		private static const HALF:int = ONE >> 1;
		private static const ONEQUAD:int = ONE * ONE;
		private static const MINIMAL:int = ONE * 2;
		private static const MAXIMAL:int = FIELD - MINIMAL;
		private static const MAX_SPD:int = ONE / 4;
		private static const DAMP_SPD:int = 1;
		private static const DEPTH:int = FIELDSIZE;
		private static const DEPTHQUAD:int = DEPTH * DEPTH;
		private static const RESPONSE:int = ONE * 3;
		private static const ATTRACTION:int = 2;
		
		private static const TOFLOAT:Number = 1.0 / Number(ONE);
		private static const DAMP_SPD_F:Number = 0.999999;
		
	//	Private members
		private var m_particleCount:uint = 0;
		
		private var p:Vector.<int> = new Vector.<int>();
		private var op:Vector.<int> = new Vector.<int>();
			
		private var p_f:Vector.<Number> = new Vector.<Number>();
		private var op_f:Vector.<Number> = new Vector.<Number>();
		
		private var c:Vector.<uint> = new Vector.<uint>();//char c[DEPTH * DEPTH];
		private var g:Vector.<uint> = new Vector.<uint>();//GLOB_IDX g[DEPTH * DEPTH * 2];
		
		private var m_bitmap_data:BitmapData = null;
		
		private var m_scale:Number = 0.3;
		private var m_gravity:b2Vec2 = new b2Vec2(0.0, 1.0);
	}
}