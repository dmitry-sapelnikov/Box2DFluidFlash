package Box2DParticles
{
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.b2AABB;
	import Box2D.Common.Math.b2Mat22;
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
			for (var p_ind:uint = 0; p_ind < N; ++p_ind)
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
			
			InitOrdered();
			
			m_poly.SetAsBox(7, 40);
		}
		
		private function InitOrdered():void
		{
			for (var i:uint = 0; i < N; i++)
			{
				p[i * 2] = ONE+MINIMAL  +  (i % (FIELDSIZE-5)) * ONE + i / 10;
				p[i * 2 + 1] = ONE+MINIMAL + (i / (FIELDSIZE-5)) * ONE; 
				
				op[i * 2] = p[i * 2];
				op[i * 2 + 1] = p[i * 2 + 1];
			}
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
			for (var i:uint = 0; i < N; i++)
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
		
		private function CollideCircle(pos:b2Vec2, r:Number, xf:b2Transform):void
		{		
			var tMat:b2Mat22 = xf.R;
			var cX:Number = xf.position.x + (tMat.col1.x * pos.x + tMat.col2.x * pos.y);
			var cY:Number = xf.position.y + (tMat.col1.y * pos.x + tMat.col2.y * pos.y);
			
			const pR:Number = 0.5 * m_scale;
			const border:Number = 0.0;
			const maxDist:Number = r + border; 
			//const maxDistTol:Number = maxDist - DIST_TOL;
			
			var to_p:b2Vec2 = new b2Vec2();
			for (var i:uint = 0; i < (N << 1); i += 2)
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
					p_f[i] += to_p.x * delta;
					p_f[i + 1] += to_p.y * delta;
				}
			}
		}
		
		private function CollidePolygon(poly:b2PolygonShape, xf:b2Transform):void
		{
			const pR:Number = 0.5 * m_scale;
			const border:Number = 0.0;
			for (var i:uint = 0; i < (N << 1); i += 2)
			{
				const pushDir:b2Vec3 = PointPolyPushDirection(poly, xf, new b2Vec2(p_f[i], p_f[i + 1]), border);
				//	The current particle is inside the polygon
				if (pushDir != null)
				{
					const len:Number = Math.min(pR, (pushDir.z + border) * 0.9);
					p_f[i] += pushDir.x * len;
					p_f[i + 1] += pushDir.y * len;
				}
			}
		}
		
		public function PointPolyPushDirection(poly:b2PolygonShape, xf:b2Transform, p:b2Vec2, r:Number) : b2Vec3
		{
			var tVec:b2Vec2;
			
			//b2Vec2 pLocal = b2MulT(xf.R, p - xf.position);
			var tMat:b2Mat22 = xf.R;
			var tX:Number = p.x - xf.position.x;
			var tY:Number = p.y - xf.position.y;
			var pLocalX:Number = (tX*tMat.col1.x + tY*tMat.col1.y);
			var pLocalY:Number = (tX*tMat.col2.x + tY*tMat.col2.y);
			
			const vertices:Vector.<b2Vec2> = poly.GetVertices();
			const normals:Vector.<b2Vec2> = poly.GetNormals();
			const vertCount:uint = vertices.length;
			
			var pushNormal:b2Vec2 = null;
			var minDepth:Number = -10000000;
			for (var i:int = 0; i < vertCount; ++i)
			{
				//float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
				tVec = vertices[i];
				tX = pLocalX - tVec.x;
				tY = pLocalY - tVec.y;
				tVec = normals[i];
				var dot:Number = (tVec.x * tX + tVec.y * tY);
				if (dot >= r)
				{
					return null;
				}
				else
				{
					if (dot > minDepth)
					{
						pushNormal = normals[i];
						minDepth = dot;
					}
				}
			}
			return new b2Vec3(pushNormal.x, pushNormal.y, -minDepth);
		}
		
		public override function Step(step:b2TimeStep):void
		{
			for (var cell_ind:uint = 0; cell_ind < DEPTH * DEPTH; ++cell_ind)
			{
				c[cell_ind] = 0;
			}
			
			// Solving and broad phase
			var i:uint = 0;
			for (i = 0; i < (N << 1); i += 2)
			{
				/*#ifdef SPD_CONST
				if (int(p[i] - op[i]) > +MAX_SPD) op[i] = p[i] - MAX_SPD; 
				if (int(p[i] - op[i]) < -MAX_SPD) op[i] = p[i] + MAX_SPD; 
				if (int(p[i + 1] - op[i + 1]) > +MAX_SPD) op[i + 1] = p[i + 1] - MAX_SPD; 
				if (int(p[i + 1] - op[i + 1]) < -MAX_SPD) op[i + 1] = p[i + 1] + MAX_SPD;   
				#endif*/
				
				var tmp:int = p[i];
				p[i] += p[i] - op[i];
				op[i] = tmp;
				
				tmp = p[i + 1];
				p[i + 1] += p[i + 1] - op[i + 1] + GRAVITY;
				op[i + 1] = tmp;
				
				if (p[i] < MINIMAL) p[i] = MINIMAL; else if (p[i] > MAXIMAL) p[i] = MAXIMAL;
				if (p[i + 1] < MINIMAL) p[i + 1] = MINIMAL; else if (p[i + 1] > MAXIMAL) p[i + 1] = MAXIMAL; 
				
				//#ifdef SPD_DAMP
				//op[i] += sign_i(p[i] - op[i]) * DAMP_SPD;
				//op[i + 1] += sign_i_spec(p[i + 1] - op[i + 1]) * DAMP_SPD;
				//op[i] = p[i] - int(Number(p[i] - op[i]) * DAMP_SPD_F);
				//op[i + 1] = p[i + 1] - int(Number(p[i + 1] - op[i + 1]) * DAMP_SPD_F);
				//#endif
				
				// broad phase
				tmp = (p[i] >> ONEBITS) + (p[i + 1] >> ONEBITS) * DEPTH;
				
				AddParticleToCell(i, tmp);
			}
			
			updateFloatPositions();
			CollideWithWorld();
			FloatToIntPositions();
			// Narrow phase
			for (i = 0; i < (N << 1); i += 2)
			{
				var glob:uint = (p[i] >> ONEBITS) + (p[i + 1] >> ONEBITS)* DEPTH;
				
				if (c[glob] == 2) // current cell
				{
					//GLOB_IDX_P idx = &g[glob << 1];
					if (g[glob << 1] != i)
						Collide(i, g[glob << 1]); 
					else 
						Collide(i, g[(glob << 1) + 1]);
				}                            
				
				// four cells around
				CollideInCell(i, glob + 1);
				glob += DEPTH; 
				CollideInCell(i, glob);
				CollideInCell(i, glob - 1);
				CollideInCell(i, glob + 1);
			}
		}
		
		private function updateFloatPositions():void
		{
			for (var i:uint = 0; i < N * 2; ++i)
			{
				p_f[i] = p[i] * TOFLOAT * m_scale;
				op_f[i] = op[i] * TOFLOAT * m_scale;
			}
		}
		
		private function FloatToIntPositions():void
		{
			for (var i:uint = 0; i < N * 2; ++i)
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
						CollidePolygon(polyShape, body.GetTransform());
					}
					else
					{
						if (fixture.GetShape() is b2CircleShape)
						{
							var circleShape:b2CircleShape = fixture.GetShape() as b2CircleShape;
							CollideCircle(circleShape.GetLocalPosition(), circleShape.GetRadius(), body.GetTransform());
						}
					}
					fixture = fixture.GetNext();
				}
				bodyEdge = bodyEdge.nextBody;
			}
		}
		
	//	Private constants
		private static const N:uint = 1000;
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
		//CELLSIZE	=	4,
		//CELLSIZE_BYTES= 2,
		private static const DEPTH:int = FIELDSIZE;
		private static const DEPTHQUAD:int = DEPTH * DEPTH;
		private static const GRAVITY:int = 20;
		private static const RESPONSE:int = ONE * 4;
		private static const ATTRACTION:int = 2;
		
		private static const TOFLOAT:Number = 1.0 / Number(ONE);
		private static const DAMP_SPD_F:Number = 0.999999;
		
	//	Private members
		private var p:Vector.<int> = new Vector.<int>(); //p[2 * N]
		private var op:Vector.<int> = new Vector.<int>() //op[2 * N];
			
		private var p_f:Vector.<Number> = new Vector.<Number>(); //p[2 * N]
		private var op_f:Vector.<Number> = new Vector.<Number>(); //p[2 * N]
		
		private var c:Vector.<uint> = new Vector.<uint>();//char c[DEPTH * DEPTH];
		private var g:Vector.<uint> = new Vector.<uint>();//GLOB_IDX g[DEPTH * DEPTH * 2];
		
		private var m_bitmap_data:BitmapData = null;
		
		private var m_poly:b2PolygonShape = new b2PolygonShape();
		private var m_polyTransform:b2Transform = new b2Transform();
		
		private var m_scale:Number = 0.3;
	}
}