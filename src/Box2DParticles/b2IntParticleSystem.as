package Box2DParticles
{
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2Color;
	import Box2D.Dynamics.Controllers.b2Controller;
	import Box2D.Dynamics.b2DebugDraw;
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
			}
			
			for (var cell_ind:uint = 0; cell_ind < DEPTH * DEPTH; ++cell_ind)
			{
				c.push(0);
				g.push(0, 0);
			}
			
			InitOrdered();
		}
		
		/*private function InitRand():void
		{
			for (var i:uint = 0; i < N; i++)
			{
				p[i * 2] = Random(FIELD - MINIMAL * 2) + MINIMAL;
				p[i * 2 + 1] = Random(FIELD - MINIMAL * 2) + MINIMAL; 
				
				op[i * 2] = p[i * 2];
				op[i * 2 + 1] = p[i * 2 + 1];
			}
		}*/
		
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
				g[(c[cell]++) + (cell << 1)] = i;
		}
		
		/*__forceinline void CollideInCell(const int i, const int cell)
		{               
			if (c[cell] == 1)
			{
				Collide(i, g[(cell << 1)]);
			}
			else
				if (c[cell] == 2)
				{
					GLOB_IDX_P idx = &g[(cell << 1)];
					Collide(i, idx[0]);
					Collide(i, idx[1]);
				}         
		}*/
		
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
		
/*		__forceinline void Collide(int i, int i2)
		{
			//if (i == i2) return;
			
			int dx = p[i] - p[i2];              
			int dy = p[i + 1] - p[i2 + 1];
			
			int rad = dx * dx - ONEQUAD + dy * dy;
			
			if (rad >= 0) return;
			
			rad = rad / RESPONSE; // ONE * 3
			dx = (dx * rad) >> ONEBITS;
			dy = (dy * rad) >> ONEBITS;
			
			p[i] -= dx; p[i + 1] -= dy;
			p[i2] += dx; p[i2 + 1] += dy;
		}*/
		
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
			const mult:Number = 0.2;
			
			if (m_bitmap_data == null)
			{
				m_bitmap_data = new BitmapData(debugDraw.GetSprite().width, debugDraw.GetSprite().height, true, 0x00ffffff);
				var bitmap:Bitmap = new Bitmap(m_bitmap_data); 
				debugDraw.GetSprite().addChild(bitmap);
			}
			else
			{
				m_bitmap_data.fillRect(new Rectangle(0, 0, m_bitmap_data.width, m_bitmap_data.height), 0x00ffffff);
			}
			
			for (var i:uint = 0; i < N; i++)
			{
				var position:b2Vec2 = new b2Vec2(p[i * 2] * TOFLOAT, p[i * 2 + 1] * TOFLOAT);
				position.Multiply(mult * debugDraw.GetDrawScale());
				m_bitmap_data.setPixel32(Math.round(position.x), Math.round(position.y), 0xff000000); 
				//debugDraw.DrawCircle(position, 0.5 * mult * 0.1, color);
			}
			
			var aabbVert:Array = [
				new b2Vec2(MINIMAL * TOFLOAT * mult, MINIMAL * TOFLOAT * mult),
				new b2Vec2(MAXIMAL * TOFLOAT * mult, MINIMAL * TOFLOAT * mult),
				new b2Vec2(MAXIMAL * TOFLOAT * mult, MAXIMAL * TOFLOAT * mult),
				new b2Vec2(MINIMAL * TOFLOAT * mult, MAXIMAL * TOFLOAT * mult)];
			debugDraw.DrawPolygon(aabbVert, 4, color);
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
				
				if (p[i] < MINIMAL) p[i] = MINIMAL; else if (p[i] > MAXIMAL) p[i] = MAXIMAL;
				
				tmp = p[i + 1];
				p[i + 1] += p[i + 1] - op[i + 1] + GRAVITY;
				op[i + 1] = tmp;
				
				//#ifdef SPD_DAMP
				//op[i] += sign_i(p[i] - op[i]) * DAMP_SPD;
				//op[i + 1] += sign_i_spec(p[i + 1] - op[i + 1]) * DAMP_SPD;
				//op[i] = p[i] - int(Number(p[i] - op[i]) * DAMP_SPD_F);
				//op[i + 1] = p[i + 1] - int(Number(p[i + 1] - op[i + 1]) * DAMP_SPD_F);
				//#endif
				
				if (p[i + 1] < MINIMAL) p[i + 1] = MINIMAL; else if (p[i + 1] > MAXIMAL) p[i + 1] = MAXIMAL; 
				
				// broad phase
				tmp = (p[i] >> ONEBITS) + (p[i + 1] >> ONEBITS) * DEPTH;
				
				AddParticleToCell(i, tmp);
			}
			
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
		
	//	Private constants
		private static const N:uint = 4000;
		private static const ONE:int = 16384;   // 2^14
		private static const ONEBITS:int = 14;
		private static const FIELDSIZE:int = 75;
		private static const FIELD:int = ONE * FIELDSIZE;
		private static const ONEQUAD:int = ONE * ONE;
		private static const MINIMAL:int = ONE * 2;
		private static const MAXIMAL:int = FIELD - MINIMAL;
		private static const MAX_SPD:int = ONE / 4;
		private static const DAMP_SPD:int = 1;
		//CELLSIZE	=	4,
		//CELLSIZE_BYTES= 2,
		private static const DEPTH:int = FIELDSIZE;
		private static const DEPTHQUAD:int = DEPTH * DEPTH;
		private static const GRAVITY:int = 1;
		private static const RESPONSE:int = ONE * 4;
		private static const ATTRACTION:int = 2;
		
		private static const TOFLOAT:Number = 1.0 / Number(ONE);
		private static const DAMP_SPD_F:Number = 0.999999;
		
	//	Private members
		private var p:Vector.<int> = new Vector.<int>(); //p[2 * N]
		private var op:Vector.<int> = new Vector.<int>() //op[2 * N];
		
		private var c:Vector.<uint> = new Vector.<uint>();//char c[DEPTH * DEPTH];
		private var g:Vector.<uint> = new Vector.<uint>();//GLOB_IDX g[DEPTH * DEPTH * 2];
		
		private var m_bitmap_data:BitmapData = null;
	}
}