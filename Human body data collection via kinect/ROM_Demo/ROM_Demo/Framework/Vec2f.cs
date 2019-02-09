using System;
using System.Collections.Generic;

namespace ROM_Demo.Framework {
	class Vec2f {
		private float _x = 0f;
		private float _y = 0f;

		public float X {
			get { return _x; }
			set { _x = value; }
		}
		public float Y {
			get { return _y; }
			set { _y = value; }
		}

		public Vec2f(float x, float y) {
			_y = y;
			_x = x;
		}

		public float Distance() {
			return (float)Math.Sqrt(DistanceSquared());
		}

		public float DistanceSquared() {
			return _x * _x + _y * _y;
		}

		public float Dot(Vec2f cVector) {
			return _x * cVector._x + _y * cVector._y;
		}

		public void Normalize() {
			var dist = Distance();

			_x /= dist;
			_y /= dist;
		}
	}
}
