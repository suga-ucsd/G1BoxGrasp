using System;
using Hazel;

namespace G1Sort
{
	/// <summary>
	/// Small math helpers shared by every G1BoxGrasp file.
	///
	/// Frame convention used everywhere (the engine is Y-up):
	///   yaw  = atan2(-z, x)
	///   left = (fwd.Z, 0, -fwd.X)
	/// "Flat" = projected onto the horizontal XZ plane (Y dropped).
	/// </summary>
	internal static class Geo
	{
		/// <summary>Dot product of two vectors.</summary>
		public static float Dot(Vector3 a, Vector3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

		/// <summary>Absolute value (avoids Math.Abs float/double overload noise).</summary>
		public static float Abs(float v) => v < 0f ? -v : v;

		/// <summary>Drops the vertical component: (x, 0, z).</summary>
		public static Vector3 Flat(Vector3 v) => new Vector3(v.X, 0f, v.Z);

		/// <summary>Clamp to [0, 1]. Hazel's Mathf has no Clamp01.</summary>
		public static float Clamp01(float v) => v < 0f ? 0f : (v > 1f ? 1f : v);

		/// <summary>
		/// Horizontal unit direction of v. Returns <paramref name="fallback"/> if v is
		/// (almost) vertical or zero, so callers never get a NaN direction.
		/// </summary>
		public static Vector3 FlatDir(Vector3 v, Vector3 fallback)
		{
			float len = Flat(v).Length();
			return len > 0.0001f ? new Vector3(v.X / len, 0f, v.Z / len) : fallback;
		}

		/// <summary>Horizontal "left" of a forward direction, in this engine's frame.</summary>
		public static Vector3 LeftOf(Vector3 fwd) => new Vector3(fwd.Z, 0f, -fwd.X);

		/// <summary>Yaw (radians) of a horizontal direction: atan2(-z, x).</summary>
		public static float YawFromDir(Vector3 d) => Mathf.Atan2(-d.Z, d.X);

		/// <summary>Wraps an angle to (-180, 180] degrees.</summary>
		public static float WrapDeg(float deg)
		{
			deg = deg % 360f;
			if (deg > 180f) deg -= 360f;
			if (deg < -180f) deg += 360f;
			return deg;
		}

		/// <summary>Angle interpolation along the shortest path (never the long way round).</summary>
		public static float LerpAngleDeg(float aDeg, float bDeg, float t) => aDeg + WrapDeg(bDeg - aDeg) * t;

		/// <summary>
		/// Moves <paramref name="current"/> toward <paramref name="target"/> by at most
		/// maxRatePerSec * ts. maxRatePerSec &lt;= 0 disables the limit.
		/// </summary>
		public static float Slew(float current, float target, float maxRatePerSec, float ts)
		{
			if (maxRatePerSec <= 0f) return target;
			float maxStep = maxRatePerSec * Math.Max(ts, 0f);
			float diff = target - current;
			if (diff > maxStep) diff = maxStep;
			if (diff < -maxStep) diff = -maxStep;
			return current + diff;
		}

		/// <summary>
		/// Rate-limited write of an actuator position target in DEGREES.
		/// Reads the actuator's current (commanded) value, slews toward the target,
		/// writes back in radians. The only way joints are direct-driven in this script.
		/// </summary>
		public static void WriteJointDeg(MujocoActuator? act, float targetDeg, float maxRateDegPerSec, float ts)
		{
			if (act == null) return;
			float cur = (float)act.Value * Mathf.Rad2Deg;
			act.Value = Slew(cur, targetDeg, maxRateDegPerSec, ts) * Mathf.Deg2Rad;
		}
	}
}
