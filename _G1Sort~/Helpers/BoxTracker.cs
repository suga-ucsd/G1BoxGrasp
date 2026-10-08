using System;
using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Tracks the target box over time.
    ///
    /// - Position : last sampled box position (full 3D).
    /// - Velocity : exponentially filtered HORIZONTAL velocity from position deltas.
    /// - Teleported: set for one frame when the box jumps faster than maxSpeed
    ///   (e.g. a pooled box recycled by the spawner). Velocity is zeroed instead of
    ///   producing a huge bogus value.
    ///
    /// Fed every frame by G1BoxGrasp.UpdateEstimate (BoxModel.cs).
    /// </summary>

    internal sealed class BoxTracker
    {
        public Vector3 Position { get; private set; }
        public Vector3 Velocity { get; private set; }
        public bool Teleported { get; private set; }

        /// <summary>True once at least one position has been received.</summary>
        public bool HasSample => m_Has;

        private Vector3 m_Prev;
        private bool m_Has;

        /// <summary>Forget everything (new target).</summary>
        public void Reset()
        {
            m_Has = false;
            Velocity = new Vector3(0f, 0f, 0f);
            Teleported = false;
        }

        /// <summary>
        /// Feed one position sample.
        /// </summary>
        /// <param name="pos">Box world position this frame.</param>
        /// <param name="dt">Frame time (s).</param>
        /// <param name="filterTime">Velocity filter time constant (s).</param>
        /// <param name="maxSpeed">Speeds above this are treated as a teleport.</param>
        public void Update(Vector3 pos, float dt, float filterTime, float maxSpeed)
        {
            Teleported = false;
            if (!m_Has)
            {
                m_Has = true;
                m_Prev = pos;
                Position = pos;
                Velocity = new Vector3(0f, 0f, 0f);
                return;
            }
            if (dt > 0.00001f)
            {
                Vector3 d = new Vector3(pos.X - m_Prev.X, 0f, pos.Z - m_Prev.Z);
                Vector3 v = d * (1f / dt);
                if (v.Length() > maxSpeed)
                {
                    Teleported = true;
                    Velocity = new Vector3(0f, 0f, 0f);
                }
                else
                {
                    float a = 1f - (float)Math.Exp(-dt / Math.Max(filterTime, 0.001f));
                    Velocity = Velocity + (v - Velocity) * a;
                }
            }
            m_Prev = pos;
            Position = pos;
        }

        /// <summary>Constant-velocity prediction t seconds ahead.</summary>
        public Vector3 Predict(float t) => Position + Velocity * t;
    }
}
