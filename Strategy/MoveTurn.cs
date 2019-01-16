using System.Numerics;
using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeBall2018.Strategy
{
    internal class MoveTurn : ITurn
    {
        private readonly double _VelX;
        private readonly double _VelY;
        private readonly bool _UseNitro;

        public MoveTurn(double velX, double velY, bool useNitro)
        {
            _VelX = velX;
            _VelY = velY;
            _UseNitro = useNitro;
        }

        public MoveTurn(Vector2 vel, bool useNitro)
        {
            _VelX = vel.X;
            _VelY = vel.Y;
            _UseNitro = useNitro;
        }

        public void Apply(Action action)
        {
            action.target_velocity_x = _VelX;
            action.target_velocity_z = _VelY;
            action.use_nitro = _UseNitro;
        }
    }
}
