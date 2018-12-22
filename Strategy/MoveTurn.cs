using System.Numerics;
using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeBall2018.Strategy
{
    internal class MoveTurn : ITurn
    {
        private readonly double _VelX;
        private readonly double _VelY;

        public MoveTurn(double velX, double velY)
        {
            _VelX = velX;
            _VelY = velY;
        }

        public MoveTurn(Vector2 vel)
        {
            _VelX = vel.X;
            _VelY = vel.Y;
        }

        public void Apply(Action action)
        {
            action.target_velocity_x = _VelX;
            action.target_velocity_z = _VelY;
        }
    }
}
