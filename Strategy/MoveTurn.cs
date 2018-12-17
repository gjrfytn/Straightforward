using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeBall2018.Strategy
{
    class MoveTurn : ITurn
    {
        private readonly double _VelX;
        private readonly double _VelY;
        private readonly double _VelZ;

        public MoveTurn(double velX, double velY, double velZ)
        {
            _VelX = velX;
            _VelY = velY;
            _VelZ = velZ;
        }

        public void Apply(Action action)
        {
            action.target_velocity_x = _VelX;
            action.target_velocity_y = _VelY;
            action.target_velocity_z = _VelZ;
        }
    }
}
