using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeBall2018.Strategy
{
    internal class PhysicsSolver
    {
        private const float _RobotJumpVel = 15.25f;
        private const float _RobotJumpGravityCorrection = 0.65f;

        private readonly Rules _Rules;

        public PhysicsSolver(Rules rules)
        {
            _Rules = rules;
        }

        public float GetJumpVelocityAfter(float dt) => _RobotJumpVel - ((float)_Rules.GRAVITY + _RobotJumpGravityCorrection) * dt;

        public float GetJumpHeightAfter(float dt) => _RobotJumpVel * dt - ((float)_Rules.GRAVITY + _RobotJumpGravityCorrection) * dt * dt / 2;

        public float GetZVelAfter(float dt, float vel0) => (float)(vel0 - _Rules.GRAVITY * dt);

        public float GetDZAfter(float dt, float vel0) => (float)(vel0 * dt - _Rules.GRAVITY * dt * dt / 2);
    }
}
