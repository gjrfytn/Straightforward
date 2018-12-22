using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeBall2018.Strategy
{
    internal class JumpTurn : ITurn
    {
        private readonly double _JumpSpeed;

        public JumpTurn(double jumpSpeed)
        {
            _JumpSpeed = jumpSpeed;
        }

        public void Apply(Action action)
        {
            action.jump_speed = _JumpSpeed;
        }
    }
}
