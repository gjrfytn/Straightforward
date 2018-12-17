using System.Numerics;
using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;
using Com.CodeGame.CodeBall2018.Strategy;

namespace Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk
{
    public sealed class MyStrategy : IStrategy
    {
        private const double _ToShootDistance = 2;

        private Vector2 _RobotXY;
        private Vector2 _EnemyGoalXY;
        private Vector2 _BallXY;

        public void Act(Robot me, Rules rules, Game game, Action action)
        {
            _RobotXY = new Vector2((float)me.x, (float)me.z);
            _EnemyGoalXY = new Vector2(0, (float)rules.arena.depth / 2) + new Vector2(0, (float)rules.arena.goal_depth / 2);
            _BallXY = new Vector2((float)game.ball.x, (float)game.ball.z);

            var turn = Move(me);

            turn.Apply(action);
        }

        private MoveTurn Move(Robot robot)
        {
            var fromEnemyGoalToBall = _BallXY - _EnemyGoalXY;
            var targetPosB = Vector2.Multiply(Vector2.Normalize(fromEnemyGoalToBall), (float)_ToShootDistance);
            var targetPosO = TransformFromBallSpace(targetPosB);
            var targetPosR = TransformToRobotSpace(targetPosO);
            var accel = 100 * Vector2.Normalize(targetPosR);

            return new MoveTurn(accel.X, 0, accel.Y);
        }

        private Vector2 TransformToRobotSpace(Vector2 v) => v - _RobotXY;
        private Vector2 TransformFromBallSpace(Vector2 v) => v + _BallXY;
    }
}
