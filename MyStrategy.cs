using System.Linq;
using System.Numerics;
using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;
using Com.CodeGame.CodeBall2018.Strategy;

namespace Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk
{
    public sealed class MyStrategy : IStrategy
    {
        private const float _PaceDistance = 2;
        private const float _StrikeDistance = 0.1f;
        private const float _SupportDistance = 1.3f;
        private const float _Acceleration = 100;
        private const float _JumpSpeed = 100;
        private const float _GoalDangerDistance = 20;
        private Vector2 _RobotXY;
        private Vector3 _RobotXYZ;
        private Vector2 _EnemyGoalXY;
        private Vector2 _TeamGoalXY;
        private Vector2 _BallXY;
        private Vector3 _BallXYZ;

        public void Act(Robot me, Rules rules, Game game, Action action)
        {
            Initialize(me, rules, game);

            var turn = MakeTurn(me, rules, game);

            turn.Apply(action);
        }

        private void Initialize(Robot robot, Rules rules, Game game)
        {
            _RobotXY = new Vector2((float)robot.x, (float)robot.z);
            _RobotXYZ = new Vector3((float)robot.x, (float)robot.z, (float)robot.y);
            _EnemyGoalXY = new Vector2(0, (float)rules.arena.depth / 2) + new Vector2(0, (float)rules.arena.goal_depth / 2);
            _TeamGoalXY = -_EnemyGoalXY;
            _BallXY = new Vector2((float)game.ball.x, (float)game.ball.z);
            _BallXYZ = new Vector3((float)game.ball.x, (float)game.ball.z, (float)game.ball.y);
        }

        private ITurn MakeTurn(Robot robot, Rules rules, Game game)
        {
            var turn = TryStrike(robot, rules, game);

            if (turn != null)
                return turn;

            if (IsThisRobotClosestToBall(robot, game) || IsThisRobotTheLastHope(robot, game))
            {
                return PlayForward(robot, rules, game);
            }
            else
            {
                return PlaySupport(robot, rules, game);
            }
        }

        private bool IsThisRobotTheLastHope(Robot robot, Game game)
        {
            if (_RobotXY.Y < _BallXY.Y && game.robots.Where(r => r.is_teammate && r.id != robot.id).All(r => r.z > _BallXY.Y))
                return true;

            return false;
        }

        private bool IsThisRobotClosestToBall(Robot robot, Game game)
        {
            var closestRobot = game.robots.Where(r => r.is_teammate)
                                          .Select(r => new { RobotId = r.id, Distance = Vector2.Distance(new Vector2((float)r.x, (float)r.z), _BallXY) })
                                          .OrderBy(r => r.Distance)
                                          .First();

            return closestRobot.RobotId == robot.id;
        }

        private ITurn PlaySupport(Robot robot, Rules rules, Game game)
        {
            var fromTeamGoalToBall = _BallXY - _TeamGoalXY;
            var targetPosG = fromTeamGoalToBall / _SupportDistance;
            var targetPosO = TransformFromTeamGoalSpace(targetPosG);
            var targetPosR = TransformToRobotSpace(targetPosO);
            var accel = _Acceleration * Vector2.Normalize(targetPosR);

            return new MoveTurn(accel.X, 0, accel.Y);
        }

        private ITurn PlayForward(Robot robot, Rules rules, Game game)
        {
            var fromEnemyGoalToBall = _BallXY - _EnemyGoalXY;
            var targetPosB = _PaceDistance * Vector2.Normalize(fromEnemyGoalToBall);
            var targetPosO = TransformFromBallSpace(targetPosB);
            var targetPosR = TransformToRobotSpace(targetPosO);
            var accel = _Acceleration * Vector2.Normalize(targetPosR);

            return new MoveTurn(accel.X, 0, accel.Y);
        }

        private ITurn TryStrike(Robot robot, Rules rules, Game game)
        {
            var danger = IsGoalInDanger();

            if (Vector3.Distance(_RobotXYZ, _BallXYZ) < game.ball.radius + robot.radius + (danger ? 2 * _StrikeDistance : _StrikeDistance))
            {
                var strikeDirection = new Vector3(Vector2.Normalize(-(_RobotXY - _BallXY)), 0);

                if (danger)
                {
                    if (strikeDirection.Y > 0)
                        return new JumpTurn(_JumpSpeed);
                }
                else
                {
                    var goalLeftB = new Vector3(TransformToBallSpace(new Vector2(-(float)rules.arena.goal_width / 2, (float)rules.arena.depth / 2)), 0);
                    var goalRightB = new Vector3(TransformToBallSpace(new Vector2((float)rules.arena.goal_width / 2, (float)rules.arena.depth / 2)), 0);

                    if (Vector3.Cross(strikeDirection, goalLeftB).Z > 0 && Vector3.Cross(goalRightB, strikeDirection).Z > 0)
                        return new JumpTurn(_JumpSpeed);
                }
            }

            return null;
        }

        private bool IsGoalInDanger()
        {
            return Vector2.Distance(_TeamGoalXY, _BallXY) < _GoalDangerDistance;
        }

        private Vector2 TransformToRobotSpace(Vector2 v) => v - _RobotXY;
        private Vector2 TransformToBallSpace(Vector2 v) => v - _BallXY;
        private Vector2 TransformFromBallSpace(Vector2 v) => v + _BallXY;
        private Vector2 TransformFromTeamGoalSpace(Vector2 v) => v + _TeamGoalXY;
    }
}
