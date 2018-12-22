using System.Collections.Generic;
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

        //Robot _Robot;
        //Rules _Rules;
        //Game _Game;

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
            //_Robot = robot;
            //_Rules = rules;
            //_Game = game;
            _RobotXY = new Vector2((float)robot.x, (float)robot.z);
            _RobotXYZ = new Vector3((float)robot.x, (float)robot.z, (float)robot.y);
            _EnemyGoalXY = new Vector2(0, (float)rules.arena.depth / 2) + new Vector2(0, (float)rules.arena.goal_depth / 2);
            _TeamGoalXY = -_EnemyGoalXY;
            _BallXY = new Vector2((float)game.ball.x, (float)game.ball.z);
            _BallXYZ = new Vector3((float)game.ball.x, (float)game.ball.z, (float)game.ball.y);
        }

        private ITurn MakeTurn(Robot robot, Rules rules, Game game)
        {
            _Spheres.Clear();

            var turn = TryStrike(robot, rules, game);

            if (turn != null)
                return turn;

            turn = TryBlockAir(robot, rules, game);

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

        private static float GetJumpHeight(float dt, Rules rules)
        {
            const float robotJumpVel = 15.25f;
            const float gravityCorrection = 0.65f;

            return robotJumpVel * dt - ((float)rules.GRAVITY + gravityCorrection) * dt * dt / 2 + 1;
        }

        private ITurn TryBlockAir(Robot robot, Rules rules, Game game)
        {
            if (robot.touch && _BallXYZ.Z > 2.2 && _BallXY.Y > _RobotXY.Y) //TODO
            {
                var ballVel = new Vector3((float)game.ball.velocity_x, (float)game.ball.velocity_z, (float)game.ball.velocity_y);
                var robotVel = new Vector2((float)robot.velocity_x, (float)robot.velocity_z);

                if (System.Math.Abs(ScaleVectorToHorizontal(new Vector3(robotVel, 7.48f)).Length() - ScaleVectorToHorizontal(ballVel).Length()) > 5) //TODO ballVel.Z - неправильно
                {
                    const float dt = 0.25f;//TODO

                    var turn = JumpIfWillHitAfterDt(rules, ballVel, robotVel, dt);

                    if (turn != null)
                        return turn;
                }

                if (System.Math.Abs(ScaleVectorToHorizontal(new Vector3(robotVel, 0)).Length() - ScaleVectorToHorizontal(ballVel).Length()) > 5)
                {
                    const float dt = 0.5f;//TODO

                    return JumpIfWillHitAfterDt(rules, ballVel, robotVel, dt);
                }
            }

            return null;
        }

        private ITurn JumpIfWillHitAfterDt(Rules rules, Vector3 ballVel, Vector2 robotVel, float dt)
        {
            var ballDXYZ = new Vector3(ballVel.X * dt,
                                       ballVel.Y * dt,
                                       (float)(ballVel.Z * dt - rules.GRAVITY * dt * dt / 2));

            var robotXYZB = TransformToBallSpace(_RobotXYZ + new Vector3(robotVel.X * dt,
                                                                         robotVel.Y * dt,
                                                                         GetJumpHeight(dt, rules)));

            if ((ballDXYZ - robotXYZB).Y > 0 && Vector3.Distance(robotXYZB, ballDXYZ) < 2.95)//TODO
                return new JumpTurn(_JumpSpeed);

            return null;
        }

        private Vector3 ScaleVectorToHorizontal(Vector3 v) => new Vector3(1.1f * v.X, 1.3f * v.Y, 0.7f * v.Z);

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

            return MakePath(accel);
        }

        private ITurn PlayForward(Robot robot, Rules rules, Game game)
        {
            var fromEnemyGoalToBall = _BallXY - _EnemyGoalXY;
            var antiGoalDirectionB = 20 * Vector2.Normalize(fromEnemyGoalToBall);
            var ballVel = new Vector2((float)game.ball.velocity_x, (float)game.ball.velocity_z);
            var ballPosDirectionB = antiGoalDirectionB + ballVel;
            var targetPosB = _PaceDistance * Vector2.Normalize(ballPosDirectionB);
            var targetPosO = TransformFromBallSpace(targetPosB);
            var targetPosR = TransformToRobotSpace(targetPosO);
            var accel = _Acceleration * Vector2.Normalize(targetPosR);

            return MakePath(accel);
        }

        private ITurn MakePath(Vector2 accel)
        {
            if (accel.Y < 0 && _BallXY.Y < _RobotXY.Y && Vector3.Distance(_RobotXYZ, _BallXYZ) < 4)
            {
                var ballPos = TransformToRobotSpace(_BallXY);
                var cos = Vector2.Dot(ballPos, accel) / (ballPos.Length() * accel.Length());
                if (cos > 0.75)
                {
                    var goalPos = TransformToRobotSpace(_TeamGoalXY);

                    var cross = Vector3.Cross(new Vector3(ballPos, 0), new Vector3(goalPos, 0));

                    accel = Vector2.Transform(accel, Matrix3x2.CreateRotation(System.Math.Sign(cross.Z) * 0.733f));
                }
            }

            return new MoveTurn(accel);
        }

        private ITurn TryStrike(Robot robot, Rules rules, Game game)
        {
            if (!robot.touch)
                return null;

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

        private bool IsGoalInDanger() => Vector2.Distance(_TeamGoalXY, _BallXY) < _GoalDangerDistance;

        private Vector2 TransformToRobotSpace(Vector2 v) => v - _RobotXY;
        private Vector2 TransformToBallSpace(Vector2 v) => v - _BallXY;
        private Vector3 TransformToBallSpace(Vector3 v) => v - _BallXYZ;
        private Vector2 TransformFromRobotSpace(Vector2 v) => v + _RobotXY;
        private Vector2 TransformFromBallSpace(Vector2 v) => v + _BallXY;
        private Vector3 TransformFromBallSpace(Vector3 v) => v + _BallXYZ;
        private Vector2 TransformFromTeamGoalSpace(Vector2 v) => v + _TeamGoalXY;

        private List<(Vector3 pos, float r, Vector3 col)> _Spheres = new List<(Vector3 pos, float r, Vector3 col)>();

        public string CustomRendering()
        {
            var json = "[";

            foreach (var (pos, r, col) in _Spheres)
            {
                json += $"{{\"Sphere\":{{\"x\":{pos.X},\"y\":{pos.Z},\"z\":{pos.Y},\"radius\":{r},\"r\":{col.X},\"g\":{col.Y},\"b\":{col.Z},\"a\":{0.5}}}}},";
            }

            return json.Remove(json.Length - 1) + "]";
        }
    }
}
