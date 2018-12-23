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
        private const float _SupportDistance = 1.4f;
        private const float _Acceleration = 100;
        private const float _JumpSpeed = 100;
        private const float _GoalDangerDistance = 20;

        private Robot _Robot;
        private Rules _Rules;
        private Game _Game;

        private Vector2 _RobotXY;
        private Vector3 _RobotXYZ;
        private Vector2 _EnemyGoalXY;
        private Vector2 _TeamGoalXY;
        private Vector2 _BallXY;
        private Vector3 _BallXYZ;
        private Vector3 _BallVel;

        public void Act(Robot me, Rules rules, Game game, Action action)
        {
            Initialize(me, rules, game);

            var turn = MakeTurn();

            turn.Apply(action);
        }

        private void Initialize(Robot robot, Rules rules, Game game)
        {
            _Robot = robot;
            _Rules = rules;
            _Game = game;
            _RobotXY = new Vector2((float)robot.x, (float)robot.z);
            _RobotXYZ = new Vector3((float)robot.x, (float)robot.z, (float)robot.y);
            _EnemyGoalXY = new Vector2(0, (float)rules.arena.depth / 2) + new Vector2(0, (float)rules.arena.goal_depth / 2);
            _TeamGoalXY = -_EnemyGoalXY;
            _BallXY = new Vector2((float)game.ball.x, (float)game.ball.z);
            _BallXYZ = new Vector3((float)game.ball.x, (float)game.ball.z, (float)game.ball.y);
            _BallVel = new Vector3((float)_Game.ball.velocity_x, (float)_Game.ball.velocity_z, (float)_Game.ball.velocity_y);
        }

        private ITurn MakeTurn()
        {
            _Spheres.Clear();

            var turn = TryStrike();

            if (turn != null)
                return turn;

            if (_Robot.touch)
            {
                turn = TryBlockAir();

                if (turn != null)
                    return turn;

                if (IsThisRobotClosestToBall() || IsThisRobotTheLastHope())
                {
                    return PlayForward();
                }
                else
                {
                    return PlaySupport();
                }
            }

            return new EmptyTurn();
        }

        private float GetJumpVelocity(float dt)
        {
            const float robotJumpVel = 15.25f;
            const float gravityCorrection = 0.65f;

            return robotJumpVel - ((float)_Rules.GRAVITY + gravityCorrection) * dt;
        }

        private float GetJumpHeight(float dt)
        {
            const float robotJumpVel = 15.25f;
            const float gravityCorrection = 0.65f;

            return robotJumpVel * dt - ((float)_Rules.GRAVITY + gravityCorrection) * dt * dt / 2;
        }

        private ITurn TryBlockAir()
        {
            if (_BallXYZ.Z > 2.2 && _BallXY.Y > _RobotXY.Y) //TODO !!!
            {
                var robotVel = new Vector2((float)_Robot.velocity_x, (float)_Robot.velocity_z);

                for (var dt = 0.1f; dt <= 0.5f; dt += 0.05f)
                {
                    var ballPos = GetBallPosDt(dt);

                    var robotPos = _RobotXYZ + new Vector3(robotVel.X * dt,
                                               robotVel.Y * dt,
                                               GetJumpHeight(dt));

                    var robotVel2 = new Vector3(robotVel, GetJumpVelocity(dt));
                    var ballVel2 = new Vector3(_BallVel.X, _BallVel.Y, (float)(_BallVel.Z - _Rules.GRAVITY * dt));

                    var ballToGoal = new Vector3(_EnemyGoalXY, (float)_Rules.arena.goal_height / 4) - ballPos;
                    var ballToRobot = robotPos - ballPos;

                    var dot = Vector3.Dot(ballToRobot, ballToGoal) / ballToGoal.Length();

                    if (dot < 0 && Vector3.Distance(robotPos, ballPos) < 3 && (robotVel2 - ballVel2).Length() > 20)
                        return new JumpTurn(_JumpSpeed);
                }
            }

            return null;
        }

        private Vector3 GetBallPosDt(float dt)
        {
            var pos = _BallXYZ + new Vector3(_BallVel.X * dt,
                                              _BallVel.Y * dt,
                                              (float)(_BallVel.Z * dt - _Rules.GRAVITY * dt * dt / 2));

            var rightWall = (float)_Rules.arena.width / 2;
            var leftWall = -rightWall;
            if (pos.X > rightWall)
            {
                pos.X = rightWall - 0.7f * (pos.X - rightWall);
            }
            else if (pos.X < leftWall)
            {
                pos.X = leftWall - 0.7f * (pos.X - leftWall);
            }

            var frontWall = (float)_Rules.arena.depth / 2;
            var backWall = -frontWall;
            if (pos.Y > frontWall)
            {
                pos.Y = frontWall - 0.7f * (pos.Y - frontWall);
            }
            else if (pos.Y < backWall)
            {
                pos.Y = backWall - 0.7f * (pos.Y - backWall);
            }

            return pos;
        }

        private ITurn JumpIfWillHitAfterDt(Vector3 ballVel, Vector2 robotVel, float dt)
        {
            var ballDXYZ = new Vector3(ballVel.X * dt,
                                       ballVel.Y * dt,
                                       (float)(ballVel.Z * dt - _Rules.GRAVITY * dt * dt / 2));

            var robotXYZB = TransformToBallSpace(_RobotXYZ + new Vector3(robotVel.X * dt,
                                                                         robotVel.Y * dt,
                                                                         GetJumpHeight(dt)));

            if ((ballDXYZ - robotXYZB).Y > 0 && Vector3.Distance(robotXYZB, ballDXYZ) < 2.95)//TODO
                return new JumpTurn(_JumpSpeed);

            return null;
        }

        private Vector3 ScaleVectorToHorizontal(Vector3 v) => new Vector3(1.1f * v.X, 1.3f * v.Y, 0.7f * v.Z);

        private bool IsThisRobotTheLastHope()
        {
            if (_RobotXY.Y < _BallXY.Y && _Game.robots.Where(r => r.is_teammate && r.id != _Robot.id).All(r => r.z > _BallXY.Y))
                return true;

            return false;
        }

        private bool IsThisRobotClosestToBall()
        {
            var closestRobot = _Game.robots.Where(r => r.is_teammate)
                                          .Select(r => new { RobotId = r.id, Distance = Vector2.Distance(new Vector2((float)r.x, (float)r.z), _BallXY) })
                                          .OrderBy(r => r.Distance)
                                          .First();

            return closestRobot.RobotId == _Robot.id;
        }

        private Vector2 GetBallGroundTouchPos()
        {
            float t = 1;
            float dt = 1;
            bool? wasGreater = null;
            while (true)
            {
                var ballXYZ = GetBallPosDt(t);

                if (ballXYZ.Z > 1.8 && ballXYZ.Z < 2.2)
                {
                    _Spheres.Add((ballXYZ, 0.7f, new Vector3(1, 0, 0)));

                    return new Vector2(ballXYZ.X, ballXYZ.Y);
                }
                else if (ballXYZ.Z > 2)
                {
                    if (wasGreater.HasValue && !wasGreater.Value)
                    {
                        dt /= 2;
                    }

                    wasGreater = true;

                    t += dt;
                }
                else
                {
                    if (wasGreater.HasValue && wasGreater.Value)
                    {
                        dt /= 2;
                    }

                    wasGreater = false;

                    t -= dt;
                }
            }

            throw new System.Exception("Should not be here.");
        }

        private ITurn PlaySupport()
        {
            var ballPos = _BallXYZ.Z > 3 ? GetBallGroundTouchPos() : _BallXY;

            var fromTeamGoalToBall = ballPos - _TeamGoalXY;
            var targetPosG = fromTeamGoalToBall / _SupportDistance;
            var targetPosO = TransformFromTeamGoalSpace(targetPosG);
            var targetPosR = TransformToRobotSpace(targetPosO);
            var accel = _Acceleration * Vector2.Normalize(targetPosR);

            return MakePath(accel);
        }

        private ITurn PlayForward()
        {
            var ballPos = _BallXYZ.Z > 3 ? GetBallGroundTouchPos() : _BallXY;

            var fromEnemyGoalToBall = ballPos - _EnemyGoalXY;
            var antiGoalDirectionB = 15 * Vector2.Normalize(fromEnemyGoalToBall);
            var ballVel = new Vector2((float)_Game.ball.velocity_x, (float)_Game.ball.velocity_z);
            var normal = Vector2.Normalize(new Vector2(-antiGoalDirectionB.Y, antiGoalDirectionB.X));
            var ballVelProjected = Vector2.Dot(ballVel, normal) * normal;
            var ballPosDirectionB = antiGoalDirectionB + ballVelProjected;
            var targetPosB = _PaceDistance * Vector2.Normalize(ballPosDirectionB);
            var targetPosO = targetPosB + ballPos;
            var targetPosR = TransformToRobotSpace(targetPosO);

            var normalizer = _BallXYZ.Z > 4 && targetPosR.Length() < 1 ? 10 : 1;

            var accel = _Acceleration * Vector2.Normalize(targetPosR) / normalizer;

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

        private ITurn TryStrike()
        {
            var danger = IsGoalInDanger();

            var strikeRadius = _Robot.touch ? (danger ? 2 * _StrikeDistance : _StrikeDistance) : _Rules.ROBOT_MAX_RADIUS - _Rules.ROBOT_MIN_RADIUS;
            if (Vector3.Distance(_RobotXYZ, _BallXYZ) < _Game.ball.radius + _Robot.radius + strikeRadius)
            {
                var strikeDirection = new Vector3(Vector2.Normalize(-(_RobotXY - _BallXY)), 0);

                if (danger)
                {
                    if (strikeDirection.Y > 0)
                        return new JumpTurn(_JumpSpeed);
                }
                else
                {
                    var goalLeftB = new Vector3(TransformToBallSpace(new Vector2(-(float)_Rules.arena.goal_width / 2, (float)_Rules.arena.depth / 2)), 0);
                    var goalRightB = new Vector3(TransformToBallSpace(new Vector2((float)_Rules.arena.goal_width / 2, (float)_Rules.arena.depth / 2)), 0);

                    if (Vector3.Cross(strikeDirection, goalLeftB).Z > 0 && Vector3.Cross(goalRightB, strikeDirection).Z > 0)
                        return new JumpTurn(_JumpSpeed);
                }
            }

            return null;
        }

        private bool IsGoalInDanger() => Vector2.Distance(_TeamGoalXY, _BallXY) < _GoalDangerDistance;

        private Vector2 TransformToRobotSpace(Vector2 v) => v - _RobotXY;
        private Vector3 TransformToRobotSpace(Vector3 v) => v - _RobotXYZ;
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
