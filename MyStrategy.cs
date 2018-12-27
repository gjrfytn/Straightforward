using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;
using Com.CodeGame.CodeBall2018.Strategy;

namespace Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk
{
    public sealed class MyStrategy : IStrategy
    {
        private const float _ForwardPaceDistance = 2;
        private const float _StrikeDistance = 0.1f; //TODO
        private const float _SupportDistanceDivider = 1.4f;
        private const float _Acceleration = 100;
        private const float _JumpSpeed = 100;
        private const float _GoalDangerDistance = 20;
        private const float _RobotJumpVel = 15.25f;
        private const float _RobotJumpGravityCorrection = 0.65f;
        private const float _BallLandingPredictionMinHeight = 4;

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

        private float GetJumpVelocity(float dt) => _RobotJumpVel - ((float)_Rules.GRAVITY + _RobotJumpGravityCorrection) * dt;

        private float GetJumpHeight(float dt) => _RobotJumpVel * dt - ((float)_Rules.GRAVITY + _RobotJumpGravityCorrection) * dt * dt / 2;

        private ITurn TryBlockAir()
        {
            const double ballInterceptionHeight = 2.2;

            if (_BallXYZ.Z > ballInterceptionHeight && _BallXY.Y > _RobotXY.Y) //TODO !!!
            {
                var currentRobotVel = new Vector2((float)_Robot.velocity_x, (float)_Robot.velocity_z);

                const float minReactionTime = 0.1f;
                const float maxReactionTime = 0.5f;
                const float dtStep = 0.05f;
                for (var dt = minReactionTime; dt <= maxReactionTime; dt += dtStep)
                {
                    var ballPos = GetBallPosDt(dt);

                    var robotPos = _RobotXYZ + new Vector3(currentRobotVel.X * dt,
                                               currentRobotVel.Y * dt,
                                               GetJumpHeight(dt));

                    var robotVel = new Vector3(currentRobotVel, GetJumpVelocity(dt));
                    var ballVel = new Vector3(_BallVel.X, _BallVel.Y, (float)(_BallVel.Z - _Rules.GRAVITY * dt));

                    var goalAimHeight = (float)_Rules.arena.goal_height / 4;
                    var ballToGoal = new Vector3(_EnemyGoalXY, goalAimHeight) - ballPos;
                    var ballToRobot = robotPos - ballPos;

                    var dot = Vector3.Dot(ballToRobot, ballToGoal) / ballToGoal.Length();

                    const float touchDist = 3;
                    const int requiredVelDiff = 20;
                    if (dot < 0 && Vector3.Distance(robotPos, ballPos) < touchDist && (robotVel - ballVel).Length() > requiredVelDiff)
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
            const float wallDampingCoef = 0.7f;
            if (pos.X > rightWall)
            {
                pos.X = rightWall - wallDampingCoef * (pos.X - rightWall);
            }
            else if (pos.X < leftWall)
            {
                pos.X = leftWall - wallDampingCoef * (pos.X - leftWall);
            }

            var frontWall = (float)_Rules.arena.depth / 2;
            var backWall = -frontWall;
            if (pos.Y > frontWall)
            {
                pos.Y = frontWall - wallDampingCoef * (pos.Y - frontWall);
            }
            else if (pos.Y < backWall)
            {
                pos.Y = backWall - wallDampingCoef * (pos.Y - backWall);
            }

            return pos;
        }

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

        private (Vector2 pos, float dt) GetBallPosAtHeight(float h)
        {
            float t = 1;
            float dt = 1;
            bool? wasGreater = null;
            while (true)
            {
                var ballXYZ = GetBallPosDt(t);

                const float errorEpsilon = 0.2f;
                if (ballXYZ.Z > h - errorEpsilon && ballXYZ.Z < h + errorEpsilon)
                    return (new Vector2(ballXYZ.X, ballXYZ.Y), t);

                if (ballXYZ.Z > h)
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
            var ballPos = PredictBallPositionForSupport();

            var fromTeamGoalToBall = ballPos - _TeamGoalXY;
            var targetPosG = fromTeamGoalToBall / _SupportDistanceDivider;
            var targetPosO = TransformFromTeamGoalSpace(targetPosG);
            var targetPosR = TransformToRobotSpace(targetPosO);
            var accel = _Acceleration * Vector2.Normalize(targetPosR);

            return MakePath(accel);
        }

        private ITurn PlayForward()
        {
            Vector2 ballPos;
            var speed = _Acceleration;
            if (_BallXYZ.Z > _BallLandingPredictionMinHeight)
            {
                float dt;
                (ballPos, dt) = GetBallPosAtHeight((float)_Game.ball.radius);

                if (_BallVel.Y > 0 && _RobotXY.Y < _BallXY.Y)
                {
                    var dist = Vector2.Distance(_RobotXY, _BallXY);
                    speed = dist / dt;
                }
            }
            else
                ballPos = _BallXY;

            var fromEnemyGoalToBall = ballPos - _EnemyGoalXY;
            const int antiGoalVectorLength = 15;
            var antiGoalDirectionB = antiGoalVectorLength * Vector2.Normalize(fromEnemyGoalToBall);
            var ballVel = new Vector2((float)_Game.ball.velocity_x, (float)_Game.ball.velocity_z);
            var normal = GetNormal(antiGoalDirectionB);
            var ballVelProjected = Vector2.Dot(ballVel, normal) * normal;
            var ballPosDirectionB = antiGoalDirectionB + ballVelProjected;
            var targetPosB = _ForwardPaceDistance * Vector2.Normalize(ballPosDirectionB);
            var targetPosO = targetPosB + ballPos;
            var targetPosR = TransformToRobotSpace(targetPosO);

            const int DecelerationBallMinHeight = 4;
            const int DecelerationToTargetPosDist = 1;
            const int DecelerationRate = 10;
            var normalizer = _BallXYZ.Z > DecelerationBallMinHeight && targetPosR.Length() < DecelerationToTargetPosDist ? DecelerationRate : 1; //TODO Сделать неравномерный разгон.

            var accel = speed * Vector2.Normalize(targetPosR);

            if (speed < _Rules.ROBOT_MAX_GROUND_SPEED)
            {
                var robotToBall = ballPos - _RobotXY;
                var targetIsBehindTheBall = Vector2.Dot(robotToBall, targetPosB) > 0;

                var strafeDir = targetIsBehindTheBall ? Vector2.Normalize(targetPosB) : GetNormal(targetPosB);

                var devation = Vector2.Dot(robotToBall, strafeDir);
                if (devation < 0)
                {
                    strafeDir = -strafeDir;
                    devation = -devation;
                }

                var speedReserve = (float)_Rules.ROBOT_MAX_GROUND_SPEED - speed;
                var strafeVel = System.Math.Clamp(devation, 0, speedReserve) * strafeDir;

                accel = accel + strafeVel;
            }

            return MakePath(accel / normalizer);
        }

        private static Vector2 GetNormal(Vector2 v) => Vector2.Normalize(new Vector2(-v.Y, v.X));

        private Vector2 PredictBallPositionForSupport()
        {
            Vector2 ballPos;
            if (_BallXYZ.Z > _BallLandingPredictionMinHeight)
            {
                ballPos = GetBallPosAtHeight((float)_Game.ball.radius).pos;

                if (_BallVel.Y > 0)
                {
                    const float ratio = 0.75f;
                    ballPos = Vector2.Lerp(_BallXY, ballPos, ratio);
                }
            }
            else
                ballPos = _BallXY;

            return ballPos;
        }

        private ITurn MakePath(Vector2 accel)
        {
            const int fallbackBallAvoidDist = 1;
            if (accel.Y < 0 && _BallXY.Y < _RobotXY.Y && Vector3.Distance(_RobotXYZ, _BallXYZ) < _Robot.radius + _Game.ball.radius + fallbackBallAvoidDist) //TODO avoid falling ball
            {
                var ballPos = TransformToRobotSpace(_BallXY);
                var cos = Vector2.Dot(ballPos, accel) / (ballPos.Length() * accel.Length());
                const double maxAvoidAngleCos = 0.75;
                if (cos > maxAvoidAngleCos)
                {
                    var goalPos = TransformToRobotSpace(_TeamGoalXY);

                    var cross = Vector3.Cross(new Vector3(ballPos, 0), new Vector3(goalPos, 0));

                    const float avoidAngleRad = 0.733f;
                    accel = Vector2.Transform(accel, Matrix3x2.CreateRotation(System.Math.Sign(cross.Z) * avoidAngleRad));
                }
            }

            return new MoveTurn(accel);
        }

        private ITurn TryStrike()
        {
            var danger = IsGoalInDanger();

            const float dangerStrikeDistanceMult = 2;
            var strikeRadius = _Robot.touch ? (danger ? dangerStrikeDistanceMult * _StrikeDistance : _StrikeDistance) : _Rules.ROBOT_MAX_RADIUS - _Rules.ROBOT_MIN_RADIUS;
            if (Vector3.Distance(_RobotXYZ, _BallXYZ) < _Game.ball.radius + _Robot.radius + strikeRadius)
            {
                var strikeDirection = new Vector3(Vector2.Normalize(_BallXY - _RobotXY), 0);

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
