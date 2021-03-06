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
        private const float _BallLandingPredictionMinHeight = 4;

        private static PhysicsSolver _Physics;

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

            _Physics = new PhysicsSolver(rules);

            _RobotXY = new Vector2((float)robot.x, (float)robot.z);
            _RobotXYZ = new Vector3(_RobotXY, (float)robot.y);
            _EnemyGoalXY = new Vector2(0, (float)rules.arena.depth / 2) + new Vector2(0, (float)rules.arena.goal_depth / 2);
            _TeamGoalXY = -_EnemyGoalXY;
            _BallXY = new Vector2((float)game.ball.x, (float)game.ball.z);
            _BallXYZ = new Vector3(_BallXY, (float)game.ball.y);
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
                    var (ballPos, ballVel) = _Physics.GetBallParamDt(dt, _BallXYZ, _BallVel);

                    var robotPos = _RobotXYZ + new Vector3(currentRobotVel.X * dt,
                                               currentRobotVel.Y * dt,
                                               _Physics.GetJumpHeightAfter(dt));

                    var robotVel = new Vector3(currentRobotVel, _Physics.GetJumpVelocityAfter(dt));

                    var goalAimHeight = (float)_Rules.arena.goal_height / 4;
                    var ballToGoal = new Vector3(_EnemyGoalXY, goalAimHeight) - ballPos;
                    var ballToRobot = robotPos - ballPos;

                    var dot = Vector3.Dot(ballToRobot, ballToGoal) / ballToGoal.Length();

                    const float touchDist = 2.98f;
                    const int requiredVelDiff = 30;
                    if (dot < 0 && Vector3.Distance(robotPos, ballPos) < touchDist && (robotVel - new Vector3(ballVel.X, ballVel.Y, 0.75f * ballVel.Z)).Length() > requiredVelDiff)//TODO 0.75f
                        return new JumpTurn(_JumpSpeed);
                }
            }

            return null;
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
                (ballPos, dt) = _Physics.GetBallPosAtHeight((float)_Game.ball.radius, _BallXYZ, _BallVel);

                var dist = Vector2.Distance(_RobotXY, ballPos);
                speed = dist / dt;
                if (_RobotXY.Y < ballPos.Y)
                {
                    if (speed < _Rules.ROBOT_MAX_GROUND_SPEED - 5)
                        speed -= 5;
                }
                else
                {
                    if (speed > 5)
                        speed += 5;
                }
            }
            else
                ballPos = _BallXY;

            Vector2 ballPosDirectionB;
            if (IsBallAlmostInEnemyGoal(ballPos))
                ballPosDirectionB = ballPos - (_EnemyGoalXY + 30 * Vector2.UnitY);
            else
            {
                var fromEnemyGoalToBall = ballPos - _EnemyGoalXY;
                const int antiGoalVectorLength = 15;
                var antiGoalDirectionB = antiGoalVectorLength * Vector2.Normalize(fromEnemyGoalToBall);
                var ballVel = new Vector2(_BallVel.X, _BallVel.Y);
                var normal = GetNormal(antiGoalDirectionB);
                var ballVelProjected = Vector2.Dot(ballVel, normal) * normal;
                ballPosDirectionB = antiGoalDirectionB + ballVelProjected;
            }

            var targetPosB = _ForwardPaceDistance * Vector2.Normalize(ballPosDirectionB);
            var targetPosO = targetPosB + ballPos;
            var targetPosR = TransformToRobotSpace(targetPosO);

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

            return MakePath(accel);
        }

        private static Vector2 GetNormal(Vector2 v) => Vector2.Normalize(new Vector2(-v.Y, v.X));

        private Vector2 PredictBallPositionForSupport()
        {
            Vector2 ballPos;
            if (_BallXYZ.Z > _BallLandingPredictionMinHeight)
            {
                ballPos = _Physics.GetBallPosAtHeight((float)_Game.ball.radius, _BallXYZ, _BallVel).pos;

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
            var ballPos = TransformToRobotSpace(_BallXY);
            const int fallbackBallAvoidDist = 1;
            if (accel.Y < 0 && _BallXY.Y < _RobotXY.Y && Vector2.Distance(_RobotXY, _BallXY) < _Robot.radius + _Game.ball.radius + fallbackBallAvoidDist) //TODO avoid falling ball
            {
                const float dt = 1;

                var ballZAfterDt = _BallXYZ.Z + _Physics.GetDZAfter(dt, _BallVel.Z);
                if (ballZAfterDt < _Rules.BALL_RADIUS + _Robot.radius + 1)//TODO 1
                {
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
            }
            else if (_RobotXY.Y < -_Rules.arena.depth / 2 - _Robot.radius)
                accel += _Acceleration * Vector2.Normalize(ballPos);

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

                if (!_Robot.touch || danger)
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

        private bool IsBallAlmostInEnemyGoal(Vector2 ballPos) => ballPos.X > -_Rules.arena.goal_width / 2 && ballPos.X < _Rules.arena.goal_width / 2 && ballPos.Y > GetEnemyGoalZoneY(ballPos.X);
        private float GetEnemyGoalZoneY(float x) => (float)_Rules.arena.depth / 2 + 0.023f * x * x - 10;
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
