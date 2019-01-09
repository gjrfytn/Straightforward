using System.Numerics;
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

        public (Vector3 pos, Vector3 vel) GetBallParamDt(float dt, Vector3 ballPos, Vector3 ballVel)
        {
            var pos = ballPos + new Vector3(ballVel.X * dt,
                                            ballVel.Y * dt,
                                            GetDZAfter(dt, ballVel.Z));

            var vel = new Vector3(ballVel.X, ballVel.Y, GetZVelAfter(dt, ballVel.Z));

            var rightWall = (float)_Rules.arena.width / 2;
            var leftWall = -rightWall;
            const float wallDampingCoef = 0.7f;
            var ballRadius = (float)_Rules.BALL_RADIUS;

            if (pos.X + ballRadius > rightWall)
            {
                pos.X = rightWall - ballRadius - wallDampingCoef * (pos.X + ballRadius - rightWall);
                vel.X = -wallDampingCoef * vel.X;
            }
            else if (pos.X - ballRadius < leftWall)
            {
                pos.X = leftWall + ballRadius - wallDampingCoef * (pos.X - ballRadius - leftWall);
                vel.X = -wallDampingCoef * vel.X;
            }

            var frontWall = (float)_Rules.arena.depth / 2;
            var backWall = -frontWall;
            var enemyGoalShrinkValue = 0.5f;

            //TODO Refactor
            if (pos.Y + ballRadius > frontWall)
            {
                var yAtReflection = frontWall - ballRadius;
                var tAtReflection = (yAtReflection - ballPos.Y) / ballVel.Y;

                var posAtReflection = ballPos + new Vector3(ballVel.X * tAtReflection,
                                                            yAtReflection,
                                                            GetDZAfter(tAtReflection, ballVel.Z));

                if (posAtReflection.X < -_Rules.arena.goal_width / 2 + enemyGoalShrinkValue || posAtReflection.X > _Rules.arena.goal_width / 2 - enemyGoalShrinkValue || posAtReflection.Z > _Rules.arena.goal_height - enemyGoalShrinkValue)
                {
                    pos.Y = frontWall - ballRadius - wallDampingCoef * (pos.Y + ballRadius - frontWall);
                    vel.Y = -wallDampingCoef * vel.Y;
                }
            }
            else if (pos.Y - ballRadius < backWall)
            {
                var yAtReflection = backWall + ballRadius;
                var tAtReflection = (yAtReflection - ballPos.Y) / ballVel.Y;

                var posAtReflection = ballPos + new Vector3(ballVel.X * tAtReflection,
                                                            yAtReflection,
                                                            GetDZAfter(tAtReflection, ballVel.Z));

                if (posAtReflection.X < -_Rules.arena.goal_width / 2 || posAtReflection.X > _Rules.arena.goal_width / 2 || posAtReflection.Z > _Rules.arena.goal_height)
                {
                    pos.Y = backWall + ballRadius - wallDampingCoef * (pos.Y - ballRadius - backWall);
                    vel.Y = -wallDampingCoef * vel.Y;
                }
            }

            return (pos, vel);
        }
    }
}
