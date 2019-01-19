using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeBall2018.Strategy
{
    internal class PhysicsSolver
    {
        private const float _RobotJumpVel = 15.25f;
        private const float _RobotJumpGravityCorrection = 0.65f;

        private static int _CurrentTick = -1;
        private static List<(float dt, Vector3 pos, Vector3 vel)> _BallParametersCache = new List<(float dt, Vector3 pos, Vector3 vel)>();

        private readonly Rules _Rules;
        private readonly Game _Game;

        public PhysicsSolver(Rules rules, Game game)
        {
            _Rules = rules;
            _Game = game;

            if (_CurrentTick != _Game.current_tick)
            {
                _CurrentTick = _Game.current_tick;
                _BallParametersCache.Clear();
            }

        }

        public float GetJumpVelocityAfter(float dt) => _RobotJumpVel - ((float)_Rules.GRAVITY + _RobotJumpGravityCorrection) * dt;

        public float GetJumpHeightAfter(float dt) => _RobotJumpVel * dt - ((float)_Rules.GRAVITY + _RobotJumpGravityCorrection) * dt * dt / 2;

        public float GetZVelAfter(float dt, float vel0) => (float)(vel0 - _Rules.GRAVITY * dt);

        public float GetDZAfter(float dt, float vel0) => (float)(vel0 * dt - _Rules.GRAVITY * dt * dt / 2);

        public (Vector3 pos, Vector3 vel) GetBallParamDt(float dt, Vector3 ballPos, Vector3 ballVel)
        {
            if (_BallParametersCache.Any(p => p.dt == dt))
            {
                var parameters = _BallParametersCache.First(p => p.dt == dt);

                return (parameters.pos, parameters.vel);
            }

            var ball = new Entity
            {
                arena_e = (float)_Rules.BALL_ARENA_E,
                mass = (float)_Rules.BALL_MASS,
                radius = (float)_Game.ball.radius,
                position = new Vector3((float)_Game.ball.x, (float)_Game.ball.y, (float)_Game.ball.z),
                velocity = new Vector3((float)_Game.ball.velocity_x, (float)_Game.ball.velocity_y, (float)_Game.ball.velocity_z)
            };

            for (var i = 0; i < dt * _Rules.TICKS_PER_SECOND; ++i)
            {
                Tick(ball);
            }

            var pos = new Vector3(ball.position.X, ball.position.Z, ball.position.Y);
            var vel = new Vector3(ball.velocity.X, ball.velocity.Z, ball.velocity.Y);

            _BallParametersCache.Add((dt, pos, vel));

            return (pos, vel);
        }

        public (Vector2 pos, float dt) GetBallPosAtHeight(float h, Vector3 ballPos, Vector3 ballVel)
        {
            var dt = GetDtToReachBallZ(ballPos.Z, h, ballVel.Z);

            var epsilon = 0.5;
            var guard = 0;
            while (true)
            {
                var ballXYZ = GetBallParamDt(dt, ballPos, ballVel).pos;

                var error = ballXYZ.Z - h;
                if (System.Math.Abs(error) < epsilon)
                    return (new Vector2(ballXYZ.X, ballXYZ.Y), dt);

                dt += error / 100;

                guard++;
                if (guard == 100)
                {
                    dt = GetDtToReachBallZ(ballPos.Z, h, ballVel.Z);
                    ballXYZ = GetBallParamDt(dt, ballPos, ballVel).pos;

                    return (new Vector2(ballXYZ.X, ballXYZ.Y), dt);
                }
            }
        }

        private float GetDtToReachBallZ(float currentZ, float targetZ, float velocityZ)
        {
            var a = (float)-_Rules.GRAVITY;
            var b = 2 * velocityZ;
            var c = 2 * (currentZ - targetZ);

            var d = b * b - 4 * a * c;

            var x1 = (-b - (float)System.Math.Sqrt(d)) / (2 * a);
            var x2 = (-b + (float)System.Math.Sqrt(d)) / (2 * a);

            var min = System.Math.Min(x1, x2);

            return min < 0 ? System.Math.Max(x1, x2) : min;
        }

        private class Entity
        {
            public Vector3 position;
            public float radius;
            public float mass;
            public float arena_e;
            public float radius_change_speed;
            public Vector3 velocity;
        }

        private void CollideEntities(Entity a, Entity b)
        {
            var delta_position = b.position - a.position;
            var distance = delta_position.Length();
            var penetration = a.radius + b.radius - distance;
            if (penetration > 0)
            {
                var k_a = (1 / a.mass) / ((1 / a.mass) + (1 / b.mass));
                var k_b = (1 / b.mass) / ((1 / a.mass) + (1 / b.mass));
                var normal = Vector3.Normalize(delta_position);
                a.position -= normal * penetration * k_a;
                b.position += normal * penetration * k_b;
                var delta_velocity = Vector3.Dot(b.velocity - a.velocity, normal)
                - b.radius_change_speed - a.radius_change_speed;
                if (delta_velocity < 0)
                {
                    var impulse = (float)((1 + (_Rules.MIN_HIT_E + _Rules.MAX_HIT_E) / 2) * delta_velocity) * normal; //TODO
                    a.velocity += impulse * k_a;
                    b.velocity -= impulse * k_b;
                }
            }
        }

        private Vector3? CollideWithArena(Entity e)
        {
            var (dist, normal) = DanToArena(e.position);
            var penetration = e.radius - dist;
            if (penetration > 0)
            {
                e.position += penetration * normal;
                var velocity = Vector3.Dot(e.velocity, normal) - e.radius_change_speed;
                if (velocity < 0)
                {
                    e.velocity -= (1 + e.arena_e) * velocity * normal;

                    return normal;
                }
            }

            return null;
        }

        private void Move(Entity e, float delta_time)
        {
            e.velocity = Clamp(e.velocity, (float)_Rules.MAX_ENTITY_SPEED);
            e.position += e.velocity * delta_time;
            e.position.Y -= (float)_Rules.GRAVITY * delta_time * delta_time / 2;
            e.velocity.Y -= (float)_Rules.GRAVITY * delta_time;
        }

        private void Update(Entity ball, float delta_time)
        {
            //shuffle(robots); //TODO
            //foreach (var robot in _Game.robots)
            //{
            //    if (robot.touch)
            //    {
            //        var target_velocity = clamp(
            //        robot.action.target_velocity,
            //        _Rules.ROBOT_MAX_GROUND_SPEED);
            //        target_velocity -= robot.touch_normal
            //        * Vector3.Dot(robot.touch_normal, target_velocity);
            //        var target_velocity_change = target_velocity - robot.velocity;
            //        if (target_velocity_change.Length() > 0)
            //        {
            //            var acceleration = _Rules.ROBOT_ACCELERATION * Math.Max(0, robot.touch_normal.Y);
            //            robot.velocity += clamp(
            //            Vector3.Normalize(target_velocity_change) * acceleration * delta_time,
            //            target_velocity_change.Length());
            //        }
            //    }

            //    if (robot.action.use_nitro)
            //    {
            //        var target_velocity_change = clamp(
            //        robot.action.target_velocity - robot.velocity,
            //        robot.nitro * _Rules.NITRO_POINT_VELOCITY_CHANGE);
            //        if (target_velocity_change.Length() > 0)
            //        {
            //            var acceleration = Vector3.Normalize(target_velocity_change)
            //            * (float)_Rules.ROBOT_NITRO_ACCELERATION;
            //            var velocity_change = clamp(
            //        acceleration * delta_time,
            //        target_velocity_change.Length());
            //            robot.velocity += velocity_change;
            //            robot.nitro -= velocity_change.Length()
            //        / _Rules.NITRO_POINT_VELOCITY_CHANGE;
            //        }

            //        move(robot, delta_time);
            //        robot.radius = _Rules.ROBOT_MIN_RADIUS + (_Rules.ROBOT_MAX_RADIUS - _Rules.ROBOT_MIN_RADIUS)
            //        * robot.action.jump_speed / _Rules.ROBOT_MAX_JUMP_SPEED;
            //        robot.radius_change_speed = robot.action.jump_speed;
            //    }
            //}

            Move(ball, delta_time);

            //for (var i = 0; i < _Game.robots.Length; ++i)
            //    for (var j = 0; j < i; ++j)
            //        collide_entities(_Game.robots[i], _Game.robots[j]);

            //foreach (var robot in _Game.robots)
            //{
            //    collide_entities(robot, _Game.ball);
            //    var collision_normal = collide_with_arena(robot);
            //    if (collision_normal == null)
            //        robot.touch = false;
            //    else
            //    {
            //        robot.touch = true;
            //        robot.touch_normal = collision_normal;
            //    }
            //}

            CollideWithArena(ball);

            //if (abs(_Game.ball.position.Z) > _Rules.arena.depth / 2 + _Game.ball.radius) //TODO
            //    goal_scored();

            //foreach (var robot in _Game.robots)
            //{
            //    if (robot.nitro == _Rules.MAX_NITRO_AMOUNT)
            //        continue;

            //    foreach (var pack in _Game.nitro_packs)
            //    {
            //        if (!pack.alive)
            //            continue;

            //        if ((robot.position - pack.position).Length() <= robot.radius + pack.radius)
            //        {
            //            robot.nitro = _Rules.MAX_NITRO_AMOUNT;
            //            pack.alive = false;
            //            pack.respawn_ticks = _Rules.NITRO_PACK_RESPAWN_TICKS;
            //        }
            //    }
            //}
        }

        private void Tick(Entity ball)
        {
            var delta_time = 1.0f / _Rules.TICKS_PER_SECOND;
            //for (var _ = 0; _ < _Rules.MICROTICKS_PER_TICK; ++_)
            Update(ball, delta_time/* / _Rules.MICROTICKS_PER_TICK*/);

            //foreach (var pack in _Game.nitro_packs)
            //{
            //    if (pack.alive)
            //        continue;

            //    pack.respawn_ticks -= 1;
            //    if (pack.respawn_ticks == 0)
            //        pack.alive = true;
            //}
        }

        private (float dist, Vector3 normal) DanToPlane(Vector3 point, Vector3 pointOnPlane, Vector3 plane_normal) => (Vector3.Dot(point - pointOnPlane, plane_normal), plane_normal);

        private (float dist, Vector3 normal) DanToSphereInner(Vector3 point, Vector3 sphereCenter, float sphereRadius) => (sphereRadius - (point - sphereCenter).Length(), Vector3.Normalize(sphereCenter - point));

        private (float dist, Vector3 normal) DanToSphereOuter(Vector3 point, Vector3 sphereCenter, float sphereRadius) => ((point - sphereCenter).Length() - sphereRadius, Vector3.Normalize(point - sphereCenter));

        private (float dist, Vector3 normal) DanToArenaQuarter(Vector3 point)
        {
            //TODO
            //// Ground
            //var dan = DanToPlane(point, new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            //// Ceiling
            //dan = Min(dan, DanToPlane(point, new Vector3(0, (float)_Rules.arena.height, 0), new Vector3(0, -1, 0)));

            // Ceiling
            var dan = DanToPlane(point, new Vector3(0, (float)_Rules.arena.height, 0), new Vector3(0, -1, 0));
            // Side x
            dan = Min(dan, DanToPlane(point, new Vector3((float)_Rules.arena.width / 2, 0, 0), new Vector3(-1, 0, 0)));
            // Side z (goal)
            dan = Min(dan, DanToPlane(
            point,
            new Vector3(0, 0, ((float)_Rules.arena.depth / 2) + (float)_Rules.arena.goal_depth),
            new Vector3(0, 0, -1)));
            // Side z
            var v = new Vector2(point.X, point.Y) - new Vector2(
            ((float)_Rules.arena.goal_width / 2) - (float)_Rules.arena.goal_top_radius,
            (float)_Rules.arena.goal_height - (float)_Rules.arena.goal_top_radius);

            if (point.X >= (_Rules.arena.goal_width / 2) + _Rules.arena.goal_side_radius
            || point.Y >= _Rules.arena.goal_height + _Rules.arena.goal_side_radius
            || (
            v.X > 0
            && v.Y > 0
            && v.Length() >= _Rules.arena.goal_top_radius + _Rules.arena.goal_side_radius))
                dan = Min(dan, DanToPlane(point, new Vector3(0, 0, (float)_Rules.arena.depth / 2), new Vector3(0, 0, -1)));
            // Side x & ceiling (goal)
            if (point.Z >= (_Rules.arena.depth / 2) + _Rules.arena.goal_side_radius)
            {
                // x
                dan = Min(dan, DanToPlane(
                point,
                new Vector3((float)_Rules.arena.goal_width / 2, 0, 0),
                new Vector3(-1, 0, 0)));
                // y
                dan = Min(dan, DanToPlane(point, new Vector3(0, (float)_Rules.arena.goal_height, 0), new Vector3(0, -1, 0)));
            }

            // Goal back corners
            //assert _Rules.arena.bottom_radius == _Rules.arena.goal_top_radius //TODO
            if (point.Z > (_Rules.arena.depth / 2) + _Rules.arena.goal_depth - _Rules.arena.bottom_radius)
                dan = Min(dan, DanToSphereInner(
                point,
                new Vector3(
                (float)System.Math.Clamp(
                point.X,
                _Rules.arena.bottom_radius - (_Rules.arena.goal_width / 2),
                (_Rules.arena.goal_width / 2) - _Rules.arena.bottom_radius
                ),
                (float)System.Math.Clamp(
                point.Y,
                _Rules.arena.bottom_radius,
                _Rules.arena.goal_height - _Rules.arena.goal_top_radius
                ),
                ((float)_Rules.arena.depth / 2) + (float)_Rules.arena.goal_depth - (float)_Rules.arena.bottom_radius),
                (float)_Rules.arena.bottom_radius));

            // Corner
            if (point.X > (_Rules.arena.width / 2) - _Rules.arena.corner_radius
            && point.Z > (_Rules.arena.depth / 2) - _Rules.arena.corner_radius)
                dan = Min(dan, DanToSphereInner(
                point,
                new Vector3(
                ((float)_Rules.arena.width / 2) - (float)_Rules.arena.corner_radius,
                point.Y,
                ((float)_Rules.arena.depth / 2) - (float)_Rules.arena.corner_radius
                ),
                (float)_Rules.arena.corner_radius));

            // Goal outer corner
            if (point.Z < (_Rules.arena.depth / 2) + _Rules.arena.goal_side_radius)
            {
                // Side x
                if (point.X < (_Rules.arena.goal_width / 2) + _Rules.arena.goal_side_radius)
                    dan = Min(dan, DanToSphereOuter(
                    point,
                    new Vector3(
                    ((float)_Rules.arena.goal_width / 2) + (float)_Rules.arena.goal_side_radius,
                    point.Y,
                    ((float)_Rules.arena.depth / 2) + (float)_Rules.arena.goal_side_radius
                    ),
                    (float)_Rules.arena.goal_side_radius));

                // Ceiling
                if (point.Y < _Rules.arena.goal_height + _Rules.arena.goal_side_radius)
                    dan = Min(dan, DanToSphereOuter(
                    point,
                    new Vector3(
                    point.X,
                    (float)_Rules.arena.goal_height + (float)_Rules.arena.goal_side_radius,
                    ((float)_Rules.arena.depth / 2) + (float)_Rules.arena.goal_side_radius
                    ),
                    (float)_Rules.arena.goal_side_radius));
                // Top corner
                var o = new Vector2(
                ((float)_Rules.arena.goal_width / 2) - (float)_Rules.arena.goal_top_radius,
                (float)_Rules.arena.goal_height - (float)_Rules.arena.goal_top_radius
                );
                var v2 = new Vector2(point.X, point.Y) - o;
                if (v2.X > 0 && v2.Y > 0)
                {
                    var o2 = o + Vector2.Normalize(v2) * (float)(_Rules.arena.goal_top_radius + _Rules.arena.goal_side_radius);
                    dan = Min(dan, DanToSphereOuter(
                    point,
                    new Vector3(o2.X, o2.Y, ((float)_Rules.arena.depth / 2) + (float)_Rules.arena.goal_side_radius),
                    (float)_Rules.arena.goal_side_radius));
                }
            }

            // Goal inside top corners
            if (point.Z > (_Rules.arena.depth / 2) + _Rules.arena.goal_side_radius
            && point.Y > _Rules.arena.goal_height - _Rules.arena.goal_top_radius)
            {
                // Side x
                if (point.X > (_Rules.arena.goal_width / 2) - _Rules.arena.goal_top_radius)
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(
                    ((float)_Rules.arena.goal_width / 2) - (float)_Rules.arena.goal_top_radius,
                    (float)_Rules.arena.goal_height - (float)_Rules.arena.goal_top_radius,
                    point.Z
                    ),
                    (float)_Rules.arena.goal_top_radius));
                // Side z
                if (point.Z > (_Rules.arena.depth / 2) + _Rules.arena.goal_depth - _Rules.arena.goal_top_radius)
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(
                    point.X,
                    (float)_Rules.arena.goal_height - (float)_Rules.arena.goal_top_radius,
                    ((float)_Rules.arena.depth / 2) + (float)_Rules.arena.goal_depth - (float)_Rules.arena.goal_top_radius
                    ),
                    (float)_Rules.arena.goal_top_radius));
            }

            // Bottom corners
            if (point.Y < _Rules.arena.bottom_radius)
            {
                // Side x
                if (point.X > (_Rules.arena.width / 2) - _Rules.arena.bottom_radius)
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(
                    ((float)_Rules.arena.width / 2) - (float)_Rules.arena.bottom_radius,
                    (float)_Rules.arena.bottom_radius,
                    point.Z
                    ),
                    (float)_Rules.arena.bottom_radius));

                // Side z
                if (point.Z > (_Rules.arena.depth / 2) - _Rules.arena.bottom_radius
                && point.X >= (_Rules.arena.goal_width / 2) + _Rules.arena.goal_side_radius)
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(
                    point.X,
                    (float)_Rules.arena.bottom_radius,
                    ((float)_Rules.arena.depth / 2) - (float)_Rules.arena.bottom_radius
                    ),
                    (float)_Rules.arena.bottom_radius));

                // Side z (goal)
                if (point.Z > (_Rules.arena.depth / 2) + _Rules.arena.goal_depth - _Rules.arena.bottom_radius)
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(
                    point.X,
                    (float)_Rules.arena.bottom_radius,
                    ((float)_Rules.arena.depth / 2) + (float)_Rules.arena.goal_depth - (float)_Rules.arena.bottom_radius
                    ),
                    (float)_Rules.arena.bottom_radius));

                // Goal outer corner
                var o = new Vector2(
                ((float)_Rules.arena.goal_width / 2) + (float)_Rules.arena.goal_side_radius,
                ((float)_Rules.arena.depth / 2) + (float)_Rules.arena.goal_side_radius
                );
                var v2 = new Vector2(point.X, point.Z) - o;
                if (v2.X < 0 && v2.Y < 0
                && v2.Length() < _Rules.arena.goal_side_radius + _Rules.arena.bottom_radius)
                {
                    var o2 = o + Vector2.Normalize(v2) * ((float)_Rules.arena.goal_side_radius + (float)_Rules.arena.bottom_radius);
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(o2.X, (float)_Rules.arena.bottom_radius, o2.Y),
                    (float)_Rules.arena.bottom_radius));
                }

                // Side x (goal)
                if (point.Z >= (_Rules.arena.depth / 2) + _Rules.arena.goal_side_radius
                && point.X > (_Rules.arena.goal_width / 2) - _Rules.arena.bottom_radius)
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(
                    ((float)_Rules.arena.goal_width / 2) - (float)_Rules.arena.bottom_radius,
                    (float)_Rules.arena.bottom_radius,
                    point.Z
                    ),
                    (float)_Rules.arena.bottom_radius));

                // Corner
                if (point.X > (_Rules.arena.width / 2) - _Rules.arena.corner_radius
                && point.Z > (_Rules.arena.depth / 2) - _Rules.arena.corner_radius)
                {
                    var corner_o = new Vector2(
                    ((float)_Rules.arena.width / 2) - (float)_Rules.arena.corner_radius,
                    ((float)_Rules.arena.depth / 2) - (float)_Rules.arena.corner_radius
                    );
                    var n = new Vector2(point.X, point.Z) - corner_o;
                    var dist = n.Length();
                    if (dist > _Rules.arena.corner_radius - _Rules.arena.bottom_radius)
                    {
                        var n2 = n / dist;
                        var o2 = corner_o + n2 * ((float)_Rules.arena.corner_radius - (float)_Rules.arena.bottom_radius);
                        dan = Min(dan, DanToSphereInner(
                        point,
                        new Vector3(o2.X, (float)_Rules.arena.bottom_radius, o2.Y),
                        (float)_Rules.arena.bottom_radius));
                    }
                }
            }

            // Ceiling corners
            if (point.Y > _Rules.arena.height - _Rules.arena.top_radius)
            {
                // Side x
                if (point.X > (_Rules.arena.width / 2) - _Rules.arena.top_radius)
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(
                    ((float)_Rules.arena.width / 2) - (float)_Rules.arena.top_radius,
                    (float)_Rules.arena.height - (float)_Rules.arena.top_radius,
                    point.Z
                    ),
                    (float)_Rules.arena.top_radius));

                // Side z
                if (point.Z > (_Rules.arena.depth / 2) - _Rules.arena.top_radius)
                    dan = Min(dan, DanToSphereInner(
                    point,
                    new Vector3(
                    point.X,
                    (float)_Rules.arena.height - (float)_Rules.arena.top_radius,
                    ((float)_Rules.arena.depth / 2) - (float)_Rules.arena.top_radius
                    ),
                    (float)_Rules.arena.top_radius));

                // Corner
                if (point.X > (_Rules.arena.width / 2) - _Rules.arena.corner_radius
                && point.Z > (_Rules.arena.depth / 2) - _Rules.arena.corner_radius)
                {
                    var corner_o = new Vector2(
                    ((float)_Rules.arena.width / 2) - (float)_Rules.arena.corner_radius,
                    ((float)_Rules.arena.depth / 2) - (float)_Rules.arena.corner_radius
                    );
                    var dv = new Vector2(point.X, point.Z) - corner_o;
                    if (dv.Length() > _Rules.arena.corner_radius - _Rules.arena.top_radius)
                    {
                        var n = Vector2.Normalize(dv);
                        var o2 = corner_o + n * ((float)_Rules.arena.corner_radius - (float)_Rules.arena.top_radius);
                        dan = Min(dan, DanToSphereInner(
                        point,
                        new Vector3(o2.X, (float)_Rules.arena.height - (float)_Rules.arena.top_radius, o2.Y),
                        (float)_Rules.arena.top_radius));
                    }
                }
            }

            return dan;
        }

        private (float dist, Vector3 normal) Min((float dist, Vector3 normal) dn1, (float dist, Vector3 normal) dn2) => dn1.dist < dn2.dist ? dn1 : dn2;

        private (float dist, Vector3 normal) DanToArena(Vector3 point)
        {
            var negate_x = point.X < 0;
            var negate_z = point.Z < 0;
            if (negate_x)
                point.X = -point.X;

            if (negate_z)
                point.Z = -point.Z;

            var result = DanToArenaQuarter(point);
            if (negate_x)
                result.normal.X = -result.normal.X;

            if (negate_z)
                result.normal.Z = -result.normal.Z;

            return result;
        }

        private Vector3 Clamp(Vector3 v, float limit) => v.Length() > limit ? limit * Vector3.Normalize(v) : v;//TODO
    }
}
