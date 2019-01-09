using System.Collections.Generic;
using System.Linq;
using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeBall2018.Strategy
{
    internal class Match
    {
        public IEnumerable<Robot> Teammates { get; private set; }

        public Match(Game game)
        {
            Teammates = game.robots.Where(r => r.is_teammate).ToList();
        }
    }
}
