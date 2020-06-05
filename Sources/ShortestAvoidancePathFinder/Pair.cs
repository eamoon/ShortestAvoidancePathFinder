using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFinder
{
    public class Pair<TSource, TKey>
    {
        public TSource Source { get; set; }

        public TKey Key { get; set; }

        public Pair(TSource source, TKey key)
        {
            Source = source;
            Key = key;
        }
    }
}
