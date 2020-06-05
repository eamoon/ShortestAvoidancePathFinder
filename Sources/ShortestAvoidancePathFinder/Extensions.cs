using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFinder
{
    public static class Extensions
    {
        public static bool EqualsWithEps(this double value, double another, double eps)
        {
            return Math.Abs(value - another) < eps;
        }

        public static List<List<T>> GetDoubleDirectionAlongElements<T>(this IList<T> elements, int startIndex, int endIndex)
        {
            var count = elements.Count;

            var ptr = startIndex;
            var forwardList = new List<T>();
            int current;
            do
            {
                forwardList.Add(elements[ptr]);
                current = ptr;
                ptr = (ptr + 1 + count) % count;
            }
            while (current != endIndex);

            ptr = startIndex;
            var backwardList = new List<T>();
            do
            {
                backwardList.Add(elements[ptr]);
                current = ptr;
                ptr = (ptr - 1 + count) % count;
            }
            while (current != endIndex);

            return new List<List<T>> { forwardList, backwardList };
        }

        public static List<List<T>> GetDoubleDirectionAlongElementsExcludeEnds<T>(this IList<T> elements, int startIndex, int endIndex)
        {
            var count = elements.Count;

            var ptr = startIndex;
            var forwardList = new List<T>();
            while (true)
            {
                ptr = (ptr + 1 + count) % count;
                if (ptr == endIndex)
                    break;
                forwardList.Add(elements[ptr]);
            }

            ptr = startIndex;
            var backwardList = new List<T>();
            while (true)
            {
                ptr = (ptr - 1 + count) % count;
                if (ptr == endIndex)
                    break;
                backwardList.Add(elements[ptr]);
            }

            return new List<List<T>> { forwardList, backwardList };
        }

        public static TSource MinBy<TSource, TKey>(this IEnumerable<TSource> sources,
              Func<TSource, TKey> selector, IComparer<TKey> comparer = null)
        {
            return sources.MinByKey(selector, comparer).Source;
        }

        public static Pair<TSource, TKey> MinByKey<TSource, TKey>(this IEnumerable<TSource> sources,
            Func<TSource, TKey> selector, IComparer<TKey> comparer = null)
        {
            var min = sources.First();

            if (comparer == null)
                comparer = Comparer<TKey>.Default;

            TKey minKey = selector(min);
            foreach (var source in sources)
            {
                var sourceKey = selector(source);
                if (comparer.Compare(sourceKey, minKey) < 0)
                {
                    min = source;
                    minKey = sourceKey;
                }
            }

            return new Pair<TSource, TKey>(min, minKey);
        }

        public static TSource MaxBy<TSource, TKey>(this IEnumerable<TSource> sources,
            Func<TSource, TKey> selector, IComparer<TKey> comparer = null)
        {
            return sources.MaxByKey(selector, comparer).Source;
        }

        public static Pair<TSource, TKey> MaxByKey<TSource, TKey>(this IEnumerable<TSource> sources,
             Func<TSource, TKey> selector, IComparer<TKey> comparer = null)
        {
            var max = sources.First();

            if (comparer == null)
                comparer = Comparer<TKey>.Default;

            TKey maxKey = selector(max);
            foreach (var source in sources)
            {
                var sourceKey = selector(source);
                if (comparer.Compare(sourceKey, maxKey) > 0)
                {
                    max = source;
                    maxKey = sourceKey;
                }
            }

            return new Pair<TSource, TKey>(max, maxKey);
        }
    }
}
