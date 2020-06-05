using NetTopologySuite.Algorithm;
using NetTopologySuite.Geometries;
using System;
using System.Collections.Generic;
using System.Linq;
using QuickGraph;

namespace PathFinder
{
    /// <summary>
    /// The behavior that should be performed when the starting or ending point of a path is inside an obstacle.
    /// </summary>
    public enum BreakThroughObstaclesBahavior
    {
        /// <summary>
        /// No action
        /// </summary>
        None,
        /// <summary>
        /// Perpendicular to the nearest obstacle edge. If it is the edge of the obstacle hole, the pathfinding will fail
        /// </summary>
        NearestObstacleEdge,
        /// <summary>
        /// Perpendicular to an obstacle edge, making the total path distance shortest
        /// </summary>
        ShortestDistanceObstacleEdge,
        /// <summary>
        /// Connect to nearest obstacle point.
        /// </summary>
        NearestObstaclePoint
    }

    public partial class ShortestAvoidancePathFinder
    {
        class ObstacleGraphDatas
        {
            public List<Polygon> Obstacles { get; set; } = new List<Polygon>();

            public List<ObstacleGraphPoint> Points { get; set; } = new List<ObstacleGraphPoint>();

            public List<ObstacleGraphLine> Lines { get; set; } = new List<ObstacleGraphLine>();
        }

        class PossibleCrashEdges
        {
            // red black tree
            internal SortedSet<ObstacleGraphLine> edges = new SortedSet<ObstacleGraphLine>();

            public void Add(ObstacleGraphLine line)
            {
                edges.Add(line);
            }

            public ObstacleGraphLine TakeMin()
            {
                return edges.Min;
            }

            public bool Remove(ObstacleGraphLine line)
            {
                return edges.Remove(line);
            }

            public bool Any(Func<ObstacleGraphLine, bool> predicate)
            {
                return edges.Any(e => predicate(e));
            }
        }

        class ObstacleGraphLine : IComparable<ObstacleGraphLine>
        {
            private static int globalId = 0;
            public int Id { get; set; }

            public LineSegment Line { get; set; }

            public LineSegment CorrespondingRay { get; set; }

            public ObstacleGraphLine(Coordinate p1, Coordinate p2)
            {
                Line = new LineSegment(p1, p2);
                Id = globalId++;
            }

            public int CompareTo(ObstacleGraphLine other)
            {
                if (Id == other.Id || EqualsTopologicallyWithTolerance(Line, other.Line))
                    return 0;

                var rayP0 = CorrespondingRay.P0;
                var rayP1 = CorrespondingRay.P1;
                var param = GetRayParameter(rayP0, rayP1, Line.P0, Line.P1);
                var otherParam = GetRayParameter(rayP0, rayP1, other.Line.P0, other.Line.P1);

                // You should allow for tolerance when comparing doubles
                if (param.EqualsWithEps(otherParam, tol))
                    return Comparer<double>.Default.Compare(AngleUtility.AngleBetween(rayP0, rayP1, GetAdjacentPoint(rayP1)),
                      AngleUtility.AngleBetween(rayP0, rayP1, other.GetAdjacentPoint(rayP1)));

                return Comparer<double>.Default.Compare(param, otherParam);
            }

            private double GetRayParameter(Coordinate ray0, Coordinate ray1, Coordinate lineP0, Coordinate lineP1)
            {
                var dirCrossValue = CrossValue(ray1.X - ray0.X, ray1.Y - ray0.Y, lineP1.X - lineP0.X, lineP1.Y - lineP0.Y);
                //if (dirCrossValue == 0)
                //    return double.NaN;

                if (dirCrossValue.EqualsWithEps(0, tol))
                    return double.NaN;

                var deltaCrossValue = CrossValue(lineP0.X - ray0.X, lineP0.Y - ray0.Y, lineP1.X - lineP0.X, lineP1.Y - lineP0.Y);
                return deltaCrossValue / dirCrossValue * ray0.Distance(ray1);


                double CrossValue(double x1, double y1, double x2, double y2)
                {
                    return x1 * y2 - y1 * x2;
                }
            }

            private bool EqualsTopologicallyWithTolerance(LineSegment l1, LineSegment l2)
            {
                return (l1.P0.Equals2D(l2.P0, tol) && l1.P1.Equals2D(l2.P1, tol)) || (l1.P0.Equals2D(l2.P1, tol) && l1.P1.Equals2D(l2.P0, tol));
            }

            private Coordinate GetAdjacentPoint(Coordinate point)
            {
                if (!Line.P0.Equals2D(point, tol))
                    return Line.P0;

                if (!Line.P1.Equals2D(point, tol))
                    return Line.P1;

                throw new ArgumentException("The specified point is not the end of the line segment. The adjacency point cannot be obtained!");
            }
        }

        class ObstacleGraphPoint : IEquatable<ObstacleGraphPoint>
        {
            public Polygon Polygon { get; set; }

            public ObstacleGraphLine[] AdjacentLines { get; set; }

            public Coordinate Point { get; set; }

            public ObstacleGraphPoint(Polygon polygon, Coordinate previousPoint, Coordinate point, Coordinate nextPoint)
            {
                Polygon = polygon;
                Point = point;
                AdjacentLines = new ObstacleGraphLine[2] { new ObstacleGraphLine(point, previousPoint), new ObstacleGraphLine(point, nextPoint) };
            }

            public ObstacleGraphPoint(Coordinate point)
            {
                AdjacentLines = new ObstacleGraphLine[0];
                Polygon = Polygon.Empty;
                Point = point;
            }

            public bool Equals(ObstacleGraphPoint other)
            {
                return Point.Equals2D(other.Point, tol);
            }
        }

        class GraphEdge : IEdge<ObstacleGraphPoint>
        {
            public double Length { get; }

            public ObstacleGraphPoint Source { get; }

            public ObstacleGraphPoint Target { get; }

            public GraphEdge(ObstacleGraphPoint source, ObstacleGraphPoint target)
            {
                Length = source.Point.Distance(target.Point);
                Source = source;
                Target = target;
            }
        }
    }
}
