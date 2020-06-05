using NetTopologySuite.Algorithm;
using NetTopologySuite.Algorithm.Distance;
using NetTopologySuite.Geometries;
using NetTopologySuite.Geometries.Utilities;
using NetTopologySuite.Mathematics;
using NetTopologySuite.Operation.Buffer;
using NetTopologySuite.Simplify;
using System;
using System.Collections.Generic;
using System.Linq;
using QuickGraph;
using QuickGraph.Algorithms.ShortestPath;
using QuickGraph.Algorithms.Observers;

namespace PathFinder
{
    /// <summary>
    ///  Obstacle avoidance shortest path finder, including the direct shortest path and the shortest path along the obstacles
    /// </summary>
    public partial class ShortestAvoidancePathFinder
    {
        private const double twoPI = Math.PI * 2;

        // Too little precision can lead to errors in calculations.
        public const double minBufferDistance = 0.00001;

        public const double defaultSimplifyTolerance = 0.01;

        // Make sure there is only one vertex per corner.
        public const double defaultMitreLimit = 1e8;

        private const double tol = 1e-8;

        private Geometry originGeometry;

        private bool IsBuilded = false;

        private BidirectionalGraph<ObstacleGraphPoint, GraphEdge> graph = new BidirectionalGraph<ObstacleGraphPoint, GraphEdge>();

        /// <summary>
        /// Parameter about bufferOp.
        /// </summary>
        public BufferParameters BufferParameter { get; set; }

        private double bufferDistance = minBufferDistance;
        /// <summary>
        /// The buffer distance of all obstacles(polygon and polyline)
        /// </summary>
        public double BufferDistance
        {
            get => bufferDistance;
            set 
            {
                bufferDistance = value < minBufferDistance ? minBufferDistance : value;
                BufferParameter.MitreLimit = defaultMitreLimit;
            } 
        }

        /// <summary>
        /// Tolerance of VWSimplifier.
        /// </summary>
        public double SimplifyTolerance { get; set; } = defaultSimplifyTolerance;

        /// <summary>
        /// The obstacles after bufferOp.
        /// </summary>
        public Geometry Buffer { get; private set; }

        /// <summary>
        /// The behavior that should be performed when the starting or ending point of a path is inside an obstacle.
        /// </summary>
        public BreakThroughObstaclesBahavior InternalPointBehavior { get; set; } = BreakThroughObstaclesBahavior.NearestObstacleEdge;

        private ObstacleGraphDatas obstacleDatas;

        private RobustLineIntersector lineIntersector;

        /// <summary>
        /// Use a set of obstacles to initialize the pathfinder.
        /// </summary>
        /// <param name="geom"></param>
        public ShortestAvoidancePathFinder(Geometry geom)
        {
            originGeometry = geom;
            BufferParameter = new BufferParameters(2, EndCapStyle.Flat, JoinStyle.Mitre, defaultMitreLimit);
            lineIntersector = new RobustLineIntersector();
        }

        /// <summary>
        /// Build the visual graph. It can be cached so that subsequent pathfinding does not need to be built again.
        /// </summary>
        public void BuildVisualGraph()
        {
            ParseObstacleDatas(originGeometry);

            foreach (var origin in obstacleDatas.Points)
            {
                BuildVisualGraphCore(origin, obstacleDatas.Points);
            }

            IsBuilded = true;
        }

        /// <summary>
        /// Find the direct shortest path according to the specified starting and ending points.
        /// </summary>
        /// <param name="start">the specified starting point</param>
        /// <param name="end">the specified ending point</param>
        /// <returns>the direct shortest path</returns>
        public List<LineSegment> FindShortestPath(Coordinate start, Coordinate end)
        {
            return FindShortestPathCore(start, end).Select(e => new LineSegment(e.Source.Point, e.Target.Point)).ToList();
        }

        /// <summary>
        /// Find the shortest path along the obstacles according to the specified starting and ending points.
        /// The orthogonal direction of the obstacles gap will automatically be resolved to the direction with the largest number of obstacle edges.
        /// </summary>
        /// <param name="start">the specified starting point</param>
        /// <param name="end">the specified ending point</param>
        /// <returns>the shortest path along the obstacles</returns>
        public List<LineSegment> FindAlongShortestPath(Coordinate start, Coordinate end)
        {
            var edges = FindShortestPath(start, end);
            var maxGroupKey = obstacleDatas.Lines.GroupBy(ol => new Vector2D(ol.Line.P0, ol.Line.P1).Normalize()).MaxBy(g => g.Count()).Key;
            edges = AdjustToAlongPath(edges, maxGroupKey);
            return AdjustToAlongPath(edges, maxGroupKey);
        }

        /// <summary>
        /// Find the shortest path along the obstacles according to the specified starting and ending points.
        /// </summary>
        /// <param name="start">the specified starting point</param>
        /// <param name="end">the specified ending point</param>
        /// <param name="alignDirection">The orthogonal direction of the obstacles gap</param>
        /// <returns>the shortest path along the obstacles</returns>
        public List<LineSegment> FindAlongShortestPath(Coordinate start, Coordinate end, Vector2D alignDirection)
        {
            var edges = FindShortestPath(start, end);
            edges = AdjustToAlongPath(edges, alignDirection);
            return AdjustToAlongPath(edges, alignDirection);
        }

        /// <summary>
        /// Find the direct shortest path according to the specified starting and ending points.
        /// </summary>
        /// <param name="start">the specified starting point</param>
        /// <param name="end">the specified ending point</param>
        /// <returns>the direct shortest path points</returns>
        public List<Coordinate> FindShortestPathPoints(Coordinate start, Coordinate end)
        {
            var edges = FindShortestPathCore(start, end);
            edges.Add(edges.Last());
            return edges.Select(e => e.Source.Point).ToList();
        }

        /// <summary>
        /// Add a new obstacle to the original obstacle set. It will cause the next pathfinding to rebuild the visual graph.
        /// </summary>
        /// <param name="obstacle"></param>
        public void AddObstacle(Geometry obstacle)
        {
            if (obstacle == null || obstacle.IsEmpty)
                return;

            GeometryCombiner.SkipEmpty = true;
            originGeometry = GeometryCombiner.Combine(originGeometry, obstacle);
            IsBuilded = false;
        }

        /// <summary>
        /// Get the visual graph.
        /// </summary>
        /// <returns>the visual graph</returns>
        public List<LineSegment> GetVisualGraph()
        {
            if (!IsBuilded)
            {
                BuildVisualGraph();
            }

            return graph.Edges.Select(e => new LineSegment(e.Source.Point, e.Target.Point)).ToList();
        }

        /// <summary>
        /// Get the visual graph with the specified starting and ending points.
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <returns></returns>
        public List<LineSegment> GetVisualGraph(Coordinate start, Coordinate end)
        {
            if (!IsBuilded)
            {
                BuildVisualGraph();
            }

            List<ObstacleGraphPoint> tempPoints;
            var obstaclePoints = obstacleDatas.Points;
            var rawPoints = HandlePathEnds(start, end, out tempPoints);

            obstaclePoints.AddRange(tempPoints);
            tempPoints.ForEach(tp => BuildVisualGraphCore(tp, obstaclePoints, true));

            var result = graph.Edges.Select(e => new LineSegment(e.Source.Point, e.Target.Point)).ToList();

            tempPoints.ForEach(tp => graph.RemoveVertex(tp));
            rawPoints.ForEach(rp => graph.RemoveVertex(rp));

            if (tempPoints.Count > 0)
                obstaclePoints.RemoveRange(obstaclePoints.Count - tempPoints.Count, tempPoints.Count);

            return result;
        }

        private void ParseObstacleDatas(Geometry geom)
        {
            obstacleDatas = new ObstacleGraphDatas();
            var buffer = BufferOp.Buffer(geom, BufferDistance, BufferParameter);
            Buffer = VWSimplifier.Simplify(buffer, SimplifyTolerance);
            obstacleDatas.Obstacles = PolygonExtracter.GetPolygons(Buffer).OfType<Polygon>().ToList();
            foreach (var polygon in obstacleDatas.Obstacles)
            {
                foreach (LineString lineString in LinearComponentExtracter.GetLines(polygon))
                {
                    var points = lineString.Coordinates.ToList();
                    if (lineString.IsRing)
                        points.RemoveAt(lineString.NumPoints - 1);

                    var count = points.Count;
                    for (int i = 0; i < count; i++)
                    {
                        var preIndex = MathUtil.Wrap(i - 1, count);
                        var nextIndex = MathUtil.Wrap(i + 1, count);
                        var obstaclePoint = new ObstacleGraphPoint(polygon, points[preIndex], points[i], points[nextIndex]);
                        obstacleDatas.Points.Add(obstaclePoint);
                        obstacleDatas.Lines.Add(obstaclePoint.AdjacentLines[0]);
                    }
                }
            }
        }

        private void BuildVisualGraphCore(ObstacleGraphPoint origin, IEnumerable<ObstacleGraphPoint> obstaclePoints, bool isPathPoint = false)
        {
            var originPoint = origin.Point;
            var angleOrderedObstaclePoints = obstaclePoints.Where(op => op != origin).OrderBy(op =>
            {
                var angle = AngleUtility.Angle(originPoint, op.Point);
                if (angle < 0)
                    angle += twoPI;

                return angle;
            }/*, new DoubleComparer()*/).ThenBy(op => originPoint.Distance(op.Point));

            //var isLog = false;
            //if (originPoint.Equals2D(new Coordinate(65, 685.6), 0.01))
            //{
            //    isLog = true;
            //}
            //else
            //    isLog = false;

            var crashEdges = new PossibleCrashEdges();
            var ray = new LineSegment(originPoint, new Coordinate(1e6, originPoint.Y));
            var crashLines = obstacleDatas.Lines.Where(ol => IsIntersectProper(ol.Line, ray)
                && !CollinearWithEps(ray.P0, ol.Line.P0, ol.Line.P1));

            foreach (var crashLine in crashLines)
            {
                crashLine.CorrespondingRay = ray;
                crashEdges.Add(crashLine);
            }

            bool preVisible = false;
            Coordinate preObstaclePoint = null;
            foreach (var obstaclePoint in angleOrderedObstaclePoints)
            {
                //if (originPoint.Equals2D(new Coordinate(65, 685.6), 0.01) &&
                //    obstaclePoint.Point.Equals2D(new Coordinate(40.2, 83.2), 0.01))
                //{

                //}

                var visible = false;
                var rotateLineSegment = new LineSegment(originPoint, obstaclePoint.Point);
                var rotateLine = new LineString(new Coordinate[] { rotateLineSegment.P0, rotateLineSegment.P1 });

                if (obstaclePoint.Polygon.IsEmpty || !IsLineIntersectPolygonInterior(rotateLine, obstaclePoint.Polygon))
                {
                    if (preObstaclePoint == null || 
                        /*Orientation.Index(originPoint, preObstaclePoint, obstaclePoint.Point) != OrientationIndex.Collinear*/
                        !CollinearWithEps(originPoint, preObstaclePoint, obstaclePoint.Point))
                    {
                        var firstCrashEdge = crashEdges.TakeMin()?.Line;
                        if (firstCrashEdge == null || !IsIntersectProper(firstCrashEdge, rotateLineSegment))
                            visible = true;
                    }
                    else if (preVisible)
                    {
                        var intervalLine = new LineSegment(preObstaclePoint, obstaclePoint.Point);
                        if (!crashEdges.Any(ol => IsIntersectProper(ol.Line, intervalLine)))
                        {
                            visible = true;
                        }
                    }
                }

                if (visible)
                {
                    graph.AddVerticesAndEdge(new GraphEdge(origin, obstaclePoint));

                    if (isPathPoint)
                        graph.AddVerticesAndEdge(new GraphEdge(obstaclePoint, origin));
                }

                var addCrashEdgeLines = new List<ObstacleGraphLine>(2);
                foreach (var line in obstaclePoint.AdjacentLines)
                {
                    if (CollinearWithEps(originPoint, obstaclePoint.Point, line.Line.P1))
                        continue;

                    line.CorrespondingRay = rotateLineSegment;
                    var clockType = Orientation.Index(originPoint, obstaclePoint.Point, line.Line.P1);
                    if (clockType == OrientationIndex.CounterClockwise)
                    {
                        addCrashEdgeLines.Add(line);
                    }
                    else if (clockType == OrientationIndex.Clockwise)
                    {
                        crashEdges.Remove(line);
                        //var result = crashEdges.Remove(line);
                        //if (!result && isLog)
                        //{
                        //    LogManager.Write($"{line.Line} remove failed! ");
                        //    LogManager.Write($"The current obstacle point is {obstaclePoint.Point}");
                        //    var txt = "The current collision line set is";
                        //    var geoms = GeometryCombiner.Combine(crashEdges.edges.Select(e => e.Line.ToGeometry(new GeometryFactory())));

                        //    if (geoms != null)
                        //        LogManager.Write(txt + geoms.ToString());
                        //}
                    }
                }

                addCrashEdgeLines.ForEach(l => crashEdges.Add(l));

                preVisible = visible;
                preObstaclePoint = obstaclePoint.Point;
            }

            bool IsLineIntersectPolygonInterior(LineString line, Polygon polygon)
            {
                var matrix = line.Relate(polygon);
                return !(matrix.IsTouches(line.Dimension, polygon.Dimension) || matrix.IsDisjoint());
            }

            bool CollinearWithEps(Coordinate p1, Coordinate p2, Coordinate p3)
            {
                var angle = AngleUtility.AngleBetween(p1, p2, p3);
                return angle.EqualsWithEps(0, tol) || angle.EqualsWithEps(Math.PI, tol);
            }
        }

        private List<ObstacleGraphPoint> HandlePathEnds(Coordinate start, Coordinate end,
             out List<ObstacleGraphPoint> tempPoints)
        {
            var rawPoints = new List<ObstacleGraphPoint>(2);
            tempPoints = new List<ObstacleGraphPoint>();
            foreach (var p in new Coordinate[] { start, end })
            {
                var obstaclePoint = new ObstacleGraphPoint(p);
                rawPoints.Add(obstaclePoint);

                if(InternalPointBehavior == BreakThroughObstaclesBahavior.None)
                {
                    tempPoints.Add(obstaclePoint);
                    continue;
                }

                var point = new Point(p);
                var externalPolygon = obstacleDatas.Obstacles.FirstOrDefault(op => op.Contains(point));

                if(externalPolygon == null)
                {
                    tempPoints.Add(obstaclePoint);
                    continue;
                }

                if (InternalPointBehavior == BreakThroughObstaclesBahavior.NearestObstacleEdge)
                {
                    var ptDist = new PointPairDistance();
                    DistanceToPoint.ComputeDistance(externalPolygon, p, ptDist);
                    var targetPoint = ptDist[0];
                    if(externalPolygon.Coordinates.Any(c => c.Equals2D(targetPoint, tol)))
                    {
                        var targetObstaclePoint = obstacleDatas.Points.First(op => op.Point.Equals2D(targetPoint, tol));
                        AddToGraph(obstaclePoint, targetObstaclePoint);
                        //AddToGraph(obstaclePoint, new ObstaclePoint(targetPoint));
                    }
                    else
                    {
                        targetPoint = GetOffsetAdjustPoint(p, targetPoint, false);
                        var targetObstaclePoint = new ObstacleGraphPoint(targetPoint);
                        AddToGraph(obstaclePoint, targetObstaclePoint);
                        tempPoints.Add(targetObstaclePoint);
                    }
                    continue;
                }

                if (InternalPointBehavior == BreakThroughObstaclesBahavior.NearestObstaclePoint)
                {
                    var nearestPoint = externalPolygon.Coordinates.OrderBy(c => c.Distance(p)).FirstOrDefault();
                    if (nearestPoint != null)
                        AddToGraph(obstaclePoint, obstacleDatas.Points.First(op => op.Point.Equals2D(nearestPoint, tol)));
                    //AddToGraph(obstaclePoint, new ObstaclePoint(nearestPoint));
                    continue;
                }

                if (InternalPointBehavior == BreakThroughObstaclesBahavior.ShortestDistanceObstacleEdge)
                {
                    var lineStrings = LinearComponentExtracter.GetLines(externalPolygon).OfType<LineString>();
                    foreach (var lineString in lineStrings)
                    {
                        var points = lineString.Coordinates;
                        for (int i = 0; i < points.Length - 1; i++)
                        {
                            var nearestPoint = new LineSegment(points[i], points[i + 1]).ClosestPoint(p);
                            if (nearestPoint.Equals2D(p, tol))
                            {
                                //coos.Add(GetOffsetAdjustPoint(p, nearestPoint, false));
                                tempPoints.Add(obstaclePoint);
                            }
                            else
                            {
                                var testPoint = GetOffsetAdjustPoint(p, nearestPoint, true);
                                if (externalPolygon.Contains(new LineString(new Coordinate[] { testPoint, p })))
                                {
                                    if(nearestPoint.Equals2D(points[i], tol) || nearestPoint.Equals2D(points[i + 1], tol))
                                    {
                                        //AddToGraph(obstaclePoint, obstacleDatas.Points.First(op => op.Point.Equals2D(nearestPoint, tol)));
                                        AddToGraph(obstaclePoint, new ObstacleGraphPoint(nearestPoint));
                                        // 待重构
                                    }
                                    else
                                    {
                                        var targetPoint = GetOffsetAdjustPoint(p, nearestPoint, false);
                                        var targetObstaclePoint = new ObstacleGraphPoint(targetPoint);
                                        AddToGraph(obstaclePoint, targetObstaclePoint);
                                        tempPoints.Add(targetObstaclePoint);
                                    }
                                }
                            }
                        }
                    }
                    continue;
                }
            }

            return rawPoints;

            void AddToGraph(ObstacleGraphPoint p1, ObstacleGraphPoint p2)
            {
                graph.AddVerticesAndEdge(new GraphEdge(p1, p2));
                graph.AddVerticesAndEdge(new GraphEdge(p2, p1));
            }
        }

        private List<GraphEdge> FindShortestPathCore(Coordinate start, Coordinate end)
        {
            if (!IsBuilded)
            {
                BuildVisualGraph();
            }

            List<ObstacleGraphPoint> tempPoints;
            var obstaclePoints = obstacleDatas.Points;
            var rawPoints = HandlePathEnds(start, end, out tempPoints);

            obstaclePoints.AddRange(tempPoints);
            tempPoints.ForEach(tp => BuildVisualGraphCore(tp, obstaclePoints, true));

            var algorithm = new DijkstraShortestPathAlgorithm<ObstacleGraphPoint, GraphEdge>(graph, e => e.Length);
            var observer = new VertexPredecessorRecorderObserver<ObstacleGraphPoint, GraphEdge>();
            try
            {
                using (observer.Attach(algorithm))
                    algorithm.Compute(rawPoints[0]);
            }
            catch
            {
                return new List<GraphEdge>();
            }

            tempPoints.ForEach(tp => graph.RemoveVertex(tp));
            rawPoints.ForEach(rp => graph.RemoveVertex(rp));

            if (tempPoints.Count > 0)
                obstaclePoints.RemoveRange(obstaclePoints.Count - tempPoints.Count, tempPoints.Count);

            if (observer.TryGetPath(rawPoints[1], out IEnumerable<GraphEdge> path))
            {
                return path.ToList();
            }

            //return observer.VertexPredecessors.ToList().Select(p => new LineSegment(p.Value.Source.Point, p.Value.Target.Point)).ToList();
            return new List<GraphEdge>();
        }

        private bool IsIntersectProper(LineSegment l1, LineSegment l2)
        {
            lineIntersector.ComputeIntersection(l1.P0, l1.P1, l2.P0, l2.P1);
            return lineIntersector.IsProper;
        }

        // adjust break through point location so that it could be adapt to tolerance.
        private Coordinate GetOffsetAdjustPoint(Coordinate refer, Coordinate adjust, bool isInner, double eps = tol)
        {
            var offsetVec = new Vector2D(refer, adjust).Normalize().Multiply(eps);
            if (isInner)
                offsetVec = offsetVec.Negate();
            return new Vector2D(adjust).Add(offsetVec).ToCoordinate();
        }

        private List<LineSegment> AdjustToAlongPath(List<LineSegment> edges, Vector2D alignDirection)
        {
            GeometryCombiner.SkipEmpty = true;
            var resultLines = new List<LineSegment>();
            foreach (var edge in edges)
            {
                var start = edge.P0;
                var end = edge.P1;
                var lineDirection = new Vector2D(start, end);
                if((lineDirection.AngleTo(alignDirection) % (Math.PI / 2)).EqualsWithEps(0, tol))
                {
                    resultLines.Add(new LineSegment(start, end));
                    continue;
                }

                var orthProjectVec = alignDirection.Multiply(lineDirection.Dot(alignDirection));
                var orthComponentVec = lineDirection.Subtract(orthProjectVec);

                var recPoints = new Coordinate[]
                {
                    start,
                    orthProjectVec.Add(new Vector2D(start)).ToCoordinate(),
                    end,
                    orthComponentVec.Add(new Vector2D(start)).ToCoordinate(),
                    start
                };
                var rec = new Polygon(new LinearRing(recPoints));

                //List<Polygon> obstaclePolygons;
                //if (edge.Source.Polygon.IsEmpty || edge.Target.Polygon.IsEmpty)
                //    obstaclePolygons = obstacleDatas.Obstacles.Where(o => o.Overlaps(rec)).ToList();
                //else
                //{
                //    obstaclePolygons = new List<Polygon>(2);
                //    if (!edge.Source.Polygon.IsEmpty)
                //        obstaclePolygons.Add(edge.Source.Polygon);
                //    if (!edge.Target.Polygon.IsEmpty && edge.Target.Polygon != edge.Source.Polygon)
                //        obstaclePolygons.Add(edge.Target.Polygon);
                //}

                var obstaclePolygons = obstacleDatas.Obstacles.Where(o => o.Overlaps(rec)).ToList();

                var differ = rec.Difference(GeometryCombiner.Combine(obstaclePolygons));
                if(differ == null || differ.IsEmpty)
                {
                    resultLines.Add(new LineSegment(start, end));
                    continue;
                }

                var differPolygon = PolygonExtracter.GetPolygons(differ).OfType<Polygon>().Select(p => p.Shell).MaxBy(s => s.Area);
                if (differPolygon == null)
                {
                    throw new Exception("shouldn't come here.");
                }

                var shellPoints = differPolygon.Coordinates.ToList();
                shellPoints.RemoveAt(shellPoints.Count - 1);

                var startIndex = shellPoints.FindIndex(sp => sp.Equals2D(start, tol));
                var endIndex = shellPoints.FindIndex(sp => sp.Equals2D(end, tol));
                var indexDiffer = Math.Abs(startIndex - endIndex);

                if (obstaclePolygons.Count == 1 && (indexDiffer == 1 || indexDiffer == shellPoints.Count - 1))
                {
                    resultLines.Add(new LineSegment(start, end));
                    continue;
                }

                if (startIndex == -1 || endIndex == -1)
                {
                    resultLines.Add(new LineSegment(start, end));
                    continue;
                }

                var paths = shellPoints.GetDoubleDirectionAlongElementsExcludeEnds(startIndex, endIndex);

                List<Coordinate> middlePath;
                if (paths.All(p => p.Count == 1) && obstaclePolygons.Count == 1)
                {
                    middlePath = paths.MinBy(p => obstaclePolygons[0].Distance(new
                        Point(new Vector2D(start).Add(new Vector2D(p[0])).Add(new Vector2D(end)).Divide(3).ToCoordinate())));
                }
                else
                    middlePath = paths.MaxBy(p => p.Count);

                if(middlePath.Count == 0)
                {
                    resultLines.Add(new LineSegment(start, end));
                    continue;
                }
                else if(middlePath.Count == 1)
                {
                    resultLines.Add(new LineSegment(start, middlePath[0]));
                    resultLines.Add(new LineSegment(middlePath[0], end));
                    continue;
                }
                else
                {
                    var secondStartIndex = 0;
                    var startDistance = double.MaxValue;
                    var secondEndIndex = 0;
                    var endDistance = double.MaxValue;
                    for (int i = 0; i < middlePath.Count; i++)
                    {
                        var sDistance = middlePath[i].Distance(start);
                        if (sDistance < startDistance)
                        {
                            secondStartIndex = i;
                            startDistance = sDistance;
                        }
                        var nDistance = middlePath[i].Distance(end);
                        if (nDistance < endDistance)
                        {
                            secondEndIndex = i;
                            endDistance = nDistance;
                        }
                    }

                    resultLines.Add(new LineSegment(start, middlePath[secondStartIndex]));

                    for (int i = secondStartIndex; i < secondEndIndex; i++)
                    {
                        resultLines.Add(new LineSegment(middlePath[i], middlePath[i+ 1]));
                    }

                    resultLines.Add(new LineSegment(middlePath[secondEndIndex], end));
                }
            }

            return resultLines;
        }
    }
}
