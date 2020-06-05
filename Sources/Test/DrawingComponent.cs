using PathFinder;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace Test
{
    public abstract class DrawingComponent : DrawingVisual
    {
        public Geometry Geometry { get; protected set; }

        public bool IsReadOnly { get; set; }

        public void Render(Pen pen, Transform transform = null, Brush brush = null)
        {
            if (transform != null)
                TransformGeometry(transform);

            using (var context = RenderOpen())
            {
                context.DrawGeometry(brush ?? DrawingCanvas.DefaultBrush, pen, Geometry);
            }
        }

        protected abstract void TransformGeometry(Transform transform);
    }

    public class DrawingPolygon : DrawingComponent
    {
        public List<Point> Shell { get; private set; }

        public List<List<Point>> Holes { get; set; }

        public bool IsValid => Shell != null;

        public bool HasHoles => Holes != null && Holes.Count > 0;

        public DrawingPolygon(NetTopologySuite.Geometries.Polygon polygon)
        {
            Update(polygon.Shell.Coordinates.Select(c => c.ToPoint()), polygon.Holes.Select(lr => lr.Coordinates.Select(c => c.ToPoint())));
        }

        public DrawingPolygon(IEnumerable<Point> shell, IEnumerable<IEnumerable<Point>> holes)
        {
            Update(shell, holes);
        }

        public DrawingPolygon(IEnumerable<Point> shell)
        {
            Update(shell);
        }

        public void Update(IEnumerable<Point> shell, IEnumerable<IEnumerable<Point>> holes)
        {
            if (shell == null || shell.Count() <= 2)
                return;

            Shell = shell.ToList();
            if (Shell.First() != Shell.Last())
                Shell.Add(Shell.First());

            Holes = holes.Select(h => h.ToList()).ToList();

            var pathGeom = new PathGeometry();
            var figures = new PathFigureCollection();
            foreach (var component in holes.Union(new IEnumerable<Point>[] { shell }))
            {
                var figure = new PathFigure(component.First(), component.Skip(1).Select(p => new LineSegment(p, true)), true);
                figures.Add(figure);
            }
            pathGeom.Figures = figures;
            Geometry = pathGeom;
        }

        public void Update(IEnumerable<Point> shell)
        {
            if (shell == null || shell.Count() <= 2)
                return;

            Shell = shell.ToList();
            if (Shell.First() != Shell.Last())
                Shell.Add(Shell.First());

            var pathGeom = new PathGeometry();
            var figure = new PathFigure(shell.First(), shell.Skip(1).Select(p => new LineSegment(p, true)), true);
            pathGeom.Figures = new PathFigureCollection { figure };
            Geometry = pathGeom;
        }

        protected override void TransformGeometry(Transform transform)
        {
            if (HasHoles)
                Update(Shell.Select(p => transform.Transform(p)), Holes.Select(h => h.Select(p => transform.Transform(p))));
            else
                Update(Shell.Select(p => transform.Transform(p)));
        }
    }

    public class DrawingLineString : DrawingComponent
    {
        public List<Point> Points { get; private set; }

        public bool Closed { get; private set; }

        public DrawingLineString(NetTopologySuite.Geometries.LineString lineString)
        {
            Update(lineString.Coordinates.Select(c => c.ToPoint()), lineString.IsClosed);
        }

        public DrawingLineString(IEnumerable<Point> points, bool closed)
        {
            Update(points, closed);
        }

        public void Update(IEnumerable<Point> points, bool closed)
        {
            if (points == null || points.Count() <= 1 || (closed && points.Count() <= 2))
                return;

            Points = points.ToList();
            Closed = closed;

            if (closed && Points.First() != Points.Last())
                Points.Add(Points.First());

            var pathGeom = new PathGeometry();
            var figure = new PathFigure(points.First(), points.Skip(1).Select(p => new LineSegment(p, true)), closed);
            pathGeom.Figures = new PathFigureCollection { figure };
            figure.IsFilled = false;

            Geometry = pathGeom;
        }

        protected override void TransformGeometry(Transform transform)
        {
            Update(Points.Select(p => transform.Transform(p)), Closed);
        }
    }

    public class DrawingPoint : DrawingComponent
    {
        public Point Point { get; set; }

        public DrawingPoint(Point point)
        {
            Update(point);
        }

        public void Update(Point point)
        {
            Point = point;
            Geometry = new EllipseGeometry(point, 3, 3);
        }

        protected override void TransformGeometry(Transform transform)
        {
            Update(transform.Transform(Point));
        }
    }

    public class DrawingVisualEdges : DrawingComponent
    {
        private IEnumerable<LineGeometry> lines;

        public DrawingVisualEdges(ShortestAvoidancePathFinder finder, IEnumerable<Point> points)
        {
            IsReadOnly = true;
            Update(finder, points);
        }

        public void Render(Pen pen)
        {
            using (var context = RenderOpen())
            {
                foreach (var line in lines)
                {
                    context.DrawGeometry(DrawingCanvas.DefaultBrush, pen, line);
                }
            }
        }

        public void  Update(ShortestAvoidancePathFinder finder, IEnumerable<Point> points)
        {
            var targetPoints = points.Take(2).ToList();
            var graph = targetPoints.Count == 2 ?
                finder.GetVisualGraph(targetPoints[0].ToCoordinate(), targetPoints[1].ToCoordinate())
                : finder.GetVisualGraph();
            lines = graph.Select(l => new LineGeometry(l.P0.ToPoint(), l.P1.ToPoint()));
        }

        protected override void TransformGeometry(Transform transform)
        {
            
        }
    }
}
