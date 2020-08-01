using NetTopologySuite.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using PathFinder;

namespace Test
{
    public abstract class CanvasState
    {
        public DrawingCanvas Canvas { get; set; }

        public virtual string StateTip { get; }

        public CanvasState(DrawingCanvas canvas)
        {
            Canvas = canvas;
        }

        public virtual void HandleMouseDown(MouseButtonEventArgs e)
        {
            Canvas.Focus();
        }

        public virtual void HandleMouseMove(MouseEventArgs e)
        {
            Canvas.StateInfo.Point = e.GetPosition(Canvas);
        }

        public virtual void HandleMouseUp(MouseButtonEventArgs e)
        {

        }

        public virtual void HandleKeyDown(KeyEventArgs e)
        {

        }
    }

    public class SelectCanvasState : CanvasState
    {
        private DrawingComponent selectedVisual;
        private bool isDragging = false;
        private Point prePoint;
        private Vector moveVector;

        public override string StateTip => "Select";

        public SelectCanvasState(DrawingCanvas canvas) : base(canvas)
        {
        }

        public override void HandleMouseDown(MouseButtonEventArgs e)
        {
            base.HandleMouseDown(e);
            if (e.ChangedButton != MouseButton.Left)
                return;

            isDragging = true;
            var position = e.GetPosition(Canvas);
            var visual = Canvas.GetVisual(position);

            if (visual != null && visual.IsReadOnly)
                return;

            selectedVisual = visual;
            if (selectedVisual != null)
            {
                selectedVisual.Render(DrawingCanvas.HighLightPen);
                Canvas.CaptureMouse();
            }
        }

        public override void HandleMouseMove(MouseEventArgs e)
        {
            base.HandleMouseMove(e);

            var currentPoint = e.GetPosition(Canvas);
            moveVector = currentPoint - prePoint;
            prePoint = currentPoint;

            if (isDragging && selectedVisual != null)
            {
                if (!(selectedVisual is DrawingPoint))
                    Canvas._version++;

                var transform = new TranslateTransform(moveVector.X, moveVector.Y);
                selectedVisual.Render(DrawingCanvas.HighLightPen, transform);
            }
        }

        public override void HandleMouseUp(MouseButtonEventArgs e)
        {
            base.HandleMouseUp(e);
            if (e.ChangedButton != MouseButton.Left)
                return;

            isDragging = false;
            if (selectedVisual != null)
            {
                selectedVisual.Render(DrawingCanvas.DefaultPen);
                Canvas.ReleaseMouseCapture();
            }
        }

        public override void HandleKeyDown(KeyEventArgs e)
        {
            base.HandleKeyDown(e);
            if(e.Key == Key.Delete && selectedVisual != null)
            {
                if(!(selectedVisual is DrawingPoint))
                    Canvas._version++;

                Canvas.RemoveVisual(selectedVisual);
                selectedVisual = null;
            }
        }
    }

    public abstract class AddCanvasState : CanvasState
    {
        protected const double alignEps = 0.05;

        protected Point prePoint;

        protected List<Point> points = new List<Point>();

        protected DrawingComponent element;

        public AddCanvasState(DrawingCanvas canvas) : base(canvas)
        {
        }

        public override void HandleMouseDown(MouseButtonEventArgs e)
        {
            base.HandleMouseDown(e);
            if (e.ChangedButton != MouseButton.Left)
                return;

            Canvas.StateInfo.Tip = "Drawing... double click to end";

            var point = e.GetPosition(Canvas);
            point = AlignPointToXY(point);
            if (prePoint == default || (prePoint != default && point != prePoint))
                points.Add(point);
            prePoint = point;

            if (e.ClickCount == 2)
            {
                prePoint = default;
                points.Clear();

                // The user might double click in the same place.
                if (element != null)
                {
                    CreationComplete(element);
                    element = null;
                }

                Canvas.StateInfo.Tip = StateTip;
            }
        }

        protected virtual void CreationComplete(DrawingComponent com)
        {

        }

        protected virtual Point AlignPointToXY(Point point)
        {
            return AlignPointToXY(point, points.LastOrDefault());
        }

        protected virtual Point AlignPointToXY(Point point, Point startPoint)
        {
            if (startPoint == null)
                return point;

            var vec = new Vector2D(point.X - startPoint.X, point.Y - startPoint.Y);
            var angle = Math.Abs(vec.Angle());
            if (angle.EqualsWithEps(0, alignEps) || angle.EqualsWithEps(Math.PI, alignEps))
            {
                return new Point(startPoint.X + vec.X, startPoint.Y);
            }
            if (angle.EqualsWithEps(Math.PI / 2, alignEps))
            {
                return new Point(startPoint.X, startPoint.Y + vec.Y);
            }

            return point;
        }
    }

    public class AddPolygonCanvasState : AddCanvasState
    {
        public override string StateTip => "Drawing Polygon";

        public AddPolygonCanvasState(DrawingCanvas canvas) : base(canvas)
        {
        }

        public override void HandleMouseMove(MouseEventArgs e)
        {
            base.HandleMouseMove(e);
            if (prePoint == default)
                return;

            var targetPoints = new List<Point>(points);

            var currentPoint = e.GetPosition(Canvas);
            currentPoint = AlignPointToXY(currentPoint);

            targetPoints.Add(currentPoint);

            if (element == null)
            {
                element = new DrawingLineString(targetPoints, false);
                Canvas.AddVisual(element);
            }
            else
            {
                if(targetPoints.Count <= 2)
                {
                    (element as DrawingLineString).Update(targetPoints, false);
                }
                else
                {
                    if(element is DrawingLineString)
                    {
                        Canvas.RemoveVisual(element);
                        element = new DrawingPolygon(targetPoints);
                        Canvas.AddVisual(element);
                    }
                    else
                    {
                        (element as DrawingPolygon).Update(targetPoints);
                    }
                }
            }

            element.Render(DrawingCanvas.DefaultPen);
        }

        protected override void CreationComplete(DrawingComponent ele)
        {
            if (ele is DrawingLineString)
                Canvas.RemoveVisual(ele);
            else
            {
                var container = Canvas.GetVisuals().OfType<DrawingPolygon>()
                    .FirstOrDefault(v => v.Geometry.FillContains(ele.Geometry));
                if(container != null)
                {
                    var holes = new List<IEnumerable<Point>> { (ele as DrawingPolygon).Shell };
                    if (container.HasHoles)
                        holes.AddRange(container.Holes);
                    container.Update(container.Shell, holes);
                    container.Render(DrawingCanvas.DefaultPen);
                    Canvas.RemoveVisual(ele);
                }

                Canvas._version++;
            }
        }

        protected override Point AlignPointToXY(Point point)
        {
            point = base.AlignPointToXY(point);
            // the current point should align with first point on polygon too.
            if(points.Count >= 2)
            {
                point = AlignPointToXY(point, points[0]);
            }

            return point;
        }
    }

    public class AddLineStringCanvasState : AddCanvasState
    {
        public override string StateTip => "Drawing PolyLine";

        public AddLineStringCanvasState(DrawingCanvas canvas) : base(canvas)
        {
        }

        public override void HandleMouseMove(MouseEventArgs e)
        {
            base.HandleMouseMove(e);
            if (prePoint == default)
                return;

            var targetPoints = new List<Point>(points);

            var currentPoint = e.GetPosition(Canvas);
            currentPoint = AlignPointToXY(currentPoint);
            targetPoints.Add(currentPoint);

            if (element == null)
            {
                element = new DrawingLineString(targetPoints, targetPoints.First() == targetPoints.Last());
                Canvas.AddVisual(element);
            }
            else
            {
                (element as DrawingLineString).Update(targetPoints, targetPoints.First() == targetPoints.Last());
            }

            element.Render(DrawingCanvas.DefaultPen);
        }

        protected override void CreationComplete(DrawingComponent com)
        {
            Canvas._version++;
        }
    }

    public class AddPontCanvasState : CanvasState
    {
        private Point prePoint;

        public override string StateTip => "Drawing Point";

        public AddPontCanvasState(DrawingCanvas canvas) : base(canvas)
        {
        }

        public override void HandleMouseDown(MouseButtonEventArgs e)
        {
            base.HandleMouseDown(e);
            if (e.ChangedButton != MouseButton.Left)
                return;

            var point = e.GetPosition(Canvas);
            if(prePoint == default || (prePoint != default && point != prePoint))
            {
                var drawingPoint = new DrawingPoint(point);
                drawingPoint.Render(DrawingCanvas.DefaultPen);
                Canvas.AddVisual(drawingPoint);
            }

            prePoint = point;
        }
    }
}
