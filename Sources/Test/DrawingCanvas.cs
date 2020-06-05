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
    public class DrawingCanvas : Canvas
    {
        //static DrawingCanvas()
        //{
        //    DefaultStyleKeyProperty.OverrideMetadata(typeof(DrawingCanvas), new FrameworkPropertyMetadata(typeof(DrawingCanvas)));
        //}

        private List<DrawingComponent> visuals = new List<DrawingComponent>();

        internal int _version = 0;

        public static Pen DefaultPen { get; } = new Pen(Brushes.Gray, 2) { LineJoin = PenLineJoin.Bevel };

        public static Brush DefaultBrush { get; } = new SolidColorBrush(Color.FromArgb(40, 0, 0, 180));

        public static Pen HighLightPen { get; } = new Pen(Brushes.Red, 2) { LineJoin = PenLineJoin.Bevel };

        public static Pen DarkPen { get; } = new Pen(Brushes.DarkGray, 1) { LineJoin = PenLineJoin.Bevel };

        public CanvasState CurrentState => StateMachine.CurrentState;

        public CanvasStateMachine StateMachine { get; private set; }

        public CanvasStateBarInfo StateInfo { get; set; }

        public DrawingCanvas()
        {
            StateMachine = new CanvasStateMachine(this);
            StateMachine.Register(
                typeof(SelectCanvasState),
                typeof(AddPolygonCanvasState),
                typeof(AddLineStringCanvasState),
                typeof(AddPontCanvasState)
                );
            Focusable = true;
        }

        public List<DrawingComponent> GetDrawingComponents() => visuals;

        protected override int VisualChildrenCount => visuals.Count;

        protected override Visual GetVisualChild(int index)
        {
            return visuals[index];
        }

        public void AddVisual(DrawingComponent visual)
        {
            visuals.Add(visual);
            AddVisualChild(visual);
            AddLogicalChild(visual);
        }

        public void RemoveVisual(DrawingComponent visual)
        {
            visuals.Remove(visual);
            RemoveVisualChild(visual);
            RemoveLogicalChild(visual);
        }

        public void ClearVisuals()
        {
            visuals.ForEach(v =>
            {
                RemoveVisualChild(v);
                RemoveLogicalChild(v);
            });
            visuals.Clear();
        }

        public DrawingComponent GetVisual(Point point)
        {
            var result = VisualTreeHelper.HitTest(this, point);
            return result.VisualHit as DrawingComponent;
        }

        public List<DrawingComponent> GetVisuals() => visuals;

        private List<Geometry> GetDrawingGeometrys(Drawing drawing)
        {
            var result = new List<Geometry>();
            if (drawing is DrawingGroup group)
            {
                foreach (var subDrawing in group.Children)
                {
                    result.AddRange(GetDrawingGeometrys(subDrawing));
                }
            }
            else if (drawing is GeometryDrawing geometryDrawing)
            {
                result.Add(geometryDrawing.Geometry);
            }

            return result;
        }

        protected override void OnMouseDown(MouseButtonEventArgs e)
        {
            base.OnMouseDown(e);
            CurrentState.HandleMouseDown(e);
        }

        protected override void OnMouseUp(MouseButtonEventArgs e)
        {
            base.OnMouseUp(e);
            CurrentState.HandleMouseUp(e);
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            CurrentState.HandleMouseMove(e);
        }

        protected override void OnKeyDown(KeyEventArgs e)
        {
            base.OnKeyDown(e);
            CurrentState.HandleKeyDown(e);
        }
    }
}
