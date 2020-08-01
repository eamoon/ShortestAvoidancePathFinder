using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Threading;
using Microsoft.Win32;
using NetTopologySuite.Geometries;
using NetTopologySuite.Geometries.Utilities;
using NetTopologySuite.IO;
using PathFinder;

namespace Test
{
    /// <summary>
    /// MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private FinderSettings finderSettings;
        public MainWindow()
        {
            InitializeComponent();
            var stateInfo = new CanvasStateBarInfo();
            stateBar.DataContext = stateInfo;
            canvas.StateInfo = stateInfo;
            toolBar.DataContext = canvas.StateMachine;
            canvas.StateMachine.IsSelect = true;

            finderSettings = new FinderSettings(canvas);
            finderSettingPanel.DataContext = finderSettings;
        }

        ShortestAvoidancePathFinder finder;
        private int preVersion = -1;
        private List<DrawingLineString> paths = new List<DrawingLineString>();
        private void ComputeShorestPath()
        {
            var components = canvas.GetDrawingComponents();
            var points = components.OfType<DrawingPoint>().ToList();
            if (points.Count < 2)
            {
                if(paths.Count > 0)
                {
                    paths.ForEach(p => canvas.RemoveVisual(p));
                    paths.Clear();
                }
                return;
            }

            if (points.Count % 2 != 0)
                points.RemoveAt(points.Count - 1);

            var pointsCount = points.Count;

            if (preVersion == -1 || (preVersion != canvas._version))
            {
                var geom = GeometryCombiner.Combine(components.Where(c => !c.IsReadOnly && !(c is DrawingPoint)).Select(c => c.ToNTSGeometry()));
                finder = new ShortestAvoidancePathFinder(geom);
                finder.BufferDistance = finderSettings.BufferDistance;
                finder.InternalPointBehavior = (BreakThroughObstaclesBahavior)finderSettings.ObstacleBehavior;
                finder.BuildVisualGraph();
                preVersion = canvas._version;
            }

            finder.InternalPointBehavior = (BreakThroughObstaclesBahavior)finderSettings.ObstacleBehavior;

            int pathIndex = 0;
            for (int i = 0; i < pointsCount; i += 2, pathIndex++)
            {
                var path = finderSettings.PathCategory == 0 ?
                    finder.FindShortestPath(points[i].Point.ToCoordinate(), points[i + 1].Point.ToCoordinate())
                    : finder.FindAlongShortestPath(points[i].Point.ToCoordinate(), points[i + 1].Point.ToCoordinate());

                if (path.Count == 0)
                {
                    pathIndex--;
                    continue;
                }

                var coos = path.Select(l => l.P0).ToList();
                coos.Add(path.Last().P1);
                var targetPoints = coos.Select(c => c.ToPoint());

                if(pathIndex < paths.Count)
                {
                    paths[pathIndex].Update(targetPoints, false);
                    paths[pathIndex].Render(DrawingCanvas.HighLightPen);
                }
                else
                {
                    var drawingLineString = new DrawingLineString(targetPoints, false);
                    drawingLineString.IsReadOnly = true;
                    drawingLineString.Render(DrawingCanvas.HighLightPen);
                    canvas.AddVisual(drawingLineString);
                    paths.Add(drawingLineString);
                }
            }

            if(pathIndex < paths.Count)
            {
                for (int i = pathIndex; i < paths.Count; i++)
                {
                    canvas.RemoveVisual(paths[i]);
                }
                paths.RemoveRange(pathIndex, paths.Count - pathIndex);
            }
        }

        private void ComputeShorestPathClick(object sender, RoutedEventArgs e)
        {
            ComputeShorestPath();
        }

        DispatcherTimer timer;
        private void BeginComputeShorestPathClick(object sender, RoutedEventArgs e)
        {
            if (timer != null && timer.IsEnabled)
                return;

            timer = new DispatcherTimer();
            timer.Interval = TimeSpan.FromMilliseconds(30);
            timer.Tick += (s, arg) => ComputeShorestPath();
            timer.Start();
            beginBtn.IsEnabled = false;
            canvas.StateMachine.IsSelect = true;
        }
        
        private void StopComputeShorestPathClick(object sender, RoutedEventArgs e)
        {
            if (timer != null)
                timer.Stop();
            beginBtn.IsEnabled = true;
        }

        DrawingVisualEdges visualGraph;
        private void DrawVisualGraphClick(object sender, RoutedEventArgs e)
        {
            if (finder == null)
                return;

            var components = canvas.GetDrawingComponents();
            var points = components.OfType<DrawingPoint>().Select(p => p.Point);

            if (visualGraph == null)
            {
                visualGraph = new DrawingVisualEdges(finder, points);
                canvas.AddVisual(visualGraph);
            }
            else
            {
                visualGraph.Update(finder, points);
            }

            visualGraph.Render(DrawingCanvas.DarkPen);
        }

        private void SaveClick(object sender, RoutedEventArgs e)
        {
            if (canvas.GetVisuals().Count == 0)
                return;

            var dialog = new SaveFileDialog();
            dialog.DefaultExt = "nts";
            dialog.Filter = "NTS Geometry File|*.nts";
            if (dialog.ShowDialog() == true)
            {
                using (var stream = dialog.OpenFile())
                {
                    GeometryCombiner.SkipEmpty = true;
                    var visuals = canvas.GetVisuals();
                    paths.ForEach(p => visuals.Remove(p));

                    var geom = GeometryCombiner.Combine(visuals.Select(d => d.ToNTSGeometry()));
                    var writer = new StreamWriter(stream);
                    writer.Write(geom.ToString());
                    writer.Flush();
                }
            }
        }

        private void LoadClick(object sender, RoutedEventArgs e)
        {
            var dialog = new OpenFileDialog();
            dialog.Filter = "NTS Geometry File|*.nts";
            if (dialog.ShowDialog() == true)
            {
                using (var stream = dialog.OpenFile())
                {
                    var reader = new WKTReader();
                    try
                    {
                        var geom = reader.Read(stream);
                        if (geom != null && !geom.IsEmpty)
                        {
                            var polygons = PolygonExtracter.GetPolygons(geom).OfType<Polygon>().Select(p => new DrawingPolygon(p));
                            foreach (var polygon in polygons)
                            {
                                polygon.Render(DrawingCanvas.DefaultPen);
                                canvas.AddVisual(polygon);
                            }

                            var lineStrings = LineStringExtracter.GetLines(geom).OfType<LineString>().Select(p => new DrawingLineString(p));
                            foreach (var lineString in lineStrings)
                            {
                                lineString.Render(DrawingCanvas.DefaultPen);
                                canvas.AddVisual(lineString);
                            }

                            var points = PointExtracter.GetPoints(geom).OfType<NetTopologySuite.Geometries.Point>()
                                .Select(p => new DrawingPoint(new System.Windows.Point(p.X, p.Y)));
                            foreach (var point in points)
                            {
                                point.Render(DrawingCanvas.DefaultPen);
                                canvas.AddVisual(point);
                            }
                        }
                    }
                    catch
                    {
                        MessageBox.Show("Read failed! The file may have been corrupted!");
                    }
                }
            }
        }

        private void ClearClick(object sender, RoutedEventArgs e)
        {
            canvas.ClearVisuals();
        }
    }
}
