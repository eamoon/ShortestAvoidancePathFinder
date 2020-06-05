using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NetTopologySuite.Geometries;
using System.Windows;

namespace Test
{
    public static class WPFToNTSExtensions
    {
        public static Geometry ToNTSGeometry(this DrawingComponent component)
        {
            if (component is DrawingPolygon drawingPolygon)
                return drawingPolygon.ToNTSGeometry();

            if (component is DrawingLineString drawingLineString)
                return drawingLineString.ToNTSGeometry();

            if (component is DrawingPoint drawingPoint)
                return drawingPoint.ToNTSGeometry();

            return null;
        }

        public static Geometry ToNTSGeometry(this DrawingPolygon drawingPolygon)
        {
            var shell = drawingPolygon.Shell.Select(p => p.ToCoordinate()).ToArray();
            if (drawingPolygon.HasHoles)
            {
                var holes = drawingPolygon.Holes.Select(h => new LinearRing(h.Select(p => p.ToCoordinate()).ToArray())).ToArray();
                return new Polygon(new LinearRing(shell), holes);
            }

            return new Polygon(new LinearRing(shell));
        }

        public static Geometry ToNTSGeometry(this DrawingLineString drawingLineString)
        {
            return new LineString(drawingLineString.Points.Select(p => p.ToCoordinate()).ToArray());
        }

        public static Geometry ToNTSGeometry(this DrawingPoint drawingPoint)
        {
            return new NetTopologySuite.Geometries.Point(drawingPoint.Point.ToCoordinate());
        }

        public static Coordinate ToCoordinate(this System.Windows.Point point)
        {
            return new Coordinate(point.X, point.Y);
        }

        public static System.Windows.Point ToPoint(this Coordinate coordinate)
        {
            return new System.Windows.Point(coordinate.X, coordinate.Y);
        }
    }
}
