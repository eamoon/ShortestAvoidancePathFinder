using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Test
{
    public class FinderSettings : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private DrawingCanvas drawingCanvas;
        public FinderSettings(DrawingCanvas canvas)
        {
            drawingCanvas = canvas;
        }

        private double bufferDistance;
        public double BufferDistance
        {
            get => bufferDistance;
            set
            {
                drawingCanvas._version++;
                bufferDistance = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(BufferDistance)));
            }
        }

        private int obstacleBehavior = 1;
        public int ObstacleBehavior
        {
            get => obstacleBehavior;
            set
            {
                obstacleBehavior = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ObstacleBehavior)));
            }
        }

        private int pathCategory = 0;
        public int PathCategory
        {
            get => pathCategory;
            set
            {
                pathCategory = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(PathCategory)));
            }
        }
    }

    public enum PathCategory
    {
        ShorestPath,
        AlongShorestPath
    }
}
