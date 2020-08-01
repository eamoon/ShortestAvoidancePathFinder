using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;

namespace Test
{
    public class CanvasStateBarInfo : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private Point point;
        public Point Point
        {
            get => point;
            set
            {
                point = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Point)));
            }
        }

        private string tip = "Ready";
        public string Tip
        {
            get => tip;
            set
            {
                tip = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Tip)));
            }
        }
    }

    public class CanvasStateMachine : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private DrawingCanvas canvas;

        public CanvasState CurrentState { get; private set; }

        public List<CanvasState> AllStates { get; private set; } = new List<CanvasState>();

        public CanvasStateMachine(DrawingCanvas drawingCanvas)
        {
            canvas = drawingCanvas;
        }

        public void Register(params Type[] stateTypes)
        {
            foreach (var type in stateTypes.Where(t => t.IsSubclassOf(typeof(CanvasState))))
            {
                AllStates.Add(Activator.CreateInstance(type, canvas) as CanvasState);
            }

            //IsSelect = true;
        }

        private bool ChangeState<T>() where T : CanvasState
        {
            var newState = AllStates.FirstOrDefault(s => s is T);
            if(newState != null)
            {
                CurrentState = newState;
                if (canvas.StateInfo != null)
                    canvas.StateInfo.Tip = newState.StateTip;
                return true;
            }

            return false;
        }

        private bool isSelect;
        public bool IsSelect
        {
            get => isSelect;
            set
            {
                isSelect = value;
                if (value)
                    ChangeState<SelectCanvasState>();
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsSelect)));
            }
        }

        private bool isDrawPolygon;
        public bool IsDrawPolygon
        {
            get => isDrawPolygon;
            set
            {
                isDrawPolygon = value;
                if (value)
                    ChangeState<AddPolygonCanvasState>();
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsDrawPolygon)));
            }
        }

        private bool isDrawPoint;
        public bool IsDrawPoint
        {
            get => isDrawPoint;
            set
            {
                isDrawPoint = value;
                if (value)
                    ChangeState<AddPontCanvasState>();
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsDrawPoint)));
            }
        }

        private bool isDrawLineString;
        public bool IsDrawLineString
        {
            get => isDrawLineString;
            set
            {
                isDrawLineString = value;
                if (value)
                    ChangeState<AddLineStringCanvasState>();
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsDrawLineString)));
            }
        }
    }
}
