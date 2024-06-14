using devDept.Eyeshot;
using devDept.Eyeshot.Entities;
using devDept.Geometry;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Eyeshot.FunctionPlot
{
    public partial class Form1 : Form
    {
        int _rows = 100;
        int _columns = 100;

        float _time = 0;
        float _amplitude = 2;
        float _waveNumber = 1;

        Point3D _sourcePosition = new Point3D(25, 25);
        Point3D _fastMeshSourcePosition;
        Point3D _fastPointCloudSourcePosition;

        FastMesh _fastMesh;
        FastPointCloud _fastPointCloud;

        System.Threading.Timer _timer;

        public Form1()
        {
            InitializeComponent();
        }

        protected override void OnLoad(EventArgs e)
        {
            design1.ActiveViewport.Grid.Visible = false;
            design1.Rendered.ShowEdges = false;

            LoadEntities();

            design1.SetView(devDept.Eyeshot.viewType.Trimetric);
            design1.ZoomFit();

            _timer = new System.Threading.Timer(Timer_tick1, null, 0, 100);

            base.OnLoad(e);
        }

        private void Timer_tick1(object state)
        {
            _time++;

            UpdateMesh();

            UpdateCloud();

            BeginInvoke(new Action(CompileFastEntities), null);
        }

        private void CompileFastEntities()
        {
            // 아래 처리를 해야 모델이 움직이는 효과를 낼 수 있다.
            CompileParams compileParams = new CompileParams(design1);
            _fastMesh.Compile(compileParams);
            _fastPointCloud.Compile(compileParams);
            design1.Invalidate();
        }

        private void UpdateCloud()
        {
            for (int i = 0; i < _fastPointCloud.PointArray.Length - 2; i = i + 3)
            {
                double x = _fastPointCloud.PointArray[i] - _fastPointCloudSourcePosition.X;
                double y = _fastPointCloud.PointArray[i + 1] - _fastPointCloudSourcePosition.Y;

                double dist = Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2));
                _fastPointCloud.PointArray[i + 2] = WaveEquation(dist);
            }
        }

        private void UpdateMesh()
        {
            for (int i = 0; i < _fastMesh.PointArray.Length - 2; i = i + 3)
            {
                double x = _fastMesh.PointArray[i] - _fastMeshSourcePosition.X;
                double y = _fastMesh.PointArray[i + 1] - _fastMeshSourcePosition.Y;

                double dist = Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2));
                _fastMesh.PointArray[i + 2] = WaveEquation(dist);

                // 길이 값이 음수일 경우
                if (dist < Utility.ZERO_TOLERANCE)
                {
                    _fastMesh.NormalArray[i] = 0;
                    _fastMesh.NormalArray[i + 1] = 0;
                    _fastMesh.NormalArray[i + 2] = 1;
                }
                else
                {
                    float fx = (float)(_amplitude * _waveNumber / 2 * Math.Cos(_waveNumber / 2 * dist - _time) * x / (2 * dist));
                    float fy = (float)(_amplitude * _waveNumber / 2 * Math.Cos(_waveNumber / 2 * dist - _time) * y / (2 * dist));
                    float norm = (float)(Math.Sqrt(Math.Pow(fx, 2) + Math.Pow(fy, 2)));
                    _fastMesh.NormalArray[i] = -fx / norm;
                    _fastMesh.NormalArray[i + 1] = -fy / norm;
                    _fastMesh.NormalArray[i + 2] = 1 / norm;
                }
            }
        }

        private float WaveEquation(double dist)
        {
            return (float)(_amplitude * Math.Sin(_waveNumber * dist / 2 - _time));
        }

        private void LoadEntities()
        {
            Mesh surface = CreateMesh();

            surface.Translate(60, 0);
            design1.Entities.Add(surface);

            _fastMesh = surface.ConvertToFastMesh();
            _fastMesh.Translate(0, 60);
            design1.Entities.Add(_fastMesh);

            PointCloud cloud = CreatePointCloud();
            cloud.LineWeightMethod = colorMethodType.byEntity;
            cloud.LineWeight = 3;
            design1.Entities.Add(cloud);

            _fastPointCloud = cloud.ConvertToFastPointCloud();
            _fastPointCloud.Translate(0, 60);
            design1.Entities.Add(_fastPointCloud);

            _fastMeshSourcePosition = (Point3D)_sourcePosition.Clone();
            _fastMeshSourcePosition.TransformBy(new Translation(60, 60));

            _fastPointCloudSourcePosition = (Point3D)_sourcePosition.Clone();
            _fastPointCloudSourcePosition.TransformBy(new Translation(0, 60));
        }

        private PointCloud CreatePointCloud()
        {
            List<Point3D> vertices = new List<Point3D>(_rows * _columns);
            CreateVertices(_sourcePosition.X, _sourcePosition.Y, vertices);
            PointCloud surface = new PointCloud(vertices);
            return surface;
        }

        private Mesh CreateMesh()
        {
            Mesh surface = new Mesh();

            List<Point3D> vertices = new List<Point3D>(_rows * _columns);
            surface.NormalAveragingMode = Mesh.normalAveragingType.Averaged;

            CreateVertices(_sourcePosition.X, _sourcePosition.Y, vertices);

            List<SmoothTriangle> triangles = new List<SmoothTriangle>((_rows - 1) * (_columns - 1) * 2);

            for (int i = 0; i < _rows - 1; i++)
                for (int j = 0; j < _columns - 1; j++)
                {
                    triangles.Add(new SmoothTriangle(
                        j + i * _columns,
                        j + i * _columns + 1,
                        j + (i + 1) * _columns + 1
                        ));

                    triangles.Add(new SmoothTriangle(
                        j + i * _columns,
                        j + (i + 1) * _columns + 1,
                        j + (i + 1) * _columns));
                }

            surface.Vertices = vertices.ToArray();
            surface.Triangles = triangles.ToArray();

            return surface;
        }

        private void CreateVertices(double sourcePositionX, double sourcePositionY, List<Point3D> vertices)
        {
            for (int i = 0; i < _rows; i++)

                for (int j = 0; j < _columns; j++)
                {
                    double x = (j / 2.0);
                    double y = (i / 2.0);

                    double f = 0;

                    double den = Math.Sqrt(Math.Pow((x - sourcePositionX), 2) + Math.Pow((y - sourcePositionY), 2));

                    if (den != 0)
                        f = _amplitude * Math.Sin(_waveNumber * den / 2);

                    int red = (int)(200 - y * 2);
                    int green = (int)(200 - x * 2);
                    int blue = (int)(50);

                    // 0~255 값 안넘어가게 처리
                    Utility.LimitRange<int>(0, ref red, 255);
                    Utility.LimitRange<int>(0, ref green, 255);
                    Utility.LimitRange<int>(0, ref blue, 255);

                    vertices.Add(new PointRGB(x, y, f, (byte)red, (byte)green, (byte)blue));
                }
        }
    }
}
