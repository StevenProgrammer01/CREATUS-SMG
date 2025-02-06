using LiveCharts;
using LiveCharts.Wpf;
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;
using System.Threading.Tasks;
using System.Windows;
using System.ComponentModel;
using System.Windows.Media;

namespace ServerCreatus
{
    public class MainViewModel : INotifyPropertyChanged
    {
        private ChartValues<double> _co2;
        private ChartValues<double> _o2;

        public ChartValues<double> co2
        {
            get => _co2;
            set { _co2 = value; OnPropertyChanged(nameof(co2)); }
        }

        public ChartValues<double> o2
        {
            get => _o2;
            set { _o2 = value; OnPropertyChanged(nameof(o2)); }
        }

        public MainViewModel()
        {
            co2 = new ChartValues<double> { 400, 450, 480, 500 };
            o2 = new ChartValues<double> { 21, 20.8, 20.5, 20.2 };
        }

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName) =>
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }

    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName) =>
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        public ChartValues<double> co2Values { get; set; }
        public ChartValues<double> o2Values { get; set; }

        private UdpClient _udpServer;
        private const int PORT = 8000;

        public MainWindow()
        {
            

            InitializeComponent();
            DataContext = new MainViewModel();

            co2Values = new ChartValues<double>();
            o2Values = new ChartValues<double>();

            CO2Chart.Series = new SeriesCollection
            {
                new LineSeries { Title = "CO2 ppm", Values = co2Values,Stroke = new SolidColorBrush(Color.FromRgb(255, 0, 0)), // Rojo puro
        Fill = Brushes.Transparent, }

            };

            O2Chart.Series = new SeriesCollection
            {
                new LineSeries { Title = "O2 0-25%", Values = o2Values }
            };

            Task.Run(ReceiveData);
        }

        private async Task ReceiveData()
        {
            _udpServer = new UdpClient(PORT);
            while (true)
            {
                var results = await _udpServer.ReceiveAsync();
                string data = Encoding.UTF8.GetString(results.Buffer);

                try
                {
                    var sensorData = JsonConvert.DeserializeObject<SensorData>(data);
                    Console.WriteLine("Datos recibidos: " + JsonConvert.SerializeObject(sensorData));
                    Console.WriteLine(sensorData.co2);
                    Console.WriteLine(sensorData.o2);
                    Dispatcher.Invoke(() =>
                    {
                        co2Values.Add(sensorData.co2);
                        o2Values.Add(sensorData.o2);

                        // Notificar a la UI que los datos han cambiado
                        OnPropertyChanged(nameof(co2Values));
                        OnPropertyChanged(nameof(o2Values));
                    });
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error al procesar datos: " + ex.Message);
                }
            }
        }

        public class SensorData
        {
            public double co2 { get; set; }
            public double o2 { get; set; }
            public double temperature { get; set; }
            public double humidity { get; set; }
        }
    }
}
