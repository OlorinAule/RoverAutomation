using Iot.Device.Hcsr04;
using System.Device.Gpio;
using System.Device.I2c;
using System.Device.Pwm;
using System.Diagnostics;
using System.IO.Ports;
using System.Text;

namespace Sterowanie
{
    class AutonomiczneSterowanie
    {
        static void Main(string[] args)
        {
            Thread thread = new(DataCollection);
            thread.Start();
            Thread.Sleep(5000);

            // Wszystkie wymiary są podane w centymetrach
            int reactionBorder = 150;
            float keepRearDistance = 100;
            int frontCriticalDistance = 100;

            byte autoSpeed = 127;

            // zmienne programu
            bool isWheelsOnPos = false;
            byte targetWheelAngle = 90;
            byte speed = 0;
            bool isBack = false;
            int CurrentTarget = 0;
            bool isOn = true;
            int refreshRate = 200;
            int loopCounter = 0;
            bool dataReaded = true;
            int maxWheelTurn = 30;
            byte maxLeft = (byte)(90 - maxWheelTurn);
            byte maxRight = (byte)(90 + maxWheelTurn);
            bool isAutoOn = false;

            // Pin do komunikacji z systemem kamery
            GpioController gpio = new();
            gpio.OpenPin(26, PinMode.Output, PinValue.Low);

            // Lista punktów celów
            List<Point> pointList = new();

            // Serial do komunikacji z arduino
            using SerialPort sp = new SerialPort("/dev/ttyACM0");
            sp.Encoding = Encoding.UTF8;
            sp.DataBits = 8;
            sp.BaudRate = 115200;
            sp.StopBits = StopBits.One;
            sp.Parity = Parity.None;
            sp.Handshake = Handshake.None;
            sp.RtsEnable = false;
            sp.ReadTimeout = 1000;
            sp.WriteTimeout = 1000;

            // Odzczyt listy celów
            GPSPointRead();

            // Aktualny cel
            Point targetPos = new();

            NextTarget();

            // Otwarcie komunikcji z arduino
            sp.Open();
            sp.DtrEnable = false;
            Thread.Sleep(1000);
            sp.BaseStream.Flush();
            Thread.Sleep(1000);
            sp.DtrEnable = true;
            Thread.Sleep(1000);

            StreamWriter log = new StreamWriter("LastLog.txt");

            Console.CancelKeyPress += (a, b) =>
            {
                sp.Close();
                isOn = false;
                log.Close();
            };


            Stopwatch endWatch = new();
            Stopwatch timer = new();
            Stopwatch ComTimeout = new();
            Stopwatch sw = Stopwatch.StartNew();
            Stopwatch sw2 = Stopwatch.StartNew();
            timer.Start();
            ComTimeout.Start();

            DataWrite();

            while (isOn)
            {
                // Blokada prędkości odświerzania
                if (timer.ElapsedMilliseconds >= refreshRate)
                {
                    timer.Restart();

                    // Pobieranie informacji z arduino/łazika
                    DataRead();

                    // Pętla decyzyjna
                    if (isAutoOn)//auto pin on
                    {
                        Console.Clear();

                        Console.WriteLine("Pętla nr {0}.", loopCounter);
                        loopCounter++;

                        // Uruchomienie nagrywania
                        gpio.Write(26, PinValue.High);
                        Console.WriteLine("Nagrywanie włączone");

                        // Sprawdzanie czy jest wykryta pozycja przez gps
                        if (gPSData[0].latitude == 0 || gPSData[0].longitude == 0)
                        {
                            Console.WriteLine("Brak sygnału z satelit {0}", gPSData[0].sateliteNumber);
                        }

                        else if (CritCheck())
                        {

                        }

                        else if (PositionCalc())
                        {

                        }

                        else if (FrontCheck())
                        {

                        }

                        else if (SecondaryCheck())
                        {

                        }

                        else if (BackCheck())
                        {

                        }

                        else
                        {
                            NoMoveCheck();
                        }
                    }

                    else
                    {
                        // Zatrzymanie nagrywania
                        gpio.Write(26, PinValue.Low);

                        speed = 0;
                        Console.WriteLine("nie ma sygnału on");
                    }

                    //Wyłączenie systemu w przypadku braku komunikacji
                    if (ComTimeout.ElapsedMilliseconds > 1000)
                    {
                        Console.WriteLine("Tryb auto wyłączony");
                        isAutoOn = false;
                    }

                    //Wysłanie informacji do arduino/łazika
                    DataWrite();
                }

                if (sw2.ElapsedMilliseconds > 1000)
                {
                    log.WriteLine(String.Format("{0}\t{1}\t{2}\t{3}", sw.Elapsed, gPSData[0].latitude, gPSData[0].longitude, imuData[0].Angle));
                    sw2.Restart();
                }
            }

            Console.WriteLine("Zamknięcie pętli");
            thread.Abort();
            Console.WriteLine("Wyłączenie drugiego wątku");

            void DataRead()
            {
                // Sprawdzanie czy są dane do odczytania
                if (sp.BytesToRead >= 1)
                {
                    // Odczyt danych z arduino
                    byte[] dataIn = new byte[1];
                    sp.Read(dataIn, 0, dataIn.Length);
                    Console.WriteLine("Odczytane dane: " + String.Join(", ", dataIn));

                    // Dekodowanie danych
                    isAutoOn = true;
                    isWheelsOnPos = dataIn[0] == 1;
                    dataReaded = true;
                    ComTimeout.Restart();
                }
            }

            void DataWrite()
            {
                if (dataReaded)
                {
                    // Kodowanie informacji
                    byte BackMode = (byte)(isBack ? 1 : 0);
                    byte[] dataOut = new byte[3];
                    dataOut[0] = speed;
                    dataOut[1] = targetWheelAngle;
                    dataOut[2] = BackMode;

                    // Wysłnie informacji
                    sp.Write(dataOut, 0, dataOut.Length);
                    Console.WriteLine("Wysłane dane: " + String.Join(", ", dataOut));
                    dataReaded = false;
                }
            }

            bool CritCheck()
            {
                bool isSafe = SonicData[0] > frontCriticalDistance && SonicData[1] > frontCriticalDistance;
                Console.WriteLine("Pomiar z czujników przednich: L={0}, P={1}", SonicData[0], SonicData[1]);

                if (isSafe)
                {
                    return false;
                }
                else
                {
                    Console.WriteLine("Krytyczne zatrzymanie");
                    if (BackCheck())
                    {
                        return true;
                    }
                    else
                    {
                        NoMoveCheck();
                        return true;
                    }
                }
            }

            bool PositionCalc()
            {
                Console.WriteLine("Aktualny punkt: {0}, {1}", CurrentTarget, targetPos);
                Console.WriteLine("Aktualne położenie: {0}, {1}", gPSData[0].latitude, gPSData[0].longitude);

                double dLatitude = targetPos.latitude - gPSData[0].latitude;
                double dLongitude = targetPos.longitude - gPSData[0].longitude;

                Console.WriteLine("Rożnica położenia: {0}, {1}", dLatitude, dLongitude);

                double gpsAngle = (Math.Atan2(dLatitude, dLongitude) * 180 / Math.PI);

                Console.WriteLine("Kąt GPS: {0}", gpsAngle);
                Console.WriteLine("Kąt czujników: {0}", imuData[0].Angle);

                double dAngle = gpsAngle - imuData[0].Angle;

                Console.WriteLine("Różnica kąta: {0}", dAngle);

                double positionDeadZone = 0.001;
                if (Math.Abs(dLatitude) < positionDeadZone && Math.Abs(dLongitude) < positionDeadZone)
                {
                    // Koniec trasy/następny cel
                    NextTarget();
                    return true;
                }

                double AngleDeadZone = 20;
                if (Math.Abs(dAngle) > AngleDeadZone)
                {
                    // Sprawdzenie czy cel jest po prawej stronie
                    bool isRight = gpsAngle < imuData[0].Angle;

                    Console.WriteLine("Cel jest {0} stopni na {1}", Math.Abs(dAngle), isRight ? "prawo" : "lewo");

                    if (!TurnCheck(isRight))
                    {
                        isBack = false;
                        SetWheels((byte)(isRight ? maxRight : maxLeft));
                        return true;
                    }

                    return false;
                }

                return false;
            }

            bool TurnCheck(bool isRight)
            {
                int Direction = isRight ? -1 : 1;
                if (RotCheck(Direction)) return true;
                return false;
            }

            bool RotCheck(int direction)
            {
                Console.WriteLine("Sprawdzanie przeszkód " + (direction == 0 ? "z przodu" : (direction == 1 ? "po prawej stronie" : "po lewej stronie")));
                int obstacleMin = 3;
                int count = 0;
                for (int i = 0; i < 10; i++)
                {
                    double dist = rotSensorDatas[i + (direction * 10) + 10].Distance;
                    double angl = rotSensorDatas[i + (direction * 10) + 10].Angle;
                    if (dist < reactionBorder)
                    {
                        count++;
                    }
                    Console.WriteLine("Pomiar nr {0}, odległość: {1}, kąt: {2}", i, dist, angl);
                }

                Console.WriteLine("{0} czujników wykryło coś poniżej {1} cm.", count, reactionBorder);
                if (count >= obstacleMin)
                {
                    Console.WriteLine("Wykryto przeszkodę");
                    return true;
                }
                else
                {
                    Console.WriteLine("Brak przeszkód");
                    return false;
                }
            }

            bool FrontCheck()
            {
                Console.WriteLine("Sprawadzanie przodu pojazdu");
                bool obstacleDetected = SonicData[0] < reactionBorder || SonicData[1] < reactionBorder;

                if (isBack)
                {
                    Console.WriteLine("Pojazd w trybie wstecznym");
                    return false;
                }

                if (!obstacleDetected && !RotCheck(0))
                {
                    if (SonicData[2] < keepRearDistance)
                    {
                        Console.WriteLine("Lewy czujnik: {0}", SonicData[2]);
                        SetWheels((byte)(90 - (maxWheelTurn / 2)));
                    }
                    else if (SonicData[3] < keepRearDistance)
                    {
                        Console.WriteLine("Prawy czujnik: {0}", SonicData[3]);
                        SetWheels((byte)(90 + (maxWheelTurn / 2)));
                    }
                    else SetWheels(90);
                    return true;
                }
                return false;
            }

            bool SecondaryCheck()
            {
                Console.WriteLine("Sprawdzanie boków pojazdu");

                if (!RotCheck(-1))
                {
                    isBack = false;
                    SetWheels(maxLeft);
                    return true;
                }

                if (!RotCheck(1))
                {
                    isBack = false;
                    SetWheels(maxRight);
                    return true;
                }

                return false;
            }

            void SetWheels(byte WheelAngle)
            {
                if (targetWheelAngle == WheelAngle && isWheelsOnPos)
                {
                    speed = autoSpeed;
                    Console.WriteLine("Koła ustawione w pozycji {0} stopni. Jedź z prędkością {1}",
                        WheelAngle, speed);
                }
                else
                {
                    speed = 0;
                    targetWheelAngle = WheelAngle;
                    Console.WriteLine("Ustawianie kół na pozycję {0} stopni.", WheelAngle);
                }
                endWatch.Reset();
            }

            bool BackCheck()
            {
                Console.WriteLine("Sprawdzanie tyłu pojazdu");
                if (SonicData[4] > reactionBorder)
                {
                    isBack = true;
                    SetWheels(90);
                    return true;
                }

                return false;
            }

            void NoMoveCheck()
            {
                if (!endWatch.IsRunning) endWatch.Start();

                if (endWatch.ElapsedMilliseconds < 10000)
                {
                    Console.WriteLine("Brak możliwości ruchu. Ponowne sprawdzenie");

                }
                else
                {
                    // Błąd
                    Console.WriteLine("Błąd systemu: Nie ma możliwości bezpiecznego poruszenia się");
                    speed = 0;
                    isOn = false;
                }
            }

            void GPSPointRead()
            {
                try
                {
                    StreamReader reader = new StreamReader("GPSPoints.txt");

                    string data;

                    while (true)
                    {
                        data = reader.ReadLine();

                        if (data != null)
                        {
                            string[] splittedData = new string[2];

                            splittedData = data.Split("\t");

                            Point point = new Point();
                            point.latitude = Double.Parse(splittedData[0], System.Globalization.CultureInfo.InvariantCulture);
                            point.longitude = Double.Parse(splittedData[1], System.Globalization.CultureInfo.InvariantCulture);

                            pointList.Add(point);
                        }
                        else break;
                    }

                    reader.Close();
                }
                catch (Exception exception)
                {
                    Console.WriteLine("Wystąpił błąd podczas odczytu celów.");
                    Console.WriteLine(exception.Message);

                    return;
                }
            }

            void NextTarget()
            {
                Console.WriteLine("Sprawdzanie następnego punktu");
                if (pointList.Count > CurrentTarget)
                {
                    Console.WriteLine("Następny punkt istnieje i został przypisany");
                    targetPos = pointList[CurrentTarget];

                    CurrentTarget++;
                }
                else
                {
                    Console.WriteLine("Koniec trasy");
                    speed = 0;
                    isOn = false;
                }
            }
        }

        public static SynchronizedCollection<GPSData> gPSData = new();
        public static SynchronizedCollection<IMUData> imuData = new();
        public static SynchronizedCollection<double> SonicData = new();
        public static SynchronizedCollection<RotSensorData> rotSensorDatas = new();

        public class GPSData
        {
            public TimeOnly time { get; set; }
            public double latitude { get; set; }
            public char latitudeDirection { get; set; }
            public double longitude { get; set; }
            public char longitudeDirection { get; set; }
            public double altitude { get; set; }
            public double magneticVariation { get; set; }
            public char magneticVariationDirection { get; set; }
            public int sateliteNumber { get; set; }
        }
        public class IMUData
        {
            public double Ax { get; set; }
            public double Ay { get; set; }
            public double Az { get; set; }
            public double Gx { get; set; }
            public double Gy { get; set; }
            public double Gz { get; set; }
            public double Magx { get; set; }
            public double Magy { get; set; }
            public double Magz { get; set; }
            public double MagAngle { get; set; }
            public double GyroAngle { get; set; }
            public double Angle { get; set; }

        }
        public class RotSensorData
        {
            public int Angle { get; set; }
            public double Distance { get; set; }
        }

        class Point
        {
            public double latitude;
            public double longitude;
            public override string ToString()
            {
                return latitude + "\t" + longitude;
            }
        }

        public static void DataCollection()
        {
            // Inicjalizacja ogólna czujników ultradźwiękowych HCSR04
            GpioController controller = new GpioController();
            Hcsr04[] sensors = new Hcsr04[5];                               // Tu zmienić liczbę sensorów minus sensor serva
            Hcsr04 ServoSensor;
            SensorInit();                                                   // Przypisanie do konkretnych pinów rpi

            // Dane dla Serwa
            int range = 150;
            int dAngle = 5;
            int MesurementNumber = (range / dAngle) + 1;

            // Inicjalizcja Serwonapędu
            int pwmFreq = 50;
            double dutyCyclePerc = ((90 + 8) / 1800d) + 0.023;
            var Servo = PwmChannel.Create(0, 0, pwmFreq, dutyCyclePerc);    // Pin sygnałowy dla serwa - GPIO 12 Vcc - 5 V
            controller.OpenPin(16, PinMode.Output, PinValue.High);           // Pin dla przekaźnika serwa - GPIO 16

            // Inicjalizacja GPS
            using SerialPort sp = new SerialPort("/dev/ttyS0");             // Komunikacja serialowa UART. Piny: 14 - TXD, 15 - RXD, Vcc - 3,3 V
            GPS_Init();

            // Inicjalizacja Akcelerometru, źyroskopu i magnetometru
            I2cDevice MPU6050, GY271;                                       // Komunikacja przez I2C. Piny dla I2C: 2 - sda(zielony), 3 - scl(żółty) Vcc - 3,3 V (żołty)

            // Dane kalibracji Akcelerometru
            short[] Offsets = new short[6];
            byte Loops = 8;
            int loopCount = 0;

            // Pełna inicjalizacja wraz z wgraniem ustawień i kalibracją MPU6050
            GY271Init();
            MPU_Init();

            // Zmienne dla danych z pomiarów
            IMUData IMUData = new();
            GPSData gpsData = new();
            double[] sonicData = new double[sensors.Length];
            RotSensorData[] rotSensorData = new RotSensorData[MesurementNumber];

            // Przygotowanie zmiennych do transferu danych do głownego programu
            gPSData.Add(gpsData);
            imuData.Add(IMUData);
            for (int i = 0; i < sensors.Length; i++)
            {
                SonicData.Add(sonicData[i]);
            }
            for (int i = 0; i < MesurementNumber; i++)
            {
                rotSensorData[i] = new RotSensorData() { Angle = 0, Distance = 0 };
                rotSensorDatas.Add(rotSensorData[i]);
            }

            // Przgotowanie zadań do asynchronicznych pomiarów
            Task ServoTask = new Task(() => ServoRotAction());
            Task[] SensorTasks = new Task[sensors.Length];
            for (int i = 0; i < sensors.Length; i++)
            {
                SensorTasks[i] = new Task(() => SensorMesure(i));
            }

            // Kalibracja MPU6050
            Calibrate();

            // Zamknięcie seriala dla GPS oraz wylączenie Servo po zamknięciu aplikacji
            Console.CancelKeyPress += (a, b) =>
            {
                Servo.Stop();
                controller.Write(16, PinValue.High);
                sp.Close();
            };

            // Stopery dla pomiarów czasu do debugowania
            Stopwatch stopwatch = new();
            Stopwatch stopwatch2 = new();

            stopwatch.Start();
            stopwatch2.Start();

            // Uruchomienie pierwszych pomiarów HCSR04
            controller.Write(16, PinValue.Low);
            Servo.Start();
            ServoTask.Start();
            for (int i = 0; i < sensors.Length; i++)
            {
                SensorTasks[i].Start();
            }

            // Petla pobierania danych
            while (true)
            {
                // Jeżeli system jest wyłączony nie następuje pobieranie danych

                if (ServoTask.IsCompleted)      // Ponowne uruchomienie pomiaru jeżeli został już zakończony
                {
                    ServoTask = new Task(() => ServoRotAction());
                    ServoTask.Start();
                }
                for (int i = 0; i < sensors.Length; i++)
                {
                    if (SensorTasks[i].IsCompleted)
                    {
                        SensorTasks[i] = new Task(() => SensorMesure(i));
                        SensorTasks[i].Start();
                    }
                }

                // Pomiar dla MPU6050 i GY271
                UpdateData();

                // Pomiar GPS co sekundę
                if (stopwatch.ElapsedMilliseconds > 1000)
                {
                    ReadGPS();
                    stopwatch.Restart();
                }

                // Przesłanie danych by były dostępne dla głownego programu
                gPSData.Insert(0, gpsData);
                imuData.Insert(0, IMUData);
                imuData.RemoveAt(1);
                gPSData.RemoveAt(1);
            }

            void SensorInit()
            {

                // Inicjalizacja Przedniego lewego czujnika odległości
                int TrigPinFL = 17;
                int EchoPinFL = 27;
                sensors[0] = new Hcsr04(controller, TrigPinFL, EchoPinFL, true);

                // Inicjalizacja Przedniego prawego czujnika odległości
                int TrigPinFR = 24;
                int EchoPinFR = 25;
                sensors[1] = new Hcsr04(controller, TrigPinFR, EchoPinFR, true);

                // Inicjalizacja lewego czujnika odległości
                int TrigPinL = 5;
                int EchoPinL = 6;
                sensors[2] = new Hcsr04(controller, TrigPinL, EchoPinL, true);

                // Inicjalizacja prawego czujnika odległości
                int TrigPinR = 22;
                int EchoPinR = 10;
                sensors[3] = new Hcsr04(controller, TrigPinR, EchoPinR, true);

                // Inicjalizacja tylnego czujnika odległości
                int TrigPinB = 13;
                int EchoPinB = 19;
                sensors[4] = new Hcsr04(controller, TrigPinB, EchoPinB, true);

                // Inicjalizacja Obrotowego czujnika odległości
                int TrigPinRot = 23;
                int EchoPinRot = 18;
                ServoSensor = new Hcsr04(controller, TrigPinRot, EchoPinRot, true);
            }

            void MPU_Init()
            {
                // Adresy MPU6050
                byte Device_Address_MPU = 0x68;
                byte PWR_MGMT_1 = 0x6B;
                byte SMPLRT_DIV = 0x19;
                byte CONFIG = 0x1A;
                byte GYRO_CONFIG = 0x1B;
                byte INT_ENABLE = 0x38;

                MPU6050 = I2cDevice.Create(new I2cConnectionSettings(1, Device_Address_MPU));

                //write to sample rate register
                MPU6050.Write(new byte[] { SMPLRT_DIV, 7 });

                //Write to power management register
                MPU6050.Write(new byte[] { PWR_MGMT_1, 1 });

                //Write to Configuration register
                MPU6050.Write(new byte[] { CONFIG, 0 });

                //Write to Gyro configuration register
                MPU6050.Write(new byte[] { GYRO_CONFIG, 24 });

                //Write to interrupt enable register
                MPU6050.Write(new byte[] { INT_ENABLE, 1 });

                //write to sample rate register
                MPU6050.Write(new byte[] { SMPLRT_DIV, 7 });

                //Write to power management register
                MPU6050.Write(new byte[] { PWR_MGMT_1, 1 });

                //Write to Configuration register
                MPU6050.Write(new byte[] { CONFIG, 0 });

                //Write to Gyro configuration register
                MPU6050.Write(new byte[] { GYRO_CONFIG, 24 });

                //Write to interrupt enable register
                MPU6050.Write(new byte[] { INT_ENABLE, 0b_0100_0001 });
            }

            Int16 Read2Byte(byte reg, I2cDevice device)
            {
                byte[] addr = new byte[1];
                addr[0] = reg;
                byte[] data = new byte[2];
                try
                {
                    if (device == GY271)
                    {
                        device.WriteRead(addr, data);
                        return (((short)((data[1] << 8) | data[0])));
                    }
                    else
                    {
                        device.WriteRead(addr, data);
                        return (((short)((data[0] << 8) | data[1])));
                    }
                }
                catch
                {
                    Console.WriteLine("Błąd");
                    return 0;
                }
            }

            byte ReadByte(byte reg, I2cDevice device)
            {
                byte[] addr = new byte[1];
                addr[0] = reg;
                byte[] data = new byte[1];
                try
                {
                    device.WriteRead(addr, data);
                    return data[0];
                }
                catch
                {
                    Console.WriteLine("Błąd");
                    return 0;
                }
            }

            void Calibrate()
            {
                Console.WriteLine("Kalibracja...");
                CalibrateGyro(Loops);
                //CalibrateAccel(Loops);
                Console.WriteLine("Kalibracja zakończona");
                Console.WriteLine(String.Join("\t", Offsets));
            }

            void CalibrateGyro(byte loops)
            {
                float kP = 0.3f;
                float kI = 90;
                float x;
                x = (float)((100 - Map(loops, 1, 5, 20, 0)) * .01);
                kP *= x;
                kI *= x;

                PID(0x43, kP, kI, loops);
            }

            void CalibrateAccel(byte loops)
            {
                float kP = 0.3f;
                float kI = 20;
                float x;
                x = (float)((100 - Map(loops, 1, 5, 20, 0)) * .01);
                kP *= x;
                kI *= x;

                PID(0x3B, kP, kI, loops);
            }

            void PID(byte ReadAddress, float kP, float kI, byte loops)
            {
                byte SaveAddress = (ReadAddress == (byte)0x3B) ? (byte)0x06 : (byte)0x13;
                //Console.WriteLine(SaveAddress.ToString("X2"));

                short Data = 0;
                float Reading;
                ushort[] BitZero = new ushort[3];
                byte shift = (byte)((SaveAddress == 0x77) ? 3 : 2);
                float Error, PTerm;
                float[] ITerm = new float[3];
                short eSample;
                uint eSum;
                //Console.WriteLine('>');
                for (int i = 0; i < 3; i++)
                {
                    Data = Read2Byte((byte)(SaveAddress + (i * shift)), MPU6050);// reads 1 or more 16 bit integers (Word)
                    Reading = Data;
                    if (SaveAddress != 0x13)
                    {
                        BitZero[i] = (ushort)(Data & 1);                                       // Capture Bit Zero to properly handle Accelerometer calibration
                        ITerm[i] = ((float)Reading) * 8;
                    }
                    else
                    {
                        ITerm[i] = Reading * 4;
                    }
                    //Console.WriteLine("Bit0 " + Reading + "\t" + BitZero[i] + "\t" + ITerm[i]);
                }
                //Console.ReadLine();
                for (int L = 0; L < loops; L++)
                {
                    eSample = 0;
                    for (int c = 0; c < 100; c++)
                    {// 100 PI Calculations
                        eSum = 0;
                        for (int i = 0; i < 3; i++)
                        {
                            Data = Read2Byte((byte)(ReadAddress + (i * 2)), MPU6050);// reads 1 or more 16 bit integers (Word)
                            Reading = Data;
                            if ((ReadAddress == 0x3B) && (i == 2)) Reading = Reading - 16384;    //remove Gravity
                            Error = -Reading;
                            eSum += (uint)(Math.Abs(Reading));
                            PTerm = kP * Error;
                            ITerm[i] += (Error * 0.001f) * kI;               // Integral term 1000 Calculations a second = 0.001
                            if (SaveAddress != 0x13)
                            {
                                Data = (short)Math.Round((PTerm + ITerm[i]) / 8);       //Compute PID Output
                                Data = (short)(((Data) & 0xFFFE) | BitZero[i]);          // Insert Bit0 Saved at beginning
                            }
                            else Data = (short)Math.Round((PTerm + ITerm[i]) / 4);  //Compute PID Output
                            byte[] bytes = BitConverter.GetBytes(Data);
                            MPU6050.Write(new byte[] { (byte)(SaveAddress + (i * shift)), bytes[1] });
                            MPU6050.Write(new byte[] { (byte)(SaveAddress + (i * shift) + 1), bytes[0] });
                            //if (ReadAddress == 0x3B)
                            //{
                            //	short read = Read2Byte((byte)(SaveAddress + (i * shift)), MPU6050);
                            //	Console.WriteLine(Reading + "\t" + Data + "\t" + read);
                            //	Console.WriteLine(String.Join("\t", new object[] { Error, eSum, PTerm, ITerm[i] }));
                            //}
                        }
                        //if (ReadAddress == 0x3B) Console.ReadLine();
                        if ((c == 99) && eSum > 1000)
                        {                       // Error is still to great to continue 
                            c = 0;
                            //Console.Write('*');
                            //Console.Write(eSum);
                        }
                        if ((eSum * ((ReadAddress == 0x3B) ? .05 : 1)) < 5) eSample++;  // Successfully found offsets prepare to  advance
                        if ((eSum < 100) && (c > 10) && (eSample >= 10)) break;     // Advance to next Loop
                        Thread.Sleep(1);
                    }
                    //Console.WriteLine('.');
                    kP *= .75f;
                    kI *= .75f;
                    for (int i = 0; i < 3; i++)
                    {
                        if (SaveAddress != 0x13)
                        {
                            Offsets[i] = Data;
                            Data = (short)Math.Round((ITerm[i]) / 8);       //Compute PID Output
                            Data = (short)(((Data) & 0xFFFE) | BitZero[i]);  // Insert Bit0 Saved at beginning
                        }
                        else
                        {
                            Offsets[i + 3] = Data;
                            Data = (short)Math.Round((ITerm[i]) / 4);
                        }
                        //Console.WriteLine(Data);
                        byte[] bytes = BitConverter.GetBytes(Data);
                        MPU6050.Write(new byte[] { (byte)(SaveAddress + (i * shift)), bytes[1] });
                        MPU6050.Write(new byte[] { (byte)(SaveAddress + (i * shift) + 1), bytes[0] });
                    }
                }
                byte USR_CRL = ReadByte(0x6A, MPU6050);
                MPU6050.Write(new byte[] { 0x6A, (byte)(USR_CRL + 4) });
                MPU6050.Write(new byte[] { 0x6A, (byte)(USR_CRL + 8) });
            }

            float Map(float s, float a1, float a2, float b1, float b2)
            {
                float maped = b1 + (s - a1) * (b2 - b1) / (a2 - a1);
                return maped;
            }

            void UpdateData()
            {
                double MagX = IMUData.Magx;
                double MagY = IMUData.Magy;
                double MagZ = IMUData.Magz;
                double Ax = IMUData.Ax;
                double Ay = IMUData.Ay;
                double Az = IMUData.Az;
                double Gx = IMUData.Gx;
                double Gy = IMUData.Gy;
                double Gz = IMUData.Gz;
                double MagAngle = IMUData.MagAngle;
                double GyroAngle = IMUData.GyroAngle;
                double Angle = IMUData.Angle;

                // Adresy MPU6050
                byte ACCEL_XOUT_H = 0x3B;
                byte ACCEL_YOUT_H = 0x3D;
                byte ACCEL_ZOUT_H = 0x3F;
                byte GYRO_XOUT_H = 0x43;
                byte GYRO_YOUT_H = 0x45;
                byte GYRO_ZOUT_H = 0x47;

                // Martwa strefa MPU6050
                double AccDeadZone = 0.05;
                double GyroDeaZone = 0.50;

                // Adresy GY271
                byte RegXLo = 0x00;
                byte RegXHi = 0x01;
                byte RegYLo = 0x02;
                byte RegYHi = 0x03;
                byte RegZLo = 0x04;
                byte RegZHi = 0x05;

                // Macierz kalibracji magnetometru
                double[,] A = {
                    { 1.019457, -0.023199, -0.010013},
                    { -0.023199, 1.066198, -0.102175},
                    { -0.010013, -0.102175, 0.924774}
                };

                // Skumulowany bląd magnetometru
                double[] b = { 710.304860, -383.296437, -684.839558 };

                // Martwa strefa magnetometru
                double magDeadZone = 30;

                // Różnica pomiaru z rzeczywistą północą
                double declinationAngle = -1.73;

                // Sprawdzanie czy dane są dostępne dla GY271
                byte temData = ReadByte(0x06, GY271);
                bool isDataReadyMag = (temData & 5) > 0;
                if (isDataReadyMag)
                {
                    // odczyt surowych danych
                    var rawX = Read2Byte(RegXLo, GY271);
                    var rawY = Read2Byte(RegYLo, GY271);
                    var rawZ = Read2Byte(RegZLo, GY271);

                    //Kalibracja danych
                    double[] SoftCal = { rawX - b[0], rawY - b[1], rawZ - b[2] };

                    var CalX = SoftCal[0] * A[0, 0] + SoftCal[1] * A[0, 1] + SoftCal[2] * A[0, 2];
                    var CalY = SoftCal[0] * A[1, 0] + SoftCal[1] * A[1, 1] + SoftCal[2] * A[1, 2];
                    var CalZ = SoftCal[0] * A[2, 0] + SoftCal[1] * A[2, 1] + SoftCal[2] * A[2, 2];

                    // ustawianie martwej strefy magnetometru
                    var LastX = MagX == 0 ? 0 : MagX;
                    var LastY = MagY == 0 ? 0 : MagY;
                    var LastZ = MagZ == 0 ? 0 : MagZ;

                    MagX = Math.Abs(CalX - LastX) > magDeadZone ? CalX : LastX;
                    MagY = Math.Abs(CalY - LastY) > magDeadZone ? CalY : LastY;
                    MagZ = Math.Abs(CalZ - LastZ) > magDeadZone ? CalZ : LastZ;

                    // Wyliczanie kąta

                    var lastMag = MagAngle == 0 ? 180 : MagAngle;

                    var heading = Math.Atan2(MagY, MagX);
                    heading = heading + declinationAngle;
                    if (heading > 2 * Math.PI)
                        heading = heading - 2 * Math.PI;

                    if (heading < 0)
                        heading = heading + 2 * Math.PI;

                    MagAngle = (heading * 180 / Math.PI);
                    var tempAngle = MagAngle + (360 * loopCount);

                    // Zliczanie i dodawanie pełnych obrotów urządzenia
                    if (Math.Abs(lastMag - tempAngle) > 300)
                    {
                        if (lastMag > tempAngle) loopCount++;
                        if (lastMag < tempAngle) loopCount--;
                    }
                    //Console.WriteLine($"{MagAngle}\t{lastMag}\t{tempAngle}");
                    MagAngle += 360 * loopCount;

                    IMUData.Magx = MagX;
                    IMUData.Magy = MagY;
                    IMUData.Magz = MagZ;
                    IMUData.MagAngle = MagAngle;
                }

                // Sprawdzanie czy dane są dostępne dla MPU6050
                bool isDataReadyMPU = (ReadByte(0x3A, MPU6050) & 1) > 0;
                if (isDataReadyMPU)
                {
                    // Surowy odczyt z akcelerometru
                    var acc_x = Read2Byte(ACCEL_XOUT_H, MPU6050);
                    var acc_y = Read2Byte(ACCEL_YOUT_H, MPU6050);
                    var acc_z = Read2Byte(ACCEL_ZOUT_H, MPU6050);

                    // Surowy odczyt z żyroskopu
                    var gyro_x = Read2Byte(GYRO_XOUT_H, MPU6050);
                    var gyro_y = Read2Byte(GYRO_YOUT_H, MPU6050);
                    var gyro_z = Read2Byte(GYRO_ZOUT_H, MPU6050);

                    // Skalowanie pomiarów do wartości g i stopni/s
                    var scaledAx = acc_x / 16384.0;
                    var scaledAy = acc_y / 16384.0;
                    var scaledAz = acc_z / 16384.0;

                    var scaledGx = gyro_x / 131.0;
                    var scaledGy = gyro_y / 131.0;
                    var scaledGz = gyro_z / 131.0;

                    // Sprawdzanie czy pomiar wychodzi poza martwą strefę
                    Ax = Math.Abs(scaledAx) > AccDeadZone ? scaledAx : 0;
                    Ay = Math.Abs(scaledAy) > AccDeadZone ? scaledAy : 0;
                    Az = Math.Abs(scaledAz + 1) > AccDeadZone ? scaledAz : -1;

                    Gx = Math.Abs(scaledGx) > GyroDeaZone ? scaledGx : 0;
                    Gy = Math.Abs(scaledGy) > GyroDeaZone ? scaledGy : 0;
                    Gz = Math.Abs(scaledGz) > GyroDeaZone ? scaledGz : 0;

                    float dt = 0.005f;
                    float scale = 9.5f;

                    GyroAngle -= Gz * dt * scale;

                    // Reset danych żyroskopu w przypadu dużej różnicy
                    if (GyroAngle != 0 && Math.Abs(GyroAngle - MagAngle) > 90) GyroAngle = MagAngle;

                    IMUData.Ax = Ax;
                    IMUData.Ay = Ay;
                    IMUData.Az = Az;
                    IMUData.Gx = Gx;
                    IMUData.Gy = Gy;
                    IMUData.Gz = Gz;
                }

                float W = 0.8f;                                         // Waga danych z żyroskopu

                GyroAngle = GyroAngle == 0 ? MagAngle : GyroAngle;      // Pierwsze przypisanie wartości do kąta z żyroskopu
                Angle = (GyroAngle * W) + (MagAngle * (1 - W));         // Oblicznia kąta z obu pomiarów

                IMUData.Angle = Angle;
                IMUData.GyroAngle = GyroAngle;
            }

            void GPS_Init()
            {
                sp.Encoding = Encoding.UTF8;
                sp.BaudRate = 9600;
                sp.ReadTimeout = 1000;
                sp.WriteTimeout = 1000;
                sp.Open();

                //PMTK_SET_NMEA_OUTPUT_GGAONLY
                sp.Write("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
                //Task.Delay(200);
                //PMTK_SET_NMEA_UPDATE_1HZ
                sp.Write("$PMTK220,1000*1F");
                //Task.Delay(200);
                //PGCMD_ANTENNA
                sp.Write("$PGCMD,33,1*6C");
                //Task.Delay(200);
            }

            void ReadGPS()
            {
                try
                {
                    int bytes = sp.BytesToRead;
                    while (bytes > 100)
                    {
                        // sprawdzanie czy tekst nie jest pusty
                        string existingData = sp.ReadLine();
                        if (existingData == null)
                        {
                            continue;
                        }
                        string[] splitData = existingData.Split(',');



                        // sprawdzanie czy tekst jest w oczekiwanym formacie i dekodowanie danych
                        if (splitData[0].EndsWith("GGA"))
                        {
                            gpsData.time = new TimeOnly(int.Parse(splitData[1].Substring(0, 2)),
                                int.Parse(splitData[1].Substring(2, 2)),
                                int.Parse(splitData[1].Substring(4, 2)),
                                int.Parse(splitData[1].Substring(7, 2)));

                            if (splitData[2].Length > 0)
                            {
                                gpsData.latitude = double.Parse(splitData[2], System.Globalization.CultureInfo.InvariantCulture);
                            }
                            if (splitData[3].Length > 0)
                            {
                                gpsData.latitudeDirection = char.Parse(splitData[3]);
                            }
                            if (splitData[4].Length > 0)
                            {
                                gpsData.longitude = double.Parse(splitData[4], System.Globalization.CultureInfo.InvariantCulture);
                            }
                            if (splitData[5].Length > 0)
                            {
                                gpsData.longitudeDirection = char.Parse(splitData[5]);
                            }

                            gpsData.sateliteNumber = int.Parse(splitData[7]);

                            if (splitData[9].Length > 0)
                            {
                                gpsData.altitude = double.Parse(splitData[9], System.Globalization.CultureInfo.InvariantCulture);
                            }
                        }



                        bytes = sp.BytesToRead;
                    }
                }
                catch (Exception exception)
                {
                    Console.WriteLine(exception.Message);
                }
            }

            void ServoRotAction()
            {
                // Obrót serwomechanizmu w lewo
                for (int i = 0; i < MesurementNumber; i++)
                {
                    RotMesure(i);
                }

                // Obrót serwomechanizmu w prawo
                for (int i = MesurementNumber - 1; i >= 0; i--)
                {
                    RotMesure(i);
                }
            }

            void RotMesure(int i)
            {
                // Obliczanie wartości PWM dla określonego kąta
                var Angle = (i * dAngle) + ((180 - range) / 2);
                var outPwm = ((Angle + 8) / 1800d) + 0.023;
                Servo.DutyCycle = outPwm;

                // Przerwa na obrót serwa
                Task.Delay(3 * dAngle);

                double distance = GetDistance(100); // 100 jest liczbą kodową dla czujnika na serwie
                rotSensorData[i] = new RotSensorData { Distance = distance, Angle = Angle };

                // Przeniesienie danych do bezpiecznej przestrzeni
                rotSensorDatas.Insert(i, rotSensorData[i]);
                rotSensorDatas.RemoveAt(i + 1);
            }

            void GY271Init()
            {
                // Adresy rejetrów
                byte Device_Address = 0x0D;
                byte RegCTRL1 = 0x09;    // Control Register--> MSB(OSR:2,RNG:2,ODR:2,MODE:2)LSB
                byte RegCTRL2 = 0x0A;    // Control Register2--> MSB(Soft_RS:1,Rol_PNT:1,none:5,INT_ENB:1)LSB
                byte RegFBR = 0x0B;      // SET/RESET Period Register--> MSB(FBR:8)LSB

                // Definicje byte dla pierwszego rejestru sterującego
                byte Mode_Standby = 0b00000000;
                byte Mode_Continuous = 0b00000001;
                byte ODR_10Hz = 0b00000000;
                byte ODR_50Hz = 0b00000100;
                byte ODR_100Hz = 0b00001000;
                byte ODR_200Hz = 0b00001100;
                byte RNG_2G = 0b00000000;
                byte RNG_8G = 0b00010000;
                byte OSR_512 = 0b00000000;
                byte OSR_256 = 0b01000000;
                byte OSR_128 = 0b10000000;
                byte OSR_64 = 0b11000000;

                // Inicjalizacja
                GY271 = I2cDevice.Create(new I2cConnectionSettings(1, Device_Address));

                byte ctrl1 = (byte)(Mode_Continuous | ODR_200Hz | RNG_8G | OSR_512);

                GY271.Write(new byte[] { RegCTRL1, ctrl1 });
                GY271.Write(new byte[] { RegFBR, 0x01 });
            }

            double GetDistance(int sensorIndex)
            {
                Hcsr04 TempSensor = sensorIndex == 100 ? ServoSensor : sensors[sensorIndex];
                UnitsNet.Length length;

                if (sensorIndex == 0 || sensorIndex == 1 || sensorIndex == 2 || sensorIndex == 3) return 400;
                if (TempSensor.TryGetDistance(out length))
                {
                    //Console.WriteLine("Czujnik nr. {0}, pomiar: {1}", sensorIndex, length.Centimeters);
                    return length.Centimeters;
                }
                else
                {
                    //Console.WriteLine("Błąd odczytu czujnika nr: " + sensorIndex);
                    return 400;
                }
            }

            void SensorMesure(int Sensorindex)
            {
                double distance = GetDistance(Sensorindex);
                sonicData[Sensorindex] = distance;

                SonicData.Insert(Sensorindex, sonicData[Sensorindex]);
                SonicData.RemoveAt(Sensorindex + 1);
            }
        }
    }
}