// ShipMonitor - live 3D ASCII ship for the IMU-FuzzyPID board (Nucleo-F446RE + MPU6050).
// Reads the board's serial dashboard (heading, yaw rate, roll/pitch, rudder command) and draws a
// shaded 3D ship on a compass-marked sea that turns, rolls, pitches and swings its rudder live.
//
// Build (the .NET Framework compiler ships with Windows, nothing to install):
//   C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe /nologo /optimize ShipMonitor.cs
// Run (close PuTTY first: only one program can hold the COM port):
//   ShipMonitor.exe                      COM7 at 230400 baud
//   ShipMonitor.exe COM5 115200          another port / baud
//   ShipMonitor.exe --frame 30 5 -3 15   print one frame (heading, roll, pitch, rudder) and exit
//
// Same conventions as the firmware: heading + = clockwise (N = boot heading = setpoint),
// roll + = starboard down, pitch + = bow down, rudder + = trailing edge to starboard.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;

struct V3
{
    public double X, Y, Z;  // body: X bow, Y starboard, Z up.   world: X north, Y east, Z up
    public V3(double x, double y, double z) { X = x; Y = y; Z = z; }
    public static V3 operator +(V3 a, V3 b) { return new V3(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
    public static V3 operator -(V3 a, V3 b) { return new V3(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
    public static V3 operator *(V3 a, double k) { return new V3(a.X * k, a.Y * k, a.Z * k); }
    public static double Dot(V3 a, V3 b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }
    public static V3 Cross(V3 a, V3 b) { return new V3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X); }
    public V3 Unit() { double l = Math.Sqrt(Dot(this, this)); return l > 1e-12 ? this * (1 / l) : this; }
}

struct Tri
{
    public V3 A, B, C; public int Mat;
    public Tri(V3 a, V3 b, V3 c, int mat) { A = a; B = b; C = c; Mat = mat; }
}

static class ShipMonitor
{
    const int HULL = 0, BOTTOM = 1, DECK = 2, HOUSE = 3, FUNNEL = 4, RUDDER = 5;
    static readonly string[] MatColor = { "97", "31", "33", "37", "93", "95" };  // ANSI white, red, yellow, grey, bright yellow, magenta
    const string Ramp = ".:-=+*#%@";                                            // shading, dark -> bright
    const double Deg = Math.PI / 180, SeaRadius = 9.5, RingRadius = 7.8;

    // Latest board values {heading, yaw rate, roll, pitch, rudder}, written by the reader thread.
    static readonly object Gate = new object();
    static readonly double[] Board = new double[5];
    static DateTime lastData = DateTime.MinValue;
    static string link = "starting";
    static volatile bool running = true;

    static readonly List<Tri> ShipMesh = BuildShip();

    static int Main(string[] args)
    {
        if (args.Length == 5 && args[0] == "--frame")
        {
            double[] v = new double[4];
            for (int i = 0; i < 4; i++) v[i] = double.Parse(args[i + 1], CultureInfo.InvariantCulture);
            Console.Write(Frame(100, 34, v[0], v[1], v[2], v[3], 0, false));
            return 0;
        }
        string portName = args.Length > 0 ? args[0] : "COM7";
        int baud = args.Length > 1 ? int.Parse(args[1]) : 230400;

        EnableVirtualTerminal();
        Console.CancelKeyPress += (s, e) => { e.Cancel = true; running = false; };
        var reader = new Thread(() => ReadLoop(portName, baud));
        reader.IsBackground = true;
        reader.Start();

        Stream o = Console.OpenStandardOutput();
        double[] show = new double[5];  // smoothed copy, so 10 Hz board data animates at 25 fps
        int lastW = 0, lastH = 0;
        var clock = Stopwatch.StartNew();
        Write(o, "\x1b[?25l\x1b[2J");
        while (running)
        {
            long t0 = clock.ElapsedMilliseconds;
            int W = Console.WindowWidth - 1, H = Console.WindowHeight - 3;
            if (W != lastW || H != lastH) { Write(o, "\x1b[2J"); lastW = W; lastH = H; }

            double[] target; DateTime last; string lk;
            lock (Gate) { target = (double[])Board.Clone(); last = lastData; lk = link; }
            for (int i = 0; i < 5; i++)
            {
                double d = target[i] - show[i];
                show[i] += 0.3 * (i == 0 ? Wrap180(d) : d);
            }
            show[0] = Wrap180(show[0]);

            var sb = new StringBuilder("\x1b[H");
            if (W < 50 || H < 16)
                sb.Append("\x1b[0mEnlarge the window to at least 51 x 19.\x1b[K");
            else
            {
                double age = (DateTime.Now - last).TotalSeconds;
                string state = last == DateTime.MinValue ? "\x1b[91mNO DATA YET\x1b[0m"
                             : age > 1 ? string.Format(CultureInfo.InvariantCulture, "\x1b[91mNO DATA {0:0.0} s\x1b[0m", age)
                             : "\x1b[92mLIVE\x1b[0m";
                sb.Append(Pad(string.Format(" SHIP MONITOR  |  {0}  |  ", lk), W, state)).Append("\r\n");
                sb.Append(Pad(string.Format(CultureInfo.InvariantCulture,
                    " Heading {0,7:+0.0;-0.0} deg | Yaw rate {1,7:+0.00;-0.00} deg/s | Roll {2,6:+0.0;-0.0} deg | Pitch {3,6:+0.0;-0.0} deg | Rudder {4,6:+0.0;-0.0} deg {5}",
                    target[0], target[1], target[2], target[3], target[4], RudderBar(target[4])), W, "")).Append("\r\n");
                sb.Append(Frame(W, H, show[0], show[2], show[3], show[4], clock.ElapsedMilliseconds / 1000.0, true));
                sb.Append(Pad(" N = boot heading = setpoint   roll + = stbd down   pitch + = bow down   rudder + = to stbd   Ctrl+C quits", W, ""));
            }
            Write(o, sb.ToString());
            Thread.Sleep((int)Math.Max(0, 40 - (clock.ElapsedMilliseconds - t0)));
        }
        Write(o, "\x1b[0m\x1b[?25h\r\n");
        return 0;
    }

    // ---- Serial: parse the board's dashboard lines, reopen the port if it drops ----
    static void ReadLoop(string name, int baud)
    {
        var hdg = new Regex(@"Heading \(psi\)\s*:\s*([-+]?\d+\.\d+)");
        var yaw = new Regex(@"Yaw Rate \(r\)\s*:\s*([-+]?\d+\.\d+)");
        var tilt = new Regex(@"Roll / Pitch\s*:\s*([-+]?\d+\.\d+)\s*/\s*([-+]?\d+\.\d+)");
        var rud = new Regex(@"Steering Cmd \(delta_cmd\)\s*:\s*([-+]?\d+\.\d+)");
        while (running)
        {
            try
            {
                SetLink("opening " + name);
                using (var port = new SerialPort(name, baud))
                {
                    port.NewLine = "\r\n";
                    port.ReadTimeout = 1000;
                    port.Open();
                    SetLink(string.Format("{0} @ {1}", name, baud));
                    while (running)
                    {
                        string line;
                        try { line = port.ReadLine(); } catch (TimeoutException) { continue; }
                        Match m;
                        lock (Gate)
                        {
                            if ((m = hdg.Match(line)).Success) { Board[0] = Num(m, 1); lastData = DateTime.Now; }
                            else if ((m = yaw.Match(line)).Success) Board[1] = Num(m, 1);
                            else if ((m = tilt.Match(line)).Success) { Board[2] = Num(m, 1); Board[3] = Num(m, 2); }
                            else if ((m = rud.Match(line)).Success) Board[4] = Num(m, 1);
                        }
                    }
                }
            }
            catch (UnauthorizedAccessException) { SetLink(name + " is busy - close PuTTY / other serial monitors"); }
            catch (Exception ex) { SetLink(name + ": " + ex.Message.Trim()); }
            Thread.Sleep(1000);
        }
    }

    static double Num(Match m, int g) { return double.Parse(m.Groups[g].Value, CultureInfo.InvariantCulture); }
    static void SetLink(string s) { lock (Gate) link = s; }

    // ---- Ship model, body frame: x to the bow, y to starboard, z up, waterline at z = 0 ----
    static V3 Mirror(V3 v) { return new V3(v.X, -v.Y, v.Z); }

    static void Quad(List<Tri> m, V3 a, V3 b, V3 c, V3 d, int mat) { m.Add(new Tri(a, b, c, mat)); m.Add(new Tri(a, c, d, mat)); }

    static void Box(List<Tri> m, double x0, double x1, double y0, double y1, double z0, double z1, int mat)
    {
        var p = new V3[8];  // corner i: bit 0 picks x1, bit 1 picks y1, bit 2 picks z1
        for (int i = 0; i < 8; i++) p[i] = new V3((i & 1) != 0 ? x1 : x0, (i & 2) != 0 ? y1 : y0, (i & 4) != 0 ? z1 : z0);
        Quad(m, p[0], p[1], p[3], p[2], mat); Quad(m, p[4], p[5], p[7], p[6], mat);  // bottom, top
        Quad(m, p[0], p[1], p[5], p[4], mat); Quad(m, p[2], p[3], p[7], p[6], mat);  // port, starboard
        Quad(m, p[0], p[2], p[6], p[4], mat); Quad(m, p[1], p[3], p[7], p[5], mat);  // aft, forward
    }

    static List<Tri> BuildShip()
    {
        // Hull stations bow -> stern, port side (starboard mirrors): deck edge, chine at the waterline, keel.
        V3[] deck  = { new V3(5.4, 0, 1.10),  new V3(2.8, -1.55, 0.85),  new V3(-1.5, -1.75, 0.75),  new V3(-4.8, -1.55, 0.80) };
        V3[] chine = { new V3(5.0, 0, -0.05), new V3(2.8, -1.40, -0.10), new V3(-1.5, -1.60, -0.12), new V3(-4.8, -1.40, -0.08) };
        V3[] keel  = { new V3(4.5, 0, -0.75), new V3(2.8, 0, -0.95),     new V3(-1.5, 0, -1.00),     new V3(-4.8, 0, -0.80) };
        var m = new List<Tri>();
        for (int i = 0; i < 3; i++)
        {
            Quad(m, deck[i], Mirror(deck[i]), Mirror(deck[i + 1]), deck[i + 1], DECK);
            Quad(m, deck[i], deck[i + 1], chine[i + 1], chine[i], HULL);
            Quad(m, Mirror(deck[i]), Mirror(deck[i + 1]), Mirror(chine[i + 1]), Mirror(chine[i]), HULL);
            Quad(m, chine[i], chine[i + 1], keel[i + 1], keel[i], BOTTOM);
            Quad(m, Mirror(chine[i]), Mirror(chine[i + 1]), keel[i + 1], keel[i], BOTTOM);
        }
        Quad(m, deck[3], Mirror(deck[3]), Mirror(chine[3]), chine[3], HULL);  // transom
        m.Add(new Tri(chine[3], Mirror(chine[3]), keel[3], BOTTOM));
        Box(m, -2.4, 0.4, -1.05, 1.05, 0.75, 1.95, HOUSE);     // wheelhouse
        Box(m, -3.6, -2.9, -0.35, 0.35, 0.80, 2.60, FUNNEL);   // funnel
        Box(m, 2.15, 2.35, -0.10, 0.10, 0.85, 2.60, HOUSE);    // foremast
        return m;
    }

    // Rudder blade hinged on the transom, swung by delta (+ = trailing edge to starboard).
    static List<Tri> Rudder(double delta)
    {
        var blade = new List<Tri>();
        Box(blade, -1.0, 0, -0.09, 0.09, -1.20, -0.15, RUDDER);  // hinge line at x = 0
        double c = Math.Cos(delta * Deg), s = Math.Sin(delta * Deg);
        var m = new List<Tri>();
        foreach (Tri t in blade)
            m.Add(new Tri(Swing(t.A, c, s), Swing(t.B, c, s), Swing(t.C, c, s), RUDDER));
        return m;
    }

    static V3 Swing(V3 v, double c, double s) { return new V3(-4.8 + v.X * c + v.Y * s, -v.X * s + v.Y * c, v.Z); }

    // Body -> world: roll about x (+ stbd down), pitch about y (+ bow down), heading about z (+ clockwise).
    static V3 ToWorld(V3 v, double sr, double cr, double sp, double cp, double sh, double ch)
    {
        double y = v.Y * cr + v.Z * sr, z = v.Z * cr - v.Y * sr;
        double x = v.X * cp + z * sp; z = z * cp - v.X * sp;
        return new V3(x * ch - y * sh, x * sh + y * ch, z);
    }

    // ---- Renderer: perspective camera south-south-west of the ship, z-buffered ASCII shading ----
    static string Frame(int W, int H, double hdg, double roll, double pitch, double rudder, double t, bool color)
    {
        var ch = new char[W * H];
        var col = new string[W * H];
        var zb = new double[W * H];
        for (int i = 0; i < ch.Length; i++) { ch[i] = ' '; zb[i] = double.MaxValue; }

        double az = 210 * Deg, el = 35 * Deg, dist = 20;
        var eye = new V3(dist * Math.Cos(el) * Math.Cos(az), dist * Math.Cos(el) * Math.Sin(az), dist * Math.Sin(el));
        var f = (new V3(0, 0, -0.2) - eye).Unit();
        var right = V3.Cross(new V3(0, 0, 1), f).Unit();  // world axes are N, E, Up, so up x f points right
        var up = V3.Cross(f, right);
        double F = Math.Min(W * 0.9, H * 2.6), cx = W / 2.0, cy = H * 0.45;
        Func<V3, V3> project = p =>
        {
            var d = p - eye;
            double z = V3.Dot(d, f);
            return new V3(cx + F * V3.Dot(d, right) / z, cy - 0.5 * F * V3.Dot(d, up) / z, z);  // 0.5: cells are ~2:1
        };
        Action<V3, char, string> plot = (p, c, k) =>
        {
            var s = project(p);
            int x = (int)Math.Floor(s.X), y = (int)Math.Floor(s.Y);
            if (x >= 0 && x < W && y >= 0 && y < H) { ch[y * W + x] = c; col[y * W + x] = k; }
        };

        // Sea: cast each cell onto the water plane, animated swell inside a disc around the ship.
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
            {
                var dir = f + right * ((x + 0.5 - cx) / F) + up * (-(y + 0.5 - cy) / (0.5 * F));
                if (dir.Z > -1e-6) continue;
                var p = eye + dir * (-eye.Z / dir.Z);
                if (p.X * p.X + p.Y * p.Y > SeaRadius * SeaRadius) continue;
                double w = Math.Sin(0.9 * p.X + 1.7 * t) + Math.Sin(0.7 * p.Y - 1.1 * t + 0.5 * p.X);
                ch[y * W + x] = w > 1.3 ? '~' : w > 0.6 ? '.' : ' ';
                col[y * W + x] = w > 1.3 ? "94" : "34";
            }

        // Compass ring (N = boot heading = setpoint) and the current heading line out to it.
        for (int a = 0; a < 360; a += 5)
            plot(new V3(RingRadius * Math.Cos(a * Deg), RingRadius * Math.Sin(a * Deg), 0), a % 30 == 0 ? '+' : '.', "90");
        string[] names = { "N", "E", "S", "W" };
        for (int i = 0; i < 4; i++)
            plot(new V3((RingRadius + 0.9) * Math.Cos(i * 90 * Deg), (RingRadius + 0.9) * Math.Sin(i * 90 * Deg), 0), names[i][0], "96");
        for (double r = 5.9; r < RingRadius + 0.5; r += 0.35)
            plot(new V3(r * Math.Cos(hdg * Deg), r * Math.Sin(hdg * Deg), 0), r > RingRadius - 0.2 ? '@' : 'o', "92");

        // Ship: flat-shaded triangles, z-buffered, drawn over the sea.
        double sr = Math.Sin(roll * Deg), cr = Math.Cos(roll * Deg), sp = Math.Sin(pitch * Deg), cp = Math.Cos(pitch * Deg);
        double sh = Math.Sin(hdg * Deg), chd = Math.Cos(hdg * Deg);
        var light = new V3(-0.25, -0.35, 0.90).Unit();  // high, from the camera side: deck brightest
        var tris = new List<Tri>(ShipMesh);
        tris.AddRange(Rudder(Math.Max(-30, Math.Min(30, rudder))));
        foreach (Tri tr in tris)
        {
            V3 a = ToWorld(tr.A, sr, cr, sp, cp, sh, chd), b = ToWorld(tr.B, sr, cr, sp, cp, sh, chd), c = ToWorld(tr.C, sr, cr, sp, cp, sh, chd);
            var n = V3.Cross(b - a, c - a).Unit();
            if (V3.Dot(n, eye - a) < 0) n = n * -1;  // light the side facing the camera
            double lum = 0.12 + 0.88 * Math.Max(0, V3.Dot(n, light));
            Raster(project(a), project(b), project(c), Ramp[(int)(lum * (Ramp.Length - 0.01))], MatColor[tr.Mat], W, H, ch, col, zb);
        }

        var sb = new StringBuilder();
        for (int y = 0; y < H; y++)
        {
            string cur = null;
            for (int x = 0; x < W; x++)
            {
                int i = y * W + x;
                if (color && col[i] != cur) { sb.Append("\x1b[").Append(col[i] ?? "0").Append('m'); cur = col[i]; }
                sb.Append(ch[i]);
            }
            if (color) sb.Append("\x1b[0m");
            sb.Append("\r\n");
        }
        return sb.ToString();
    }

    // Fill one projected triangle (x, y = cell coords, z = depth) where it is nearer than what is there.
    static void Raster(V3 a, V3 b, V3 c, char glyph, string color, int W, int H, char[] ch, string[] col, double[] zb)
    {
        if (a.Z < 0.1 || b.Z < 0.1 || c.Z < 0.1) return;
        double area = (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
        if (Math.Abs(area) < 1e-9) return;
        int x0 = Math.Max(0, (int)Math.Floor(Math.Min(a.X, Math.Min(b.X, c.X))));
        int x1 = Math.Min(W - 1, (int)Math.Ceiling(Math.Max(a.X, Math.Max(b.X, c.X))));
        int y0 = Math.Max(0, (int)Math.Floor(Math.Min(a.Y, Math.Min(b.Y, c.Y))));
        int y1 = Math.Min(H - 1, (int)Math.Ceiling(Math.Max(a.Y, Math.Max(b.Y, c.Y))));
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
            {
                double px = x + 0.5, py = y + 0.5;
                double w0 = ((b.X - px) * (c.Y - py) - (b.Y - py) * (c.X - px)) / area;
                double w1 = ((c.X - px) * (a.Y - py) - (c.Y - py) * (a.X - px)) / area;
                double w2 = 1 - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0) continue;
                double z = w0 * a.Z + w1 * b.Z + w2 * c.Z;
                int i = y * W + x;
                if (z < zb[i]) { zb[i] = z; ch[i] = glyph; col[i] = color; }
            }
    }

    // ---- Small helpers ----
    static double Wrap180(double a) { a %= 360; return a > 180 ? a - 360 : a < -180 ? a + 360 : a; }

    static string RudderBar(double d)  // [    <<<|     ] PORT..STBD, 10 cells a side for +-30 deg
    {
        int n = (int)Math.Round(Math.Min(1, Math.Abs(d) / 30) * 10);
        return "PORT [" + new string(' ', d < 0 ? 10 - n : 10) + new string('<', d < 0 ? n : 0) + "|"
             + new string('>', d > 0 ? n : 0) + new string(' ', d > 0 ? 10 - n : 10) + "] STBD";
    }

    static string Pad(string s, int w, string tail)  // plain text cut / padded to w, then an (escape-coded) tail
    {
        if (s.Length > w) s = s.Substring(0, w);
        return s + tail + "\x1b[K";
    }

    static void Write(Stream o, string s) { byte[] b = Encoding.ASCII.GetBytes(s); o.Write(b, 0, b.Length); o.Flush(); }

    [DllImport("kernel32.dll")] static extern IntPtr GetStdHandle(int n);
    [DllImport("kernel32.dll")] static extern bool GetConsoleMode(IntPtr h, out uint mode);
    [DllImport("kernel32.dll")] static extern bool SetConsoleMode(IntPtr h, uint mode);

    static void EnableVirtualTerminal()  // ANSI colours / cursor moves in conhost too (Windows Terminal has them on)
    {
        IntPtr h = GetStdHandle(-11);
        uint mode;
        if (GetConsoleMode(h, out mode)) SetConsoleMode(h, mode | 4);
    }
}
