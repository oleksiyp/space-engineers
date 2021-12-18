using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        public List<T> GetBlocksOfType<T>(List<T> blocks, Func<T, bool> collect = null) where T : class
        {
            if (blocks == null)
            {
                blocks = new List<T>();
                GridTerminalSystem.GetBlocksOfType(blocks, collect);
                blocks.RemoveAll(it => !((IMyTerminalBlock)it).IsSameConstructAs(Me));
            }
            return blocks;
        }

        public T GetBlockOfType<T>(T block, Func<T, bool> collect = null) where T : class
        {
            if (block == null)
            {
                List<T> blocks = GetBlocksOfType(null as List<T>, collect);
                if (blocks.Count > 0)
                {
                    block = blocks[0];
                }
                else
                {
                    throw new Exception("missing block: " + typeof(T).Name);
                }
            }
            return block;
        }

        public static void ApplyAction<T>(List<T> blocks, string actionName) where T : class
        {
            foreach (IMyTerminalBlock block in blocks)
            {
                block.ApplyAction(actionName);
            }
        }

        public static void Each<T>(List<T> blocks, Action<T> action) where T : class
        {
            foreach (T block in blocks)
            {
                action.Invoke(block);
            }
        }
        public IMyBroadcastListener RegisterBroadcastListener(IMyBroadcastListener listener, String tag, String callback = null)
        {
            if (listener == null)
            {
                listener = IGC.RegisterBroadcastListener(tag);
                if (callback != null)
                {
                    listener.SetMessageCallback(callback);
                }
            }
            return listener;
        }

        public static void setThrust(IMyThrust t, double thrust)
        {
            if (thrust > t.MaxThrust)
            {
                thrust = t.MaxThrust;
            }
            else if (thrust < 0)
            {
                thrust = 0;
            }

            t.ThrustOverride = (float)(thrust * t.MaxThrust / t.MaxEffectiveThrust);
        }

        public class ExitFlag
        {
            public bool ShouldExit { get; set; }
        }

        public class TimerRoutineRunner
        {
            private Dictionary<String, TimerStateMachine> machines = new Dictionary<String, TimerStateMachine>();
            private Program program;
            private int counter = 0;


            public TimerRoutineRunner(Program program)
            {
                this.program = program;
            }

            public bool isRunning(String name)
            {
                if (machines.ContainsKey(name))
                {
                    return machines[name].Running;
                }
                return false;
            }

            public void Start(String name, Func<ExitFlag, IEnumerable<double>> routineFn)
            {
                Stop(name);
                ExitFlag flag = new ExitFlag();
                TimerStateMachine tsm = new TimerStateMachine(program, routineFn(flag), flag);
                tsm.Start();
                machines.Remove(name);
                machines.Add(name, tsm);
            }

            public bool Toggle(String name, Func<ExitFlag, IEnumerable<double>> routineFn)
            {
                if (isRunning(name))
                {
                    Stop(name);
                    return true;
                }
                else
                {
                    ExitFlag flag = new ExitFlag();
                    TimerStateMachine tsm = new TimerStateMachine(program, routineFn(flag), flag);
                    tsm.Start();
                    machines.Remove(name);
                    machines.Add(name, tsm);
                    return false;
                }
            }

            public String Start(Func<ExitFlag, IEnumerable<double>> routineFn)
            {
                String id = "###id" + (counter++).ToString();
                Start(id, routineFn);
                return id;
            }

            public void Stop(String name)
            {
                if (machines.ContainsKey(name))
                {
                    TimerStateMachine machine = machines[name];
                    if (machine != null)
                    {
                        machine.Stop();
                    }
                }
            }

            public void Run()
            {
                List<String> toRemove = new List<String>();
                foreach (var key in machines.Keys)
                {
                    TimerStateMachine m = machines[key];
                    m.Run();
                    if (!m.Running)
                    {
                        toRemove.Add(key);
                    }
                }
                foreach (var key in toRemove)
                {
                    machines.Remove(key);
                }
            }
        }

        /// <summary>
        /// Quick usage:
        /// <para>1. A persistent instance for each sequence you want to run in parallel.</para>
        /// <para>2. Create instance(s) in Program() and execute <see cref="Run"/> in Main().</para>
        /// </summary>
        public class TimerStateMachine
        {
            public readonly Program Program;

            /// <summary>
            /// Wether the timer starts automatically at initialization and auto-restarts it's done iterating.
            /// </summary>
            public bool AutoStart { get; set; }

            /// <summary>
            /// Returns true if a sequence is actively being cycled through.
            /// False if it ended or no sequence is assigned anymore.
            /// </summary>
            public bool Running { get; private set; }

            /// <summary>
            /// Setting this will change what sequence will be used when it's (re)started.
            /// </summary>
            public IEnumerable<double> Sequence;

            /// <summary>
            /// Time left until the next part is called.
            /// </summary>
            public double SequenceTimer { get; private set; }

            public ExitFlag ExitFlag { get; private set; }

            private IEnumerator<double> sequenceSM;

            public TimerStateMachine(Program program, IEnumerable<double> sequence = null, ExitFlag flag = null, bool autoStart = false)
            {
                Program = program;
                Sequence = sequence;
                AutoStart = autoStart;
                ExitFlag = flag;

                if (AutoStart)
                {
                    Start();
                }
            }

            /// <summary>
            /// (Re)Starts sequence, even if already running.
            /// Don't forget <see cref="IMyGridProgramRuntimeInfo.UpdateFrequency"/>.
            /// </summary>
            public void Start()
            {
                SetSequenceSM(Sequence);
            }

            public void Stop()
            {
                if (ExitFlag != null)
                {
                    ExitFlag.ShouldExit = true;
                }
                else
                {
                    SetSequenceSM(null);
                }
            }

            /// <summary>
            /// <para>Call this in your <see cref="Rocket.Main(string, UpdateType)"/> and have a reasonable update frequency, usually Update10 is good for small delays, Update100 for 2s or more delays.</para>
            /// <para>Checks if enough time passed and executes the next chunk in the sequence.</para>
            /// <para>Does nothing if no sequence is assigned or it's ended.</para>
            /// </summary>
            public void Run()
            {
                if (sequenceSM == null)
                    return;

                SequenceTimer -= Program.Runtime.TimeSinceLastRun.TotalSeconds;

                if (SequenceTimer > 0)
                    return;

                bool hasValue;
                while (true)
                {
                    hasValue = sequenceSM.MoveNext();

                    if (hasValue)
                    {
                        SequenceTimer = sequenceSM.Current;

                        if (SequenceTimer <= kEpsilon)
                        {
                            if (Program.Runtime.CurrentInstructionCount < Program.Runtime.MaxInstructionCount * 0.8)
                            {
                                continue;
                            }
                        } 
                        else if (SequenceTimer <= -0.5)
                        {
                            hasValue = false;
                        }
                    }
                    break;
                }

                if (!hasValue)
                {
                    if (AutoStart)
                        SetSequenceSM(Sequence);
                    else
                        SetSequenceSM(null);
                }
            }

            private void SetSequenceSM(IEnumerable<double> seq)
            {
                Running = false;
                SequenceTimer = 0;

                sequenceSM?.Dispose();
                sequenceSM = null;

                if (seq != null)
                {
                    Running = true;
                    sequenceSM = seq.GetEnumerator();
                }
            }
        }

        public Quaternion GetGridRotation(IMyRemoteControl remote)
        {

            return Quaternion.CreateFromRotationMatrix(remote.WorldMatrix.GetOrientation());
        }


        public static Vector3D QuaternionToPitchRollYaw(QuaternionD data)
        {
            double q2sqr = data.Y * data.Y;
            double t0 = -2.0 * (q2sqr + data.Z * data.Z) + 1.0;
            double t1 = +2.0 * (data.X * data.Y + data.W * data.Z);
            double t2 = -2.0 * (data.X * data.Z - data.W * data.Y);
            double t3 = +2.0 * (data.Y * data.Z + data.W * data.X);
            double t4 = -2.0 * (data.X * data.X + q2sqr) + 1.0;

            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;

            return new Vector3D(
                Math.Asin(t2),
                Math.Atan2(t1, t0),
                Math.Atan2(t3, t4)
            );
        }
        public float QuaternionDot(Quaternion a, Quaternion b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
        }

        public const double kEpsilon = 0.000001;

        public double QuaternionAngle(Quaternion a, Quaternion b)
        {
            float dot = Math.Min(Math.Abs(QuaternionDot(a, b)), 1);
            if (dot > 1.0 - kEpsilon)
            {
                return 0;
            }
            else
            {
                return Math.Acos(dot) * 360 / Math.PI;
            }
        }

        public void SetGyroAngles(IMyGyro gyro, Vector3D angles)
        {
            gyro.Pitch = (float)angles.X;
            gyro.Roll = (float)angles.Y;
            gyro.Yaw = (float)angles.Z;
        }

        class TimeSource
        {
            public double Time { get; set; } = 0;

            public void Progress(IMyGridProgramRuntimeInfo info)
            {
                Time += info.TimeSinceLastRun.TotalMilliseconds;
            }
        }

        class MissileGuidance
        {
            public double Gain = 3;
            public double DampnersGain = 0.8;
            double PNGain = 3;

            double TimeSaved;

            public IMyTextPanel TextPanel { get; }

            double YawSaved;
            double PitchSaved;
            double MissileAccel = 10;

            Vector3D previousPosition;
            Vector3D previousTargetPosition;

            public List<IMyThrust> ForwardThrusters;
            public List<IMyGyro> Gyros;
            public TimeSource TimeSource;
            public IMyRemoteControl RemoteControl;

            public MissileGuidance(
                List<IMyThrust> forwardThrusters,
                List<IMyGyro> gyros,
                IMyRemoteControl rc,
                TimeSource timeSource,
                IMyTextPanel textPanel
                )
            {
                this.ForwardThrusters = forwardThrusters;
                this.Gyros = gyros;
                this.RemoteControl = rc;
                this.TimeSource = timeSource;
                this.TimeSaved = timeSource.Time;
                this.TextPanel = textPanel;
            }

            public void RefreshMass(List<IMyTerminalBlock> blocks)
            {
                double missileMass = 0;
                double missileThrust = 0;

                Each(blocks, b => missileMass += b.Mass);
                Each(ForwardThrusters, t => missileThrust += t.MaxThrust);

                MissileAccel = missileThrust / missileMass;
            }

            public void Update(Vector3D targetPosition)
            {
                if (Gyros.Count == 0)
                {
                    return;
                }

                double TimePrev = TimeSaved;
                double TickMs = Math.Max(TimeSource.Time - TimePrev, 0.018);
                TimeSaved = TimeSource.Time;

                Vector3D MissilePosition = Gyros[0].CubeGrid.WorldVolume.Center;
                Vector3D MissilePositionPrev = previousPosition;
                Vector3D MissileVelocity = (MissilePosition - MissilePositionPrev) / TickMs;

                Vector3D TargetPositionPrev = previousTargetPosition;
                Vector3D TargetVelocity = (targetPosition - previousTargetPosition) / TickMs;

                previousTargetPosition = targetPosition;
                previousPosition = MissilePosition;

                TextPanel.WriteText(TargetVelocity.ToString() + "\n" + MissileVelocity.ToString());

                //Uses RdavNav Navigation APN Guidance System
                //-----------------------------------------------

                //Setup LOS rates and PN system
                Vector3D LOS_Old = Vector3D.Normalize(TargetPositionPrev - MissilePositionPrev);
                Vector3D LOS_New = Vector3D.Normalize(targetPosition - MissilePosition);
                double len = (TargetVelocity - MissileVelocity).Length();

                Vector3D Rel_Vel = len < kEpsilon ? Vector3D.Zero : (TargetVelocity - MissileVelocity) / len;

                //And Assigners
                Vector3D am;
                double LOS_Rate;
                Vector3D LOS_Delta;
                Vector3D MissileForwards = ForwardThrusters[0].WorldMatrix.Backward;


                //Vector/Rotation Rates
                if (LOS_Old.Length() == 0)
                { LOS_Delta = new Vector3D(0, 0, 0); LOS_Rate = 0.0; }
                else
                { LOS_Delta = LOS_New - LOS_Old; LOS_Rate = LOS_Delta.Length() / TickMs; }

                //-----------------------------------------------

                //Closing Velocity
                double Vclosing = (TargetVelocity - MissileVelocity).Length();

                //If Under Gravity Use Gravitational Accel
                Vector3D GravityComp = -RemoteControl.GetNaturalGravity();

                //Calculate the final lateral acceleration
                Vector3D LateralDirection = Rel_Vel.Length() < kEpsilon ? LOS_New : Vector3D.Normalize(Vector3D.Cross(Vector3D.Cross(Rel_Vel, LOS_New), Rel_Vel));
                Vector3D LateralAccelerationComponent = LateralDirection * PNGain * LOS_Rate * Vclosing + LOS_Delta * 9.8 * (0.5 * PNGain); //Eases Onto Target Collision LOS_Delta * 9.8 * (0.5 * Gain)

                //If Impossible Solution (ie maxes turn rate) Use Drift Cancelling For Minimum T
                double OversteerReqt = (LateralAccelerationComponent).Length() / MissileAccel;
                if (OversteerReqt > 0.98)
                {
                    LateralAccelerationComponent = MissileAccel * Vector3D.Normalize(LateralAccelerationComponent + (OversteerReqt * Vector3D.Normalize(-MissileVelocity)) * 40);
                }

                //Calculates And Applies Thrust In Correct Direction (Performs own inequality check)
                double ThrustPower = Vector3D.Dot(MissileForwards, Vector3D.Normalize(LateralAccelerationComponent));
                if (ThrustPower == Double.NaN)
                {
                    ThrustPower = 0;
                }

                ThrustPower = MathHelper.Clamp(ThrustPower, 0.4, 1); //for improved thrust performance on the get-go
                foreach (IMyThrust thruster in ForwardThrusters)
                {
                    if (thruster.ThrustOverride != (thruster.MaxThrust * ThrustPower)) //12 increment inequality to help conserve on performance
                    { thruster.ThrustOverride = (float)(thruster.MaxThrust * ThrustPower); }
                }

                //Calculates Remaining Force Component And Adds Along LOS
                double RejectedAccel = Math.Sqrt(MissileAccel * MissileAccel - LateralAccelerationComponent.LengthSquared()); //Accel has to be determined whichever way you slice it
                if (double.IsNaN(RejectedAccel)) { RejectedAccel = 0; }
                LateralAccelerationComponent = LateralAccelerationComponent + LOS_New * RejectedAccel;

                //-----------------------------------------------

                //Guides To Target Using Gyros
                am = Vector3D.Normalize(LateralAccelerationComponent + GravityComp);

                //Retrieving Forwards And Up
                Vector3D ShipUp = ForwardThrusters[0].WorldMatrix.Up;
                Vector3D ShipForward = ForwardThrusters[0].WorldMatrix.Backward; //Backward for thrusters

                //Create And Use Inverse Quatinion                   
                Quaternion Quat_Two = Quaternion.CreateFromForwardUp(ShipForward, ShipUp);
                var InvQuat = Quaternion.Inverse(Quat_Two);

                Vector3D RCReferenceFrameVector = Vector3D.Transform(am, InvQuat); //Target Vector In Terms Of RC Block

                //Convert To Local Azimuth And Elevation
                double ShipForwardAzimuth = 0; double ShipForwardElevation = 0;
                Vector3D.GetAzimuthAndElevation(RCReferenceFrameVector, out ShipForwardAzimuth, out ShipForwardElevation);


                double YawPrev = YawSaved;
                double PitchPrev = PitchSaved;

                //Post Setting Factors
                YawSaved = ShipForwardAzimuth;
                PitchSaved = ShipForwardElevation;

                //Applies Some PID Damping
                ShipForwardAzimuth = ShipForwardAzimuth + DampnersGain * ((ShipForwardAzimuth - YawPrev) / TickMs);
                ShipForwardElevation = ShipForwardElevation + DampnersGain * ((ShipForwardElevation - PitchPrev) / TickMs);

                //Does Some Rotations To Provide For any Gyro-Orientation
                var REF_Matrix = MatrixD.CreateWorld(ForwardThrusters[0].GetPosition(), (Vector3)ShipForward, (Vector3)ShipUp).GetOrientation();
                var Vector = Vector3.Transform((new Vector3D(ShipForwardElevation, ShipForwardAzimuth, 0)), REF_Matrix); //Converts To World
                var TRANS_VECT = Vector3.Transform(Vector, Matrix.Transpose(Gyros[0].WorldMatrix.GetOrientation()));  //Converts To Gyro Local

                //Logic Checks for NaN's
                if (double.IsNaN(TRANS_VECT.X) || double.IsNaN(TRANS_VECT.Y) || double.IsNaN(TRANS_VECT.Z))
                { return; }

                //Applies To Scenario
                foreach (IMyGyro gyro in Gyros)
                {
                    gyro.Pitch = (float)MathHelper.Clamp((-TRANS_VECT.X) * Gain, -1000, 1000);
                    gyro.Yaw = (float)MathHelper.Clamp(((-TRANS_VECT.Y)) * Gain, -1000, 1000);
                    gyro.Roll = (float)MathHelper.Clamp(((-TRANS_VECT.Z)) * Gain, -1000, 1000);
                }
            }
        }

        public static QuaternionD ToDouble(Quaternion q)
        {
            return new QuaternionD(q.X, q.Y, q.Z, q.W);
        }


        public static QuaternionD SmoothDamp(QuaternionD rot, QuaternionD target, ref QuaternionD deriv, double time, double deltaTime)
        {
            if (time < kEpsilon)
            {
                return rot;
            }
            // account for double-cover
            var Dot = QuaternionD.Dot(rot, target);
            var Multi = Dot > 0f ? 1f : -1f;
            target.X *= Multi;
            target.Y *= Multi;
            target.Z *= Multi;
            target.W *= Multi;

            //deriv = new QuaternionD(0, 0, 0, 0);

            // smooth damp (nlerp approx)
            var Result = Vector4D.Normalize(new Vector4D(
                SmoothDamp(rot.X, target.X, ref deriv.X, time, Double.PositiveInfinity, deltaTime),
                SmoothDamp(rot.Y, target.Y, ref deriv.Y, time, Double.PositiveInfinity, deltaTime),
                SmoothDamp(rot.Z, target.Z, ref deriv.Z, time, Double.PositiveInfinity, deltaTime),
                SmoothDamp(rot.W, target.W, ref deriv.W, time, Double.PositiveInfinity, deltaTime)
            ));

            // ensure deriv is tangent
            var derivError = ProjectOnVector(new Vector4D(deriv.X, deriv.Y, deriv.Z, deriv.W), Result);
            deriv.X -= derivError.X;
            deriv.Y -= derivError.Y;
            deriv.Z -= derivError.Z;
            deriv.W -= derivError.W;

            return new QuaternionD(Result.X, Result.Y, Result.Z, Result.W);
        }

        public static Vector4D ProjectOnVector(Vector4D a, Vector4D b)
        {
            return b * (Vector4D.Dot(a, b) / Vector4D.Dot(b, b));
        }

        public static double SmoothDamp(double current, double target, ref double currentVelocity, double smoothTime, double maxSpeed, double deltaTime)
        {
            // Based on Game Programming Gems 4 Chapter 1.10
            smoothTime = Math.Max(0.0001F, smoothTime);
            double omega = 2F / smoothTime;

            double x = omega * deltaTime;
            double exp = 1F / (1F + x + 0.48F * x * x + 0.235F * x * x * x);
            double change = current - target;
            double originalTo = target;

            // Clamp maximum speed
            double maxChange = maxSpeed * smoothTime;
            change = Math.Min(Math.Max(change, -maxChange), maxChange);
            target = current - change;

            double temp = (currentVelocity + omega * change) * deltaTime;
            currentVelocity = (currentVelocity - omega * temp) * exp;
            double output = target + (change + temp) * exp;

            // Prevent overshooting
            if (originalTo - current > 0.0F == output > originalTo)
            {
                output = originalTo;
                currentVelocity = (output - originalTo) / deltaTime;
            }

            return output;
        }

        public class SEFix
        {
            public static T[] arr<T>(params T[] arg)
            {
                return arg; //becuse SE is stupid
            }
        }
        public class ColorUtils
        {
            private static double oo64 = 1.0 / 64.0;
            private static double[][] map = new double[][] {
        new double[] { 0*oo64, 48*oo64, 12*oo64, 60*oo64,  3*oo64, 51*oo64, 15*oo64, 63*oo64},
        new double[] {32*oo64, 16*oo64, 44*oo64, 28*oo64, 35*oo64, 19*oo64, 47*oo64, 31*oo64},
        new double[] { 8*oo64, 56*oo64,  4*oo64, 52*oo64, 11*oo64, 59*oo64,  7*oo64, 55*oo64},
        new double[] {40*oo64, 24*oo64, 36*oo64, 20*oo64, 43*oo64, 27*oo64, 39*oo64, 23*oo64},
        new double[] { 2*oo64, 50*oo64, 14*oo64, 62*oo64,  1*oo64, 49*oo64, 13*oo64, 61*oo64},
        new double[] {34*oo64, 18*oo64, 46*oo64, 30*oo64, 33*oo64, 17*oo64, 45*oo64, 29*oo64},
        new double[] {10*oo64, 58*oo64,  6*oo64, 54*oo64,  9*oo64, 57*oo64,  5*oo64, 53*oo64},
        new double[] {42*oo64, 26*oo64, 38*oo64, 22*oo64, 41*oo64, 25*oo64, 37*oo64, 21*oo64}
    };

            private static int[][] palette = new int[][] {
        SEFix.arr( 255, 255, 0),
        SEFix.arr( 255, 0, 0),
        SEFix.arr( 0, 0, 255),
        SEFix.arr( 0, 255, 0),
        SEFix.arr( 255, 255, 255),
        SEFix.arr( 97, 97, 97),
        SEFix.arr( 0, 0, 0)
    };
            private static string[] colorStrings = new string[] {
        "\uE004", //oh but it works fine with *strings* -_-
        "\uE003",
        "\uE002",
        "\uE001",
        "\uE007\u0458",
        "\uE00D",
        "\u2014\u0060"
    };

            private static int redC = 300;
            private static int greenC = 540;
            private static int blueC = 150;

            private static double compareColors(int r1, int g1, int b1, int r2, int g2, int b2)
            {
                double dl = ((r1 * redC + g1 * greenC + b1 * blueC) - (r2 * redC + g2 * greenC + b2 * blueC)) / 255000.0;
                double dr = (r1 - r2) / 255.0, dg = (g1 - g2) / 255.0, db = (b1 - b2) / 255.0;
                return ((dr * dr * redC + dg * dg * greenC + db * db * blueC) * 0.0075 + dl * dl);
            }
            private static double calcError(int r, int g, int b, int r0, int g0, int b0, int[] color1, int[] color2, double ratio)
            {
                return compareColors(r, g, b, r0, g0, b0) +
                    compareColors(color1[0], color1[1], color1[2], color2[0], color2[1], color2[2]) * 0.03 * (Math.Abs(ratio - 0.5) + 0.5) *
                    (1 + (color1[0] == color1[1] && color1[0] == color1[2] && color1[0] == color2[0] &&
                     color1[0] == color2[1] && color1[0] == color2[2] ? 0.03 : 0));
            }
            private static int makeRatio(int r, int g, int b, int[] c1, int[] c2)
            {
                int ratio = 32;
                if (c1[0] != c2[0] || c1[1] != c2[1] || c1[2] != c2[2])
                {
                    ratio =
                        ((c2[0] != c1[0] ? redC * 64 * (r - c1[0]) / (c2[0] - c1[0]) : 0) +
                         (c2[1] != c1[1] ? greenC * 64 * (g - c1[1]) / (c2[1] - c1[1]) : 0) +
                         (c1[2] != c2[2] ? blueC * 64 * (b - c1[2]) / (c2[2] - c1[2]) : 0)) /
                        ((c2[0] != c1[0] ? redC : 0) +
                         (c2[1] != c1[1] ? greenC : 0) +
                         (c2[2] != c1[2] ? blueC : 0));
                    if (ratio < 0)
                        ratio = 0;
                    else if (ratio > 63)
                        ratio = 63;
                }
                return ratio;
            }
            private static int[] createMix(int r, int g, int b)
            {
                int[] result = SEFix.arr(0, 0, 32);
                double minPenalty = Single.MaxValue;
                for (int i = 0; i < palette.Length; i++)
                {
                    for (int j = i; j < palette.Length; j++)
                    {
                        int ratio = makeRatio(r, g, b, palette[i], palette[j]);
                        double penalty = calcError(
                            r, g, b,
                            palette[i][0] + ratio * (palette[j][0] - palette[i][0]) / 64,
                            palette[i][1] + ratio * (palette[j][1] - palette[i][1]) / 64,
                            palette[i][2] + ratio * (palette[j][2] - palette[i][2]) / 64,
                            palette[i], palette[j],
                            (double)ratio / 64.0);
                        if (penalty < minPenalty)
                        {
                            minPenalty = penalty;
                            result[0] = i;
                            result[1] = j;
                            result[2] = ratio;
                        }
                    }
                }
                return result;
            }
            public static string[][] genDitherPattern(int r, int g, int b)
            {
                int[] mix = createMix(r, g, b);
                string[][] dithered = new string[8][];
                for (int x = 0; x < 8; x++)
                {
                    dithered[x] = new string[8];
                    for (int y = 0; y < 8; y++)
                    {
                        double mapValue = map[y & 7][x & 7];
                        double ratio = mix[2] / 64.0;
                        dithered[x][y] = colorStrings[mix[mapValue < ratio ? 1 : 0]];
                    }
                }
                return dithered;
            }
        }
        public class Ascii
        {
            private static int offset = 0x21;
            // binary numbers represent bitmap glyphs, three bits to a line
            // eg 9346 == 010 010 010 000 010 == !
            //    5265 == 001 010 010 010 001 == (
            private static short[] glyphs = SEFix.arr<short>(
                9346, 23040, 24445, 15602,
                17057, 10923, 9216, 5265,
                17556, 21824, 1488, 20,
                448, 2, 672, 31599,
                11415, 25255, 29326, 23497,
                31118, 10666, 29370, 10922,
                10954, 1040, 1044, 5393,
                3640, 17492, 25218, 15203,
                11245, 27566, 14627, 27502,
                31143, 31140, 14827, 23533,
                29847, 12906, 23469, 18727,
                24557, 27501, 11114, 27556,
                11131, 27565, 14478, 29842,
                23403, 23378, 23549, 23213,
                23186, 29351, 13459, 2184,
                25750, 10752, 7, 17408,
                239, 18862, 227, 4843,
                1395, 14756, 1886, 18861,
                8595, 4302, 18805, 25745,
                509, 429, 170, 1396,
                1369, 228, 1934, 18851,
                363, 362, 383, 341,
                2766, 3671, 5521, 9234,
                17620, 1920
            );
            public static short getGlyph(char code)
            {
                return glyphs[code - offset];
            }
        }

        public class Graphics
        {
            public readonly int width;
            public readonly int height;
            private IMyTextPanel console;
            private string[] screen;
            private string[] screenLines;
            private string[][] fgDither;
            private string[][] bgDither;
            private int[] clip = new int[4];
            private Dictionary<string, string[][]> oldPatterns = new Dictionary<string, string[][]>();
            public Graphics(int w, int h, IMyTextPanel c)
            {
                width = w;
                height = h;
                screen = new string[height * width];
                screenLines = new string[width * height + height - 1];
                console = c;
                console.FontSize = 0.1f;
                console.Font = "Monospace";
                console.ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;
                mask();
                setFG(255, 255, 255);
                setBG(0, 0, 0);
            }
            public void setFG(int r, int g, int b)
            {
                string k = r + ":" + g + ":" + b;
                if (!oldPatterns.TryGetValue(k, out fgDither))
                {
                    fgDither = ColorUtils.genDitherPattern(r, g, b);
                    oldPatterns[k] = fgDither;
                }
            }
            public void setBG(int r, int g, int b)
            {
                string k = r + ":" + g + ":" + b;
                if (!oldPatterns.TryGetValue(k, out bgDither))
                {
                    bgDither = ColorUtils.genDitherPattern(r, g, b);
                    oldPatterns[k] = bgDither;
                }
            }
            public void mask(int x1, int y1, int x2, int y2)
            {
                clip[0] = x1;
                clip[1] = y1;
                clip[2] = x2;
                clip[3] = y2;
            }
            public void mask()
            {
                clip[0] = 0;
                clip[1] = 0;
                clip[2] = width - 1;
                clip[3] = height - 1;
            }
            public void paint()
            {
                for (int i = 0; i < height; i++)
                {
                    screenLines[i] = string.Join(null, screen, i * width, width) + "\n";
                }
                console.WriteText(string.Concat(screenLines));
            }
            public void clear()
            {
                for (int i = 0; i < 8; i++)
                {
                    for (int j = 0; j < width; j += 8)
                    {
                        Array.Copy(bgDither[i], 0, screen, i * width + j, 8);
                    }
                }
                int size = width * height;
                int half = width * height >> 1;
                for (int i = width * 8; i < size; i *= 2)
                {
                    int copyLength = i;
                    if (i > half)
                    {
                        copyLength = size - i;
                    }
                    Array.Copy(screen, 0, screen, i, copyLength);
                }
            }
            public void pixel(int x, int y)
            {
                if (x >= clip[0] && x <= clip[2] && y >= clip[1] && y <= clip[3])
                {
                    screen[width * y + x] = fgDither[y & 7][x & 7];
                }
            }
            public void line(int x0, int y0, int x1, int y1)
            {
                if (x0 == x1)
                {
                    int high = Math.Max(y1, y0);
                    for (int y = Math.Min(y1, y0); y <= high; y++)
                    {
                        pixel(x0, y);
                    }
                }
                else if (y0 == y1)
                {
                    int high = Math.Max(x1, x0);
                    for (int x = Math.Min(x1, x0); x <= high; x++)
                    {
                        pixel(x, y0);
                    }
                }
                else
                {
                    bool yLonger = false;
                    int incrementVal, endVal;
                    int shortLen = y1 - y0;
                    int longLen = x1 - x0;
                    if (Math.Abs(shortLen) > Math.Abs(longLen))
                    {
                        int swap = shortLen;
                        shortLen = longLen;
                        longLen = swap;
                        yLonger = true;
                    }
                    endVal = longLen;
                    if (longLen < 0)
                    {
                        incrementVal = -1;
                        longLen = -longLen;
                    }
                    else incrementVal = 1;
                    int decInc;
                    if (longLen == 0) decInc = 0;
                    else decInc = (shortLen << 16) / longLen;
                    int j = 0;
                    if (yLonger)
                    {
                        for (int i = 0; i - incrementVal != endVal; i += incrementVal)
                        {
                            pixel(x0 + (j >> 16), y0 + i);
                            j += decInc;
                        }
                    }
                    else
                    {
                        for (int i = 0; i - incrementVal != endVal; i += incrementVal)
                        {
                            pixel(x0 + i, y0 + (j >> 16));
                            j += decInc;
                        }
                    }
                }
            }
            private void flatBottom(int x1, int y1, int x2, int y2, int x3, int y3)
            {
                float invslope1 = (float)(x2 - x1) / (y2 - y1);
                float invslope2 = (float)(x3 - x1) / (y3 - y1);
                float curx1 = x1;
                float curx2 = x1;
                for (int scanlineY = y1; scanlineY <= y2; scanlineY++)
                {
                    line((int)curx1, scanlineY, (int)curx2, scanlineY);
                    curx1 += invslope1;
                    curx2 += invslope2;
                }
            }
            private void flatTop(int x1, int y1, int x2, int y2, int x3, int y3)
            {
                float invslope1 = (float)(x3 - x1) / (y3 - y1);
                float invslope2 = (float)(x3 - x2) / (y3 - y2);
                float curx1 = x3;
                float curx2 = x3;
                for (int scanlineY = y3; scanlineY > y1; scanlineY--)
                {
                    curx1 -= invslope1;
                    curx2 -= invslope2;
                    line((int)curx1, scanlineY, (int)curx2, scanlineY);
                }
            }
            private void swap(ref int a, ref int b)
            {
                int c = a;
                a = b;
                b = c;
            }
            public void tri(string m, int x1, int y1, int x2, int y2, int x3, int y3)
            {
                if (m == "line")
                {
                    line(x1, y1, x2, y2);
                    line(x2, y2, x3, y3);
                    line(x3, y3, x1, y1);
                }
                else if (m == "fill")
                {
                    if (y1 > y3)
                    {
                        swap(ref y1, ref y3);
                        swap(ref x1, ref x3);
                    }
                    if (y1 > y2)
                    {
                        swap(ref y1, ref y2);
                        swap(ref x1, ref x2);
                    }
                    if (y2 > y3)
                    {
                        swap(ref y2, ref y3);
                        swap(ref x2, ref x3);
                    }
                    if (y2 == y3)
                    {
                        flatBottom(x1, y1, x2, y2, x3, y3);
                    }
                    else if (y1 == y2)
                    {
                        flatTop(x1, y1, x2, y2, x3, y3);
                    }
                    else
                    {
                        int x4 = (int)(x1 + ((float)(y2 - y1) / (float)(y3 - y1)) * (x3 - x1));
                        flatBottom(x1, y1, x2, y2, x4, y2);
                        flatTop(x2, y2, x4, y2, x3, y3);
                    }
                }
            }
            public void rect(string m, int xb, int yb, int w, int h)
            {
                if (m == "line")
                {
                    line(xb, yb, xb, yb + h - 1);
                    line(xb, yb, xb + w - 1, yb);
                    line(xb + w - 1, yb, xb + w - 1, yb + h - 1);
                    line(xb, yb + h - 1, xb + w - 1, yb + h - 1);
                }
                else if (m == "fill")
                {
                    for (int x = xb; x < xb + w; x++)
                    {
                        for (int y = yb; y < yb + h; y++)
                        {
                            pixel(x, y);
                        }
                    }
                }
            }
            public void ellipse(string m, int cx, int cy, int rx, int ry)
            {
                int rx2 = rx * rx;
                int ry2 = ry * ry;
                if (m == "fill")
                {
                    int rxsys = rx2 * ry2;
                    pixel(cx, cy);
                    for (int i = 1; i < rx * ry; i++)
                    {
                        int x = i % rx;
                        int y = i / rx;
                        if (ry2 * x * x + rx2 * y * y <= rxsys)
                        {
                            pixel(cx + x, cy + y);
                            pixel(cx - x, cy - y);
                            //if (x && y) { //unnecessary (prevents overdrawing pixels)
                            pixel(cx + x, cy - y);
                            pixel(cx - x, cy + y);
                            //}
                        }
                    }
                }
                else if (m == "line")
                {
                    int frx2 = 4 * rx2;
                    int fry2 = 4 * ry2;
                    int s = 2 * ry2 + rx2 * (1 - 2 * ry);
                    int y = ry;
                    for (int x = 0; ry2 * x <= rx2 * y; x++)
                    {
                        pixel(cx + x, cy + y);
                        pixel(cx - x, cy + y);
                        pixel(cx + x, cy - y);
                        pixel(cx - x, cy - y);
                        if (s >= 0)
                        {
                            s += frx2 * (1 - y);
                            y--;
                        }
                        s += ry2 * ((4 * x) + 6);
                    }
                    y = 0;
                    s = 2 * rx2 + ry2 * (1 - 2 * rx);
                    for (int x = rx; rx2 * y <= ry2 * x; y++)
                    {
                        pixel(cx + x, cy + y);
                        pixel(cx - x, cy + y);
                        pixel(cx + x, cy - y);
                        pixel(cx - x, cy - y);
                        if (s >= 0)
                        {
                            s += fry2 * (1 - x);
                            x--;
                        }
                        s += rx2 * ((4 * y) + 6);
                    }
                }
            }
            public void circle(string m, int cx, int cy, int r)
            {
                if (m == "fill")
                {
                    int rr = r * r;
                    pixel(cx, cy);
                    for (int i = 1; i < r * r; i++)
                    {
                        int x = i % r;
                        int y = i / r;
                        if (x * x + y * y < rr)
                        {
                            pixel(cx + x, cy + y);
                            pixel(cx - x, cy - y);
                            if (x > 0 && y > 0)
                            {
                                pixel(cx + x, cy - y);
                                pixel(cx - x, cy + y);
                            }
                        }
                    }
                }
                else if (m == "line")
                {
                    int x = r;
                    int y = 0;
                    int do2 = 1 - x;
                    while (y <= x)
                    {
                        pixel(cx + x, cy + y);
                        pixel(cx + y, cy + x);
                        pixel(cx - x, cy + y);
                        pixel(cx - y, cy + x);
                        pixel(cx - x, cy - y);
                        pixel(cx - y, cy - x);
                        pixel(cx + x, cy - y);
                        pixel(cx + y, cy - x);
                        y++;
                        if (do2 <= 0)
                        {
                            do2 += 2 * y + 1;
                        }
                        else
                        {
                            do2 += 2 * (y - --x) + 1;
                        }
                    }
                }
            }
            public void print(int x, int y, string text)
            {
                int x1 = x;
                int y1 = y;
                for (int i = 0; i < text.Length; i++)
                {
                    switch (text[i])
                    {
                        case '\n':
                            y1 += 6;
                            x1 = x;
                            break;
                        case ' ':
                            x1 += 4;
                            break;
                        default:
                            short glyph = Ascii.getGlyph(text[i]);
                            int j = 14;
                            do
                            {
                                if ((glyph & 1) != 0)
                                {
                                    pixel(x1 + j % 3, y1 - 4 + j / 3);
                                }
                                glyph >>= 1;
                                j--;
                            } while (glyph > 0);
                            x1 += 4;
                            break;
                    }
                }
            }
        }
        public class CubeViewer
        {
            private static int[][] cubeSides = new int[][] {
           SEFix.arr(
            1, 1,-1,
            1, 1, 1,
           -1, 1, 1,
           -1, 1,-1
        ), SEFix.arr(
           -1, 1,-1,
           -1, 1, 1,
           -1,-1, 1,
           -1,-1,-1
        ), SEFix.arr(
            1,-1, 1,
            1,-1,-1,
           -1,-1,-1,
           -1,-1, 1
        ), SEFix.arr(
            1,-1, 1,
            1, 1, 1,
            1, 1,-1,
            1,-1,-1
        ),
        SEFix.arr(
            1,-1, 1,
            1, 1, 1,
           -1, 1, 1,
           -1,-1, 1
        ),
        SEFix.arr(
            1,-1,-1,
            1, 1,-1,
           -1, 1,-1,
           -1,-1,-1
        )
    };
            private static int[][] colors = new int[][] {
        SEFix.arr(60,60,60),
        SEFix.arr(150,150,150),
        SEFix.arr(190,190,190),
        SEFix.arr(150,150,150),
        SEFix.arr(40,40,40),
        SEFix.arr(255,255,255)
    };
            private int scale;
            private double angle1_ = 0;
            private double angle2_;
            private double cosA = 1;
            private double sinA = 0;
            private double cosB;
            private double sinB;
            private double angle1
            {
                get { return angle1_; }
                set
                {
                    angle1_ = value + Math.Ceiling(-value / (2 * Math.PI)) * 2 * Math.PI;
                    cosA = Math.Cos(value);
                    sinA = Math.Sin(value);
                }
            }
            private double angle2
            {
                get { return angle2_; }
                set
                {
                    angle2_ = value + Math.Ceiling(-value / (2 * Math.PI)) * 2 * Math.PI;
                    cosB = Math.Cos(value);
                    sinB = Math.Sin(value);
                }
            }
            private int[][] sides = new int[][] { new int[12], new int[12], new int[12] };
            public CubeViewer(double b)
            {
                angle2 = b;
            }
            private void rotZY(ref double x1, ref double y1, ref double z1)
            {
                double x = x1, y = y1, z = z1;
                x1 = x * cosA + y * sinA;
                y1 = cosB * (y * cosA - x * sinA) + z * sinB;
                z1 = z * cosB - (y * cosA - x * sinA) * sinB;
            }
            private void pickSides()
            {
                double direction = angle1 / Math.PI * 2;
                sides[0] = cubeSides[(int)direction % 4];
                sides[1] = cubeSides[((int)direction + 1) % 4];
                sides[2] = cubeSides[4 + (int)(angle2 / Math.PI)];
            }
            private void cube3d(int x, int y, int z, int[][] polys)
            {
                for (int i = 0; i < 3; i++)
                {
                    for (int c = 0; c < 10; c += 3)
                    {
                        double x1 = x + sides[i][c] * 0.5, y1 = y + sides[i][c + 1] * 0.5, z1 = z + sides[i][c + 2] * 0.5;
                        rotZY(ref x1, ref y1, ref z1);

                        polys[i][8] = (int)(((c == 0 ? y1 * 100 : polys[i][8]) + y1 * 100) / 2);
                        polys[i][9] = Array.IndexOf(cubeSides, sides[i]);
                        polys[i][c / 3 * 2] = (int)(scale * x1);
                        polys[i][c / 3 * 2 + 1] = (int)(scale * z1);
                    }
                }
            }
            public IEnumerable<double> draw(bool vectorMode, int x, int y, int[][] blocks, double inc, int s, Graphics g)
            {
                scale = s;
                angle1 += inc;
                pickSides();
                int[][] allPolys = new int[blocks.Length * 3][];
                for (int b = 0; b < blocks.Length; b++)
                {
                    allPolys[b * 3] = new int[10]; allPolys[b * 3 + 1] = new int[10]; allPolys[b * 3 + 2] = new int[10];
                    cube3d(blocks[b][0], blocks[b][1], blocks[b][2], new int[][] { allPolys[b * 3], allPolys[b * 3 + 1], allPolys[b * 3 + 2] });
                    yield return 0;
                }
                yield return 0.01;
                Array.Sort(allPolys, delegate (int[] a, int[] b)
                {
                    return a[8].CompareTo(b[8]);
                });
                for (int i = 0; i < allPolys.Length; i++)
                {
                    yield return 0;
                    if (!vectorMode)
                    {
                        g.setFG(colors[allPolys[i][9]][0], colors[allPolys[i][9]][1], colors[allPolys[i][9]][2]);
                    }
                    else
                    {
                        g.setFG(0, 0, 0);
                    }
                    g.tri("fill", x + allPolys[i][0], y + allPolys[i][1],
                        x + allPolys[i][2], y + allPolys[i][3], x + allPolys[i][6], y + allPolys[i][7]);
                    g.tri("fill", x + allPolys[i][6], y + allPolys[i][7],
                        x + allPolys[i][2], y + allPolys[i][3], x + allPolys[i][4], y + allPolys[i][5]);
                    if (vectorMode)
                    {
                        g.setFG(0, 255, 0);
                        g.line(x + allPolys[i][0], y + allPolys[i][1], x + allPolys[i][2], y + allPolys[i][3]);
                        g.line(x + allPolys[i][2], y + allPolys[i][3], x + allPolys[i][4], y + allPolys[i][5]);
                        g.line(x + allPolys[i][4], y + allPolys[i][5], x + allPolys[i][6], y + allPolys[i][7]);
                        g.line(x + allPolys[i][6], y + allPolys[i][7], x + allPolys[i][0], y + allPolys[i][1]);
                    }
                }
            }
        }
    }
}
