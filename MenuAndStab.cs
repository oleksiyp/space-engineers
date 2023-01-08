#if DEBUG
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using VRage.Game.ModAPI.Ingame;
using VRageMath;
using static System.Net.Mime.MediaTypeNames;
using static VRage.MyMiniDump;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
#endif
        public static Program ProgramInstance;
        public static TimerRoutineRunner RoutineRunner;
        public static TimeSource ProgramTimeSource;
        public static Random ProgramRandom;
        public static bool FirstIteration = true;

        public Program()
        {
            ProgramInstance = this;
            ProgramRandom = new Random();
            ProgramTimeSource = new TimeSource();
            RoutineRunner = new TimerRoutineRunner();
            Runtime.UpdateFrequency = UpdateFrequency.Once;
        }

        public struct MenuOption
        {
            public readonly String Name;
            public readonly String Command;
            public readonly Menu SubMenu;

            public MenuOption(String command, String name, Menu subMenu)
            {
                this.Name = name;
                this.Command = command;
                this.SubMenu = subMenu;
            }
        }

        public const int MENU_ITEMS_ON_SCREEN = 5;
        public const int MENU_CHARS = 20;
        public const int MENU_WIDTH = 4 * MENU_CHARS;

        public class Menu
        {
            private int current;
            private int scroll;
            private List<MenuOption> options = new List<MenuOption>();

            public Menu option(String command, String text)
            {
                options.Add(new MenuOption(command, text, null));
                return this;
            }

            public Menu submenu(String command, String text, Menu menu)
            {
                options.Add(new MenuOption(command, text, menu));
                return this;
            }

            internal MenuOption Selected()
            {
                if (options.Count == 0)
                {
                    throw new Exception("Menu without options");
                }
                if (current < 0)
                {
                    throw new Exception("Bad value of current menu pointer");
                }
                if (current >= options.Count)
                {
                    throw new Exception("Bad value of current menu pointer (too big)");
                }
                return options[current];
            }

            internal void Prev()
            {
                current = Math.Max(current - 1, 0);
                scroll = Math.Min(scroll, current);
            }

            internal void Next()
            {
                current = Math.Max(Math.Min(current + 1, options.Count - 1), 0);
                scroll = Math.Max(scroll, current - MENU_ITEMS_ON_SCREEN + 1);
            }

            public int Scroll
            {
                get { return scroll; }
            }

            public int Current
            {
                get { return current; }
            }

            public List<MenuOption> Options
            {
                get { return options; }
            }
        }

        public class SpaceShipProgram : ScriptKind
        {
            private Graphics g;
            private IMyTextPanel lcd;
            private WaitCondition commandWait;
            private Stack<Menu> menuStack = new Stack<Menu>();
            private String hstabRoutine;

            private Menu mainMenu = 
                new Menu()
                .submenu("%hstab", "HSTAB",
                    new Menu()
                      .option("%hstabon", "ON")
                      .option("%hstaboff", "OFF")
                );

            public SpaceShipProgram()
            {
            }

            public IEnumerable<WaitCondition> Start(string arguments)
            {
                lcd = ProgramInstance.GetBlockOfType(lcd, (p) => p.DisplayNameText == "cockpit LCD");
                if (g == null) g = new Graphics(165, 165, lcd);
                if (menuStack.Count == 0)
                {
                    menuStack.Push(mainMenu);
                }
                g.setBG(0, 0, 0);
                g.clear();
                DrawMenu();
                DrawWindowBorders();
                DrawStatus();
                g.paint();
                RoutineRunner.Start(CmdDispatch());
                yield break;
            }

            private void DrawStatus()
            {
                int x = g.width / 2 + 1;
                int w = g.width - x;
                int y = 0;
                int h = (MENU_ITEMS_ON_SCREEN + 2) * 7;

                g.rect("fill", x, y, w, h);
                g.print(x, y, "abc");
            }

            public void DrawWindowBorders()
            {
                g.setFG(180, 0, 0);
                g.line(0, (MENU_ITEMS_ON_SCREEN + 2) * 7, g.width, (MENU_ITEMS_ON_SCREEN + 2) * 7);
                g.line(g.width / 2, 0, g.width / 2, (MENU_ITEMS_ON_SCREEN + 2) * 7);
            }

            public void DrawMenu()
            {
                int gw = g.width / 2;
                g.setFG(0, 0, 0);
                g.rect("fill", 0, 0, gw, 6 * (MENU_ITEMS_ON_SCREEN + 2));
                int s = (gw - MENU_WIDTH) / 2;
                int y = 10;

                Menu menu = menuStack.Peek();
                if (menu.Scroll > 0)
                {
                    g.setFG(0, 0, 0);
                    g.rect("fill", s, y + 1 - 6, MENU_WIDTH, 7);
                    g.setFG(255, 0, 0);
                    int off = (MENU_WIDTH - 12) / 2;
                    g.print(s + off, y, "...");
                }

                y += 7;
                

                bool etcAfterLast = true;
                for (int i = 0; i < MENU_ITEMS_ON_SCREEN; i++)
                {
                    int pos = menu.Scroll + i;
                    if (pos == menu.Options.Count - 1)
                    {
                        etcAfterLast = false;
                    }

                    if (pos < 0 || pos >= menu.Options.Count)
                    {
                        continue;
                    }

                    MenuOption option = menu.Options[pos];

                    String text = option.Name;

                    int maxw = MENU_CHARS;
                    if (option.SubMenu != null) {
                        int ww = MENU_CHARS - 3;
                        maxw = Math.Min(ww, maxw);
                    }

                    if (text.Length > maxw)
                    {
                        maxw -= 3;
                        text = text.Substring(0, maxw) + "...";
                    }

                    if (option.SubMenu != null)
                    {
                        text += " >";
                    }

                    if (pos == menu.Current)
                    {
                        g.setFG(255, 0, 0);
                    }
                    else
                    {
                        g.setFG(0, 0, 0);
                    }
                    g.rect("fill", s, y + 1 - 6, MENU_WIDTH, 7);
                    if (pos == menu.Current)
                    {
                        g.setFG(0, 0, 0);
                    }
                    else
                    {
                        g.setFG(255, 0, 0);
                    }
                    int off = (MENU_WIDTH - 4 * text.Length) / 2;
                    g.print(s + off, y, text);
                    
                    y += 7;
                }
                if (etcAfterLast)
                {
                    int off = (MENU_WIDTH - 12) / 2;
                    g.print(s + off, y, "...");
                }
            }

            void StopHstab()
            {
                if (hstabRoutine != null)
                {
                    RoutineRunner.Stop(hstabRoutine);
                }
                hstabRoutine = null;
            }

            public IEnumerable<WaitCondition> CmdDispatch()
            {
                commandWait = CommandsReceived("%up", "%down", "%exit", "%enter", "%hstabon", "%hstaboff")
                    .WithStatus("cmd in");

                yield return commandWait;

                if (commandWait.IsCommandTriggered("%up"))
                {
                    Menu menu = menuStack.Peek();
                    menu.Prev();
                    DrawMenu();
                    g.paint();
                }
                else if (commandWait.IsCommandTriggered("%down"))
                {
                    Menu menu = menuStack.Peek();
                    menu.Next();
                    DrawMenu();
                    g.paint();
                }
                else if (commandWait.IsCommandTriggered("%enter"))
                {
                    Menu menu = menuStack.Peek();
                    MenuOption selected = menu.Selected();
                    if (selected.SubMenu != null)
                    {
                        menuStack.Push((Menu)selected.SubMenu);
                        DrawMenu();
                        g.paint();
                    }
                    else
                    {
                        RoutineRunner.Run(selected.Command, UpdateType.Terminal);
                    }
                }
                else if (commandWait.IsCommandTriggered("%exit"))
                {
                    if (menuStack.Count >= 2)
                    {
                        menuStack.Pop();
                        DrawMenu();
                        g.paint();
                    }
                }
                else if (commandWait.IsCommandTriggered("%hstabon"))
                {
                    StopHstab();
                    Hstabber hstabber = new Hstabber();
                    hstabRoutine = RoutineRunner.Start(hstabber.Run());
                }
                else if (commandWait.IsCommandTriggered("%hstaboff"))
                {
                    StopHstab();
                }
            }

            public class Hstabber
            {

                const string gyroExcludeName = "Exclude";
                const string statusScreenName = "Alignment"; //(Optional) Name of status screen
                const string shipName = "\n         [SHIP NAME GOES HERE]"; //(Optional) Name of your ship

                bool shouldAlign = true; //If the script should attempt to stabalize by default
                bool referenceOnSameGridAsProgram = true; //if true, only searches for reference blocks on
                                                          //the same grid as the program block (should help with docking small vessels)

                const double angleTolerance = 0; //How many degrees the code will allow before it overrides user control

                //---PID Constants
                const double proportionalConstant = 2;
                const double derivativeConstant = .5;

                const double yawSpeedModifier = 1;

                /*  
                ====================================================  
                    Don't touch anything below this <3 - Whiplash  
                ====================================================  
                */

                const double updatesPerSecond = 10;
                const double timeLimit = 1 / updatesPerSecond;
                double angleRoll = 0;
                double anglePitch = 0;
                bool canTolerate = true;
                string stableStatus = ">> Disabled <<";
                string gravityMagnitudeString;
                string overrideStatus;

                List<IMyGyro> gyros = new List<IMyGyro>();
                List<IMyShipController> shipControllers = new List<IMyShipController>();

                PID pitchPID;
                PID rollPID;

                public Hstabber()
                {
                    pitchPID = new PID(proportionalConstant, 0, derivativeConstant, -10, 10, timeLimit);
                    rollPID = new PID(proportionalConstant, 0, derivativeConstant, -10, 10, timeLimit);
                }

                bool ShouldFetch(IMyTerminalBlock block)
                {
                    if (block is IMyShipController)
                    {
                        if (referenceOnSameGridAsProgram)
                        {
                            return block.CubeGrid == ProgramInstance.Me.CubeGrid;
                        }
                        else
                        {
                            return true;
                        }
                    }
                    else
                    {
                        return false;
                    }
                }

                IMyShipController GetControlledShipController(List<IMyShipController> controllers)
                {
                    if (controllers.Count == 0)
                        return null;

                    foreach (IMyShipController thisController in controllers)
                    {
                        if (thisController.IsUnderControl && thisController.CanControlShip)
                            return thisController;
                    }

                    return controllers[0];
                }

                void AlignWithGravity()
                {
                    //---Find our refrence and comparision blocks    
                    ProgramInstance.GridTerminalSystem.GetBlocksOfType(shipControllers, ShouldFetch);

                    //---Check for any cases that would lead to code failure
                    if (shipControllers.Count == 0)
                    {
                        ProgramInstance.Echo($"ERROR: No ship controller was found");
                        return;
                    }

                    //---Assign our reference block
                    IMyShipController referenceBlock = GetControlledShipController(shipControllers);

                    //---Populate gyro list
                    gyros.Clear();
                    ProgramInstance.GridTerminalSystem.GetBlocksOfType(gyros, block => block.CubeGrid == referenceBlock.CubeGrid && !block.CustomName.Contains(gyroExcludeName));

                    if (gyros.Count == 0)
                    {
                        ProgramInstance.Echo("ERROR: No gyros found on ship");
                        return;
                    }

                    //---Get gravity vector    
                    var referenceOrigin = referenceBlock.GetPosition();
                    var gravityVec = referenceBlock.GetNaturalGravity();
                    var gravityVecLength = gravityVec.Length();
                    gravityMagnitudeString = Math.Round(gravityVecLength, 2).ToString() + " m/s²";
                    if (gravityVec.LengthSquared() == 0)
                    {
                        gravityMagnitudeString = "No Gravity";

                        foreach (IMyGyro thisGyro in gyros)
                        {
                            thisGyro.SetValue("Override", false);
                        }
                        overrideStatus = "";
                        stableStatus = ">> Disabled <<";

                        shouldAlign = false;

                        angleRoll = 0; angleRoll = 0;
                        return;
                    }

                    //---Dir'n vectors of the reference block     
                    var referenceForward = referenceBlock.WorldMatrix.Forward;
                    var referenceLeft = referenceBlock.WorldMatrix.Left;
                    var referenceUp = referenceBlock.WorldMatrix.Up;

                    //---Get Roll and Pitch Angles 
                    anglePitch = Math.Acos(MathHelper.Clamp(gravityVec.Dot(referenceForward) / gravityVecLength, -1, 1)) - Math.PI / 2;

                    Vector3D planetRelativeLeftVec = referenceForward.Cross(gravityVec);
                    angleRoll = VectorAngleBetween(referenceLeft, planetRelativeLeftVec);
                    angleRoll *= VectorCompareDirection(VectorProjection(referenceLeft, gravityVec), gravityVec); //ccw is positive 

                    anglePitch *= -1; angleRoll *= -1;


                    //---Get Raw Deviation angle    
                    double rawDevAngle = Math.Acos(MathHelper.Clamp(gravityVec.Dot(referenceForward) / gravityVec.Length() * 180 / Math.PI, -1, 1));

                    //---Angle controller    
                    double rollSpeed = rollPID.Control(angleRoll); //Math.Round(angleRoll * proportionalConstant + (angleRoll - lastAngleRoll) / timeElapsed * derivativeConstant, 2);
                    double pitchSpeed = pitchPID.Control(anglePitch); //Math.Round(anglePitch * proportionalConstant + (anglePitch - lastAnglePitch) / timeElapsed * derivativeConstant, 2);                                                                                                                                                            //w.H]i\p

                    //var mouseInput = referenceBlock.RotationIndicator;
                    var inputVec = referenceBlock.MoveIndicator;

                    //rollSpeed = rollSpeed / gyros.Count;
                    //pitchSpeed = pitchSpeed / gyros.Count;

                    //---Check if we are inside our tolerances  
                    canTolerate = true;

                    if (Math.Abs(anglePitch * 180 / Math.PI) > angleTolerance)
                    {
                        canTolerate = false;
                    }

                    if (Math.Abs(angleRoll * 180 / Math.PI) > angleTolerance)
                    {
                        canTolerate = false;
                    }

                    //---Set appropriate gyro override  
                    if (shouldAlign && !canTolerate)
                    {
                        //do gyros
                        ApplyGyroOverride(pitchSpeed, inputVec.X * yawSpeedModifier, -rollSpeed, gyros, referenceBlock);

                        overrideStatus = $"\n\n           SAFETY OVERRIDE ACTIVE"; //\nYaw : {yawSpeed}";
                    }
                    else
                    {
                        foreach (IMyGyro thisGyro in gyros)
                        {
                            thisGyro.SetValue("Override", false);
                        }
                        overrideStatus = "";
                    }
                }

                void StatusScreens()
                {
                    //---get the parts of our string  
                    double roll_deg = angleRoll / Math.PI * 180;
                    double pitch_deg = -anglePitch / Math.PI * 180;
                    string rollStatusString = AngleStatus(roll_deg);
                    string pitchStatusString = AngleStatus(pitch_deg);

                    //---Construct our final string  
                    string statusScreenMessage = shipName
                        + "\n            Natural Gravity: " + gravityMagnitudeString
                        + "\n            Stabilizer: " + stableStatus
                        + "\n\n            Roll Angle: " + Math.Round(roll_deg, 2).ToString() + " degrees\n           " + rollStatusString
                        + "\n\n            Pitch Angle: " + Math.Round(pitch_deg, 2).ToString() + " degrees\n           " + pitchStatusString
                        + overrideStatus;


                    //---Write to screens  
                    var screens = new List<IMyTerminalBlock>();
                    //GridTerminalSystem.SearchBlocksOfName(statusScreenName, screens, block => block is IMyTextPanel);

                    if (screens.Count == 0)
                        return;

                    foreach (IMyTextPanel thisScreen in screens)
                    {
                        thisScreen.WritePublicText(statusScreenMessage);
                        thisScreen.ShowTextureOnScreen();
                        thisScreen.ShowPublicTextOnScreen();
                    }
                }

                const string align_15 = " [-15](-)-------0----------[+15]";
                const string align_14 = " [-15]-(-)------0----------[+15]";
                const string align_12 = " [-15]--(-)-----0----------[+15]";
                const string align_10 = " [-15]---(-)----0----------[+15]";
                const string align_8 = " [-15]----(-)---0----------[+15]";
                const string align_6 = " [-15]-----(-)--0----------[+15]";
                const string align_4 = " [-15]------(-)-0----------[+15]";
                const string align_2 = " [-15]-------(-)0----------[+15]";
                const string align0 = " [-15]---------(0)---------[+15]";
                const string align2 = " [-15]----------0(-)-------[+15]";
                const string align4 = " [-15]----------0-(-)------[+15]";
                const string align6 = " [-15]----------0--(-)-----[+15]";
                const string align8 = " [-15]----------0---(-)----[+15]";
                const string align10 = " [-15]----------0----(-)---[+15]";
                const string align12 = " [-15]----------0-----(-)--[+15]";
                const string align14 = " [-15]----------0------(-)-[+15]";
                const string align15 = " [-15]----------0-------(-)[+15]";

                string AngleStatus(double angle)
                {
                    if (angle > 15)
                        return align15;
                    else if (angle > 14)
                        return align14;
                    else if (angle > 12)
                        return align12;
                    else if (angle > 10)
                        return align10;
                    else if (angle > 8)
                        return align8;
                    else if (angle > 6)
                        return align6;
                    else if (angle > 4)
                        return align4;
                    else if (angle > 2)
                        return align2;
                    else if (angle > -2)
                        return align0;
                    else if (angle > -4)
                        return align_2;
                    else if (angle > -6)
                        return align_4;
                    else if (angle > -8)
                        return align_6;
                    else if (angle > -10)
                        return align_8;
                    else if (angle > -12)
                        return align_10;
                    else if (angle > -14)
                        return align_12;
                    else if (angle > -15)
                        return align_14;
                    else
                        return align_15;
                }

                public IEnumerable<WaitCondition> Run()
                {
                    AlignWithGravity();
                    yield return new WaitCondition()
                        .WithDelay(100);
                }

                Vector3D VectorProjection(Vector3D a, Vector3D b) //proj a on b    
                {
                    Vector3D projection = a.Dot(b) / b.LengthSquared() * b;
                    return projection;
                }

                int VectorCompareDirection(Vector3D a, Vector3D b) //returns -1 if vectors return negative dot product 
                {
                    double check = a.Dot(b);
                    if (check < 0)
                        return -1;
                    else
                        return 1;
                }

                double VectorAngleBetween(Vector3D a, Vector3D b) //returns radians 
                {
                    if (a.LengthSquared() == 0 || b.LengthSquared() == 0)
                        return 0;
                    else
                        return Math.Acos(MathHelper.Clamp(a.Dot(b) / a.Length() / b.Length(), -1, 1));
                }

                //Whip's ApplyGyroOverride Method v9 - 8/19/17
                void ApplyGyroOverride(double pitch_speed, double yaw_speed, double roll_speed, List<IMyGyro> gyro_list, IMyTerminalBlock reference)
                {
                    var rotationVec = new Vector3D(-pitch_speed, yaw_speed, roll_speed); //because keen does some weird stuff with signs 
                    var shipMatrix = reference.WorldMatrix;
                    var relativeRotationVec = Vector3D.TransformNormal(rotationVec, shipMatrix);

                    foreach (var thisGyro in gyro_list)
                    {
                        var gyroMatrix = thisGyro.WorldMatrix;
                        var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(gyroMatrix));

                        thisGyro.Pitch = (float)transformedRotationVec.X;
                        thisGyro.Yaw = (float)transformedRotationVec.Y;
                        thisGyro.Roll = (float)transformedRotationVec.Z;
                        thisGyro.GyroOverride = true;
                    }
                }

                //Whip's PID controller class v6 - 11/22/17
                public class PID
                {
                    double _kP = 0;
                    double _kI = 0;
                    double _kD = 0;
                    double _integralDecayRatio = 0;
                    double _lowerBound = 0;
                    double _upperBound = 0;
                    double _timeStep = 0;
                    double _inverseTimeStep = 0;
                    double _errorSum = 0;
                    double _lastError = 0;
                    bool _firstRun = true;
                    bool _integralDecay = false;
                    public double Value { get; private set; }

                    public PID(double kP, double kI, double kD, double lowerBound, double upperBound, double timeStep)
                    {
                        _kP = kP;
                        _kI = kI;
                        _kD = kD;
                        _lowerBound = lowerBound;
                        _upperBound = upperBound;
                        _timeStep = timeStep;
                        _inverseTimeStep = 1 / _timeStep;
                        _integralDecay = false;
                    }

                    public PID(double kP, double kI, double kD, double integralDecayRatio, double timeStep)
                    {
                        _kP = kP;
                        _kI = kI;
                        _kD = kD;
                        _timeStep = timeStep;
                        _inverseTimeStep = 1 / _timeStep;
                        _integralDecayRatio = integralDecayRatio;
                        _integralDecay = true;
                    }

                    public double Control(double error)
                    {
                        //Compute derivative term
                        var errorDerivative = (error - _lastError) * _inverseTimeStep;

                        if (_firstRun)
                        {
                            errorDerivative = 0;
                            _firstRun = false;
                        }

                        //Compute integral term
                        if (!_integralDecay)
                        {
                            _errorSum += error * _timeStep;

                            //Clamp integral term
                            if (_errorSum > _upperBound)
                                _errorSum = _upperBound;
                            else if (_errorSum < _lowerBound)
                                _errorSum = _lowerBound;
                        }
                        else
                        {
                            _errorSum = _errorSum * (1.0 - _integralDecayRatio) + error * _timeStep;
                        }

                        //Store this error as last error
                        _lastError = error;

                        //Construct output
                        this.Value = _kP * error + _kI * _errorSum + _kD * errorDerivative;
                        return this.Value;
                    }

                    public double Control(double error, double timeStep)
                    {
                        _timeStep = timeStep;
                        _inverseTimeStep = 1 / _timeStep;
                        return Control(error);
                    }

                    public void Reset()
                    {
                        _errorSum = 0;
                        _lastError = 0;
                        _firstRun = true;
                    }
                }

            }

        }

        public class ScriptKinds
        {
            internal Dictionary<String, ScriptKind> kinds = new Dictionary<string, ScriptKind>();

            public void Register(ScriptKind kind)
            {
                kinds[kind.GetType().Name] = kind;
            }
        }

        public interface ScriptKind
        {
            IEnumerable<WaitCondition> Start(String arguments);
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (FirstIteration)
            {
                string[] arr = Me.CustomData.Split(":".ToCharArray(), 2);
                ScriptKind kind = new SpaceShipProgram();
                string arg = arr.Length >= 2 ? arr[1] : "";
                RoutineRunner.Start("Main", kind.Start(arg), 0, false);
                FirstIteration = false;
            }
            ProgramTimeSource.Progress();
            RoutineRunner.Run(argument, updateSource);
        }

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
        public IMyBroadcastListener RegisterBroadcastListener(IMyBroadcastListener listener, String tag, String callback = "!!SAME_AS_TAG!!")
        {
            if (listener == null)
            {
                listener = IGC.RegisterBroadcastListener(tag);
                if (callback == "!!SAME_AS_TAG!!")
                {
                    callback = tag;
                }
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
            private int counter = 0;

            public bool isRunning(String name)
            {
                if (machines.ContainsKey(name))
                {
                    return machines[name].Running;
                }
                return false;
            }

            public void RefreshUpdateFrequency(bool once)
            {
                UpdateFrequency freq = ProgramInstance.Runtime.UpdateFrequency
                    & ~(UpdateFrequency.Update1 | UpdateFrequency.Update10 | UpdateFrequency.Update100);

                foreach (var tsm in machines.Values)
                {
                    if (tsm.Running)
                    {
                        if (tsm.IterationSpeed >= 100)
                        {
                            freq |= UpdateFrequency.Update100;
                        }
                        else if (tsm.IterationSpeed >= 10)
                        {
                            freq |= UpdateFrequency.Update10;
                        }
                        else if (tsm.IterationSpeed >= 1)
                        {
                            freq |= UpdateFrequency.Update1;
                        }
                    }
                }

                if (once)
                {
                    freq |= UpdateFrequency.Once;
                }

                ProgramInstance.Runtime.UpdateFrequency = freq;
            }

            public void Start(String name, IEnumerable<WaitCondition> routine,
                int iterationSpeed,
                bool repeat,
                bool graceful = true
                )
            {
                if (machines.ContainsKey(name))
                {
                    TimerStateMachine tsm = machines[name];
                    tsm.IterationSpeed = iterationSpeed;
                    tsm.Repeat = repeat;
                    tsm.Restart(routine, graceful);
                }
                else
                {

                    TimerStateMachine tsm = new TimerStateMachine(routine, iterationSpeed, repeat);
                    tsm.Start();
                    machines.Add(name, tsm);
                }
                RefreshUpdateFrequency(true);
            }

            public bool Toggle(String name, IEnumerable<WaitCondition> routine, int iterationSpeed, bool repeat)
            {
                if (isRunning(name))
                {
                    Stop(name);
                    return true;
                }
                else
                {
                    TimerStateMachine tsm = new TimerStateMachine(routine, iterationSpeed, repeat);
                    tsm.Start();
                    machines.Remove(name);
                    machines.Add(name, tsm);
                    RefreshUpdateFrequency(true);
                    return false;
                }
            }

            public String Start(IEnumerable<WaitCondition> routine, int iterationSpeed = 100, bool repeat = true)
            {
                String id = "###id" + (counter++).ToString();
                Start(id, routine, iterationSpeed, repeat);
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

            public void Run(string argument, UpdateType updateSource)
            {
                List<String> toRemove = new List<String>();
                List<String> keys = new List<string>(machines.Keys);
                foreach (var key in keys)
                {
                    TimerStateMachine m = machines[key];
                    ProgramInstance.Echo(m.GetStatus());
                    if (m.ShouldRun(updateSource))
                    {
                        m.Run(argument, updateSource);
                    }
                    if (!m.Running)
                    {
                        toRemove.Add(key);
                    }
                }
                foreach (var key in toRemove)
                {
                    machines.Remove(key);
                }
                if (toRemove.Count > 0)
                {
                    RefreshUpdateFrequency(false);
                }
            }

        }

        public class WaitCondition
        {
            public bool exitOnce;
            public bool exitRepeat;
            public double delay = Double.MaxValue;
            public HashSet<string> messageTags = new HashSet<string>();
            public HashSet<string> commands = new HashSet<string>();
            public bool yieldTick;
            public bool yieldTickIfLimit;
            public Action<ExitFlag> exitFlagProvider = null;

            private double timePassed;
            private bool tickYielded;
            private HashSet<string> triggeredMessageTags = new HashSet<string>();
            private HashSet<string> triggeredCommands = new HashSet<string>();
            private double lastTick = 0.0;
            private string status;

            private Dictionary<String, IMyBroadcastListener> broadcastListeners = new Dictionary<string, IMyBroadcastListener>();

            internal void Enter(ExitFlag exitFlag)
            {
                lastTick = ProgramTimeSource.Time;

                timePassed = 0;
                tickYielded = false;
                triggeredMessageTags.Clear();
                triggeredCommands.Clear();

                foreach (var tag in messageTags)
                {
                    if (!broadcastListeners.ContainsKey(tag))
                    {
                        broadcastListeners[tag] = ProgramInstance.RegisterBroadcastListener(null, tag);
                    }

                    if (broadcastListeners[tag].HasPendingMessage)
                    {
                        triggeredMessageTags.Add(tag);
                    }
                }

                if (exitFlagProvider != null)
                {
                    exitFlagProvider.Invoke(exitFlag);
                }
            }

            internal void Progress(string argument, UpdateType updateSource)
            {
                double now = ProgramTimeSource.Time;
                if (lastTick > kEpsilon)
                {
                    timePassed += (now - lastTick);
                }
                lastTick = now;

                if (updateSource.HasFlag(UpdateType.IGC))
                {
                    triggeredMessageTags.Add(argument);
                }

                if (
                    updateSource.HasFlag(UpdateType.Terminal) ||
                    updateSource.HasFlag(UpdateType.Script) ||
                    updateSource.HasFlag(UpdateType.Trigger))
                {
                    triggeredCommands.Add(argument);
                }

                tickYielded = true;
            }

            internal bool HasPendingMessages()
            {
                foreach (var tag in messageTags)
                {
                    if (triggeredMessageTags.Contains(tag))
                    {
                        return true;
                    }
                }

                return false;
            }

            internal bool HasPendingCommands()
            {
                foreach (var tag in commands)
                {
                    if (triggeredCommands.Contains(tag))
                    {
                        return true;
                    }
                }

                return false;
            }

            internal bool ShouldInterrupt()
            {
                if (yieldTick && tickYielded)
                {
                    return true;
                }

                if (timePassed >= delay)
                {
                    return true;
                }

                if (HasPendingMessages())
                {
                    return true;
                }

                if (HasPendingCommands())
                {
                    return true;
                }

                return false;
            }

            public string GetStatus()
            {
                return status;
            }

            public WaitCondition WithDelay(double value)
            {
                delay = value;
                return this;
            }

            public WaitCondition WithYieldTick()
            {
                yieldTick = true;
                return this;
            }

            public WaitCondition WithYieldTickIfLimit()
            {
                yieldTickIfLimit = true;
                return this;
            }

            public WaitCondition WithExitOnce()
            {
                exitOnce = true;
                return this;
            }

            public WaitCondition WithExitRepeat(double value)
            {
                exitRepeat = true;
                return this;
            }

            public WaitCondition WithMessagesReceived(params string[] tags)
            {
                messageTags.UnionWith(tags.ToList());
                return this;
            }

            public WaitCondition WithCommandsReceived(params string[] cmds)
            {
                commands.UnionWith(cmds.ToList());
                return this;
            }


            public WaitCondition WithExitFlagProvider(Action<ExitFlag> provider)
            {
                exitFlagProvider = provider;
                return this;
            }

            public WaitCondition WithStatus(string status)
            {
                this.status = status;
                return this;
            }

            public bool IsMessageTriggered(string tag)
            {
                return triggeredMessageTags.Contains(tag);
            }

            public IEnumerable<MyIGCMessage> GetMessages(String tag)
            {
                if (broadcastListeners.ContainsKey(tag))
                {
                    IMyBroadcastListener listener = broadcastListeners[tag];
                    while (listener.HasPendingMessage)
                    {
                        yield return listener.AcceptMessage();
                    }
                }
            }


            public bool IsCommandTriggered(string cmd)
            {
                return triggeredCommands.Contains(cmd);
            }



            public override String ToString()
            {
                List<String> waitConditions = new List<string>();
                List<String> triggers = new List<string>();

                if (exitOnce)
                {
                    waitConditions.Add("eo");
                }

                if (exitRepeat)
                {
                    waitConditions.Add("er");
                }

                if (yieldTick)
                {
                    waitConditions.Add("yt");
                }

                if (yieldTickIfLimit)
                {
                    waitConditions.Add("ytil");
                }

                if (delay != double.MaxValue)
                {
                    waitConditions.Add(String.Format("d={0:F1}", delay));
                }

                if (commands.Count > 0)
                {
                    waitConditions.Add("c=<" + String.Join("|", commands) + ">");
                }

                if (messageTags.Count > 0)
                {
                    waitConditions.Add("m=<" + String.Join("|", messageTags) + ">");
                }

                if (timePassed > 0)
                {
                    triggers.Add("tp=" + timePassed);
                }

                if (tickYielded)
                {
                    triggers.Add("ty");
                }

                if (triggeredCommands.Count > 0)
                {
                    triggers.Add("c=<" + String.Join("|", triggeredCommands) + ">");
                }

                if (triggeredMessageTags.Count > 0)
                {
                    triggers.Add("m=<" + String.Join("|", triggeredMessageTags) + ">");
                }


                return "WaitFor(" + String.Join(",", waitConditions) + ";" + String.Join(",", triggers) + ")";
            }
        }

        public static WaitCondition Delay(double value)
        {
            return new WaitCondition
            {
                delay = value
            };
        }

        public static WaitCondition YieldTick()
        {
            return new WaitCondition
            {
                yieldTick = true
            };
        }

        public static WaitCondition YieldTickIfLimit()
        {
            return new WaitCondition
            {
                yieldTickIfLimit = true
            };
        }

        public static WaitCondition ExitOnce()
        {
            return new WaitCondition
            {
                exitOnce = true
            };
        }

        public static WaitCondition ExitRepeat(double value)
        {
            return new WaitCondition
            {
                exitRepeat = true
            };
        }

        public static WaitCondition MessagesReceived(params string[] tags)
        {
            return new WaitCondition
            {
                messageTags = new HashSet<string>(tags.ToList())
            };
        }

        public static WaitCondition CommandsReceived(params string[] cmds)
        {
            return new WaitCondition
            {
                commands = new HashSet<string>(cmds.ToList())
            };
        }

        public class TimerStateMachine
        {
            public bool Repeat { get; set; }
            public bool Running { get; private set; }

            public ExitFlag ExitFlag;
            public IEnumerable<WaitCondition> Sequence;
            public IEnumerable<WaitCondition> NextSequence;

            public WaitCondition CurrentWaitCondition { get; private set; }

            private IEnumerator<WaitCondition> sequenceSM;
            public int IterationSpeed { get; set; }

            public static char[] PROGRESS = "-\\|/".ToCharArray();

            public bool hasProgress;
            private long activations = 0;

            public TimerStateMachine(
                IEnumerable<WaitCondition> sequence = null,
                int iterationSpeed = 10,
                bool autoStart = false
            )
            {
                Sequence = sequence;
                Repeat = autoStart;
                ExitFlag = new ExitFlag();
                this.IterationSpeed = iterationSpeed;
            }

            public void Start()
            {
                SetSequenceSM(Sequence);
            }

            public void Restart(IEnumerable<WaitCondition> next, bool graceful)
            {
                if (sequenceSM == null || !graceful)
                {
                    Sequence = next;
                    SetSequenceSM(Sequence);
                }
                else
                {
                    ExitFlag.ShouldExit = true;
                    NextSequence = next;
                }
            }

            public void Stop()
            {
                ExitFlag.ShouldExit = true;
                NextSequence = null;
            }

            public void Run(string argument, UpdateType updateSource)
            {
                if (sequenceSM == null)
                {
                    return;
                }

                CurrentWaitCondition.Progress(argument, updateSource);


                while (true)
                {
                    if (!CurrentWaitCondition.ShouldInterrupt())
                    {
                        return;
                    }

                    bool doneSeq;
                    while (true)
                    {
                        hasProgress = true;
                        doneSeq = !sequenceSM.MoveNext();

                        if (!doneSeq)
                        {
                            CurrentWaitCondition = sequenceSM.Current;

                            CurrentWaitCondition.Enter(ExitFlag);

                            if (CurrentWaitCondition.exitOnce)
                            {
                                doneSeq = true;
                                break;
                            }
                            else if (CurrentWaitCondition.exitRepeat)
                            {
                                doneSeq = true;
                                NextSequence = null;
                                ExitFlag.ShouldExit = true;
                                break;
                            }
                            else if (CurrentWaitCondition.yieldTick)
                            {
                                break;
                            }

                            if (CurrentWaitCondition.yieldTickIfLimit)
                            {
                                if (ProgramInstance.Runtime.CurrentInstructionCount < ProgramInstance.Runtime.MaxInstructionCount * 0.8)
                                {
                                    continue;
                                }
                            }

                            if (CurrentWaitCondition.HasPendingMessages())
                            {
                                continue;
                            }

                            if (CurrentWaitCondition.HasPendingCommands())
                            {
                                continue;
                            }

                            if (CurrentWaitCondition.delay <= kEpsilon)
                            {
                                continue;
                            }
                        }
                        break;
                    }


                    if (doneSeq)
                    {
                        if (Repeat && !ExitFlag.ShouldExit)
                        {
                            if (NextSequence != null)
                            {
                                Sequence = NextSequence;
                                ExitFlag.ShouldExit = false;
                            }
                            SetSequenceSM(Sequence);
                            continue;
                        }
                        else
                        {
                            if (NextSequence != null)
                            {
                                Sequence = NextSequence;
                                SetSequenceSM(Sequence);
                                ExitFlag.ShouldExit = false;
                                continue;
                            }
                            SetSequenceSM(null);
                        }
                    }
                    break;
                }
            }

            public String GetStatus()
            {
                string status = CurrentWaitCondition.GetStatus();
                if (status == null)
                {
                    status = CurrentWaitCondition.ToString();
                }

                if (hasProgress)
                {
                    activations++;
                    hasProgress = false;
                }
                return PROGRESS[activations % PROGRESS.Length] + " " + status;
            }

            private void SetSequenceSM(IEnumerable<WaitCondition> seq)
            {
                Running = false;
                CurrentWaitCondition = new WaitCondition
                {
                    delay = 0
                };

                sequenceSM?.Dispose();
                sequenceSM = null;

                if (seq != null)
                {
                    Running = true;
                    sequenceSM = seq.GetEnumerator();
                }
            }

            internal bool ShouldRun(UpdateType updateSource)
            {
                if (updateSource.HasFlag(UpdateType.Terminal))
                {
                    return true;
                }
                if (updateSource.HasFlag(UpdateType.IGC))
                {
                    return true;
                }
                if (updateSource.HasFlag(UpdateType.Mod))
                {
                    return true;
                }
                if (updateSource.HasFlag(UpdateType.Once))
                {
                    return true;
                }
                if (updateSource.HasFlag(UpdateType.Script))
                {
                    return true;
                }
                if (updateSource.HasFlag(UpdateType.Trigger))
                {
                    return true;
                }
                if (updateSource.HasFlag(UpdateType.Update100))
                {
                    return IterationSpeed >= 100;
                }
                if (updateSource.HasFlag(UpdateType.Update10))
                {
                    return IterationSpeed >= 10;
                }
                if (updateSource.HasFlag(UpdateType.Update1))
                {
                    return IterationSpeed >= 1;
                }

                return false;
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

        public class TimeSource
        {
            public double Time { get; set; } = 0;

            public void Progress()
            {
                Time += ProgramInstance.Runtime.TimeSinceLastRun.TotalMilliseconds;
            }
        }

        public class MissileGuidance
        {
            public double Gain = 3;
            public double DampnersGain = 3;
            public double PNGain = 3;
            public double MaxThrust = 2.0;

            private double TimeSaved;

            public IMyTextSurface TextPanel { get; }

            private double YawSaved;
            private double PitchSaved;
            private double MissileAccel = 10;

            private Vector3D previousPosition;
            private Vector3D previousTargetPosition;

            private List<IMyThrust> ForwardThrusters;
            private List<IMyGyro> Gyros;
            private IMyShipController Controller;

            public MissileGuidance(
                List<IMyThrust> forwardThrusters,
                List<IMyGyro> gyros,
                IMyShipController controller,
                IMyTextSurface textPanel
                )
            {
                this.ForwardThrusters = forwardThrusters;
                this.Gyros = gyros;
                this.Controller = controller;
                this.TimeSaved = ProgramTimeSource.Time;
                this.TextPanel = textPanel;
            }

            public void RefreshMass(List<IMyCubeBlock> blocks)
            {
                double missileMass = 0;
                double missileThrust = 0;

                Each(blocks, b => missileMass += b.Mass);
                Each(ForwardThrusters, t => missileThrust += t.MaxThrust);

                MissileAccel = missileThrust / missileMass;
            }

            public Vector3D GetPos()
            {
                return Gyros[0].CubeGrid.WorldVolume.Center;
            }

            public void Update(Vector3D targetPosition)
            {
                if (Gyros.Count == 0)
                {
                    return;
                }

                double TimePrev = TimeSaved;
                double TickMs = Math.Max(ProgramTimeSource.Time - TimePrev, 0.018);
                TimeSaved = ProgramTimeSource.Time;

                Vector3D MissilePosition = Gyros[0].CubeGrid.WorldVolume.Center;
                Vector3D MissilePositionPrev = previousPosition;
                Vector3D MissileVelocity = (MissilePosition - MissilePositionPrev) / TickMs;

                Vector3D TargetPositionPrev = previousTargetPosition;
                Vector3D TargetVelocity = (targetPosition - previousTargetPosition) / TickMs;

                previousTargetPosition = targetPosition;
                previousPosition = MissilePosition;

                //TextPanel.WriteText(TargetVelocity.ToString() + "\n" + MissileVelocity.ToString());

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
                Vector3D GravityComp = -Controller.GetNaturalGravity();

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

                ThrustPower = Math.Min(MaxThrust, MathHelper.Clamp(ThrustPower, 0.4, 1)); //for improved thrust performance on the get-go
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
                am = Vector3D.Normalize(LateralAccelerationComponent + Vector3.Multiply(GravityComp, 2));

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
                    gyro.GyroOverride = true;
                    gyro.Pitch = (float)MathHelper.Clamp((-TRANS_VECT.X) * Gain, -1000, 1000);
                    gyro.Yaw = (float)MathHelper.Clamp(((-TRANS_VECT.Y)) * Gain, -1000, 1000);
                    gyro.Roll = (float)MathHelper.Clamp(((-TRANS_VECT.Z)) * Gain, -1000, 1000);
                }
            }

            public void CancelOverride()
            {
                foreach (IMyThrust thrust in ForwardThrusters)
                {
                    thrust.ThrustOverride = 0;
                }
                foreach (IMyGyro gyro in Gyros)
                {
                    gyro.GyroOverride = false;
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
            private char fg, bg;
            private char[] screen;
            private int[] clip = new int[4];
            const double BIT_SPACING = 255.0 / 7.0;

            public Graphics(int w, int h, IMyTextPanel c)
            {
                width = w;
                height = h;
                screen = new char[height * (width + 1)];
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
                // string k = r + ":" + g + ":" + b;
                int rr = (int)Math.Round(r / BIT_SPACING);
                int gg = (int)Math.Round(g / BIT_SPACING);
                int bb = (int)Math.Round(b / BIT_SPACING);
                int c = (rr << 6) + (gg << 3) + bb;
                fg = (char)(0xe100 + c);
            }
            public void setBG(int r, int g, int b)
            {
                int rr = (int)Math.Round(r / BIT_SPACING);
                int gg = (int)Math.Round(g / BIT_SPACING);
                int bb = (int)Math.Round(b / BIT_SPACING);
                int c = (rr << 6) + (gg << 3) + bb;
                bg = (char)(0xe100 + c);
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
                console.WriteText(string.Join(null, screen));
            }
            public void clear()
            {
                screen = Enumerable.Repeat(bg, (width + 1) * height).ToArray();
                for (int i = 0; i < height; i++)
                {
                    screen[i * (width + 1) + width] = '\n';
                }
            }
            public void pixel(int x, int y)
            {
                if (x >= clip[0] && x <= clip[2] && y >= clip[1] && y <= clip[3])
                {
                    screen[(width + 1) * y + x] = fg;
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
#if DEBUG
    }
}
#endif
