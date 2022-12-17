#if DEBUG
using Sandbox;
using Sandbox.Game.Entities;
using Sandbox.ModAPI.Ingame;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VRage.Game.ModAPI.Ingame;
using VRageMath;
using static VRage.Game.MyObjectBuilder_ControllerSchemaDefinition;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
#endif
        public static Program ProgramInstance;
        public static TimerRoutineRunner RoutineRunner;
        public static TimeSource ProgramTimeSource;
        public static Random ProgramRandom;
        public static Mesh MeshNetwork = new Mesh();
        public static ScriptKinds ProgramScriptKinds = new ScriptKinds();
        private static bool FirstIteration = true;

        public static CommandCenterProgram commandCenterProgram;
        public static MissileProgram missileProgram;
        public static AimShipProgram aimShipProgram;

        public Program()
        {
            ProgramInstance = this;
            ProgramRandom = new Random();
            ProgramTimeSource = new TimeSource();
            RoutineRunner = new TimerRoutineRunner();
            Runtime.UpdateFrequency = UpdateFrequency.Once;
            commandCenterProgram = new CommandCenterProgram(ProgramScriptKinds);
            missileProgram = new MissileProgram(ProgramScriptKinds);
            aimShipProgram = new AimShipProgram(ProgramScriptKinds);
        }

        public struct Aim
        {
            public String name;
            public Vector3 position;
        }

        public class CommandCenterProgram : ScriptKind
        {
            private Graphics g;
            private IMyTextPanel lcd;
            private WaitCondition msgWaitCondition;
            private List<Aim> aims = new List<Aim>();
            private int currentParticipant = 0;
            private int currentAim = 0;
            WaitCondition commandWait;

            public CommandCenterProgram(ScriptKinds kinds)
            {
                kinds.Register(this);
            }

            public IEnumerable<WaitCondition> Start(string arguments)
            {
                MeshNetwork.Start(API.Target.targetAimedCall);
                lcd = ProgramInstance.GetBlockOfType(lcd, (p) => p.DisplayNameText == "LCD");
                if (g == null) g = new Graphics(165, 165, lcd);
                g.clear();
                g.paint();
                g.setFG(0, 255, 0);
                RoutineRunner.Start(Print());
                RoutineRunner.Start(StartMessageLoop(), 0, false);
                RoutineRunner.Start(CmdDispatch());
                yield break;
            }

            public IEnumerable<WaitCondition> Print()
            {
                ExitFlag exitFlag = new ExitFlag();

                while (!exitFlag.ShouldExit)
                {
                    g.clear();
                    var y = 3;
                    var id = 0;

                    g.setFG(255, 0, 0);

                    g.print(3, y, "Aims:");
                    y += 6;

                    id = 0;
                    foreach (var aim in aims)
                    {
                        g.print(3, 3 + y, (id == currentAim ? ">" : " ") + "#" + id + " " + aim.name + " " + aim.position.ToString());
                        y += 6;
                        id++;
                    }

                    g.line(3, y, 10, y);
                    y += 6;
                    id = 0;


                    g.setFG(0, 255, 0);
                    foreach (var part in MeshNetwork.Participants)
                    {
                        g.print(3, y, (id == currentParticipant ? ">" : " ") + "#" + id + " " + part.gridName + " " + part.status);
                        y += 6;
                        id++;
                    }


                    g.paint();
                    yield return Delay(100)
                        .WithStatus("monitoring")
                        .WithExitFlagProvider(ef => exitFlag = ef);
                }
                yield break;
            }

            public IEnumerable<WaitCondition> CmdDispatch()
            {
                commandWait = CommandsReceived("%nextaim", "%prevaim", "%nextmiss", "%prevmiss", "%launch")
                    .WithStatus("cmd in");

                yield return commandWait;

                if (commandWait.IsCommandTriggered("%nextaim"))
                {
                    currentAim = Math.Min(currentAim + 1, aims.Count - 1);
                }
                if (commandWait.IsCommandTriggered("%prevaim"))
                {
                    currentAim = Math.Max(currentAim - 1, 0);
                }
                if (commandWait.IsCommandTriggered("%nextmiss"))
                {
                    currentParticipant = Math.Min(currentParticipant + 1, MeshNetwork.Participants.Count - 1);
                }
                if (commandWait.IsCommandTriggered("%prevmiss"))
                {
                    currentParticipant = Math.Max(currentParticipant - 1, 0);
                }
                if (commandWait.IsCommandTriggered("%launch"))
                {
                    var request = new API.Target.LaunchRequest();
                    request.Position = aims[currentAim].position;

                    foreach (var msg in API.Common.Rpc(
                        API.Target.launchCall,
                        MeshNetwork.Participants[currentParticipant],
                        request, (a) => { }))
                    {
                        yield return msg;
                    }
                }
            }

            public IEnumerable<WaitCondition> StartMessageLoop()
            {
                msgWaitCondition = MessagesReceived(API.Target.targetAimedCall.RequestTag)
                    .WithStatus("rpc server");

                RoutineRunner.Start(DispatchLoop(), 0, true);
                yield break;
            }

            public IEnumerable<WaitCondition> DispatchLoop()
            {
                yield return msgWaitCondition;

                foreach (var msg in msgWaitCondition.GetMessages(API.Target.targetAimedCall.RequestTag))
                {
                    API.Target.TargetAimedRequest request = Serializer.DeSerialize<API.Target.TargetAimedRequest>(msg.As<string>());
                    RoutineRunner.Start(RespondAim(request), 0, false);
                }
            }


            private IEnumerable<WaitCondition> RespondAim(API.Target.TargetAimedRequest request)
            {
                Aim aim = new Aim();
                aim.name = request.Name;
                aim.position = request.Position;

                if (aims.Count > 5)
                {
                    aims.RemoveAt(0);
                }
                aims.Add(aim);


                ProgramInstance.IGC.SendBroadcastMessage(
                    request.ResponseTag,
                    new API.Target.TargetAimedResponse
                    {
                        Source = MeshNetwork.Me.source,
                        Cookie = request.Cookie
                    }.Serialize());

                yield break;
            }
        }

        public class AimShipProgram : ScriptKind
        {
            WaitCondition commandWait;
            IMyCameraBlock camera;

            public AimShipProgram(ScriptKinds kinds)
            {
                kinds.Register(this);
            }

            IEnumerable<WaitCondition> ScriptKind.Start(string arguments)
            {
                MeshNetwork.Start();
                RoutineRunner.Start(CmdDispatch());
                camera = ProgramInstance.GetBlockOfType(camera);
                camera.EnableRaycast = true;
                yield break;
            }

            public IEnumerable<WaitCondition> CmdDispatch()
            {
                commandWait = CommandsReceived("%aim")
                    .WithStatus("cmd in");

                yield return commandWait;

                if (commandWait.IsCommandTriggered("%aim"))
                {
                    ProgramInstance.Echo(camera.RaycastDistanceLimit.ToString());
                    if (camera.CanScan(1000))
                    {
                        var result = camera.Raycast(1000, 0, 0);

                        if (!result.IsEmpty())
                        {
                            API.Target.TargetAimedRequest request = new API.Target.TargetAimedRequest();
                            request.Name = result.Name;
                            request.Position = result.HitPosition.Value;

                            Dictionary<Mesh.Participant, API.Target.TargetAimedResponse> responses = new Dictionary<Mesh.Participant, API.Target.TargetAimedResponse>();

                            foreach (var wc in API.Common.Rpc(API.Target.targetAimedCall, request, responses))
                            {
                                yield return wc;
                            }
                        }
                    }
                }
            }
        }


        public class MissileProgram : ScriptKind
        {
            MissileGuidance guidance;
            IMyRemoteControl rc;
            Vector3D targetVector;
            WaitCondition msgWaitCondition;
            string stage = "s0";

            public MissileProgram(ScriptKinds kinds)
            {
                kinds.Register(this);
            }

            public IEnumerable<WaitCondition> Start(string args)
            {
                rc = ProgramInstance.GetBlockOfType(rc);
                MeshNetwork.Start(API.Target.launchCall);
                RoutineRunner.Start(StartMessageLoop(), 0, false);

                if (guidance == null)
                {
                    guidance = new MissileGuidance(
                        ProgramInstance.GetBlocksOfType<IMyThrust>(null, c => c.Orientation.Forward == Base6Directions.GetOppositeDirection(rc.Orientation.Forward)),
                        ProgramInstance.GetBlocksOfType<IMyGyro>(null),
                        rc,
                        ProgramInstance.GetBlockOfType<IMyProgrammableBlock>(null).GetSurface(0)
                    );

                    guidance.RefreshMass(ProgramInstance.GetBlocksOfType<IMyCubeBlock>(null));

                }

                yield break;
            }

            public IEnumerable<WaitCondition> StartMessageLoop()
            {
                msgWaitCondition = MessagesReceived(API.Target.launchCall.RequestTag)
                    .WithStatus("rpc server");

                RoutineRunner.Start(DispatchLoop(), 0, true);
                yield break;
            }

            public IEnumerable<WaitCondition> DispatchLoop()
            {
                yield return msgWaitCondition;

                foreach (var msg in msgWaitCondition.GetMessages(API.Target.launchCall.RequestTag))
                {
                    API.Target.LaunchRequest request = Serializer.DeSerialize<API.Target.LaunchRequest>(msg.As<string>());
                    RoutineRunner.Start(RespondLaunch(request), 0, false);
                }
            }

            private IEnumerable<WaitCondition> RespondLaunch(API.Target.LaunchRequest request)
            {
                RoutineRunner.Start(Fly(request.Position), 1, false);

                ProgramInstance.IGC.SendBroadcastMessage(
                    request.ResponseTag,
                    new API.Target.LaunchResponse
                    {
                        Source = MeshNetwork.Me.source,
                        Cookie = request.Cookie
                    }.Serialize()
                );

                yield break;
            }

            private IEnumerable<WaitCondition> Fly(Vector3 position)
            {
                foreach (var mergeBlock in ProgramInstance.GetBlocksOfType<IMyShipMergeBlock>(null))
                {
                    mergeBlock.Enabled = false;
                }
                targetVector = Vector3.Subtract(guidance.GetPos(), Vector3.Multiply(Vector3.Normalize(rc.GetNaturalGravity()), 100));

                stage = "s1";

                foreach (var wc in GoThere(50))
                {
                    yield return wc;
                }

                stage = "s2";
                targetVector = position;//  Vector3.Subtract(position, Vector3.Multiply(Vector3.Normalize(rc.GetNaturalGravity()), 10));

                foreach (var wc in GoThere(50))
                {
                    yield return wc;
                }

                foreach (var bl in ProgramInstance.GetBlocksOfType<IMyBatteryBlock>(null))
                {
                    bl.Enabled = false;
                }

                yield break;
            }


            public IEnumerable<WaitCondition> GoThere(double d)
            {
                ExitFlag exitFlag = new ExitFlag();
                try
                {
                    while (!exitFlag.ShouldExit)
                    {
                        guidance.Update(targetVector);

                        double dist = Vector3.Distance(guidance.GetPos(), targetVector);
                        MeshNetwork.Status = stage + " " + dist;

                        if (dist < d)
                        {
                            break;
                        }

                        yield return Delay(100)
                            .WithExitFlagProvider(ef => exitFlag = ef)
                            .WithStatus("go " + dist);
                    }
                }
                finally
                {
                    guidance.CancelOverride();
                }
            }

            public void Save()
            {
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
                if (arr.Length == 0 || !ProgramScriptKinds.kinds.ContainsKey(arr[0]))
                {
                    throw new Exception("specify CustomData of programable block 'cmd:args', where cmd is out of set: " +
                        String.Join(", ", ProgramScriptKinds.kinds.Keys));
                }

                ScriptKind kind = ProgramScriptKinds.kinds[arr[0]];
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

            int[][] allPolys;
            long yieldSkipper = 0;
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

            class Comparer : IComparer<int[]>
            {
                public int Compare(int[] x, int[] y)
                {
                    return x[8].CompareTo(y[8]);
                }
            }

            public IEnumerable<WaitCondition> Draw(bool vectorMode, int x, int y, int[][] blocks, int n, double inc, int s, Graphics g)
            {
                scale = s;
                angle1 = inc;
                pickSides();
                int polysSz = n * 3;
                while (allPolys == null || allPolys.Length < polysSz)
                {
                    int newSz = allPolys == null ? 128 : allPolys.Length * 2;
                    allPolys = new int[newSz][];
                }
                for (int b = 0; b < n; b++)
                {
                    allPolys[b * 3] = new int[10];
                    allPolys[b * 3 + 1] = new int[10];
                    allPolys[b * 3 + 2] = new int[10];
                    cube3d(blocks[b][0], blocks[b][1], blocks[b][2], new int[][] { allPolys[b * 3], allPolys[b * 3 + 1], allPolys[b * 3 + 2] });
                    if (yieldSkipper++ == 10)
                    {
                        yieldSkipper = 0;
                        yield return YieldTickIfLimit();
                    }
                }

                yield return YieldTick();
                Array.Sort(allPolys, 0, polysSz, new Comparer());

                for (int i = 0; i < polysSz; i++)
                {

                    if (yieldSkipper++ == 10)
                    {
                        yieldSkipper = 0;
                        yield return YieldTickIfLimit();
                    }
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

            private static void swap(int[][] a, int i, int j)
            {
                int[] temp = a[i];
                a[i] = a[j];
                a[j] = temp;
            }
        }


        public abstract class Serializable
        {
            protected Dictionary<String, Field> fields = new Dictionary<String, Field>();
            public abstract void SaveToFields();

            public abstract void LoadFields(Dictionary<String, Field> fields);

            public string Serialize()
            {
                SaveToFields();
                return Field.DicToString(fields);
            }
            public Dictionary<String, Field> GetFields()
            {
                SaveToFields();
                return fields;
            }
        }

        public class Field
        {
            private Byte[] value;
            public Dictionary<string, Field> children = new Dictionary<string, Field>();

            public Field(Byte[] value)
            {
                this.value = value;
            }

            public Field(Serializable sObject)
            {
                if (sObject == null)
                {
                    children["__null"] = new Field(true);
                }
                else
                {
                    sObject.SaveToFields();
                    children = sObject.GetFields();
                }
            }

            public Field(Vector3 value)
            {
                children["x"] = new Field(value.X);
                children["y"] = new Field(value.Y);
                children["z"] = new Field(value.Z);
            }

            public Field(float value)
            {
                this.value = BitConverter.GetBytes(value);
            }

            public Field(double value)
            {
                this.value = BitConverter.GetBytes(value);
            }

            public Field(int value)
            {
                this.value = BitConverter.GetBytes(value);
            }

            public Field(long value)
            {
                this.value = BitConverter.GetBytes(value);
            }

            public Field(bool value)
            {
                this.value = BitConverter.GetBytes(value);
            }

            public Field(string value)
            {
                if (value == null)
                {
                    children["__null"] = new Field(true);
                }
                else
                {
                    this.value = Encoding.ASCII.GetBytes(value);
                }
            }

            public Field(Dictionary<string, Field> children)
            {
                this.children = children;
            }

            public Field(List<Field> lst)
            {
                Dictionary<string, Field> dict = new Dictionary<string, Field>();
                dict.Add("_count", new Field(lst.Count));
                for (int i = 0; i < lst.Count; i++)
                {
                    dict.Add(i.ToString(), lst[i]);
                }
                this.children = dict;
            }

            public Field(List<string> lst)
            {
                Dictionary<string, Field> dict = new Dictionary<string, Field>();
                dict.Add("_count", new Field(lst.Count));
                for (int i = 0; i < lst.Count; i++)
                {
                    dict.Add(i.ToString(), new Field(lst[i]));
                }
                this.children = dict;
            }

            public Field(List<long> lst)
            {
                Dictionary<string, Field> dict = new Dictionary<string, Field>();
                dict.Add("_count", new Field(lst.Count));
                for (int i = 0; i < lst.Count; i++)
                {
                    dict.Add(i.ToString(), new Field(lst[i]));
                }
                this.children = dict;
            }

            private List<T> fetchList<T>(Func<Field, T> convert)
            {
                List<T> result = new List<T>();

                int count = children["_count"].GetInt();

                for (int i = 0; i < count; i++)
                {
                    result.Add(convert.Invoke(children[i.ToString()]));
                }


                return result;
            }

            public List<T> GetList<T>() where T : Serializable, new()
            {
                return fetchList(f => f.GetObject<T>());

            }

            public List<string> GetStringList()
            {
                return fetchList(f => f.GetString());
            }

            public List<long> GetLongList()
            {
                return fetchList(f => f.GetLong());
            }

            public List<int> GetIntList()
            {
                return fetchList(f => f.GetInt());
            }

            public Vector3 GetVector3()
            {
                Vector3 vector = new Vector3();
                vector.X = children["x"].GetFloat();
                vector.Y = children["y"].GetFloat();
                vector.Z = children["z"].GetFloat();
                return vector;
            }
            public float GetFloat()
            {
                return BitConverter.ToSingle(value, 0);
            }

            public double GetDouble()
            {
                return BitConverter.ToDouble(value, 0);
            }

            public int GetInt()
            {
                return BitConverter.ToInt32(value, 0);
            }

            public long GetLong()
            {
                return BitConverter.ToInt64(value, 0);
            }

            public bool GetBool()
            {
                return BitConverter.ToBoolean(value, 0);
            }

            public string GetString()
            {
                if (children.ContainsKey("__null"))
                {
                    return null;
                }
                else
                {
                    return ASCIIEncoding.ASCII.GetString(value);
                }
            }

            public T GetObject<T>() where T : Serializable, new()
            {
                if (children.ContainsKey("__null"))
                {
                    return null;
                }
                else
                {
                    T obj = new T();
                    obj.LoadFields(children);
                    return obj;
                }
            }


            public Byte[] GetBytes()
            {
                return value;
            }


            public static string DicToString(Dictionary<string, Field> fields)
            {
                string result = "";
                foreach (KeyValuePair<string, Field> child in fields)
                {
                    result += child.Key + ":" + child.Value.ToString();
                };
                return result;
            }

            private static string BytesToString(byte[] ba)
            {
                string hex = BitConverter.ToString(ba);
                return hex.Replace("-", "");
            }

            public override string ToString()
            {
                string result = "{";
                if (value != null)
                {
                    result += BytesToString(value);
                }
                else
                {
                    result += DicToString(children);
                }
                result += "}";
                return result;
            }
        }


        class Serializer
        {
            private static int closingBracket(string s)
            {
                int nextOpening = s.IndexOf('{');
                int nextClosing = s.IndexOf('}');
                while (nextOpening != -1 && nextOpening < nextClosing)
                {
                    nextOpening = s.IndexOf('{', nextOpening + 1);
                    nextClosing = s.IndexOf('}', nextClosing + 1);
                }
                return nextClosing;
            }


            private static Byte[] StringToBytes(string byteString)
            {
                int NumberChars = byteString.Length;
                byte[] bytes = new byte[NumberChars / 2];
                for (int i = 0; i < NumberChars; i += 2)
                {
                    bytes[i / 2] = Convert.ToByte(byteString.Substring(i, 2), 16);
                }
                return bytes;
            }

            public static Field parseField(string field)
            {
                string value = "";
                if (field.IndexOf(':') != -1)
                {
                    return new Field(StringToFields(field));
                }
                for (int i = 0; i < field.Length; i++)
                {
                    value += field[i];
                }
                return new Field(StringToBytes(value));
            }

            private static Dictionary<String, Field> StringToFields(string fields)
            {
                Dictionary<String, Field> result = new Dictionary<String, Field>();
                string fieldName = "";
                for (int i = 0; i < fields.Length; i++)
                {
                    if (fields[i] == '}')
                    {
                        return result;
                    }
                    else if (fields[i] == ':' && fields[i + 1] == '{')
                    {
                        string subField = fields.Substring(i + 2);
                        int subEnd = closingBracket(subField);
                        subField = subField.Substring(0, subEnd);
                        result.Add(fieldName, parseField(subField));
                        i += 2 + subEnd;
                        fieldName = "";
                    }
                    else
                    {
                        fieldName += fields[i];
                    }
                }
                return result;
            }

            public static T DeSerialize<T>(string obj) where T : Serializable, new()
            {
                T result = new T();
                Dictionary<String, Field> fieldsFromString = StringToFields(obj);
                result.LoadFields(fieldsFromString);
                return result;
            }
        }


        public partial class Mesh
        {

            public static long DOWN_TIMEOUT = 2000;
            public static long REMOVE_TIMEOUT = 3000;
            private static string PINGS_TAG = "%mesh:pings";


            public class Participant : Serializable
            {
                public long source;
                public String gridName;
                public long sequence;
                public List<string> supportedCalls;
                public double localTimestamp = ProgramTimeSource.Time;
                public bool aliveStatus;
                public string status = "";

                public override void LoadFields(Dictionary<string, Field> fields)
                {
                    source = fields["source"].GetLong();
                    sequence = fields["sequence"].GetLong();
                    gridName = fields["gridName"].GetString();
                    supportedCalls = fields["supportedCalls"].GetStringList();
                    if (fields.ContainsKey("status"))
                    {
                        status = fields["status"].GetString();
                    }
                }

                public override void SaveToFields()
                {
                    fields["source"] = new Field(source);
                    fields["sequence"] = new Field(sequence);
                    fields["gridName"] = new Field(gridName);
                    fields["supportedCalls"] = new Field(supportedCalls);
                    fields["status"] = new Field(status);
                }

                internal bool IsAlive()
                {
                    return ProgramTimeSource.Time - localTimestamp < DOWN_TIMEOUT;
                }

                internal bool ShouldRemove()
                {
                    return ProgramTimeSource.Time - localTimestamp > REMOVE_TIMEOUT;
                }

                public override string ToString()
                {
                    return gridName;
                }
            }

            public delegate void ParticipantsListener(Participant participant, bool alive);

            private Dictionary<long, Participant> participants = new Dictionary<long, Participant>();
            private List<ParticipantsListener> participantsListeners = new List<ParticipantsListener>();
            private List<string> callsSupported = new List<string>();

            public int seq = 0;

            public Participant Me { get; internal set; }

            public List<Participant> Participants
            {
                get { return participants.Values.ToList(); }
            }

            public string Status = "";

            public void AddListener(ParticipantsListener listener)
            {
                foreach (var participant in participants.Values)
                {
                    listener.Invoke(participant, participant.IsAlive());
                }
                participantsListeners.Add(listener);
            }

            public void RemoveListener(ParticipantsListener listener)
            {
                participantsListeners.Remove(listener);
            }

            public List<Participant> getParticipants()
            {
                return new List<Participant>(participants.Values).FindAll(p => p.IsAlive());
            }

            internal Participant NextMe()
            {
                return new Participant
                {
                    source = ProgramInstance.Me.EntityId,
                    gridName = ProgramInstance.Me.CubeGrid.DisplayName,
                    supportedCalls = callsSupported,
                    sequence = seq++,
                    status = Status
                };
            }

            public IEnumerable<WaitCondition> PingerLoop()
            {
                Participant participant = NextMe();
                Me = participant;

                KeepAlive(participant);
                ProgramInstance.IGC.SendBroadcastMessage(PINGS_TAG, participant.Serialize());

                yield return Delay(500)
                    .WithStatus("ping out");

                List<long> toRemove = new List<long>();
                foreach (long id in participants.Keys)
                {
                    if (!participants[id].IsAlive())
                    {
                        if (participant.aliveStatus)
                        {
                            foreach (var listener in participantsListeners)
                            {
                                listener.Invoke(participants[id], false);
                            }
                        }
                        participant.aliveStatus = false;
                    }

                    if (participants[id].ShouldRemove())
                    {
                        toRemove.Add(id);
                    }
                }

                foreach (long id in toRemove)
                {
                    participants.Remove(id);
                }
            }

            public IEnumerable<WaitCondition> PingReceiverLoop()
            {
                WaitCondition wc = MessagesReceived(PINGS_TAG)
                    .WithStatus("ping in " + participants.Count);

                yield return wc;

                foreach (var msg in wc.GetMessages(PINGS_TAG))
                {
                    string data = msg.As<string>();
                    var participant = Serializer.DeSerialize<Participant>(data);
                    KeepAlive(participant);
                }
            }

            private bool KeepAlive(Participant participant)
            {
                long id = participant.source;
                bool wasAlive = false;
                long lastSeq = -1;
                if (participants.ContainsKey(id))
                {
                    wasAlive = participants[id].aliveStatus;
                    lastSeq = participants[id].sequence;
                    participants.Remove(id);
                }

                participants[id] = participant;

                if (!wasAlive)
                {
                    foreach (var listener in participantsListeners)
                    {
                        listener.Invoke(participant, true);
                    }
                }

                participant.aliveStatus = true;

                return participant.sequence > lastSeq;
            }

            public void Start(params API.Common.EntryPoint[] supportedCalls)
            {
                foreach (var call in supportedCalls)
                {
                    callsSupported.Add(call.RequestTag);
                }
                Me = NextMe();
                RoutineRunner.Start(PingerLoop(), 100, true);
                RoutineRunner.Start(PingReceiverLoop(), 0, true);
            }
        }

        public partial class API
        {
            public partial class Target
            {
                public static Common.APICall<TargetAimedRequest, TargetAimedResponse> targetAimedCall =
                    new Common.APICall<TargetAimedRequest, TargetAimedResponse>
                    {
                        RequestTag = "%target:aimed:request",
                        ResponseTagPrefix = "%target:aimed:response:"
                    };


                public class TargetAimedRequest : Serializable, Common.Request
                {
                    public long Cookie { get; set; }
                    public List<long> Recepients { get; set; }

                    public String ResponseTag { get; set; }

                    public String Name { get; set; }
                    public Vector3 Position { get; set; }

                    public override void LoadFields(Dictionary<string, Field> fields)
                    {
                        Cookie = fields["cookie"].GetInt();
                        Recepients = fields["recepients"].GetLongList();
                        ResponseTag = fields["responseTag"].GetString();
                        Name = fields["name"].GetString();
                        Position = fields["pos"].GetVector3();
                    }

                    public override void SaveToFields()
                    {
                        fields["cookie"] = new Field(Cookie);
                        fields["recepients"] = new Field(Recepients);
                        fields["responseTag"] = new Field(ResponseTag);
                        fields["name"] = new Field(Name);
                        fields["pos"] = new Field(Position);
                    }
                }

                public class TargetAimedResponse : Serializable, Common.Response
                {
                    public long Cookie { get; set; }
                    public long Source { get; set; }

                    public override void LoadFields(Dictionary<string, Field> fields)
                    {
                        Cookie = fields["cookie"].GetInt();
                        Source = fields["source"].GetLong();
                    }

                    public override void SaveToFields()
                    {
                        fields["cookie"] = new Field(Cookie);
                        fields["source"] = new Field(Source);
                    }
                }
            }

            public partial class Target
            {
                public static Common.APICall<LaunchRequest, LaunchResponse> launchCall =
                    new Common.APICall<LaunchRequest, LaunchResponse>
                    {
                        RequestTag = "%missile:launch:request",
                        ResponseTagPrefix = "%missile:launch:response:"
                    };


                public class LaunchRequest : Serializable, Common.Request
                {
                    public long Cookie { get; set; }
                    public List<long> Recepients { get; set; }

                    public String ResponseTag { get; set; }

                    public String Name { get; set; }
                    public Vector3 Position { get; set; }

                    public override void LoadFields(Dictionary<string, Field> fields)
                    {
                        Cookie = fields["cookie"].GetInt();
                        Recepients = fields["recepients"].GetLongList();
                        ResponseTag = fields["responseTag"].GetString();
                        Name = fields["name"].GetString();
                        Position = fields["pos"].GetVector3();
                    }

                    public override void SaveToFields()
                    {
                        fields["cookie"] = new Field(Cookie);
                        fields["recepients"] = new Field(Recepients);
                        fields["responseTag"] = new Field(ResponseTag);
                        fields["name"] = new Field(Name);
                        fields["pos"] = new Field(Position);
                    }
                }

                public class LaunchResponse : Serializable, Common.Response
                {
                    public long Cookie { get; set; }
                    public long Source { get; set; }

                    public override void LoadFields(Dictionary<string, Field> fields)
                    {
                        Cookie = fields["cookie"].GetInt();
                        Source = fields["source"].GetLong();
                    }

                    public override void SaveToFields()
                    {
                        fields["cookie"] = new Field(Cookie);
                        fields["source"] = new Field(Source);
                    }
                }
            }

            public partial class Common
            {

                public interface Request
                {
                    long Cookie { get; set; }
                    List<long> Recepients { get; set; }
                    string ResponseTag { get; set; }
                }

                public interface Response
                {
                    long Cookie { get; set; }
                    long Source { get; set; }
                }

                public interface EntryPoint
                {
                    string RequestTag { get; }
                }

                public class APICall<I, O> : EntryPoint where I : Request where O : Response
                {
                    public string RequestTag { get; set; }
                    public string ResponseTagPrefix { get; set; }

                    public List<Mesh.Participant> FilterParticipants(List<Mesh.Participant> participants)
                    {
                        return participants.FindAll(p => p.supportedCalls.Contains(RequestTag));
                    }
                }

                public static IEnumerable<WaitCondition> Rpc<I, O>(
                    API.Common.APICall<I, O> call,
                    List<Mesh.Participant> participants,
                    I request,
                    Dictionary<Mesh.Participant, O> responses
                    ) where I : Serializable, Request where O : Serializable, Response, new()
                {
                    if (participants.Count == 0)
                    {
                        yield break;
                    }

                    string responseTag = call.ResponseTagPrefix + ":" + ProgramInstance.Me.EntityId;
                    request.Recepients = participants.ConvertAll(p => p.source);
                    request.ResponseTag = responseTag;
                    request.Cookie = ProgramRandom.Next();

                    ProgramInstance.RegisterBroadcastListener(null, request.ResponseTag);

                    ProgramInstance.IGC.SendBroadcastMessage(
                        call.RequestTag,
                        request.Serialize()
                    );

                    responses.Clear();

                    ExitFlag exitFlag = new ExitFlag();
                    while (!exitFlag.ShouldExit)
                    {
                        WaitCondition wc = MessagesReceived(responseTag)
                            .WithDelay(3000)
                            .WithExitFlagProvider(ef => exitFlag = ef)
                            .WithStatus("rpc " + call.RequestTag);

                        yield return wc;

                        foreach (var msg in wc.GetMessages(responseTag))
                        {
                            O response = Serializer.DeSerialize<O>(msg.As<string>());
                            Mesh.Participant p = participants.Find(f => f.source == response.Source);
                            if (p != null)
                            {
                                ProgramInstance.Echo(p.ToString());
                                responses.Add(p, response);
                                ProgramInstance.Echo(responses.Count + " " + participants.Count);
                                if (responses.Count == participants.Count)
                                {
                                    yield break;
                                }
                            }
                        }
                    }
                }

                public static IEnumerable<WaitCondition> Rpc<I, O>(
                    API.Common.APICall<I, O> call,
                    Mesh.Participant participant,
                    I request,
                    Action<O> responseAction
                    ) where I : Serializable, Request where O : Serializable, Response, new()
                {
                    List<Mesh.Participant> list = new List<Mesh.Participant>();
                    list.Add(participant);
                    Dictionary<Mesh.Participant, O> responses = new Dictionary<Mesh.Participant, O>();
                    foreach (var wc in Rpc(call, list, request, responses))
                    {
                        yield return wc;
                    }
                    responseAction(responses[participant]);
                    yield break;
                }

                public static IEnumerable<WaitCondition> Rpc<I, O>(
                    API.Common.APICall<I, O> call,
                    I request,
                    Dictionary<Mesh.Participant, O> responses
                    ) where I : Serializable, Request where O : Serializable, Response, new()
                {
                    List<Mesh.Participant> participants = call.FilterParticipants(MeshNetwork.getParticipants());
                    foreach (var wc in Rpc(call, participants, request, responses))
                    {
                        yield return wc;
                    }
                }
            }
        }
#if DEBUG
    }
}
#endif
