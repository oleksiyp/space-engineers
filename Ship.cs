using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        ShipProgram shipProgram = new ShipProgram(ProgramScriptKinds);

        class ShipProgram : ScriptKind
        {
            MissileGuidance guidance;
            IMyCockpit cockpit;
            Vector3D targetVector;
            IMyCameraBlock camera;
            IMyShipConnector connector;
            WaitCondition commandWait;

            public ShipProgram(ScriptKinds kinds)
            {
                kinds.Register(this);
            }

            public IEnumerable<WaitCondition> Start(string args)
            {
                RoutineRunner.Start(CmdDispatch(), 0, true);
                
                MeshNetwork.Start();

                MeshNetwork.AddListener((participant, alive) =>
                    ProgramInstance.Echo(MeshNetwork.getParticipants().Count().ToString()));
                yield break;
            }

            public IEnumerable<WaitCondition> CmdDispatch()
            {
                commandWait = CommandsReceived("%up", "%down", "%scan", "%connect")
                    .WithStatus("cmd in");

                cockpit = ProgramInstance.GetBlockOfType(cockpit);

                if (guidance == null)
                {
                    guidance = new MissileGuidance(
                        ProgramInstance.GetBlocksOfType<IMyThrust>(null, c => c.Orientation.Forward == Base6Directions.Direction.Backward),
                        ProgramInstance.GetBlocksOfType<IMyGyro>(null),
                        cockpit,
                        cockpit.GetSurface(0)
                    );

                    guidance.RefreshMass(ProgramInstance.GetBlocksOfType<IMyCubeBlock>(null));

                }

                yield return commandWait;

                if (commandWait.IsCommandTriggered("%up"))
                {
                    RoutineRunner.Start("Go", GoUpRoutine(100), 1, false);
                }
                if (commandWait.IsCommandTriggered("%down"))
                {
                    RoutineRunner.Start("Go", GoUpRoutine(-100), 1, false);
                }
                if (commandWait.IsCommandTriggered("%stop"))
                {
                    RoutineRunner.Stop("Go");
                }
                if (commandWait.IsCommandTriggered("%connect"))
                {
                    RoutineRunner.Toggle("Connect", ConnectRoutine(), 0, false);
                }
            }

            public IEnumerable<WaitCondition> ConnectRoutine()
            {
                connector = ProgramInstance.GetBlockOfType(connector);

                ExitFlag exitFlag = new ExitFlag();

                Dictionary<Mesh.Participant, API.Base.IntrospectConnectorsResponse> responses = new Dictionary<Mesh.Participant, API.Base.IntrospectConnectorsResponse>();

                API.Base.IntrospectConnectorsRequest request = new API.Base.IntrospectConnectorsRequest();

                foreach (var wc in API.Common.Rpc(API.Base.IntrospectConnectorsCall, request, responses))
                {
                    yield return wc;
                }


                API.Base.Connector nearest = null;

                foreach (var resp in responses)
                {
                    foreach (var connector in resp.Value.Connectors)
                    {
                        if (nearest == null || DistToMe(nearest) > DistToMe(connector))
                        {
                            nearest = connector;
                        }
                    }
                }

                IMyShipConnector shipConnector = ProgramInstance.GetBlockOfType<IMyShipConnector>(null);

                if (nearest == null || shipConnector == null)
                {
                    yield break;
                }

                guidance.MaxThrust = 0;

                while (!exitFlag.ShouldExit) {
                    Vector3D pos = shipConnector.GetPosition() + shipConnector.WorldMatrix.Forward * shipConnector.WorldVolume.GetBoundingBox().;
                    double d = (nearest.Position - pos).Length();
                    yield return Delay(500).WithStatus("connector dist\nk" + d);
                }
            }

            private double DistToMe(API.Base.Connector nearest)
            {
                return (ProgramInstance.Me.GetPosition() - nearest.Position).Length();
            }

            public IEnumerable<WaitCondition> GoUpRoutine(double meters)
            {
                ExitFlag exitFlag = new ExitFlag();

                targetVector = cockpit.GetPosition() +
                    cockpit.GetTotalGravity() * -meters;

                try
                {
                    while (!exitFlag.ShouldExit)
                    {
                        guidance.Update(targetVector);

                        yield return Delay(100)
                            .WithExitFlagProvider(ef => exitFlag = ef)
                            .WithStatus("go " + meters);
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
    }
}
