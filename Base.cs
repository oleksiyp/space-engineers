using Sandbox.ModAPI.Ingame;
using System.Collections.Generic;

namespace IngameScript
{

    partial class Program : MyGridProgram
    {
        BaseProgram baseProgram = new BaseProgram(ProgramScriptKinds);

        class BaseProgram : ScriptKind
        {
            private WaitCondition msgWaitCondition;

            public BaseProgram(ScriptKinds kinds)
            {
                kinds.Register(this);
            }

            public IEnumerable<WaitCondition> Start(string arguments)
            {
                msgWaitCondition = MessagesReceived(API.Base.IntrospectConnectorsCall.RequestTag)
                    .WithStatus("rpc server");

                RoutineRunner.Start(DispatchLoop(), 0, true);
                MeshNetwork.Start(API.Base.IntrospectConnectorsCall);
                yield break;
            }

            public IEnumerable<WaitCondition> DispatchLoop()
            {
                yield return msgWaitCondition;

                foreach (var msg in msgWaitCondition.GetMessages(API.Base.IntrospectConnectorsCall.RequestTag))
                {
                    API.Base.IntrospectConnectorsRequest request = Serializer.DeSerialize<API.Base.IntrospectConnectorsRequest>(msg.As<string>());
                    RoutineRunner.Start(RespondConnectors(request), 0, false);
                }
            }


            private IEnumerable<WaitCondition> RespondConnectors(API.Base.IntrospectConnectorsRequest request)
            {
                var connectors = ProgramInstance.GetBlocksOfType<IMyShipConnector>(null);

                ProgramInstance.IGC.SendBroadcastMessage(
                    request.ResponseTag,
                    new API.Base.IntrospectConnectorsResponse
                    {
                        Source = MeshNetwork.Me.source,
                        Cookie = request.Cookie,
                        Connectors = connectors.ConvertAll(c =>
                        new API.Base.Connector
                        {
                            Name = c.DisplayNameText,
                            Position = c.GetPosition() + c.WorldMatrix.Forward,
                            Direction = c.WorldMatrix.Forward
                        })
                    }.Serialize());

                yield break;
            }
        }
    }
}
