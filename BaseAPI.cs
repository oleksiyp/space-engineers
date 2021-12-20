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
        public partial class API
        {
            public partial class Base
            {
                public static Common.APICall<IntrospectConnectorsRequest, IntrospectConnectorsResponse> IntrospectConnectorsCall =
                    new Common.APICall<IntrospectConnectorsRequest, IntrospectConnectorsResponse>
                    {
                        RequestTag = "%base:introspectConnectors:request",
                        ResponseTagPrefix = "%base:introspectConnectors:response:"
                    };

                public class Connector : Serializable
                {
                    public long Id;
                    public String Name;
                    public Vector3D Position;
                    public Vector3D Direction;

                    public override void LoadFields(Dictionary<string, Field> fields)
                    {
                        Id = fields["id"].GetLong();
                        Name = fields["name"].GetString();
                        Position = fields["position"].GetVector3();
                        Direction = fields["direction"].GetVector3();
                    }

                    public override void SaveToFields()
                    {
                        fields["id"] = new Field(Id);
                        fields["name"] = new Field(Name);
                        fields["position"] = new Field(Position);
                        fields["direction"] = new Field(Direction);
                    }
                }

                public class IntrospectConnectorsRequest : Serializable, Common.Request
                {
                    public long Cookie { get; set; }
                    public List<long> Recepients { get; set; }

                    public String ResponseTag { get; set; }

                    public override void LoadFields(Dictionary<string, Field> fields)
                    {
                        Cookie = fields["cookie"].GetInt();
                        Recepients = fields["recepients"].GetLongList();
                        ResponseTag = fields["responseTag"].GetString();
                    }

                    public override void SaveToFields()
                    {
                        fields["cookie"] = new Field(Cookie);
                        fields["recepients"] = new Field(Recepients);
                        fields["responseTag"] = new Field(ResponseTag);
                    }
                }

                public class IntrospectConnectorsResponse : Serializable, Common.Response
                {
                    public long Cookie { get; set; }
                    public long Source { get; set; }

                    public List<Connector> Connectors { get; set; }

                    public override void LoadFields(Dictionary<string, Field> fields)
                    {
                        Cookie = fields["cookie"].GetInt();
                        Source = fields["source"].GetLong();
                        Connectors = fields["connector"].GetList<Connector>();
                    }

                    public override void SaveToFields()
                    {
                        fields["cookie"] = new Field(Cookie);
                        fields["source"] = new Field(Source);
                        fields["connector"] = new Field(Connectors.ConvertAll(c => new Field(c)));
                    }
                }
            }
        }
    }
}
