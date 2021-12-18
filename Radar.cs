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
        bool lcdPart;
        Graphics g;
        List<IMyCameraBlock> cameras;
        IMyTextPanel lcdPanel;
        int cnt = 0;

        int minx = int.MaxValue, miny = int.MaxValue, minz = int.MaxValue;
        int maxx = int.MinValue, maxy = int.MinValue, maxz = int.MinValue;

        TimerRoutineRunner runner;
        Random rnd;
        IMyBroadcastListener listener;

        CubeViewer cv;

        int[,] radarState;

        public Program()
        {
            runner = new TimerRoutineRunner(this);
            lcdPart = GetBlocksOfType<IMyTextPanel>(null).Count > 0;
            if (lcdPart)
            {
                lcdPanel = GetBlockOfType(lcdPanel);
                lcdPanel.ContentType = ContentType.SCRIPT;
                lcdPanel.FontSize = 0.1f;
                listener = IGC.RegisterBroadcastListener("display");
                cv = new CubeViewer(-Math.PI / 8); //y angle

                runner.Start(DrawRoutine);
                Runtime.UpdateFrequency = UpdateFrequency.Update1;
            }
            else
            {
                cameras = GetBlocksOfType(cameras);


                foreach (var c in cameras)
                {
                    var p = c.Position;

                    if (p.X < minx)
                    {
                        minx = p.X;
                    }

                    if (p.Y < miny)
                    {
                        miny = p.Y;
                    }

                    if (p.X > maxx)
                    {
                        maxx = p.X;
                    }

                    if (p.Y > maxy)
                    {
                        maxy = p.Y;
                    }

                    c.EnableRaycast = true;                    
                }
                radarState = new int[maxx - minx + 1, maxy - miny + 1];
                Runtime.UpdateFrequency = UpdateFrequency.Update1;
            }
            rnd = new Random();
        }

        public void Save()
        {
        }


        Dictionary<Vector2I, int> cubes = new Dictionary<Vector2I, int>();
        int[][] cubesArr;


        IEnumerable<Double> DrawRoutine(ExitFlag exitFlag) {
            while (!exitFlag.ShouldExit)
            {
                if (cubesArr == null || cubesArr.Length != cubes.Count)
                {
                    cubesArr = new int[cubes.Count][];
                }

                int i = 0;
                foreach (var key in cubes.Keys)
                {
                    cubesArr[i++] = SEFix.arr(key.X, cubes[new Vector2I(key.X, key.Y)], key.Y);
                    yield return 0;
                }

                g.clear();
                Echo(cubesArr.Length.ToString());
                foreach (var delay in cv.draw(true, 80, 80, cubesArr, Math.PI / 32, 3, g))
                {
                    yield return delay;
                }
                g.paint();
                yield return 0.1;
            }
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (lcdPart)
            {
                if (g == null)
                {
                    g = new Graphics(160, 160, lcdPanel);
                    g.setBG(0, 0, 0);
                    g.clear();
                }
                while (listener.HasPendingMessage)
                {
                    var msg = listener.AcceptMessage();
                    String []arr = ((String)msg.Data).Split(':');

                    int x = int.Parse(arr[0]);
                    int y = int.Parse(arr[1]);
                    int c = int.Parse(arr[2]);

                    var key = new Vector2I(x, y);
                    if (c == -1)
                    {
                        if (cubes.ContainsKey(key))
                        {
                            cubes.Remove(key);
                        }
                    } else {
                        cubes[key] = c;
                    }
                }
            }
            else
            {
                var c = cameras[rnd.Next(cameras.Count)];

                var x = (c.Position.X - minx) - (maxx - minx) / 2;
                var y = (c.Position.Y - miny) -(maxy - miny) / 2;
                    
                if (c.CanScan(10000))
                {
                    var t = c.Raycast(10000);
                    int color = 0;
                    if (t.HitPosition.HasValue)
                    {
                        var len = (t.HitPosition.Value - c.GetPosition()).Length() / 5;
                        Echo(len.ToString());
                        if (len < 20)
                        {
                            color = (int)Math.Min(20.0, Math.Max(0.0, len)) - 10;
                        } else
                        {
                            color = -1;
                        }
                        var msg = x + ":" + y + ":" + color;
                        IGC.SendBroadcastMessage("display", msg);
                    }
                }
            }

            runner.Run();
        }
    }
}
