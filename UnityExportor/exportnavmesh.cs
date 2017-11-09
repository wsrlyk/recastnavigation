using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class exportnavmesh : UnityEditor.EditorWindow
{
    [UnityEditor.MenuItem("customNav/window")]
    static void show()
    {
        UnityEditor.EditorWindow.GetWindow<exportnavmesh>().Show();
    }

    void OnGUI()
    {
        if (GUILayout.Button("gen navmesh JSON"))
        {
            string outstring = GenNavMesh(0);
            string outfile = Application.dataPath + "\\navinfo.json.txt";
            System.IO.File.WriteAllText(outfile, outstring);
        }
        if (GUILayout.Button("gen navmesh OBJ"))
        {
            string outstring = GenNavMesh(1);
            string outfile = Application.dataPath + "\\navinfo.obj";
            System.IO.File.WriteAllText(outfile, outstring);
        }
        if (GUILayout.Button("gen navmesh recast"))
        {
            string outstring = GenNavMesh(2);
            string outfile = Application.dataPath + "\\navinfo.txt";
            System.IO.File.WriteAllText(outfile, outstring);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="style">0 json 1 obj</param>
    /// <returns></returns>
    string GenNavMesh(int style)
    {
        System.Threading.Thread.CurrentThread.CurrentCulture = System.Globalization.CultureInfo.InvariantCulture;
        UnityEngine.AI.NavMeshTriangulation navtri = UnityEngine.AI.NavMesh.CalculateTriangulation();

        //{
        //    var obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        //    var mf = obj.GetComponent<MeshFilter>();
        //    Mesh m = new Mesh();
        //    m.vertices = navtri.vertices;
        //    m.triangles = navtri.indices;
        //    mf.mesh = m;
        //}




        Dictionary<int, int> indexmap = new Dictionary<int, int>();
        List<Vector3> repos = new List<Vector3>();
        for (int i = 0; i < navtri.vertices.Length; i++)
        {
            int ito = -1;
            for (int j = 0; j < repos.Count; j++)
            {
                if (Vector3.Distance(navtri.vertices[i], repos[j]) < 0.01)
                {
                    ito = j;
                    break;
                }
            }
            if (ito < 0)
            {
                indexmap[i] = repos.Count;
                repos.Add(navtri.vertices[i]);
            }
            else
            {
                indexmap[i] = ito;
            }
        }

        //关系是 index 公用的三角形表示他们共同组成多边形
        //多边形之间的连接用顶点位置识别
        List<int> polylast = new List<int>();
        List<int[]> polys = new List<int[]>();
        
        List<int> areas = new List<int>();

        int lastArea = int.MaxValue;
        for (int i = 0; i < navtri.indices.Length / 3; i++)
        {
            int i0 = navtri.indices[i * 3 + 0];
            int i1 = navtri.indices[i * 3 + 1];
            int i2 = navtri.indices[i * 3 + 2];

            if ((polylast.Contains(i0) || polylast.Contains(i1) || polylast.Contains(i2)) && lastArea == navtri.areas[i])
            {
                if (polylast.Contains(i0) == false)
                    polylast.Add(i0);
                if (polylast.Contains(i1) == false)
                    polylast.Add(i1);
                if (polylast.Contains(i2) == false)
                    polylast.Add(i2);
            }
            else
            {
                if (polylast.Count > 0)
                {
                    polys.Add(polylast.ToArray());
                    areas.Add(lastArea);
                }
                polylast.Clear();
                polylast.Add(i0);
                polylast.Add(i1);
                polylast.Add(i2);
                lastArea = navtri.areas[i];
            }
        }
        if (polylast.Count > 0)
            polys.Add(polylast.ToArray());

        System.Text.StringBuilder outnav = new System.Text.StringBuilder();
        if (style == 0)
        {
            outnav.Append("{\"v\":[\n");
            for (int i = 0; i < repos.Count; i++)
            {
                if (i > 0)
                    outnav.Append(",\n");

                outnav.Append("[").Append(repos[i].x).Append(",").Append(repos[i].y).Append(",").Append(repos[i].z).Append("]");
            }
            outnav.Append("\n],\"p\":[\n");

            for (int i = 0; i < polys.Count; i++)
            {
                string outs = indexmap[polys[i][0]].ToString();
                for (int j = 1; j < polys[i].Length; j++)
                {
                    outs += "," + indexmap[polys[i][j]];
                }

                if (i > 0)
                    outnav.Append(",\n");

                outnav.Append("[").Append(outs).Append("]");
            }
            outnav.Append("\n]}");
        }
        else if (style == 1)
        {
            for (int i = 0; i < repos.Count; i++)
            {//unity 对obj 做了 x轴 -1
                outnav.Append("v ").Append(repos[i].x * -1).Append(" ").Append(repos[i].y).Append(" ").Append(repos[i].z).Append("\r\n");
            }
            outnav.Append("\r\n");
            for (int i = 0; i < polys.Count; i++)
            {
                outnav.Append("f");
                //逆向
                for (int j = polys[i].Length - 1; j >= 0; j--)
                {
                    outnav.Append(" ").Append(indexmap[polys[i][j]] + 1);
                }

                outnav.Append("\r\n");
            }
        }
        else if (style == 2)
        {
            outnav.Append("v\n");
            for (int i = 0; i < repos.Count; i++)
            {
                outnav.Append(repos[i].x).Append(" ").Append(repos[i].y).Append(" ").Append(repos[i].z).Append("\n");
            }

            int maxVertPerPoly = 0;
            outnav.Append("p\n");
            for (int i = 0; i < polys.Count; i++)
            {
                outnav.Append(indexmap[polys[i][0]]);
                for (int j = 1; j < polys[i].Length; j++)
                {
                    outnav.Append(" ").Append(indexmap[polys[i][j]]);
                }

                outnav.Append("\n");

                maxVertPerPoly = System.Math.Max(maxVertPerPoly, polys[i].Length);
            }

            //outnav.Append("a\n");
            //for (int i = 0; i < areas.Count; ++i)
            //{
            //    outnav.Append(areas[i]).Append("\n");
            //}

            outnav.Append("nvp\n").Append(maxVertPerPoly).Append("\n");
        }

        return outnav.ToString();
    }
}
