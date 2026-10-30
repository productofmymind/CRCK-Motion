// ycd <-> ycd.xml through the user's own CodeWalker install (CodeWalker.Core.dll is loaded from there, never shipped).
// Built on first use by CRCK Motion with the C# compiler that comes with Windows (.NET Framework 4).
//   ycdtool <CodeWalker folder> to-xml <in.ycd> <out.ycd.xml>
//   ycdtool <CodeWalker folder> to-ycd <in.ycd.xml> <out.ycd>
using System;
using System.IO;
using System.Linq;
using System.Reflection;

static class YcdTool
{
    static string folder;

    static int Main(string[] args)
    {
        if (args.Length != 4) { Console.Error.WriteLine("usage: ycdtool <CodeWalker folder> to-xml|to-ycd <in> <out>"); return 2; }
        folder = args[0];
        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            var path = Path.Combine(folder, new AssemblyName(e.Name).Name + ".dll");
            return File.Exists(path) ? Assembly.Load(File.ReadAllBytes(path)) : null;
        };
        try
        {
            // from bytes: .NET Framework refuses LoadFrom on folders it treats as remote (synced drives, downloads)
            var core = Assembly.Load(File.ReadAllBytes(Path.Combine(folder, "CodeWalker.Core.dll")));
            Func<string, Type> T = n => core.GetType("CodeWalker.GameFiles." + n, true);
            if (args[1] == "to-xml")
            {
                var ycd = Activator.CreateInstance(T("YcdFile"));
                var load = T("RpfFile").GetMethods(BindingFlags.Public | BindingFlags.Static)
                    .First(m => m.Name == "LoadResourceFile" && m.IsGenericMethod && m.GetParameters().Length == 3)
                    .MakeGenericMethod(T("YcdFile"));
                load.Invoke(null, new object[] { ycd, File.ReadAllBytes(args[2]), 46u });
                var xml = (string)T("YcdXml").GetMethod("GetXml", new[] { T("YcdFile") }).Invoke(null, new[] { ycd });
                File.WriteAllText(args[3], xml);
            }
            else if (args[1] == "to-ycd")
            {
                var doc = new System.Xml.XmlDocument();
                doc.LoadXml(File.ReadAllText(args[2]));
                var ycd = T("XmlYcd").GetMethods(BindingFlags.Public | BindingFlags.Static)
                    .First(m => m.Name == "GetYcd" && m.GetParameters()[0].ParameterType == typeof(System.Xml.XmlDocument))
                    .Invoke(null, new object[] { doc });
                var data = (byte[])T("YcdFile").GetMethod("Save", Type.EmptyTypes).Invoke(ycd, null);
                File.WriteAllBytes(args[3], data);
            }
            else { Console.Error.WriteLine("unknown command " + args[1]); return 2; }
            return 0;
        }
        catch (Exception ex)
        {
            while (ex is TargetInvocationException && ex.InnerException != null) ex = ex.InnerException;
            Console.Error.WriteLine(ex.GetType().Name + ": " + ex.Message);
            return 1;
        }
    }
}
