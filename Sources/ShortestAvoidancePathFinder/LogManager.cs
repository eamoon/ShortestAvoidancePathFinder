using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFinder
{
    public class LogManager
    {
        static object locker = new object();

        public static void Write(string log)
        {
            lock (locker)
            {
                string LogAddress = Environment.CurrentDirectory + "\\Log";
                if (!Directory.Exists(LogAddress + "\\PRG"))
                {
                    Directory.CreateDirectory(LogAddress + "\\PRG");
                }
                LogAddress = string.Concat(LogAddress, "\\PRG\\",
                 DateTime.Now.Year, '-', DateTime.Now.Month, '-',
                 DateTime.Now.Day, "_program.log");

                using (var sw = new StreamWriter(LogAddress, true))
                {
                    sw.WriteLine($"[{DateTime.Now}] --- {log}");
                }
            }
        }
    }
}
