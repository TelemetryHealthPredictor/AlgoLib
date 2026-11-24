using System;
using System.Collections.Generic;
using System.Text;

namespace AlgoLib.Data.Result_Objects
{
    public class CusumResult
    {
        public required List<string> AnomalousParameterNames { get; set; }
        public required List<double> AnomalousParameterValues { get; set; }
        public required double CusumRaw { get; set; }
    }
}
