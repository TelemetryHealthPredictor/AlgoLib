using System;
using System.Collections.Generic;
using System.Text;

namespace AlgoLib.Data.Result_Objects
{
    public class AlgorithmResult
    {
        public required int HealthScore { get; init; }
        public required Dictionary<string, double>? MahalanobisAnomalies { get; init; }
        public required Dictionary<string, double>? CusumAnomalies { get; init; }
        public required Boolean InAlert { get; init; }
    }
}
