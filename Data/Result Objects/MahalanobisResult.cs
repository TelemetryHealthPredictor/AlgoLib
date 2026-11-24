using System;
using System.Collections.Generic;
using System.Text;

namespace AlgoLib.Data.Result_Objects
{
    public class MahalanobisResult
    {
        public required double DistanceSquared { get; init; }
        public required Dictionary<string, double> OutlierParameters { get; init; }
        public required double DeltaT { get; init; }
    }
}
