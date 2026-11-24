using System;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;

namespace AlgoLib.Data.Result_Objects
{
    public class KalmanFilterResult
    {
        public required Vector<double> ParameterZScores { get; init; }
        public required int[] IndicesMeasured { get; init; }
        public required double[] ParameterValues { get; init; }
        public required double DeltaT { get; init; }
    }
}
