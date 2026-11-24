using System;
using System.Collections.Generic;
using System.Text;
using AlgoLib.Data.Result_Objects;

namespace AlgoLib.Algorithms.Abstractions
{
    public interface IFusion
    {
        public AlgorithmResult FusionDelegate(Tuple<MahalanobisResult, CusumResult> combinedResult);
    }
}
