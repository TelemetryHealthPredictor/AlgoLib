using System;
using System.Collections.Generic;
using System.Text;
using AlgoLib.Data.Result_Objects;

namespace AlgoLib.Algorithms.Abstractions
{
    public interface IFusion
    {
        public void Init
        (
            double[] cusumLevels,
            double[] d2Levels,
            double d2Weight,
            double tDecay,
            double hysteresisEntryValue,
            double hysteresisExitValue
        );

        public AlgorithmResult FusionDelegate(Tuple<MahalanobisResult, CusumResult> combinedResult);
    }
}
