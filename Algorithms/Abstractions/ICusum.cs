using System;
using System.Collections.Generic;
using System.Text;
using AlgoLib.Data.Result_Objects;

namespace AlgoLib.Algorithms.Abstractions
{
    public interface ICusum
    {
        public void Init(double[] slacks, double[] thresholds);

        /// <param name="kalmanResult">The result of the most recent step of the Kalman Filter</param>
        /// <returns>All parameters' new CUSUM values</returns>
        public double[] RecalculateCusums(KalmanFilterResult kalmanResult);

        /// <summary>
        /// Computes which CUSUM values this time round ended up above their thresholds
        /// </summary>
        public void ComputeAnomalies();

        public double ComputeCusumRaw();

        public CusumResult GetStepResult();
    }
}
