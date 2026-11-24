using System;
using System.Collections.Generic;
using System.Text;
using AlgoLib.Data.Result_Objects;
using MathNet.Numerics.LinearAlgebra;

namespace AlgoLib.Algorithms.Abstractions
{
    public interface IMahalanobis
    {
        public void Init(double stepSize, Matrix<double> covarianceMatrix, double distanceSquaredThreshold);

        /// <param name="kalmanResult">The result at each step of the Kalman Filter</param>
        /// <returns>Mahalanobis Distance squared</returns>
        public double ComputeD2(KalmanFilterResult kalmanResult);

        /// <returns>Whether the computed distance squared is above its threshold</returns>
        public bool IsAnomaly();

        /// <summary>
        /// Intended to be called if IsAnomaly, to find which parameters caused the anomaly
        /// </summary>
        public void FindOutlierParameters();

        /// <returns>The result of this Mahalanobis Distance calculation</returns>
        public MahalanobisResult GetResult();
    }
}
