using System;
using System.Collections.Generic;
using System.Text;
using AlgoLib.Data.Result_Objects;
using MathNet.Numerics.LinearAlgebra;
using ParameterDataLib;

namespace AlgoLib.Algorithms.Abstractions
{
    public interface IKalmanFilter
    {
        public void Init(Vector<double> qc, Matrix<double> r0, double forgettingFactor);

        public void Predict(ParameterData[] measurements);

        public void ComputeInnovation();

        public void Update();

        public Vector<double>? ComputeZScores();

        public void SageHusa();

        public KalmanFilterResult? GetStepResult();
    }
}
