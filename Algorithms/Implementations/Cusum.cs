using AlgoLib.Algorithms.Abstractions;
using AlgoLib.Data.Result_Objects;
using ThpUtils.Naming;

namespace AlgoLib.Algorithms.Implementations
{
    public class Cusum : ICusum
    {
        private enum Stage
        {
            None,
            Recalculated,
            AnomaliesComputed,
            RawComputed
        }

        private readonly string[] _parameterNames;

        // ---- State carried across steps, reset by Init ----
        private readonly double[] _positiveCusums;
        private readonly double[] _negativeCusums;

        // ---- Baseline, set by Init ----
        private bool _isInitialized;
        private double[] _slacks = null!;      // K per parameter
        private double[] _thresholds = null!;  // h per parameter

        // ---- Current step only ----
        private Stage _stage;
        private KalmanFilterResult? _kalmanResult;
        private readonly List<string> _anomalousNames = [];
        private readonly List<double> _anomalousValues = [];
        private readonly List<double> _excesses = [];
        private double _cusumRaw;

        public Cusum(NamingData namingData)
        {
            _parameterNames = namingData.ParameterNames;

            _positiveCusums = new double[_parameterNames.Length];
            _negativeCusums = new double[_parameterNames.Length];
        }

        /// <summary>Sets the baseline values and resets the CUSUM as if it never ran.</summary>
        /// <param name="slacks">Slack (K) per parameter, in units of standard deviations.</param>
        /// <param name="thresholds">Decision threshold (h) per parameter, in units of standard deviations.</param>
        public void Init(double[] slacks, double[] thresholds)
        {
            _slacks = (double[])slacks.Clone();
            _thresholds = (double[])thresholds.Clone();

            Array.Clear(_positiveCusums);
            Array.Clear(_negativeCusums);
            ClearStep();

            _isInitialized = true;
        }

        public double[] RecalculateCusums(KalmanFilterResult kalmanResult)
        {
            EnsureInitialized();

            ClearStep();
            _kalmanResult = kalmanResult;

            foreach (int i in kalmanResult.IndicesMeasured)
            {
                double zScore = kalmanResult.ParameterZScores[i];
                double k = _slacks[i];

                _positiveCusums[i] = Math.Max(0.0, _positiveCusums[i] + zScore - k);
                _negativeCusums[i] = Math.Max(0.0, _negativeCusums[i] - zScore - k);
            }

            double[] result = new double[_parameterNames.Length];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = Math.Max(_positiveCusums[i], _negativeCusums[i]);
            }

            _stage = Stage.Recalculated;
            return result;
        }

        public void ComputeAnomalies()
        {
            KalmanFilterResult kalmanResult = Require(Stage.Recalculated, nameof(RecalculateCusums));

            _anomalousNames.Clear();
            _anomalousValues.Clear();
            _excesses.Clear();

            foreach (int i in kalmanResult.IndicesMeasured)
            {
                double ratio = Math.Max(_positiveCusums[i], _negativeCusums[i]) / _thresholds[i];

                if (ratio > 1.0)
                {
                    _excesses.Add(ratio - 1.0);
                    _anomalousNames.Add(_parameterNames[i]);
                    _anomalousValues.Add(kalmanResult.ParameterValues[i]);
                }
            }

            _stage = Stage.AnomaliesComputed;
        }

        public double ComputeCusumRaw()
        {
            Require(Stage.AnomaliesComputed, nameof(ComputeAnomalies));

            _cusumRaw = _excesses.Sum();

            _stage = Stage.RawComputed;

            return _cusumRaw;
        }

        public CusumResult GetStepResult()
        {
            Require(Stage.RawComputed, nameof(ComputeCusumRaw));

            return new CusumResult()
            {
                AnomalousParameterNames = [.. _anomalousNames],
                AnomalousParameterValues = [.. _anomalousValues],
                CusumRaw = _cusumRaw
            };
        }

        private void EnsureInitialized()
        {
            if (!_isInitialized)
                throw new InvalidOperationException($"{nameof(Init)} must be called first.");
        }

        private KalmanFilterResult Require(Stage required, string requiredStep)
        {
            EnsureInitialized();

            if (_stage < required)
                throw new InvalidOperationException($"{requiredStep} must be called first.");

            return _kalmanResult!;
        }

        private void ClearStep()
        {
            _stage = Stage.None;
            _kalmanResult = null;
            _anomalousNames.Clear();
            _anomalousValues.Clear();
            _excesses.Clear();
            _cusumRaw = 0.0;
        }
    }
}