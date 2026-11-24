using AlgoLib.Algorithms.Abstractions;
using AlgoLib.Data.Result_Objects;
using AlgoLib.Utilities;

namespace TelemetryHealthPredictor.Algorithm.Delegates.Implementations
{
    public class Fusion : IFusion
    {
        private const double MaxScore = 100.0;
        private const double MinScore = 1.0;

        private readonly AnchorLadder _cusumLadder;
        private readonly AnchorLadder _d2Ladder;

        private readonly double _d2Weight;
        private readonly double _tDecay;
        private readonly double _hysteresisEntryValue, _hysteresisExitValue;

        // State carried between frames.
        private double _decayedHealthScore = MaxScore;
        private bool _inAlert;

        /// <param name="cusumLevels">
        /// A list of levels for CUSUM Raw.
        /// Each one represents how often it will be at that value or higher.
        /// Index 0 - the value that is exceeded 100% of the time in the dataset.
        /// Index 1 - the value that is exceeded 10% of the time in the dataset, etc.
        /// </param>
        /// <param name="d2Levels">
        /// A list of levels for Mahalanobis Distance squared.
        /// Each one represents how often it will be at that value or higher.
        /// Index 0 - the value that is exceeded 100% of the time in the dataset.
        /// Index 1 - the value that is exceeded 10% of the time in the dataset, etc.
        /// Must be the same length as cusumLevels, with the same index meaning the same frequency.
        /// </param>
        /// <param name="d2Weight">
        /// How much weight is given to Mahalanobis Distance in the health score calculation.
        /// 1 - d2Weight = the weight given to CUSUM.
        /// </param>
        /// <param name="tDecay">
        /// Recovery time constant of the health score, in seconds.
        /// The score drops instantly when the pre-decay score drops, but recovers
        /// toward 100 gradually: the remaining gap to 100 shrinks by a factor of
        /// exp(-DeltaT / tDecay) per frame. Zero or less disables decay.
        /// </param>
        /// <param name="hysteresisEntryValue">
        /// Health score needs to go below EntryValue to be considered alert worthy.
        /// </param>
        /// <param name="hysteresisExitValue">
        /// Health score needs to go back above ExitValue to stop constituting an alert.
        /// </param>
        public Fusion
        (
            double[] cusumLevels,
            double[] d2Levels,
            double d2Weight,
            double tDecay,
            double hysteresisEntryValue,
            double hysteresisExitValue
        )
        {
            _cusumLadder = new AnchorLadder(cusumLevels);
            _d2Ladder = new AnchorLadder(d2Levels);
            _d2Weight = d2Weight;
            _tDecay = tDecay;
            _hysteresisEntryValue = hysteresisEntryValue;
            _hysteresisExitValue = hysteresisExitValue;
        }

        public AlgorithmResult FusionDelegate(Tuple<MahalanobisResult, CusumResult> combinedResult)
        {
            MahalanobisResult mahalanobis = combinedResult.Item1;
            CusumResult cusum = combinedResult.Item2;

            double standardizedMahalanobis = Standardize(_d2Ladder, mahalanobis.DistanceSquared);
            double standardizedCusum = Standardize(_cusumLadder, cusum.CusumRaw);

            double preDecayScore = _d2Weight * standardizedMahalanobis + (1 - _d2Weight) * standardizedCusum;

            UpdateHealthScore(preDecayScore, mahalanobis.DeltaT);

            return new AlgorithmResult()
            {
                HealthScore = (int)Math.Round(_decayedHealthScore),
                MahalanobisAnomalies = mahalanobis.OutlierParameters,
                CusumAnomalies = cusum.AnomalousParameterNames
                    .Zip(cusum.AnomalousParameterValues)
                    .ToDictionary(pair => pair.First, pair => pair.Second),
                InAlert = _inAlert
            };
        }

        /// <summary>
        /// Maps a raw value to a 1-100 score: level 0 maps to 100, the top level maps to 1, linear in between.
        /// </summary>
        private static double Standardize(AnchorLadder ladder, double value)
        {
            double fraction = ladder.GetLevel(value) / ladder.LastIndex;

            return MaxScore - (MaxScore - MinScore) * fraction;
        }

        /// <summary>
        /// Applies decay (drops are immediate, recovery is gradual toward 100),
        /// then updates the alert state using hysteresis on the unrounded score.
        /// </summary>
        private void UpdateHealthScore(double preDecayScore, double deltaT)
        {
            double score = preDecayScore;

            if (_tDecay > 0)
            {
                double dt = Math.Max(0, deltaT);
                double recovering = MaxScore - (MaxScore - _decayedHealthScore) * Math.Exp(-dt / _tDecay);
                score = Math.Min(preDecayScore, recovering);
            }

            _decayedHealthScore = score;

            if (!_inAlert && score < _hysteresisEntryValue)
            {
                _inAlert = true;
            }
            else if (_inAlert && score > _hysteresisExitValue)
            {
                _inAlert = false;
            }
        }
    }
}