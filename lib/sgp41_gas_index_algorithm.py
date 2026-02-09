"""
Sensirion Gas Index Algorithm - MicroPython Port
Copyright (c) 2022, Sensirion AG
BSD-3-Clause License

Ported from C version 3.2.0

Variable mapping (C struct field -> Python attribute):
  mAlgorithm_Type                                    -> _algorithm_type
  mSamplingInterval                                  -> _sampling_interval
  mIndex_Offset                                      -> _index_offset
  mSraw_Minimum                                      -> _sraw_minimum
  mGating_Max_Duration_Minutes                       -> _gating_max_duration_minutes
  mInit_Duration_Mean                                -> _init_duration_mean
  mInit_Duration_Variance                            -> _init_duration_variance
  mGating_Threshold                                  -> _gating_threshold
  mIndex_Gain                                        -> _index_gain
  mTau_Mean_Hours                                    -> _tau_mean_hours
  mTau_Variance_Hours                                -> _tau_variance_hours
  mSraw_Std_Initial                                  -> _sraw_std_initial
  mUptime                                            -> _uptime
  mSraw                                              -> _sraw
  mGas_Index                                         -> _gas_index
  m_Mean_Variance_Estimator___Initialized            -> _mve_initialized
  m_Mean_Variance_Estimator___Mean                   -> _mve_mean
  m_Mean_Variance_Estimator___Sraw_Offset            -> _mve_sraw_offset
  m_Mean_Variance_Estimator___Std                    -> _mve_std
  m_Mean_Variance_Estimator___Gamma_Mean             -> _mve_gamma_mean
  m_Mean_Variance_Estimator___Gamma_Variance         -> _mve_gamma_variance
  m_Mean_Variance_Estimator___Gamma_Initial_Mean     -> _mve_gamma_initial_mean
  m_Mean_Variance_Estimator___Gamma_Initial_Variance -> _mve_gamma_initial_variance
  m_Mean_Variance_Estimator__Gamma_Mean              -> _mve_gc_gamma_mean
  m_Mean_Variance_Estimator__Gamma_Variance          -> _mve_gc_gamma_variance
  m_Mean_Variance_Estimator___Uptime_Gamma           -> _mve_uptime_gamma
  m_Mean_Variance_Estimator___Uptime_Gating          -> _mve_uptime_gating
  m_Mean_Variance_Estimator___Gating_Duration_Minutes-> _mve_gating_duration_minutes
  m_Mean_Variance_Estimator___Sigmoid__K             -> _mve_sigmoid_k
  m_Mean_Variance_Estimator___Sigmoid__X0            -> _mve_sigmoid_x0
  m_Mox_Model__Sraw_Std                              -> _mox_sraw_std
  m_Mox_Model__Sraw_Mean                             -> _mox_sraw_mean
  m_Sigmoid_Scaled__K                                -> _sigmoid_scaled_k
  m_Sigmoid_Scaled__X0                               -> _sigmoid_scaled_x0
  m_Sigmoid_Scaled__Offset_Default                   -> _sigmoid_scaled_offset_default
  m_Adaptive_Lowpass__A1                             -> _lp_a1
  m_Adaptive_Lowpass__A2                             -> _lp_a2
  m_Adaptive_Lowpass___Initialized                   -> _lp_initialized
  m_Adaptive_Lowpass___X1                            -> _lp_x1
  m_Adaptive_Lowpass___X2                            -> _lp_x2
  m_Adaptive_Lowpass___X3                            -> _lp_x3
"""

import math


class GasIndexAlgorithm:
    """Gas Index Algorithm for VOC and NOx index calculation from SGP4x raw signals."""

    # Algorithm types
    ALGORITHM_TYPE_VOC = 0
    ALGORITHM_TYPE_NOX = 1

    # Constants (from #define in header)
    _DEFAULT_SAMPLING_INTERVAL = 1.0
    _INITIAL_BLACKOUT = 45.0
    _INDEX_GAIN = 230.0
    _SRAW_STD_INITIAL = 50.0
    _SRAW_STD_BONUS_VOC = 220.0
    _SRAW_STD_NOX = 2000.0
    _TAU_MEAN_HOURS = 12.0
    _TAU_VARIANCE_HOURS = 12.0
    _TAU_INITIAL_MEAN_VOC = 20.0
    _TAU_INITIAL_MEAN_NOX = 1200.0
    _INIT_DURATION_MEAN_VOC = 3600.0 * 0.75
    _INIT_DURATION_MEAN_NOX = 3600.0 * 4.75
    _INIT_TRANSITION_MEAN = 0.01
    _TAU_INITIAL_VARIANCE = 2500.0
    _INIT_DURATION_VARIANCE_VOC = 3600.0 * 1.45
    _INIT_DURATION_VARIANCE_NOX = 3600.0 * 5.70
    _INIT_TRANSITION_VARIANCE = 0.01
    _GATING_THRESHOLD_VOC = 340.0
    _GATING_THRESHOLD_NOX = 30.0
    _GATING_THRESHOLD_INITIAL = 510.0
    _GATING_THRESHOLD_TRANSITION = 0.09
    _GATING_VOC_MAX_DURATION_MINUTES = 60.0 * 3.0
    _GATING_NOX_MAX_DURATION_MINUTES = 60.0 * 12.0
    _GATING_MAX_RATIO = 0.3
    _SIGMOID_L = 500.0
    _SIGMOID_K_VOC = -0.0065
    _SIGMOID_X0_VOC = 213.0
    _SIGMOID_K_NOX = -0.0101
    _SIGMOID_X0_NOX = 614.0
    _VOC_INDEX_OFFSET_DEFAULT = 100.0
    _NOX_INDEX_OFFSET_DEFAULT = 1.0
    _LP_TAU_FAST = 20.0
    _LP_TAU_SLOW = 500.0
    _LP_ALPHA = -0.2
    _VOC_SRAW_MINIMUM = 20000
    _NOX_SRAW_MINIMUM = 10000
    _PERSISTENCE_UPTIME_GAMMA = 3.0 * 3600.0
    _TUNING_INDEX_OFFSET_MIN = 1
    _TUNING_INDEX_OFFSET_MAX = 250
    _TUNING_LEARNING_TIME_OFFSET_HOURS_MIN = 1
    _TUNING_LEARNING_TIME_OFFSET_HOURS_MAX = 1000
    _TUNING_LEARNING_TIME_GAIN_HOURS_MIN = 1
    _TUNING_LEARNING_TIME_GAIN_HOURS_MAX = 1000
    _TUNING_GATING_MAX_DURATION_MINUTES_MIN = 0
    _TUNING_GATING_MAX_DURATION_MINUTES_MAX = 3000
    _TUNING_STD_INITIAL_MIN = 10
    _TUNING_STD_INITIAL_MAX = 5000
    _TUNING_GAIN_FACTOR_MIN = 1
    _TUNING_GAIN_FACTOR_MAX = 1000
    _MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING = 64.0
    _MEAN_VARIANCE_ESTIMATOR_ADDITIONAL_GAMMA_MEAN_SCALING = 8.0
    _MEAN_VARIANCE_ESTIMATOR_FIX16_MAX = 32767.0

    def __init__(self, algorithm_type, sampling_interval=None):
        """
        Initialize the gas index algorithm.

        Args:
            algorithm_type: ALGORITHM_TYPE_VOC (0) or ALGORITHM_TYPE_NOX (1)
            sampling_interval: Sampling interval in seconds (default 1.0).
                               Tested for 1s and 10s.
        """
        if sampling_interval is None:
            sampling_interval = self._DEFAULT_SAMPLING_INTERVAL

        self._algorithm_type = int(algorithm_type)
        self._sampling_interval = float(sampling_interval)

        # C: GasIndexAlgorithm_init_with_sampling_interval
        if algorithm_type == self.ALGORITHM_TYPE_NOX:
            self._index_offset = self._NOX_INDEX_OFFSET_DEFAULT
            self._sraw_minimum = self._NOX_SRAW_MINIMUM
            self._gating_max_duration_minutes = self._GATING_NOX_MAX_DURATION_MINUTES
            self._init_duration_mean = self._INIT_DURATION_MEAN_NOX
            self._init_duration_variance = self._INIT_DURATION_VARIANCE_NOX
            self._gating_threshold = self._GATING_THRESHOLD_NOX
        else:
            self._index_offset = self._VOC_INDEX_OFFSET_DEFAULT
            self._sraw_minimum = self._VOC_SRAW_MINIMUM
            self._gating_max_duration_minutes = self._GATING_VOC_MAX_DURATION_MINUTES
            self._init_duration_mean = self._INIT_DURATION_MEAN_VOC
            self._init_duration_variance = self._INIT_DURATION_VARIANCE_VOC
            self._gating_threshold = self._GATING_THRESHOLD_VOC

        self._index_gain = self._INDEX_GAIN
        self._tau_mean_hours = self._TAU_MEAN_HOURS
        self._tau_variance_hours = self._TAU_VARIANCE_HOURS
        self._sraw_std_initial = self._SRAW_STD_INITIAL

        # State variables (initialized by reset -> _init_instances)
        self._uptime = 0.0
        self._sraw = 0.0
        self._gas_index = 0.0

        # Mean variance estimator state
        self._mve_initialized = False
        self._mve_mean = 0.0
        self._mve_sraw_offset = 0.0
        self._mve_std = 0.0
        self._mve_gamma_mean = 0.0
        self._mve_gamma_variance = 0.0
        self._mve_gamma_initial_mean = 0.0
        self._mve_gamma_initial_variance = 0.0
        self._mve_gc_gamma_mean = 0.0
        self._mve_gc_gamma_variance = 0.0
        self._mve_uptime_gamma = 0.0
        self._mve_uptime_gating = 0.0
        self._mve_gating_duration_minutes = 0.0
        self._mve_sigmoid_k = 0.0
        self._mve_sigmoid_x0 = 0.0

        # Mox model state
        self._mox_sraw_std = 0.0
        self._mox_sraw_mean = 0.0

        # Sigmoid scaled state
        self._sigmoid_scaled_k = 0.0
        self._sigmoid_scaled_x0 = 0.0
        self._sigmoid_scaled_offset_default = 0.0

        # Adaptive lowpass state
        self._lp_a1 = 0.0
        self._lp_a2 = 0.0
        self._lp_initialized = False
        self._lp_x1 = 0.0
        self._lp_x2 = 0.0
        self._lp_x3 = 0.0

        self.reset()

    # ----------------------------------------------------------------
    # C: GasIndexAlgorithm_reset
    # ----------------------------------------------------------------
    def reset(self):
        """Reset internal states. Previously set tuning parameters are preserved."""
        self._uptime = 0.0
        self._sraw = 0.0
        self._gas_index = 0.0
        self._init_instances()

    # ----------------------------------------------------------------
    # C: GasIndexAlgorithm__init_instances
    # ----------------------------------------------------------------
    def _init_instances(self):
        self._mean_variance_estimator_set_parameters()
        self._mox_model_set_parameters(
            self._mean_variance_estimator_get_std(),
            self._mean_variance_estimator_get_mean(),
        )
        if self._algorithm_type == self.ALGORITHM_TYPE_NOX:
            self._sigmoid_scaled_set_parameters(
                self._SIGMOID_X0_NOX,
                self._SIGMOID_K_NOX,
                self._NOX_INDEX_OFFSET_DEFAULT,
            )
        else:
            self._sigmoid_scaled_set_parameters(
                self._SIGMOID_X0_VOC,
                self._SIGMOID_K_VOC,
                self._VOC_INDEX_OFFSET_DEFAULT,
            )
        self._adaptive_lowpass_set_parameters()

    # ----------------------------------------------------------------
    # C: GasIndexAlgorithm_get_sampling_interval
    # ----------------------------------------------------------------
    def get_sampling_interval(self):
        """Get the sampling interval in seconds."""
        return self._sampling_interval

    # ----------------------------------------------------------------
    # C: GasIndexAlgorithm_get_states
    # ----------------------------------------------------------------
    def get_states(self):
        """
        Get current algorithm states for persistence.
        NOTE: Only valid for VOC after at least 3 hours of continuous operation.

        Returns:
            tuple: (state0, state1)
        """
        state0 = self._mean_variance_estimator_get_mean()
        state1 = self._mean_variance_estimator_get_std()
        return (state0, state1)

    # ----------------------------------------------------------------
    # C: GasIndexAlgorithm_set_states
    # ----------------------------------------------------------------
    def set_states(self, state0, state1):
        """
        Restore previously saved states to skip initial learning phase.
        NOTE: Only for VOC. Do not use after interruptions > 10 minutes.

        Args:
            state0: Previously saved state0
            state1: Previously saved state1
        """
        self._mean_variance_estimator_set_states(
            state0, state1, self._PERSISTENCE_UPTIME_GAMMA
        )
        self._mox_model_set_parameters(
            self._mean_variance_estimator_get_std(),
            self._mean_variance_estimator_get_mean(),
        )
        self._sraw = state0

    # ----------------------------------------------------------------
    # C: GasIndexAlgorithm_set_tuning_parameters
    # ----------------------------------------------------------------
    def set_tuning_parameters(
        self,
        index_offset,
        learning_time_offset_hours,
        learning_time_gain_hours,
        gating_max_duration_minutes,
        std_initial,
        gain_factor,
    ):
        """Set parameters to customize the algorithm."""
        self._index_offset = float(index_offset)
        self._tau_mean_hours = float(learning_time_offset_hours)
        self._tau_variance_hours = float(learning_time_gain_hours)
        self._gating_max_duration_minutes = float(gating_max_duration_minutes)
        self._sraw_std_initial = float(std_initial)
        self._index_gain = float(gain_factor)
        self._init_instances()

    # ----------------------------------------------------------------
    # C: GasIndexAlgorithm_get_tuning_parameters
    # ----------------------------------------------------------------
    def get_tuning_parameters(self):
        """
        Get current tuning parameters.

        Returns:
            tuple: (index_offset, learning_time_offset_hours,
                    learning_time_gain_hours, gating_max_duration_minutes,
                    std_initial, gain_factor)
        """
        return (
            int(self._index_offset),
            int(self._tau_mean_hours),
            int(self._tau_variance_hours),
            int(self._gating_max_duration_minutes),
            int(self._sraw_std_initial),
            int(self._index_gain),
        )

    # ----------------------------------------------------------------
    # C: GasIndexAlgorithm_process
    # ----------------------------------------------------------------
    def process(self, sraw):
        """
        Calculate the gas index from the raw sensor value.

        Args:
            sraw: Raw value from the SGP4x sensor (int)

        Returns:
            int: Gas index value (0 during blackout, 1..500 afterwards)
        """
        if self._uptime <= self._INITIAL_BLACKOUT:
            self._uptime = self._uptime + self._sampling_interval
        else:
            if (sraw > 0) and (sraw < 65000):
                if sraw < (self._sraw_minimum + 1):
                    sraw = self._sraw_minimum + 1
                elif sraw > (self._sraw_minimum + 32767):
                    sraw = self._sraw_minimum + 32767
                self._sraw = float(sraw - self._sraw_minimum)

            if (
                self._algorithm_type == self.ALGORITHM_TYPE_VOC
                or self._mean_variance_estimator_is_initialized()
            ):
                self._gas_index = self._mox_model_process(self._sraw)
                self._gas_index = self._sigmoid_scaled_process(self._gas_index)
            else:
                self._gas_index = self._index_offset

            self._gas_index = self._adaptive_lowpass_process(self._gas_index)
            if self._gas_index < 0.5:
                self._gas_index = 0.5

            if self._sraw > 0.0:
                self._mean_variance_estimator_process(self._sraw)
                self._mox_model_set_parameters(
                    self._mean_variance_estimator_get_std(),
                    self._mean_variance_estimator_get_mean(),
                )

        return int(self._gas_index + 0.5)

    # ================================================================
    # Mean Variance Estimator
    # ================================================================

    # C: GasIndexAlgorithm__mean_variance_estimator__set_parameters
    def _mean_variance_estimator_set_parameters(self):
        self._mve_initialized = False
        self._mve_mean = 0.0
        self._mve_sraw_offset = 0.0
        self._mve_std = self._sraw_std_initial
        self._mve_gamma_mean = (
            (
                self._MEAN_VARIANCE_ESTIMATOR_ADDITIONAL_GAMMA_MEAN_SCALING
                * self._MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING
            )
            * (self._sampling_interval / 3600.0)
        ) / (self._tau_mean_hours + (self._sampling_interval / 3600.0))
        self._mve_gamma_variance = (
            self._MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING
            * (self._sampling_interval / 3600.0)
        ) / (self._tau_variance_hours + (self._sampling_interval / 3600.0))
        if self._algorithm_type == self.ALGORITHM_TYPE_NOX:
            self._mve_gamma_initial_mean = (
                (
                    self._MEAN_VARIANCE_ESTIMATOR_ADDITIONAL_GAMMA_MEAN_SCALING
                    * self._MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING
                )
                * self._sampling_interval
            ) / (self._TAU_INITIAL_MEAN_NOX + self._sampling_interval)
        else:
            self._mve_gamma_initial_mean = (
                (
                    self._MEAN_VARIANCE_ESTIMATOR_ADDITIONAL_GAMMA_MEAN_SCALING
                    * self._MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING
                )
                * self._sampling_interval
            ) / (self._TAU_INITIAL_MEAN_VOC + self._sampling_interval)
        self._mve_gamma_initial_variance = (
            self._MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING
            * self._sampling_interval
        ) / (self._TAU_INITIAL_VARIANCE + self._sampling_interval)
        self._mve_gc_gamma_mean = 0.0
        self._mve_gc_gamma_variance = 0.0
        self._mve_uptime_gamma = 0.0
        self._mve_uptime_gating = 0.0
        self._mve_gating_duration_minutes = 0.0

    # C: GasIndexAlgorithm__mean_variance_estimator__set_states
    def _mean_variance_estimator_set_states(self, mean, std, uptime_gamma):
        self._mve_mean = mean
        self._mve_std = std
        self._mve_uptime_gamma = uptime_gamma
        self._mve_initialized = True

    # C: GasIndexAlgorithm__mean_variance_estimator__get_std
    def _mean_variance_estimator_get_std(self):
        return self._mve_std

    # C: GasIndexAlgorithm__mean_variance_estimator__get_mean
    def _mean_variance_estimator_get_mean(self):
        return self._mve_mean + self._mve_sraw_offset

    # C: GasIndexAlgorithm__mean_variance_estimator__is_initialized
    def _mean_variance_estimator_is_initialized(self):
        return self._mve_initialized

    # C: GasIndexAlgorithm__mean_variance_estimator___calculate_gamma
    def _mean_variance_estimator_calculate_gamma(self):
        uptime_limit = (
            self._MEAN_VARIANCE_ESTIMATOR_FIX16_MAX - self._sampling_interval
        )

        if self._mve_uptime_gamma < uptime_limit:
            self._mve_uptime_gamma = (
                self._mve_uptime_gamma + self._sampling_interval
            )

        if self._mve_uptime_gating < uptime_limit:
            self._mve_uptime_gating = (
                self._mve_uptime_gating + self._sampling_interval
            )

        self._mean_variance_estimator_sigmoid_set_parameters(
            self._init_duration_mean, self._INIT_TRANSITION_MEAN
        )
        sigmoid_gamma_mean = self._mean_variance_estimator_sigmoid_process(
            self._mve_uptime_gamma
        )

        gamma_mean = self._mve_gamma_mean + (
            (self._mve_gamma_initial_mean - self._mve_gamma_mean)
            * sigmoid_gamma_mean
        )

        gating_threshold_mean = self._gating_threshold + (
            (self._GATING_THRESHOLD_INITIAL - self._gating_threshold)
            * self._mean_variance_estimator_sigmoid_process(
                self._mve_uptime_gating
            )
        )

        self._mean_variance_estimator_sigmoid_set_parameters(
            gating_threshold_mean, self._GATING_THRESHOLD_TRANSITION
        )
        sigmoid_gating_mean = self._mean_variance_estimator_sigmoid_process(
            self._gas_index
        )

        self._mve_gc_gamma_mean = sigmoid_gating_mean * gamma_mean

        self._mean_variance_estimator_sigmoid_set_parameters(
            self._init_duration_variance, self._INIT_TRANSITION_VARIANCE
        )
        sigmoid_gamma_variance = self._mean_variance_estimator_sigmoid_process(
            self._mve_uptime_gamma
        )

        gamma_variance = self._mve_gamma_variance + (
            (self._mve_gamma_initial_variance - self._mve_gamma_variance)
            * (sigmoid_gamma_variance - sigmoid_gamma_mean)
        )

        gating_threshold_variance = self._gating_threshold + (
            (self._GATING_THRESHOLD_INITIAL - self._gating_threshold)
            * self._mean_variance_estimator_sigmoid_process(
                self._mve_uptime_gating
            )
        )

        self._mean_variance_estimator_sigmoid_set_parameters(
            gating_threshold_variance, self._GATING_THRESHOLD_TRANSITION
        )
        sigmoid_gating_variance = (
            self._mean_variance_estimator_sigmoid_process(self._gas_index)
        )

        self._mve_gc_gamma_variance = sigmoid_gating_variance * gamma_variance

        self._mve_gating_duration_minutes = (
            self._mve_gating_duration_minutes
            + (
                (self._sampling_interval / 60.0)
                * (
                    ((1.0 - sigmoid_gating_mean) * (1.0 + self._GATING_MAX_RATIO))
                    - self._GATING_MAX_RATIO
                )
            )
        )

        if self._mve_gating_duration_minutes < 0.0:
            self._mve_gating_duration_minutes = 0.0

        if self._mve_gating_duration_minutes > self._gating_max_duration_minutes:
            self._mve_uptime_gating = 0.0

    # C: GasIndexAlgorithm__mean_variance_estimator__process
    def _mean_variance_estimator_process(self, sraw):
        if self._mve_initialized == False:
            self._mve_initialized = True
            self._mve_sraw_offset = sraw
            self._mve_mean = 0.0
        else:
            if (self._mve_mean >= 100.0) or (self._mve_mean <= -100.0):
                self._mve_sraw_offset = self._mve_sraw_offset + self._mve_mean
                self._mve_mean = 0.0

            sraw = sraw - self._mve_sraw_offset
            self._mean_variance_estimator_calculate_gamma()

            delta_sgp = (sraw - self._mve_mean) / (
                self._MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING
            )

            if delta_sgp < 0.0:
                c = self._mve_std - delta_sgp
            else:
                c = self._mve_std + delta_sgp

            additional_scaling = 1.0
            if c > 1440.0:
                additional_scaling = (c / 1440.0) * (c / 1440.0)

            self._mve_std = math.sqrt(
                additional_scaling
                * (
                    self._MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING
                    - self._mve_gc_gamma_variance
                )
            ) * math.sqrt(
                (
                    self._mve_std
                    * (
                        self._mve_std
                        / (
                            self._MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING
                            * additional_scaling
                        )
                    )
                )
                + (
                    (
                        (self._mve_gc_gamma_variance * delta_sgp)
                        / additional_scaling
                    )
                    * delta_sgp
                )
            )

            self._mve_mean = self._mve_mean + (
                (self._mve_gc_gamma_mean * delta_sgp)
                / self._MEAN_VARIANCE_ESTIMATOR_ADDITIONAL_GAMMA_MEAN_SCALING
            )

    # ================================================================
    # Mean Variance Estimator - Sigmoid sub-component
    # ================================================================

    # C: GasIndexAlgorithm__mean_variance_estimator___sigmoid__set_parameters
    def _mean_variance_estimator_sigmoid_set_parameters(self, x0, k):
        self._mve_sigmoid_k = k
        self._mve_sigmoid_x0 = x0

    # C: GasIndexAlgorithm__mean_variance_estimator___sigmoid__process
    def _mean_variance_estimator_sigmoid_process(self, sample):
        x = self._mve_sigmoid_k * (sample - self._mve_sigmoid_x0)
        if x < -50.0:
            return 1.0
        elif x > 50.0:
            return 0.0
        else:
            return 1.0 / (1.0 + math.exp(x))

    # ================================================================
    # Mox Model
    # ================================================================

    # C: GasIndexAlgorithm__mox_model__set_parameters
    def _mox_model_set_parameters(self, sraw_std, sraw_mean):
        self._mox_sraw_std = sraw_std
        self._mox_sraw_mean = sraw_mean

    # C: GasIndexAlgorithm__mox_model__process
    def _mox_model_process(self, sraw):
        if self._algorithm_type == self.ALGORITHM_TYPE_NOX:
            return (
                ((sraw - self._mox_sraw_mean) / self._SRAW_STD_NOX)
                * self._index_gain
            )
        else:
            return (
                (
                    (sraw - self._mox_sraw_mean)
                    / (-1.0 * (self._mox_sraw_std + self._SRAW_STD_BONUS_VOC))
                )
                * self._index_gain
            )

    # ================================================================
    # Sigmoid Scaled
    # ================================================================

    # C: GasIndexAlgorithm__sigmoid_scaled__set_parameters
    def _sigmoid_scaled_set_parameters(self, x0, k, offset_default):
        self._sigmoid_scaled_k = k
        self._sigmoid_scaled_x0 = x0
        self._sigmoid_scaled_offset_default = offset_default

    # C: GasIndexAlgorithm__sigmoid_scaled__process
    def _sigmoid_scaled_process(self, sample):
        x = self._sigmoid_scaled_k * (sample - self._sigmoid_scaled_x0)
        if x < -50.0:
            return self._SIGMOID_L
        elif x > 50.0:
            return 0.0
        else:
            if sample >= 0.0:
                if self._sigmoid_scaled_offset_default == 1.0:
                    shift = (500.0 / 499.0) * (1.0 - self._index_offset)
                else:
                    shift = (
                        self._SIGMOID_L - (5.0 * self._index_offset)
                    ) / 4.0
                return (
                    (self._SIGMOID_L + shift) / (1.0 + math.exp(x))
                ) - shift
            else:
                return (
                    self._index_offset / self._sigmoid_scaled_offset_default
                ) * (self._SIGMOID_L / (1.0 + math.exp(x)))

    # ================================================================
    # Adaptive Lowpass
    # ================================================================

    # C: GasIndexAlgorithm__adaptive_lowpass__set_parameters
    def _adaptive_lowpass_set_parameters(self):
        self._lp_a1 = self._sampling_interval / (
            self._LP_TAU_FAST + self._sampling_interval
        )
        self._lp_a2 = self._sampling_interval / (
            self._LP_TAU_SLOW + self._sampling_interval
        )
        self._lp_initialized = False

    # C: GasIndexAlgorithm__adaptive_lowpass__process
    def _adaptive_lowpass_process(self, sample):
        if self._lp_initialized == False:
            self._lp_x1 = sample
            self._lp_x2 = sample
            self._lp_x3 = sample
            self._lp_initialized = True

        self._lp_x1 = (
            ((1.0 - self._lp_a1) * self._lp_x1) + (self._lp_a1 * sample)
        )
        self._lp_x2 = (
            ((1.0 - self._lp_a2) * self._lp_x2) + (self._lp_a2 * sample)
        )

        abs_delta = self._lp_x1 - self._lp_x2
        if abs_delta < 0.0:
            abs_delta = -1.0 * abs_delta

        f1 = math.exp(self._LP_ALPHA * abs_delta)
        tau_a = (
            (self._LP_TAU_SLOW - self._LP_TAU_FAST) * f1
        ) + self._LP_TAU_FAST
        a3 = self._sampling_interval / (self._sampling_interval + tau_a)

        self._lp_x3 = ((1.0 - a3) * self._lp_x3) + (a3 * sample)
        return self._lp_x3
