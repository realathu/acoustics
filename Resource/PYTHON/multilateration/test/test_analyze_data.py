import numpy as np

import multilateration.analyze_data as andat
import signal_generation.source as sg_source
import signal_generation.receiver as sg_rec
import signal_generation.positioning as sg_pos 


def test_given_idealized_signals_when_calculate_lags_then_logical_results():
    # Pinger configuration
    pinger = sg_source.Pinger(
        frequency=10.0,
        sampling_frequency=100.0,
        pulse_length=100,
        period=5000,
    )
    pinger_position = sg_pos.Position(
        x=20.0,
        y=20.0,
        z=0.0,
    )



    # Hydrophone configuration
    port_hyd_pos = sg_pos.Position(
        x=-1.0,
        y=0.0,
        z=0.0,
    )
    starboard_hyd_pos = sg_pos.Position(
        x=1.0,
        y=0.0,
        z=0.0,
    )
    stern_hyd_pos = sg_pos.Position(
        x=0.0,
        y=-1.0,
        z=0.0,
    )
    hydrophone_array = sg_rec.HydrophoneArray(
        positions = [port_hyd_pos, starboard_hyd_pos, stern_hyd_pos]
    )



    # Generate signals
    output_signal = pinger.generate_signal(
    amplitude=0.6,
    length=5000,
    )
    
    data_array = hydrophone_array.generate_signals(
        output_length = 500,
        source = pinger,
        source_position = pinger_position,
    )



    # Cross-correlate signals
    results_array = andat.calculate_xcorr_lag_array(data_array)

    ref_array = np.zeros(3)

    assert np.array_equal(results_array, ref_array)



def test_given_equal_input_when_calculate_lags_then_all_zero():
    data_port = np.zeros(4096)
    data_port[0] = 1
    data_starboard = np.zeros(4096)
    data_starboard[0] = 1
    data_stern = np.zeros(4096)
    data_stern[0] = 1
    data_array = np.array([data_port, data_starboard, data_stern])

    ref_array = np.zeros(3)

    results_array = andat.calculate_xcorr_lag_array(data_array)
    
    assert np.array_equal(results_array, ref_array)


def test_analyze_data():
    data_port = np.zeros(4096)
    data_port[0] = 1
    data_starboard = np.zeros(4096)
    data_starboard[1] = 1
    data_stern = np.zeros(4096)
    data_stern[2] = 1
    data_array = np.array([data_port, data_starboard, data_stern])

    ref_array = np.array([-1,-2,-1])

    results_array = andat.calculate_xcorr_lag_array(data_array)
    
    assert np.array_equal(results_array, ref_array)