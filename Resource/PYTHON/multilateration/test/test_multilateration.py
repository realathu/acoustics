import numpy as np
import math
import multilateration.parameters as param
import multilateration.multilateration as mult


"""
 * @file
 * 
 * @brief File to implement tests for code analysis
 * 
 * Required tests:
 *      1. Memory usage and leakage
 *      2. DMA correctly set up
 *      3. ADC correctly set up
 *      4. ETH correctly set up
 *      5. Filter coefficients
 *      6. ...
 * 
 * @warning For optimum performance, the code should be "public", such 
 * that this file has access to the code. This is not done as of 22.12.2020,
 * however should pherhaps be implemented over the following days/weeks 
"""


"""
   * @brief Function that generates TOA of the sound-signals for the different
   * hydrophones. Returns the calculated TOA indirectly via @p lag_array
   * 
   * The TOA is calculated based on the following parameters:
   *    @p HYD_xxx_POS   
   *    @p SOURCE_POS
   *    @p SOUND_SPEED
   *    @p SAMPLE_FREQUENCY
   * all of which are given in "parameters.h"
   * 
   * @param lag_array An array giving the time the sound will be detected
   * by the hydrophones. The array expands to
   *    @p lag_array = { lag_port, lag_starboard, lag_stern }
   * 
   * @param bool_valid_parameters An int describing if the parameters
   * given in parameters.h are valid.
"""
# Test-functions for acoustic trilateration

bool_valid_parameters = 0


def valid_number_of_hydrophones():
    return param.HydrophoneDetailes.NUM_HYDROPHONES < 3


def calculate_toa_array():
    """
    Arg:
    in the orginal code is was written:
        uint32_t lag_array[NUM_HYDROPHONES],
        uint8_t& bool_valid_parameters - this one is removed

    return:
    lag_array
    """

    # Calculating the distance [m] between the sound-source and the hydrophones
    dist_src_port = float(
        math.sqrt(
            math.pow(
                param.TestParameters.SOURCE_POS_X - param.HydrophoneDetails.PORT_HYD_X,
                2,
            )
            + math.pow(
                param.TestParameters.SOURCE_POS_Y - param.HydrophoneDetails.PORT_HYD_Y,
                2,
            )
            + math.pow(
                param.TestParameters.SOURCE_POS_Z - param.HydrophoneDetails.PORT_HYD_Z,
                2,
            )
        )
    )
    dist_src_starboard = float(
        math.sqrt(
            math.pow(
                param.TestParameters.SOURCE_POS_X
                - param.HydrophoneDetails.STARBOARD_HYD_X,
                2,
            )
            + math.pow(
                param.TestParameters.SOURCE_POS_Y
                - param.HydrophoneDetails.STARBOARD_HYD_Y,
                2,
            )
            + math.pow(
                param.TestParameters.SOURCE_POS_Z
                - param.HydrophoneDetails.STARBOARD_HYD_Z,
                2,
            )
        )
    )
    dist_src_stern = float(
        math.sqrt(
            math.pow(
                param.TestParameters.SOURCE_POS_X - param.HydrophoneDetails.STERN_HYD_X,
                2,
            )
            + math.pow(
                param.TestParameters.SOURCE_POS_Y - param.HydrophoneDetails.STERN_HYD_Y,
                2,
            )
            + math.pow(
                param.TestParameters.SOURCE_POS_Z - param.HydrophoneDetails.STERN_HYD_Z,
                2,
            )
        )
    )

    # Calculating the numer of samples it takes for the sound to arrive at x hyd

    # Since the TOA uses lag (direct measuremend), these values are uint32_t
    # converting to int32 works as floor. Should be rounded.
    samp_port = np.int32(
        param.DSPConstants.SAMPLE_FREQUENCY
        * (dist_src_port / param.PhysicalConstants.SOUND_SPEED)
    )  #
    samp_starboard = np.int32(
        param.DSPConstants.SAMPLE_FREQUENCY
        * (dist_src_starboard / param.PhysicalConstants.SOUND_SPEED)
    )  #
    samp_stern = np.int32(
        param.DSPConstants.SAMPLE_FREQUENCY
        * (dist_src_stern / param.PhysicalConstants.SOUND_SPEED)
    )  # *

    lag_port_starboard = samp_port - samp_starboard
    lag_port_stern = samp_port - samp_stern
    lag_startboard_stern = samp_starboard - samp_stern

    return [lag_port_starboard, lag_port_stern, lag_startboard_stern]


"""
   * @brief Function that checks if the simulated sound-source position is  within 
   * @p MARGIN_POS_ESTIMATE distance from the estimated position
   * 
   * Writes the distance estimated and the actual distance to the terminal
"""


def test_trilateration_algorithm():

    """
    Creating the TOA-intervals.
    Writing out an error-msg if the parameters are wrong
    """

    lag_array = calculate_toa_array()

    if not valid_number_of_hydrophones:
        print("\nAt least one parameter is invalid")
        return

    # Initializing the estimate for x-pos and y-pos

    x_pos_es, y_pos_es = mult.trilaterate_pinger_position(lag_array)

    """
    * Calculating the distance between the estimated position and the 
    * actual position. Only considering x and y, since we only have
    * three hydrophones
    """
    distance_diff = float(
        math.sqrt(
            math.pow(x_pos_es - param.TestParameters.SOURCE_POS_X, 2)
            + math.pow(y_pos_es - param.TestParameters.SOURCE_POS_Y, 2)
        )
    )

    print(f"\nThe estimated pinger position is at (x,y) = ({x_pos_es}, {y_pos_es})")
    print(
        f"\nThe actual pinger position is at (x,y) = ({param.TestParameters.SOURCE_POS_X}, {param.TestParameters.SOURCE_POS_Y})"
    )
    print(
        f"\nThe difference between the actual position and the estimated position is {distance_diff} m"
    )
