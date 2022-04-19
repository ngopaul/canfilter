import matplotlib.pyplot as plt
import math
import numpy as np
import pandas
import seaborn as sns
import pandas as pd
from typing import Dict, List, Collection
from enum import Enum

sns.set()


class Value(Enum):
    UNKNOWN = -1
    UNUSED = 0
    CONST = 1
    POSS = 2
    COUNTER = 3
    MULTI = 4
    CRC = 5
    PHYS = 6


def hex_to_binary(hex_str, pad_to=1, return_np_bool_array=False):
    """ 
    Convert a hex string to a binary string, padded to the given length. 
    
    Args:
        hex_str: a string which represents a hexadecimal number (can have 0x prefix, which is ignored)
        pad_to: how long the returned binary number should be, minimum. If the hex number converted 
            binary is longer than pad_to, then pad_to is ignored. Otherwise, 0's will be added to the
            beginning of the converted binary string until the final string is of the length pad_to.
    Returns:
        str: a binary string equivalent to hex_str's value in hex
    Raises:
        ValueError: invalid literal for int() with base 16, if the given hex_str is invalid
    >>> hex_to_binary("0000430000000091")
    '10000110000000000000000000000000000000010010001'
    >>> hex_to_binary("0000430000000091", 64)
    '0000000000000000010000110000000000000000000000000000000010010001'
    >>> hex_to_binary("0")
    '0'
    >>> hex_to_binary("0", 64)
    '0000000000000000000000000000000000000000000000000000000000000000'
    """
    n = int(hex_str, 16)
    bStr = ''
    while n > 0:
        bStr = str(n % 2) + bStr
        n = n >> 1
    return_string = bStr.zfill(pad_to)
    if not return_np_bool_array:
        return return_string
    return np.array([c == '1' for c in return_string])


def int_to_binary_string(val, width):
    """
    Given an int, return the width-long binary equivalent of the int in the form of a binary string.
    
    Will not fail if the val does not fit into the width. Will just truncate the higher values.
    """
    return str(bin(((1 << width) - 1) & val))[2:].zfill(width)


def mask_hex_str(hex_str, position, length, pad_to=64, byteorder='big', signed=False):
    """
    Returns a decimal which corresponds to the value of the hex string provided, looking only at the values
    specified by the position and offset.
    If the length is a multiple of 8, then byteorder and signed will be used to obtain the final decimal value.
    
    Args:
        hex_str: a string which represents a hexadecimal number (can have 0x prefix, which is ignored)
        position: a 0-indexed postion within the signal after conversion to binary
        length: the length of the signal to look at
        pad_to: the size the binary string should be padded with 0s to.
    Returns:
        int: a decimal value which corresponds to the value of the hex_string provided, after converting to binary,
            padding, and masking.
    Raises:
        ValueError
    
    >>> mask_hex_str("0000430000000091", 0, 64)
    73667279061137
    >>> mask_hex_str("0000430000000091", 0, 64, byteorder='little')
    10448351135503941632
    >>> mask_hex_str("0000430000000091", 0, 64, byteorder='little', signed=True)
    -7998392938205609984
    """
    if position < 0 or length <= 0:
        raise ValueError("Position and length must be at least 0 and greater than 0 respectively")
    if position + length > pad_to:
        raise ValueError(f"Position {position} and length {length} must be contained in the size of the data, {pad_to}")
    binary_representation = hex_to_binary(hex_str, pad_to=pad_to)[position:position + length]
    return mask_bin_str(binary_representation, byteorder=byteorder, signed=signed)


def mask_bin_str(bin_str, byteorder='big', signed=False):
    binary_representation = bin_str
    round_length_to_8 = (len(binary_representation) + 7) & (-8)
    binary_representation = binary_representation.zfill(round_length_to_8)

    length_in_bytes = round_length_to_8 // 8
    byte_representation = int(binary_representation, 2).to_bytes(length_in_bytes, byteorder=byteorder)
    int_representation = int.from_bytes(byte_representation, byteorder=byteorder, signed=signed)

    return int_representation


def get_raw_signal_values(can_dataframe: pd.DataFrame, message_id: int, position: int, length: int,
                          bus_limit: int = 10, return_np_bool_array: bool = False, pad_to_at_least: int = 0):
    """
    Given a pandas datafrome, CAN ID, and cutoff parameters, return the raw signal values recorded in the dataframe
        for that ID.

    :param can_dataframe: Pandas dataframe of CAN messages with headers:
        'MessageID' (integer ID)
        'Bus' (integer bus num)
        'MessageLength' (integer byte size of message, generally 1, 2, 8)
        'Time' (unix time in seconds)
        'Message' (hexadecimal string representing the data)
    :param message_id: CAN ID
    :param position: bit start position of obtaining the raw signal value
    :param length: bit length for obtaining the raw signal value
    :param bus_limit: maximum number of buses to accept
    :param return_np_bool_array: if True, each return value is an 2D array of boolean values representing the actual
    bits in the message. Otherwise will return a binary string.
    :param pad_to_at_least: pad the output value to this bit size. Will not truncate if original size is larger.
    :return: xs, ys
        xs: List of Pandas Series, containing timestamps
            Length of the list ranges from 0 (no data) to bus_limit
        ys: List of 1D numpy arrays of binary strings (or 2D numpy arrays of booleans).
            Length of the list ranges from 0 (no data) to bus_limit
    """
    message_data = can_dataframe.where(can_dataframe['MessageID'] == message_id)
    buses = message_data['Bus'].unique()
    xs, ys = [], []
    bus_counter = 0
    for bus in buses:
        if math.isnan(bus):
            continue
        if bus_counter >= bus_limit:
            break
        message_data_for_bus = message_data.loc[message_data['Bus'] == bus]
        # the pad_to length from "MessageLength" in the csv data is measured in bytes, convert to bits
        pad_to = int(message_data_for_bus["MessageLength"].reset_index(drop=True)[0] * 8)
        pad_to = max(pad_to_at_least, pad_to)
        x = message_data_for_bus['Time']
        y = message_data_for_bus['Message'].apply(
            lambda hex_str: hex_to_binary(hex_str, pad_to=pad_to, return_np_bool_array=return_np_bool_array)[
                            position:position + length])
        xs.append(x)
        ys.append(y)
        bus_counter += 1
    return xs, ys


def plot_message_id(csv_data, message_id, position, length, byteorder='big', signed=False, scale=1, offset=0,
                    plot_fxn=plt.scatter):
    """
    Given a pandas csv of CAN data, and location of what to plot and how to interpret the CAN signal, plot the value.
    Also return the integer values.
    :param csv_data:
    :param message_id:
    :param position:
    :param length:
    :param byteorder:
    :param signed:
    :param scale:
    :param offset:
    :param plot_fxn:
    :return:
    """
    message_data = csv_data.where(csv_data['MessageID'] == message_id)
    buses = message_data['Bus'].unique()
    xs, ys = [], []

    for bus in buses:
        if math.isnan(bus):
            continue
        message_data_for_bus = message_data.loc[message_data['Bus'] == bus]
        # the pad_to length from "MessageLength" in the csv data is measured in bytes, convert to bits
        pad_to = int(message_data_for_bus["MessageLength"].reset_index(drop=True)[0] * 8)
        x = message_data_for_bus['Time']
        y = message_data_for_bus['Message'].apply(
            lambda hex_str: mask_hex_str(hex_str, position, length, pad_to, byteorder, signed) * scale + offset)
        if plot_fxn is None:
            pass
        elif plot_fxn == plt.scatter:
            plot_fxn(x, y, label=f"Message {message_id}, bus {bus}, position {position}, "
                                 f"length {length}, scale {scale}, offset {offset}", s=1)
        else:
            plot_fxn(x, y, label=f"Message {message_id}, bus {bus}, position {position}, "
                                 f"length {length}, scale {scale}, offset {offset}")
        xs.append(x)
        ys.append(y)
    if plot_fxn is not None:
        plt.legend(loc='upper left')
        plt.show()
    return xs, ys


def describe_known_signal(cantools_db, message_name, signal_name, csv_data=None, plot_fxn=plt.scatter, silent=False):
    """
    Prints CAN info by default.
    Returns the information known about a given signal: x values, y values (if given csv_data, else None), and CAN info
    Also runs plot_message_id if given csv_data.
    """
    message = cantools_db.get_message_by_name(message_name)
    for signal in message.signals:
        if signal.name == signal_name:
            break
    assert signal.name == signal_name, "This message/signal combination is not in the cantools db."
    if not silent:
        print(f"message name: {message.name}\n"
              f"message frame_id: {message.frame_id}\n"
              f"message is_extended_frame: {message.is_extended_frame}\n"
              f"message length: {message.length}\n"
              f"message comment: {message.comment}\n"
              f"message bus_name: {message.bus_name}\n"
              )
        print(f"signal name: {signal.name}\n"
              f"signal start (1 indexed): {signal.start}\n"
              f"signal length in bits: {signal.length}\n"
              f"signal byte_order: {signal.byte_order}\n"
              f"signal is_signed: {signal.is_signed}\n"
              f"signal is_float: {signal.is_float}\n"
              f"signal scale: {signal.scale}\n"
              f"signal offset: {signal.offset}\n"
              f"signal minimum: {signal.minimum}\n"
              f"signal maximum: {signal.maximum}\n"
              f"signal comment: {signal.comment}\n"
              )
    byte_order = signal.byte_order.replace("_endian", "")

    start = signal.start // 8 * 8 + (7 - signal.start % 8)

    if not silent:
        print(f'Actual start: {start}')

    if csv_data is not None:
        xs, ys = plot_message_id(csv_data, message.frame_id, start, signal.length, byte_order, signal.is_signed,
                                 signal.scale, signal.offset, plot_fxn)
    else:
        xs, ys = None, None

    value_dict = {
        "message_length": message.length,
        "frame_id": message.frame_id,
        "start": start,
        "signal_length": signal.length,
        "byte_order": byte_order,
        "is_signed": signal.is_signed,
        "scale": signal.scale,
        "offset": signal.offset,
    }

    return xs, ys, value_dict


def get_message_id_length(csv_data, message_id, bus: Collection = (0, 1, 2)):
    """Based on a csv of CAN data, return the MessageLength of a given message id on a given bus. """
    first_data = csv_data[(csv_data['MessageID'] == message_id) & (csv_data['Bus'].isin(bus))].iloc[0]
    return int(first_data['MessageLength'])


def preprocess(can_dataframe: pd.DataFrame) -> (Dict, Dict):
    """
    Preprocess a pandas dataframe of CAN signals.
    Take only the first bus's signal for each CAN ID, and return bit_flip_rates and raw_signal_values

    :param can_dataframe: Pandas dataframe of CAN messages with headers:
        'MessageID' (integer ID)
        'Bus' (integer bus num)
        'MessageLength' (integer byte size of message, generally 1, 2, 8)
        'Time' (unix time in seconds)
        'Message' (hexadecimal string representing the data)
    :return:
        Dictionary mapping CAN ID to bit flip rates,
        Dictionary mapping CAN ID to (x, y)
            x: panda series of timestamps
            y: raw signal values (2D binary array). each row is a value
    """
    ids = can_dataframe['MessageID'].unique()
    bit_flip_rates = {}
    raw_signal_values = {}
    for id in ids:
        xs, ys = get_raw_signal_values(can_dataframe, id, 0, 64, bus_limit=1, return_np_bool_array=True,
                                       pad_to_at_least=64)
        xs = xs[0]
        ys = ys[0]
        raw_signal_values[id] = (xs, ys)
        signal_length = np.shape(ys)[0]
        bit_flip_count = np.diff(ys, axis=0).sum(axis=0)
        bit_flip_rates[id] = bit_flip_count / signal_length
    return bit_flip_rates, raw_signal_values


def find_runs(x):
    """Find runs of consecutive items in an array.
    Credit: @alimanfoo
    """

    # ensure array
    x = np.asanyarray(x)
    if x.ndim != 1:
        raise ValueError('only 1D array supported')
    n = x.shape[0]

    # handle empty array
    if n == 0:
        return np.array([]), np.array([]), np.array([])

    else:
        # find run starts
        loc_run_start = np.empty(n, dtype=bool)
        loc_run_start[0] = True
        np.not_equal(x[:-1], x[1:], out=loc_run_start[1:])
        run_starts = np.nonzero(loc_run_start)[0]

        # find run values
        run_values = x[loc_run_start]

        # find run lengths
        run_lengths = np.diff(np.append(run_starts, n))

        return run_values, run_starts, run_lengths


def score_messages(message_relevancy_dict: Dict) -> pd.DataFrame:
    """
    Given a dictionary message_relevancy dictionary, return the scores of each of the relevant messages, in order,
    as a pandas dataframe.
    :param message_relevancy_dict: Mapping from message ID to a boolean array, where the boolean values represent
        whether the bit is relevant or not. There may be some binary bits labeled as relevant when they are not;
        this is why scoring them is important.
    :return:
        scored_messages, a pandas dataframe with columns:
        MessageID | Score | BitwiseRelevancy
    """
    # The score is as follows:
    # Score = Run Score + True Count Score
    # RunScore = (# of 3-4 long runs of True) + (# of 5-6 long runs of True) * 2 + (# 7+ long runs of True) * 3
    # True Count Score = (# Trues)/64 * 5

    message_scores = []

    for messageID, boolean_relevancy in message_relevancy_dict.items():
        run_score = 0
        run_values, run_starts, run_lengths = find_runs(boolean_relevancy)
        for i, run_value in enumerate(run_values):
            if run_value:
                if 3 <= run_lengths[i] <= 4:
                    run_score += 1
                elif 5 <= run_lengths[i] <= 6:
                    run_score += 2
                elif run_lengths[i] >= 7:
                    run_score += 3
        true_count_score = sum(boolean_relevancy) / 64 * 5
        score = run_score + true_count_score
        message_scores.append(
            {
                "MessageID": messageID,
                "Score": score,
                "BitwiseRelevancy": ''.join(boolean_relevancy.astype(int).astype(str))
            }
        )

    scored_messages = pd.DataFrame(message_scores)
    scored_messages.sort_values("Score", ascending=False, inplace=True)
    return scored_messages
