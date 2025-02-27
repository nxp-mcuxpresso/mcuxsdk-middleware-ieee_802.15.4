# Continuous Test menu

The Continuous Test menu displays the following modes of operation:

1.  **Idle mode**: This mode sets the transceiver into Idle mode.

2.  **Burst PRBS transmission using packet mode**: This mode verifies the transceiver operational state by transmitting over the air a Pseudo Random Binary Sequence payload of 65 bytes.

3.  **Continuous modulated transmission**: This mode generates and loads a PRBS9 pattern into transceiver’s TX buffer and is used to verify the modulated output power of the transceiver.
4.  **Continuous unmodulated transmission**: This mode sets the transceiver to continuously transmit an unmodulated carrier \(CW\) and is used to measure the output power of the transceiver.
5.  **Continuous reception**: This mode sets the transceiver into Receive sequence.
6.  **Continuous energy detect**: This mode sets the transceiver to perform the Energy Detection operation. It returns the maximal energy on the specified channel.
7.  **Continuous scan**: This mode returns values of Energy Detect operation performed on all RF channels.
8.  **Continuous CCA**: This mode performs Continuous Clear Channel Assessment \(CCA\) procedure and returns whether the specified channel is Busy or Idle.

**Parent topic:**[CLI test description](../topics/cli_tests_description.md)

