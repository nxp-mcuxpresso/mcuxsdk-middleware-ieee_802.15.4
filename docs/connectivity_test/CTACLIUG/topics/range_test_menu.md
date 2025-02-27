# Range Test menu

This is a simple and quick test in which data packets are exchanged between two devices.

-   On the transmitter side, the device initiates the data transfer and expects to receive a packet containing the same data payload. An average RSSI calculation is performed based on the incoming packet’s RSSI and the number of sent packets. If the receiving packet does not contain the same data payload, RSSI calculation is not performed, and a ‘packet dropped’ message is shown in CLI.
-   On the receiver side, the device sends back the message to originator.

Range tests can be performed in three different modes:

1.  Transmission without expected Acknowledgment.
2.  Transmission with expected Acknowledgment.
3.  Transmission with expected Enhanced Acknowledgment


```{include} ../topics/steps_to_perform_a_range_test_procedure.md
:heading-offset: 3
```

**Parent topic:**[CLI test description](../topics/cli_tests_description.md)

