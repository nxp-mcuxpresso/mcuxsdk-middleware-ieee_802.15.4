# CLI shortcut command description

Commands from this section are used for configuring the device. All the shortcut commands modifications, except **\[x\]**and **\[c\]**, are displayed in the device state.

Shortcut commands cover the following configuration settings:

1.  **Device operation mode**:
    -   **\[t\]**: configures the device in **transmitter** mode
    -   **\[r\]**: configures the device in **receiver** mode
2.  **Channel settings:**
    -   Channel number range between \[11-26\], default value = 11
    -   **\[q\]**:**increments** channel number
    -   **\[w\]**: **decrements** channel number
3.  **Output power settings:**
    -   Output power range between \[0-32\], default value = 5
    -   **\[a\]**: **increments** output power
    -   **\[s\]**: **decrements** output power
4.  **Crystal trim value settings:**
    -   XTAL trim range between \[0-127\], default value = 0
    -   **\[d\]**: **increases** XTAL trim value
    -   **\[f\]**: **decreases** XTAL trim value
5.  **Data payload length settings:**
    -   **\[n\]: increases** the data payload length
    -   **\[m\]**: **decreases** the data payload length
6.  **CCA threshold settings:**
    -   CCA Threshold range between \[0-110\], default value=80
    -   **\[k\]**: **increases** CCA Threshold in Carrier Sense Test
    -   **\[l\]**: **decreases** CCA Threshold in Carrier Sense Test
7.  **Packet acknowledgment settings:**
    -   Default value = NoAck.
    -   **\[z\]**: sets Acknowledgement requirement \(NoAck/Ack/EnhancedAck\).
8.  **Packet source/address settings:**
    -   Source address default value = 0xBEAD
    -   Destination address default value = 0xFFFF
    -   **\[x\]**: sets **source** address for packets
    -   **\[c\]**: sets **destination** address for packets

**Parent topic:**[Connectivity Test CLI](../topics/connectivity_test_cli.md)

