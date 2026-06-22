# UART Protocol

Why? Because our cameras (Sipeed MaixCams) process images on their own boards, and we need to send the output to the rest of our bot.

The link is bidirectional. The bot (pi) can send commands to a camera to control what it streams, and the camera streams back either detection packets or JPEG images depending on the current mode.

## Bot → Camera (commands)

The bot controls each camera by sending a command frame. Cameras start in **stop/idle** mode, switch mode on receipt, and keep that mode until the next command.

| Data        | Data Type | Size   | Description                                                          |
| ----------- | --------- | ------ | ------------------------------------------------------------------- |
| start_flag  | byte      | 8 bits | `0xAA`, signals the start of a command frame.                       |
| command     | byte      | 8 bits | `0x00` = stop (idle), `0x01` = detect (stream packets), `0x02` = debug (stream compressed images), `0x03` = training (stream full-quality images). |
| debug_quality | byte | 8 bits | Only present for debug commands. JPEG quality from `1` to `100`. |

Modes:

- **stop (`0x00`)**: camera goes idle and sends nothing.
- **detect (`0x01`)**: camera streams detection packets (see "Camera → Bot (detection)" below). The pi must explicitly request this mode when it needs vision data.
- **debug (`0x02 <quality>`)**: camera broadcasts compressed JPEG frames (see "Camera → Bot (image)" below) so the bot can forward them to a web server. The quality byte is sent by the pi so debug quality can be changed without new camera firmware.
- **training (`0x03`)**: camera broadcasts full-quality JPEG frames (see "Camera → Bot (image)" below) so the bot can save them for training.

## Camera → Bot (detection)


| Data           | Data Type    | Size                         | Unit    | Description                                                                         |
| -------------- | ------------ | ---------------------------- | ------- | ----------------------------------------------------------------------------------- |
| start_flag     | byte         | 8 bits | 🇦🇺    | All 1s to signal start of a new packet. NOTE: no other byte can have all 1s.        |
| info_bits      | booleans     | 3 bits of 0 + 5 info bits | bit     | Can see line, Can see ball, Can see goal, Goal is yellow, Camera is okay.            |
| ball_direction | signed int   | 8 bits                       | degrees | Signed angle from the centre (1 bit sign, 7 bits integer)                           |
| ball_distance  | unsigned int | 8 bits                       | cm      | Rough distance to ball                                                              |
| goal_direction | signed int   | 8 bits                       | degrees | Signed angle from the centre to the goal if applicable (1 bit sign, 7 bits integer) |
| goal_distance  | unsigned int | 8 bits                       | cm      | Rough distance to the goal                                                          |
| line_direction | signed int   | 8 bits                       | degrees | Signed angle from the centre to the closest point on the closest white line (1 bit sign, 7 bits integer). Only meaningful when the "Can see line" bit is set. |
| line_distance  | unsigned int | 8 bits                       | cm      | Rough distance to the closest point on the closest white line.                      |

## Camera → Bot (image)

Sent only while in **debug** or **training** mode. Each frame is a JPEG image wrapped in a simple length-prefixed frame. The magic prefix is chosen so it can't be confused with the `0xff`-delimited detection packets.

| Data    | Data Type    | Size      | Description                                              |
| ------- | ------------ | --------- | ------------------------------------------------------- |
| magic   | bytes        | 4 bytes   | `0xAB 0xCD 0xEF 0x01`, signals the start of an image frame. |
| length  | unsigned int | 4 bytes   | Big-endian length of the JPEG payload in bytes.          |
| payload | bytes        | `length` bytes | The JPEG-encoded image data.                       |

The payload must be a complete JPEG: it starts with `0xFF 0xD8` and ends with `0xFF 0xD9`. The camera sends one complete framed image at a time and does not start the next image frame until the previous header and payload have been fully written to UART.

In **debug** mode, JPEG quality is reduced to keep frames small enough for web debugging over UART. In **training** mode, JPEG quality is set to full quality and the pi saves every received frame to disk as fast as the UART connection allows.

Note: at 115200 baud the link carries only ~14 KB/s, so image streaming runs well under 1 fps unless frames are very small.
