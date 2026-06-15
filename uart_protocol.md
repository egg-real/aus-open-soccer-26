# UART Protocol

Why? Because our cameras (Sipeed MaixCams) process images on their own boards, and we need to send the output to the rest of our bot.

## Camera → Bot


| Data           | Data Type    | Size                         | Unit    | Description                                                                         |
| -------------- | ------------ | ---------------------------- | ------- | ----------------------------------------------------------------------------------- |
| start_flag     | byte         | 8 bits | 🇦🇺    | All 1s to signal start of a new packet. NOTE: no other byte can have all 1s.        |
| info_bits      | booleans     | 4 bits of 0 + 4 info bits | bit     | Can see ball, Can see goal, Goal is yellow, Camera is okay.                         |
| ball_direction | signed int   | 8 bits                       | degrees | Signed angle from the centre (1 bit sign, 7 bits integer)                           |
| ball_distance  | unsigned int | 8 bits                       | cm      | Rough distance to ball                                                              |
| wall_direction | signed int   | 8 bits                       | degrees | Signed angle from the centre to the closest wall (1 bit sign, 7 bits integer)       |
| wall_distance  | unsigned int | 8 bits                       | cm      | Rough distance to the closest wall                                                  |
| goal_direction | signed int   | 8 bits                       | degrees | Signed angle from the centre to the goal if applicable (1 bit sign, 7 bits integer) |
| goal_distance  | unsigned int | 8 bits                       | cm      | Rough distance to the closest wall                                                  |
| line_i         | list         | unlimited, 16 bits each line | cm      | A list of equations of lines in the form y=mx+c, where the first 8 bits represent m and the next 8 represent c. (0,0) is the bottom left corner where the white lines meet from the bot's perspective facing forward. Forward is postive y and right is positive x.

