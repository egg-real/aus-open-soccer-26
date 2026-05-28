# UART Protocol
Why? Because our cameras (Sipeed MaixCams) process images on their own boards, and we need to send the output to the rest of our bot.

## Camera → Bot
|Data|Data Type|Size|Unit|Description|
|-|-|-|-|-|
|ball_direction|signed int|8 bits|degrees|Signed angle from the centre (1 bit sign, 7 bits integer)
|ball_distance|unsigned int|8 bits|cm|Rough distance to ball|
|wall_distance|||||