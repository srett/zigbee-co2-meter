# A CO2 monitor that is also an RGB lamp

Will show CO2 level via the RGB LED (the usual green -> yellow -> red transition),
but also expose an endpoint that's an RGB dimmable lamp.

When you turn on the lamp, the LED will alternate between showing the
CO2 level (800ms) and the configured color/brightness of the lamp (200ms).

Stuff could be made configurable via more zigbee attributes but getting shit
to work at all was annoying enough, so I'll postpone this for another decade or two.
