#!/bin/bash
# screen_params="-screen 0 800x600x24 -core -ac -dpi 96"
# screen_params="-ac -extension GLX"
# screen_params="-ac"
# screen_params="-screen 0 800x600x24 -ld 10000000"
# screen_params="-screen 0 1x1x24 -ld 5000000 -ls 5000000 +iglx"
screen_params="-screen 0 1x1x24"
echo "Xvfb (virtual framebuffer) will run with the following screen parameters: \"$screen_params\""
env LIBGL_DEBUG=verbose xvfb-run -s "$screen_params" $@
