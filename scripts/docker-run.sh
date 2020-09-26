#!/bin/sh
# Run a command interactively in a Docker container (give no arguments to enter bash)
docker run --rm -w /robofleet_client -it robofleet-client $@
