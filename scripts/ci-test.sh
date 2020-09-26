#!/bin/sh
# Run the checks for continuous integration
scripts/docker-build.sh
docker run --rm -w /robofleet_client robofleet-client scripts/destructive/run-tests.sh
