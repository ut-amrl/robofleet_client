FROM ros:melodic-robot
RUN apt-get update && apt-get install -y \
  vim \
  qt5-default \
  libqt5websockets5-dev
COPY . /robofleet_client
